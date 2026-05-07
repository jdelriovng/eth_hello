/**
 * ESP32-ETH02 — Low Power Sensor Dashboard
 *
 * Ciclo de trabajo:
 *   1. Boot desde deep sleep (RTC timer)
 *   2. Inicializar sensores + Ethernet
 *   3. Esperar link Ethernet (timeout configurable)
 *   4. Servir peticiones HTTP
 *   5. Si no hay petición durante IDLE_TIMEOUT_S → apagar ETH → deep sleep
 *   6. Si no hay link en LINK_TIMEOUT_S         → deep sleep directamente
 *   7. Si llevamos despiertos MAX_AWAKE_S        → dormir igualmente
 *
 * Todos los parámetros de temporización están en lp_config al principio.
 */

#include <stdio.h>
#include <string.h>
#include <time.h>

#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"


#include "i2c_bus.h"
#include "aht20.h"
#include "bmp280.h"



/* Handle global del wrapper i2cdev_bus_wrapper.c */
extern i2c_bus_device_handle_t g_bmp280_bus_dev;

#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_eth.h"
#include "esp_mac.h"

#include "esp_wifi.h"

#define WIFI_AP_SSID      "GoPro-AP"
#define WIFI_AP_PASS      "GoPro123"
#define WIFI_AP_CHANNEL   11
#define WIFI_AP_MAX_CONN  4
#define WIFI_AP_IP        "192.168.4.1"
#define WIFI_AP_GW        "192.168.4.1"
#define WIFI_AP_NETMASK   "255.255.255.0"

#include "esp_http_client.h"

static esp_netif_t *s_wifi_ap_netif = NULL; 




/* ================================================================== */
/*  PARÁMETROS DE LOW POWER — edita aquí                               */
/* ================================================================== */
typedef struct {
    bool     continuous;         /* true = nunca dormir (modo siempre activo) */
    uint32_t sleep_duration_s;   /* tiempo dormido entre ciclos       */
    uint32_t link_timeout_s;     /* max espera para que ETH negocie   */
    uint32_t idle_timeout_s;     /* inactividad antes de dormir        */
    uint32_t max_awake_s;        /* tiempo maximo despierto en total   */
} lp_config_t;

static const lp_config_t LP = {
    .continuous       = true, /* true = nunca dormir                 */
    .sleep_duration_s = 30,    /* tiempo dormido entre ciclos         */
    .link_timeout_s   = 15,     /* si no hay link en 8 s -> dormir     */
    .idle_timeout_s   = 10,    /* 10 s sin peticion -> dormir         */
    .max_awake_s      = 60,    /* maximo 60 s despierto               */
};

/* ================================================================== */
/*  Pines                                                               */
/* ================================================================== */
#define I2C_SDA_GPIO        14
#define I2C_SCL_GPIO        15

#define ETH_PHY_ADDR         1
#define ETH_PHY_RST_GPIO    (-1)
#define ETH_PHY_POWER_GPIO  16
#define ETH_MDC_GPIO        23
#define ETH_MDIO_GPIO       18

#define BMP280_ADDR_ACTUAL  0x77   // SDO=VCC confirmado por scanner

#define AHT20_ADDRRES 0x38

static const char *TAG = "LP_SENSORS";




/* ================================================================== */
/*  RTC memory — persiste durante deep sleep                           */
/* ================================================================== */
typedef struct {
    uint32_t boot_count;
    uint32_t total_sleep_s;
    float    last_aht_temp;
    float    last_aht_hum;
    float    last_bmp_temp;
    float    last_bmp_press;
    bool     last_aht_valid;
    bool     last_bmp_valid;
} rtc_data_t;

static RTC_DATA_ATTR rtc_data_t rtc;

static i2c_bus_device_handle_t s_aht20_bus_dev = NULL;
/* ================================================================== */
/*  Estado global                                                       */
/* ================================================================== */
#define ETH_LINK_UP_BIT   BIT0
#define ETH_LINK_DOWN_BIT BIT1



static EventGroupHandle_t  s_eth_eg       = NULL;
static SemaphoreHandle_t   s_sensor_mutex = NULL;

static i2c_bus_handle_t    s_i2c_bus    = NULL;
static aht20_dev_handle_t  s_aht20_dev  = NULL;
static bmp280_t            s_bmp280_dev;

static bool  s_aht20_ok   = false;
static bool  s_bmp280_ok  = false;
static float s_aht_temp   = 0.0f;
static float s_aht_hum    = 0.0f;
static float s_bmp_temp   = 0.0f;
static float s_bmp_press  = 0.0f;

static esp_netif_t     *s_eth_netif  = NULL;
static esp_eth_handle_t s_eth_handle = NULL;
static httpd_handle_t   s_http_srv   = NULL;

/* timestamp de boot (µs) para calcular tiempo despierto */
static int64_t s_boot_us = 0;

/* contador de peticiones HTTP activas (protegido por mutex ligero) */
static volatile int s_active_requests = 0;

/* ================================================================== */
/*  Utilidades de tiempo                                                */
/* ================================================================== */
static int32_t awake_seconds(void)
{
    return (int32_t)((esp_timer_get_time() - s_boot_us) / 1000000LL);
}

/* ================================================================== */
/*  Deep sleep                                                          */
/* ================================================================== */
static void go_to_sleep(const char *reason)
{
    if (LP.continuous) {
        ESP_LOGW(TAG, "go_to_sleep(%s) ignorado — modo continuo activo", reason);
        return;
    }
    ESP_LOGI(TAG, "Durmiendo (%s). Ciclo #%lu | despierto %lds | "
             "próximo wake en %lus",
             reason, (unsigned long)rtc.boot_count,
             (long)awake_seconds(),
             (unsigned long)LP.sleep_duration_s);

    /* Shutdown ordenado: HTTP -> netif -> ETH driver -> GPIO
     * El orden importa: detach netif antes de stop evita
     * los errores de rm_mac_filter en el EMAC. */
    if (s_http_srv) {
        httpd_stop(s_http_srv);
        s_http_srv = NULL;
        ESP_LOGI(TAG, "[LP] HTTP server detenido");
    }
    if (s_eth_handle) {
        /*
         * Orden correcto para evitar rm_mac_filter errors:
         * 1. esp_eth_stop() — para la negociacion y el DMA
         * 2. El driver emite DISCONNECTED/STOP internamente
         * El glue de netif recibe el evento y limpia sus filtros
         * de forma ordenada dentro del propio stack de eventos.
         */
        esp_eth_stop(s_eth_handle);
        vTaskDelay(pdMS_TO_TICKS(200));
        ESP_LOGI(TAG, "[LP] ETH detenido");
    }
    /* Bajar GPIO16 para que el PHY pierda alimentación durante el sleep.
     * Así el próximo boot siempre arranca con el PHY en estado limpio. */
    ESP_LOGI(TAG, "[LP] GPIO%d -> 0 (PHY off antes de dormir)", ETH_PHY_POWER_GPIO);
    gpio_set_level(ETH_PHY_POWER_GPIO, 0);
    /* gpio_hold_en mantiene el nivel bajo durante todo el deep sleep.
     * Sin esto el GPIO puede flotar o subir al entrar en sleep. */
    gpio_hold_en(ETH_PHY_POWER_GPIO);

    /* Guardar últimas lecturas en RTC memory */
    if (xSemaphoreTake(s_sensor_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        rtc.last_aht_temp  = s_aht_temp;
        rtc.last_aht_hum   = s_aht_hum;
        rtc.last_bmp_temp  = s_bmp_temp;
        rtc.last_bmp_press = s_bmp_press;
        rtc.last_aht_valid = s_aht20_ok;
        rtc.last_bmp_valid = s_bmp280_ok;
        xSemaphoreGive(s_sensor_mutex);
    }

    rtc.total_sleep_s += LP.sleep_duration_s;

    esp_sleep_enable_timer_wakeup((uint64_t)LP.sleep_duration_s * 1000000ULL);
    esp_deep_sleep_start();
}

/* ================================================================== */
/*  AHT20 bare-metal helpers — via i2c_bus (sin driver/i2c.h)         */
/* ================================================================== */

static void aht20_hw_trigger(void)
{
    // mem_address=0xAC es el comando, data son los dos bytes restantes
    uint8_t data[2] = {0x33, 0x00};
    i2c_bus_write_bytes(s_aht20_bus_dev, 0xAC, sizeof(data), data);
}

static esp_err_t aht20_hw_read(float *temp, float *hum)
{
    uint8_t buf[6] = {0};
    // mem_address=0 significa leer sin registro previo (raw read)
    esp_err_t ret = i2c_bus_read_bytes(s_aht20_bus_dev, 0, sizeof(buf), buf);
    if (ret != ESP_OK) return ret;
    if (buf[0] & 0x80) return ESP_ERR_NOT_FINISHED;  // busy flag

    uint32_t rh = ((uint32_t)buf[1] << 12) | ((uint32_t)buf[2] << 4) | (buf[3] >> 4);
    uint32_t rt = (((uint32_t)buf[3] & 0x0F) << 16) | ((uint32_t)buf[4] << 8) | buf[5];
    *hum  = (float)rh / 1048576.0f * 100.0f;
    *temp = (float)rt / 1048576.0f * 200.0f - 50.0f;
    return ESP_OK;
}
static void read_sensors(void)
{
    if (xSemaphoreTake(s_sensor_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return;

    if (s_aht20_ok) {
        aht20_hw_trigger();
        vTaskDelay(pdMS_TO_TICKS(80));
        float t = 0, h = 0;
        if (aht20_hw_read(&t, &h) == ESP_OK) {
            s_aht_temp = t; s_aht_hum = h;
        } else { s_aht20_ok = false; }
    }
    if (s_bmp280_ok) {
        float t = 0, p = 0, hd = 0;
        if (bmp280_read_float(&s_bmp280_dev, &t, &p, &hd) == ESP_OK) {
            s_bmp_temp = t; s_bmp_press = p / 100.0f;
        } else { s_bmp280_ok = false; }
    }

    xSemaphoreGive(s_sensor_mutex);
}
/* ================================================================== */
/*  Sensores                                                            */
/* ================================================================== */
static void init_sensors(void)
{
    /*
     * Estrategia para coexistencia AHT20 (i2c_bus) + BMP280 (i2cdev):
     *
     * En ESP-IDF v5.x ambas librerias llaman a i2c_new_master_bus()
     * y la segunda falla con ESP_ERR_INVALID_STATE.
     *
     * Solucion: crear el bus con i2c_bus primero, obtener el handle
     * nativo (i2c_master_bus_handle_t) e inyectarlo en i2cdev mediante
     * i2cdev_init_port(). Asi i2cdev reutiliza el bus existente en lugar
     * de intentar crear uno nuevo.
     */

    /* 1. Crear bus I2C via i2c_bus (necesario para AHT20) */
    const i2c_config_t i2c_conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_SDA_GPIO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_io_num       = I2C_SCL_GPIO,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    s_i2c_bus = i2c_bus_create(I2C_NUM_0, &i2c_conf);
    if (!s_i2c_bus) {
        ESP_LOGE(TAG, "No se pudo crear bus I2C");
        return;
    }

    /* 2. i2cdev no se usa — BMP280 se inicializa via wrapper local
     *    definido en este mismo fichero (ver seccion BMP280 WRAPPER). */
    /* 3. AHT20 — inicializar bus device, luego leer con helpers bare-metal */
    aht20_i2c_config_t aht_cfg = {
        .bus_inst = s_i2c_bus,
        .i2c_addr = AHT20_ADDRRES_0,
    };
    if (aht20_new_sensor(&aht_cfg, &s_aht20_dev) == ESP_OK) {
        // Crear device handle para los helpers bare-metal
        s_aht20_bus_dev = i2c_bus_device_create(s_i2c_bus, 0x38, 0);

        vTaskDelay(pdMS_TO_TICKS(500));
        aht20_hw_trigger();
        vTaskDelay(pdMS_TO_TICKS(80));
        float t = 0, h = 0;
        if (aht20_hw_read(&t, &h) == ESP_OK) {
            s_aht20_ok = true;
            s_aht_temp = t; s_aht_hum = h;
            ESP_LOGI(TAG, "AHT20: %.2f°C  %.2f%%", t, h);
        } else {
            ESP_LOGW(TAG, "AHT20: lectura inicial fallo");
            aht20_del_sensor(s_aht20_dev);
            s_aht20_dev = NULL;
        }
    }
    /* 4. BMP280 via wrapper (i2cdev_bus_wrapper.c sobre i2c_bus)
     *    g_bmp280_bus_dev es el handle que el wrapper usa para
     *    redirigir las llamadas i2c_dev_read/write a i2c_bus. */
    g_bmp280_bus_dev = i2c_bus_device_create(s_i2c_bus, BMP280_ADDR_ACTUAL, 0);
    if (!g_bmp280_bus_dev) {
        ESP_LOGW(TAG, "BMP280: i2c_bus_device_create fallo");
    } else {
        memset(&s_bmp280_dev, 0, sizeof(s_bmp280_dev));
        s_bmp280_dev.i2c_dev.port = I2C_NUM_0;
        s_bmp280_dev.i2c_dev.addr = BMP280_ADDR_ACTUAL;
        s_bmp280_dev.i2c_dev.mutex = xSemaphoreCreateMutex();

        bmp280_params_t bmp_params;
        bmp280_init_default_params(&bmp_params);

        if (bmp280_init(&s_bmp280_dev, &bmp_params) == ESP_OK) {
            s_bmp280_ok = true;
            float t = 0, p = 0, hd = 0;
            if (bmp280_read_float(&s_bmp280_dev, &t, &p, &hd) == ESP_OK) {
                s_bmp_temp = t; s_bmp_press = p / 100.0f;
            }
            ESP_LOGI(TAG, "BMP280: %.2f°C  %.2fhPa", s_bmp_temp, s_bmp_press);
        } else {
            ESP_LOGW(TAG, "BMP280: bmp280_init fallo");
        }
    }
}

    

/* ================================================================== */
/*  HTTP handlers                                                       */
/* ================================================================== */

static esp_err_t options_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Private-Network", "true");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}


static esp_err_t sensors_get_handler(httpd_req_t *req)
{
    s_active_requests++;
    read_sensors();

    char at[16]="null", ah[16]="null", bt[16]="null", bp[16]="null";
    if (xSemaphoreTake(s_sensor_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (s_aht20_ok) { snprintf(at,16,"%.2f",s_aht_temp); snprintf(ah,16,"%.2f",s_aht_hum); }
        if (s_bmp280_ok){ snprintf(bt,16,"%.2f",s_bmp_temp); snprintf(bp,16,"%.2f",s_bmp_press); }
        xSemaphoreGive(s_sensor_mutex);
    }

    char resp[256];
    snprintf(resp, sizeof(resp),
        "{\"ok\":true,"
        "\"aht20\":{\"detected\":%s,\"temperature\":%s,\"humidity\":%s},"
        "\"bmp280\":{\"detected\":%s,\"temperature\":%s,\"pressure_hpa\":%s},"
        "\"uptime_s\":%ld,\"boot_count\":%lu}",
        s_aht20_ok?"true":"false", at, ah,
        s_bmp280_ok?"true":"false", bt, bp,
        (long)awake_seconds(), (unsigned long)rtc.boot_count);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Private-Network", "true");
    esp_err_t err = httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    s_active_requests--;
    return err;
}

static esp_err_t status_get_handler(httpd_req_t *req)
{
    s_active_requests++;

    esp_netif_ip_info_t ip = {0};
    esp_netif_get_ip_info(s_eth_netif, &ip);
    const esp_partition_t *running = esp_ota_get_running_partition();

    char resp[768];
    snprintf(resp, sizeof(resp),
        "{\"ok\":true,"
        "\"ip\":\"" IPSTR "\","
        "\"gw\":\"" IPSTR "\","
        "\"netmask\":\"" IPSTR "\","
        "\"running_partition\":\"%s\","
        "\"aht20_detected\":%s,"
        "\"bmp280_detected\":%s,"
        "\"uptime_s\":%ld,"
        "\"boot_count\":%lu,"
        "\"continuous\":%s,"
        "\"sleep_duration_s\":%lu,"
        "\"idle_timeout_s\":%lu,"
        "\"max_awake_s\":%lu}",
        IP2STR(&ip.ip), IP2STR(&ip.gw), IP2STR(&ip.netmask),
        running ? running->label : "unknown",
        s_aht20_ok?"true":"false",
        s_bmp280_ok?"true":"false",
        (long)awake_seconds(),
        (unsigned long)rtc.boot_count,
        LP.continuous?"true":"false",
        (unsigned long)LP.sleep_duration_s,
        (unsigned long)LP.idle_timeout_s,
        (unsigned long)LP.max_awake_s);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Private-Network", "true");
    esp_err_t err = httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    s_active_requests--;
    return err;
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    s_active_requests++;
    httpd_resp_set_type(req, "text/html");

    const char *head =
        "<!doctype html><html lang='es'>"
        "<head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>ESP32 LP Sensors</title>"
        "<style>"
        "*{box-sizing:border-box;margin:0;padding:0}"
        "body{font-family:'Courier New',monospace;background:#0a0c10;color:#c9d1d9;padding:16px}"
        ".topbar{display:flex;align-items:center;justify-content:space-between;"
        "  padding-bottom:12px;border-bottom:1px solid #21262d;margin-bottom:16px}"
        ".title{font-size:13px;font-weight:700;letter-spacing:.12em;color:#e6edf3}"
        ".badge{font-size:10px;padding:2px 8px;border-radius:3px;font-weight:700;letter-spacing:.08em}"
        ".badge-ok{background:#0d2a0d;color:#3fb950;border:1px solid #238636}"
        ".badge-off{background:#2d0f0f;color:#f85149;border:1px solid #6e2020}"
        ".badge-warn{background:#2a1f00;color:#d29922;border:1px solid #6e4c00}"
        ".live{font-size:11px;color:#8b949e;display:flex;align-items:center;gap:6px}"
        ".dot{width:7px;height:7px;border-radius:50%;background:#3fb950}"
        ".grid4{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:10px;margin-bottom:14px}"
        ".grid2{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px;margin-bottom:14px}"
        "@media(max-width:640px){.grid4{grid-template-columns:repeat(2,1fr)}.grid2{grid-template-columns:1fr}}"
        ".metric{background:#0d1117;border:1px solid #21262d;border-radius:6px;padding:12px 14px}"
        ".mlabel{font-size:10px;color:#8b949e;letter-spacing:.1em;text-transform:uppercase;margin-bottom:6px}"
        ".mval{font-size:26px;font-weight:700;color:#e6edf3}"
        ".munit{font-size:12px;color:#8b949e;margin-left:3px}"
        ".msub{font-size:10px;margin-top:5px}"
        ".chart-card{background:#0d1117;border:1px solid #21262d;border-radius:6px;padding:14px}"
        ".chart-hdr{display:flex;align-items:center;justify-content:space-between;margin-bottom:10px}"
        ".chart-title{font-size:10px;color:#8b949e;text-transform:uppercase;letter-spacing:.1em}"
        ".legend{display:flex;gap:12px;font-size:10px;color:#8b949e}"
        ".ld{width:8px;height:8px;border-radius:1px;display:inline-block;margin-right:3px}"
        ".lp-card{background:#0d1117;border:1px solid #21262d;border-radius:6px;padding:12px 14px;margin-bottom:14px}"
        ".lp-row{display:flex;gap:20px;flex-wrap:wrap;font-size:11px;color:#8b949e}"
        ".lp-item{display:flex;flex-direction:column;gap:2px}"
        ".lp-val{font-size:13px;color:#e6edf3}"
        ".progress-bar{background:#161b22;border-radius:4px;height:6px;overflow:hidden;margin-top:8px}"
        ".progress-fill{height:100%;border-radius:4px;background:#378ADD;transition:width 1s linear}"
        ".sep{height:1px;background:#21262d;margin:14px 0}"
        ".inforow{display:flex;gap:20px;flex-wrap:wrap;font-size:11px;color:#8b949e}"
        ".iitem{display:flex;flex-direction:column;gap:2px}"
        ".ival{font-size:12px;color:#e6edf3}"
        ".ticker{font-size:10px;color:#484f58;text-align:right;margin-top:10px}"
        "</style></head><body>";

    const char *body1 =
        "<div class='topbar'>"
        "  <div style='display:flex;align-items:center;gap:10px'>"
        "    <span class='title'>ESP32 LP · SENSOR DASHBOARD</span>"
        "    <span class='badge badge-ok'>ETH ONLINE</span>"
        "  </div>"
        "  <div class='live'><div class='dot'></div><span id='tick'>LIVE</span></div>"
        "</div>"

        "<div class='lp-card'>"
        "  <div class='lp-row'>"
        "    <div class='lp-item'><span style='font-size:9px;letter-spacing:.1em;text-transform:uppercase;color:#8b949e'>Ciclo</span>"
        "      <span class='lp-val' id='lp-boot'>-</span></div>"
        "    <div class='lp-item'><span style='font-size:9px;letter-spacing:.1em;text-transform:uppercase;color:#8b949e'>Despierto</span>"
        "      <span class='lp-val' id='lp-up'>-</span></div>"
        "    <div class='lp-item'><span style='font-size:9px;letter-spacing:.1em;text-transform:uppercase;color:#8b949e'>Sleep</span>"
        "      <span class='lp-val' id='lp-sleep'>-</span></div>"
        "    <div class='lp-item'><span style='font-size:9px;letter-spacing:.1em;text-transform:uppercase;color:#8b949e'>Idle timeout</span>"
        "      <span class='lp-val' id='lp-idle'>-</span></div>"
        "    <div class='lp-item'><span style='font-size:9px;letter-spacing:.1em;text-transform:uppercase;color:#8b949e'>Max despierto</span>"
        "      <span class='lp-val' id='lp-max'>-</span></div>"
        "  </div>"
        "  <div class='progress-bar'><div class='progress-fill' id='awake-bar' style='width:0%'></div></div>"
        "</div>"

        "<div class='grid4'>"
        "  <div class='metric'><div class='mlabel'>AHT20 Temp</div>"
        "    <div><span class='mval' id='v-at'>--</span><span class='munit'>°C</span></div>"
        "    <div class='msub'><span id='s-aht' class='badge'>--</span></div></div>"
        "  <div class='metric'><div class='mlabel'>AHT20 Hum</div>"
        "    <div><span class='mval' id='v-ah'>--</span><span class='munit'>%</span></div>"
        "    <div class='msub'>&nbsp;</div></div>"
        "  <div class='metric'><div class='mlabel'>BMP280 Temp</div>"
        "    <div><span class='mval' id='v-bt'>--</span><span class='munit'>°C</span></div>"
        "    <div class='msub'><span id='s-bmp' class='badge'>--</span></div></div>"
        "  <div class='metric'><div class='mlabel'>Presión</div>"
        "    <div><span class='mval' id='v-bp'>--</span><span class='munit'>hPa</span></div>"
        "    <div class='msub'>&nbsp;</div></div>"
        "</div>";

    const char *body2 =
        "<div class='grid2'>"
        "  <div class='chart-card'>"
        "    <div class='chart-hdr'>"
        "      <span class='chart-title'>Temperatura</span>"
        "      <div class='legend'>"
        "        <span><span class='ld' style='background:#378ADD'></span>AHT20</span>"
        "        <span><span class='ld' style='background:#EF9F27'></span>BMP280</span>"
        "      </div></div>"
        "    <div style='position:relative;width:100%;height:160px'><canvas id='cTemp'></canvas></div>"
        "  </div>"
        "  <div class='chart-card'>"
        "    <div class='chart-hdr'>"
        "      <span class='chart-title'>Humedad / Presión</span>"
        "      <div class='legend'>"
        "        <span><span class='ld' style='background:#1D9E75'></span>Hum</span>"
        "        <span><span class='ld' style='background:#D4537E'></span>hPa</span>"
        "      </div></div>"
        "    <div style='position:relative;width:100%;height:160px'><canvas id='cHP'></canvas></div>"
        "  </div>"
        "</div>"

        "<div class='sep'></div>"
        "<div class='inforow'>"
        "  <div class='iitem'><span style='font-size:9px;color:#8b949e;letter-spacing:.1em;text-transform:uppercase'>IP</span>"
        "    <span class='ival' id='ip'>-</span></div>"
        "  <div class='iitem'><span style='font-size:9px;color:#8b949e;letter-spacing:.1em;text-transform:uppercase'>Gateway</span>"
        "    <span class='ival' id='gw'>-</span></div>"
        "  <div class='iitem'><span style='font-size:9px;color:#8b949e;letter-spacing:.1em;text-transform:uppercase'>Partición</span>"
        "    <span class='ival' id='part'>-</span></div>"
        "  <div class='iitem'><span style='font-size:9px;color:#8b949e;letter-spacing:.1em;text-transform:uppercase'>OTA</span>"
        "    <span class='ival'>POST /update</span></div>"
        "</div>"
        "<div class='ticker' id='counter'>muestras: 0</div>";

    const char *script =
        "<script src='https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.1/chart.umd.js'></script>"
        "<script>"
        "const MAX=40;"
        "const mkLabels=()=>Array(MAX).fill('');"
        "const mkData=()=>Array(MAX).fill(null);"
        "const sharedOpts={"
        "  responsive:true,maintainAspectRatio:false,"
        "  animation:{duration:400},"
        "  plugins:{legend:{display:false}},"
        "  elements:{point:{radius:0},line:{tension:0.4,borderWidth:1.5}}"
        "};"
        "const gridColor='rgba(255,255,255,.06)';"
        "const mkScale=cb=>({grid:{color:gridColor},ticks:{font:{size:9},color:'#484f58',callback:cb}});"

        "const cT=new Chart(document.getElementById('cTemp'),{"
        "  type:'line',"
        "  data:{labels:mkLabels(),datasets:["
        "    {label:'AHT20',data:mkData(),borderColor:'#378ADD',backgroundColor:'rgba(55,138,221,.08)',fill:true},"
        "    {label:'BMP280',data:mkData(),borderColor:'#EF9F27',backgroundColor:'rgba(239,159,39,.08)',fill:true}"
        "  ]},"
        "  options:{...sharedOpts,scales:{x:{display:false},y:mkScale(v=>v==null?'':v.toFixed(1)+'°')}}"
        "});"
        "const cH=new Chart(document.getElementById('cHP'),{"
        "  type:'line',"
        "  data:{labels:mkLabels(),datasets:["
        "    {label:'Hum',data:mkData(),borderColor:'#1D9E75',backgroundColor:'rgba(29,158,117,.08)',fill:true,yAxisID:'yL'},"
        "    {label:'hPa',data:mkData(),borderColor:'#D4537E',backgroundColor:'rgba(212,83,126,.08)',fill:true,yAxisID:'yR'}"
        "  ]},"
        "  options:{...sharedOpts,scales:{"
        "    x:{display:false},"
        "    yL:{position:'left',grid:{color:gridColor},ticks:{font:{size:9},color:'rgba(29,158,117,.8)',callback:v=>v==null?'':v.toFixed(0)+'%'}},"
        "    yR:{position:'right',grid:{display:false},ticks:{font:{size:9},color:'rgba(212,83,126,.8)',callback:v=>v==null?'':v.toFixed(0)}}"
        "  }}"
        "});"

        "function push(c,vals){"
        "  if(c.data.labels.length>=MAX){c.data.labels.shift();c.data.datasets.forEach(d=>d.data.shift());}"
        "  c.data.labels.push('');vals.forEach((v,i)=>c.data.datasets[i].data.push(v));c.update('none');"
        "}"
        "function setBadge(id,ok){"
        "  const el=document.getElementById(id);"
        "  el.textContent=ok?'ONLINE':'OFFLINE';"
        "  el.className='badge '+(ok?'badge-ok':'badge-off');"
        "}"
        "function txt(id,v){document.getElementById(id).textContent=v;}"
        "function fmt(v,d){return(v===null||v===undefined)?'--':Number(v).toFixed(d||2);}"

        "let count=0,elapsed=0,maxAwake=60;"
        "async function refresh(){"
        "  try{"
        "    const sj=await(await fetch('/status',{cache:'no-store'})).json();"
        "    txt('ip',sj.ip||'-');txt('gw',sj.gw||'-');txt('part',sj.running_partition||'-');"
        "    txt('lp-boot','#'+sj.boot_count);"
        "    txt('lp-up', sj.uptime_s+'s');"
        "    txt('lp-sleep', sj.continuous?'—':sj.sleep_duration_s+'s');"
        "    txt('lp-idle', sj.idle_timeout_s+'s');"
        "    txt('lp-max', sj.continuous?'—':sj.max_awake_s+'s');"
        "    maxAwake=sj.max_awake_s||60;"
        "    const pct=Math.min(100,Math.round(sj.uptime_s/maxAwake*100));"
        "    document.getElementById('awake-bar').style.width=pct+'%';"
        "    document.getElementById('awake-bar').style.background=pct>80?'#f85149':pct>60?'#d29922':'#378ADD';"
        "  }catch(e){}"
        "  try{"
        "    const j=await(await fetch('/sensors',{cache:'no-store'})).json();"
        "    const aOk=!!(j.aht20&&j.aht20.detected);"
        "    const bOk=!!(j.bmp280&&j.bmp280.detected);"
        "    setBadge('s-aht',aOk);setBadge('s-bmp',bOk);"
        "    txt('v-at',fmt(j.aht20?j.aht20.temperature:null,1));"
        "    txt('v-ah',fmt(j.aht20?j.aht20.humidity:null,1));"
        "    txt('v-bt',fmt(j.bmp280?j.bmp280.temperature:null,1));"
        "    txt('v-bp',fmt(j.bmp280?j.bmp280.pressure_hpa:null,1));"
        "    push(cT,[aOk?j.aht20.temperature:null,bOk?j.bmp280.temperature:null]);"
        "    push(cH,[aOk?j.aht20.humidity:null,bOk?j.bmp280.pressure_hpa:null]);"
        "    count++;elapsed=0;"
        "    txt('counter','muestras: '+count);"
        "  }catch(e){}"
        "}"
        "refresh();"
        "setInterval(refresh,3000);"
        "setInterval(()=>{elapsed++;txt('tick','LIVE — actualizado hace '+elapsed+'s');},1000);"
        "</script></body></html>";
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Private-Network", "true");
    httpd_resp_send_chunk(req, head,   HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, body1,  HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, body2,  HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, script, HTTPD_RESP_USE_STRLEN);
    esp_err_t err = httpd_resp_send_chunk(req, NULL, 0);
    s_active_requests--;
    return err;
}

static esp_err_t ota_post_handler(httpd_req_t *req)
{
    s_active_requests++;
    const esp_partition_t *up = esp_ota_get_next_update_partition(NULL);
    if (!up) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
        s_active_requests--;
        return ESP_FAIL;
    }

    esp_ota_handle_t h = 0;
    if (esp_ota_begin(up, OTA_SIZE_UNKNOWN, &h) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota_begin failed");
        s_active_requests--;
        return ESP_FAIL;
    }

    char buf[1024];
    int remaining = req->content_len;
    while (remaining > 0) {
        int n = httpd_req_recv(req, buf, remaining > (int)sizeof(buf) ? (int)sizeof(buf) : remaining);
        if (n <= 0 || esp_ota_write(h, buf, n) != ESP_OK) {
            esp_ota_abort(h);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA write failed");
            s_active_requests--;
            return ESP_FAIL;
        }
        remaining -= n;
    }

    if (esp_ota_end(h) != ESP_OK || esp_ota_set_boot_partition(up) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA finalize failed");
        s_active_requests--;
        return ESP_FAIL;
    }

    httpd_resp_sendstr(req, "OTA OK. Rebooting...");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

/* ================================================================== */
/*  HTTP server                                                         */
/* ================================================================== */
static httpd_handle_t start_webserver(void)
{
    httpd_config_t cfg  = HTTPD_DEFAULT_CONFIG();
    cfg.server_port     = 80;
    cfg.max_uri_handlers = 8;

    httpd_handle_t srv = NULL;
    if (httpd_start(&srv, &cfg) != ESP_OK) return NULL;

    const httpd_uri_t uris[] = {
        { .uri="/*",      .method=HTTP_OPTIONS, .handler=options_handler  },
        { .uri="/",       .method=HTTP_GET,  .handler=root_get_handler    },
        { .uri="/status", .method=HTTP_GET,  .handler=status_get_handler  },
        { .uri="/sensors",.method=HTTP_GET,  .handler=sensors_get_handler },
        { .uri="/update", .method=HTTP_POST, .handler=ota_post_handler    },
    };
    for (int i = 0; i < 5; i++) httpd_register_uri_handler(srv, &uris[i]);
    ESP_LOGI(TAG, "HTTP server iniciado");
    return srv;
}

/* ================================================================== */
/*  Ethernet                                                            */
/* ================================================================== */
static void on_eth_event(void *arg, esp_event_base_t base,
                         int32_t id, void *data)
{
    switch (id) {
        case ETHERNET_EVENT_START:
            ESP_LOGI(TAG, "[ETH] Driver started");
            break;
        case ETHERNET_EVENT_STOP:
            ESP_LOGW(TAG, "[ETH] Driver stopped");
            xEventGroupSetBits(s_eth_eg, ETH_LINK_DOWN_BIT);
            break;
        case ETHERNET_EVENT_CONNECTED: {
            uint8_t mac[6] = {0};
            esp_eth_ioctl(s_eth_handle, ETH_CMD_G_MAC_ADDR, mac);
            ESP_LOGI(TAG, "[ETH] Link UP -- MAC %02x:%02x:%02x:%02x:%02x:%02x",
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
            xEventGroupSetBits(s_eth_eg, ETH_LINK_UP_BIT);
            break;
        }
        case ETHERNET_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "[ETH] Link DOWN");
            xEventGroupSetBits(s_eth_eg, ETH_LINK_DOWN_BIT);
            break;
        default:
            ESP_LOGI(TAG, "[ETH] Event id=%ld", (long)id);
            break;
    }
}

static void set_static_ip(void)
{
    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(s_eth_netif));
    esp_netif_ip_info_t ip = {0};
    ip.ip.addr      = ESP_IP4TOADDR(192, 168, 10, 99);
    ip.gw.addr      = ESP_IP4TOADDR(192, 168, 10, 100);
    ip.netmask.addr = ESP_IP4TOADDR(255, 255, 255, 0);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(s_eth_netif, &ip));
}

static void init_ethernet(void)
{
    /*
     * Ciclo de reset explícito del PHY.
     * Tras deep sleep el ESP32 resetea pero el LAN8720 puede no perder
     * alimentación, quedando en estado inconsistente. Bajar GPIO16
     * durante >=100 ms fuerza al PHY a reiniciarse completamente antes
     * de que el EMAC intente negociar el enlace RMII.
     */
    ESP_LOGI(TAG, "[ETH] PHY power OFF (GPIO%d)", ETH_PHY_POWER_GPIO);
    /* Liberar hold antes de cambiar el nivel (puede estar activo
     * desde el sleep anterior). */
    gpio_hold_dis(ETH_PHY_POWER_GPIO);
    gpio_reset_pin(ETH_PHY_POWER_GPIO);
    gpio_set_direction(ETH_PHY_POWER_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(ETH_PHY_POWER_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(150));
    ESP_LOGI(TAG, "[ETH] PHY power ON, esperando estabilizacion...");
    gpio_set_level(ETH_PHY_POWER_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(300));
    ESP_LOGI(TAG, "[ETH] PHY listo, iniciando EMAC");

    esp_netif_config_t nc = ESP_NETIF_DEFAULT_ETH();
    s_eth_netif = esp_netif_new(&nc);

    esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, on_eth_event, NULL);

    eth_mac_config_t        mc  = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t        pc  = ETH_PHY_DEFAULT_CONFIG();
    pc.phy_addr       = ETH_PHY_ADDR;
    pc.reset_gpio_num = ETH_PHY_RST_GPIO;
    pc.reset_timeout_ms    = 1000;
    pc.autonego_timeout_ms = 5000;

    eth_esp32_emac_config_t ec  = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    ec.smi_gpio.mdc_num         = ETH_MDC_GPIO;
    ec.smi_gpio.mdio_num        = ETH_MDIO_GPIO;
    ec.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    ec.clock_config.rmii.clock_gpio = EMAC_CLK_IN_GPIO;

    ESP_LOGI(TAG, "[ETH] Instalando driver (PHY addr=%d, MDC=%d, MDIO=%d)",
             ETH_PHY_ADDR, ETH_MDC_GPIO, ETH_MDIO_GPIO);
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&ec, &mc);
    esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&pc);
    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);

    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_cfg, &s_eth_handle));
    ESP_LOGI(TAG, "[ETH] Driver instalado OK");
    ESP_ERROR_CHECK(esp_netif_attach(s_eth_netif, esp_eth_new_netif_glue(s_eth_handle)));
    ESP_LOGI(TAG, "[ETH] Netif attached, arrancando ETH...");
    ESP_ERROR_CHECK(esp_eth_start(s_eth_handle));
    ESP_LOGI(TAG, "[ETH] esp_eth_start OK — esperando negociacion RMII");
}


static void init_wifi_ap(void)
{
    /* Crear netif para el AP */
    s_wifi_ap_netif = esp_netif_create_default_wifi_ap();

    /* Configurar IP estática del AP */
    esp_netif_ip_info_t ap_ip = {0};
    esp_netif_dhcps_stop(s_wifi_ap_netif);
    ap_ip.ip.addr      = esp_ip4addr_aton(WIFI_AP_IP);
    ap_ip.gw.addr      = esp_ip4addr_aton(WIFI_AP_GW);
    ap_ip.netmask.addr = esp_ip4addr_aton(WIFI_AP_NETMASK);
    esp_netif_set_ip_info(s_wifi_ap_netif, &ap_ip);
    esp_netif_dhcps_start(s_wifi_ap_netif);

    /* Inicializar WiFi */
    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    wifi_config_t ap_cfg = {
    .ap = {
        .ssid           = WIFI_AP_SSID,
        .ssid_len       = strlen(WIFI_AP_SSID),
        .channel        = WIFI_AP_CHANNEL,
        .password       = WIFI_AP_PASS,
        .max_connection = WIFI_AP_MAX_CONN,
        .authmode       = WIFI_AUTH_WPA2_PSK,
        .pmf_cfg = {
            .required = false,
            .capable  = false,   /* desactiva PMF completamente */
        },
    },
};

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));

esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
esp_wifi_config_80211_tx_rate(WIFI_IF_AP, WIFI_PHY_RATE_MCS7_SGI); /* máximo throughput */

    ESP_ERROR_CHECK(esp_wifi_start());

    esp_wifi_set_inactive_time(WIFI_IF_AP, 60);  /* keepalive más agresivo */


    ESP_LOGI(TAG, "[WIFI] AP iniciado — SSID:%s  IP:%s", WIFI_AP_SSID, WIFI_AP_IP);
}

static void enable_nat(void)
{
    /* NAT del AP WiFi (192.168.4.x) hacia ETH (192.168.10.x) */
    esp_netif_t *ap  = s_wifi_ap_netif;
    esp_netif_t *eth = s_eth_netif;

    /* Habilitar IP forwarding entre interfaces */
    esp_netif_napt_enable(ap);
    ESP_LOGI(TAG, "[NAT] NAPT habilitado: 192.168.4.x → 192.168.10.x");
    (void)eth;
}
/* ================================================================== */
/*  Tarea principal de gestión del ciclo LP                            */
/* ================================================================== */
static void lp_manager_task(void *arg)
{
    /* 1. Esperar link Ethernet (con timeout) */
    ESP_LOGI(TAG, "[LP] Esperando link ETH (timeout %lus)...",
             (unsigned long)LP.link_timeout_s);
    EventBits_t bits = xEventGroupWaitBits(
        s_eth_eg, ETH_LINK_UP_BIT, pdFALSE, pdFALSE,
        pdMS_TO_TICKS(LP.link_timeout_s * 1000));

    if (!(bits & ETH_LINK_UP_BIT)) {
        ESP_LOGW(TAG, "[LP] Link timeout tras %lus — durmiendo",
                 (unsigned long)LP.link_timeout_s);
        go_to_sleep("link_timeout");
    }
    ESP_LOGI(TAG, "[LP] Link UP en %lds desde boot", (long)awake_seconds());

    /* 2. Link up: configurar IP y arrancar servidor */
    set_static_ip();

    vTaskDelay(pdMS_TO_TICKS(100));
    enable_nat();          /* <-- añadir esta línea */
    s_http_srv = start_webserver();


    /* 3. Bucle de watchdog */
    int32_t last_request_s = awake_seconds();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        int32_t now = awake_seconds();

        /* ¿Petición en curso? resetear timer de inactividad */
        if (s_active_requests > 0) {
            last_request_s = now;
        }

        /* En modo continuo solo vigilamos link_down */
        if (LP.continuous) {
            if (xEventGroupGetBits(s_eth_eg) & ETH_LINK_DOWN_BIT) {
                ESP_LOGW(TAG, "[LP] Link caido en modo continuo — reiniciando");
                esp_restart();
            }
            continue;
        }

        /* ¿Tiempo maximo despierto superado? */
        if (now >= (int32_t)LP.max_awake_s) {
            for (int w = 0; w < 5 && s_active_requests > 0; w++) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            go_to_sleep("max_awake");
        }

        /* ¿Link caido? */
        if (xEventGroupGetBits(s_eth_eg) & ETH_LINK_DOWN_BIT) {
            go_to_sleep("link_down");
        }

        /* ¿Inactividad superada? */
        if ((now - last_request_s) >= (int32_t)LP.idle_timeout_s) {
            go_to_sleep("idle_timeout");
        }
    }
}

/* ================================================================== */
/*  app_main                                                            */
/* ================================================================== */
void app_main(void)
{
    s_boot_us = esp_timer_get_time();
    rtc.boot_count++;

    esp_sleep_wakeup_cause_t wakeup = esp_sleep_get_wakeup_cause();
    const char *wakeup_str =
        wakeup == ESP_SLEEP_WAKEUP_TIMER    ? "TIMER"    :
        wakeup == ESP_SLEEP_WAKEUP_UNDEFINED? "POWER_ON" : "OTHER";
    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, "Boot #%lu | wakeup: %s (%d)",
             (unsigned long)rtc.boot_count, wakeup_str, (int)wakeup);
    ESP_LOGI(TAG, "Modo: %s", LP.continuous ? "CONTINUO (sin sleep)" : "LOW POWER");
    ESP_LOGI(TAG, "Sleep config: dormido=%lus  idle=%lus  max_awake=%lus",
             (unsigned long)LP.sleep_duration_s,
             (unsigned long)LP.idle_timeout_s,
             (unsigned long)LP.max_awake_s);
    ESP_LOGI(TAG, "============================================");

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    s_sensor_mutex = xSemaphoreCreateMutex();
    s_eth_eg       = xEventGroupCreate();
    configASSERT(s_sensor_mutex);
    configASSERT(s_eth_eg);

    /* Si hay datos previos en RTC memory, usarlos como valores iniciales
     * para que el servidor pueda responder inmediatamente mientras lee */
    if (rtc.boot_count > 1) {
        s_aht_temp  = rtc.last_aht_temp;
        s_aht_hum   = rtc.last_aht_hum;
        s_bmp_temp  = rtc.last_bmp_temp;
        s_bmp_press = rtc.last_bmp_press;
        s_aht20_ok  = rtc.last_aht_valid;
        s_bmp280_ok = rtc.last_bmp_valid;
    }

    init_sensors();

    xTaskCreate(lp_manager_task, "lp_mgr", 4096, NULL, 5, NULL);


    init_wifi_ap();
    // NAT se activa después de que ETH tenga IP, desde lp_manager_task
  
    init_ethernet();
}