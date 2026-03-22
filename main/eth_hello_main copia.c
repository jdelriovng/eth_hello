#include <stdio.h>
#include <string.h>
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2c_bus.h"

#include "aht20.h"
#include "bmp280.h"

#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "driver/gpio.h"

#include "esp_eth.h"
#include "esp_mac.h"


#define I2C_SDA_GPIO 14
#define I2C_SCL_GPIO 15

static void read_sensors(void);
static i2c_bus_handle_t i2c_bus = NULL;

static bool aht20_detected = false;
static bool bmp280_detected = false;

static float aht_temp = 0;
static float aht_hum = 0;

static float bmp_temp = 0;
static float bmp_press = 0;

static aht20_dev_handle_t aht20_dev = NULL;
static bmp280_t bmp280_dev;

static const char *TAG = "ETH_HELLO";

// ESP32-ETH02 / LAN8720
#define ETH_PHY_ADDR        1
#define ETH_PHY_RST_GPIO    (-1)
#define ETH_PHY_POWER_GPIO  16
#define ETH_MDC_GPIO        23
#define ETH_MDIO_GPIO       18

static esp_netif_t *eth_netif = NULL;
static esp_eth_handle_t eth_handle = NULL;
static httpd_handle_t http_server = NULL;

static esp_err_t sensors_get_handler(httpd_req_t *req)
{
    read_sensors();

    char resp[512];
    char aht_temp_s[32] = "null";
    char aht_hum_s[32] = "null";
    char bmp_temp_s[32] = "null";
    char bmp_press_s[32] = "null";

    if (aht20_detected) {
        snprintf(aht_temp_s, sizeof(aht_temp_s), "%.2f", aht_temp);
        snprintf(aht_hum_s, sizeof(aht_hum_s), "%.2f", aht_hum);
    }

    if (bmp280_detected) {
        snprintf(bmp_temp_s, sizeof(bmp_temp_s), "%.2f", bmp_temp);
        snprintf(bmp_press_s, sizeof(bmp_press_s), "%.2f", bmp_press);
    }

    snprintf(resp, sizeof(resp),
             "{"
             "\"ok\":true,"
             "\"aht20\":{"
                 "\"detected\":%s,"
                 "\"temperature\":%s,"
                 "\"humidity\":%s"
             "},"
             "\"bmp280\":{"
                 "\"detected\":%s,"
                 "\"temperature\":%s,"
                 "\"pressure_hpa\":%s"
             "}"
             "}",
             aht20_detected ? "true" : "false",
             aht_temp_s,
             aht_hum_s,
             bmp280_detected ? "true" : "false",
             bmp_temp_s,
             bmp_press_s);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
}

static void init_sensors(void)
{
    ESP_LOGI(TAG, "Init I2C (i2c_bus)...");

    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SCL_GPIO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };

    i2c_bus = i2c_bus_create(I2C_NUM_0, &i2c_conf);

    if (!i2c_bus) {
        ESP_LOGE(TAG, "Failed to create I2C bus");
        return;
    }
for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    if (i2c_dev_probe(i2c_bus, addr) == ESP_OK) {
        ESP_LOGI(TAG, "I2C device found at 0x%02X", addr);
    }
}

    // -------- AHT20 --------
    aht20_i2c_config_t aht20_cfg = {
        .bus_inst = i2c_bus,
        .i2c_addr = AHT20_ADDRRES_0,
    };
if (aht20_new_sensor(&aht20_cfg, &aht20_dev) == ESP_OK) {
    uint32_t t_raw = 0;
    uint32_t h_raw = 0;
    float temp = 0;
    float hum = 0;

    if (aht20_read_temperature_humidity(aht20_dev, &t_raw, &temp, &h_raw, &hum) == ESP_OK) {
        aht20_detected = true;
        aht_temp = temp;
        aht_hum = hum;
        ESP_LOGI(TAG, "AHT20 detected");
    } else {
        aht20_detected = false;
        ESP_LOGW(TAG, "AHT20 not detected");
        aht20_del_sensor(aht20_dev);
        aht20_dev = NULL;
    }
} else {
    aht20_detected = false;
    ESP_LOGW(TAG, "AHT20 handle creation failed");
}

    // -------- BMP280 --------
    memset(&bmp280_dev, 0, sizeof(bmp280_dev));

    bmp280_params_t params;
    bmp280_init_default_params(&params);

    if (bmp280_init_desc(&bmp280_dev, BMP280_I2C_ADDRESS_0, I2C_NUM_0, I2C_SDA_GPIO, I2C_SCL_GPIO) == ESP_OK) {
        if (bmp280_init(&bmp280_dev, &params) == ESP_OK) {
            bmp280_detected = true;
            ESP_LOGI(TAG, "BMP280 detected");
        } else {
            bmp280_detected = false;
        }
    }
}

static void read_sensors(void)
{
    if (aht20_detected && aht20_dev) {
        uint32_t t_raw = 0;
        uint32_t h_raw = 0;
        float temp = 0;
        float hum = 0;

        if (aht20_read_temperature_humidity(
                aht20_dev,
                &t_raw, &temp,
                &h_raw, &hum) == ESP_OK) {
            aht_temp = temp;
            aht_hum = hum;
        } else {
            aht20_detected = false;
            ESP_LOGW(TAG, "AHT20 read failed, marking sensor as not detected");
        }
    }

    if (bmp280_detected) {
        float temp = 0, press = 0, hum_dummy = 0;

        if (bmp280_read_float(&bmp280_dev, &temp, &press, &hum_dummy) == ESP_OK) {
            bmp_temp = temp;
            bmp_press = press / 100.0f;
        } else {
            bmp280_detected = false;
            ESP_LOGW(TAG, "BMP280 read failed, marking sensor as not detected");
        }
    }
}




static esp_err_t status_get_handler(httpd_req_t *req)
{
    char resp[512];
    esp_netif_ip_info_t ip_info = {0};

    esp_err_t err = esp_netif_get_ip_info(eth_netif, &ip_info);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No IP info");
        return ESP_FAIL;
    }

    const esp_partition_t *running = esp_ota_get_running_partition();

    snprintf(resp, sizeof(resp),
             "{"
             "\"ok\":true,"
             "\"ip\":\"" IPSTR "\","
             "\"gw\":\"" IPSTR "\","
             "\"netmask\":\"" IPSTR "\","
             "\"running_partition\":\"%s\","
             "\"aht20_detected\":%s,"
             "\"bmp280_detected\":%s"
             "}",
             IP2STR(&ip_info.ip),
             IP2STR(&ip_info.gw),
             IP2STR(&ip_info.netmask),
             running ? running->label : "unknown",
             aht20_detected ? "true" : "false",
             bmp280_detected ? "true" : "false");

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    const char *html =
        "<!doctype html>"
        "<html lang='es'>"
        "<head>"
        "<meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>ETH Hello Sensors</title>"
        "<style>"
        "body{font-family:Arial,sans-serif;background:#0f172a;color:#e2e8f0;margin:0;padding:24px;}"
        ".wrap{max-width:900px;margin:0 auto;}"
        "h1{margin:0 0 8px;font-size:28px;}"
        ".sub{color:#94a3b8;margin-bottom:24px;}"
        ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:16px;}"
        ".card{background:#111827;border:1px solid #1f2937;border-radius:16px;padding:18px;box-shadow:0 4px 20px rgba(0,0,0,.25);}"
        ".title{font-size:14px;color:#94a3b8;margin-bottom:8px;text-transform:uppercase;letter-spacing:.08em;}"
        ".value{font-size:30px;font-weight:700;margin:6px 0;}"
        ".small{font-size:14px;color:#cbd5e1;line-height:1.6;}"
        ".ok{color:#22c55e;font-weight:700;}"
        ".bad{color:#ef4444;font-weight:700;}"
        ".muted{color:#94a3b8;}"
        ".footer{margin-top:20px;color:#94a3b8;font-size:13px;}"
        "code{background:#0b1220;padding:2px 6px;border-radius:6px;}"
        "</style>"
        "</head>"
        "<body>"
        "<div class='wrap'>"
        "<h1>ESP32 ETH Sensors</h1>"
        "<div class='sub'>Estado Ethernet, OTA y sensores en tiempo real</div>"

        "<div class='grid'>"
        "<div class='card'>"
        "<div class='title'>Red</div>"
        "<div class='small'>IP: <span id='ip'>-</span></div>"
        "<div class='small'>Gateway: <span id='gw'>-</span></div>"
        "<div class='small'>Máscara: <span id='mask'>-</span></div>"
        "<div class='small'>Partición: <span id='part'>-</span></div>"
        "</div>"

        "<div class='card'>"
        "<div class='title'>AHT20</div>"
        "<div class='small'>Estado: <span id='aht_state'>-</span></div>"
        "<div class='value'><span id='aht_temp'>--</span><span class='muted'> °C</span></div>"
        "<div class='small'>Humedad: <span id='aht_hum'>--</span> %</div>"
        "</div>"

        "<div class='card'>"
        "<div class='title'>BMP280</div>"
        "<div class='small'>Estado: <span id='bmp_state'>-</span></div>"
        "<div class='value'><span id='bmp_temp'>--</span><span class='muted'> °C</span></div>"
        "<div class='small'>Presión: <span id='bmp_press'>--</span> hPa</div>"
        "</div>"
        "</div>"

        "<div class='footer'>"
        "Endpoints: <code>/status</code> | <code>/sensors</code> | OTA: <code>POST /update</code>"
        "</div>"
        "</div>"

        "<script>"
        "function txt(id,v){document.getElementById(id).textContent=v;}"
        "function state(id,det){"
        " const el=document.getElementById(id);"
        " el.textContent=det?'detectado':'no detectado';"
        " el.className=det?'ok':'bad';"
        "}"
        "function fmt(v,d=2){"
        " return (v===null||v===undefined)?'--':Number(v).toFixed(d);"
        "}"
        "async function refresh(){"
        " try{"
        "   const st=await fetch('/status',{cache:'no-store'});"
        "   const sj=await st.json();"
        "   txt('ip', sj.ip ?? '-');"
        "   txt('gw', sj.gw ?? '-');"
        "   txt('mask', sj.netmask ?? '-');"
        "   txt('part', sj.running_partition ?? '-');"
        " }catch(e){"
        "   txt('ip','error'); txt('gw','error'); txt('mask','error'); txt('part','error');"
        " }"
        " try{"
        "   const se=await fetch('/sensors',{cache:'no-store'});"
        "   const j=await se.json();"
        "   state('aht_state', !!(j.aht20 && j.aht20.detected));"
        "   state('bmp_state', !!(j.bmp280 && j.bmp280.detected));"
        "   txt('aht_temp', fmt(j.aht20 ? j.aht20.temperature : null));"
        "   txt('aht_hum', fmt(j.aht20 ? j.aht20.humidity : null));"
        "   txt('bmp_temp', fmt(j.bmp280 ? j.bmp280.temperature : null));"
        "   txt('bmp_press', fmt(j.bmp280 ? j.bmp280.pressure_hpa : null));"
        " }catch(e){"
        "   state('aht_state', false); state('bmp_state', false);"
        "   txt('aht_temp','--'); txt('aht_hum','--'); txt('bmp_temp','--'); txt('bmp_press','--');"
        " }"
        "}"
        "refresh();"
        "setInterval(refresh, 3000);"
        "</script>"
        "</body>"
        "</html>";

    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}


static esp_err_t ota_post_handler(httpd_req_t *req)
{
    esp_ota_handle_t ota_handle = 0;
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);

    if (!update_partition) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
        return ESP_FAIL;
    }

    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "esp_ota_begin failed");
        return err;
    }

    char buf[1024];
    int remaining = req->content_len;

    while (remaining > 0) {
        int recv_len = httpd_req_recv(req, buf, remaining > sizeof(buf) ? sizeof(buf) : remaining);
        if (recv_len <= 0) {
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive failed");
            return ESP_FAIL;
        }

        err = esp_ota_write(ota_handle, buf, recv_len);
        if (err != ESP_OK) {
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "esp_ota_write failed");
            return err;
        }

        remaining -= recv_len;
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "esp_ota_end failed");
        return err;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "esp_ota_set_boot_partition failed");
        return err;
    }

    httpd_resp_sendstr(req, "OTA OK. Rebooting...");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}


static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_get_handler,
            .user_ctx = NULL
        };

        httpd_uri_t update = {
            .uri = "/update",
            .method = HTTP_POST,
            .handler = ota_post_handler,
            .user_ctx = NULL
        };

        httpd_uri_t status = {
            .uri = "/status",
            .method = HTTP_GET,
            .handler = status_get_handler,
            .user_ctx = NULL
        };
        httpd_uri_t sensors = {
            .uri = "/sensors",
            .method = HTTP_GET,
            .handler = sensors_get_handler,
            .user_ctx = NULL
        };
httpd_register_uri_handler(server, &root);
httpd_register_uri_handler(server, &status);
httpd_register_uri_handler(server, &sensors);
httpd_register_uri_handler(server, &update);

        ESP_LOGI(TAG, "HTTP server started");
    }

    return server;
}

static void stop_webserver(void)
{
    if (http_server) {
        httpd_stop(http_server);
        http_server = NULL;
        ESP_LOGI(TAG, "HTTP server stopped");
    }
}

static void set_static_ip(void)
{
    esp_netif_ip_info_t ip_info = {0};

    ip_info.ip.addr      = ESP_IP4TOADDR(192, 168, 10, 99);
    ip_info.gw.addr      = ESP_IP4TOADDR(192, 168, 10, 100);
    ip_info.netmask.addr = ESP_IP4TOADDR(255, 255, 255, 0);

    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(eth_netif));
    ESP_ERROR_CHECK(esp_netif_set_ip_info(eth_netif, &ip_info));

    ESP_LOGI(TAG, "Static IP configured:");
    ESP_LOGI(TAG, "IP      : 192.168.10.99");
    ESP_LOGI(TAG, "MASK    : 255.255.255.0");
    ESP_LOGI(TAG, "GATEWAY : 192.168.10.100");
}
static void on_eth_event(void *arg, esp_event_base_t event_base,
                         int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};

    switch (event_id) {
        case ETHERNET_EVENT_CONNECTED:
            esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
            ESP_LOGI(TAG, "Ethernet Link Up");
            ESP_LOGI(TAG, "MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                     mac_addr[0], mac_addr[1], mac_addr[2],
                     mac_addr[3], mac_addr[4], mac_addr[5]);

            set_static_ip();
            vTaskDelay(pdMS_TO_TICKS(200));

            if (!http_server) {
                http_server = start_webserver();
            }
            break;

        case ETHERNET_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "Ethernet Link Down");
            stop_webserver();
            break;

        case ETHERNET_EVENT_START:
            ESP_LOGI(TAG, "Ethernet Started");
            break;

        case ETHERNET_EVENT_STOP:
            ESP_LOGW(TAG, "Ethernet Stopped");
            stop_webserver();
            break;

        default:
            ESP_LOGI(TAG, "Ethernet event id=%ld", (long)event_id);
            break;
    }
}

static void init_ethernet(void)
{
    gpio_reset_pin(ETH_PHY_POWER_GPIO);
    gpio_set_direction(ETH_PHY_POWER_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(ETH_PHY_POWER_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    eth_netif = esp_netif_new(&netif_cfg);
    ESP_ERROR_CHECK(eth_netif ? ESP_OK : ESP_FAIL);

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &on_eth_event, NULL));

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();

    phy_config.phy_addr = ETH_PHY_ADDR;
    phy_config.reset_gpio_num = ETH_PHY_RST_GPIO;

    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    esp32_emac_config.smi_gpio.mdc_num = ETH_MDC_GPIO;
    esp32_emac_config.smi_gpio.mdio_num = ETH_MDIO_GPIO;
    esp32_emac_config.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    esp32_emac_config.clock_config.rmii.clock_gpio = EMAC_CLK_IN_GPIO;

    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_config);

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);

    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
}

void app_main(void)
{

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_LOGI(TAG, "Starting Ethernet Hello World via OTA");
    init_sensors();
    init_ethernet();
}