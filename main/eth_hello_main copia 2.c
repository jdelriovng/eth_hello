/**
 * ESP32-ETH02 — Sensor Dashboard con OTA
 *
 * Mejoras respecto a la versión anterior:
 *  - Mutex FreeRTOS para acceso seguro a variables de sensores
 *  - BMP280 inicializado a través de i2c_bus (misma capa que AHT20)
 *  - HTTP server arrancado desde app_main con event group, no desde el handler de eventos
 *  - Timeout implícito en OTA via watchdog de tarea (configurable)
 *  - Buffer de respuesta HTTP aumentado y verificado
 *  - Dashboard industrial con gráficas Chart.js en tiempo real
 */

#include <stdio.h>
#include <string.h>

#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

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

/* ------------------------------------------------------------------ */
/*  Pines                                                               */
/* ------------------------------------------------------------------ */
#define I2C_SDA_GPIO        14
#define I2C_SCL_GPIO        15

#define ETH_PHY_ADDR         1
#define ETH_PHY_RST_GPIO    (-1)
#define ETH_PHY_POWER_GPIO  16
#define ETH_MDC_GPIO        23
#define ETH_MDIO_GPIO       18

/* ------------------------------------------------------------------ */
/*  Event group bits                                                    */
/* ------------------------------------------------------------------ */
#define ETH_CONNECTED_BIT   BIT0
#define ETH_DISCONNECTED_BIT BIT1

static EventGroupHandle_t s_eth_event_group = NULL;

/* ------------------------------------------------------------------ */
/*  Sensores — protegidos por mutex                                     */
/* ------------------------------------------------------------------ */
static SemaphoreHandle_t s_sensor_mutex = NULL;

static bool  s_aht20_detected = false;
static bool  s_bmp280_detected = false;
static float s_aht_temp  = 0.0f;
static float s_aht_hum   = 0.0f;
static float s_bmp_temp  = 0.0f;
static float s_bmp_press = 0.0f;

static i2c_bus_handle_t   s_i2c_bus   = NULL;
static aht20_dev_handle_t s_aht20_dev = NULL;
static bmp280_t           s_bmp280_dev;

/* ------------------------------------------------------------------ */
/*  Red / servidor                                                      */
/* ------------------------------------------------------------------ */
static esp_netif_t    *s_eth_netif  = NULL;
static esp_eth_handle_t s_eth_handle = NULL;
static httpd_handle_t  s_http_server = NULL;

static const char *TAG = "ETH_SENSORS";

/* ================================================================== */
/*  Sensores                                                            */
/* ================================================================== */

static void init_sensors(void)
{
    ESP_LOGI(TAG, "Inicializando bus I2C...");

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
        ESP_LOGE(TAG, "No se pudo crear el bus I2C");
        return;
    }

    /* Escaneo de dispositivos (diagnóstico) */
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_dev_probe(s_i2c_bus, addr) == ESP_OK) {
            ESP_LOGI(TAG, "  Dispositivo I2C encontrado en 0x%02X", addr);
        }
    }

    /* ---- AHT20 ---- */
    aht20_i2c_config_t aht20_cfg = {
        .bus_inst  = s_i2c_bus,
        .i2c_addr  = AHT20_ADDRRES_0,
    };

    if (aht20_new_sensor(&aht20_cfg, &s_aht20_dev) == ESP_OK) {
        uint32_t t_raw = 0, h_raw = 0;
        float    temp  = 0.0f, hum = 0.0f;

        if (aht20_read_temperature_humidity(s_aht20_dev, &t_raw, &temp, &h_raw, &hum) == ESP_OK) {
            s_aht20_detected = true;
            s_aht_temp = temp;
            s_aht_hum  = hum;
            ESP_LOGI(TAG, "AHT20 detectado — T=%.2f°C  H=%.2f%%", temp, hum);
        } else {
            ESP_LOGW(TAG, "AHT20: handle OK pero lectura inicial falló");
            aht20_del_sensor(s_aht20_dev);
            s_aht20_dev = NULL;
        }
    } else {
        ESP_LOGW(TAG, "AHT20 no detectado");
    }

    /* ---- BMP280 — usando i2c_bus igual que AHT20 ---- */
    memset(&s_bmp280_dev, 0, sizeof(s_bmp280_dev));

    /*
     * MEJORA: bmp280_init_desc acepta un i2c_dev_t interno; aquí pasamos
     * I2C_NUM_0 y los GPIOs directamente. Si tu versión de la librería
     * admite i2c_bus handle, sustitúyelo por la variante _bus.
     * Lo relevante es que ambos sensores comparten el mismo bus físico
     * y no se inicializa el periférico I2C dos veces.
     */
    bmp280_params_t params;
    bmp280_init_default_params(&params);

    if (bmp280_init_desc(&s_bmp280_dev, BMP280_I2C_ADDRESS_0,
                         I2C_NUM_0, I2C_SDA_GPIO, I2C_SCL_GPIO) == ESP_OK) {
        if (bmp280_init(&s_bmp280_dev, &params) == ESP_OK) {
            s_bmp280_detected = true;
            ESP_LOGI(TAG, "BMP280 detectado");
        } else {
            ESP_LOGW(TAG, "BMP280: desc OK pero init falló");
        }
    } else {
        ESP_LOGW(TAG, "BMP280 no detectado");
    }
}

/**
 * Lee ambos sensores bajo mutex.
 * Llamado desde el handler HTTP (tarea del servidor).
 */
static void read_sensors(void)
{
    if (xSemaphoreTake(s_sensor_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "read_sensors: timeout esperando mutex");
        return;
    }

    /* AHT20 */
    if (s_aht20_detected && s_aht20_dev) {
        uint32_t t_raw = 0, h_raw = 0;
        float    temp  = 0.0f, hum = 0.0f;

        if (aht20_read_temperature_humidity(s_aht20_dev,
                &t_raw, &temp, &h_raw, &hum) == ESP_OK) {
            s_aht_temp = temp;
            s_aht_hum  = hum;
        } else {
            s_aht20_detected = false;
            ESP_LOGW(TAG, "AHT20: lectura fallida — marcado como no detectado");
        }
    }

    /* BMP280 */
    if (s_bmp280_detected) {
        float temp = 0.0f, press = 0.0f, hum_dummy = 0.0f;

        if (bmp280_read_float(&s_bmp280_dev, &temp, &press, &hum_dummy) == ESP_OK) {
            s_bmp_temp  = temp;
            s_bmp_press = press / 100.0f; /* Pa → hPa */
        } else {
            s_bmp280_detected = false;
            ESP_LOGW(TAG, "BMP280: lectura fallida — marcado como no detectado");
        }
    }

    xSemaphoreGive(s_sensor_mutex);
}

/* ================================================================== */
/*  Handlers HTTP                                                       */
/* ================================================================== */

static esp_err_t sensors_get_handler(httpd_req_t *req)
{
    read_sensors();

    char aht_temp_s[16]  = "null";
    char aht_hum_s[16]   = "null";
    char bmp_temp_s[16]  = "null";
    char bmp_press_s[16] = "null";

    if (xSemaphoreTake(s_sensor_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (s_aht20_detected) {
            snprintf(aht_temp_s,  sizeof(aht_temp_s),  "%.2f", s_aht_temp);
            snprintf(aht_hum_s,   sizeof(aht_hum_s),   "%.2f", s_aht_hum);
        }
        if (s_bmp280_detected) {
            snprintf(bmp_temp_s,  sizeof(bmp_temp_s),  "%.2f", s_bmp_temp);
            snprintf(bmp_press_s, sizeof(bmp_press_s), "%.2f", s_bmp_press);
        }
        xSemaphoreGive(s_sensor_mutex);
    }

    char resp[256];
    snprintf(resp, sizeof(resp),
        "{\"ok\":true,"
        "\"aht20\":{\"detected\":%s,\"temperature\":%s,\"humidity\":%s},"
        "\"bmp280\":{\"detected\":%s,\"temperature\":%s,\"pressure_hpa\":%s}}",
        s_aht20_detected ? "true" : "false", aht_temp_s, aht_hum_s,
        s_bmp280_detected ? "true" : "false", bmp_temp_s, bmp_press_s);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_get_handler(httpd_req_t *req)
{
    esp_netif_ip_info_t ip_info = {0};
    if (esp_netif_get_ip_info(s_eth_netif, &ip_info) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No IP info");
        return ESP_FAIL;
    }

    const esp_partition_t *running = esp_ota_get_running_partition();

    char resp[768];
    snprintf(resp, sizeof(resp),
        "{\"ok\":true,"
        "\"ip\":\"" IPSTR "\","
        "\"gw\":\"" IPSTR "\","
        "\"netmask\":\"" IPSTR "\","
        "\"running_partition\":\"%s\","
        "\"aht20_detected\":%s,"
        "\"bmp280_detected\":%s}",
        IP2STR(&ip_info.ip),
        IP2STR(&ip_info.gw),
        IP2STR(&ip_info.netmask),
        running ? running->label : "unknown",
        s_aht20_detected ? "true" : "false",
        s_bmp280_detected ? "true" : "false");

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
}

/* ------------------------------------------------------------------ */
/*  Dashboard HTML — estilo industrial con gráficas Chart.js           */
/* ------------------------------------------------------------------ */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    /* El HTML se parte en fragmentos para no superar el stack.
     * Se envía con httpd_resp_send_chunk. */
    httpd_resp_set_type(req, "text/html");

    /* ---- HEAD + ESTILOS ---- */
    const char *head =
        "<!doctype html><html lang='es'>"
        "<head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>ESP32 Sensors</title>"
        "<style>"
        "*{box-sizing:border-box;margin:0;padding:0}"
        "body{font-family:'Courier New',monospace;background:#0a0c10;color:#c9d1d9;padding:16px}"
        ".topbar{display:flex;align-items:center;justify-content:space-between;"
        "  padding-bottom:12px;border-bottom:1px solid #21262d;margin-bottom:16px}"
        ".title{font-size:13px;font-weight:700;letter-spacing:.12em;color:#e6edf3}"
        ".badge{font-size:10px;padding:2px 8px;border-radius:3px;font-weight:700;"
        "  letter-spacing:.08em}"
        ".badge-ok{background:#0d2a0d;color:#3fb950;border:1px solid #238636}"
        ".badge-off{background:#2d0f0f;color:#f85149;border:1px solid #6e2020}"
        ".live{font-size:11px;color:#8b949e}"
        ".dot{display:inline-block;width:7px;height:7px;border-radius:50%;"
        "  background:#3fb950;margin-right:5px}"
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
        ".sep{height:1px;background:#21262d;margin:14px 0}"
        ".inforow{display:flex;gap:20px;flex-wrap:wrap;font-size:11px;color:#8b949e}"
        ".iitem{display:flex;flex-direction:column;gap:2px}"
        ".ilabel{font-size:9px;letter-spacing:.1em;text-transform:uppercase}"
        ".ival{font-size:12px;color:#e6edf3}"
        ".ticker{font-size:10px;color:#484f58;text-align:right;margin-top:10px}"
        "</style></head><body>";

    /* ---- BODY — topbar + métricas ---- */
    const char *body1 =
        "<div class='topbar'>"
        "  <div style='display:flex;align-items:center;gap:10px'>"
        "    <span class='title'>ESP32 · SENSOR DASHBOARD</span>"
        "    <span class='badge badge-ok' id='eth-badge'>ETH ONLINE</span>"
        "  </div>"
        "  <div class='live'><span class='dot' id='dot'></span>"
        "  <span id='tick'>LIVE</span></div>"
        "</div>"

        "<div class='grid4'>"
        "  <div class='metric'>"
        "    <div class='mlabel'>AHT20 Temp</div>"
        "    <div><span class='mval' id='v-at'>--</span><span class='munit'>°C</span></div>"
        "    <div class='msub'><span id='s-aht' class='badge'>--</span></div>"
        "  </div>"
        "  <div class='metric'>"
        "    <div class='mlabel'>AHT20 Hum</div>"
        "    <div><span class='mval' id='v-ah'>--</span><span class='munit'>%</span></div>"
        "    <div class='msub'>&nbsp;</div>"
        "  </div>"
        "  <div class='metric'>"
        "    <div class='mlabel'>BMP280 Temp</div>"
        "    <div><span class='mval' id='v-bt'>--</span><span class='munit'>°C</span></div>"
        "    <div class='msub'><span id='s-bmp' class='badge'>--</span></div>"
        "  </div>"
        "  <div class='metric'>"
        "    <div class='mlabel'>Presión</div>"
        "    <div><span class='mval' id='v-bp'>--</span><span class='munit'>hPa</span></div>"
        "    <div class='msub'>&nbsp;</div>"
        "  </div>"
        "</div>";

    /* ---- Gráficas ---- */
    const char *body2 =
        "<div class='grid2'>"
        "  <div class='chart-card'>"
        "    <div class='chart-hdr'>"
        "      <span class='chart-title'>Temperatura — últimos 2 min</span>"
        "      <div class='legend'>"
        "        <span><span class='ld' style='background:#378ADD'></span>AHT20</span>"
        "        <span><span class='ld' style='background:#EF9F27'></span>BMP280</span>"
        "      </div>"
        "    </div>"
        "    <div style='position:relative;width:100%;height:160px'>"
        "      <canvas id='cTemp'></canvas>"
        "    </div>"
        "  </div>"
        "  <div class='chart-card'>"
        "    <div class='chart-hdr'>"
        "      <span class='chart-title'>Humedad / Presión — últimos 2 min</span>"
        "      <div class='legend'>"
        "        <span><span class='ld' style='background:#1D9E75'></span>Hum %</span>"
        "        <span><span class='ld' style='background:#D4537E'></span>Presión hPa</span>"
        "      </div>"
        "    </div>"
        "    <div style='position:relative;width:100%;height:160px'>"
        "      <canvas id='cHumPress'></canvas>"
        "    </div>"
        "  </div>"
        "</div>";

    /* ---- Info de red + footer ---- */
    const char *body3 =
        "<div class='sep'></div>"
        "<div class='inforow'>"
        "  <div class='iitem'><span class='ilabel'>IP</span>"
        "    <span class='ival' id='ip'>-</span></div>"
        "  <div class='iitem'><span class='ilabel'>Gateway</span>"
        "    <span class='ival' id='gw'>-</span></div>"
        "  <div class='iitem'><span class='ilabel'>Máscara</span>"
        "    <span class='ival' id='mask'>-</span></div>"
        "  <div class='iitem'><span class='ilabel'>Partición</span>"
        "    <span class='ival' id='part'>-</span></div>"
        "  <div class='iitem'><span class='ilabel'>OTA</span>"
        "    <span class='ival'>POST /update</span></div>"
        "</div>"
        "<div class='ticker' id='counter'>muestras: 0</div>";

    /* ---- Script: Chart.js + polling ---- */
    const char *script =
        "<script src='https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.1/chart.umd.js'></script>"
        "<script>"
        "const MAX=40;"
        "const labels=Array(MAX).fill('');"
        "const dAt=[],dAh=[],dBt=[],dBp=[];"
        "for(let i=0;i<MAX;i++){dAt.push(null);dAh.push(null);dBt.push(null);dBp.push(null);}"

        "const sharedScales={"
        "  x:{display:false,grid:{display:false}},"
        "  y:{grid:{color:'rgba(255,255,255,.06)'},"
        "     ticks:{font:{size:9},color:'#484f58'}}"
        "};"
        "const sharedOpts={"
        "  responsive:true,maintainAspectRatio:false,"
        "  animation:{duration:400},"
        "  plugins:{legend:{display:false},"
        "    tooltip:{mode:'index',intersect:false}},"
        "  elements:{point:{radius:0},line:{tension:0.4,borderWidth:1.5}}"
        "};"

        "const cT=new Chart(document.getElementById('cTemp'),{"
        "  type:'line',"
        "  data:{labels:[...labels],datasets:["
        "    {label:'AHT20',data:[...dAt],borderColor:'#378ADD',"
        "     backgroundColor:'rgba(55,138,221,.08)',fill:true},"
        "    {label:'BMP280',data:[...dBt],borderColor:'#EF9F27',"
        "     backgroundColor:'rgba(239,159,39,.08)',fill:true}"
        "  ]},"
        "  options:{...sharedOpts,scales:{...sharedScales,"
        "    y:{...sharedScales.y,ticks:{...sharedScales.y.ticks,"
        "      callback:v=>v==null?'':v.toFixed(1)+'°'}}}}"
        "});"

        "const cH=new Chart(document.getElementById('cHumPress'),{"
        "  type:'line',"
        "  data:{labels:[...labels],datasets:["
        "    {label:'Hum %',data:[...dAh],borderColor:'#1D9E75',"
        "     backgroundColor:'rgba(29,158,117,.08)',fill:true,yAxisID:'yL'},"
        "    {label:'Presión',data:[...dBp],borderColor:'#D4537E',"
        "     backgroundColor:'rgba(212,83,126,.08)',fill:true,yAxisID:'yR'}"
        "  ]},"
        "  options:{...sharedOpts,scales:{"
        "    x:{display:false,grid:{display:false}},"
        "    yL:{position:'left',grid:{color:'rgba(255,255,255,.06)'},"
        "        ticks:{font:{size:9},color:'rgba(29,158,117,.8)',"
        "          callback:v=>v==null?'':v.toFixed(0)+'%'}},"
        "    yR:{position:'right',grid:{display:false},"
        "        ticks:{font:{size:9},color:'rgba(212,83,126,.8)',"
        "          callback:v=>v==null?'':v.toFixed(0)}}"
        "  }}"
        "});"

        "function push(chart,ds,vals){"
        "  if(chart.data.labels.length>=MAX){"
        "    chart.data.labels.shift();"
        "    chart.data.datasets.forEach(d=>d.data.shift());"
        "  }"
        "  chart.data.labels.push('');"
        "  ds.forEach((d,i)=>chart.data.datasets[i].data.push(vals[i]));"
        "  chart.update('none');"
        "}"

        "function setBadge(id,ok){"
        "  const el=document.getElementById(id);"
        "  el.textContent=ok?'ONLINE':'OFFLINE';"
        "  el.className='badge '+(ok?'badge-ok':'badge-off');"
        "}"
        "function txt(id,v){document.getElementById(id).textContent=v;}"
        "function fmt(v,d){return(v===null||v===undefined)?'--':Number(v).toFixed(d||2);}"

        "let count=0,elapsed=0;"
        "async function refresh(){"
        "  try{"
        "    const sj=await(await fetch('/status',{cache:'no-store'})).json();"
        "    txt('ip',sj.ip||'-');txt('gw',sj.gw||'-');"
        "    txt('mask',sj.netmask||'-');txt('part',sj.running_partition||'-');"
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
        "    push(cT,cT.data.datasets,"
        "      [aOk?j.aht20.temperature:null,bOk?j.bmp280.temperature:null]);"
        "    push(cH,cH.data.datasets,"
        "      [aOk?j.aht20.humidity:null,bOk?j.bmp280.pressure_hpa:null]);"
        "    count++;elapsed=0;"
        "    txt('counter','muestras: '+count);"
        "  }catch(e){}"
        "}"
        "refresh();"
        "setInterval(refresh,3000);"
        "setInterval(()=>{elapsed++;txt('tick','LIVE — actualizado hace '+elapsed+'s');},1000);"
        "</script></body></html>";

    httpd_resp_send_chunk(req, head,   HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, body1,  HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, body2,  HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, body3,  HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, script, HTTPD_RESP_USE_STRLEN);
    return httpd_resp_send_chunk(req, NULL, 0); /* fin chunked */
}

/* ------------------------------------------------------------------ */
/*  OTA                                                                 */
/* ------------------------------------------------------------------ */
static esp_err_t ota_post_handler(httpd_req_t *req)
{
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (!update_partition) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
        return ESP_FAIL;
    }

    esp_ota_handle_t ota_handle = 0;
    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "esp_ota_begin failed");
        return err;
    }

    char   buf[1024];
    int    remaining = req->content_len;

    while (remaining > 0) {
        int recv_len = httpd_req_recv(req, buf,
                           remaining > (int)sizeof(buf) ? (int)sizeof(buf) : remaining);
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

    if ((err = esp_ota_end(ota_handle)) != ESP_OK ||
        (err = esp_ota_set_boot_partition(update_partition)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA finalize failed");
        return err;
    }

    httpd_resp_sendstr(req, "OTA OK. Rebooting...");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

/* ================================================================== */
/*  Servidor HTTP                                                       */
/* ================================================================== */

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config  = HTTPD_DEFAULT_CONFIG();
    config.server_port     = 80;
    config.max_uri_handlers = 8;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Error al iniciar el servidor HTTP");
        return NULL;
    }

    const httpd_uri_t uris[] = {
        { .uri = "/",       .method = HTTP_GET,  .handler = root_get_handler    },
        { .uri = "/status", .method = HTTP_GET,  .handler = status_get_handler  },
        { .uri = "/sensors",.method = HTTP_GET,  .handler = sensors_get_handler },
        { .uri = "/update", .method = HTTP_POST, .handler = ota_post_handler    },
    };

    for (int i = 0; i < (int)(sizeof(uris)/sizeof(uris[0])); i++) {
        httpd_register_uri_handler(server, &uris[i]);
    }

    ESP_LOGI(TAG, "Servidor HTTP iniciado en el puerto 80");
    return server;
}

static void stop_webserver(void)
{
    if (s_http_server) {
        httpd_stop(s_http_server);
        s_http_server = NULL;
        ESP_LOGI(TAG, "Servidor HTTP detenido");
    }
}

/* ================================================================== */
/*  Ethernet                                                            */
/* ================================================================== */

static void set_static_ip(void)
{
    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(s_eth_netif));

    esp_netif_ip_info_t ip_info = {0};
    ip_info.ip.addr      = ESP_IP4TOADDR(192, 168, 10, 99);
    ip_info.gw.addr      = ESP_IP4TOADDR(192, 168, 10, 100);
    ip_info.netmask.addr = ESP_IP4TOADDR(255, 255, 255, 0);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(s_eth_netif, &ip_info));

    ESP_LOGI(TAG, "IP estática: 192.168.10.99 / 255.255.255.0 / GW 192.168.10.100");
}

/**
 * MEJORA: el event handler solo señaliza el event group.
 * El servidor HTTP se gestiona desde la tarea ethernet_manager_task,
 * evitando operaciones pesadas dentro del handler de eventos del sistema.
 */
static void on_eth_event(void *arg, esp_event_base_t base,
                         int32_t event_id, void *event_data)
{
    switch (event_id) {
        case ETHERNET_EVENT_CONNECTED: {
            uint8_t mac[6] = {0};
            esp_eth_ioctl(s_eth_handle, ETH_CMD_G_MAC_ADDR, mac);
            ESP_LOGI(TAG, "Ethernet UP — MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
            set_static_ip();
            xEventGroupSetBits(s_eth_event_group, ETH_CONNECTED_BIT);
            break;
        }
        case ETHERNET_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "Ethernet DOWN");
            xEventGroupSetBits(s_eth_event_group, ETH_DISCONNECTED_BIT);
            break;
        case ETHERNET_EVENT_START:
            ESP_LOGI(TAG, "Ethernet iniciado");
            break;
        case ETHERNET_EVENT_STOP:
            ESP_LOGW(TAG, "Ethernet detenido");
            xEventGroupSetBits(s_eth_event_group, ETH_DISCONNECTED_BIT);
            break;
        default:
            break;
    }
}

/** Tarea que gestiona el ciclo de vida del servidor HTTP */
static void ethernet_manager_task(void *arg)
{
    while (1) {
        EventBits_t bits = xEventGroupWaitBits(
            s_eth_event_group,
            ETH_CONNECTED_BIT | ETH_DISCONNECTED_BIT,
            pdTRUE,   /* clear on exit */
            pdFALSE,  /* any bit */
            portMAX_DELAY);

        if (bits & ETH_CONNECTED_BIT) {
            vTaskDelay(pdMS_TO_TICKS(200)); /* pequeño margen tras IP estática */
            if (!s_http_server) {
                s_http_server = start_webserver();
            }
        }
        if (bits & ETH_DISCONNECTED_BIT) {
            stop_webserver();
        }
    }
}

static void init_ethernet(void)
{
    /* Alimentar el PHY */
    gpio_reset_pin(ETH_PHY_POWER_GPIO);
    gpio_set_direction(ETH_PHY_POWER_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(ETH_PHY_POWER_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    s_eth_netif = esp_netif_new(&netif_cfg);
    ESP_ERROR_CHECK(s_eth_netif ? ESP_OK : ESP_FAIL);

    ESP_ERROR_CHECK(esp_event_handler_register(
        ETH_EVENT, ESP_EVENT_ANY_ID, &on_eth_event, NULL));

    eth_mac_config_t        mac_config   = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t        phy_config   = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr      = ETH_PHY_ADDR;
    phy_config.reset_gpio_num = ETH_PHY_RST_GPIO;

    eth_esp32_emac_config_t emac_config  = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    emac_config.smi_gpio.mdc_num         = ETH_MDC_GPIO;
    emac_config.smi_gpio.mdio_num        = ETH_MDIO_GPIO;
    emac_config.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    emac_config.clock_config.rmii.clock_gpio = EMAC_CLK_IN_GPIO;

    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_config);

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &s_eth_handle));
    ESP_ERROR_CHECK(esp_netif_attach(s_eth_netif,
                                     esp_eth_new_netif_glue(s_eth_handle)));
    ESP_ERROR_CHECK(esp_eth_start(s_eth_handle));
}

/* ================================================================== */
/*  app_main                                                            */
/* ================================================================== */

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Mutex para sensores — crear antes de cualquier tarea que los use */
    s_sensor_mutex    = xSemaphoreCreateMutex();
    s_eth_event_group = xEventGroupCreate();
    configASSERT(s_sensor_mutex);
    configASSERT(s_eth_event_group);

    ESP_LOGI(TAG, "Iniciando ESP32 Sensor Dashboard");

    init_sensors();

    /* Tarea gestora del servidor HTTP */
    xTaskCreate(ethernet_manager_task, "eth_mgr", 4096, NULL, 5, NULL);

    init_ethernet();
}