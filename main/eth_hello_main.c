#include <stdio.h>
#include <string.h>
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "driver/gpio.h"

#include "esp_eth.h"
#include "esp_mac.h"

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
             "\"running_partition\":\"%s\""
             "}",
             IP2STR(&ip_info.ip),
             IP2STR(&ip_info.gw),
             IP2STR(&ip_info.netmask),
             running ? running->label : "unknown");

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, "hello");
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

        httpd_register_uri_handler(server, &root);
        httpd_register_uri_handler(server, &update);
        httpd_register_uri_handler(server, &status);

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
    init_ethernet();
}