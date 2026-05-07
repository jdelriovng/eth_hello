#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define SDA_GPIO  14
#define SCL_GPIO  15
#define AHT20_ADDR 0x38

static const char *TAG = "AHT20_TEST";

// Lee el byte de estado del AHT20
static uint8_t aht20_read_status(void)
{
    uint8_t status = 0xFF;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT20_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &status, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return status;
}

// Envía comando de inicialización 0xBE 0x08 0x00
static void aht20_send_init(void)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT20_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0xBE, true);
    i2c_master_write_byte(cmd, 0x08, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
}

// Envía comando trigger de medición 0xAC 0x33 0x00
static void aht20_trigger_measure(void)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT20_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0xAC, true);
    i2c_master_write_byte(cmd, 0x33, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
}

// Lee 6 bytes de datos y calcula temperatura y humedad
static esp_err_t aht20_read_data(float *temp, float *hum)
{
    uint8_t buf[6] = {0};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT20_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buf, 5, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &buf[5], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) return ret;

    // bit7 del byte0 = busy flag
    if (buf[0] & 0x80) {
        ESP_LOGW(TAG, "  status=0x%02X → busy flag activo", buf[0]);
        return ESP_ERR_NOT_FINISHED;
    }

    uint32_t raw_hum  = ((uint32_t)buf[1] << 12) | ((uint32_t)buf[2] << 4) | (buf[3] >> 4);
    uint32_t raw_temp = (((uint32_t)buf[3] & 0x0F) << 16) | ((uint32_t)buf[4] << 8) | buf[5];

    *hum  = (float)raw_hum  / 1048576.0f * 100.0f;
    *temp = (float)raw_temp / 1048576.0f * 200.0f - 50.0f;
    return ESP_OK;
}

void app_main(void)
{
    // Init I2C
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_GPIO, .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = SCL_GPIO, .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    ESP_LOGI(TAG, "Esperando 500ms arranque sensor...");
    vTaskDelay(pdMS_TO_TICKS(500));

    // Leer estado inicial
    uint8_t status = aht20_read_status();
    ESP_LOGI(TAG, "Status inicial: 0x%02X  (bit3=calibrado: %s)",
             status, (status & 0x08) ? "SI" : "NO");

    // Si no está calibrado, enviar init
    if (!(status & 0x08)) {
        ESP_LOGI(TAG, "Enviando comando init 0xBE...");
        aht20_send_init();
        vTaskDelay(pdMS_TO_TICKS(100));
        status = aht20_read_status();
        ESP_LOGI(TAG, "Status post-init: 0x%02X  (bit3=calibrado: %s)",
                 status, (status & 0x08) ? "SI" : "NO");
    }

    // Loop de medición
    while (1) {
        ESP_LOGI(TAG, "--- Triggering medicion ---");
        aht20_trigger_measure();
        vTaskDelay(pdMS_TO_TICKS(80));  // datasheet: esperar 80ms

        float t = 0, h = 0;
        esp_err_t ret = aht20_read_data(&t, &h);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✓ Temp: %.2f°C   Hum: %.2f%%", t, h);
        } else if (ret == ESP_ERR_NOT_FINISHED) {
            ESP_LOGW(TAG, "Busy, reintentando en 100ms...");
            vTaskDelay(pdMS_TO_TICKS(100));
            ret = aht20_read_data(&t, &h);
            if (ret == ESP_OK)
                ESP_LOGI(TAG, "✓ Temp: %.2f°C   Hum: %.2f%%", t, h);
            else
                ESP_LOGE(TAG, "Fallo tras reintento");
        } else {
            ESP_LOGE(TAG, "Error I2C: %s", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}