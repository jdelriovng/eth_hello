#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define SDA_GPIO    14
#define SCL_GPIO    15
#define I2C_FREQ    100000

static const char *TAG = "I2C_SCAN";

void app_main(void)
{
    // Inicializar bus I2C bare-metal, sin ninguna librería externa
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = SDA_GPIO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_io_num       = SCL_GPIO,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    ESP_LOGI(TAG, "Escaneando bus I2C (SDA=%d SCL=%d)...", SDA_GPIO, SCL_GPIO);

    int found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(10));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            const char *who =
                addr == 0x38 ? " ← AHT20"  :
                addr == 0x76 ? " ← BMP280 (SDO=GND)" :
                addr == 0x77 ? " ← BMP280 (SDO=VCC)" : "";
            ESP_LOGI(TAG, "  [ENCONTRADO] 0x%02X%s", addr, who);
            found++;
        }
    }

    if (found == 0) {
        ESP_LOGW(TAG, "No se encontro ningun dispositivo.");
        ESP_LOGW(TAG, "Revisa: cableado, pullups 4.7k a 3.3V, alimentacion sensores.");
    } else {
        ESP_LOGI(TAG, "Total encontrados: %d", found);
    }

    // Repetir cada 3s para poder reconectar sensores en caliente
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}