/*
 * i2cdev_bus_wrapper.c
 *
 * Implementacion minima de la API i2cdev (esp-idf-lib) sobre i2c_bus
 * (Espressif component). Permite usar bmp280.c sin tener i2cdev real,
 * evitando la colision de buses cuando AHT20 ya usa i2c_bus.
 *
 * Solo implementa las funciones que bmp280.c realmente llama:
 *   - i2cdev_init / i2cdev_done
 *   - i2c_dev_create_mutex / i2c_dev_delete_mutex / i2c_dev_take_mutex / i2c_dev_give_mutex
 *   - i2c_dev_probe
 *   - i2c_dev_read / i2c_dev_write
 *   - i2c_dev_read_reg / i2c_dev_write_reg (las mas usadas)
 *
 * Uso:
 *   1. Añadir este fichero a tu componente main.
 *   2. En CMakeLists.txt, asegurarte de que i2cdev NO está en REQUIRES
 *      (o si está, que este fichero se compila antes para que sus
 *      símbolos tengan precedencia — en la práctica, al estar en main
 *      ya tiene prioridad sobre el componente externo).
 *   3. Añadir el include path de i2cdev para que bmp280.c encuentre
 *      i2cdev.h, pero el .c de i2cdev real NO se compila.
 *
 * El handle externo s_bmp280_bus_dev debe ser asignado antes de
 * cualquier llamada a i2c_dev_read/write. Se expone como variable
 * global debil para que main.c pueda asignarlo.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "i2cdev.h"
#include "i2c_bus.h"

static const char *TAG = "i2cdev_wrap";

/* Handle de i2c_bus del BMP280 — asignado desde main antes de init */
i2c_bus_device_handle_t g_bmp280_bus_dev = NULL;

/* ------------------------------------------------------------------ */
/*  Init / done (no-ops: el bus ya lo gestiona i2c_bus)               */
/* ------------------------------------------------------------------ */
esp_err_t i2cdev_init(void)
{
    ESP_LOGI(TAG, "i2cdev_init (wrapper no-op)");
    return ESP_OK;
}

esp_err_t i2cdev_done(void)
{
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*  Mutex helpers                                                       */
/* ------------------------------------------------------------------ */
esp_err_t i2c_dev_create_mutex(i2c_dev_t *dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (!dev->mutex) dev->mutex = xSemaphoreCreateMutex();
    return dev->mutex ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t i2c_dev_delete_mutex(i2c_dev_t *dev)
{
    if (!dev || !dev->mutex) return ESP_ERR_INVALID_ARG;
    vSemaphoreDelete(dev->mutex);
    dev->mutex = NULL;
    return ESP_OK;
}

esp_err_t i2c_dev_take_mutex(i2c_dev_t *dev)
{
    if (!dev || !dev->mutex) return ESP_ERR_INVALID_ARG;
    return xSemaphoreTake(dev->mutex, pdMS_TO_TICKS(1000)) == pdTRUE
           ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t i2c_dev_give_mutex(i2c_dev_t *dev)
{
    if (!dev || !dev->mutex) return ESP_ERR_INVALID_ARG;
    xSemaphoreGive(dev->mutex);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*  Probe                                                               */
/* ------------------------------------------------------------------ */
esp_err_t i2c_dev_probe(const i2c_dev_t *dev, i2c_dev_type_t operation_type)
{
    (void)operation_type;
    if (!g_bmp280_bus_dev) return ESP_ERR_INVALID_STATE;
    /* Una lectura de 1 byte es suficiente para detectar presencia */
    uint8_t dummy = 0;
    return i2c_bus_read_bytes(g_bmp280_bus_dev, NULL_I2C_MEM_ADDR, 1, &dummy);
}

/* ------------------------------------------------------------------ */
/*  Read / Write raw                                                    */
/* ------------------------------------------------------------------ */
esp_err_t i2c_dev_read(const i2c_dev_t *dev, const void *out_data,
                        size_t out_size, void *in_data, size_t in_size)
{
    if (!g_bmp280_bus_dev) return ESP_ERR_INVALID_STATE;

    if (out_data && out_size) {
        esp_err_t err = i2c_bus_write_bytes(g_bmp280_bus_dev,
                            NULL_I2C_MEM_ADDR, out_size, (uint8_t *)out_data);
        if (err != ESP_OK) return err;
    }
    if (in_data && in_size) {
        return i2c_bus_read_bytes(g_bmp280_bus_dev,
                                  NULL_I2C_MEM_ADDR, in_size, (uint8_t *)in_data);
    }
    return ESP_OK;
}

esp_err_t i2c_dev_write(const i2c_dev_t *dev, const void *out_reg,
                         size_t out_reg_size, const void *out_data, size_t out_size)
{
    if (!g_bmp280_bus_dev) return ESP_ERR_INVALID_STATE;

    /* Combinar registro + datos en un unico buffer para i2c_bus */
    uint8_t buf[out_reg_size + out_size];
    if (out_reg && out_reg_size) memcpy(buf, out_reg, out_reg_size);
    if (out_data && out_size)    memcpy(buf + out_reg_size, out_data, out_size);

    return i2c_bus_write_bytes(g_bmp280_bus_dev,
                               NULL_I2C_MEM_ADDR,
                               out_reg_size + out_size, buf);
}

/* ------------------------------------------------------------------ */
/*  Read / Write con registro (las mas usadas por bmp280.c)            */
/* ------------------------------------------------------------------ */
esp_err_t i2c_dev_read_reg(const i2c_dev_t *dev, uint8_t reg,
                            void *in_data, size_t in_size)
{
    if (!g_bmp280_bus_dev) return ESP_ERR_INVALID_STATE;
    return i2c_bus_read_bytes(g_bmp280_bus_dev, reg, in_size, (uint8_t *)in_data);
}

esp_err_t i2c_dev_write_reg(const i2c_dev_t *dev, uint8_t reg,
                             const void *out_data, size_t out_size)
{
    if (!g_bmp280_bus_dev) return ESP_ERR_INVALID_STATE;
    return i2c_bus_write_bytes(g_bmp280_bus_dev, reg, out_size, (uint8_t *)out_data);
}