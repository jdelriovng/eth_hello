#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"

static const char *TAG = "GOPRO_BLE";

/* ============================================================
 * UUIDs GoPro
 * ============================================================ */

static const ble_uuid16_t gopro_service_uuid =
    BLE_UUID16_INIT(0xFEA6);

/* Control (WRITE) */
static const ble_uuid128_t gopro_ctrl_char_uuid =
    BLE_UUID128_INIT(
        0x1b,0xc5,0xd5,0xa5,0x02,0x00,
        0x46,0x90,0xe3,0x11,
        0x8d,0xaa,0x72,0x00,0xf9,0xb5
    );

/* Status (NOTIFY) */
static const ble_uuid128_t gopro_status_char_uuid =
    BLE_UUID128_INIT(
        0x1b,0xc5,0xd5,0xa5,0x02,0x00,
        0x46,0x90,0xe3,0x11,
        0x8d,0xaa,0x73,0x00,0xf9,0xb5
    );

/* ============================================================
 * Commands (OpenGoPro BLE)
 * ============================================================ */

static const uint8_t CMD_WIFI_ON[]    = {0x03, 0x17, 0x01, 0x01};
static const uint8_t CMD_WIFI_AP[]    = {0x03, 0x17, 0x02, 0x01};
static const uint8_t CMD_KEEPALIVE[]  = {0x03, 0x20, 0x01, 0x01};

/* ============================================================
 * Globals
 * ============================================================ */

static uint16_t conn_handle;
static uint16_t ctrl_handle;
static uint16_t status_handle;

/* ============================================================
 * State machine
 * ============================================================ */

typedef enum {
    GOPRO_IDLE,
    GOPRO_ENCRYPTED,
    GOPRO_WIFI_ON_SENT,
    GOPRO_AP_SENT,
    GOPRO_READY
} gopro_state_t;

static gopro_state_t gopro_state = GOPRO_IDLE;

/* ============================================================
 * Forward declarations
 * ============================================================ */

static void ble_start_scan(void);
static int gatt_svc_cb(uint16_t, const struct ble_gatt_error *,
                       const struct ble_gatt_svc *, void *);
static int gatt_chr_cb(uint16_t, const struct ble_gatt_error *,
                       const struct ble_gatt_chr *, void *);
static int gap_event_cb(struct ble_gap_event *, void *);

/* ============================================================
 * Helper
 * ============================================================ */

static void send_cmd(const uint8_t *cmd, size_t len)
{
    ble_gattc_write_no_rsp(
        conn_handle,
        ctrl_handle,
        ble_hs_mbuf_from_flat(cmd, len)
    );
}

/* ============================================================
 * NOTIFY handler (STATUS)
 * ============================================================ */

static void gopro_handle_status(uint8_t *data, uint16_t len)
{
    if (len < 4) return;

    /* data[0]=0x03, data[1]=cmd, data[2]=sub, data[3]=status */
    uint8_t cmd = data[1];
    uint8_t sub = data[2];
    uint8_t status = data[3];

    if (status != 0x00) {
        ESP_LOGW(TAG, "⚠️ Command error status=0x%02X", status);
        return;
    }

    if (gopro_state == GOPRO_WIFI_ON_SENT &&
        cmd == 0x17 && sub == 0x01) {

        ESP_LOGI(TAG, "✅ ACK WiFi ON → enviando modo AP");
        send_cmd(CMD_WIFI_AP, sizeof(CMD_WIFI_AP));
        gopro_state = GOPRO_AP_SENT;
        return;
    }

    if (gopro_state == GOPRO_AP_SENT &&
        cmd == 0x17 && sub == 0x02) {

        ESP_LOGI(TAG, "✅ WiFi AP listo → keep-alive");
        send_cmd(CMD_KEEPALIVE, sizeof(CMD_KEEPALIVE));
        gopro_state = GOPRO_READY;
        return;
    }
}

/* ============================================================
 * GATT discovery
 * ============================================================ */

static int gatt_chr_cb(uint16_t conn,
                       const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr,
                       void *arg)
{
    if (error->status == BLE_HS_EDONE) {
        ESP_LOGI(TAG, "✅ Characteristic discovery DONE");

        /* Enable NOTIFY (CCCD = val_handle + 1) */
        uint16_t cccd = status_handle + 1;
        uint8_t notify_on[2] = {0x01, 0x00};

        ble_gattc_write_no_rsp(
            conn_handle,
            cccd,
            ble_hs_mbuf_from_flat(notify_on, sizeof(notify_on))
        );

        ESP_LOGI(TAG, "🔔 NOTIFY habilitado");

        /* Primer comando solo tras cifrado */
        if (gopro_state == GOPRO_ENCRYPTED) {
            ESP_LOGI(TAG, "📡 Enviando WiFi ON");
            send_cmd(CMD_WIFI_ON, sizeof(CMD_WIFI_ON));
            gopro_state = GOPRO_WIFI_ON_SENT;
        }

        return 0;
    }

    if (ble_uuid_cmp(&chr->uuid.u, &gopro_ctrl_char_uuid.u) == 0) {
        ctrl_handle = chr->val_handle;
        ESP_LOGI(TAG, "CTRL handle = 0x%04X", ctrl_handle);
    }

    if (ble_uuid_cmp(&chr->uuid.u, &gopro_status_char_uuid.u) == 0) {
        status_handle = chr->val_handle;
        ESP_LOGI(TAG, "STATUS handle = 0x%04X", status_handle);
    }

    return 0;
}

static int gatt_svc_cb(uint16_t conn,
                       const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *svc,
                       void *arg)
{
    if (error->status == BLE_HS_EDONE) return 0;
    if (error->status != 0) return 0;

    if (ble_uuid_cmp(&svc->uuid.u, &gopro_service_uuid.u) == 0) {
        ESP_LOGI(TAG, "✅ GoPro service encontrado");
        ble_gattc_disc_all_chrs(
            conn,
            svc->start_handle,
            svc->end_handle,
            gatt_chr_cb,
            NULL
        );
    }
    return 0;
}

/* ============================================================
 * GAP events
 * ============================================================ */

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_DISC: {
        struct ble_hs_adv_fields fields;
        ble_hs_adv_parse_fields(&fields,
                                event->disc.data,
                                event->disc.length_data);

        if (fields.name && fields.name_len > 0) {
            char name[32] = {0};
            memcpy(name, fields.name,
                   fields.name_len < sizeof(name)-1 ?
                   fields.name_len : sizeof(name)-1);

            ESP_LOGI(TAG, "🔍 BLE: %s", name);

            if (strstr(name, "GoPro")) {
                ESP_LOGI(TAG, "🎯 GoPro encontrada, conectando...");
                ble_gap_disc_cancel();
                ble_gap_connect(
                    BLE_OWN_ADDR_PUBLIC,
                    &event->disc.addr,
                    30000,
                    NULL,
                    gap_event_cb,
                    NULL
                );
            }
        }
        return 0;
    }

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            ESP_LOGI(TAG, "✅ Connected");
            conn_handle = event->connect.conn_handle;

            /* Forzar pairing / encryption */
            ble_gap_security_initiate(conn_handle);

            ble_gattc_disc_all_svcs(
                conn_handle,
                gatt_svc_cb,
                NULL
            );
        }
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        if (event->enc_change.status == 0) {
            ESP_LOGI(TAG, "🔐 Link encrypted (bonded)");
            gopro_state = GOPRO_ENCRYPTED;
        } else {
            ESP_LOGE(TAG, "❌ Encryption failed");
        }
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX:
        gopro_handle_status(
            event->notify_rx.om->om_data,
            event->notify_rx.om->om_len
        );
        return 0;

    default:
        return 0;
    }
}

/* ============================================================
 * BLE init helpers
 * ============================================================ */

static void ble_start_scan(void)
{
    struct ble_gap_disc_params p = {0};
    p.itvl = 0x0010;
    p.window = 0x0010;
    p.filter_duplicates = 1;

    ble_gap_disc(
        BLE_OWN_ADDR_PUBLIC,
        BLE_HS_FOREVER,
        &p,
        gap_event_cb,
        NULL
    );
}

static void ble_on_sync(void)
{
    ESP_LOGI(TAG, "✅ BLE listo, iniciando escaneo");
    ble_start_scan();
}

static void ble_host_task(void *param)
{
    nimble_port_run();
}

/* ============================================================
 * app_main
 * ============================================================ */

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_nimble_hci_and_controller_init());
    nimble_port_init();

    /* Seguridad requerida por OpenGoPro */
    ble_hs_cfg.sm_io_cap   = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_bonding  = 1;
    ble_hs_cfg.sm_mitm     = 0;
    ble_hs_cfg.sm_sc       = 0; /* Legacy pairing */

    ble_hs_cfg.sync_cb = ble_on_sync;

    nimble_port_freertos_init(ble_host_task);
}