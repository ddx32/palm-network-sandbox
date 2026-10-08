// Palm OS Bluetooth -> PPP -> NAT -> Wi-Fi bridge.
// The Palm connects over RFCOMM (LAN Access, or DUN with AT dialing), we run a PPP server and NAT it onto Wi-Fi.
#include <ctype.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#include "esp_sdp_api.h"
#include "lwip/lwip_napt.h"
#include "lwip/stats.h"
#include "netif/ppp/pppapi.h"
#include "netif/ppp/pppos.h"
#include "secrets.h"

#define BT_NAME "PalmBridge"
#define BT_PIN "0000"
#define WIFI_COUNTRY "CZ"  // enables channels 12-13
#define UUID_LAP 0x1102  // LAN Access: Palm "Connect to: PC"
#define UUID_DUN 0x1103  // Dial-up Networking: Palm "Connect to: Modem"

static const char *TAG = "bridge";

// Bluedroid internals (not in public headers): drive the RFCOMM DCD line like a real modem.
// ponytail: called from the BTC task, not BTU; fine for one MSC frame, revisit if it misbehaves.
extern uint16_t BTA_JvRfcommGetPortHdl(uint32_t handle);
extern int PORT_Control(uint16_t handle, uint8_t signal);
#define PORT_SET_DCD 0x07
#define PORT_CLR_DCD 0x08

static void set_dcd(uint32_t spp, bool on)
{
    int r = PORT_Control(BTA_JvRfcommGetPortHdl(spp), on ? PORT_SET_DCD : PORT_CLR_DCD);
    ESP_LOGD(TAG, "DCD %s -> %d", on ? "on" : "off", r);
}

static esp_netif_t *sta;
static ppp_pcb *ppp;
static struct netif ppp_netif;
static uint32_t spp_handle;  // 0 = no Palm connected
static enum { CMD, PPP } mode;
static char line[64];
static size_t line_len;

// ---- Palm-bound flow control ----
// Bluedroid drops any write issued while RFCOMM is congested, so PPP output is queued here and
// sent one chunk at a time: next chunk on write success, the same chunk again once congestion clears.

#define TX_CHUNK 512
static RingbufHandle_t tx_rb;      // 16 KB, ~1.5 s of Palm bandwidth; overflow drops (TCP retransmits)
static SemaphoreHandle_t tx_lock;  // pump runs from both the tcpip thread and the BTC task
static uint8_t tx_buf[TX_CHUNK];
static size_t tx_len;              // bytes in tx_buf not yet confirmed sent
static bool tx_busy, tx_cong;
static uint32_t st_in, st_out, st_fail, st_drop;  // bytes from/to Palm, failed writes, queue overflows

static void tx_pump(void)
{
    bool send = false;
    xSemaphoreTake(tx_lock, portMAX_DELAY);
    if (spp_handle && !tx_busy && !tx_cong) {
        if (!tx_len) {
            size_t n;
            void *p = xRingbufferReceiveUpTo(tx_rb, &n, 0, TX_CHUNK);
            if (p) {
                memcpy(tx_buf, p, n);
                tx_len = n;
                vRingbufferReturnItem(tx_rb, p);
            }
        }
        // Mark busy before writing: the completion can arrive before esp_spp_write returns.
        send = tx_busy = tx_len > 0;
    }
    xSemaphoreGive(tx_lock);
    // Write outside the lock: esp_spp_write may block on the BTC queue, whose task takes tx_lock.
    // tx_buf is stable while tx_busy is set; esp_spp_write copies it.
    if (send && esp_spp_write(spp_handle, tx_len, tx_buf) != ESP_OK) {
        tx_busy = false;
        st_fail++;
    }
}

static void tx_reset(void)
{
    xSemaphoreTake(tx_lock, portMAX_DELAY);
    size_t n;
    void *p;
    while ((p = xRingbufferReceiveUpTo(tx_rb, &n, 0, TX_CHUNK))) vRingbufferReturnItem(tx_rb, p);
    tx_len = 0;
    tx_busy = tx_cong = false;
    xSemaphoreGive(tx_lock);
}

// ---- PPP ----

static u32_t ppp_out(ppp_pcb *pcb, const void *data, u32_t len, void *ctx)
{
    ESP_LOG_BUFFER_HEXDUMP("ppp>", data, len, ESP_LOG_DEBUG);
    if (!spp_handle) return 0;
    if (xRingbufferSend(tx_rb, data, len, 0) != pdTRUE) {
        st_drop++;
        return 0;
    }
    tx_pump();
    return len;
}

// Runs in the tcpip thread.
static void ppp_status(ppp_pcb *pcb, int err, void *ctx)
{
    if (err == PPPERR_NONE) {
        ESP_LOGI(TAG, "PPP up, Palm is " IPSTR, IP2STR(netif_ip4_gw(&ppp_netif)));
        ip_napt_enable_no(ppp_netif.num, 1);
    } else {
        ESP_LOGI(TAG, "PPP down (err %d)", err);
        mode = CMD;  // Palm may redial on the same RFCOMM link
    }
}

static void ppp_start(void)
{
    ip4_addr_t a;
    IP4_ADDR(&a, 192, 168, 44, 1);
    ppp_set_ipcp_ouraddr(ppp, &a);
    IP4_ADDR(&a, 192, 168, 44, 2);
    ppp_set_ipcp_hisaddr(ppp, &a);
    esp_netif_dns_info_t dns;
    if (esp_netif_get_dns_info(sta, ESP_NETIF_DNS_MAIN, &dns) == ESP_OK) {
        ppp_set_ipcp_dnsaddr(ppp, 0, (ip4_addr_t *)&dns.ip.u_addr.ip4);
    }
    ppp_set_silent(ppp, 1);  // Palm must see CONNECT alone; wait for its LCP before sending any PPP
    // no auth required: Palm's username/password are ignored
    if (pppapi_listen(ppp) != ERR_OK) {
        // ponytail: previous session still terminating (LCP TermReq timeouts, ~6 s); the next PPP
        // frame from the Palm retries once it's dead. Force-kill the pcb if that wait matters.
        ESP_LOGW(TAG, "PPP busy closing previous session, retrying on next frame");
        return;
    }
    mode = PPP;
    ESP_LOGI(TAG, "PPP listening");
}

// ---- Modem emulation ----

static void reply(const char *s)
{
    ESP_LOGI(TAG, "modem -> %s", s + 2);  // skip leading CRLF
    esp_spp_write(spp_handle, strlen(s), (uint8_t *)s);
}

// Palm sends setup and dial in one line, e.g. "ATQ0E0&D0&C1V1S7=90L1DT*99#". &D is not a dial.
static bool is_dial(const char *l)
{
    for (const char *p = l + 2; *p; p++) {
        if (toupper((int)*p) == 'D' && p[-1] != '&' && p[1] && strchr("TtPp0123456789*#", p[1])) return true;
    }
    return false;
}

static void spp_input(uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len && mode == CMD; i++) {
        char c = data[i];
        if (c == 0x7E) {  // PPP frame without dialing (LAN Access style): go straight to PPP
            ppp_start();
            data += i;
            len -= i;
            break;
        }
        if (c != '\r' && c != '\n') {
            if (line_len < sizeof(line) - 1) line[line_len++] = c;
            // Windows direct-cable handshake (what Linux dund --msdun answers), sent without CR
            if (line_len >= 6 && memcmp(line + line_len - 6, "CLIENT", 6) == 0) {
                ESP_LOGI(TAG, "modem <- CLIENT");
                line_len = 0;
                esp_spp_write(spp_handle, 12, (uint8_t *)"CLIENTSERVER");
                set_dcd(spp_handle, true);
                ppp_start();
                data += i + 1;
                len -= i + 1;
                break;
            }
            continue;
        }
        if (!line_len) continue;
        line[line_len] = 0;
        line_len = 0;
        ESP_LOGI(TAG, "modem <- %s", line);
        char *at = line;
        while (*at == '+' || *at == ' ') at++;  // " +++ATQ0H0": escape sequence glued to the next command
        if (at != line) memmove(line, at, strlen(at) + 1);
        if (strncasecmp(line, "AT", 2) == 0 && is_dial(line)) {
            reply("\r\nCONNECT\r\n");
            set_dcd(spp_handle, true);
            ppp_start();
            data += i + 1;
            len -= i + 1;
            break;
        }
        if (strstr(line, "H0") || strstr(line, "h0")) set_dcd(spp_handle, false);  // hang-up
        reply(strncasecmp(line, "AT", 2) == 0 ? "\r\nOK\r\n" : "\r\nERROR\r\n");
    }
    if (mode == PPP && len) {
        st_in += len;
        pppos_input_tcpip(ppp, data, len);
    }
}

// ---- Bluetooth ----

static void sdp_cb(esp_sdp_cb_event_t event, esp_sdp_cb_param_t *param) {}

static void add_record(uint8_t scn, uint16_t uuid, char *name)
{
    esp_bluetooth_sdp_record_t rec = {0};
    rec.hdr.type = ESP_SDP_TYPE_RAW;
    rec.hdr.uuid.len = ESP_UUID_LEN_16;
    rec.hdr.uuid.uuid.uuid16 = uuid;
    rec.hdr.service_name = name;
    rec.hdr.service_name_length = strlen(name) + 1;  // includes NUL, as esp_sdp requires
    rec.hdr.rfcomm_channel_number = scn;
    rec.hdr.l2cap_psm = -1;
    rec.hdr.profile_version = 0x0100;
    esp_sdp_create_record(&rec);
}

static void spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT:
        esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_SLAVE, 0, "Serial Port");
        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(TAG, "RFCOMM channel %d, BT discoverable as " BT_NAME, param->start.scn);
        add_record(param->start.scn, UUID_DUN, "Dial-up Networking");
        add_record(param->start.scn, UUID_LAP, "LAN Access using PPP");
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(TAG, "Palm connected");
        spp_handle = param->srv_open.handle;
        set_dcd(spp_handle, false);  // no carrier until the Palm dials
        mode = CMD;
        line_len = 0;
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(TAG, "Palm disconnected");
        spp_handle = 0;
        tx_reset();
        if (mode == PPP) pppapi_close(ppp, 1);
        mode = CMD;
        break;
    case ESP_SPP_WRITE_EVT:
        ESP_LOGD(TAG, "spp write status %d len %d cong %d", param->write.status, param->write.len, param->write.cong);
        xSemaphoreTake(tx_lock, portMAX_DELAY);
        if (mode != PPP || !tx_busy) {  // modem replies bypass the queue
            xSemaphoreGive(tx_lock);
            break;
        }
        tx_busy = false;
        if (param->write.status == ESP_SPP_SUCCESS) {
            st_out += tx_len;
            tx_len = 0;
        } else {
            st_fail++;  // keep tx_buf and resend
        }
        tx_cong = param->write.cong;  // current state; a failed chunk resends at once if already clear
        xSemaphoreGive(tx_lock);
        tx_pump();
        break;
    case ESP_SPP_CONG_EVT:
        ESP_LOGD(TAG, "spp cong %d", param->cong.cong);
        tx_cong = param->cong.cong;
        tx_pump();
        break;
    case ESP_SPP_DATA_IND_EVT:
        if (mode == PPP) ESP_LOG_BUFFER_HEXDUMP(TAG, param->data_ind.data, param->data_ind.len, ESP_LOG_DEBUG);
        spp_input(param->data_ind.data, param->data_ind.len);
        break;
    default:
        break;
    }
}

static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    if (event == ESP_BT_GAP_AUTH_CMPL_EVT) {
        ESP_LOGI(TAG, "pairing %s", param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS ? "ok" : "failed");
    }
}

static void bt_init(void)
{
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    esp_bluedroid_config_t bd_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    bd_cfg.ssp_en = false;  // Palm OS 5 is BT 1.1: legacy PIN pairing only
    ESP_ERROR_CHECK(esp_bluedroid_init_with_cfg(&bd_cfg));
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    esp_bt_gap_set_device_name(BT_NAME);
    esp_bt_pin_code_t pin = BT_PIN;
    esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_FIXED, strlen(BT_PIN), pin);
    // Look like a phone with networking so Palm's "phone" picker lists us.
    esp_bt_cod_t cod = {.major = ESP_BT_COD_MAJOR_DEV_PHONE, .minor = 3 /* smartphone */,
                        .service = ESP_BT_COD_SRVC_NETWORKING | ESP_BT_COD_SRVC_TELEPHONY};
    esp_bt_gap_set_cod(cod, ESP_BT_SET_COD_ALL);
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_cb));

    ESP_ERROR_CHECK(esp_sdp_register_callback(sdp_cb));
    ESP_ERROR_CHECK(esp_sdp_init());
    ESP_ERROR_CHECK(esp_spp_register_callback(spp_cb));
    esp_spp_cfg_t spp_cfg = {.mode = ESP_SPP_MODE_CB, .enable_l2cap_ertm = true};
    ESP_ERROR_CHECK(esp_spp_enhanced_init(&spp_cfg));
}

// ---- Wi-Fi ----

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi disconnected, reason %d", ((wifi_event_sta_disconnected_t *)data)->reason);
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "Wi-Fi up, " IPSTR, IP2STR(&((ip_event_got_ip_t *)data)->ip_info.ip));
    }
}

static void wifi_init(void)
{
    sta = esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL));
    wifi_config_t wc = {.sta = {.ssid = WIFI_SSID, .password = WIFI_PASS}};
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
#ifdef WIFI_MAC
    uint8_t mac[6] = WIFI_MAC;  // optional override from secrets.h
    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, mac));
#endif
    ESP_ERROR_CHECK(esp_wifi_set_country_code(WIFI_COUNTRY, true));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void stats_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (mode != PPP) continue;
        ESP_LOGI(TAG, "stats: in %lu out %lu fail %lu drop %lu queued %u busy %d cong %d", st_in, st_out, st_fail,
                 st_drop, (unsigned)(16 * 1024 - xRingbufferGetCurFreeSize(tx_rb)), tx_busy, tx_cong);
        wifi_ap_record_t ap;
        int rssi = esp_wifi_sta_get_ap_info(&ap) == ESP_OK ? ap.rssi : 0;
        ESP_LOGI(TAG, "ip: recv %u xmit %u fw %u drop %u rterr %u | udp recv %u proterr %u | icmp xmit %u | rssi %d",
                 lwip_stats.ip.recv, lwip_stats.ip.xmit, lwip_stats.ip.fw, lwip_stats.ip.drop, lwip_stats.ip.rterr,
                 lwip_stats.udp.recv, lwip_stats.udp.proterr, lwip_stats.icmp.xmit, rssi);
        ESP_LOGI(TAG, "tcp: recv %u xmit %u proterr %u drop %u", lwip_stats.tcp.recv, lwip_stats.tcp.xmit,
                 lwip_stats.tcp.proterr, lwip_stats.tcp.drop);
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    // Uncomment to hexdump PPP traffic (slows the link: serial log is 115200 baud)
    // esp_log_level_set(TAG, ESP_LOG_DEBUG);
    // esp_log_level_set("ppp>", ESP_LOG_DEBUG);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init();
    tx_rb = xRingbufferCreate(16 * 1024, RINGBUF_TYPE_BYTEBUF);
    tx_lock = xSemaphoreCreateMutex();
    xTaskCreate(stats_task, "stats", 2560, NULL, 1, NULL);
    ppp = pppapi_pppos_create(&ppp_netif, ppp_out, ppp_status, NULL);
    bt_init();
}
