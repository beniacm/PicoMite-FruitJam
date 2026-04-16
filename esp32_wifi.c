/*
 * ESP32-C6 WiFi support for PicoMite on Adafruit Fruit Jam
 * Uses NINA SPI protocol to communicate with ESP32-C6 co-processor
 * running nina-fw firmware (Adafruit fork with fruitjam_c6 board).
 *
 * TCP/IP stack runs on the ESP32 - RP2350 sends commands via SPI.
 * Protocol based on Arduino WiFiNINA library.
 */

#ifdef ADAFRUIT_FRUIT_JAM

#include "MMBasic_Includes.h"
#include "Hardware_Includes.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include <string.h>
#include <stdio.h>

// ============================================================================
// Pin Definitions (Fruit Jam ESP32-C6 SPI)
// ============================================================================
#define NINA_SPI        spi1
#define NINA_SCK_PIN    30
#define NINA_MOSI_PIN   31
#define NINA_MISO_PIN   28
#define NINA_CS_PIN     46
#define NINA_ACK_PIN    3
#define NINA_RESET_PIN  22   // shared with TLV320 codec
#define NINA_SPI_FREQ   8000000

// ============================================================================
// NINA Protocol Constants
// ============================================================================
#define START_CMD       0xE0
#define END_CMD         0xEE
#define ERR_CMD         0xEF
#define REPLY_FLAG      0x80
#define DUMMY_BYTE      0xFF
#define NO_SOCKET       255

// WiFi Status Codes
#define WL_IDLE_STATUS      0
#define WL_NO_SSID_AVAIL    1
#define WL_SCAN_COMPLETED   2
#define WL_CONNECTED        3
#define WL_CONNECT_FAILED   4
#define WL_CONNECTION_LOST  5
#define WL_DISCONNECTED     6

// NINA Commands
#define CMD_SET_NET             0x10
#define CMD_SET_PASSPHRASE      0x11
#define CMD_SET_IP_CONFIG       0x14
#define CMD_SET_HOSTNAME        0x16
#define CMD_GET_CONN_STATUS     0x20
#define CMD_GET_IPADDR          0x21
#define CMD_GET_MACADDR         0x22
#define CMD_GET_CURR_SSID       0x23
#define CMD_GET_CURR_RSSI       0x25
#define CMD_SCAN_NETWORKS       0x27
#define CMD_START_SERVER_TCP    0x28
#define CMD_GET_STATE_TCP       0x29
#define CMD_AVAIL_DATA_TCP      0x2B
#define CMD_GET_DATA_TCP        0x2C
#define CMD_START_CLIENT_TCP    0x2D
#define CMD_STOP_CLIENT_TCP     0x2E
#define CMD_GET_CLIENT_STATE    0x2F
#define CMD_DISCONNECT          0x30
#define CMD_REQ_HOST_BY_NAME    0x34
#define CMD_GET_HOST_BY_NAME    0x35
#define CMD_GET_FW_VERSION      0x37
#define CMD_GET_SOCKET          0x3F
#define CMD_SEND_DATA_TCP       0x44
#define CMD_GET_DATABUF_TCP     0x45
#define CMD_SEND_DATA_UDP       0x46

// ============================================================================
// Data Types
// ============================================================================
typedef struct {
    uint8_t *data;
    uint16_t len;
} nina_param_t;

// ============================================================================
// Module State
// ============================================================================
volatile int WIFIconnected = 0;
static bool nina_initialized = false;
static int8_t tcp_socket = -1;
static uint8_t spi_buf[520]; // SPI transfer buffer

// ============================================================================
// SPI Transport Layer
// ============================================================================

static void nina_spi_init(void) {
    spi_init(NINA_SPI, NINA_SPI_FREQ);
    spi_set_format(NINA_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(NINA_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(NINA_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(NINA_MISO_PIN, GPIO_FUNC_SPI);

    gpio_init(NINA_CS_PIN);
    gpio_set_dir(NINA_CS_PIN, GPIO_OUT);
    gpio_put(NINA_CS_PIN, 1);

    gpio_init(NINA_ACK_PIN);
    gpio_set_dir(NINA_ACK_PIN, GPIO_IN);

    // Reserve pins from BASIC reconfiguration
    ExtCfg(PINMAP[NINA_SCK_PIN], EXT_BOOT_RESERVED, 0);
    ExtCfg(PINMAP[NINA_MOSI_PIN], EXT_BOOT_RESERVED, 0);
    ExtCfg(PINMAP[NINA_MISO_PIN], EXT_BOOT_RESERVED, 0);
    ExtCfg(PINMAP[NINA_CS_PIN], EXT_BOOT_RESERVED, 0);
    ExtCfg(PINMAP[NINA_ACK_PIN], EXT_BOOT_RESERVED, 0);
}

static bool nina_wait_ack(bool state, uint32_t timeout_ms) {
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    while (gpio_get(NINA_ACK_PIN) != state) {
        if (absolute_time_diff_us(get_absolute_time(), deadline) < 0)
            return false;
        busy_wait_us(10);
    }
    return true;
}

static inline void nina_cs_assert(void) {
    gpio_put(NINA_CS_PIN, 0);
}

static inline void nina_cs_deassert(void) {
    gpio_put(NINA_CS_PIN, 1);
}

static uint8_t nina_spi_transfer(uint8_t data) {
    uint8_t rx;
    spi_write_read_blocking(NINA_SPI, &data, &rx, 1);
    return rx;
}

// ============================================================================
// NINA Protocol Layer
// ============================================================================

static bool nina_send_cmd(uint8_t cmd, uint8_t num_params,
                          const nina_param_t *params) {
    // Wait for ESP32 ready
    if (!nina_wait_ack(false, 5000)) {
        return false;
    }

    nina_cs_assert();
    if (!nina_wait_ack(true, 100)) {
        nina_cs_deassert();
        return false;
    }

    int bytes_sent = 0;
    nina_spi_transfer(START_CMD); bytes_sent++;
    nina_spi_transfer(cmd & ~REPLY_FLAG); bytes_sent++;
    nina_spi_transfer(num_params); bytes_sent++;

    for (int i = 0; i < num_params; i++) {
        if (params[i].len > 255) {
            // 16-bit length for large params
            nina_spi_transfer((params[i].len >> 8) & 0xFF); bytes_sent++;
            nina_spi_transfer(params[i].len & 0xFF); bytes_sent++;
        } else {
            nina_spi_transfer(params[i].len & 0xFF); bytes_sent++;
        }
        for (int j = 0; j < params[i].len; j++) {
            nina_spi_transfer(params[i].data[j]); bytes_sent++;
        }
    }

    nina_spi_transfer(END_CMD); bytes_sent++;

    // Pad to 4-byte alignment
    while (bytes_sent % 4 != 0) {
        nina_spi_transfer(DUMMY_BYTE);
        bytes_sent++;
    }

    nina_cs_deassert();
    return true;
}

static int nina_read_response(uint8_t expected_cmd, nina_param_t *resp,
                              int max_resp) {
    // Wait for response ready
    if (!nina_wait_ack(false, 5000)) {
        return -1;
    }

    nina_cs_assert();
    if (!nina_wait_ack(true, 100)) {
        nina_cs_deassert();
        return -1;
    }

    // Read START_CMD
    uint8_t b = nina_spi_transfer(DUMMY_BYTE);
    if (b != START_CMD && b != ERR_CMD) {
        nina_cs_deassert();
        return -1;
    }

    // Read command echo
    b = nina_spi_transfer(DUMMY_BYTE);
    if (b != (expected_cmd | REPLY_FLAG)) {
        nina_cs_deassert();
        return -1;
    }

    // Read number of response parameters
    uint8_t num_params = nina_spi_transfer(DUMMY_BYTE);
    if (num_params > max_resp) num_params = max_resp;

    // Read each parameter
    for (int i = 0; i < num_params; i++) {
        uint8_t plen = nina_spi_transfer(DUMMY_BYTE);
        // Check for 16-bit length (high bit set on first byte)
        uint16_t param_len;
        if (plen & 0x80) {
            param_len = ((plen & 0x7F) << 8) | nina_spi_transfer(DUMMY_BYTE);
        } else {
            param_len = plen;
        }
        resp[i].len = param_len;
        // Read into spi_buf at offset to avoid overlap
        uint8_t *dest = resp[i].data;
        if (!dest) {
            // Skip data if no buffer provided
            for (int j = 0; j < param_len; j++)
                nina_spi_transfer(DUMMY_BYTE);
        } else {
            for (int j = 0; j < param_len; j++)
                dest[j] = nina_spi_transfer(DUMMY_BYTE);
        }
    }

    // Read END_CMD
    nina_spi_transfer(DUMMY_BYTE); // END_CMD

    nina_cs_deassert();
    return num_params;
}

static bool nina_command(uint8_t cmd, uint8_t num_params,
                         const nina_param_t *params,
                         nina_param_t *resp, int max_resp, int *num_resp) {
    if (!nina_send_cmd(cmd, num_params, params))
        return false;
    int n = nina_read_response(cmd, resp, max_resp);
    if (n < 0) return false;
    if (num_resp) *num_resp = n;
    return true;
}

// ============================================================================
// WiFi Management
// ============================================================================

void nina_wifi_init(void) {
    if (nina_initialized) return;

    nina_spi_init();

    // Reset ESP32 to clear SPI slave state (GP22 shared with codec)
    gpio_put(NINA_RESET_PIN, 0);
    sleep_ms(10);
    gpio_put(NINA_RESET_PIN, 1);
    sleep_ms(1500);

    // Re-init audio codec since we toggled shared reset
    if (Option.audio_i2s_bclk) {
        extern void tlv320_init(void);
        extern void tlv320_enable_outputs(void);
        tlv320_init();
        sleep_ms(50);
        tlv320_enable_outputs();
    }

    // Throwaway commands to sync SPI (first after reset fails, second clears state)
    uint8_t ver[16] = {0};
    nina_param_t resp[1];
    resp[0].data = ver; resp[0].len = sizeof(ver);
    int nresp = 0;
    nina_command(CMD_GET_FW_VERSION, 0, NULL, resp, 1, &nresp);
    sleep_ms(200);
    nina_command(CMD_GET_FW_VERSION, 0, NULL, resp, 1, &nresp);
    sleep_ms(100);

    nina_initialized = true;
}

void nina_get_fw_version(char *buf) {
    nina_wifi_init();
    uint8_t ver[16] = {0};
    nina_param_t resp[1];
    resp[0].data = ver;
    resp[0].len = sizeof(ver);
    int nresp;
    if (nina_command(CMD_GET_FW_VERSION, 0, NULL, resp, 1, &nresp) && nresp > 0) {
        memcpy(buf, ver, resp[0].len);
        buf[resp[0].len] = 0;
    } else {
        strcpy(buf, "unknown");
    }
}

uint8_t nina_get_conn_status(void) {
    if (!nina_initialized) return WL_DISCONNECTED;
    uint8_t status = WL_DISCONNECTED;
    nina_param_t resp[1];
    resp[0].data = &status;
    resp[0].len = 1;
    int nresp;
    nina_command(CMD_GET_CONN_STATUS, 0, NULL, resp, 1, &nresp);
    return status;
}

bool nina_wifi_connect(const char *ssid, const char *password) {
    nina_wifi_init();

    nina_param_t params[2];
    params[0].data = (uint8_t *)ssid;
    params[0].len = strlen(ssid);
    params[1].data = (uint8_t *)password;
    params[1].len = strlen(password);

    uint8_t result;
    nina_param_t resp[1];
    resp[0].data = &result;
    resp[0].len = 1;
    int nresp;

    if (!nina_command(CMD_SET_PASSPHRASE, 2, params, resp, 1, &nresp))
        return false;

    // Poll connection status
    for (int i = 0; i < 150; i++) { // 15 seconds
        CheckAbort();
        uint8_t st = nina_get_conn_status();
        if (st == WL_CONNECTED) return true;
        if (st == WL_CONNECT_FAILED || st == WL_NO_SSID_AVAIL) return false;
        sleep_ms(100);
    }
    return false;
}

void nina_wifi_disconnect(void) {
    uint8_t dummy = 0xFF;
    nina_param_t params[1];
    params[0].data = &dummy;
    params[0].len = 1;
    nina_param_t resp[1];
    uint8_t result;
    resp[0].data = &result;
    resp[0].len = 1;
    int nresp;
    nina_command(CMD_DISCONNECT, 1, params, resp, 1, &nresp);
}

void nina_get_ip_string(char *buf) {
    if (!nina_initialized) { strcpy(buf, "0.0.0.0"); return; }
    uint8_t ip[4] = {0}, mask[4] = {0}, gw[4] = {0};
    nina_param_t resp[3];
    resp[0].data = ip; resp[0].len = 4;
    resp[1].data = mask; resp[1].len = 4;
    resp[2].data = gw; resp[2].len = 4;
    uint8_t dummy = 0xFF;
    nina_param_t params[1];
    params[0].data = &dummy;
    params[0].len = 1;
    int nresp;
    if (nina_command(CMD_GET_IPADDR, 1, params, resp, 3, &nresp) && nresp >= 1) {
        sprintf(buf, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    } else {
        strcpy(buf, "0.0.0.0");
    }
}

// ============================================================================
// DNS Resolution
// ============================================================================

static bool nina_resolve_hostname(const char *host, uint32_t *ip,
                                  uint32_t timeout_ms) {
    // First check if it's already an IP
    int a, b, c, d;
    if (sscanf(host, "%d.%d.%d.%d", &a, &b, &c, &d) == 4) {
        uint8_t *p = (uint8_t *)ip;
        p[0] = a; p[1] = b; p[2] = c; p[3] = d;
        return true;
    }

    // Request DNS resolution
    nina_param_t params[1];
    params[0].data = (uint8_t *)host;
    params[0].len = strlen(host);
    uint8_t result;
    nina_param_t resp[1];
    resp[0].data = &result;
    resp[0].len = 1;
    int nresp;

    if (!nina_command(CMD_REQ_HOST_BY_NAME, 1, params, resp, 1, &nresp)) {
        return false;
    }
    if (result != 1) {
        return false;
    }

    // Poll for result
    for (uint32_t i = 0; i < timeout_ms / 50; i++) {
        CheckAbort();
        uint8_t ip_buf[4];
        nina_param_t resp2[1];
        resp2[0].data = ip_buf;
        resp2[0].len = 4;
        if (nina_command(CMD_GET_HOST_BY_NAME, 0, NULL, resp2, 1, &nresp) &&
            nresp > 0 && resp2[0].len == 4) {
            memcpy(ip, ip_buf, 4);
            if (*ip != 0) return true;
        }
        sleep_ms(50);
    }
    return false;
}

// ============================================================================
// TCP Client
// ============================================================================

static uint8_t nina_get_socket(void) {
    uint8_t sock = NO_SOCKET;
    nina_param_t resp[1];
    resp[0].data = &sock;
    resp[0].len = 1;
    int nresp;
    nina_command(CMD_GET_SOCKET, 0, NULL, resp, 1, &nresp);
    return sock;
}

bool nina_tcp_open(const char *host, uint16_t port, uint32_t timeout_ms) {
    // Resolve hostname
    uint32_t ip;
    if (!nina_resolve_hostname(host, &ip, 5000)) {
        error("DNS resolution failed");
        return false;
    }

    // Use socket 0 (skip GET_SOCKET which may not work in all fw versions)
    uint8_t sock = 0;

    // START_CLIENT_TCP: ip(4), port(2), socket(1), type(1)
    uint8_t ip_bytes[4];
    memcpy(ip_bytes, &ip, 4);
    uint8_t port_bytes[2] = {(port >> 8) & 0xFF, port & 0xFF};
    uint8_t type = 0; // TCP

    nina_param_t params[4];
    params[0].data = ip_bytes; params[0].len = 4;
    params[1].data = port_bytes; params[1].len = 2;
    params[2].data = &sock; params[2].len = 1;
    params[3].data = &type; params[3].len = 1;

    uint8_t result;
    nina_param_t resp[1];
    resp[0].data = &result;
    resp[0].len = 1;
    int nresp;

    if (!nina_command(CMD_START_CLIENT_TCP, 4, params, resp, 1, &nresp))
        return false;

    // Poll until connected
    for (uint32_t i = 0; i < timeout_ms / 50; i++) {
        CheckAbort();
        uint8_t state;
        nina_param_t sr[1];
        sr[0].data = &state;
        sr[0].len = 1;
        nina_param_t sp[1];
        sp[0].data = &sock;
        sp[0].len = 1;
        if (nina_command(CMD_GET_CLIENT_STATE, 1, sp, sr, 1, &nresp)) {
            if (state == 4) { // ESTABLISHED
                tcp_socket = sock;
                return true;
            }
        }
        sleep_ms(50);
    }
    return false;
}

uint16_t nina_tcp_available(void) {
    if (tcp_socket < 0) return 0;
    uint8_t sock = tcp_socket;
    uint8_t avail_buf[2] = {0};
    nina_param_t params[1];
    params[0].data = &sock;
    params[0].len = 1;
    nina_param_t resp[1];
    resp[0].data = avail_buf;
    resp[0].len = 2;
    int nresp;
    if (nina_command(CMD_AVAIL_DATA_TCP, 1, params, resp, 1, &nresp) && nresp > 0) {
        return (avail_buf[0] << 8) | avail_buf[1];
    }
    return 0;
}

bool nina_tcp_send(const uint8_t *data, uint16_t len) {
    if (tcp_socket < 0) return false;
    uint8_t sock = tcp_socket;

    // Send in chunks (NINA buffer ~4096 bytes)
    while (len > 0) {
        uint16_t chunk = (len > 4000) ? 4000 : len;

        nina_param_t params[2];
        params[0].data = &sock;
        params[0].len = 1;
        params[1].data = (uint8_t *)data;
        params[1].len = chunk;

        uint8_t result_buf[2] = {0};
        nina_param_t resp[1];
        resp[0].data = result_buf;
        resp[0].len = 2;
        int nresp;

        if (!nina_command(CMD_SEND_DATA_TCP, 2, params, resp, 1, &nresp))
            return false;

        data += chunk;
        len -= chunk;
    }
    return true;
}

bool nina_tcp_recv(uint8_t *buf, uint16_t bufsize, uint16_t *received) {
    if (tcp_socket < 0) return false;
    uint8_t sock = tcp_socket;
    uint8_t req_len[2] = {(bufsize >> 8) & 0xFF, bufsize & 0xFF};

    nina_param_t params[2];
    params[0].data = &sock;
    params[0].len = 1;
    params[1].data = req_len;
    params[1].len = 2;

    nina_param_t resp[1];
    resp[0].data = buf;
    resp[0].len = bufsize;
    int nresp;

    if (nina_command(CMD_GET_DATABUF_TCP, 2, params, resp, 1, &nresp) && nresp > 0) {
        *received = resp[0].len;
        return true;
    }
    *received = 0;
    return false;
}

void nina_tcp_close(void) {
    if (tcp_socket < 0) return;
    uint8_t sock = tcp_socket;

    nina_param_t params[1];
    params[0].data = &sock;
    params[0].len = 1;
    uint8_t result;
    nina_param_t resp[1];
    resp[0].data = &result;
    resp[0].len = 1;
    int nresp;
    nina_command(CMD_STOP_CLIENT_TCP, 1, params, resp, 1, &nresp);
    tcp_socket = -1;
}

// ============================================================================
// NTP
// ============================================================================

#define NTP_DELTA 2208988800ULL // seconds between 1900 and 1970

// PicoMite RTC globals (defined in mmc_stm32.c under PICOMITEWEB,
// or we provide them here for FRUITJAM builds)
int second, minute, hour, day, month, year;
extern volatile int day_of_week;
extern int64_t TimeOffsetToUptime;
extern int64_t get_epoch(int y, int m, int d, int h, int min, int s);

void nina_ntp_sync(int32_t utc_offset_seconds) {
    // Resolve NTP server
    uint32_t ntp_ip;
    if (!nina_resolve_hostname("pool.ntp.org", &ntp_ip, 5000)) {
        error("Cannot resolve NTP server");
        return;
    }

    // Use socket 1 for NTP UDP
    uint8_t sock = 1;

    // Open UDP connection to NTP server port 123
    uint8_t ip_bytes[4];
    memcpy(ip_bytes, &ntp_ip, 4);
    uint8_t port_bytes[2] = {0, 123}; // port 123
    uint8_t local_port[2] = {0xC0, 0x00}; // local port 49152
    uint8_t type = 1; // UDP

    nina_param_t params[4];
    params[0].data = ip_bytes; params[0].len = 4;
    params[1].data = port_bytes; params[1].len = 2;
    params[2].data = &sock; params[2].len = 1;
    params[3].data = &type; params[3].len = 1;

    uint8_t result;
    nina_param_t resp[1];
    resp[0].data = &result;
    resp[0].len = 1;
    int nresp;

    if (!nina_command(CMD_START_CLIENT_TCP, 4, params, resp, 1, &nresp)) {
        error("Cannot open UDP socket");
        return;
    }

    // Build NTP request
    uint8_t ntp_req[48] = {0};
    ntp_req[0] = 0x1B; // LI=0, VN=3, Mode=3 (client)

    // Send NTP request
    nina_param_t sp[2];
    sp[0].data = &sock; sp[0].len = 1;
    sp[1].data = ntp_req; sp[1].len = 48;
    if (!nina_command(CMD_SEND_DATA_TCP, 2, sp, resp, 1, &nresp)) {
        nina_tcp_close();
        error("NTP send failed");
        return;
    }

    // Wait for response
    uint8_t ntp_resp[48];
    bool got_response = false;
    for (int i = 0; i < 50; i++) { // 5 second timeout
        CheckAbort();
        // Check available data
        nina_param_t ap[1];
        ap[0].data = &sock; ap[0].len = 1;
        uint8_t avail_buf[2] = {0};
        nina_param_t ar[1];
        ar[0].data = avail_buf; ar[0].len = 2;
        if (nina_command(CMD_AVAIL_DATA_TCP, 1, ap, ar, 1, &nresp) && nresp > 0) {
            uint16_t avail = (avail_buf[0] << 8) | avail_buf[1];
            if (avail >= 48) {
                uint8_t req_len[2] = {0, 48};
                nina_param_t rp[2];
                rp[0].data = &sock; rp[0].len = 1;
                rp[1].data = req_len; rp[1].len = 2;
                nina_param_t rr[1];
                rr[0].data = ntp_resp; rr[0].len = 48;
                if (nina_command(CMD_GET_DATABUF_TCP, 2, rp, rr, 1, &nresp)) {
                    got_response = true;
                    break;
                }
            }
        }
        sleep_ms(100);
    }

    // Close UDP socket
    nina_param_t cp[1];
    cp[0].data = &sock; cp[0].len = 1;
    nina_command(CMD_STOP_CLIENT_TCP, 1, cp, resp, 1, &nresp);

    if (!got_response) {
        error("NTP timeout");
        return;
    }

    // Extract timestamp from bytes 40-43 (big-endian)
    uint32_t ntp_time = ((uint32_t)ntp_resp[40] << 24) |
                        ((uint32_t)ntp_resp[41] << 16) |
                        ((uint32_t)ntp_resp[42] << 8) |
                        (uint32_t)ntp_resp[43];

    time_t epoch = (time_t)(ntp_time - NTP_DELTA) + utc_offset_seconds;

    // Set PicoMite RTC
    struct tm *t = gmtime(&epoch);
    if (t) {
        hour = t->tm_hour;
        minute = t->tm_min;
        second = t->tm_sec;
        day_of_week = t->tm_wday;
        if (day_of_week == 0) day_of_week = 7;
        year = t->tm_year + 1900;
        month = t->tm_mon + 1;
        day = t->tm_mday;
        TimeOffsetToUptime = get_epoch(year, month, day, hour, minute, second) - time_us_64() / 1000000;
    }
}

// ============================================================================
// PicoMite BASIC Command Handler: WEB
// ============================================================================

void cmd_web(void) {
    unsigned char *tp;

    // WEB CONNECT ssid$, password$
    tp = checkstring(cmdline, (unsigned char *)"CONNECT");
    if (tp) {
        getargs(&tp, 3, (unsigned char *)",");
        if (argc != 3) error("Syntax");
        char *ssid = (char *)getCstring(argv[0]);
        char *pass = (char *)getCstring(argv[2]);
        MMPrintString("Connecting to WiFi...\r\n");
        if (!nina_wifi_connect(ssid, pass)) {
            WIFIconnected = 0;
            error("WiFi connect failed");
        }
        WIFIconnected = 1;
        char ip[20];
        nina_get_ip_string(ip);
        MMPrintString("Connected: ");
        MMPrintString(ip);
        MMPrintString("\r\n");
        return;
    }

    // WEB DISCONNECT
    tp = checkstring(cmdline, (unsigned char *)"DISCONNECT");
    if (tp) {
        nina_wifi_disconnect();
        WIFIconnected = 0;
        MMPrintString("WiFi disconnected\r\n");
        return;
    }

    // WEB NTP offset
    tp = checkstring(cmdline, (unsigned char *)"NTP");
    if (tp) {
        if (!WIFIconnected) error("WiFi not connected");
        MMFLOAT offset = getnumber(tp);
        nina_ntp_sync((int32_t)(offset * 3600));
        return;
    }

    // WEB OPEN TCP CLIENT host$, port [, timeout]
    tp = checkstring(cmdline, (unsigned char *)"OPEN TCP CLIENT");
    if (tp) {
        if (!WIFIconnected) error("WiFi not connected");
        getargs(&tp, 5, (unsigned char *)",");
        if (argc < 3) error("Syntax");
        char *host = (char *)getCstring(argv[0]);
        int port = getint(argv[2], 1, 65535);
        int timeout = (argc >= 5) ? getint(argv[4], 1, 30000) : 5000;
        if (!nina_tcp_open(host, port, timeout))
            error("TCP connect failed");
        return;
    }

    // WEB TCP CLIENT REQUEST request$, response$() [, timeout]
    tp = checkstring(cmdline, (unsigned char *)"TCP CLIENT REQUEST");
    if (tp) {
        if (!WIFIconnected) error("WiFi not connected");
        if (tcp_socket < 0) error("No TCP connection");

        getargs(&tp, 5, (unsigned char *)",");
        if (argc < 3) error("Syntax");

        char *request = (char *)getCstring(argv[0]);
        int req_len = strlen(request);

        // Get destination array
        void *ptr = findvar(argv[2], V_FIND);
        if (g_vartbl[g_VarIndex].type != T_STR) error("Expected string array");
        char *dest = (char *)ptr;
        int array_size = g_vartbl[g_VarIndex].dims[0] + 1 - g_OptionBase;

        int timeout = (argc >= 5) ? getint(argv[4], 1, 30000) : 5000;

        // Send request
        if (!nina_tcp_send((uint8_t *)request, req_len))
            error("TCP send failed");

        // Receive response into array elements
        sleep_ms(100); // give server time to respond
        int elem = 0;
        absolute_time_t deadline = make_timeout_time_ms(timeout);

        while (elem < array_size &&
               absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
            CheckAbort();
            uint16_t avail = nina_tcp_available();
            if (avail > 0) {
                uint8_t chunk[256];
                uint16_t received;
                if (nina_tcp_recv(chunk, sizeof(chunk), &received) && received > 0) {
                    // Store in string array element
                    char *s = dest + elem * (MAXSTRLEN + 1);
                    int copy_len = (received > MAXSTRLEN) ? MAXSTRLEN : received;
                    s[0] = copy_len; // MMBasic string length byte
                    memcpy(s + 1, chunk, copy_len);
                    elem++;
                    deadline = make_timeout_time_ms(500); // reset timeout for more data
                }
            } else {
                sleep_ms(10);
            }
        }
        return;
    }

    // WEB CLOSE TCP CLIENT
    tp = checkstring(cmdline, (unsigned char *)"CLOSE TCP CLIENT");
    if (tp) {
        nina_tcp_close();
        return;
    }

    error("Unknown WEB command");
}

#endif // ADAFRUIT_FRUIT_JAM
