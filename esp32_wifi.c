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
#define CMD_INSERT_DATABUF      0x46
#define CMD_SEND_UDP_DATA       0x39

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

    bool data_flag = (cmd & 0x40) != 0; // DATA_FLAG commands use 16-bit lengths

    for (int i = 0; i < num_params; i++) {
        if (data_flag || params[i].len > 255) {
            // 16-bit length for DATA_FLAG commands or large params
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

static int nina_read_response_ex(uint8_t expected_cmd, nina_param_t *resp,
                                 int max_resp, bool data_flag) {
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
        uint16_t param_len;
        if (data_flag) {
            // DATA_FLAG responses always use 16-bit lengths (big-endian)
            uint8_t hi = nina_spi_transfer(DUMMY_BYTE);
            uint8_t lo = nina_spi_transfer(DUMMY_BYTE);
            param_len = (hi << 8) | lo;
        } else {
            uint8_t plen = nina_spi_transfer(DUMMY_BYTE);
            if (plen & 0x80) {
                param_len = ((plen & 0x7F) << 8) | nina_spi_transfer(DUMMY_BYTE);
            } else {
                param_len = plen;
            }
        }
        resp[i].len = param_len;
        uint8_t *dest = resp[i].data;
        if (!dest) {
            for (int j = 0; j < param_len; j++)
                nina_spi_transfer(DUMMY_BYTE);
        } else {
            for (int j = 0; j < param_len; j++)
                dest[j] = nina_spi_transfer(DUMMY_BYTE);
        }
    }

    // Read END_CMD
    nina_spi_transfer(DUMMY_BYTE);

    nina_cs_deassert();
    return num_params;
}

static int nina_read_response(uint8_t expected_cmd, nina_param_t *resp,
                              int max_resp) {
    return nina_read_response_ex(expected_cmd, resp, max_resp, false);
}

static bool nina_command(uint8_t cmd, uint8_t num_params,
                         const nina_param_t *params,
                         nina_param_t *resp, int max_resp, int *num_resp) {
    if (!nina_send_cmd(cmd, num_params, params))
        return false;
    // Only CMD_GET_DATABUF_TCP uses 16-bit response lengths (waitResponseData16)
    // CMD_SEND_DATA_TCP response uses 8-bit lengths (waitResponseData8)
    bool resp_data16 = (cmd == CMD_GET_DATABUF_TCP);
    int n = nina_read_response_ex(cmd, resp, max_resp, resp_data16);
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

    // Re-init audio codec (its init pulses GP22 which resets ESP32 again!)
    if (Option.audio_i2s_bclk) {
        extern void tlv320_init(void);
        extern void tlv320_enable_outputs(void);
        tlv320_init();
        sleep_ms(50);
        tlv320_enable_outputs();
    }

    // Wait for ESP32 to boot AGAIN after codec init reset GP22
    sleep_ms(1500);

    // Throwaway commands to sync SPI
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
        if (resp[0].len == 1)
            return avail_buf[0];
        // NINA firmware sends 16-bit available as little-endian (ESP32 native)
        return avail_buf[0] | (avail_buf[1] << 8);
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

        // CMD_SEND_DATA_TCP (0x44) has DATA_FLAG - nina_send_cmd handles 16-bit lengths
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
    // NINA firmware reads requested length as little-endian (ESP32 native)
    uint8_t req_len[2] = {bufsize & 0xFF, (bufsize >> 8) & 0xFF};

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

    uint8_t result;
    nina_param_t resp[1];
    resp[0].data = &result;
    resp[0].len = 1;
    int nresp;

    // Use socket 1 for UDP (socket 0 is for TCP)
    uint8_t sock = 1;
    uint8_t udp_mode = 1; // UDP

    // Open UDP connection to NTP server port 123
    uint8_t ip_bytes[4];
    memcpy(ip_bytes, &ntp_ip, 4);
    uint8_t port_bytes[2] = {0, 123}; // port 123 big-endian
    nina_param_t cli_p[4];
    cli_p[0].data = ip_bytes; cli_p[0].len = 4;
    cli_p[1].data = port_bytes; cli_p[1].len = 2;
    cli_p[2].data = &sock; cli_p[2].len = 1;
    cli_p[3].data = &udp_mode; cli_p[3].len = 1;
    if (!nina_command(CMD_START_CLIENT_TCP, 4, cli_p, resp, 1, &nresp)) {
        error("Cannot open UDP socket");
        return;
    }
    sleep_ms(100); // let socket setup complete

    // Build NTP request
    uint8_t ntp_req[48] = {0};
    ntp_req[0] = 0x1B; // LI=0, VN=3, Mode=3 (client)

    // Buffer NTP data (INSERT_DATABUF)
    nina_param_t sp[2];
    sp[0].data = &sock; sp[0].len = 1;
    sp[1].data = ntp_req; sp[1].len = 48;
    if (!nina_command(CMD_INSERT_DATABUF, 2, sp, resp, 1, &nresp)) {
        nina_param_t cp[1];
        cp[0].data = &sock; cp[0].len = 1;
        nina_command(CMD_STOP_CLIENT_TCP, 1, cp, resp, 1, &nresp);
        error("NTP send failed");
        return;
    }
    // Trigger actual UDP send (SEND_UDP_DATA)
    nina_param_t udp_p[1];
    udp_p[0].data = &sock; udp_p[0].len = 1;
    if (!nina_command(CMD_SEND_UDP_DATA, 1, udp_p, resp, 1, &nresp)) {
        nina_param_t cp[1];
        cp[0].data = &sock; cp[0].len = 1;
        nina_command(CMD_STOP_CLIENT_TCP, 1, cp, resp, 1, &nresp);
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
            uint16_t avail = (ar[0].len == 1) ? avail_buf[0] : (avail_buf[0] | (avail_buf[1] << 8));
            if (avail >= 48) {
                uint8_t req_len[2] = {48, 0}; // little-endian
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

    // WEB TCP CLIENT REQUEST request$, response%() [, timeout]
    // Response stored in integer array as raw bytes (matching PICOMITEWEB)
    tp = checkstring(cmdline, (unsigned char *)"TCP CLIENT REQUEST");
    if (tp) {
        if (!WIFIconnected) error("WiFi not connected");
        if (tcp_socket < 0) error("No TCP connection");

        getargs(&tp, 5, (unsigned char *)",");
        if (argc < 3) error("Syntax");

        // Get request string (MMBasic counted string: byte0=len, byte1+=data)
        char *request = (char *)getstring(argv[0]);
        int req_len = request[0]; // MMBasic string length

        // Get destination integer array
        int64_t *dest = NULL;
        int size = parseintegerarray(argv[2], &dest, 2, 1, NULL, true, NULL) * 8;

        int timeout = (argc >= 5) ? getint(argv[4], 1, 30000) : 5000;

        // Send request
        if (!nina_tcp_send((uint8_t *)&request[1], req_len))
            error("TCP send failed");

        // Receive response into integer array as raw bytes
        dest[0] = 0; // first element = total received bytes
        uint8_t *q = (uint8_t *)&dest[1];
        int total = 0;
        sleep_ms(200); // wait for response to arrive
        absolute_time_t deadline = make_timeout_time_ms(timeout);

        while (total < (size - 8) &&
               absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
            CheckAbort();
            uint16_t avail = nina_tcp_available();
            if (avail > 0) {
                uint16_t want = (size - 8) - total;
                if (want > 512) want = 512;
                uint16_t received;
                if (nina_tcp_recv(q + total, want, &received) && received > 0) {
                    total += received;
                    deadline = make_timeout_time_ms(500);
                }
            } else {
                sleep_ms(10);
            }
        }
        dest[0] = total;
        return;
    }

    // WEB CLOSE TCP CLIENT
    tp = checkstring(cmdline, (unsigned char *)"CLOSE TCP CLIENT");
    if (tp) {
        nina_tcp_close();
        return;
    }

    // WEB LOAD "http://host[:port]/path"
    // Downloads a BASIC program over HTTP and loads it as the current program
    tp = checkstring(cmdline, (unsigned char *)"LOAD");
    if (tp) {
        if (!WIFIconnected) error("WiFi not connected");

        char *url = (char *)getCstring(tp);

        // Parse URL: http://host[:port]/path
        if (strncasecmp(url, "http://", 7) != 0) error("URL must start with http://");
        char *hoststart = url + 7;
        char *slash = strchr(hoststart, '/');
        char *path = slash ? slash : "/";

        // Extract host and port
        char host[128];
        int port = 80;
        int hostlen = slash ? (slash - hoststart) : strlen(hoststart);
        if (hostlen >= sizeof(host)) error("Host too long");
        memcpy(host, hoststart, hostlen);
        host[hostlen] = 0;

        char *colon = strchr(host, ':');
        if (colon) {
            *colon = 0;
            port = atoi(colon + 1);
            if (port <= 0 || port > 65535) error("Invalid port");
        }

        // Connect
        MMPrintString("Connecting to "); MMPrintString(host);
        char tmp[16]; sprintf(tmp, ":%d...\r\n", port); MMPrintString(tmp);

        if (!nina_tcp_open(host, port, 5000))
            error("TCP connect failed");

        // Send HTTP GET request
        char req[384];
        int rlen = snprintf(req, sizeof(req),
            "GET %s HTTP/1.0\r\nHost: %s\r\nConnection: close\r\n\r\n",
            path, host);

        if (!nina_tcp_send((uint8_t *)req, rlen)) {
            nina_tcp_close();
            error("HTTP send failed");
        }

        // Receive full response
        #define WEB_LOAD_BUFSIZE (64 * 1024)
        uint8_t *buf = GetTempMemory(WEB_LOAD_BUFSIZE);
        int total = 0;
        sleep_ms(200);
        absolute_time_t deadline = make_timeout_time_ms(10000);

        while (total < WEB_LOAD_BUFSIZE - 1 &&
               absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
            CheckAbort();
            uint16_t avail = nina_tcp_available();
            if (avail > 0) {
                uint16_t want = WEB_LOAD_BUFSIZE - 1 - total;
                if (want > 512) want = 512;
                uint16_t received;
                if (nina_tcp_recv(buf + total, want, &received) && received > 0) {
                    total += received;
                    deadline = make_timeout_time_ms(2000);
                }
            } else {
                sleep_ms(10);
            }
        }
        buf[total] = 0;
        nina_tcp_close();

        if (total == 0) error("No response from server");

        // Check HTTP status
        if (strncmp((char *)buf, "HTTP/", 5) != 0) error("Invalid HTTP response");
        char *status = strchr((char *)buf, ' ');
        if (!status || atoi(status + 1) != 200) {
            char errmsg[64];
            snprintf(errmsg, sizeof(errmsg), "HTTP error: %.40s", status ? status + 1 : "unknown");
            error(errmsg);
        }

        // Find body (after \r\n\r\n)
        char *body = strstr((char *)buf, "\r\n\r\n");
        if (!body) error("Invalid HTTP response (no body)");
        body += 4;

        int bodylen = total - (body - (char *)buf);
        if (bodylen <= 0) error("Empty program");

        sprintf(tmp, "Loaded %d bytes\r\n", bodylen);
        MMPrintString(tmp);

        // Load program: tokenize and save to flash
        unsigned char *progbuf = GetTempMemory(bodylen + 16);
        memcpy(progbuf, body, bodylen);
        progbuf[bodylen] = 0;

        ClearSavedVars();
        SaveProgramToFlash(progbuf, true);
        return;
    }

    // WEB SAVE "http://host[:port]/path"
    // Uploads the current program via HTTP POST
    tp = checkstring(cmdline, (unsigned char *)"SAVE");
    if (tp) {
        if (!WIFIconnected) error("WiFi not connected");

        char *url = (char *)getCstring(tp);

        // Parse URL
        if (strncasecmp(url, "http://", 7) != 0) error("URL must start with http://");
        char *hoststart = url + 7;
        char *slash = strchr(hoststart, '/');
        char *path = slash ? slash : "/";

        char host[128];
        int port = 80;
        int hostlen = slash ? (slash - hoststart) : strlen(hoststart);
        if (hostlen >= sizeof(host)) error("Host too long");
        memcpy(host, hoststart, hostlen);
        host[hostlen] = 0;

        char *colon = strchr(host, ':');
        if (colon) {
            *colon = 0;
            port = atoi(colon + 1);
            if (port <= 0 || port > 65535) error("Invalid port");
        }

        // List program to buffer (reuse LIST output)
        // We read from flash_progmemory and detoken each line
        // Read current program from memory
        unsigned char *pm = ProgMemory;
        if (*pm == 0 || *pm == 0xFF) error("No program to save");

        // Use GetTempMemory for program text buffer
        #define WEB_SAVE_BUFSIZE (64 * 1024)
        uint8_t *progbuf = GetTempMemory(WEB_SAVE_BUFSIZE);
        int proglen = 0;

        // Detoken program line by line
        extern unsigned char *llist(unsigned char *b, unsigned char *p);
        while (*pm != 0 && *pm != 0xFF) {
            unsigned char *next = llist(progbuf + proglen, pm);
            int ll = strlen((char *)progbuf + proglen);
            // Add \r\n
            progbuf[proglen + ll] = '\r';
            progbuf[proglen + ll + 1] = '\n';
            proglen += ll + 2;
            if (proglen >= WEB_SAVE_BUFSIZE - 256) error("Program too large");
            pm = next;
        }
        progbuf[proglen] = 0;

        // Connect
        if (!nina_tcp_open(host, port, 5000))
            error("TCP connect failed");

        // Send HTTP POST
        char req[384];
        int rlen = snprintf(req, sizeof(req),
            "POST %s HTTP/1.0\r\nHost: %s\r\nContent-Type: text/plain\r\n"
            "Content-Length: %d\r\nConnection: close\r\n\r\n",
            path, host, proglen);

        if (!nina_tcp_send((uint8_t *)req, rlen) ||
            !nina_tcp_send(progbuf, proglen)) {
            nina_tcp_close();
            error("HTTP send failed");
        }

        // Read response (just check status)
        sleep_ms(500);
        uint8_t resp[256];
        uint16_t received = 0;
        nina_tcp_recv(resp, sizeof(resp) - 1, &received);
        resp[received] = 0;
        nina_tcp_close();

        if (received > 0 && strncmp((char *)resp, "HTTP/", 5) == 0) {
            char *st = strchr((char *)resp, ' ');
            int code = st ? atoi(st + 1) : 0;
            if (code >= 200 && code < 300) {
                char tmp[32]; sprintf(tmp, "Saved %d bytes\r\n", proglen);
                MMPrintString(tmp);
            } else {
                char errmsg[64];
                snprintf(errmsg, sizeof(errmsg), "HTTP error: %.40s", st ? st + 1 : "unknown");
                error(errmsg);
            }
        } else {
            char tmp[32]; sprintf(tmp, "Sent %d bytes\r\n", proglen);
            MMPrintString(tmp);
        }
        return;
    }

    error("Unknown WEB command");
}

// ============================================================================
// Telnet Console Server
// ============================================================================

static int telnet_server_socket = -1;  // listener socket
static int telnet_client_socket = -1;  // accepted client socket
static char telnet_txbuf[256];
static int telnet_txpos = 0;

// Telnet negotiation: WILL SUPPRESS-GO-AHEAD, DO SUPPRESS-GO-AHEAD, WILL ECHO
static const uint8_t telnet_init_options[] = {
    255, 251, 3,   // IAC WILL SGA
    255, 253, 3,   // IAC DO SGA
    255, 251, 1,   // IAC WILL ECHO
    0
};

bool nina_telnet_start(void) {
    if (telnet_server_socket >= 0) return true; // already running

    // Get a socket for the listener
    uint8_t result;
    nina_param_t resp[1];
    resp[0].data = &result;
    resp[0].len = 1;
    int nresp;

    nina_command(CMD_GET_SOCKET, 0, NULL, resp, 1, &nresp);
    uint8_t sock = result;
    if (sock == 255) return false;

    // Start TCP server on port 23
    uint8_t port_bytes[2] = {0, 23}; // port 23 big-endian
    uint8_t tcp_mode = 0; // TCP
    nina_param_t params[3];
    params[0].data = port_bytes; params[0].len = 2;
    params[1].data = &sock; params[1].len = 1;
    params[2].data = &tcp_mode; params[2].len = 1;
    if (!nina_command(CMD_START_SERVER_TCP, 3, params, resp, 1, &nresp))
        return false;

    telnet_server_socket = sock;
    telnet_client_socket = -1;
    telnet_txpos = 0;
    return true;
}

void nina_telnet_stop(void) {
    if (telnet_client_socket >= 0) {
        uint8_t sock = telnet_client_socket;
        uint8_t result;
        nina_param_t p[1], r[1];
        p[0].data = &sock; p[0].len = 1;
        r[0].data = &result; r[0].len = 1;
        int n;
        nina_command(CMD_STOP_CLIENT_TCP, 1, p, r, 1, &n);
        telnet_client_socket = -1;
    }
    if (telnet_server_socket >= 0) {
        uint8_t sock = telnet_server_socket;
        uint8_t result;
        nina_param_t p[1], r[1];
        p[0].data = &sock; p[0].len = 1;
        r[0].data = &result; r[0].len = 1;
        int n;
        nina_command(CMD_STOP_CLIENT_TCP, 1, p, r, 1, &n);
        telnet_server_socket = -1;
    }
}

// Called from routinechecks() - poll for telnet client data
void nina_telnet_poll(void) {
    if (telnet_server_socket < 0 || !WIFIconnected) return;

    int nresp;
    uint8_t result;
    nina_param_t resp1[1];
    resp1[0].data = &result; resp1[0].len = 1;

    // Accept new client if none connected
    if (telnet_client_socket < 0) {
        // AVAIL_DATA_TCP with 2 params = server accept query
        // Returns client socket index (0-3) or 255 (no client)
        uint8_t srv_sock = telnet_server_socket;
        uint8_t accept_flag = 0; // non-blocking
        nina_param_t ap[2], ar[1];
        ap[0].data = &srv_sock; ap[0].len = 1;
        ap[1].data = &accept_flag; ap[1].len = 1;
        uint8_t client_sock_buf[2] = {255, 0};
        ar[0].data = client_sock_buf; ar[0].len = 2;
        if (!nina_command(CMD_AVAIL_DATA_TCP, 2, ap, ar, 1, &nresp))
            return;

        uint8_t client_sock = client_sock_buf[0];
        if (client_sock == 255) return; // no client yet

        telnet_client_socket = client_sock;

        // Send telnet negotiation options
        nina_param_t tp[2];
        uint8_t cs = client_sock;
        tp[0].data = &cs; tp[0].len = 1;
        tp[1].data = (uint8_t *)telnet_init_options; tp[1].len = sizeof(telnet_init_options) - 1;
        nina_command(CMD_SEND_DATA_TCP, 2, tp, resp1, 1, &nresp);

        // Send welcome banner
        const char *welcome = "\r\nPicoMite Fruit Jam Telnet Console\r\n> ";
        tp[1].data = (uint8_t *)welcome; tp[1].len = strlen(welcome);
        nina_command(CMD_SEND_DATA_TCP, 2, tp, resp1, 1, &nresp);
        return;
    }

    // Check client still connected
    uint8_t cs = telnet_client_socket;
    uint8_t state = 0;
    nina_param_t sp[1], sr[1];
    sp[0].data = &cs; sp[0].len = 1;
    sr[0].data = &state; sr[0].len = 1;
    nina_command(CMD_GET_CLIENT_STATE, 1, sp, sr, 1, &nresp);
    if (state != 4 && state != 0) { // not ESTABLISHED
        // Client disconnected
        nina_param_t cp[1];
        cp[0].data = &cs; cp[0].len = 1;
        nina_command(CMD_STOP_CLIENT_TCP, 1, cp, resp1, 1, &nresp);
        telnet_client_socket = -1;
        return;
    }

    // Check for available data from client
    uint8_t avail_buf[2] = {0};
    nina_param_t ap[1], ar[1];
    ap[0].data = &cs; ap[0].len = 1;
    ar[0].data = avail_buf; ar[0].len = 2;
    if (!nina_command(CMD_AVAIL_DATA_TCP, 1, ap, ar, 1, &nresp) || nresp == 0)
        return;

    uint16_t avail = (ar[0].len == 1) ? avail_buf[0] : (avail_buf[0] | (avail_buf[1] << 8));
    if (avail == 0) return;

    // Read data
    uint8_t rxbuf[64];
    uint16_t want = avail > sizeof(rxbuf) ? sizeof(rxbuf) : avail;
    uint8_t req_len[2] = {want & 0xFF, (want >> 8) & 0xFF};
    nina_param_t rp[2], rr[1];
    rp[0].data = &cs; rp[0].len = 1;
    rp[1].data = req_len; rp[1].len = 2;
    rr[0].data = rxbuf; rr[0].len = sizeof(rxbuf);
    if (!nina_command(CMD_GET_DATABUF_TCP, 2, rp, rr, 1, &nresp))
        return;

    uint16_t received = rr[0].len;
    if (received == 0) return;

    // Feed bytes into ConsoleRxBuf
    extern volatile char ConsoleRxBuf[];
    extern volatile int ConsoleRxBufHead, ConsoleRxBufTail;
    extern volatile int MMAbort;
    static int lastchar = -1;

    for (int j = 0; j < received; j++) {
        uint8_t c = rxbuf[j];

        // Skip telnet IAC sequences (0xFF + command + option)
        if (c == 255) { j += 2; continue; }

        // Filter CR-NULL (telnet sends CR followed by NULL)
        if (lastchar == 13 && c == 0) { lastchar = -1; continue; }

        if (BreakKey && c == BreakKey) {
            MMAbort = true;
            ConsoleRxBufHead = ConsoleRxBufTail;
        } else {
            ConsoleRxBuf[ConsoleRxBufHead] = c;
            lastchar = c;
            ConsoleRxBufHead = (ConsoleRxBufHead + 1) % CONSOLE_RX_BUF_SIZE;
            if (ConsoleRxBufHead == ConsoleRxBufTail)
                ConsoleRxBufTail = (ConsoleRxBufTail + 1) % CONSOLE_RX_BUF_SIZE;
        }
    }
}

// Called from putConsole() - send output to telnet client
void nina_telnet_putc(int c, int flush) {
    if (telnet_client_socket < 0) return;

    if (flush != -1) {
        telnet_txbuf[telnet_txpos++] = c;
        // Telnet: escape IAC (255) by doubling it
        if (c == 255) telnet_txbuf[telnet_txpos++] = c;
        // Telnet: CR must be followed by NUL
        if (c == 13) telnet_txbuf[telnet_txpos++] = 0;
    }

    if (telnet_txpos >= sizeof(telnet_txbuf) - 4 || (flush == -1 && telnet_txpos) || (flush && telnet_txpos)) {
        uint8_t sock = telnet_client_socket;
        nina_param_t sp[2];
        sp[0].data = &sock; sp[0].len = 1;
        sp[1].data = (uint8_t *)telnet_txbuf; sp[1].len = telnet_txpos;
        uint8_t result;
        nina_param_t sr[1];
        sr[0].data = &result; sr[0].len = 1;
        int nresp;
        nina_command(CMD_SEND_DATA_TCP, 2, sp, sr, 1, &nresp);
        telnet_txpos = 0;
    }
}

// Auto-connect WiFi and start telnet on boot (called from main init)
void nina_wifi_autoconnect(void) {
    extern struct option_s Option;

    // Check if WiFi credentials are saved
    if (Option.SSID[0] == 0) return;

    MMPrintString("WiFi: connecting to ");
    MMPrintString((char *)Option.SSID);
    MMPrintString("...\r\n");

    nina_wifi_init();

    if (nina_wifi_connect((char *)Option.SSID, (char *)Option.PASSWORD)) {
        WIFIconnected = 1;
        char ip[20];
        nina_get_ip_string(ip);
        MMPrintString("WiFi: connected, IP ");
        MMPrintString(ip);
        MMPrintString("\r\n");

        // Start telnet server if enabled
        if (Option.Telnet) {
            if (nina_telnet_start()) {
                MMPrintString("Telnet: listening on port 23\r\n");
            }
        }
    } else {
        MMPrintString("WiFi: connection failed\r\n");
    }
}

#endif // ADAFRUIT_FRUIT_JAM
