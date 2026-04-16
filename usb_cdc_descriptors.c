// USB CDC + Reset descriptors for PicoMite Fruit Jam serial console
// Hardware USB (port 0) = CDC device + picotool reset interface
// PIO-USB (port 1) = HID host

#if defined(ADAFRUIT_FRUIT_JAM)

#include "tusb.h"
#include "device/usbd_pvt.h"
#include "pico/bootrom.h"
#include "hardware/watchdog.h"

#define USB_VID 0x2E8A  // Raspberry Pi
#define USB_PID 0x000A  // Pico SDK CDC
#define USB_BCD 0x0200

// Reset interface constants (must match picotool expectations)
#define RESET_INTERFACE_SUBCLASS  0x00
#define RESET_INTERFACE_PROTOCOL  0x01
#define RESET_REQUEST_BOOTSEL     0x01
#define RESET_REQUEST_FLASH       0x02

// Interface numbers
enum { ITF_NUM_CDC = 0, ITF_NUM_CDC_DATA, ITF_NUM_RESET, ITF_NUM_TOTAL };

// ============================================================================
// Device descriptor
// ============================================================================
tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = USB_BCD,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&desc_device;
}

// ============================================================================
// Configuration descriptor: CDC (2 interfaces) + Reset (1 vendor interface)
// ============================================================================
#define TUD_RPI_RESET_DESC_LEN 9
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_RPI_RESET_DESC_LEN)

#define EPNUM_CDC_NOTIF 0x81
#define EPNUM_CDC_OUT   0x02
#define EPNUM_CDC_IN    0x82

uint8_t const desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
    // Reset interface: vendor-specific, no endpoints, control-only
    9, TUSB_DESC_INTERFACE, ITF_NUM_RESET, 0, 0,
    TUSB_CLASS_VENDOR_SPECIFIC, RESET_INTERFACE_SUBCLASS, RESET_INTERFACE_PROTOCOL, 5,
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

// ============================================================================
// String descriptors
// ============================================================================
static char const *string_desc_arr[] = {
    (const char[]){0x09, 0x04},  // English
    "Raspberry Pi",
    "PicoMite Fruit Jam",
    "000000000000",
    "Serial Console",
    "Reset",
};

static uint16_t _desc_str[32 + 1];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    size_t chr_count;

    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))
            return NULL;
        const char *str = string_desc_arr[index];
        chr_count = strlen(str);
        if (chr_count > 31) chr_count = 31;
        for (size_t i = 0; i < chr_count; i++)
            _desc_str[1 + i] = str[i];
    }

    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}

// ============================================================================
// Reset interface: app driver (claims interface) + vendor callback (handles requests)
// ============================================================================

// Deferred reset - checked after tud_task() in main loop
volatile uint8_t usb_pending_reset = 0;

void usb_reset_check(void) {
    if (usb_pending_reset == 1) {
        usb_pending_reset = 0;
        reset_usb_boot(0, 0);
    } else if (usb_pending_reset == 2) {
        usb_pending_reset = 0;
        watchdog_reboot(0, 0, 100);
    }
}

// App driver: needed so TinyUSB claims the vendor interface during SET_CONFIGURATION
static void resetd_init(void) { }
static void resetd_reset(uint8_t rhport) { (void)rhport; }
static uint16_t resetd_open(uint8_t rhport, tusb_desc_interface_t const *desc_itf, uint16_t max_len) {
    (void)rhport; (void)max_len;
    TU_VERIFY(desc_itf->bInterfaceClass == TUSB_CLASS_VENDOR_SPECIFIC &&
              desc_itf->bInterfaceSubClass == RESET_INTERFACE_SUBCLASS &&
              desc_itf->bInterfaceProtocol == RESET_INTERFACE_PROTOCOL, 0);
    return sizeof(tusb_desc_interface_t);
}
static bool resetd_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                    tusb_control_request_t const *request) {
    (void)rhport; (void)stage; (void)request;
    return false; // vendor requests go to tud_vendor_control_xfer_cb, not here
}

static usbd_class_driver_t const _reset_driver = {
#if CFG_TUSB_DEBUG >= 2
    .name = "RESET",
#endif
    .init = resetd_init,
    .reset = resetd_reset,
    .open = resetd_open,
    .control_xfer_cb = resetd_control_xfer_cb,
    .xfer_cb = NULL,
    .sof = NULL,
};

usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {
    *driver_count = 1;
    return &_reset_driver;
}

// Vendor control request handler - TinyUSB routes all vendor requests here
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                tusb_control_request_t const *request) {
    if (stage != CONTROL_STAGE_SETUP) return true;
    if (request->wIndex != ITF_NUM_RESET) return false;

    if (request->bRequest == RESET_REQUEST_BOOTSEL) {
        tud_control_status(rhport, request);
        usb_pending_reset = 1;
        return true;
    }
    if (request->bRequest == RESET_REQUEST_FLASH) {
        tud_control_status(rhport, request);
        usb_pending_reset = 2;
        return true;
    }
    return false;
}

#endif // ADAFRUIT_FRUIT_JAM
