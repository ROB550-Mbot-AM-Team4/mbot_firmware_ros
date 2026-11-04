#include <stdio.h>
#include <time.h>
#include "pico/stdlib.h"
#include "comms/dual_cdc.h"
#include <uxr/client/profile/transport/custom/custom_transport.h>

int clock_gettime(clockid_t unused, struct timespec *tp)
{
    uint64_t m = time_us_64();
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}

// Open transport: Simple open, dual_cdc_init() already called in main
bool pico_serial_transport_open(struct uxrCustomTransport * transport)
{
    // printf("microROS transport open requested. CDC1 connected: %s\r\n", 
    //        dual_cdc_connected() ? "YES" : "NO");
    return true;
}

bool pico_serial_transport_close(struct uxrCustomTransport * transport)
{
    // printf("microROS transport close requested\r\n");
    return true;
}

size_t pico_serial_transport_write(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, uint8_t *errcode)
{
    dual_cdc_write_chars(1, (const char*)buf, len);
    return len;
}

size_t pico_serial_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
{
    uint64_t start_time_us = time_us_64();

    // Wait until at least one byte is available. The XRCE stream-framing layer
    // consumes partial reads (it runs a byte-wise state machine), so returning
    // fewer than `len` bytes is fine and avoids stalling on the full request.
    // Polling at 100 us keeps the reliable-publish ack wait close to the USB
    // round trip instead of quantizing it to 1 ms steps.
    int available;
    while ((available = dual_cdc_available(1)) <= 0) {
        dual_cdc_task();
        int64_t remaining_us = (int64_t)timeout * 1000 - (int64_t)(time_us_64() - start_time_us);
        if (remaining_us < 0) {
            *errcode = 1;
            return 0;
        }
        sleep_us(100);
    }

    size_t to_read = ((size_t)available < len) ? (size_t)available : len;
    int rc = dual_cdc_read_chars(1, (char*)buf, (int)to_read);
    return (rc > 0) ? (size_t)rc : 0;
}
