#include <Arduino.h>
#include <drivers/devices/lilygoT_HMI.h>

#include "serial.h"

#define ECHO_TEST_TXD 17
#define ECHO_TEST_RXD 18
#define BUF_SIZE (1024)

void SERIAL_init() {
    
    Serial.begin(115200); // Initialize serial communication
    Serial.println("Initializing serial");

    // No need to configure UART parameters manually in Arduino-ESP32
    // Set UART1 pins(TX: IO17, RX: I018)
    // Serial.setTX(NERD_NOS_GPIO_TX);
    // Serial.setRx(NERD_NOS_GPIO_RX);
}

void SERIAL_set_baud(int baud)
{
    ESP_LOGI(TAG, "Changing UART baud to %i", baud);
    uart_set_baudrate(UART_NUM_1, baud);
}

int SERIAL_send(uint8_t *data, int len, bool debug)
{
    if (debug)
    {
        printf("->");
        prettyHex((unsigned char *)data, len);
        printf("\n");
    }

    return uart_write_bytes(UART_NUM_1, (const char *)data, len);
}

/// @brief waits for a serial response from the device
/// @param buf buffer to read data into
/// @param buf number of ms to wait before timing out
/// @return number of bytes read, or -1 on error
int16_t SERIAL_rx(uint8_t *buf, uint16_t size, uint16_t timeout_ms)
{
    int16_t bytes_read = uart_read_bytes(UART_NUM_1, buf, size, timeout_ms / portTICK_PERIOD_MS);
    // if (bytes_read > 0) {
    //     printf("rx: ");
    //     prettyHex((unsigned char*) buf, bytes_read);
    //     printf("\n");
    // }
    return bytes_read;
}

void SERIAL_debug_rx(void)
{
    int ret;
    uint8_t buf[CHUNK_SIZE];

    ret = SERIAL_rx(buf, 100, 20);
    if (ret < 0)
    {
        fprintf(stderr, "unable to read data\n");
        return;
    }

    memset(buf, 0, 1024);
}

void SERIAL_clear_buffer(void)
{
    uart_flush(UART_NUM_1);
}
