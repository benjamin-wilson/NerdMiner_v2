#include <Arduino.h>
#include <drivers/devices/lilygoT_HMI.h>

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