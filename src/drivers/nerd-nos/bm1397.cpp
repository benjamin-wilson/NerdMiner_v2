#include "bm1397.h"

#include "serial.h"
#include <Arduino.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../devices/device.h"
#include "crc.h"


#define BM1397_RST_PIN NERD_NOS_GPIO_RST

#define TYPE_JOB 0x20
#define TYPE_CMD 0x40

#define GROUP_SINGLE 0x00
#define GROUP_ALL 0x10

#define CMD_JOB 0x01

#define CMD_SETADDRESS 0x00
#define CMD_WRITE 0x01
#define CMD_READ 0x02
#define CMD_INACTIVE 0x03

#define RESPONSE_CMD 0x00
#define RESPONSE_JOB 0x80

#define SLEEP_TIME 20
#define FREQ_MULT 25.0

#define CLOCK_ORDER_CONTROL_0 0x80
#define CLOCK_ORDER_CONTROL_1 0x84
#define ORDERED_CLOCK_ENABLE 0x20
#define CORE_REGISTER_CONTROL 0x3C
#define PLL3_PARAMETER 0x68
#define FAST_UART_CONFIGURATION 0x28
#define TICKET_MASK 0x14
#define MISC_CONTROL 0x18

typedef struct __attribute__((__packed__))
{
    uint8_t preamble[2];
    uint32_t nonce;
    uint8_t midstate_num;
    uint8_t job_id;
    uint8_t crc;
} asic_result;


static uint8_t asic_response_buffer[CHUNK_SIZE];
static uint32_t prev_nonce = 0;
static task_result result;

static uint32_t increment_bitmask(const uint32_t value, const uint32_t mask)
{
    // if mask is zero, just return the original value
    if (mask == 0)
        return value;

    uint32_t carry = (value & mask) + (mask & -mask);      // increment the least significant bit of the mask
    uint32_t overflow = carry & ~mask;                     // find overflowed bits that are not in the mask
    uint32_t new_value = (value & ~mask) | (carry & mask); // set bits according to the mask

    // Handle carry propagation
    if (overflow > 0)
    {
        uint32_t carry_mask = (overflow << 1);                // shift left to get the mask where carry should be propagated
        new_value = increment_bitmask(new_value, carry_mask); // recursively handle carry propagation
    }

    return new_value;
}



static void _reset(void)
{
    gpio_set_level(BM1397_RST_PIN, 0);

    // delay for 100ms
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // set the gpio pin high
    gpio_set_level(BM1397_RST_PIN, 1);

    // delay for 100ms
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

/// @brief
/// @param ftdi
/// @param header
/// @param data
/// @param len
static void _send_BM1397(uint8_t header, uint8_t *data, uint8_t data_len, bool debug)
{
    packet_type_t packet_type = (header & TYPE_JOB) ? JOB_PACKET : CMD_PACKET;
    uint8_t total_length = (packet_type == JOB_PACKET) ? (data_len + 6) : (data_len + 5);

    // allocate memory for buffer
    unsigned char *buf = (unsigned char *)malloc(total_length);

    // add the preamble
    buf[0] = 0x55;
    buf[1] = 0xAA;

    // add the header field
    buf[2] = header;

    // add the length field
    buf[3] = (packet_type == JOB_PACKET) ? (data_len + 4) : (data_len + 3);

    // add the data
    memcpy(buf + 4, data, data_len);

    // add the correct crc type
    if (packet_type == JOB_PACKET)
    {
        uint16_t crc16_total = crc16_false(buf + 2, data_len + 2);
        buf[4 + data_len] = (crc16_total >> 8) & 0xFF;
        buf[5 + data_len] = crc16_total & 0xFF;
    }
    else
    {
        buf[4 + data_len] = crc5(buf + 2, data_len + 2);
    }

    // send serial data
    SERIAL_send(buf, total_length, debug);

    free(buf);
}


static void _send_chain_inactive(void)
{

    unsigned char read_address[2] = {0x00, 0x00};
    // send serial data
    _send_BM1397((TYPE_CMD | GROUP_ALL | CMD_INACTIVE), read_address, 2, false);
}

static void _set_chip_address(uint8_t chipAddr)
{

    unsigned char read_address[2] = {chipAddr, 0x00};
    // send serial data
    _send_BM1397((TYPE_CMD | GROUP_SINGLE | CMD_SETADDRESS), read_address, 2, false);
}




void BM1397_set_job_difficulty_mask(int difficulty)
{

    // Default mask of 256 diff
    unsigned char job_difficulty_mask[9] = {0x00, TICKET_MASK, 0b00000000, 0b00000000, 0b00000000, 0b11111111};

    // The mask must be a power of 2 so there are no holes
    // Correct:  {0b00000000, 0b00000000, 0b11111111, 0b11111111}
    // Incorrect: {0b00000000, 0b00000000, 0b11100111, 0b11111111}
    difficulty = _largest_power_of_two(difficulty) - 1; // (difficulty - 1) if it is a pow 2 then step down to second largest for more hashrate sampling

    // convert difficulty into char array
    // Ex: 256 = {0b00000000, 0b00000000, 0b00000000, 0b11111111}, {0x00, 0x00, 0x00, 0xff}
    // Ex: 512 = {0b00000000, 0b00000000, 0b00000001, 0b11111111}, {0x00, 0x00, 0x01, 0xff}
    for (int i = 0; i < 4; i++)
    {
        char value = (difficulty >> (8 * i)) & 0xFF;
        // The char is read in backwards to the register so we need to reverse them
        // So a mask of 512 looks like 0b00000000 00000000 00000001 1111111
        // and not 0b00000000 00000000 10000000 1111111

        job_difficulty_mask[5 - i] = _reverse_bits(value);
    }

    ESP_LOGI(TAG, "Setting job ASIC mask to %d", difficulty);

    _send_BM1397((TYPE_CMD | GROUP_ALL | CMD_WRITE), job_difficulty_mask, 6, false);
}

static void _send_init(uint64_t frequency)
{

    // send serial data
    vTaskDelay(SLEEP_TIME / portTICK_PERIOD_MS);
    _send_chain_inactive();

    _set_chip_address(0x00);

    unsigned char init[6] = {0x00, CLOCK_ORDER_CONTROL_0, 0x00, 0x00, 0x00, 0x00}; // init1 - clock_order_control0
    _send_BM1397((TYPE_CMD | GROUP_ALL | CMD_WRITE), init, 6, false);

    unsigned char init2[6] = {0x00, CLOCK_ORDER_CONTROL_1, 0x00, 0x00, 0x00, 0x00}; // init2 - clock_order_control1
    _send_BM1397((TYPE_CMD | GROUP_ALL | CMD_WRITE), init2, 6, false);

    unsigned char init3[9] = {0x00, ORDERED_CLOCK_ENABLE, 0x00, 0x00, 0x00, 0x01}; // init3 - ordered_clock_enable
    _send_BM1397((TYPE_CMD | GROUP_ALL | CMD_WRITE), init3, 6, false);

    unsigned char init4[9] = {0x00, CORE_REGISTER_CONTROL, 0x80, 0x00, 0x80, 0x74}; // init4 - init_4_?
    _send_BM1397((TYPE_CMD | GROUP_ALL | CMD_WRITE), init4, 6, false);

    BM1397_set_job_difficulty_mask(256);

    unsigned char init5[9] = {0x00, PLL3_PARAMETER, 0xC0, 0x70, 0x01, 0x11}; // init5 - pll3_parameter
    _send_BM1397((TYPE_CMD | GROUP_ALL | CMD_WRITE), init5, 6, false);

    unsigned char init6[9] = {0x00, FAST_UART_CONFIGURATION, 0x06, 0x00, 0x00, 0x0F}; // init6 - fast_uart_configuration
    _send_BM1397((TYPE_CMD | GROUP_ALL | CMD_WRITE), init6, 6, false);

    BM1397_set_default_baud();

    BM1397_send_hash_frequency(frequency);
}

static void _send_read_address(void)
{

    unsigned char read_address[2] = {0x00, 0x00};
    // send serial data
    _send_BM1397((TYPE_CMD | GROUP_ALL | CMD_READ), read_address, 2, false);
}

void BM1397_init(uint64_t frequency)
{
    Serial.println("Initializing BM1397");
    ESP_LOGI(TAG, "Initializing BM1397");

    memset(asic_response_buffer, 0, sizeof(asic_response_buffer));


    //sp_rom_gpio_pad_select_gpio(BM1397_RST_PIN);
    gpio_set_direction(BM1397_RST_PIN, GPIO_MODE_OUTPUT);

    // reset the bm1397
    _reset();

    // send the init command
    _send_read_address();

    _send_init(frequency);
}


// Baud formula = 25M/((denominator+1)*8)
// The denominator is 5 bits found in the misc_control (bits 9-13)
int BM1397_set_default_baud(void)
{
    // default divider of 26 (11010) for 115,749
    unsigned char baudrate[9] = {0x00, MISC_CONTROL, 0x00, 0x00, 0b01111010, 0b00110001}; // baudrate - misc_control
    _send_BM1397((TYPE_CMD | GROUP_ALL | CMD_WRITE), baudrate, 6, false);
    return 115749;
}

// reset the BM1397 via the RTS line


void BM1397_send_hash_frequency(float frequency)
{

    unsigned char prefreq1[9] = {0x00, 0x70, 0x0F, 0x0F, 0x0F, 0x00}; // prefreq - pll0_divider

    // default 200Mhz if it fails
    unsigned char freqbuf[9] = {0x00, 0x08, 0x40, 0xA0, 0x02, 0x25}; // freqbuf - pll0_parameter

    float deffreq = 200.0;

    float fa, fb, fc1, fc2, newf;
    float f1, basef, famax = 0xf0, famin = 0x10;
    int i;

    // bound the frequency setting
    //  You can go as low as 13 but it doesn't really scale or
    //  produce any nonces
    if (frequency < 50)
    {
        f1 = 50;
    }
    else if (frequency > 500)
    {
        f1 = 500;
    }
    else
    {
        f1 = frequency;
    }

    fb = 2;
    fc1 = 1;
    fc2 = 5; // initial multiplier of 10
    if (f1 >= 500)
    {
        // halve down to '250-400'
        fb = 1;
    }
    else if (f1 <= 150)
    {
        // triple up to '300-450'
        fc1 = 3;
    }
    else if (f1 <= 250)
    {
        // double up to '300-500'
        fc1 = 2;
    }
    // else f1 is 250-500

    // f1 * fb * fc1 * fc2 is between 2500 and 5000
    // - so round up to the next 25 (freq_mult)
    basef = FREQ_MULT * ceil(f1 * fb * fc1 * fc2 / FREQ_MULT);

    // fa should be between 100 (0x64) and 200 (0xC8)
    fa = basef / FREQ_MULT;

    // code failure ... basef isn't 400 to 6000
    if (fa < famin || fa > famax)
    {
        newf = deffreq;
    }
    else
    {
        freqbuf[3] = (int)fa;
        freqbuf[4] = (int)fb;
        // fc1, fc2 'should' already be 1..15
        freqbuf[5] = (((int)fc1 & 0xf) << 4) + ((int)fc2 & 0xf);

        newf = basef / ((float)fb * (float)fc1 * (float)fc2);
    }

    for (i = 0; i < 2; i++)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        _send_BM1397((TYPE_CMD | GROUP_ALL | CMD_WRITE), prefreq1, 6, false);
    }
    for (i = 0; i < 2; i++)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        _send_BM1397((TYPE_CMD | GROUP_ALL | CMD_WRITE), freqbuf, 6, false);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Setting Frequency to %.2fMHz (%.2f)", frequency, newf);
}

asic_result *BM1397_receive_work(void)
{

    // wait for a response, wait time is pretty arbitrary
    int received = SERIAL_rx(asic_response_buffer, 9, 60000);

    if (received < 0)
    {
        ESP_LOGI(TAG, "Error in serial RX");
        return NULL;
    }
    else if (received == 0)
    {
        // Didn't find a solution, restart and try again
        return NULL;
    }

    if (received != 9 || asic_response_buffer[0] != 0xAA || asic_response_buffer[1] != 0x55)
    {
        ESP_LOGI(TAG, "Serial RX invalid %i", received);
        //ESP_LOG_BUFFER_HEX(TAG, asic_response_buffer, received);
        return NULL;
    }

    return (asic_result *)asic_response_buffer;
}


task_result *BM1397_proccess_work()
{

    asic_result *asic_result = BM1397_receive_work();

    if (asic_result == NULL)
    {
        ESP_LOGI(TAG, "return null");
        return NULL;
    }

    uint8_t nonce_found = 0;
    uint32_t first_nonce = 0;

    uint8_t rx_job_id = asic_result->job_id & 0xfc;
    uint8_t rx_midstate_index = asic_result->job_id & 0x03;

    //GlobalState *GLOBAL_STATE = (GlobalState *)pvParameters;
    // if (GLOBAL_STATE->valid_jobs[rx_job_id] == 0)
    // {
    //     ESP_LOGI(TAG, "Invalid job nonce found, id=%d", rx_job_id);
    //     return NULL;
    // }

uint32_t rolled_version = 0;

    // uint32_t rolled_version = GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs[rx_job_id]->version;
    // for (int i = 0; i < rx_midstate_index; i++)
    // {
    //     rolled_version = increment_bitmask(rolled_version, GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs[rx_job_id]->version_mask);
    // }

    // ASIC may return the same nonce multiple times
    // or one that was already found
    // most of the time it behavies however
    if (nonce_found == 0)
    {
        first_nonce = asic_result->nonce;
        nonce_found = 1;
    }
    else if (asic_result->nonce == first_nonce)
    {
        // stop if we've already seen this nonce
        return NULL;
    }

    if (asic_result->nonce == prev_nonce)
    {
        return NULL;
    }
    else
    {
        prev_nonce = asic_result->nonce;
    }

    result.job_id = rx_job_id;
    result.nonce = asic_result->nonce;
    result.rolled_version = rolled_version;

    return &result;
}
