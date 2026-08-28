#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "motor_board_protocol.h"
#include "motor_board_transport_uart.h"

static uint8_t s_rx_bytes[2048];
static size_t s_rx_read;
static size_t s_rx_written;
static char s_last_tx[64];

bool MB_Transport_IsReady(void)
{
    return true;
}

bool MB_Transport_Ensure_Rx_Active(void)
{
    return true;
}

bool MB_Transport_Send(const uint8_t *data, uint16_t length)
{
    assert(data != NULL);
    assert(length < sizeof(s_last_tx));
    memcpy(s_last_tx, data, length);
    s_last_tx[length] = '\0';
    return true;
}

bool MB_Transport_ReadByte(uint8_t *byte)
{
    if (byte == NULL || s_rx_read == s_rx_written) {
        return false;
    }
    *byte = s_rx_bytes[s_rx_read++];
    return true;
}

void MB_Transport_ClearRx(void)
{
    s_rx_read = 0U;
    s_rx_written = 0U;
}

void MB_Transport_Init(void)
{
}

void MB_Transport_GetStats(mb_transport_stats_t *stats)
{
    if (stats != NULL) {
        memset(stats, 0, sizeof(*stats));
    }
}

void MB_Transport_IRQHandler(void)
{
}

static void feed_text(const char *text)
{
    const size_t length = strlen(text);

    assert(s_rx_written + length <= sizeof(s_rx_bytes));
    memcpy(&s_rx_bytes[s_rx_written], text, length);
    s_rx_written += length;
}

static void drain_frames(unsigned int *flash_lines)
{
    mb_protocol_frame_t frame;

    while (MB_Protocol_Poll(&frame)) {
        if (frame.type == MB_FRAME_FLASH_RAW && flash_lines != NULL) {
            ++*flash_lines;
        }
    }
}

static void start_flash_read(void)
{
    MB_Transport_ClearRx();
    memset(s_last_tx, 0, sizeof(s_last_tx));
    MB_Protocol_Init();
    assert(MB_Protocol_SendReadFlash());
    assert(strcmp(s_last_tx, "$read_flash#") == 0);
}

static void test_captured_flash_snapshot(void)
{
    static const char response[] =
        "read_flash:OK!\r\n"
        "Motor_Version:1.7.3\r\n"
        "Motor_type:1\r\n"
        "Dead_Zone:1600\r\n"
        "Pulse_Line:11\r\n"
        "Pulse_Phase:30\r\n"
        "wheel_diameter:65.000\r\n"
        "P:0.800\t I:0.060\t D:0.500\r\n";
    mb_flash_config_t config = {0};
    unsigned int flash_lines = 0U;

    start_flash_read();
    feed_text(response);
    drain_frames(&flash_lines);
    assert(flash_lines == 8U);
    assert(MB_Protocol_GetFlashConfig(&config));
    assert(config.complete && !config.invalid);
    assert(config.fields_present == MB_FLASH_FIELD_REQUIRED_MASK);
    assert(config.motor_type == 1U);
    assert(config.dead_zone == 1600U);
    assert(config.pulse_line == 11U);
    assert(config.pulse_phase == 30U);
    assert(config.wheel_diameter == 65.0f);
}

static void test_incomplete_flash_snapshot_stays_locked(void)
{
    mb_flash_config_t config = {0};

    start_flash_read();
    feed_text("read_flash:OK!\nMotor_type:1\nDead_Zone:1600\n");
    drain_frames(NULL);
    assert(!MB_Protocol_GetFlashConfig(&config));
    assert(!config.complete && !config.invalid);
    assert((config.fields_present & MB_FLASH_FIELD_MOTOR_TYPE) != 0U);
    assert((config.fields_present & MB_FLASH_FIELD_DEAD_ZONE) != 0U);
}

static void test_invalid_flash_field_is_rejected(void)
{
    mb_flash_config_t config = {0};

    start_flash_read();
    feed_text("Dead_Zone:1600bad\n");
    drain_frames(NULL);
    assert(!MB_Protocol_GetFlashConfig(&config));
    assert(config.invalid);
}

static void test_deadzone_command(void)
{
    MB_Protocol_Init();
    memset(s_last_tx, 0, sizeof(s_last_tx));
    assert(MB_Protocol_SendDeadzone(1600U));
    assert(strcmp(s_last_tx, "$deadzone:1600#") == 0);
    assert(!MB_Protocol_SendDeadzone(3601U));
}

int main(void)
{
    test_captured_flash_snapshot();
    test_incomplete_flash_snapshot_stays_locked();
    test_invalid_flash_field_is_rejected();
    test_deadzone_command();
    puts("motor_board_protocol tests: OK");
    return 0;
}
