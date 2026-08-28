#include "cm7_raw_diag.h"

#include <stddef.h>

#include "stm32h7xx.h"

#ifndef SMARTCAR_RAW_DIAGNOSTICS
#define SMARTCAR_RAW_DIAGNOSTICS 0
#endif

typedef struct
{
    const char *name;
    uint32_t count;
    uint8_t used;
} raw_diag_slot_t;

#if SMARTCAR_RAW_DIAGNOSTICS

#define CM7_RAW_DIAG_WAIT_SPINS UINT32_C(20000)
#define CM7_RAW_DIAG_SLOT_COUNT UINT32_C(16)

static raw_diag_slot_t s_slots[CM7_RAW_DIAG_SLOT_COUNT];

static uint32_t raw_string_equal(const char *left, const char *right)
{
    size_t index = 0U;

    if (left == NULL || right == NULL) {
        return 0U;
    }
    while (left[index] != '\0' && right[index] != '\0') {
        if (left[index] != right[index]) {
            return 0U;
        }
        ++index;
    }
    return left[index] == right[index] ? 1U : 0U;
}

static raw_diag_slot_t *raw_find_slot(const char *name)
{
    raw_diag_slot_t *free_slot = NULL;
    uint32_t index;

    for (index = 0U; index < CM7_RAW_DIAG_SLOT_COUNT; ++index) {
        if (s_slots[index].used != 0U &&
            raw_string_equal(s_slots[index].name, name) != 0U) {
            return &s_slots[index];
        }
        if (free_slot == NULL && s_slots[index].used == 0U) {
            free_slot = &s_slots[index];
        }
    }
    if (free_slot != NULL) {
        free_slot->name = name;
        free_slot->count = 0U;
        free_slot->used = 1U;
    }
    return free_slot;
}

static uint32_t raw_wait_flag(uint32_t flag)
{
    uint32_t spins = 0U;

    while ((USART1->ISR & flag) == 0U && spins < CM7_RAW_DIAG_WAIT_SPINS) {
        ++spins;
    }
    return (USART1->ISR & flag) != 0U ? 1U : 0U;
}

static void raw_putc(char value)
{
    if ((USART1->CR1 & (USART_CR1_UE | USART_CR1_TE)) !=
        (USART_CR1_UE | USART_CR1_TE)) {
        return;
    }
    if (raw_wait_flag(USART_ISR_TXE_TXFNF) == 0U) {
        return;
    }
    USART1->TDR = (uint8_t)value;
}

static void raw_write(const char *text)
{
    uint32_t primask;
    size_t index;

    if (text == NULL) {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    for (index = 0U; text[index] != '\0'; ++index) {
        raw_putc(text[index]);
    }
    if ((USART1->CR1 & (USART_CR1_UE | USART_CR1_TE)) ==
        (USART_CR1_UE | USART_CR1_TE)) {
        (void)raw_wait_flag(USART_ISR_TC);
    }
    if (primask == 0U) {
        __enable_irq();
    }
}

static void raw_write_u32_hex(uint32_t value)
{
    static const char digits[] = "0123456789ABCDEF";
    char buffer[8];
    uint32_t primask;
    uint32_t index;

    for (index = 0U; index < 8U; ++index) {
        buffer[7U - index] = digits[value & 0x0FU];
        value >>= 4U;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    for (index = 0U; index < 8U; ++index) {
        raw_putc(buffer[index]);
    }
    if (primask == 0U) {
        __enable_irq();
    }
}

static void raw_write_value(const char *label, uint32_t value)
{
    raw_write("[RAW] ");
    raw_write(label);
    raw_write("=0x");
    raw_write_u32_hex(value);
    raw_write("\r\n");
}

static void raw_counted(const char *prefix, const char *name, uint32_t value,
                        uint32_t include_value)
{
    raw_diag_slot_t *slot = raw_find_slot(name);

    if (slot == NULL) {
        return;
    }
    ++slot->count;
    if (slot->count > 3U && (slot->count % 1000U) != 0U) {
        return;
    }
    raw_write("[RAW] ");
    raw_write(prefix);
    raw_write("=");
    raw_write(name);
    if (include_value != 0U) {
        raw_write(" value=0x");
        raw_write_u32_hex(value);
    }
    raw_write(" count=0x");
    raw_write_u32_hex(slot->count);
    raw_write("\r\n");
}

#endif

void cm7_raw_diag_marker(const char *marker)
{
#if SMARTCAR_RAW_DIAGNOSTICS
    raw_write("[RAW] ");
    raw_write(marker);
    raw_write("\r\n");
#else
    (void)marker;
#endif
}

void cm7_raw_diag_value(const char *label, uint32_t value)
{
#if SMARTCAR_RAW_DIAGNOSTICS
    raw_write_value(label, value);
#else
    (void)label;
    (void)value;
#endif
}

void cm7_raw_diag_once(const char *marker)
{
#if SMARTCAR_RAW_DIAGNOSTICS
    raw_diag_slot_t *slot = raw_find_slot(marker);

    if (slot != NULL && slot->count == 0U) {
        slot->count = 1U;
        cm7_raw_diag_marker(marker);
    }
#else
    (void)marker;
#endif
}

void cm7_raw_diag_task_enter(const char *task_name)
{
    cm7_raw_diag_marker("TASK_ENTER");
    cm7_raw_diag_value("TASK_NAME_PTR", (uint32_t)(uintptr_t)task_name);
    cm7_raw_diag_once(task_name);
}

void cm7_raw_diag_task_loop(const char *task_name)
{
#if SMARTCAR_RAW_DIAGNOSTICS
    raw_counted("TASK_LOOP", task_name, 0U, 0U);
#else
    (void)task_name;
#endif
}

void cm7_raw_diag_tx_phase(const char *phase, uint32_t value)
{
#if SMARTCAR_RAW_DIAGNOSTICS
    raw_counted("UART2_TX", phase, value, 1U);
#else
    (void)phase;
    (void)value;
#endif
}

void cm7_raw_diag_default_handler(void)
{
    cm7_raw_diag_marker("DEFAULT_HANDLER");
    for (;;) {
        __asm volatile("wfi");
    }
}

void cm7_rtos_assert_failed(const char *file, uint32_t line)
{
#if SMARTCAR_RAW_DIAGNOSTICS
    raw_write("[RAW] [ERROR] RTOS_ASSERT file_ptr=0x");
    raw_write_u32_hex((uint32_t)(uintptr_t)file);
    raw_write(" line=0x");
    raw_write_u32_hex(line);
    raw_write("\r\n");
#else
    (void)file;
    (void)line;
#endif

    __disable_irq();
    for (;;) {
        __asm volatile("wfi");
    }
}
