/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
#include "rtos_health.h"
#include "uart_link.h"
#include "motor_board_transport_uart.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FAULT_RECORD_MAGIC                 UINT32_C(0x46524C54)
#define FAULT_RECORD_VERSION               UINT32_C(1)
#define FAULT_RECORD_CHECKSUM_SEED         UINT32_C(0xA57C4E31)
#define FAULT_FRAME_CORE_WORDS             UINT32_C(8)
#define FAULT_FRAME_FLOAT_WORDS            UINT32_C(18)
#define FAULT_EXC_RETURN_FLOAT_FRAME_MASK  UINT32_C(0x10)
#define FAULT_STACK_RAM_START               UINT32_C(0x20000000)
#define FAULT_STACK_RAM_END                 UINT32_C(0x20020000)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
volatile fault_record_t g_fault_record
    __attribute__((section(".noinit"), used, aligned(8)));
static volatile uint32_t s_fault_capture_active;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void fault_capture(const uint32_t *stacked, uint32_t exc_return,
                          uint32_t exception, uint32_t fault_msp)
    __attribute__((noreturn, noinline, used));
static void fault_halt(void)
    __attribute__((noreturn, noinline, naked, used));
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t fault_record_checksum(const volatile fault_record_t *record)
{
  const volatile uint32_t *word = (const volatile uint32_t *)record;
  uint32_t checksum = FAULT_RECORD_CHECKSUM_SEED;
  uint32_t index;

  for (index = 0U;
       index < ((sizeof(fault_record_t) / sizeof(uint32_t)) - 2U);
       ++index) {
    checksum = (checksum << 5U) | (checksum >> 27U);
    checksum ^= word[index + 1U];
  }

  return checksum;
}

static uint32_t fault_read_psp(void)
{
  uint32_t value;

  __asm volatile("mrs %0, psp" : "=r"(value));
  return value;
}

static uint32_t fault_read_control(void)
{
  uint32_t value;

  __asm volatile("mrs %0, control" : "=r"(value));
  return value;
}

static uint32_t fault_frame_is_valid(const uint32_t *stacked,
                                     uint32_t exc_return)
{
  const uint32_t stacked_sp = (uint32_t)stacked;
  const uint32_t frame_words = FAULT_FRAME_CORE_WORDS +
      (((exc_return & FAULT_EXC_RETURN_FLOAT_FRAME_MASK) == 0U) ?
       FAULT_FRAME_FLOAT_WORDS : 0U);
  const uint32_t frame_bytes = frame_words * sizeof(uint32_t);

  if ((stacked_sp & (sizeof(uint32_t) - 1U)) != 0U) {
    return 0U;
  }
  if ((stacked_sp < FAULT_STACK_RAM_START) ||
      (stacked_sp > (FAULT_STACK_RAM_END - frame_bytes))) {
    return 0U;
  }

  return 1U;
}

static void fault_halt(void)
{
  __asm volatile(
      "cpsid i\n"
      "dsb sy\n"
      "1:\n"
      "wfi\n"
      "b 1b\n");
}

static void fault_capture(const uint32_t *stacked, uint32_t exc_return,
                          uint32_t exception, uint32_t fault_msp)
{
  const uint32_t frame_valid = fault_frame_is_valid(stacked, exc_return);
  const uint32_t frame_offset =
      ((exc_return & FAULT_EXC_RETURN_FLOAT_FRAME_MASK) == 0U) ?
      FAULT_FRAME_FLOAT_WORDS : 0U;
  const uint32_t *frame = stacked;

  __disable_irq();
  if (s_fault_capture_active != 0U) {
    fault_halt();
  }
  s_fault_capture_active = 1U;

  /* Commit magic last so a nested fault can never expose a partial record. */
  g_fault_record.magic = 0U;
  __DMB();
  g_fault_record.version = FAULT_RECORD_VERSION;
  g_fault_record.size = sizeof(g_fault_record);
  g_fault_record.exception = exception;
  g_fault_record.cfsr = SCB->CFSR;
  g_fault_record.hfsr = SCB->HFSR;
  g_fault_record.dfsr = SCB->DFSR;
  g_fault_record.mmfar = SCB->MMFAR;
  g_fault_record.bfar = SCB->BFAR;
  g_fault_record.shcsr = SCB->SHCSR;
  g_fault_record.msp = fault_msp;
  g_fault_record.psp = fault_read_psp();
  g_fault_record.control = fault_read_control();
  g_fault_record.exc_return = exc_return;
  g_fault_record.stacked_sp = (uint32_t)stacked;
  g_fault_record.frame_valid = frame_valid;

  if (frame_valid != 0U) {
    frame += frame_offset;
    g_fault_record.r0 = frame[0];
    g_fault_record.r1 = frame[1];
    g_fault_record.r2 = frame[2];
    g_fault_record.r3 = frame[3];
    g_fault_record.r12 = frame[4];
    g_fault_record.lr = frame[5];
    g_fault_record.pc = frame[6];
    g_fault_record.xpsr = frame[7];
  } else {
    g_fault_record.r0 = 0U;
    g_fault_record.r1 = 0U;
    g_fault_record.r2 = 0U;
    g_fault_record.r3 = 0U;
    g_fault_record.r12 = 0U;
    g_fault_record.lr = 0U;
    g_fault_record.pc = 0U;
    g_fault_record.xpsr = 0U;
  }

  g_fault_record.checksum = fault_record_checksum(&g_fault_record);
  __DMB();
  g_fault_record.magic = FAULT_RECORD_MAGIC;
  __DSB();
  fault_halt();
}

const volatile fault_record_t *fault_record_get(void)
{
  return &g_fault_record;
}

bool fault_record_is_valid(void)
{
  if (g_fault_record.magic != FAULT_RECORD_MAGIC) {
    return false;
  }
  if ((g_fault_record.version != FAULT_RECORD_VERSION) ||
      (g_fault_record.size != sizeof(g_fault_record))) {
    return false;
  }

  return g_fault_record.checksum == fault_record_checksum(&g_fault_record);
}

void fault_record_clear(void)
{
  volatile uint32_t *word = (volatile uint32_t *)&g_fault_record;
  uint32_t index;

  g_fault_record.magic = 0U;
  __DMB();
  for (index = 1U; index < (sizeof(g_fault_record) / sizeof(uint32_t)); ++index) {
    word[index] = 0U;
  }
  __DMB();
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
__attribute__((naked)) void HardFault_Handler(void)
{
  __asm volatile(
      "tst lr, #4\n"
      "ite eq\n"
      "mrseq r0, msp\n"
      "mrsne r0, psp\n"
      "mov r1, lr\n"
      "movs r2, #1\n"
      "mrs r3, msp\n"
      "b fault_capture\n");
}

/**
  * @brief This function handles Memory management fault.
  */
__attribute__((naked)) void MemManage_Handler(void)
{
  __asm volatile(
      "tst lr, #4\n"
      "ite eq\n"
      "mrseq r0, msp\n"
      "mrsne r0, psp\n"
      "mov r1, lr\n"
      "movs r2, #2\n"
      "mrs r3, msp\n"
      "b fault_capture\n");
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
__attribute__((naked)) void BusFault_Handler(void)
{
  __asm volatile(
      "tst lr, #4\n"
      "ite eq\n"
      "mrseq r0, msp\n"
      "mrsne r0, psp\n"
      "mov r1, lr\n"
      "movs r2, #3\n"
      "mrs r3, msp\n"
      "b fault_capture\n");
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
__attribute__((naked)) void UsageFault_Handler(void)
{
  __asm volatile(
      "tst lr, #4\n"
      "ite eq\n"
      "mrseq r0, msp\n"
      "mrsne r0, psp\n"
      "mov r1, lr\n"
      "movs r2, #4\n"
      "mrs r3, msp\n"
      "b fault_capture\n");
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
extern void vPortSVCHandler(void);

__attribute__((naked)) void SVC_Handler(void)
{
  __asm volatile("b vPortSVCHandler");
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */
  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */
  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
extern void xPortPendSVHandler(void);

__attribute__((naked)) void PendSV_Handler(void)
{
  __asm volatile("b xPortPendSVHandler");
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
    extern void xPortSysTickHandler(void);
    xPortSysTickHandler();
  }
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */
void DMA1_Stream0_IRQHandler(void)
{
  uart_link_handle_dma_rx_irq();
}

void USART2_IRQHandler(void)
{
  uart_link_handle_usart_irq();
}

void USART6_IRQHandler(void)
{
  MB_Transport_IRQHandler();
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
  rtos_health_record_stack_overflow((uintptr_t)task, task_name);
  rtos_health_halt();
}

void vApplicationMallocFailedHook(void)
{
  rtos_health_record_malloc_failed();
  rtos_health_halt();
}
/* USER CODE END 1 */
