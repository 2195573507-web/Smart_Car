/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32H7xx_IT_H
#define __STM32H7xx_IT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum
{
    FAULT_RECORD_EXCEPTION_HARDFAULT = 1U,
    FAULT_RECORD_EXCEPTION_MEMMANAGE = 2U,
    FAULT_RECORD_EXCEPTION_BUSFAULT = 3U,
    FAULT_RECORD_EXCEPTION_USAGEFAULT = 4U
} fault_record_exception_t;

typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t size;
    uint32_t exception;
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t dfsr;
    uint32_t mmfar;
    uint32_t bfar;
    uint32_t shcsr;
    uint32_t msp;
    uint32_t psp;
    uint32_t control;
    uint32_t exc_return;
    uint32_t stacked_sp;
    uint32_t frame_valid;
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t xpsr;
    uint32_t checksum;
} fault_record_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
extern volatile uint32_t g_cm7_svc_count;
extern volatile uint32_t g_cm7_pendsv_count;
extern volatile uint32_t g_cm7_systick_count;
/* USER CODE BEGIN EFP */
/* The record is retained in a no-init RAM section across software resets. */
extern volatile fault_record_t g_fault_record;

/* Call only after fault_record_is_valid() returns true. */
const volatile fault_record_t *fault_record_get(void);

/* Verifies the completed-record marker, format version, size, and checksum. */
bool fault_record_is_valid(void);

/* Invalidates and zeroes the retained record after normal-path reporting. */
void fault_record_clear(void);
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32H7xx_IT_H */
