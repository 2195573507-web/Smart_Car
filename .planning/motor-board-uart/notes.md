# MotorBoard UART Findings

- The current IOC/source maps PC6 to TIM3_CH1 and PC7 to TIM3_CH2.
- `main.c` calls `MX_TIM3_Init()`, but no application runtime source calls
  `HAL_TIM_PWM_Start`, `HAL_TIM_PWM_Stop`, `pwm_set_duty`, or `bsp_pwm_*`; only
  the BSP test helper references the PWM API.
- USART1 on PA9/PA10 is the debug/log UART. USART2 on PA2/PA3 is the existing
  STM32-S3 transport and is protected.
- The workspace contains no dedicated motor-board protocol manual or clearly
  identified motor-board frame examples.
- The approved route is USART6 with PC6 as TX and PC7 as RX, 115200 8N1,
  hardware flow control disabled.
- Build/static evidence cannot prove electrical UART direction, board response,
  or motor behavior.
