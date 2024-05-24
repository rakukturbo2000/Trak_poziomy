#ifndef PORT_FUNCTION


#include "stm32f1xx.h"
#define PORT_FUNCTION
typedef enum {
PIN0 = 0x00000001,
PIN1 = 0x00000002,
PIN2 = 0x00000004,
PIN3 = 0x00000008,
PIN4 = 0x00000010,
PIN5 = 0x00000020,
PIN6 = 0x00000040,
PIN7 = 0x00000080,
PIN8 = 0x00000100,
PIN9 = 0x00000200,
PIN10 = 0x00000400,
PIN11 = 0x00000800,
PIN12 = 0x00001000,
PIN13 = 0x00002000,
PIN14 = 0x00004000,
PIN15 = 0x00008000,
}GpioPin_t;

typedef enum {
/* Push-Pull */
gpio_mode_output_PP_2MHz = 2,
gpio_mode_output_PP_10MHz = 1,
gpio_mode_output_PP_50MHz = 3,
OP_PP_2MHz = 2,
OP_PP_10MHz = 1,
OP_PP_50MHz = 3,
/* Open-Drain */
gpio_mode_output_OD_2MHz = 6,
gpio_mode_output_OD_10MHz = 5,
gpio_mode_output_OD_50MHz = 7,
OP_OD_2MHz = 6,
OP_OD_10MHz = 5,
OP_OD_50MHz = 7,
/* Push-Pull */
gpio_mode_alternate_PP_2MHz = 10,
gpio_mode_alternate_PP_10MHz = 9,
gpio_mode_alternate_PP_50MHz = 11,
AF_PP_2MHz = 10,
AF_PP_10MHz = 9,
AF_PP_50MHz = 11,
/* Open-Drain */
gpio_mode_alternate_OD_2MHz = 14,
gpio_mode_alternate_OD_10MHz = 13,
gpio_mode_alternate_OD_50MHz = 15,
AF_OD_2MHz = 14,
AF_OD_10MHz = 13,
AF_OD_50MHz = 15,
/* Analog input/output (ADC; DAC) */
analog = 0,
gpio_mode_input_analog = 0,
gpio_mode_output_analog = 0,
/* Floating digital input. */
IP_floating = 4,
gpio_mode_input_floating = 4,
/* Digital input with pull-up/down (depending on the ODR reg.). */
IP_pull = 8,
gpio_mode_input_pull = 8

} GpioMode_t;





void gpio_pin_cfg(GPIO_TypeDef * const port, GpioPin_t pin, GpioMode_t mode){
pin = __builtin_ctz(pin)*4;
uint32_t volatile * cr_reg;
uint32_t cr_val;
cr_reg = &port->CRL;
if (pin > 28){
pin -= 32;
cr_reg = &port->CRH;
}
cr_val = *cr_reg;
cr_val &= ~((uint32_t)(0x0f << pin));
cr_val |= (uint32_t)(mode << pin);
*cr_reg = cr_val;
}


#endif


