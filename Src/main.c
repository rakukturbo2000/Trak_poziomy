/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "bit_banding.h"
#include "delay.h"
#include "port_function.h"
#include "stm32f1xx.h"

#define SLAVE_ADDRESS_LCD 0x4E
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
// #include "i2c.h"
/* USER CODE END Includes */
#define PRIGROUP_16G_0S ((const uint32_t)0x03)
#define PRIGROUP_8G_2S ((const uint32_t)0x04)
#define PRIGROUP_4G_4S ((const uint32_t)0x05)
#define PRIGROUP_2G_8S ((const uint32_t)0x06)
#define PRIGROUP_0G_16S ((const uint32_t)0x07)

#define ENCODER_BUTTON BB(GPIOB->IDR, PIN10)
#define START_REVERSE_BUTTON BB(GPIOB->IDR, PIN5)
#define CALIBRATION_SENSOR BB(GPIOB->IDR, PIN4)

#define SAW_LEVEL BKP->DR3
#define SAW_WIDTH BKP->DR1
#define PLANK_SIZE BKP->DR2
#define SET_SAW_LEVEL BKP->DR4
#define calibrationLevel    BKP->DR5 //  0,1mmm <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define reverseUpLevel      BKP->DR6 //  1mm <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void SystemClock_Config(void);
void InitTIM2(void);
uint8_t static positioningInaccuracy = 3; // positioningInaccuracy*2  0,1mm
uint8_t static positioningFastSlow =
    100; // odległość od set zmiany prędkości 0,1mm

char line1[16] = " ";
char line2[16] = " ";
char lines[3][16];
uint32_t volatile oldcnt = 1 << 31;
uint8_t volatile sendToDisplayFlag = 0;
uint8_t volatile displayFlag = 0;
uint8_t volatile absoluteIncrement = 0;
uint16_t volatile calculatedSet = 0;
uint16_t volatile time = 2000;
uint8_t volatile start = 0;
uint16_t volatile positionBeforeUp = 0;
uint8_t volatile reverseFlag = 0;
uint16_t volatile referenceSawLevel = 0;
uint8_t volatile writeFlag = 0;
uint8_t volatile lcdReset = 0;

// menu
uint8_t volatile selectedItemIndex = 0;
uint8_t volatile currentLineIndex = 0; // 0-1

uint8_t SetPosition(uint16_t set) {
  if (SAW_LEVEL > set + positioningFastSlow ||
      SAW_LEVEL < set - positioningFastSlow) {
    GPIOB->BRR = PIN14;
  } else {
    GPIOB->BSRR = PIN14;
  }
  if (SAW_LEVEL > set + positioningInaccuracy) {
    GPIOB->BRR = PIN12;
    GPIOB->BSRR = PIN13;
    return 1;
  } else {
    GPIOB->BSRR = PIN12;
  }
  if (SAW_LEVEL <
      ((set > positioningInaccuracy) ? (set - positioningInaccuracy) : 0)) {
    GPIOB->BRR = PIN13;
    return 1;
  } else {
    GPIOB->BSRR = PIN13;
  }
  return 0;
}

void CalculatePosition() {
  if (absoluteIncrement) {
    calculatedSet = SET_SAW_LEVEL * 10;
  } else {
    if (referenceSawLevel > (SAW_WIDTH + PLANK_SIZE * 10)) {
      calculatedSet = referenceSawLevel - SAW_WIDTH - PLANK_SIZE * 10;
    } else {
      calculatedSet = 0;
    }
  }
}

uint32_t checkEncoderButton = 0;
void CheckEncoderButton() {
  if (!ENCODER_BUTTON) {
    checkEncoderButton++;
    if (checkEncoderButton == 2000) {
      if (BB(GPIOC->ODR, PIN13)) {
        displayFlag = 0;
        BB(GPIOC->ODR, PIN13) = 0;
      } else {
        displayFlag = 1;
        BB(GPIOC->ODR, PIN13) = 1;
      }
      sendToDisplayFlag = 1;
    }
  } else {
    if (checkEncoderButton > 20 && checkEncoderButton < 2000) {
      writeFlag ^= 1;
      sendToDisplayFlag = 1;
    }
    checkEncoderButton = 0;
  }
}
uint32_t checkCalibrationSensor = 0;
void CheckCalibrationSensor() {
  if (!CALIBRATION_SENSOR) {
    checkCalibrationSensor++;
    if (checkCalibrationSensor == 20) {
      if (!BB(GPIOB->IDR, PIN13)) {
        TIM2->CNT = calibrationLevel * 2;
      }
    }
  } else {
    checkCalibrationSensor = 0;
  }
}
uint32_t checkStartReverseButton = 0;
void CheckStartReverseButton() {
  if (!START_REVERSE_BUTTON) {
    checkStartReverseButton++;
    if (checkStartReverseButton == 1000) {
    	if(selectedItemIndex == 2 && displayFlag == 1 ){
       	 TIM2->CNT = calibrationLevel * 2;
    	}
      lcdReset = 1;
      sendToDisplayFlag = 1;
    }
  } else {
    if (checkStartReverseButton > 20 && checkStartReverseButton < 1000) {
      if (!displayFlag) {
        if (!start) {
          if (absoluteIncrement) {
            reverseFlag = 0;
            CalculatePosition();
            start = 1;
          } else {
            if (reverseFlag) {
              reverseFlag = 0;
              CalculatePosition();
              start = 1;
            } else {
              referenceSawLevel = SAW_LEVEL;
              reverseFlag = 1;
              start = 1;
            }
          }
        }
      }
      sendToDisplayFlag = 1;
    }
    checkStartReverseButton = 0;
  }
}
uint32_t manualControl = 0;
uint8_t calStartFlag = 0;
void ManualControl() {
  if (!start && !(BB(GPIOB->IDR, PIN12) && BB(GPIOB->IDR, PIN13))) {
    manualControl++;
    if (manualControl > 100) {
      if (CALIBRATION_SENSOR && !calStartFlag) {
        calStartFlag = 1;
      }
      if (calStartFlag) {
        CheckCalibrationSensor();
      }
      calculatedSet = SAW_LEVEL;
      reverseFlag = 0;
    }
  } else {
    manualControl = 0;
    calStartFlag = 0;
  }
}

void ControlDelay() {
  if (start) {
    SetPosition(calculatedSet + (reverseFlag ? reverseUpLevel*10 : 0));
    if (oldcnt != SAW_LEVEL) {
      time = 2000; // ms
    } else {
      if (time) {
        time--;
      } else {
        start = 0;
        time = 2000;
        GPIOB->BSRR = PIN12 | PIN13 | PIN14;
      }
    }
  }
}

void Loop1ms() {
  IWDG->KR = 0xaaaa;
  SAW_LEVEL = TIM2->CNT / 2 ;//- SAW_WIDTH / 2;
  ManualControl();
  CheckEncoderButton();
  CheckStartReverseButton();
  //	CheckCalibrationSensor();
  ControlDelay();
}

void setDisplayContent(char line[], char parameter[], uint16_t value,
                       char symbol) {
  int offset = sprintf(line, "%s=%03d", parameter, value);
  sprintf(line + offset, " %*c\n", 16 - offset - 1, symbol);
}

int main(void) {
  /* USER CODE BEGIN 1 */
  SCB->CCR |= SCB_CCR_STKALIGN_Msk; // wyrównanie stosu do 8 bajtów
  FLASH->ACR |= 0b010;              // flash latency 72MHz
  /* Configure the system clock */
  IWDG->KR = 0x5555;
  IWDG->PR = 7;
  IWDG->RLR = 200;
  IWDG->KR = 0xaaaa;
  SystemClock_Config();
  //  IWDG->KR = 0x5555;
  //  IWDG->PR = 0;
  //  IWDG->RLR = 1000;
  //  IWDG->KR = 0xaaaa;

  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN |
                  RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPDEN;
  RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN | RCC_APB1ENR_I2C1EN |
                  RCC_APB1ENR_TIM2EN;
  /* Initialize all configured peripherals */
  //  InitI2C();
  InitTIM2();
  AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_1;
  /* USER CODE BEGIN 2 */
  PWR->CR = PWR_CR_DBP;
  gpio_pin_cfg(GPIOC, PIN13, gpio_mode_output_PP_2MHz);
  BB(GPIOB->ODR, PIN12) = 1;
  BB(GPIOB->ODR, PIN13) = 1;
  BB(GPIOB->ODR, PIN14) = 1;
  BB(GPIOB->ODR, PIN15) = 1;
  BB(GPIOB->ODR, PIN10) = 1;
  BB(GPIOB->ODR, PIN5) = 1;
  BB(GPIOB->ODR, PIN4) = 1;
  BB(GPIOB->ODR, PIN3) = 1;
  BB(GPIOB->ODR, PIN1) = 1;
  BB(GPIOB->ODR, PIN0) = 1;
  BB(GPIOA->ODR, PIN0) = 1;
  BB(GPIOA->ODR, PIN1) = 1;
  BB(GPIOA->ODR, PIN2) = 1;
  BB(GPIOA->ODR, PIN3) = 1;
  gpio_pin_cfg(GPIOB, PIN12, gpio_mode_output_OD_2MHz); // dół
  gpio_pin_cfg(GPIOB, PIN13, gpio_mode_output_OD_2MHz); // góra
  gpio_pin_cfg(GPIOB, PIN14, gpio_mode_output_OD_2MHz); // szybko/wolno
  gpio_pin_cfg(GPIOB, PIN15, gpio_mode_output_OD_2MHz); // NC
  gpio_pin_cfg(GPIOB, PIN0, gpio_mode_input_pull);
  gpio_pin_cfg(GPIOB, PIN1, gpio_mode_input_pull);
  gpio_pin_cfg(GPIOB, PIN3, gpio_mode_input_pull);
  gpio_pin_cfg(GPIOB, PIN5, gpio_mode_input_pull);
  gpio_pin_cfg(GPIOB, PIN4, gpio_mode_input_pull);
  gpio_pin_cfg(GPIOB, PIN10, gpio_mode_input_pull);
  gpio_pin_cfg(GPIOA, PIN2, gpio_mode_input_pull);
  gpio_pin_cfg(GPIOA, PIN3, gpio_mode_input_pull);
  gpio_pin_cfg(GPIOA, PIN0, gpio_mode_input_pull);
  gpio_pin_cfg(GPIOA, PIN1, gpio_mode_input_pull);
  gpio_pin_cfg(GPIOA, PIN2, gpio_mode_output_PP_2MHz);
  gpio_pin_cfg(GPIOA, PIN3, gpio_mode_output_PP_2MHz);
  gpio_pin_cfg(GPIOA, PIN4, gpio_mode_output_PP_2MHz);
  gpio_pin_cfg(GPIOA, PIN5, gpio_mode_output_PP_2MHz);
  gpio_pin_cfg(GPIOA, PIN6, gpio_mode_output_PP_2MHz);
  gpio_pin_cfg(GPIOA, PIN7, gpio_mode_output_PP_2MHz);
  gpio_pin_cfg(GPIOB, PIN6, gpio_mode_alternate_OD_2MHz);
  gpio_pin_cfg(GPIOB, PIN7, gpio_mode_alternate_OD_2MHz);
  // AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI4_PB;
  AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PB;
  //  HAL_NVIC_SetPriority(EXTI3_IRQn, 4, 0);
  //  EXTI->IMR |= EXTI_IMR_MR4;
  //  EXTI->FTSR |= EXTI_FTSR_TR4;
  EXTI->IMR |= EXTI_IMR_MR0;
  //  EXTI->FTSR |= EXTI_FTSR_TR0;
  EXTI->RTSR |= EXTI_RTSR_TR0;
  //  NVIC_EnableIRQ(EXTI4_IRQn);
  NVIC_EnableIRQ(EXTI0_IRQn);
  //  NVIC_EnableIRQ(TIM3_IRQn);
if(!calibrationLevel){
	calibrationLevel=1500;
}
if(!reverseUpLevel){
	reverseUpLevel=20;
}
if(!SAW_WIDTH){
	SAW_WIDTH=30;
}
  TIM2->CNT = SAW_LEVEL * 2;
  calculatedSet = SAW_LEVEL;
  /* USER CODE END 2 */
  Delay(500);
  lcd_init();
  if (!(RCC->CSR & RCC_CSR_IWDGRSTF)) {

    lcd_send_string("ENERGIA MROZ");

    Delay(1000);
    lcd_put_cur(1, 0);

    lcd_send_string("made by RAKUK");
    Delay(2000);
    lcd_clear();

  } else {
    RCC->CSR &= !RCC_CSR_IWDGRSTF;
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1) {
    if (TIM2->CNT < 200) {
      TIM2->CNT = 200;
    }

    if (oldcnt != SAW_LEVEL || sendToDisplayFlag ||
        BB(GPIOB->IDR, PIN3) == absoluteIncrement) {
      sendToDisplayFlag = 0;
      absoluteIncrement = !BB(GPIOB->IDR, PIN3);
      switch (displayFlag) {
      case 0: {
        if (absoluteIncrement) {
          setDisplayContent(line1, "Wysokosc", SAW_LEVEL / 10,
                            reverseFlag ? 'P' : ' ');
          setDisplayContent(line2, "Wys.zad", SET_SAW_LEVEL,
                            writeFlag ? '*' : ' ');
        } else {
          setDisplayContent(line1, "Wysokosc", SAW_LEVEL / 10,
                            reverseFlag ? 'P' : ' ');
          setDisplayContent(line2, "Deska   ", PLANK_SIZE,
                            writeFlag ? '*' : ' ');
        }
        break;
      }
      case 1:
        sprintf(lines[0], "Szrank=%02d,%01d    %c", SAW_WIDTH / 10, SAW_WIDTH % 10,
                selectedItemIndex == 0 ? writeFlag ? '*' : '<' : ' ');
        sprintf(lines[1], "Wys.cof.=%03d   %c", reverseUpLevel, selectedItemIndex == 1 ? writeFlag ? '*' : '<' : ' ');
        sprintf(lines[2], "Wys.kal.=%03d,%01d %c", calibrationLevel / 10, calibrationLevel % 10, selectedItemIndex == 2 ? writeFlag ? '*' : '<' : ' ');
        sprintf(line1,lines[currentLineIndex]);
        sprintf(line2,lines[currentLineIndex+1]);

        break;
      }
      oldcnt = SAW_LEVEL;

      if (lcdReset) {
        lcd_init();
        Delay(100);
        lcdReset = 0;
      }

      lcd_clear();
      lcd_put_cur(0, 0);
      lcd_send_string(line1);
      lcd_put_cur(1, 0);
      lcd_send_string(line2);
    }
    /* USER CODE END 3 */
  }
}

__attribute__((interrupt)) void EXTI4_IRQHandler(void) {
  if (EXTI->PR & EXTI_PR_PR4) {
    EXTI->PR = EXTI_PR_PR4;
    //     if(!BB(GPIOB->IDR,PIN1)){
    //         TIM2->CNT=calibrationLevel*2;
    //     }
  }
}
__attribute__((interrupt)) void EXTI0_IRQHandler(void) {
  uint8_t i = 0;

  if (!(EXTI->PR & EXTI_PR_PR0)) {
    return;
  };

  EXTI->PR = EXTI_PR_PR0;

  if (BB(GPIOB->IDR, PIN0) == 1) {
    if (BB(GPIOB->IDR, PIN1) == 1) {
      i++;
    }
  } else {
    if (!(BB(GPIOB->IDR, PIN1) == 1)) {
      i++;
    }
  }

  switch (displayFlag) {
  case 0:
    if (writeFlag) {
      if (absoluteIncrement) {
        if (i) {
          SET_SAW_LEVEL =
              SET_SAW_LEVEL >= 700 ? 700 : SET_SAW_LEVEL + 1 ; // grubość
        } else {
          SET_SAW_LEVEL = SET_SAW_LEVEL <=10 ? 10 : SET_SAW_LEVEL - 1 ; // grubość
        }
      } else {
        if (i) {
          PLANK_SIZE = PLANK_SIZE >=500 ? 500 : PLANK_SIZE + 1 ; // deska
        } else {
          PLANK_SIZE = PLANK_SIZE <=0 ? 0 : PLANK_SIZE - 1 ; // deska
        }
      }
    }
    break;
  case 1: {
    if (writeFlag) {
      switch (selectedItemIndex) {
      case 0: {
        if (i) {
          SAW_WIDTH = SAW_WIDTH >=100 ? 100 : SAW_WIDTH + 1; // szrank
        } else {
          SAW_WIDTH = SAW_WIDTH <=0 ? 0 : SAW_WIDTH - 1 ; // szrank
        }
        break;
      }

      case 1: {
        if (i) {
          reverseUpLevel = reverseUpLevel >= 200 ? 200 : reverseUpLevel + 1;
        } else {
          reverseUpLevel = reverseUpLevel <= 20 ? 20 : reverseUpLevel - 1;
        }
        break;
      }
	  case 2: {
        if (i) {
          calibrationLevel =
              calibrationLevel >= 3000 ? 3000 : calibrationLevel + 1;
        } else {
          calibrationLevel = calibrationLevel <= 20 ? 20 : calibrationLevel - 1;
        }
        break;
      }
      }

    } else {
      if (i) {
        selectedItemIndex = selectedItemIndex >= 2 ? 2 : selectedItemIndex + 1;
		 currentLineIndex = selectedItemIndex ==2?1:0;
      } else {
        selectedItemIndex = selectedItemIndex <= 0 ? 0 : selectedItemIndex - 1;
		currentLineIndex = selectedItemIndex ==0?0:1;
      }
    }
  } break;
  }
  sendToDisplayFlag = 1;
}

void SystemClock_Config(void) {

  RCC->CR |= RCC_CR_HSEON;
  RCC->CFGR = RCC_CFGR_PLLMULL9 | RCC_CFGR_PLLSRC | RCC_CFGR_ADCPRE_DIV6 |
              RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_USBPRE;
  while (!(RCC->CR & RCC_CR_HSERDY))
    ;
  RCC->CR |= RCC_CR_PLLON;
  FLASH->ACR |= FLASH_ACR_LATENCY_1;
  while (!(RCC->CR & RCC_CR_PLLRDY))
    ;
  RCC->CFGR |= RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    ;
  RCC->CR &= ~RCC_CR_HSION;
  SysTick_Config(72000000 / 8 / 1000);
  SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;
}

void InitTIM2(void) {

  TIM2->SMCR = TIM_SMCR_SMS_1; //|TIM_SMCR_SMS_0;
  //  TIM2->CCMR1 = TIM_CCMR1_IC1F | TIM_CCMR1_IC2F;//filter
  TIM2->CCER |= TIM_CCER_CC1P;
  TIM2->ARR = 0xffff;
  //  TIM2->DIER = TIM_DIER_UIE; //interrupt
  TIM2->PSC = 0;
  TIM2->CR1 |= TIM_CR1_CEN;
  TIM2->EGR |= TIM_EGR_UG;
}
/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state
   */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
     file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF
 * FILE****/
