/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.cc
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ComRecv.hh"
#include "ComSend.hh"
#include <LowLayer.hh>
#include <MiddleLayer.hh>
#include <array>
#include <cstdint>
#include <cstdio>

using LibMecha::LowLayer::Encoder;
using LibMecha::MiddleLayer::SBDBT;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum class Cmd : std::uint8_t {
    GET_ROT        = 0b00010,
    GET_DELTA_ROT  = 0b00011,
    GET_DEG        = 0b00100,
    GET_DELTA_DEG  = 0b00101,
    GET_RAD        = 0b01000,
    GET_DELTA_RAD  = 0b01001,
    GET_REVL       = 0b10000,
    GET_DELTA_REVL = 0b10001
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
std::array<Encoder, 5> enc {
    Encoder(TIM1, 512),
    Encoder(TIM3, 512),
    Encoder(TIM4, 512),
    Encoder(TIM2, 512),
    Encoder(TIM8, 512)
};
SendBuffer buf;
bool recvComplete;
ComSend comTx(USART2, buf);
ComRecv comRx(USART2, recvComplete, [](const std::vector<std::uint8_t> &data) -> bool {
    if(data.size() != 4) return false;
    Encoder &currEnc = enc.at(data.at(2));
    if(data.at(1) == static_cast<std::uint8_t>(Cmd::GET_ROT)) {
        std::int32_t result = currEnc.getRot();
        auto &sendData = reinterpret_cast<std::uint8_t (&)[4]>(result);
        comTx.send(sendData);
    }
    if(data.at(1) == static_cast<std::uint8_t>(Cmd::GET_DELTA_ROT)) {
        std::int32_t result = currEnc.getDeltaRot();
        auto &sendData = reinterpret_cast<std::uint8_t (&)[4]>(result);
        comTx.send(sendData);
    }
    if(data.at(1) == static_cast<std::uint8_t>(Cmd::GET_DEG)) {
        float result = currEnc.getDegree();
        auto &sendData = reinterpret_cast<std::uint8_t (&)[4]>(result);
        comTx.send(sendData);
    }
    if(data.at(1) == static_cast<std::uint8_t>(Cmd::GET_DELTA_DEG)) {
        float result = currEnc.getDeltaDegree();
        auto &sendData = reinterpret_cast<std::uint8_t (&)[4]>(result);
        comTx.send(sendData);
    }
    if(data.at(1) == static_cast<std::uint8_t>(Cmd::GET_RAD)) {
        float result = currEnc.getRadian();
        auto &sendData = reinterpret_cast<std::uint8_t (&)[4]>(result);
        comTx.send(sendData);
    }
    if(data.at(1) == static_cast<std::uint8_t>(Cmd::GET_DELTA_RAD)) {
        float result = currEnc.getDeltaRadian();
        auto &sendData = reinterpret_cast<std::uint8_t (&)[4]>(result);
        comTx.send(sendData);
    }
    if(data.at(1) == static_cast<std::uint8_t>(Cmd::GET_REVL)) {
        std::int32_t result = currEnc.getRevolution();
        auto &sendData = reinterpret_cast<std::uint8_t (&)[4]>(result);
        comTx.send(sendData);
    }
    if(data.at(1) == static_cast<std::uint8_t>(Cmd::GET_DELTA_REVL)) {
        std::int32_t result = currEnc.getDeltaRevolution();
        auto &sendData = reinterpret_cast<std::uint8_t (&)[4]>(result);
        comTx.send(sendData);
    }
    return true;
});
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config();
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main() {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    /* System interrupt init*/
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* SysTick_IRQn interrupt configuration */
    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM8_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    /* USER CODE BEGIN 2 */
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_12);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while(true) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        /*
        comTx.send({
            'T',
            'E',
            'S',
            'T'
        });
         */
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_14);
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_15);
        LL_mDelay(500);
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config() {
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
    while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_5);
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
    while(LL_PWR_IsActiveFlag_VOS() == 0);
    LL_RCC_HSI_SetCalibTrimming(16);
    LL_RCC_HSI_Enable();

    /* Wait till HSI is ready */
    while(LL_RCC_HSI_IsReady() != 1);
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_8, 168, LL_RCC_PLLP_DIV_2);
    LL_RCC_PLL_Enable();

    /* Wait till PLL is ready */
    while(LL_RCC_PLL_IsReady() != 1);
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    /* Wait till System clock is ready */
    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);
    LL_Init1msTick(168000000);
    LL_SetSystemCoreClock(168000000);
}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE {
    LL_USART_TransmitData8(USART2, ch);
    while(LL_USART_IsActiveFlag_TXE(USART2));
    return ch;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler() {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while(true);
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
void assert_failed(std::uint8_t *file, std::uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}

#endif // USE_FULL_ASSERT
