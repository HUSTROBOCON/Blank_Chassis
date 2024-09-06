/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi1;

extern SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN Private defines */
#define SPIn                        2

#define SPI1_GPIO_AF                GPIO_AF_SPI1
#define SPI1_CLK                    RCC_APB2Periph_SPI1
#define SPI1_CLK_CMD                RCC_APB2PeriphClockCmd

#define SPI1_GPIO_CLK_CMD           RCC_AHB1PeriphClockCmd

#define SPI1_SCK                    GPIO_PIN_5
#define SPI1_SCK_PINSOURCE          GPIO_PINSource5
#define SPI1_SCK_GPIO_PORT          GPIOA
#define SPI1_SCK_CLK                RCC_AHB1Periph_GPIOA

#define SPI1_MISO                   GPIO_PIN_6
#define SPI1_MISO_PINSOURCE         GPIO_PINSource6
#define SPI1_MISO_GPIO_PORT         GPIOA
#define SPI1_MISO_CLK               RCC_AHB1Periph_GPIOA

#define SPI1_MOSI                   GPIO_PIN_7
#define SPI1_MOSI_PINSOURCE         GPIO_PINSource7
#define SPI1_MOSI_GPIO_PORT         GPIOA
#define SPI1_MOSI_CLK               RCC_AHB1Periph_GPIOA

#define SPI1_CSn                    1

#define SPI1_CS_1                   GPIO_PIN_4
#define SPI1_CS_GPIO_PORT_1         GPIOA
#define SPI1_CS_CLK_1               RCC_AHB1Periph_GPIOA

#define SPI2_GPIO_AF                GPIO_AF_SPI2
#define SPI2_CLK                    RCC_APB1Periph_SPI2
#define SPI2_CLK_CMD                RCC_APB1PeriphClockCmd

#define SPI2_GPIO_CLK_CMD           RCC_AHB1PeriphClockCmd

#define SPI2_SCK                    GPIO_PIN_10
#define SPI2_SCK_PINSOURCE          GPIO_PINSource10
#define SPI2_SCK_GPIO_PORT          GPIOB
#define SPI2_SCK_CLK                RCC_AHB1Periph_GPIOB

#define SPI2_MISO                   GPIO_PIN_2
#define SPI2_MISO_PINSOURCE         GPIO_PINSource2
#define SPI2_MISO_GPIO_PORT         GPIOC
#define SPI2_MISO_CLK               RCC_AHB1Periph_GPIOC

#define SPI2_MOSI                 	GPIO_PIN_3
#define SPI2_MOSI_PINSOURCE         GPIO_PINSource3
#define SPI2_MOSI_GPIO_PORT         GPIOC
#define SPI2_MOSI_CLK               RCC_AHB1Periph_GPIOC

#define SPI2_CSn                    3

#define SPI2_CS_1                   GPIO_PIN_0
#define SPI2_CS_GPIO_PORT_1         GPIOB
#define SPI2_CS_CLK_1               RCC_AHB1Periph_GPIOB

#define SPI2_CS_2                   GPIO_PIN_1
#define SPI2_CS_GPIO_PORT_2         GPIOB
#define SPI2_CS_CLK_2               RCC_AHB1Periph_GPIOB

#define SPI2_CS_3                   GPIO_PIN_2
#define SPI2_CS_GPIO_PORT_3         GPIOB
#define SPI2_CS_CLK_3               RCC_AHB1Periph_GPIOB
/* USER CODE END Private defines */

void MX_SPI1_Init(void);
void MX_SPI2_Init(void);

/* USER CODE BEGIN Prototypes */
uint8_t SPI_ReadWriteByte(SPI_HandleTypeDef hspi,uint8_t TxData);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

