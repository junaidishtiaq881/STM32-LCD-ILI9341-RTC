/*
 * RTC.h
 *
 *  Created on: Jan 3, 2025
 *      Author: Admin
 */

#ifndef INC_RTC_H_
#define INC_RTC_H_

#include "stm32l4xx_hal.h"
#include "main.h"
extern I2C_HandleTypeDef hi2c1;
// I2C address of DS3231 (0x68 for default configuration)
#define DS3231_ADDR (0xD0)  //(0x68 for default configuration)
#define DS3231_REG_SECONDS 0X00
#define DS3231_REG_MINUTES 0X01
#define DS3231_REG_HOURS 0X02

#define DS3231_REG_CONTROL  0x0E  // Control register
#define DS3231_REG_STATUS   0x0F  // Status register

// Function prototypes
HAL_StatusTypeDef DS3231_Init(I2C_HandleTypeDef *hi2c);
uint8_t decToBcd(uint8_t val);
uint8_t bcdToDec(uint8_t val);
uint8_t Read(uint8_t reg);
void Write(uint8_t reg, uint8_t value);
void SetSeconds(uint8_t seconds);
void SetMinutes(uint8_t minutes);
void SetHours(uint8_t hours);
uint8_t GetSeconds(void);
uint8_t GetMinutes(void);
uint8_t GetHours(void);
#endif /* INC_RTC_H_ */
