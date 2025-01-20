/*
 * RTC.c
 *
 *  Created on: Jan 3, 2025
 *      Author: Admin
 */


// Helper function to convert BCD to binary
#include <stdio.h>
#include "stdlib.h"
#include "RTC.h"

// Helper function to convert BCD to binary
uint8_t decToBcd(uint8_t val) {
    return (((val / 10)*16) + (val % 10));
}

// Helper function to convert binary to BCD
uint8_t bcdToDec(uint8_t val) {
    return ((val /16)*10) + (val % 16);
}
// Initialize the DS3231 RTC
HAL_StatusTypeDef DS3231_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t data[2];

    // Wake up DS3231 if it is in the "sleep" mode
    data[0] = DS3231_REG_CONTROL;
    data[1] = 0x00;  // Normal mode (turn off the oscillator stop bit)
    return HAL_I2C_Master_Transmit(hi2c, DS3231_ADDR, data, 2, HAL_MAX_DELAY);
}

uint8_t Read(uint8_t reg) {
    uint8_t data;
    HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    return data;
}
void Write(uint8_t reg, uint8_t value) {
    HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);

}

void SetSeconds(uint8_t seconds) {
    Write(0x00, decToBcd(seconds));
                      // Register 0x00 is for seconds
}

void SetMinutes(uint8_t minutes) {
    Write(0x01, decToBcd(minutes));  // Register 0x01 is for minutes
}


void SetHours(uint8_t hours) {
    Write(0x02, decToBcd(hours));  // Register 0x02 is for hours
}
uint8_t GetSeconds() {
    return bcdToDec(Read(0x00));  // Register 0x00 is for seconds
}
uint8_t GetMinutes(void) {
    return bcdToDec(Read(0x01));  // Register 0x01 is for minutes
}

uint8_t GetHours(void) {
    return bcdToDec(Read(0x02));  // Register 0x02 is for hours
}
