/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      driver_ms5611_interface_template.c
 * @brief     driver ms5611 interface template source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2024-03-31
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2024/03/31  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_ms5611_interface.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os2.h"
#include <stdarg.h>
#include <stdio.h>

extern SPI_HandleTypeDef hspi2;
#define MS5611_CS_GPIO_Port  GPIOB
#define MS5611_CS_Pin        GPIO_PIN_12

static SPI_HandleTypeDef *g_spi = &hspi2;

/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
uint8_t ms5611_interface_iic_init(void)
{
    return 0;
}

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */
uint8_t ms5611_interface_iic_deinit(void)
{
    return 0;
}

/**
 * @brief      interface iic bus read
 * @param[in]  addr iic device write address
 * @param[in]  reg iic register address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t ms5611_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return 0;
}

/**
 * @brief     interface iic bus write
 * @param[in] addr iic device write address
 * @param[in] reg iic register address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t ms5611_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return 0;
}

/**
 * @brief  interface spi bus init
 * @return status code
 *         - 0 success
 *         - 1 spi init failed
 * @note   none
 */
uint8_t ms5611_interface_spi_init(void)
{
	HAL_GPIO_WritePin(MS5611_CS_GPIO_Port, MS5611_CS_Pin, GPIO_PIN_SET);
    return 0;
}

/**
 * @brief  interface spi bus deinit
 * @return status code
 *         - 0 success
 *         - 1 spi deinit failed
 * @note   none
 */
uint8_t ms5611_interface_spi_deinit(void)
{
    return 0;
}

/**
 * @brief      interface spi bus read
 * @param[in]  reg register address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t ms5611_interface_spi_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t status = 0;   /* 0 = OK, 1 = Error */

    HAL_GPIO_WritePin(MS5611_CS_GPIO_Port, MS5611_CS_Pin, GPIO_PIN_RESET);

    if (HAL_SPI_Transmit(g_spi, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        status = 1;
    }
    else if (HAL_SPI_Receive(g_spi, buf, len, HAL_MAX_DELAY) != HAL_OK)
    {
        status = 1;
    }

    HAL_GPIO_WritePin(MS5611_CS_GPIO_Port, MS5611_CS_Pin, GPIO_PIN_SET);
    return status;
}

/**
 * @brief     interface spi bus write
 * @param[in] reg register address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */

uint8_t ms5611_interface_spi_write(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t status = 0;

    HAL_GPIO_WritePin(MS5611_CS_GPIO_Port, MS5611_CS_Pin, GPIO_PIN_RESET);

    if (HAL_SPI_Transmit(g_spi, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        status = 1;
    }
    else if (len && HAL_SPI_Transmit(g_spi, buf, len, HAL_MAX_DELAY) != HAL_OK)
    {
        status = 1;
    }

    HAL_GPIO_WritePin(MS5611_CS_GPIO_Port, MS5611_CS_Pin, GPIO_PIN_SET);
    return status;
}

/**
 * @brief     interface delay ms
 * @param[in] ms time
 * @note      none
 */
void ms5611_interface_delay_ms(uint32_t ms)
{
	osDelay(ms);
}

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      none
 */
void ms5611_interface_debug_print(const char *const fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    printf("%s", buf);
}
