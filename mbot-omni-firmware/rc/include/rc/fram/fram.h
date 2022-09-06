/**
 * @brief
 *
 * @param
 *
 * @return     0 on success or -1 on failure.
 */
//int mb_();

#include <stdio.h>
#include <stdint.h>

#include <hardware/i2c.h>
#include <pico/stdlib.h>

#define FUJITSU_MANUF_ID 0x00A

#define MAXADDRESS 512
#define PROD_ID_MB85RC04V 0x010	// 4K
#define DENSITY_MB85RC04V 0x0	// 4K

#define MB85RC_ADDRESS_A00   0x50
#define MB85RC_ADDRESS_A01   0x52
#define MB85RC_ADDRESS_A10   0x54
#define MB85RC_ADDRESS_A11   0x56
#define MB85RC_DEFAULT_ADDRESS   MB85RC_ADDRESS_A00

#define MASTER_CODE	0xF8

/**
 * @brief
 *
 * @param
 *
 * @return     0 on success or -1 on failure.
 */
int mb_initialize_fram(i2c_inst_t* i2c);

/**
 * @brief
 *
 * @param
 *
 * @return     0 on success or -1 on failure.
 */
int mb_read_fram(i2c_inst_t* i2c, uint16_t addr, size_t length, uint8_t* data);

/**
 * @brief
 *
 * @param
 *
 * @return     0 on success or -1 on failure.
 */
int mb_write_fram(i2c_inst_t* i2c, uint16_t addr, size_t length, uint8_t* data);

/**
 * @brief
 *
 * @param
 *
 * @return     0 on success or -1 on failure.
 */
int mb_read_word_fram(i2c_inst_t* i2c, uint16_t addr, uint16_t* data);

/**
 * @brief
 *
 * @param
 *
 * @return     0 on success or -1 on failure.
 */
int mb_write_word_fram(i2c_inst_t* i2c, uint16_t addr, uint16_t data);

/**
 * @brief
 *
 * @param
 *
 * @return     0 on success or -1 on failure.
 */
int mb_erase_fram(i2c_inst_t* i2c);