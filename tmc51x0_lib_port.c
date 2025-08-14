/***************************************************************************
 * Copyright  2024 TerETAS 
 * All right reserved. See COPYRIGHT for detailed Information.
 *
 * @file       tmc51x0_lib_port.c
 *
 * @author     TerETAS
 * @brief      tmc51x0 dirver lib port
 *
 *
 * @date       2024/03/26
 ***************************************************************************/
#include "tmc51x0_lib_port.h"
#include "tmc51x0_lib.h"
/**
 * @brief   tmc51x0 spi wirte data
 * @param
 * @note
 * @retval
 */
void tmc51x0_lib_spi_send(tmc51x0_lib_handle *handle, uint8_t *buf, uint32_t num) {
    
}

/**
 * @brief   tmc51x0 spi read data
 * @param
 * @note
 * @retval
 */
void tmc51x0_lib_spi_rev(tmc51x0_lib_handle *handle, uint8_t *buf, uint32_t num) {
    
}

/**
 * @brief   tmc51x0 spi en
 * @param   en: 0=disable ,1=enable
 * @note
 * @retval
 */
void tmc51x0_lib_spi_en(tmc51x0_lib_handle *handle, uint8_t en) {
   
}

/**
 * @brief   tmc51x0 big endian little endian convert
 * @param   en: 0=disable ,1=enable
 * @note   __weak function ,need user redefine
 * @retval
 */
uint32_t tmc51x0_lib_endian_convert(uint32_t data) {
    
}

/**
 * @brief   tmc51x0 spi sen and reveice data
 * @param
 * @note   __weak function ,need user redefine
 * @retval
 */
void tmc51x0_lib_spi_send_rev(tmc51x0_lib_handle *handle, uint8_t *txbuf, uint8_t *rxbuf,
                              uint32_t num) {
    
}

/**
 * @brief   tmc51x0 lib drv enable control
 * @param
 * @note
 * @retval
 */
void tmc51x0_lib_en_ctrl(tmc51x0_lib_handle *handle, uint8_t en) {
   
}
