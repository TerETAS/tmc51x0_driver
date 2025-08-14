/***************************************************************************
 * Copyright  2024 TerETAS
 * All right reserved. See COPYRIGHT for detailed Information.
 *
 * @file       tmc51x0_lib_port.h
 *
 * @author     TerETAS
 * @brief      tmc51x0 dirver lib port
 *
 * @date       2024/03/26
 ***************************************************************************/
#ifndef __TMC5130_LIB_PORT_H__
#define __TMC5130_LIB_PORT_H__


typedef struct tmc51x0_info {
    // ------------------------------
    // Platform-specific fields
    // ------------------------------
    
    // Pointer to SPI or UART handle for communication
    // e.g., SPI_HandleTypeDef* hspi;
    
    // GPIO pin definitions for control signals (EN, SPIEN, etc.)
    // e.g., int pin_enable;
      

}tmc51x0_info;

#endif
