/***************************************************************************
 * Copyright  2025 TerETAS
 * All right reserved. See COPYRIGHT for detailed Information.
 *
 * @file       tmc51x0_lib.h
 *
 * @author     TerETAS
 * @brief      tmc51x0 dirver lib
 * @date       2024/4/11
 ***************************************************************************/
#ifndef __TMC5130_LIB_H__
#define __TMC5130_LIB_H__

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "tmc51x0_lib_register_address.h"
#include "tmc51x0_lib_port.h"

/**
 * @brief   tmc51x0 lib handle
 */
typedef struct {
    tmc51x0_info* info;
    uint32_t      dev_num;
    int (*lock)(uint8_t lock_state);  // lock_state=0: unlock;1=lock
} tmc51x0_lib_handle;

/**
 * @brief   tmc51x0 lib ramp config 
 * @note    config contain VSTART,A1,V1,AMAX,VMAX,DMAX,D1,VSTOP
 *          All of these parameters are unsigned and do not exceed 2^23,
 *          so they are stored using int, with -1 indicating that the
 *          parameter should not be modified.
 */
typedef struct {
    int vstart;
    int a1;
    int v1;
    int amax;
    int vmax;
    int dmax;
    int d1;
    int vstop;
} tmc51x0_lib_ramp_config;

/**
 * @brief  tmc51x0 lib ramp config default value
 */
#define TMC51X0_LIB_RAMP_CONFIG_DEFAULT  {-1,-1,-1,-1,-1,-1,-1,-1}

/**
 * @brief  function declaration
 */
void tmc51x0_lib_read_reg(tmc51x0_lib_handle* handle, TMC51x0_lib_reg_address addr, uint32_t* data);
void tmc51x0_lib_write_reg(tmc51x0_lib_handle* handle, TMC51x0_lib_reg_address addr, uint32_t data);
uint32_t tmc51x0_lib_endian_convert(uint32_t data);
void     tmc51x0_lib_spi_en(tmc51x0_lib_handle* handle, uint8_t en);
void     tmc51x0_lib_spi_rev(tmc51x0_lib_handle* handle, uint8_t* buf, uint32_t num);
void     tmc51x0_lib_spi_send(tmc51x0_lib_handle* handle, uint8_t* buf, uint32_t num);
void     tmc51x0_lib_handle_init(tmc51x0_lib_handle* handle, tmc51x0_info* info, uint32_t dev_num,
                                 int (*lock)(uint8_t));
void     tmc51x0_lib_spi_send_rev(tmc51x0_lib_handle* handle, uint8_t* txbuf, uint8_t* rxbuf,
                                  uint32_t num);

int tmc51x0_lib_lock(tmc51x0_lib_handle* handle);
int tmc51x0_lib_unlock(tmc51x0_lib_handle* handle);

int  tmc51x0_lib_set_ramp_mode(tmc51x0_lib_handle* handle, uint8_t mode);
int  tmc51x0_lib_get_ramp_mode(tmc51x0_lib_handle* handle, uint32_t* cur_ramp_mode);
int  tmc51x0_lib_set_target_x(tmc51x0_lib_handle* handle, int32_t target_x);
int  tmc51x0_lib_get_target_x(tmc51x0_lib_handle* handle, int32_t* cur_target_x);
int  tmc51x0_lib_set_actual_x(tmc51x0_lib_handle* handle, int32_t actual_x);
int  tmc51x0_lib_get_actual_x(tmc51x0_lib_handle* handle, int32_t* cur_actual_x);
int  tmc51x0_lib_set_target_v(tmc51x0_lib_handle* handle, uint32_t target_v);
int  tmc51x0_lib_set_vmax(tmc51x0_lib_handle* handle, uint32_t vmax);
int  tmc51x0_lib_get_target_v(tmc51x0_lib_handle* handle, uint32_t* cur_target_v);
int  tmc51x0_lib_get_actual_ramp_v(tmc51x0_lib_handle* handle, uint32_t* cur_actual_ramp_gen_v);
int  tmc51x0_lib_get_actual_step_time(tmc51x0_lib_handle* handle, uint32_t* cur_actual_step_time);
int  tmc51x0_lib_get_latch_x(tmc51x0_lib_handle* handle, int32_t* cur_latch_x);
int  tmc51x0_lib_stop_event_ctrl(tmc51x0_lib_handle* handle, uint8_t dir, uint8_t en);
int  tmc51x0_lib_set_stop_mode(tmc51x0_lib_handle* handle, uint8_t mode);
int  tmc5160_lib_clear_sg_stop(tmc51x0_lib_handle* handle);
int  tmc5160_lib_get_ramp_status(tmc51x0_lib_handle* handle, uint32_t* ramp_status);
int  tmc5160_lib_get_drv_status(tmc51x0_lib_handle* handle, uint32_t* drv_status);
void tmc51x0_lib_en_ctrl(tmc51x0_lib_handle* handle, uint8_t en);
int  tmc51x0_lib_is_v_reached(tmc51x0_lib_handle* handle);
int  tmc51x0_lib_is_x_reached(tmc51x0_lib_handle* handle);
int  tmc51x0_lib_is_actual_ramp_v_zero(tmc51x0_lib_handle* handle);
int  tmc51x0_lib_set_target_x_to_actual(tmc51x0_lib_handle* handle);
int  tmc5160_lib_is_l_stop_event(tmc51x0_lib_handle* handle);
int  tmc5160_lib_is_r_stop_event(tmc51x0_lib_handle* handle);
int  tmc5160_lib_is_actual_ramp_v_zero(tmc51x0_lib_handle* handle);
int  tmc5160_lib_get_refr_status(tmc51x0_lib_handle* handle);
int  tmc5160_lib_get_refl_status(tmc51x0_lib_handle* handle);

int tmc51x0_lib_pos_move(tmc51x0_lib_handle* handle, int32_t move_len);
int tmc5160_lib_get_sg_result(tmc51x0_lib_handle* handle, uint32_t* sg_result);
int tmc5160_lib_set_coolconf(tmc51x0_lib_handle* handle, uint32_t coolconf);
int tmc51x0_lib_set_sg_stop(tmc51x0_lib_handle* handle, uint8_t mode);
int tmc51x0_lib_set_ramp_v1(tmc51x0_lib_handle* handle, uint32_t v1);
int tmc51x0_lib_set_ramp_data(tmc51x0_lib_handle* handle, uint32_t a1, uint32_t amax, uint32_t dmax,
                              uint32_t d1);
int tmc51x0_lib_set_tcoolthrs(tmc51x0_lib_handle* handle, uint32_t tcoolthrs);
int tmc51x0_lib_set_chopconf(tmc51x0_lib_handle* handle, uint32_t chopconf);
int tmc51x0_lib_set_ihold_irun(tmc51x0_lib_handle* handle, uint32_t ihold_irun);
int tmc51x0_lib_set_tpwmthrs(tmc51x0_lib_handle* handle, uint32_t tpwmthrs);
int tmc51x0_lib_set_vdcmin(tmc51x0_lib_handle* handle, uint32_t vdcmin);
int tmc51x0_lib_set_dcctrl(tmc51x0_lib_handle* handle, uint32_t dcctrl);

int tmc51x0_lib_get_gstat(tmc51x0_lib_handle* handle, uint32_t* gstat);
int tmc51x0_lib_clear_gstat(tmc51x0_lib_handle* handle, uint32_t clear_bit);
int tmc51x0_lib_clear_ramp_stat(tmc51x0_lib_handle* handle, uint32_t clear_bit);
int tmc51x0_lib_is_sg_stop(tmc51x0_lib_handle* handle);
int tmc51x0_lib_config_ramp(tmc51x0_lib_handle* handle, tmc51x0_lib_ramp_config* config);

/***********************config example***********************

    // CHOPCONF: TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (SpreadCycle)
    tmc51x0_lib_write_reg(&tmc51x0_1_handle, 0x6C, 0x030100C3);
    // IHOLD_IRUN: IHOLD=10, IRUN=31 (max. current), IHOLDDELAY=6
    tmc51x0_lib_write_reg(&tmc51x0_1_handle, 0x10, 0x00060101);
    // TPOWERDOWN=10: Delay before power down in stand still
    tmc51x0_lib_write_reg(&tmc51x0_1_handle, 0x11, 0x0000000A);
    // EN_PWM_MODE=1 enables StealthChop (with default PWMCONF)
    tmc51x0_lib_write_reg(&tmc51x0_1_handle, 0x00, 0x00003184);
    // TPWM_THRS=500 yields a switching velocity about 35000 = ca. 30RPM
    tmc51x0_lib_write_reg(&tmc51x0_1_handle, 0x13, 0x00000500);
    //tcoolthrs
    tmc51x0_lib_write_reg(&tmc51x0_1_handle, 0x14, 0x000004FE);
    //thigh
    tmc51x0_lib_write_reg(&tmc51x0_1_handle, 0x15, 0x0000000A);
    //PWMCONF
    tmc51x0_lib_write_reg(&tmc51x0_1_handle, 0x70, 0x0004010F);
    //sw mode
    tmc51x0_lib_write_reg(&tmc51x0_1_handle, 0x34, 0x000008A3);
    //COOLCONF
    tmc51x0_lib_write_reg(&tmc51x0_1_handle, 0x6D, 0x003F0000);


    //ramp generator
    //a1 = 1 000 First acceleration
    tmc51x0_lib_write_reg(&tmc51x0_1_handle, 0x24, 0x000003E8);
    //v1= 50 000 Acceleration threshold velocity V1
    tmc51x0_lib_write_reg(&tmc51x0_1_handle, 0x25, 0x0000C350);
    //amax= 500 Acceleration above V1
    tmc51x0_lib_write_reg(&tmc51x0_1_handle, 0x26, 0x000001F4);
    //vmax= 200 000
    tmc51x0_lib_write_reg(&tmc51x0_1_handle, 0x27, 0x00010D40);
    //dmax= 700 Deceleration above V1
    tmc51x0_lib_write_reg(&tmc51x0_1_handle, 0x28, 0x000002BC);
    //d1= 1400 Deceleration below V1
    tmc51x0_lib_write_reg(&tmc51x0_1_handle, 0x2A, 0x00000578);
    //vstop= 10 Stop velocity (Near to zero)
    tmc51x0_lib_write_reg(&tmc51x0_1_handle, 0x2B, 0x0000000A);

*****************************************************/

#endif
