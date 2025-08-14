/***************************************************************************
 * Copyright  2025 TerETAS
 * All right reserved. See COPYRIGHT for detailed Information.
 *
 * @file       tmc51x0_lib.c
 *
 * @author     TerETAS
 * @brief      tmc51x0 dirver lib
 *
 *
 * @date       2025/04/11
 ***************************************************************************/
#include "tmc51x0_lib.h"

#define __weak __attribute((weak))
#define __IO   volatile



/**
 * @brief   tmc51x0 lib handle init
 * @param
 * @note
 * @retval
 */
void tmc51x0_lib_handle_init(tmc51x0_lib_handle* handle, tmc51x0_info* info, uint32_t dev_num,
                             int (*lock)(uint8_t)) {
    handle->info    = info;
    handle->dev_num = dev_num;
    handle->lock    = lock;
}

/**
 * @brief   tmc51x0 lib lock
 * @note
 */
int tmc51x0_lib_lock(tmc51x0_lib_handle* handle) {
    if (handle->lock == 0) {
        return 1;
    }

    return handle->lock(1);
}

/**
 * @brief   tmc51x0 lib unlock
 * @note
 */
int tmc51x0_lib_unlock(tmc51x0_lib_handle* handle) {
    if (handle->lock == 0) {
        return 1;
    }

    return handle->lock(0);
}

/**
 * @brief   tmc51x0 write register
 * @param
 * @note
 * @retval
 */
void tmc51x0_lib_write_reg(tmc51x0_lib_handle* handle, TMC51x0_lib_reg_address addr,
                           uint32_t data) {
    uint32_t tmp_data = 0;
    tmp_data          = tmc51x0_lib_endian_convert(data);
    addr              = addr | 0x80;
    tmc51x0_lib_spi_en(handle, 1);
    tmc51x0_lib_spi_send(handle, &addr, 1);
    tmc51x0_lib_spi_send(handle, ((uint8_t*)&tmp_data), 4);

    tmc51x0_lib_spi_en(handle, 0);
}

/**
 * @brief   tmc51x0 read register
 * @param
 * @note
 * @retval
 */
void tmc51x0_lib_read_reg(tmc51x0_lib_handle* handle, TMC51x0_lib_reg_address addr,
                          uint32_t* data) {
    uint32_t tmp_data = 0;
    tmc51x0_lib_spi_en(handle, 1);

    tmc51x0_lib_spi_send(handle, &addr, 1);
    tmc51x0_lib_spi_rev(handle, (uint8_t*)&tmp_data, 4);

    tmc51x0_lib_spi_en(handle, 0);
    // The second sequence will get the register value
    tmc51x0_lib_spi_en(handle, 1);

    tmc51x0_lib_spi_send(handle, &addr, 1);
    tmc51x0_lib_spi_rev(handle, (uint8_t*)&tmp_data, 4);

    tmc51x0_lib_spi_en(handle, 0);
    *data = tmc51x0_lib_endian_convert(tmp_data);
}

/**
 * @brief   tmc51x0 Set ramp mode
 * @param   mode =0, Positioning mode
 *          mode =1, Velocity mode to positive
 *          mode =2, Velocity mode to negative
 *          mode =3, Hold mode
 * @note    used in RAMP generator mode
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_set_ramp_mode(tmc51x0_lib_handle* handle, uint8_t mode) {
    if (mode > 3) {
        return 1;
    }

    tmc51x0_lib_lock(handle);
    tmc51x0_lib_write_reg(handle, R_TMC51x0_RAMPMODE, mode);
    tmc51x0_lib_unlock(handle);
    return 0;
}

/**
 * @brief   tmc51x0 get cur ramp mode
 * @param   [out]cur_ramp_mode: cur ramp mode address
 *           =0, Positioning mode
 *           =1, Velocity mode to positive
 *           =2, Velocity mode to negative
 *           =3, Hold mode
 * @note    used in RAMP generator mode
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_get_ramp_mode(tmc51x0_lib_handle* handle, uint32_t* cur_ramp_mode) {
    tmc51x0_lib_lock(handle);
    tmc51x0_lib_read_reg(handle, R_TMC51x0_RAMPMODE, cur_ramp_mode);
    tmc51x0_lib_unlock(handle);
    return 0;
}

/**
 * @brief   tmc51x0 Set target x
 * @param   targex_x =target
 * @note    used in RAMP generator mode
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_set_target_x(tmc51x0_lib_handle* handle, int32_t target_x) {
    tmc51x0_lib_lock(handle);
    tmc51x0_lib_write_reg(handle, R_TMC51x0_XTARGET, target_x);
    tmc51x0_lib_unlock(handle);
    return 0;
}

/**
 * @brief   tmc51x0 get target x
 * @param   [out]cur_target_x:cur target x
 * @note    used in RAMP generator mode
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_get_target_x(tmc51x0_lib_handle* handle, int32_t* cur_target_x) {
    tmc51x0_lib_lock(handle);
    tmc51x0_lib_read_reg(handle, R_TMC51x0_XTARGET, (uint32_t*)cur_target_x);
    tmc51x0_lib_unlock(handle);
    return 0;
}

/**
 * @brief   tmc51x0 set cur Actual position
 * @param   actual_x:actual x
 * @note    used in RAMP generator mode
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_set_actual_x(tmc51x0_lib_handle* handle, int32_t actual_x) {
    tmc51x0_lib_lock(handle);
    tmc51x0_lib_write_reg(handle, R_TMC51x0_XACTUAL, actual_x);
    tmc51x0_lib_unlock(handle);
    return 0;
}

/**
 * @brief   tmc51x0 get cur Actual position
 * @param   [out]cur_actual_x:cur actual x
 * @note    used in RAMP generator mode
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_get_actual_x(tmc51x0_lib_handle* handle, int32_t* cur_actual_x) {
    tmc51x0_lib_lock(handle);
    tmc51x0_lib_read_reg(handle, R_TMC51x0_XACTUAL, (uint32_t*)cur_actual_x);
    tmc51x0_lib_unlock(handle);
    return 0;
}

/**
 * @brief   tmc51x0 set  target velocity
 * @param   target_v:target_v
 * @note    used in RAMP generator mode
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_set_target_v(tmc51x0_lib_handle* handle, uint32_t target_v) {
    if (target_v > 0x008001FF) {
        return 1;
    }

    tmc51x0_lib_lock(handle);
    tmc51x0_lib_write_reg(handle, R_TMC51x0_VMAX, target_v);
    tmc51x0_lib_unlock(handle);
    return 0;
}

/**
 * @brief   tmc51x0 set  VMAX
 * @param   vmax:vmax
 * @note    used in RAMP generator mode
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_set_vmax(tmc51x0_lib_handle* handle, uint32_t vmax) {
    if (vmax > 0x008001FF) {
        return 1;
    }

    tmc51x0_lib_lock(handle);
    tmc51x0_lib_write_reg(handle, R_TMC51x0_VMAX, vmax);
    tmc51x0_lib_unlock(handle);
    return 0;
}

/**
 * @brief   tmc51x0 get cur target velocity
 * @param   [out]cur_target_v:cur target velocity
 * @note    used in RAMP generator mode
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_get_target_v(tmc51x0_lib_handle* handle, uint32_t* cur_target_v) {
    tmc51x0_lib_lock(handle);
    tmc51x0_lib_read_reg(handle, R_TMC51x0_VMAX, (uint32_t*)cur_target_v);
    tmc51x0_lib_unlock(handle);
    return 0;
}

/**
 * @brief   tmc51x0 get cur actual velocity for ramp generator
 * @param   [out]cur_actual_ramp_gen_v:cur actual velocity for ramp generator
 * @note    used in RAMP generator mode
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_get_actual_ramp_v(tmc51x0_lib_handle* handle, uint32_t* cur_actual_ramp_gen_v) {
    tmc51x0_lib_lock(handle);
    tmc51x0_lib_read_reg(handle, R_TMC51x0_VACTUAL, (uint32_t*)cur_actual_ramp_gen_v);
    tmc51x0_lib_unlock(handle);
    return 0;
}

/**
 * @brief   tmc51x0 get actual step time
 * @param   [out]cur_actual_step_time:actual step time for chopper
 * @note    Actual measured time between two 1/256 microsteps derived
 *          from the step input frequency in units of 1/fCLK. Measured
 *           value is (2^20)-1 in case of overflow or stand still.
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_get_actual_step_time(tmc51x0_lib_handle* handle, uint32_t* cur_actual_step_time) {
    tmc51x0_lib_lock(handle);
    tmc51x0_lib_read_reg(handle, R_TMC51x0_TSTEP, (uint32_t*)cur_actual_step_time);
    tmc51x0_lib_unlock(handle);
    return 0;
}

/**
 * @brief   tmc51x0 get latch position
 * @param   [out]cur_latch_x:cur latch position
 * @note    used in RAMP generator mode
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_get_latch_x(tmc51x0_lib_handle* handle, int32_t* cur_latch_x) {
    tmc51x0_lib_lock(handle);
    tmc51x0_lib_read_reg(handle, R_TMC51x0_XLATCH, (uint32_t*)cur_latch_x);
    tmc51x0_lib_unlock(handle);
    return 0;
}

/**
 * @brief   tmc51x0 set stop enable control
 * @param   dir: 0= left ref  1=right ref
 *          en: 0=disable ,1=enable
 * @note    used in RAMP generator mode
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_stop_event_ctrl(tmc51x0_lib_handle* handle, uint8_t dir, uint8_t en) {
    uint32_t tmp;
    tmc51x0_lib_lock(handle);
    tmc51x0_lib_read_reg(handle, R_TMC51x0_SW_MODE, &tmp);

    if (!dir) {
        tmp &= ~(0x00000001);
        tmp |= ((!!(uint32_t)en));
    } else {
        tmp &= ~(0x00000002);
        tmp |= ((!!(uint32_t)en) << 1);
    }

    tmc51x0_lib_write_reg(handle, R_TMC51x0_SW_MODE, tmp);

    tmc51x0_lib_unlock(handle);
    return 0;
}

/**
 * @brief   tmc51x0 set stop moed
 * @param   mode:0=hard stop, 1=soft stop
 * @note    used in RAMP generator mode
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_set_stop_mode(tmc51x0_lib_handle* handle, uint8_t mode) {
    uint32_t tmp;
    tmc51x0_lib_lock(handle);
    tmc51x0_lib_read_reg(handle, R_TMC51x0_SW_MODE, &tmp);
    tmp &= ~(0x00000800);
    tmp |= (!!mode) << 11;
    tmc51x0_lib_write_reg(handle, R_TMC51x0_SW_MODE, tmp);
    tmc51x0_lib_unlock(handle);
    return 0;
}

/**
  * @brief   tmc5160 clear StallGuard stop event
  * @param
  * @note    used in RAMP generator mode
             !! used for tmc5160
  * @retval  0=succee  ;1=failed
*/
int tmc5160_lib_clear_sg_stop(tmc51x0_lib_handle* handle) {
    tmc51x0_lib_lock(handle);
    tmc51x0_lib_write_reg(handle, R_TMC51x0_RAMP_STAT, 0x00000040);
    tmc51x0_lib_unlock(handle);
    return 0;
}

/**
 * @brief   tmc5160 get ramp status
 * @param   [out]ramp_status:Ramp & Reference Switch Status Register
 * @note    used in RAMP generator mode
 * @retval  0=succee  ;1=failed
 */
int tmc5160_lib_get_ramp_status(tmc51x0_lib_handle* handle, uint32_t* ramp_status) {
    tmc51x0_lib_lock(handle);
    tmc51x0_lib_read_reg(handle, R_TMC51x0_RAMP_STAT, ramp_status);
    tmc51x0_lib_unlock(handle);
    return 0;
}

/**
 * @brief   tmc51x0 get target position reached
 * @param
 * @note    used in RAMP generator mode
 * @retval  0=not reached  ;1=reached.
 */
int tmc51x0_lib_is_x_reached(tmc51x0_lib_handle* handle) {
    uint32_t status;
    tmc5160_lib_get_ramp_status(handle, &status);

    if (status & 0x00000200) {
        return 1;
    }

    return 0;
}

/**
 * @brief   tmc51x0 get target velocity reached
 * @param
 * @note    used in RAMP generator mode
 * @retval  0=not reached  ;1=reached.
 */
int tmc51x0_lib_is_v_reached(tmc51x0_lib_handle* handle) {
    uint32_t status;
    tmc5160_lib_get_ramp_status(handle, &status);

    if (status & 0x00000100) {
        return 1;
    }

    return 0;
}
/**
 * @brief   tmc5160 get drv status
 * @param   [out]drv_status:StallGuard2 Value and Driver Error Flags
 * @note
 * @retval  0=succee  ;1=failed
 */
int tmc5160_lib_get_drv_status(tmc51x0_lib_handle* handle, uint32_t* drv_status) {
    tmc51x0_lib_lock(handle);
    tmc51x0_lib_read_reg(handle, R_TMC51x0_DRV_STATUS, drv_status);
    tmc51x0_lib_unlock(handle);
    return 0;
}

/**
 * @brief   tmc51x0 set targert position to actual postiion
 * @param    [out]out_actual_x:actual x
 * @note    used in RAMP generator mode
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_set_target_x_to_actual(tmc51x0_lib_handle* handle) {
    int32_t actual_x;
    int32_t ret  = 0;
    ret         += tmc51x0_lib_get_actual_x(handle, &actual_x);

    if (ret) {
        return ret;
    }

    ret += tmc51x0_lib_set_target_x(handle, actual_x);

    return ret;
}

/**
 * @brief   tmc51x0 is actual ramp generator speed 0
 * @param
 * @note    used in RAMP generator mode, check ramp v is zero
 * @retval  0=not zero  ;1=zero
 */
int tmc51x0_lib_is_actual_ramp_v_zero(tmc51x0_lib_handle* handle) {
    uint32_t cur_v;
    tmc51x0_lib_get_actual_ramp_v(handle, &cur_v);
    return !cur_v;
}

/**
 * @brief   tmc5160 is actual ramp generator speed 0
 * @param
 * @note    used in RAMP generator mode, used ramp_status regsiter vzero flag
 * @retval  0=not zero  ;1=zero
 */
int tmc5160_lib_is_actual_ramp_v_zero(tmc51x0_lib_handle* handle) {
    uint32_t status;
    tmc5160_lib_get_ramp_status(handle, &status);
    return !!(status & 0x00000400);
}

/**
 * @brief   tmc5160 is r stop event happend
 * @param
 * @note    used in RAMP generator mode, used in tmc5160
 * @retval  0=not happend  ;1=happend
 */
int tmc5160_lib_is_r_stop_event(tmc51x0_lib_handle* handle) {
    uint32_t status;
    tmc5160_lib_get_ramp_status(handle, &status);

    if (status & 0x00000020) {
        return 1;

    } else {
        return 0;
    }
}

/**
 * @brief   tmc5160 is l stop event happend
 * @param
 * @note    used in RAMP generator mode
 * @retval  0=not happend  ;1=happend
 */
int tmc5160_lib_is_l_stop_event(tmc51x0_lib_handle* handle) {
    uint32_t status;
    tmc5160_lib_get_ramp_status(handle, &status);

    if (status & 0x00000010) {
        return 1;

    } else {
        return 0;
    }
}

/**
 * @brief   tmc5160 get refl switch status
 * @param
 * @note    used in RAMP generator mode
 * @retval  0=inactive  ;1=active
 */
int tmc5160_lib_get_refl_status(tmc51x0_lib_handle* handle) {
    uint32_t status;
    tmc5160_lib_get_ramp_status(handle, &status);

    if (status & 0x00000001) {
        return 1;

    } else {
        return 0;
    }
}

/**
 * @brief   tmc5160 get refr switch status
 * @param
 * @note    used in RAMP generator mode
 * @retval  0=inactive  ;1=active
 */
int tmc5160_lib_get_refr_status(tmc51x0_lib_handle* handle) {
    uint32_t status;
    tmc5160_lib_get_ramp_status(handle, &status);

    if (status & 0x00000002) {
        return 1;

    } else {
        return 0;
    }
}

/**
 * @brief   tmc51x0  position move
 * @param   move_len: The distance to move relative to the current position
 * @note    used in RAMP generator mode
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_pos_move(tmc51x0_lib_handle* handle, int32_t move_len) {
    int32_t cur_x;
    tmc51x0_lib_lock(handle);

    // get actual x position
    tmc51x0_lib_read_reg(handle, R_TMC51x0_XACTUAL, (uint32_t*)&cur_x);
    // add move length
    cur_x += move_len;
    // get cur_x to target x
    tmc51x0_lib_write_reg(handle, R_TMC51x0_XTARGET, cur_x);

    tmc51x0_lib_unlock(handle);

    return 0;
}
/**
 * @brief   tmc5160  get sg_result
 * @param
 * @note    used in RAMP generator mode
 * @retval  0=succee  ;1=failed
 */
int tmc5160_lib_get_sg_result(tmc51x0_lib_handle* handle, uint32_t* sg_result) {
    uint32_t drv_status;
    tmc5160_lib_get_drv_status(handle, &drv_status);

    *sg_result = drv_status & 0x000003FF;

    return 0;
}

/**
 * @brief   tmc5160 set SGT
 * @param   sgt value
 * @note    used in RAMP generator mode
 * @retval  0=succee  ;1=failed
 */
int tmc5160_lib_set_coolconf(tmc51x0_lib_handle* handle, uint32_t coolconf) {
    tmc51x0_lib_lock(handle);

    // write back
    tmc51x0_lib_write_reg(handle, R_TMC51x0_COOLCONF, coolconf);

    tmc51x0_lib_unlock(handle);
    return 0;
}

/**
 * @brief   tmc51x0 en sg_stop
 * @param   mode:is enable sg_stop
 * @note    used in RAMP generator mode
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_set_sg_stop(tmc51x0_lib_handle* handle, uint8_t mode) {
    uint32_t tmp;
    tmc51x0_lib_lock(handle);
    // read
    tmc51x0_lib_read_reg(handle, R_TMC51x0_SW_MODE, &tmp);
    tmp &= ~(0x00000400);
    tmp |= (!!mode) << 10;
    // write back
    tmc51x0_lib_write_reg(handle, R_TMC51x0_SW_MODE, tmp);
    tmc51x0_lib_unlock(handle);
    return 0;
}

/**
 * @brief   tmc51x0 set ramp v1
 * @param
 * @note    used in RAMP generator mode
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_set_ramp_v1(tmc51x0_lib_handle* handle, uint32_t v1) {
    tmc51x0_lib_lock(handle);
    // read
    tmc51x0_lib_write_reg(handle, R_TMC51x0_V1, v1);
    tmc51x0_lib_unlock(handle);
    return 0;
}
/**
 * @brief   tmc51x0 set ramp data
 * @param
 * @note    used in RAMP generator mode, only for a1,amax,damx,d1
 *          It is recommended to use tmc51x0_lib_config_ramp instead
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_set_ramp_data(tmc51x0_lib_handle* handle, uint32_t a1, uint32_t amax, uint32_t dmax,
                              uint32_t d1) {
    tmc51x0_lib_lock(handle);

    tmc51x0_lib_write_reg(handle, R_TMC51x0_A1, a1);
    tmc51x0_lib_write_reg(handle, R_TMC51x0_AMAX, amax);
    tmc51x0_lib_write_reg(handle, R_TMC51x0_DMAX, dmax);
    tmc51x0_lib_write_reg(handle, R_TMC51x0_D1, d1);

    tmc51x0_lib_unlock(handle);

    return 0;
}
/**
 * @brief   tmc51x0 set tcoolthrs
 * @param
 * @note    used in RAMP generator mode
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_set_tcoolthrs(tmc51x0_lib_handle* handle, uint32_t tcoolthrs) {
    tmc51x0_lib_lock(handle);
    // read
    tmc51x0_lib_write_reg(handle, R_TMC51x0_TCOOLTHRS, tcoolthrs);
    tmc51x0_lib_unlock(handle);
    return 0;
}
/**
 * @brief   tmc51x0 set chopper config
 * @param
 * @note
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_set_chopconf(tmc51x0_lib_handle* handle, uint32_t chopconf) {
    tmc51x0_lib_lock(handle);
    tmc51x0_lib_write_reg(handle, R_TMC51x0_CHOPCONF, chopconf);
    tmc51x0_lib_unlock(handle);
    return 0;
}
/**
 * @brief   tmc51x0 set General Configuration Registers
 * @param
 * @note
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_set_gconf(tmc51x0_lib_handle* handle, uint32_t gconf) {
    tmc51x0_lib_lock(handle);
    tmc51x0_lib_write_reg(handle, R_TMC51x0_GCONF, gconf);
    tmc51x0_lib_unlock(handle);
    return 0;
}
/**
 * @brief   tmc51x0 set  TPWMTHRS
 * @param
 * @note
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_set_tpwmthrs(tmc51x0_lib_handle* handle, uint32_t tpwmthrs) {
    tmc51x0_lib_lock(handle);
    tmc51x0_lib_write_reg(handle, R_TMC51x0_TPWMTHRS, tpwmthrs);
    tmc51x0_lib_unlock(handle);
    return 0;
}
/**
 * @brief   tmc51x0 set  THIGH
 * @param
 * @note
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_set_thigh(tmc51x0_lib_handle* handle, uint32_t thigh) {
    tmc51x0_lib_lock(handle);
    tmc51x0_lib_write_reg(handle, R_TMC51x0_THIGH, thigh);
    tmc51x0_lib_unlock(handle);
    return 0;
}

/**
 * @brief   tmc51x0 set  IHOLD_IRUN register
 * @param
 * @note
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_set_ihold_irun(tmc51x0_lib_handle* handle, uint32_t ihold_irun) {
    tmc51x0_lib_lock(handle);
    tmc51x0_lib_write_reg(handle, R_TMC51x0_HOLD_IRUN, ihold_irun);
    tmc51x0_lib_unlock(handle);
    return 0;
}

/**
 * @brief   tmc51x0 set  VDCMIN
 * @param
 * @note
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_set_vdcmin(tmc51x0_lib_handle* handle, uint32_t vdcmin) {
    tmc51x0_lib_lock(handle);
    tmc51x0_lib_write_reg(handle, R_TMC51x0_VDCMIN, vdcmin);
    tmc51x0_lib_unlock(handle);
    return 0;
}
/**
 * @brief   tmc51x0 set  DCCTRL
 * @param
 * @note
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_set_dcctrl(tmc51x0_lib_handle* handle, uint32_t dcctrl) {
    tmc51x0_lib_lock(handle);
    tmc51x0_lib_write_reg(handle, R_TMC51x0_DCCTRL, dcctrl);
    tmc51x0_lib_unlock(handle);
    return 0;
}

/**
 * @brief   tmc51x0 get GSTAT/global status
 * @param
 * @note
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_get_gstat(tmc51x0_lib_handle* handle, uint32_t* gstat) {
    tmc51x0_lib_lock(handle);

    tmc51x0_lib_read_reg(handle, R_TMC51x0_GSTAT, gstat);

    tmc51x0_lib_unlock(handle);

    return 0;
}

/**
 * @brief   tmc51x0 clear GSTAT/global status
 * @param
 * @note
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_clear_gstat(tmc51x0_lib_handle* handle, uint32_t clear_bit) {
    tmc51x0_lib_lock(handle);

    tmc51x0_lib_write_reg(handle, R_TMC51x0_GSTAT, clear_bit);

    tmc51x0_lib_unlock(handle);

    return 0;
}

/**
 * @brief   tmc51x0 clear RAMP_STAT
 * @param
 * @note
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_clear_ramp_stat(tmc51x0_lib_handle* handle, uint32_t clear_bit) {
    tmc51x0_lib_lock(handle);

    tmc51x0_lib_write_reg(handle, R_TMC51x0_RAMP_STAT, clear_bit);

    tmc51x0_lib_unlock(handle);

    return 0;
}

/**
 * @brief   tmc5160 is sg_stop event happend
 * @param
 * @note
 * @retval  0=succee  ;1=happend
 */
int tmc51x0_lib_is_sg_stop(tmc51x0_lib_handle* handle) {
    uint32_t tmp;
    // lock
    tmc51x0_lib_lock(handle);
    tmc51x0_lib_read_reg(handle, R_TMC51x0_RAMP_STAT, &tmp);
    // unlock
    tmc51x0_lib_unlock(handle);
    tmp &= 0x00000040;

    return !!tmp;
}

/**
 * @brief   tmc51x0 config ramp
 * @param
 * @note    used in RAMP generator mode,
 *          use tmc51x0_lib_ramp_config  to config parameters.
 *					All of these parameters are unsigned and do not exceed 2^23,
 *          so they are stored using int, with -1 indicating that the
 *          parameter should not be modified.
 * @retval  0=succee  ;1=failed
 */
int tmc51x0_lib_config_ramp(tmc51x0_lib_handle* handle, tmc51x0_lib_ramp_config* config) {
    uint32_t tmp = 0;

    if ((handle == 0) || (config == 0)) {
        return 1;
    }

    // lock
    tmc51x0_lib_lock(handle);

    if (config->vstart >= 0) {
        tmc51x0_lib_write_reg(handle, R_TMC51x0_VSTART, config->vstart);
    }
    if (config->a1 >= 0) {
        tmc51x0_lib_write_reg(handle, R_TMC51x0_A1, config->a1);
    }
    if (config->v1 >= 0) {
        tmc51x0_lib_write_reg(handle, R_TMC51x0_V1, config->v1);
    }
    if (config->amax >= 0) {
        tmc51x0_lib_write_reg(handle, R_TMC51x0_AMAX, config->amax);
    }
    if (config->vmax >= 0) {
        tmc51x0_lib_write_reg(handle, R_TMC51x0_VMAX, config->vmax);
    }
    if (config->dmax >= 0) {
        tmc51x0_lib_write_reg(handle, R_TMC51x0_DMAX, config->dmax);
    }
    if (config->d1 >= 0) {
        tmc51x0_lib_write_reg(handle, R_TMC51x0_D1, config->d1);
    }
    if (config->vstop >= 0) {
        tmc51x0_lib_write_reg(handle, R_TMC51x0_VSTOP, config->vstop);
    }

    // unlock
    tmc51x0_lib_unlock(handle);
    return !!tmp;
}
