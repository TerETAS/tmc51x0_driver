/***************************************************************************
 * Copyright  2023 TerETAS
 * All right reserved. See COPYRIGHT for detailed Information.
 *
 * @file       tmc51x0_lib_register_address.h
 *
 * @author     TerETAS
 * @brief      tmc51x0 dirver lib
 *
 * @date       2024/03/26
 ***************************************************************************/
#ifndef __TMC5130_LIB_REGISTER_ADDRESS_H__
#define __TMC5130_LIB_REGISTER_ADDRESS_H__

typedef enum {
    // General Configuration Registers
    R_TMC51x0_GCONF = 0x00,
    R_TMC51x0_GSTAT,
    R_TMC51x0_IFCNT,
    R_TMC51x0_NODECONF,
    R_TMC51x0_IOIN_OUTPUT,
    R_TMC51x0_X_COMPARE = 0x05,
    R_TMC51x0_OTP_PROG,
    R_TMC51x0_OTP_READ,
    R_TMC51x0_FACTORY_CONF,
    R_TMC51x0_SHORT_CONF,
    R_TMC51x0_DRV_CONF = 0x0A,
    R_TMC51x0_GLOBAL_SCALER,
    R_TMC51x0_OFFSET_READ,
    // Velocity Dependent Driver Feature Control Register Set
    R_TMC51x0_HOLD_IRUN = 0x10,
    R_TMC51x0_TPOWERDOWN,
    R_TMC51x0_TSTEP,
    R_TMC51x0_TPWMTHRS,
    R_TMC51x0_TCOOLTHRS,
    R_TMC51x0_THIGH,
    // Ramp Generator Registers
    R_TMC51x0_RAMPMODE = 0x20,
    R_TMC51x0_XACTUAL,
    R_TMC51x0_VACTUAL,
    R_TMC51x0_VSTART,
    R_TMC51x0_A1,
    R_TMC51x0_V1 = 0x25,
    R_TMC51x0_AMAX,
    R_TMC51x0_VMAX,
    R_TMC51x0_DMAX,
    R_TMC51x0_D1 = 0x2A,
    R_TMC51x0_VSTOP,
    R_TMC51x0_TZEROWAIT,
    R_TMC51x0_XTARGET,
    // Ramp Generator Driver Feature Control Register Set
    R_TMC51x0_VDCMIN = 0x33,
    R_TMC51x0_SW_MODE,
    R_TMC51x0_RAMP_STAT,
    R_TMC51x0_XLATCH,
    // Encoder Registers
    R_TMC51x0_ENCMODE = 0x38,
    R_TMC51x0_X_ENC,
    R_TMC51x0_ENC_CONST,
    R_TMC51x0_ENC_STATUS,
    R_TMC51x0_ENC_LATCH,
    R_TMC51x0_ENC_DEVIATION,
    // Motor Driver Registers
    R_TMC51x0_MSLUT0 = 0x60,
    R_TMC51x0_MSLUT1,
    R_TMC51x0_MSLUT2,
    R_TMC51x0_MSLUT3,
    R_TMC51x0_MSLUT4,
    R_TMC51x0_MSLUT5,
    R_TMC51x0_MSLUT6,
    R_TMC51x0_MSLUT7,
    R_TMC51x0_MSLUTSEL = 0x68,
    R_TMC51x0_MSLUTSTART,
    R_TMC51x0_MSCNT,
    R_TMC51x0_MSCURACT,
    R_TMC51x0_CHOPCONF = 0x6C,
    R_TMC51x0_COOLCONF,
    R_TMC51x0_DCCTRL,
    R_TMC51x0_DRV_STATUS = 0x6F,
    R_TMC51x0_PWMCONF,
    R_TMC51x0_PWM_SCALE,
    R_TMC51x0_PWM_AUTO,
    R_TMC51x0_LOST_STEPS,

} TMC51x0_lib_reg_address;

#endif