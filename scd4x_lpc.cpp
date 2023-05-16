/*
 * scd4x_lpc.cpp
 *
 *  Created on: May 12, 2023
 *      Author: edward62740
 */
#include "scd4x_lpc.h"
#include "scd4x_i2c.h"

namespace LPC {

  SCD4X::SCD4X(int16_t (*wu)(void), int16_t (*pd)(void), int16_t (*meas)(void),
                int16_t (*drdy)(bool *ready), int16_t (*read)(uint16_t* co2, int32_t* temp, int32_t* hum),
                int16_t (*frc)(uint16_t target, uint16_t* corr), int16_t (*persist)(void))
  {
    scd_fp.wu = wu;
    scd_fp.pd = pd;
    scd_fp.meas = meas;
    scd_fp.drdy = drdy;
    scd_fp.read = read;
    scd_fp.frc = frc;
    scd_fp.persist = persist;

    fsm.state = STATE_FIRST;
    fsm.expend = false;
    fsm.gl_min = 0xFFFF;
    fsm.ttl = _sig_get_fsm_cnt();
    fsm.limit = false;
    fsm._cons_fail = 0;

  }

lpc_ret SCD4X::sig_ext_reset_fsm(void) {
	fsm.state = STATE_FIRST;
	fsm.expend = false;
	fsm.gl_min = 0xFFFF;
	fsm.ttl = _sig_get_fsm_cnt();
	fsm.limit = false;
	fsm._cons_fail = 0;
	return ERR_NONE;
}

void SCD4X::get_last_frc(int32_t *offset, uint32_t *age, uint32_t *num)
{
	*offset = prev_frc_offset;
	*age = prev_frc_age;
	*num = frc_ctr;
}

lpc_ret SCD4X::powerOn(void) {
	if (scd_fp.wu())
		return _sig_scd_fail(true);
	_sig_scd_fail(false);
	return ERR_NONE;
}

lpc_ret SCD4X::powerOff(void) {
	if (scd_fp.pd())
		return _sig_scd_fail(true);
	_sig_scd_fail(false);
	return ERR_NONE;
}

lpc_ret SCD4X::discardMeasurement(void) {
	if (scd_fp.meas())
		return _sig_scd_fail(true);
	_sig_scd_fail(false);
	return ERR_NONE;
}

lpc_ret SCD4X::measure()
  {

	if (scd_fp.meas())
		return _sig_scd_fail(true);
	if (fsm.ttl <= 1)
		return WARN_PRETRIG; // warn if yield fn should be passed
	_sig_scd_fail(false);
	return ERR_NONE;

  }

  lpc_ret SCD4X::read(uint16_t *co2, int32_t *temp, int32_t *hum, void (*yield)(void))
  {
    bool tmp = false;
    scd_fp.drdy(&tmp);
    if(!tmp) return ERR_WAIT;

    if(scd_fp.read(co2, temp, hum)) return _sig_scd_fail(true);
    if(*co2 == 0) return _sig_scd_fail(true);
	_sig_scd_fail(false);
    if(*co2 < fsm.gl_min) fsm.gl_min = *co2;
    prev_frc_age++;
    return _proc_fsm(*co2, yield);

  }

  lpc_ret SCD4X::_proc_fsm(uint16_t co2, void (*yield)(void))
  {

	if(fsm._cons_fail > SCD4X_LPC_SCD_FAIL_TOL_TH) return ERR_PANIC;
	if (co2 < SCD4X_LPC_BASELINE_CO2_PPM) // called if co2 < 400
			{
		fsm.expend = true;
		uint32_t offset = SCD4X_LPC_BASELINE_CO2_PPM - co2;
		fsm.limit = offset > _sig_get_fsm_offset();
		uint16_t ret = 0xFFFF;
		scd_fp.frc(fsm.limit ?co2+_sig_get_fsm_offset() : SCD4X_LPC_BASELINE_CO2_PPM, &ret);
		frc_ctr++;
		prev_frc_offset = ret - 0x8000U;
		if (yield != NULL)
			yield(); //yield to app function ptr
		if (ret == 0xFFFF)
			return ERR_LPC; // return on next calls, ttl will be -ve
	}
	if (--fsm.ttl <= 0) {

		_sig_fsm_next();
		fsm.ttl = _sig_get_fsm_cnt();
		fsm.gl_min = 0xFFFF;

		if (fsm.expend) {
			fsm.expend = false;
			return ERR_NONE;
		} // do not frc again if already done

		// called iff co2 >= 400 for the entire period
		uint32_t offset = fsm.gl_min - SCD4X_LPC_BASELINE_CO2_PPM;
		fsm.limit = offset > _sig_get_fsm_offset();
		uint16_t ret = 0xFFFF;
		scd_fp.frc(co2 - (fsm.limit ? _sig_get_fsm_offset() : offset),
				&ret);
		frc_ctr++;
		prev_frc_offset = ret - 0x8000U;
		prev_frc_age = 0;
		if (yield != NULL)
			yield(); //yield to app function ptr
		if (ret == 0xFFFF)
			return ERR_LPC; // return on next calls, ttl will be -ve
	}

	return ERR_NONE;
  }

void SCD4X::_sig_fsm_next(void) {
	switch (fsm.state) {
	case STATE_FIRST: {
		fsm.state = STATE_INIT;
		break;
	}
	case STATE_INIT: {
		fsm.state = STATE_STD;
		break;
	}
	case STATE_STD: {
		fsm.state = STATE_STD;
		break;
	}
	}
}

uint32_t SCD4X::_sig_get_fsm_offset(void) {
	switch (fsm.state) {
	case STATE_FIRST:
		return SCD4X_LPC_FIRST_MAX_CAL_OFFSET;
	case STATE_INIT:
		return SCD4X_LPC_INIT_MAX_CAL_OFFSET;
	case STATE_STD:
		return SCD4X_LPC_STD_MAX_CAL_OFFSET;
	}
	return 0;
}

uint32_t SCD4X::_sig_get_fsm_cnt(void) {
	switch (fsm.state) {
	case STATE_FIRST:
		return SCD4X_LPC_FIRST_CAL_TRIG_CNT;
	case STATE_INIT:
		return SCD4X_LPC_INIT_CAL_TRIG_CNT;
	case STATE_STD:
		return SCD4X_LPC_STD_CAL_TRIG_CNT;
	}
	return 0;
}


lpc_ret SCD4X::_sig_scd_fail(bool trig)
{
	if(!trig) fsm._cons_fail = 0;
	else fsm._cons_fail++;
	return ERR_SCD;
}
}
