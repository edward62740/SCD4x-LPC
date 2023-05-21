/*
 * scd4x_lpc.h
 *
 *  Created on: May 12, 2023
 *      Author: edward62740
 */

#ifndef SENSORS_SCD4X_SCD4X_LPC_H_
#define SENSORS_SCD4X_SCD4X_LPC_H_

#include "stdint.h"
#include <functional>
#include <iostream>
#include <queue>
#include <string_view>
#include <vector>

const uint16_t SCD4X_LPC_BASELINE_CO2_PPM = 400;
const uint32_t SCD4X_LPC_FIRST_CAL_TRIG_CNT = 1;
const uint32_t SCD4X_LPC_INIT_CAL_TRIG_CNT = 576;
const uint32_t SCD4X_LPC_STD_CAL_TRIG_CNT = 2016;

const uint32_t SCD4X_LPC_FIRST_MAX_CAL_OFFSET = 10000;
const uint32_t SCD4X_LPC_INIT_MAX_CAL_OFFSET = 10000;
const uint32_t SCD4X_LPC_STD_MAX_CAL_OFFSET = 100;

const uint32_t SCD4X_LPC_SCD_FAIL_TOL_TH = 50;

const uint32_t SCD4X_LPC_CMD_MAX_LOCK_DUR_MS = 10000;
const uint32_t SCD4X_LPC_CMD_WU_LOCK_DUR_MS = 20;
const uint32_t SCD4X_LPC_CMD_PD_LOCK_DUR_MS = 1;
const uint32_t SCD4X_LPC_CMD_MEAS_LOCK_DUR_MS = 5000;
const uint32_t SCD4X_LPC_CMD_DRDY_LOCK_DUR_MS = 0;
const uint32_t SCD4X_LPC_CMD_READ_LOCK_DUR_MS = 0;
const uint32_t SCD4X_LPC_CMD_FRC_LOCK_DUR_MS = 0;
const uint32_t SCD4X_LPC_CMD_PERSIST_LOCK_DUR_MS = 800;

namespace LPC {

  typedef enum {
    ERR_NONE, ERR_SCD, ERR_LPC, ERR_WAIT, ERR_PANIC, WARN_PRETRIG
  } lpc_ret;

  class SCD4X
  {
  public:
    SCD4X(int16_t (*wu)(void), int16_t (*pd)(void), int16_t (*meas)(void),
          int16_t (*drdy)(bool *ready), int16_t (*read)(uint16_t* co2, int32_t* temp, int32_t* hum),
          int16_t (*frc)(uint16_t target, uint16_t* corr), int16_t (*persist)(void),
		  uint32_t (*getMsTick)(void));

    lpc_ret powerOn(void);
    lpc_ret powerOff(void);


    /**
      * @brief  This function is called after startup to perform a sensor read.
      * 		The reading is invalid and nothing is returned.
      * 		No calls to this API for 5000ms after this should be made.
      * @retval lpc_ret error code
      */
    lpc_ret discardMeasurement(void);



    lpc_ret measure(void);

    /**
      * @brief  This function is called at least 5000ms after measure() and returns
      * 		sensor measurement data and performs FRC if applicable.
      *
      *
      * @param  co2,temp,hum pointers to store sensor data
      * @param  yield pointer to app function
      * @retval lpc_ret error code
      */
    lpc_ret read(uint16_t *co2, int32_t *temp, int32_t *hum);

    void get_last_frc(int32_t *offset, uint32_t *age, uint32_t *num);

    lpc_ret sig_ext_reset_fsm(void);
  private:

    /* Struct of function ptrs to sensor driver */
    struct
    {
      int16_t (*wu)( void );
      int16_t (*pd)( void );
      int16_t (*meas)( void );
      int16_t (*drdy)(bool *ready );
      int16_t (*read)(uint16_t* co2, int32_t* temp, int32_t* hum);
      int16_t (*frc)(uint16_t target, uint16_t* corr);
      int16_t (*persist)( void );
      uint32_t (*getMsTick)(void);
    } scd_fp;

    typedef enum
	{
    	STATE_FIRST,
		STATE_INIT,
		STATE_STD,
	} e_state_t;

    /* Core FSM struct */
	struct {
		e_state_t state; // current state
		int32_t ttl; // num measurements left in cycle
		uint16_t gl_min; // current cycle global minimum
		bool expend; // flag to indicate early frc
		bool limit; // flag to indicate frc offset limited by _sig_get_fsm_offset
		uint32_t _cons_fail;
	} fsm;

	struct  {
	    bool state = false;
	    uint32_t lastTickMs = 0;
	    uint32_t reqLockDurMs = 0;
	} _mutex;
    bool _mutex_request_lock(uint32_t dur);
    bool _mutex_request_unlock(void);

	lpc_ret _proc_fsm(uint16_t co2);
	void _sig_fsm_next(void);
	uint32_t _sig_get_fsm_offset(void);
	uint32_t _sig_get_fsm_cnt(void);
	lpc_ret _sig_scd_fail(bool trig);
    uint32_t frc_ctr = 0; // number of frcs done
    int32_t prev_frc_offset = 0; // last offset (signed)
    uint32_t prev_frc_age = 0;

  };
}

#endif /* SENSORS_SCD4X_SCD4X_LPC_H_ */
