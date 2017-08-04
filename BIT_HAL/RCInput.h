/*
 * RCInput.h
 *
 *  Created on: 2017-8-4
 *      Author: wangbo
 */

#ifndef RCINPUT_H_
#define RCINPUT_H_

#include <inttypes.h>

#include "BIT_HAL_Namespace.h"

#define RC_INPUT_MIN_PULSEWIDTH 900
#define RC_INPUT_MAX_PULSEWIDTH 2100

class AP_HAL::RCInput {
public:
    /**
     * Call init from the platform hal instance init, so that both the type of
     * the RCInput implementation and init argument (e.g. ISRRegistry) are
     * known to the programmer. (It's too difficult to describe this dependency
     * in the C++ type system.)
     */
    void init() ;
    void teardown();

    /**
     * Return true if there has been new input since the last call to new_input()
     */
      bool new_input(void) ;

    /**
     * Return the number of valid channels in the last read
     */
      uint8_t  num_channels() ;

    /* Read a single channel at a time */
      uint16_t read(uint8_t ch)  ;

    /* Read an array of channels, return the valid count */
      uint8_t read(uint16_t* periods, uint8_t len)  ;

    /* get receiver based RSSI if available. -1 for unknown, 0 for no link, 255 for maximum link */
      int16_t get_rssi(void) ;

    /**
     * Overrides: these are really grody and don't belong here but we need
     * them at the moment to make the port work.
     * case v of:
     *  v == -1 -> no change to this channel
     *  v =   -> do not override this channel
     *  v > 0   -> set v as override.
     */

    /* set_overrides: array starts at ch 0, for len channels */
     // bool set_overrides(int16_t *overrides, uint8_t len)  ;
    /* set_override: set just a specific channel */
      //bool set_override(uint8_t channel, int16_t override)  ;
    /* clear_overrides: equivalent to setting all overrides to 0 */
      //void clear_overrides()  ;

    /* execute receiver bind */
      bool rc_bind(int dsmMode);
};


#endif /* RCINPUT_H_ */
