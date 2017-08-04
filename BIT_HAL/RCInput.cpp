/*
 * RCInput.cpp
 *
 *  Created on: 2017-8-4
 *      Author: wangbo
 */


#include <inttypes.h>

#include "HAL.h"


 void BIT_HAL::RCInput::init()
 {

 }
    void BIT_HAL::RCInput::teardown()
    {

    }

    /**
     * Return true if there has been new input since the last call to new_input()
     */
      bool BIT_HAL::RCInput::new_input(void)
      {

      }

    /**
     * Return the number of valid channels in the last read
     */
      uint8_t  BIT_HAL::RCInput::num_channels()
      {

      }

    /* Read a single channel at a time */
      uint16_t BIT_HAL::RCInput::read(uint8_t ch)
      {

      }

    /* Read an array of channels, return the valid count */
      uint8_t BIT_HAL::RCInput::read(uint16_t* periods, uint8_t len)
      {

      }

    /* get receiver based RSSI if available. -1 for unknown, 0 for no link, 255 for maximum link */
      int16_t BIT_HAL::RCInput::get_rssi(void)
      {
    	  return -1;
      }

    /**
     * Overrides: these are really grody and don't belong here but we need
     * them at the moment to make the port work.
     * case v of:
     *  v == -1 -> no change to this channel
     *  v == 0  -> do not override this channel
     *  v > 0   -> set v as override.
     */

    /* set_overrides: array starts at ch 0, for len channels */
      //bool set_overrides(int16_t *overrides, uint8_t len) = 0;
    /* set_override: set just a specific channel */
      //bool set_override(uint8_t channel, int16_t override) = 0;
    /* clear_overrides: equivalent to setting all overrides to 0 */
      //void clear_overrides() = 0;

    /* execute receiver bind */
      bool BIT_HAL::RCInput::rc_bind(int dsmMode)
      {
    	  return false;
      }


