/*
 * RCOutput.cpp
 *
 *  Created on: 2017-8-4
 *      Author: wangbo
 */

#include <inttypes.h>

#include "HAL.h"

  void BIT_HAL::RCOutput::init()
  {

  }

    /* Output freq (1/period) control */
     void     BIT_HAL::RCOutput:: set_freq(uint32_t chmask, uint16_t freq_hz)
     {

     }

     uint16_t  BIT_HAL::RCOutput::get_freq(uint8_t ch)
     {

     }

    /* Output active/highZ control, either by single channel at a time
     * or a mask of channels */
     void     BIT_HAL::RCOutput:: enable_ch(uint8_t ch)
     {

     }

     void     BIT_HAL::RCOutput:: disable_ch(uint8_t ch)
     {

     }

    /*
     * Output a single channel, possibly grouped with previous writes if
     * cork() has been called before.
     */
     void     BIT_HAL::RCOutput:: write(uint8_t ch, uint16_t period_us)
     {

     }

    /*
     * Delay subsequent calls to write() going to the underlying hardware in
     * order to group related writes together. When all the needed writes are
     * done, call push() to commit the changes.
     */
     void      BIT_HAL::RCOutput::cork()
     {

     }

    /*
     * Push pending changes to the underlying hardware. All changes between a
     * call to cork() and push() are pushed together in a single transaction.
     */
     void      BIT_HAL::RCOutput::push()
     {

     }

    /* Read back current output state, as either single channel or
     * array of channels. On boards that have a separate IO controller,
     * this returns the latest output value that the IO controller has
     * reported */
     uint16_t  BIT_HAL::RCOutput::read(uint8_t ch)
     {

     }
     void      BIT_HAL::RCOutput::read(uint16_t* period_us, uint8_t len)
     {

     }

    /* Read the current input state. This returns the last value that was written. */
     uint16_t  BIT_HAL::RCOutput::read_last_sent(uint8_t ch)
     {

     }
     void      BIT_HAL::RCOutput::read_last_sent(uint16_t* period_us, uint8_t len)
     {

     }

    /*
      set PWM to send to a set of channels when the safety switch is
      in the safe state
     */
     void      BIT_HAL::RCOutput::set_safety_pwm(uint32_t chmask, uint16_t period_us)
     {

     }

    /*
      set PWM to send to a set of channels if the FMU firmware dies
     */
     void      BIT_HAL::RCOutput::set_failsafe_pwm(uint32_t chmask, uint16_t period_us) {}

    /*
      force the safety switch on, disabling PWM output from the IO board
      return false (indicating failure) by default so that boards with no safety switch
      do not need to implement this method
     */
     bool      BIT_HAL::RCOutput::force_safety_on(void) { return false; }

    /*
      force the safety switch off, enabling PWM output from the IO board
     */
     void    BIT_HAL::RCOutput::  force_safety_off(void) {}

    /*
      If we support async sends (px4), this will force it to be serviced immediately
     */
     void      BIT_HAL::RCOutput::force_safety_no_wait(void) {}

    /*
      setup scaling of ESC output for ESCs that can output a
      percentage of power (such as UAVCAN ESCs). The values are in
      microseconds, and represent minimum and maximum PWM values which
      will be used to convert channel writes into a percentage
     */
     void      BIT_HAL::RCOutput::set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm) {}

    /*
      returns the pwm value scaled to [-1;1] regrading to set_esc_scaling ranges range without constraints.
     */
     float    BIT_HAL::RCOutput:: scale_esc_to_unity(uint16_t pwm) { return 0; }

    /*
      enable SBUS out at the given rate
     */
     bool      BIT_HAL::RCOutput::enable_sbus_out(uint16_t rate_gz) { return false; }

    /*
     * Optional method to control the update of the motors. Derived classes
     * can implement it if their HAL layer requires.
     */
     void  BIT_HAL::RCOutput::timer_tick(void) { }


     void     BIT_HAL::RCOutput::set_output_mode(enum output_mode mode) {}
