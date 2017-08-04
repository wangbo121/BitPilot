/*
 * RCOutput.h
 *
 *  Created on: 2017-8-4
 *      Author: wangbo
 */

#ifndef RCOUTPUT_H_
#define RCOUTPUT_H_

#include "BIT_HAL_Namespace.h"

#define RC_OUTPUT_MIN_PULSEWIDTH 400
#define RC_OUTPUT_MAX_PULSEWIDTH 2100

/* Define the CH_n names, indexed from 1#include "BIT_HAL_Namespace.h", if we don't have them already */
#ifndef CH_1
#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7
#define CH_9 8
#define CH_10 9
#define CH_11 10
#define CH_12 11
#define CH_13 12
#define CH_14 13
#define CH_15 14
#define CH_16 15
#define CH_17 16
#define CH_18 17
#define CH_NONE 255
#endif


class BIT_HAL::RCOutput {
public:
     void init() ;

    /* Output freq (1/period) control */
     void     set_freq(uint32_t chmask, uint16_t freq_hz) ;
     uint16_t get_freq(uint8_t ch) ;

    /* Output active/highZ control, either by single channel at a time
     * or a mask of channels */
     void     enable_ch(uint8_t ch) ;
     void     disable_ch(uint8_t ch) ;

    /*
     * Output a single channel, possibly grouped with previous writes if
     * cork() has been called before.
     */
     void     write(uint8_t ch, uint16_t period_us) ;

    /*
     * Delay subsequent calls to write() going to the underlying hardware in
     * order to group related writes together. When all the needed writes are
     * done, call push() to commit the changes.
     */
     void     cork() ;

    /*
     * Push pending changes to the underlying hardware. All changes between a
     * call to cork() and push() are pushed together in a single transaction.
     */
     void     push() ;

    /* Read back current output state, as either single channel or
     * array of channels. On boards that have a separate IO controller,
     * this returns the latest output value that the IO controller has
     * reported */
     uint16_t read(uint8_t ch) ;
     void     read(uint16_t* period_us, uint8_t len) ;

    /* Read the current input state. This returns the last value that was written. */
     uint16_t read_last_sent(uint8_t ch);
     void     read_last_sent(uint16_t* period_us, uint8_t len) ;

    /*
      set PWM to send to a set of channels when the safety switch is
      in the safe state
     */
     void     set_safety_pwm(uint32_t chmask, uint16_t period_us);

    /*
      set PWM to send to a set of channels if the FMU firmware dies
     */
     void     set_failsafe_pwm(uint32_t chmask, uint16_t period_us) ;

    /*
      force the safety switch on, disabling PWM output from the IO board
      return false (indicating failure) by default so that boards with no safety switch
      do not need to implement this method
     */
     bool     force_safety_on(void) ;

    /*
      force the safety switch off, enabling PWM output from the IO board
     */
     void     force_safety_off(void) ;

    /*
      If we support async sends (px4), this will force it to be serviced immediately
     */
     void     force_safety_no_wait(void) ;

    /*
      setup scaling of ESC output for ESCs that can output a
      percentage of power (such as UAVCAN ESCs). The values are in
      microseconds, and represent minimum and maximum PWM values which
      will be used to convert channel writes into a percentage
     */
     void     set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm) ;

    /*
      returns the pwm value scaled to [-1;1] regrading to set_esc_scaling ranges range without constraints.
     */
     float    scale_esc_to_unity(uint16_t pwm) ;

    /*
      enable SBUS out at the given rate
     */
     bool     enable_sbus_out(uint16_t rate_gz) ;

    /*
     * Optional method to control the update of the motors. Derived classes
     * can implement it if their HAL layer requires.
     */
     void timer_tick(void) ;

    /*
      output modes. Allows for support of oneshot
     */
    enum output_mode {
        MODE_PWM_NORMAL,
        MODE_PWM_ONESHOT,
        MODE_PWM_BRUSHED
    };
     void    set_output_mode(enum output_mode mode);
};




#endif /* RCOUTPUT_H_ */
