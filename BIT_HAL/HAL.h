/*
 * HAL.h
 *
 *  Created on: 2017-8-3
 *      Author: wangbo
 */

#ifndef HAL_H_
#define HAL_H_

#include "BIT_HAL_Namespace.h"
#include "BIT_HAL.h"

class AP_HAL::HAL {
public:
    HAL( AP_HAL::RCInput*    _rcin,
    		AP_HAL::RCOutput*    _rcout,
    		AP_HAL::Scheduler * _scheduler)
        :
        rcin(_rcin),
		rcout(_rcout),
		scheduler(_scheduler)
    {

        //AP_HAL::init();
    }

    struct Callbacks {
        virtual void setup() = 0;
        virtual void loop() = 0;
    };
#if 0
    struct FunCallbacks : public Callbacks {
        FunCallbacks(void (*setup_fun)(void), void (*loop_fun)(void));

        //void setup() override { _setup(); }
        //void loop() override { _loop(); }
        virtual void setup();
        virtual void loop() ;

    private:
        void (*_setup)(void);
        void (*_loop)(void);
    };
#endif

    //virtual void run(int argc, char * const argv[], Callbacks* callbacks) const = 0;
    void run(int argc, char * const argv[], Callbacks* callbacks)  const ;


    AP_HAL::RCInput*    rcin;
    AP_HAL::RCOutput*    rcout;
    AP_HAL::Scheduler * scheduler;

};



#endif /* HAL_H_ */
