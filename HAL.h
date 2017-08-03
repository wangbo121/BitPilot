/*
 * HAL.h
 *
 *  Created on: 2017-8-3
 *      Author: wangbo
 */

#ifndef HAL_H_
#define HAL_H_

#include "BIT_HAL_Namespace.h"

#include "RCInput.h"

class AP_HAL::HAL {
public:
    HAL( AP_HAL::RCInput*    _rcin)
        :
        rcin(_rcin)
    {

        //AP_HAL::init();
    }

    struct Callbacks {
        virtual void setup() = 0;
        virtual void loop() = 0;
    };

    struct FunCallbacks : public Callbacks {
        FunCallbacks(void (*setup_fun)(void), void (*loop_fun)(void));

        void setup() override { _setup(); }
        void loop() override { _loop(); }

    private:
        void (*_setup)(void);
        void (*_loop)(void);
    };

    virtual void run(int argc, char * const argv[], Callbacks* callbacks) const = 0;


    AP_HAL::RCInput*    rcin;

};



#endif /* HAL_H_ */
