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
#include "RCOutput.h"

class BIT_HAL::HAL {
public:
    HAL( BIT_HAL::RCInput*    _rcin,
    		BIT_HAL::RCOutput*    _rcout)
        :
        rcin(_rcin),
		rcout(_rcout)
    {

        //BIT_HAL::init();
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


    BIT_HAL::RCInput*    rcin;
    BIT_HAL::RCOutput*    rcout;

};



#endif /* HAL_H_ */
