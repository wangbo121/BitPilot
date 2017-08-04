/*
 * HAL.cpp
 *
 *  Created on: 2017-8-3
 *      Author: wangbo
 */



#include <assert.h>

#include "HAL.h"
#include "RCInput.h"
#include "RCOutput.h"

namespace AP_HAL {
#if 0
HAL::FunCallbacks::FunCallbacks(void (*setup_fun)(void), void (*loop_fun)(void))
    : _setup(setup_fun)
    , _loop(loop_fun)
{
    assert(setup_fun);
    assert(loop_fun);
}
#endif

#if 0
void HAL::FunCallbacks::setup()
{

}
void HAL::FunCallbacks::loop()
{

}
#endif

}

AP_HAL::RCInput rcin_driver;
AP_HAL::RCOutput rcout_driver;


static AP_HAL::HAL halInstance(&rcin_driver,&rcout_driver);

const AP_HAL::HAL& AP_HAL::get_HAL() {
    //static const AP_HAL::HAL hal;
    return halInstance;
}

void AP_HAL::HAL::run(int argc, char * const argv[], Callbacks* callbacks) const
{
	callbacks->setup();
	while(1)
	{
		callbacks->loop();
	}


}
