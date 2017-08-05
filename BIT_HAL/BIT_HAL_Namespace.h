/*
 * AP_HAL_Namespace.h
 *
 *  Created on: 2017-8-4
 *      Author: wangbo
 */

#pragma once

#ifndef AP_HAL_NAMESPACE_H_
#define AP_HAL_NAMESPACE_H_

namespace AP_HAL {

    /* Toplevel pure virtual class Hal.*/
    class HAL;


    class RCInput;
    class RCOutput;


    // Must be implemented by the concrete HALs.
    const HAL& get_HAL();
}




#endif /* AP_HAL_NAMESPACE_H_ */
