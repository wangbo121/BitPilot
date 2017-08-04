/*
 * BIT_HAL_Namespace.h
 *
 *  Created on: 2017-8-4
 *      Author: wangbo
 */

#pragma once

#ifndef BIT_HAL_NAMESPACE_H_
#define BIT_HAL_NAMESPACE_H_

namespace BIT_HAL {

    /* Toplevel pure virtual class Hal.*/
    class HAL;


    class RCInput;
    class RCOutput;


    // Must be implemented by the concrete HALs.
    const HAL& get_HAL();
}




#endif /* BIT_HAL_NAMESPACE_H_ */
