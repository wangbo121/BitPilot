/*
 * AP_HAL.h
 *
 *  Created on: 2017-8-3
 *      Author: wangbo
 */

#ifndef AP_HAL_H_
#define AP_HAL_H_

/*
 * 20170803这个文件是AP_HAL的命名空间
 * 我先测试RCInput 所以其他的先删除掉好了
 * 先有AP_HAL_Namespace.h，再有HAL.h，再有AP_HAL.h所以，
 * 在其他函数想包含hal时，要包含AP_HAL.h文件
 */
#include "BIT_HAL_Namespace.h"

/* HAL Class definition */
#include "HAL.h"

#include "RCInput.h"





#endif /* AP_HAL_H_ */
