/************************************************** 
 * @Author: shuren
 * @Date: 2022-02-17 16:06:48
 * @LastEditTime: 2022-02-18 15:23:25
 * @LastEditors: shuren
 * @Description: 
 * @FilePath: \GD32F303\code\sys_app\base_typle.h
 * @桃之夭夭，灼灼其华。之子于归， 宜其室家。
 **************************************************/

#ifndef _BASE_TYPLE_H_
#define _BASE_TYPLE_H_

#include <stdint.h>
#include <stdbool.h>
#include <float.h>

#ifndef boolean_t
    typedef bool boolean_t;
#endif
#ifndef TRUE
    #define TRUE (boolean_t)1
#endif
#ifndef FALSE
    #define FALSE (boolean_t)0
#endif
#ifndef ON
    #define ON (boolean_t)1
#endif
#ifndef OFF
    #define OFF (boolean_t)0
#endif
#ifndef float32_t
    typedef float float32_t;
#endif
#ifndef float64_t
    typedef double float64_t;
#endif

#endif
