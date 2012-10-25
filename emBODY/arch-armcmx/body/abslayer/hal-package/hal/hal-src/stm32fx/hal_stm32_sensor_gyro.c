/*
 * Copyright (C) 2012 iCub Facility - Istituto Italiano di Tecnologia
 * Author:  Marco Accame
 * email:   marco.accame@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/* @file       hal_stm32_sensor_gyro.c
	@brief      This file implements internals of the temperature sensor module.
	@author     marco.accame@iit.it
    @date       10/24/2012
**/

// - modules to be built: contains the HAL_USE_* macros ---------------------------------------------------------------
#include "hal_brdcfg_modules.h"

#ifdef HAL_USE_SENSOR_GYRO

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "string.h"
#include "hal_stm32_base_hid.h" 
#include "hal_brdcfg.h"


#include "stdio.h"

#include "hal_stm32xx_include.h"


#include "utils/hal_device_st_l3g4200d.h"

 
// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "hal_sensor_gyro.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "hal_stm32_sensor_gyro_hid.h"


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------

#define HAL_sensor_gyro_sensor2index(t)              ((uint8_t)((t)))



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------

extern const hal_sensor_gyro_cfg_t hal_sensor_gyro_cfg_default  = 
{ 
    .dummy = 0 
};

// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------

typedef struct
{
    hal_sensor_gyro_cfg_t       cfg;
    uint32_t                    initialvalue;
} hal_sensor_gyro_info_t;


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static hal_boolval_t s_hal_sensor_gyro_supported_is(hal_sensor_gyro_t sensor);
static void s_hal_sensor_gyro_initted_set(hal_sensor_gyro_t sensor);
static hal_boolval_t s_hal_sensor_gyro_initted_is(hal_sensor_gyro_t sensor);

static hal_result_t s_hal_sensor_gyro_hw_init(const hal_sensor_gyro_cfg_t *cfg);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static hal_boolval_t s_hal_sensor_gyro_initted[hal_sensor_gyro_number] = { hal_false };

static hal_sensor_gyro_info_t s_hal_sensor_gyro_info[hal_sensor_gyro_number] = { 0 };


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern hal_result_t hal_sensor_gyro_init(hal_sensor_gyro_t sensor, const hal_sensor_gyro_cfg_t *cfg)
{
    hal_result_t res = hal_res_NOK_generic; // dont remove ...
    hal_sensor_gyro_info_t *info = NULL;

    if(hal_false == s_hal_sensor_gyro_supported_is(sensor))
    {
        return(hal_res_NOK_generic);
    }
     
    if(NULL == cfg)
    {
        cfg  = &hal_sensor_gyro_cfg_default;
    }

    if(hal_true == s_hal_sensor_gyro_initted_is(sensor))
    {
        return(hal_res_OK);
    } 
    
    if(hal_res_OK != s_hal_sensor_gyro_hw_init(cfg))
    {
        return(hal_res_NOK_generic);
    }
    
    s_hal_sensor_gyro_initted_set(sensor);

    return(hal_res_OK);
}

static int32_t s_hal_convert(int16_t v)
{
    // each bit is 0.00875 ...
    //  factor is about 8.75 or 35/4
    uint8_t neg = (v < 0) ? (1) : (0);
    int32_t r = (0 == neg) ? (35*v) : (35*(-v));
    // now r is positive
    r >>= 2;
    r = (0 == neg) ? (r) : (-r);
    
    return(r);  
}
    

extern hal_result_t hal_sensor_gyro_read(hal_sensor_gyro_t sensor, hal_sensor_gyro_angular_rate_t* angrate)
{
    hal_result_t res = hal_res_NOK_generic; 
 
    if(NULL == angrate)
    {
        return(hal_res_NOK_generic);
    }
    
    angrate->xar = 0;
    angrate->yar = 0;
    angrate->zar = 0;
    int16_t xar = 0;
    int16_t yar = 0;
    int16_t zar = 0;
       
    res = hal_hal_device_st_l3g4200d_angrate_get(&xar, &yar, &zar);
    
    if(hal_res_OK == res)
    {
 
        angrate->xar = s_hal_convert(xar); //  factor is about 8.75 or 35/4
        angrate->yar = s_hal_convert(yar);
        angrate->zar = s_hal_convert(zar);        
    }
    
    return(res);
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------

// ---- isr of the module: begin ----
// empty-section
// ---- isr of the module: end ------


extern uint32_t hal_sensor_gyro_hid_getsize(const hal_cfg_t *cfg)
{
    // no memory needed
    return(0);
}

extern hal_result_t hal_sensor_gyro_hid_setmem(const hal_cfg_t *cfg, uint32_t *memory)
{
    // no memory needed
//    if(NULL == memory)
//    {
//        hal_base_hid_on_fatalerror(hal_fatalerror_missingmemory, "hal_xxx_hid_setmem(): memory missing");
//        return(hal_res_NOK_generic);
//    }


    memset(s_hal_sensor_gyro_info, 0, sizeof(s_hal_sensor_gyro_info));
    memset(s_hal_sensor_gyro_initted, hal_false, sizeof(s_hal_sensor_gyro_initted));
    return(hal_res_OK);  
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

#warning --> put hal_brdcfg_sensor_gyro__supported_mask elsewhere
extern const uint8_t hal_brdcfg_sensor_gyro__supported_mask = 0x1;

static hal_boolval_t s_hal_sensor_gyro_supported_is(hal_sensor_gyro_t sensor)
{
    return(hal_base_hid_byte_bitcheck(hal_brdcfg_sensor_gyro__supported_mask, HAL_sensor_gyro_sensor2index(sensor)) );
}

static void s_hal_sensor_gyro_initted_set(hal_sensor_gyro_t sensor)
{
    s_hal_sensor_gyro_initted[HAL_sensor_gyro_sensor2index(sensor)] = hal_true;
}

static hal_boolval_t s_hal_sensor_gyro_initted_is(hal_sensor_gyro_t sensor)
{
    return(s_hal_sensor_gyro_initted[HAL_sensor_gyro_sensor2index(sensor)]);
}


static hal_result_t s_hal_sensor_gyro_hw_init(const hal_sensor_gyro_cfg_t *cfg)
{
    return(hal_device_st_l3g4200d_init(NULL));
}


#endif//HAL_USE_SENSOR_GYRO


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



