/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author:  Valentina Gaggero
 * email:   valentina.gaggero@iit.it
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


// --------------------------------------------------------------------------------------------------------------------
// - doxy
// --------------------------------------------------------------------------------------------------------------------

/* @file       EOMappMotorController.c
    @brief      This file contains internal implementation for ..... a SW entity.
    @author     valentina.gaggero@iit.it
    @date       02/20/2012
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------
#include "stdlib.h"     // to see NULL, calloc etc.
#include "string.h"     //memcpy

// abstraction layers
#include "hal.h"

//embobj
#include "EOtheMemoryPool.h"
#include "EOtheErrorManager.h"
#include "EOemsController.h"
//#include "EOMtask_hid.h"

// application
#include "EOMappDataTransmitter.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------
#include "EOMappMotorController.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
#include "EOMappMotorController_hid.h" 




// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
#define EVT_CHECK(var, EVTMASK)             (EVTMASK == (var&EVTMASK))



#define TASK_MOTORCONTROLLER_PRIO          70   // task piority

//************************ system controller task events **************************************************
#define EVT_CALC_START                 (1 << 0) /* this evt is generated by syscontroller timer */
#define EVT_DATACOLLECTOR_FINISHED     (1 << 1) /* this evt is generated by data collector has finished */


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables. deprecated: better using _get(), _set() on static variables 
// --------------------------------------------------------------------------------------------------------------------
int16_t pwm_out = 0;
int32_t encoder_can = 0;
int32_t posref_can = 0;

// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------
//function used to configure eOMtask obj
void s_eom_appMotorController_taskInit(void *p);
static void s_eom_appMotorController_taskStartup(EOMtask *p, uint32_t t);
static void s_eom_appMotorController_taskRun(EOMtask *tsk, uint32_t evtmsgper); 

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------
static const char s_eobj_ownname[] = "EOMappMotorController";

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------
extern EOMappMotorController* eom_appMotorController_New(EOMappMotorController_cfg_t *cfg)
{
    EOMappMotorController *retptr = NULL;

    if(NULL == cfg)
    {
        return(retptr);
    }


    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOMappMotorController), 1);

    //copy obj's config
    memcpy(&retptr->cfg, cfg, sizeof(EOMappMotorController_cfg_t));
       
    retptr->st = eOm_appMotorController_st__idle; 

    retptr->mytask = eom_task_New(eom_mtask_OnAllEventsDriven,
                              TASK_MOTORCONTROLLER_PRIO ,
                              2*1024,           //stacksize: da rivedere
                              s_eom_appMotorController_taskStartup, 
                              s_eom_appMotorController_taskRun,  
                              EVT_CALC_START |EVT_DATACOLLECTOR_FINISHED,  //message queue size or all evt mask 
                              eok_reltimeINFINITE,
                              retptr, 
                              s_eom_appMotorController_taskInit, 
                              "motorController" );
    
    return(retptr);
}


extern eOresult_t eom_appMotorController_Activate(EOMappMotorController *p)
{
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }

    p->st = eOm_appMotorController_st__active;

    return(eores_OK);
}


extern eOresult_t eom_appMotorController_Deactivate(EOMappMotorController *p)
{
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }

    p->st = eOm_appMotorController_st__idle;

    return(eores_OK);
}

extern eOresult_t eom_appMotorController_Satrt2Calculate(EOMappMotorController *p)
{
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }
    
    if(eOm_appMotorController_st__idle == p->st)
    {
        return(eores_NOK_generic);
    }

    eom_task_isrSetEvent(p->mytask, EVT_CALC_START);
    return(eores_OK);   

}

extern eOresult_t eom_appMotorController_AllDataAreReady(EOMappMotorController *p)
{
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }

    if(eOm_appMotorController_st__idle == p->st)
    {
        return(eores_NOK_generic);
    }
    
    eom_task_SetEvent(p->mytask, EVT_DATACOLLECTOR_FINISHED);

    return(eores_OK);
}

    


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
void s_eom_appMotorController_taskInit(void *p)
{
    eom_task_Start(p);
}

static void s_eom_appMotorController_taskStartup(EOMtask *tsk, uint32_t t)
{
    EOMappMotorController *p = (EOMappMotorController*)eom_task_GetExternalData(tsk);

    eo_errman_Assert(eo_errman_GetHandle(), NULL != p, s_eobj_ownname, "extdata() is NULL");

    // ALE
    eo_emsController_Init(1, EMS_GENERIC);

    eo_emsController_SetPosLimits(0, 0, 4095);
    eo_emsController_SetVelMax(0, 4096);
    eo_emsController_SetPosPid(0, 1024, 0, 0, 10);
    eo_emsController_SetPosPidLimits(0, 0x7FFF, 1024);
    eo_emsController_SetVelPid(0, 1024, 0, 0, 10);
    eo_emsController_SetVelPidLimits(0, 4096, 1024);
    eo_emsController_SetControlMode(0, CM_POSITION);
}

static void s_eom_appMotorController_taskRun(EOMtask *tsk, uint32_t evtmsgper)
{
    int16_t *pwm;
    eOevent_t evt;

    uint32_t encoder_raw[6];
    
    uint8_t parity_error = 0;

    EOMappMotorController *p = (EOMappMotorController*)eom_task_GetExternalData(tsk);

    evt = (eOevent_t)evtmsgper;

    if(EVT_CHECK(evt, EVT_CALC_START))
    {
        // ALE
  
        //if (eobool_true == eo_appEncReader_isReady(p->cfg.encReader))     
            eo_appEncReader_getValues(p->cfg.encReader, encoder_raw);

        for (uint8_t b=0; b<18; ++b)
        {
            parity_error ^= (encoder_raw[0]>>b) & 1;
        }

        uint8_t bit_check = encoder_raw[0] & 0x3E;

        if (parity_error || bit_check!=0x20)
        {
            eo_emsController_SkipEncoders();
        }
        else
        {
            encoder_raw[0]>>=6;
            encoder_raw[0]&=0x0FFF;

            eo_emsController_ReadEncoders((int32_t*)encoder_raw);
        }

        static eObool_t must_reset = eobool_true;

        if (must_reset)
        {
            eo_emsController_Stop(0);
            must_reset = eobool_false;
        }

        /* 2) pid calc */
        pwm = eo_emsController_PWM();
        
        pwm_out = -pwm[0];

        /* 3) reset my state */
        p->st = eOm_appMotorController_st__active;

        /* 4) say to start transmiting data to appDataTransmitter */
        p->cfg.sig2appDataTransmitter.fn(p->cfg.sig2appDataTransmitter.argoffn);
    }
}

// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



