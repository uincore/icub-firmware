
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOmcController_H_
#define _EOmcController_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @file       EOmcController.h
    @brief      This header file implements public interface to a motor controller.
    @author     alessandro.scalzo@iit.it
    @date       27/03/2012
**/

/** @defgroup eo_mcController Object EOmcController
    Does something.
     
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EoMotionControl.h"


// - public #define  --------------------------------------------------------------------------------------------------

//#define MC_CAN_DEBUG

// - declaration of public user-defined types ------------------------------------------------------------------------- 

typedef enum
{
    emscontroller_board_DONTCARE                = 16,
    emscontroller_board_NO_CONTROL              = 0,
    emscontroller_board_ANKLE                   = 1,
    emscontroller_board_UPPERLEG                = 2,
    emscontroller_board_WAIST                   = 3,
    emscontroller_board_SHOULDER                = 4,
    emscontroller_board_HEAD_neckpitch_neckroll = 5,
    emscontroller_board_HEAD_neckyaw_eyes       = 6,
    emscontroller_board_FACE_eyelids_jaw        = 7,
    emscontroller_board_FACE_lips               = 8,
    emscontroller_board_HAND_1                  = 9,
    emscontroller_board_HAND_2                  = 10,
    emscontroller_board_FOREARM                 = 11
} eOemscontroller_board_t;


typedef enum
{
    emscontroller_actuation_2FOC                = 0,
    emscontroller_actuation_LOCAL               = 1
} eOemscontroller_actuation_t;

// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------

 

/** @}            
    end of group eo_mcController  
 **/

#ifdef __cplusplus
}       // closing brace for extern "C"
#endif 

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

