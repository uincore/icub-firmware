/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
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


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "string.h"

#include "EoCommon.h"
#include "EoError.h"
#include "EOtheErrorManager.h"


#include "EOtheCANservice.h"
#include "EOtheCANmapping.h"
#include "EOtheCANprotocol.h"

#include "EOaction.h"

#include "EOMtheEMSconfigurator.h"

#include "EOtheEntities.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheCANdiscovery.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheCANdiscovery_hid.h"


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables. deprecated: better using _get(), _set() on static variables 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static eOresult_t s_eo_candiscovery_sendGetFWVersion(uint32_t dontaskmask);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static EOtheCANdiscovery s_eo_thecandiscovery = 
{
    .initted                = eobool_false,
    .discoverytimer   = NULL
};

//static const char s_eobj_ownname[] = "EOtheCANdiscovery";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern EOtheCANdiscovery* eo_candiscovery_Initialise(void)
{
    if(eobool_true == s_eo_thecandiscovery.initted)
    {
        return(&s_eo_thecandiscovery);
    }
    

    s_eo_thecandiscovery.discoverytimer = eo_timer_New(); 

    s_eo_thecandiscovery.initted = eobool_true;
    
    return(&s_eo_thecandiscovery);   
}


extern EOtheCANdiscovery* eo_candiscovery_GetHandle(void)
{
    return(eo_candiscovery_Initialise());
}


extern eOresult_t eo_candiscovery_Start(EOtheCANdiscovery *p)
{   
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }
    
    EOaction_strg astg = {0};
    EOaction *action = (EOaction*)&astg;
       
    eo_action_SetEvent(action, emsconfigurator_evt_userdef01, eom_emsconfigurator_GetTask(eom_emsconfigurator_GetHandle()));
    eo_timer_Start(s_eo_thecandiscovery.discoverytimer, eok_abstimeNOW, 250*eok_reltime1ms, eo_tmrmode_FOREVER, action);
    
    return(eores_OK);       
}


extern eOresult_t eo_candiscovery_Stop(EOtheCANdiscovery *p)
{   
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }
          
    eo_candiscovery_SignalDetectedCANboards(eo_candiscovery_GetHandle());
    
    return(eo_timer_Stop(s_eo_thecandiscovery.discoverytimer));       
}

extern eOresult_t eo_candiscovery_SignalDetectedCANboards(EOtheCANdiscovery *p)
{
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }

    eOerrmanDescriptor_t des = {0};
    des.code = eoerror_code_get(eoerror_category_System, eoerror_value_SYS_canservices_board_detected);
    
    uint8_t numofjomos = eo_entities_NumOfJoints(eo_entities_GetHandle());
    uint8_t i=0;
    for(i=0; i<numofjomos; i++)
    {        
        eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, i, 0);        
        eOcanmap_location_t loc = {0};
        if(eores_OK == eo_canmap_GetEntityLocation(eo_canmap_GetHandle(), id32, &loc, NULL, NULL))
        {
            // send a message... must retrieve the board info 
            const eOcanmap_board_extended_t * board = eo_canmap_GetBoard(eo_canmap_GetHandle(), loc);
            des.sourcedevice    = (eOcanport1 == loc.port) ? (eo_errman_sourcedevice_canbus1) : (eo_errman_sourcedevice_canbus2);
            des.sourceaddress   = loc.addr;
            des.par16           = i; 
            memcpy(&des.par64, &board->detected, sizeof(eObrd_typeandversions_t));
            eo_errman_Error(eo_errman_GetHandle(), eo_errortype_info, NULL, NULL, &des);
        }               
    }
     
    return(eores_OK);
    
}

extern eOresult_t eo_candiscovery_CheckCanBoardsAreReady(EOtheCANdiscovery *p, uint32_t dontaskmask)
{
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }
    
    return(s_eo_candiscovery_sendGetFWVersion(dontaskmask));
}

// must return true only if all can boards have replied and their protocol version matches with target
extern eObool_t eo_candiscovery_areCanBoardsReady(EOtheCANdiscovery *p, uint32_t *canBoardsReady, uint32_t *canBoardsChecked)
{
    eObool_t res = eobool_true;
    
    if((NULL == p) || (NULL == canBoardsReady))
    {
        return(eobool_false);
    }
    
    *canBoardsReady = 0;
    
    uint8_t numofjomos = eo_entities_NumOfJoints(eo_entities_GetHandle());
    uint8_t i=0;
    for(i=0; i<numofjomos; i++)
    {
        if(NULL != canBoardsChecked)
        {
            *canBoardsChecked |= (1<<i);
        }
        
        eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, i, 0);        
        eOcanmap_location_t location = {0};
        if(eores_OK == eo_canmap_GetEntityLocation(eo_canmap_GetHandle(), id32, &location, NULL, NULL))
        {
            // must retrieve the extended board info 
            const eOcanmap_board_extended_t * board = eo_canmap_GetBoard(eo_canmap_GetHandle(), location);
            eObool_t ready = eobool_false;
            uint8_t major = board->detected.protocolversion.major;
            uint8_t minor = board->detected.protocolversion.minor;
            if(0 != major)
            {  // if not zero, then the board has replied and everything is filled. however i must check if there is a match
                if((board->board.props.requiredprotocol.major == major) && (board->board.props.requiredprotocol.minor == minor))
                {
                    ready = eobool_true;
                    *canBoardsReady |= (1<<i);
                }
            } 
            res &= ready;
        }               
    }
    
    return(res);       
}


extern eOresult_t eo_candiscovery_ManageDetectedBoard(EOtheCANdiscovery *p, eOcanmap_location_t loc, eObool_t match, eObrd_typeandversions_t *detected)
{
    eOresult_t res = eores_OK;
    
    // at first we send diagnostics
    if(eobool_false == match)
    {
        char str[64] = {0};
        snprintf(str, sizeof(str), "wrong fwver: %d %d", detected->firmwareversion.major, detected->firmwareversion.minor); 
        eo_errman_Error(eo_errman_GetHandle(), eo_errortype_error, str, NULL, &eo_errman_DescrUnspecified);
    }
    
    // then we put the detected inside the EOtheCANmapping
    eo_canmap_BoardSetDetected(eo_canmap_GetHandle(), loc, detected);
 

    #warning marco.accame: i prefer not to stop the request procedure in here. i would rather do it at tick of the timer inside eom_emsconfigurator_hid_userdef_ProcessUserdef01Event()
//    // if all connected can boards are ready, then we stop the request procedure
//    if(eobool_true == eo_appTheDB_areConnectedCanBoardsReady(db, &canBoardsReady, NULL))
//    {
//        eo_emsapplBody_checkCanBoards_Stop(eo_emsapplBody_GetHandle());
//        eo_emsapplBody_sendConfig2canboards(eo_emsapplBody_GetHandle());
//        //if MC4, enable MAIS and BCastPolicy
//        eOmn_appl_runMode_t appl_run_mode = eo_emsapplBody_GetAppRunMode(eo_emsapplBody_GetHandle());
//        if((applrunMode__skinAndMc4 == appl_run_mode) || (applrunMode__mc4Only == appl_run_mode))
//        {   
//			eo_emsapplBody_MAISstart(eo_emsapplBody_GetHandle());
//        }
//    }    
    
    return(res);
}

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static eOresult_t s_eo_candiscovery_sendGetFWVersion(uint32_t dontaskmask)
{  
    // i check only mc. thus, i get the number of mc jomos, i search for the i-th board and i send a get fw version query to it.
    
    uint8_t numofjomos = eo_entities_NumOfJoints(eo_entities_GetHandle());
    uint8_t i=0;
    for(i=0; i<numofjomos; i++)
    {
        if( ((1<<i) & dontaskmask) == (1<<i))
        {
            continue;
        }
        
        eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, i, 0);        
        eOcanmap_location_t location = {0};
        if(eores_OK == eo_canmap_GetEntityLocation(eo_canmap_GetHandle(), id32, &location, NULL, NULL))
        {
            // send a message... must retrieve the board info 
            const eOcanmap_board_extended_t * board = eo_canmap_GetBoard(eo_canmap_GetHandle(), location);
            // i decide to send the request only if the board has not replied. if it replied but the protocol does
            // not match then i dont send the command anymore.
            if(0 == board->detected.protocolversion.major)
            {  // must ask if the board has not responded yet
                eOcanprot_command_t command = {0};
                command.class = eocanprot_msgclass_pollingMotorControl;
                command.type  = ICUBCANPROTO_POL_MC_CMD__GET_FIRMWARE_VERSION;
                command.value = (void*)&board->board.props.requiredprotocol;
                eo_canserv_SendCommandToEntity(eo_canserv_GetHandle(), &command, id32);                  
            }                        
        }               
    }
    
    return(eores_OK);
}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



