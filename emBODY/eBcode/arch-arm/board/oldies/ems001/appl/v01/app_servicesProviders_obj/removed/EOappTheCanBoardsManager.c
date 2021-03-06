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

/* @file       eo_appl_encodersReaderModule.c
    @brief      This file implements emcodersReader Module.
    @author    valentina.gaggero@iit.it
    @date       06/27/2012
**/



// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------
#include "stdlib.h"
#include "string.h"
#include "hal.h"
//#include "hal_debugPin.h"

#include "eOcommon.h"
#include "EOtheMemoryPool.h"
#include "EOappCanServicesProvider.h"
#include "EOappTheDataBase.h"

#include "EOMtheEMSapplCfg.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------
#include "EOappTheCanBoardsManager.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
#include "EOappTheCanBoardsManager_hid.h"


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables. deprecated: better using _get(), _set() on static variables 
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------
//config
static eOresult_t s_eo_appTheCanBrdsMng_ConfigBoard(EOappTheCanBrdsMng *p, eObrd_boardId_t bid);
static eOresult_t s_eo_appTheCanBrdsMng_ConfigMC4(EOappTheCanBrdsMng *p, eObrd_boardId_t bid);
static eOresult_t s_eo_appTheCanBrdsMng_Config1FOC(EOappTheCanBrdsMng *p, eObrd_boardId_t bid);
static eOresult_t s_eo_appTheCanBrdsMng_ConfigSkin(EOappTheCanBrdsMng *p, eObrd_boardId_t bid);

//start
static eOresult_t s_eo_appTheCanBrdsMng_StartBoard(EOappTheCanBrdsMng *p, eo_emsCanNetTopo_boardTopoInfo_t *board_ptr);
static eOresult_t s_eo_appTheCanBrdsMng_StartMotorboard(EOappTheCanBrdsMng *p, eo_emsCanNetTopo_boardTopoInfo_t *board_ptr);

//stop
static eOresult_t s_eo_appTheCanBrdsMng_StopBoard(EOappTheCanBrdsMng *p, eo_emsCanNetTopo_boardTopoInfo_t *board_ptr);
static eOresult_t s_eo_appTheCanBrdsMng_StopMotorboard(EOappTheCanBrdsMng *p, eo_emsCanNetTopo_boardTopoInfo_t *board_ptr);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------
static EOappTheCanBrdsMng s_theCanBrdsManager = 
{
    EO_INIT(.st)    eO_appTheCanBrdsMng_st__NOTinited
};

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------
extern EOappTheCanBrdsMng* eo_appTheCanBrdsMng_Initialise(eOappTheCanBrdsMng_cfg_t *cfg)
{
    EOappTheCanBrdsMng *retptr = NULL;

    if((NULL == cfg) || (NULL == cfg->appCanSP_ptr))
    {
        return(retptr);
    }
    
    retptr = &s_theCanBrdsManager;

    //save in obj its configuration
    memcpy(&(retptr->cfg), cfg, sizeof(eOappTheCanBrdsMng_cfg_t));
    
    retptr->totalnumofcanboards = eo_appTheDB_GetNumeberOfCanboards(eo_appTheDB_GetHandle());
    retptr->st = eO_appTheCanBrdsMng_st__inited;
    
    return(retptr);
}


extern EOappTheCanBrdsMng* eo_appTheCanBrdsMng_GetHandle(void)
{
    return((s_theCanBrdsManager.st == eO_appTheCanBrdsMng_st__inited) ? (&s_theCanBrdsManager) : (NULL));
}

extern eOresult_t eo_appTheCanBrdsMng_ConfigAllBoards(EOappTheCanBrdsMng *p)
{
    eObrd_boardId_t bid;
    eOresult_t res = eores_OK;
    
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }

    for(bid=0; ((bid<p->totalnumofcanboards) && (eores_OK==res)); bid++)
    {
        res  = s_eo_appTheCanBrdsMng_ConfigBoard(p, bid);
    }
    return(res);
}

extern eOresult_t eo_appTheCanBrdsMng_StartAllBoards(EOappTheCanBrdsMng *p)
{

    eObrd_boardId_t bid;
    eo_emsCanNetTopo_boardTopoInfo_t *b_ptr; 
    eOresult_t res = eores_OK;
    
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }
    
    for(i=0; ((i<p->cfg_emsCanNetTopo_canBoards__ptr->size) && (eores_OK==res)); i++)
    {
        res  = s_eo_appTheCanBrdsMng_StartBoard(p, &b_ptr[i]);
    }

    
    return(res);
    
    
}



extern eOresult_t eo_appTheCanBrdsMng_StopAllBoards(EOappTheCanBrdsMng *p)
{

    eo_emsCanNetTopo_boardId_t i;
    eo_emsCanNetTopo_boardTopoInfo_t *b_ptr; 
    eOresult_t res = eores_OK;
    
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }
    
    b_ptr = (eo_emsCanNetTopo_boardTopoInfo_t*)p->cfg_emsCanNetTopo_canBoards__ptr->item_array_data; 
    for(i=0; ((i<p->cfg_emsCanNetTopo_canBoards__ptr->size) && (eores_OK==res)); i++)
    {
        res  = s_eo_appTheCanBrdsMng_StopBoard(p, &b_ptr[i]);
    }
    return(res);
    
    
}


extern eOresult_t eo_appTheCanBrdsMng_GetAppRunMode(EOappTheCanBrdsMng *p, eOmn_appl_runMode_t *apprunmode)
{
    
    eo_emsCanNetTopo_boardId_t          i;
    eo_emsCanNetTopo_boardTopoInfo_t    *b_ptr = NULL; 
    eOboolvalues_t                      mc4_isConnected = eobool_false;
    eOboolvalues_t                      skin_isConnected = eobool_false;
    eOboolvalues_t                      foc_isConnected = eobool_false;
    
    if((NULL == p) || (NULL == apprunmode))
    {
        return(eores_NOK_nullpointer);
    }

    
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }
    
    b_ptr = (eo_emsCanNetTopo_boardTopoInfo_t*)p->cfg_emsCanNetTopo_canBoards__ptr->item_array_data; 
    for(i=0; i<p->cfg_emsCanNetTopo_canBoards__ptr->size; i++)
    {
        switch(b_ptr[i].boardtype)
        {
            case eobrd_mc4:
            {
                mc4_isConnected = eobool_true;    
            }break;

            case eobrd_1foc:
            {
                foc_isConnected = eobool_true;    
            }break;
            
            case eobrd_skin:
            {
                skin_isConnected = eobool_true;    
            }break;
        };     
    }
    
    
    
    if(mc4_isConnected)
    {
        if(skin_isConnected)
        {
            *apprunmode = applrunMode__skinAndMc4;
            return(eores_OK);    
        }
        else
        {
            *apprunmode = applrunMode__mc4Only;
            return(eores_OK); 
        }    
    }

    if((foc_isConnected) && (!skin_isConnected))
    {
        *apprunmode = applrunMode__2foc;
        return(eores_OK); 
    }

    if((skin_isConnected) && (!mc4_isConnected) && (!foc_isConnected))
    {
        *apprunmode = applrunMode__skinOnly;
        return(eores_OK);  
    }

    return(eores_NOK_generic); 

}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
static eOresult_t s_eo_appTheCanBrdsMng_ConfigBoard(EOappTheCanBrdsMng *p, eObrd_boardId_t bid)
{

    eOresult_t      res;
    eObrd_types_t   type;
    
    res = eo_appTheDB_GetTypeOfCanboard(eo_appTheDB_GetHandle(), bid, &type);
    if(eores_OK != res)
    {
        return(res);
    }
    
    switch(type)
    {
        case eobrd_mc4:
        {
            res = s_eo_appTheCanBrdsMng_ConfigMC4(p, bid);
        }break;  

        case eobrd_1foc:
        {
            res = s_eo_appTheCanBrdsMng_Config1FOC(p, bid);
        }break;            

        case eobrd_skin:
        {
            res = s_eo_appTheCanBrdsMng_ConfigSkin(p, bid);
        }break; 

        case eobrd_mais:
        {
            res = eores_OK; //config by pc104
        }break; 

        case eobrd_strain:
        {
            res = eores_OK; //config by pc104
        }break; 

        
        default:
        {
            res = eores_NOK_generic;
        }

    };
    return(res);
}

static eOresult_t s_eo_appTheCanBrdsMng_ConfigMC4(EOappTheCanBrdsMng *p, eObrd_boardId_t bid)
{
    eOresult_t                              res;
    eo_appCanSP_canLocation                 canLoc;
    EOMtheEMSapplCfg                        *applCfg;
    eo_icubCanProto_motorAxis_t             axis;
    eo_icubCanProto_msgCommand_t            msgCmd = 
    {
        EO_INIT(.class) eo_icubCanProto_msgCmdClass_pollingMotorBoard,
        EO_INIT(.cmdId) 0
    };
    //it is possible set only 4 info to broadcast.
    uint8_t                                 bcastMsg_payload[4] = 
    {
        ICUBCANPROTO_PER_MB_CMD_POSITION,
        ICUBCANPROTO_PER_MB_CMD_PID_VAL,
        ICUBCANPROTO_PER_MB_CMD_PID_ERROR,
        ICUBCANPROTO_PER_MB_CMD_VELOCITY
    };    

    canLoc.emscanport = board_ptr->emscanport;
    canLoc.canaddr = board_ptr->canaddr;
    
    //get application config
    applCfg = eom_emsapplcfg_GetHandle();

    for(axis=eo_icubCanProto_mAxis_0; axis < eo_icubCanProtomotorAxis_maxNum4CanBoard; axis++)
    {
        canLoc.axis = axis;
        
        // 1) set bcast policy
        msgCmd.cmdId = ICUBCANPROTO_POL_MB_CMD__SET_BCAST_POLICY;
        res = eo_appCanSP_SendCmd(p->cfg.appCanSP_ptr, &canLoc, msgCmd, bcastMsg_payload);
        if(eores_OK != res)
        {
            return(res);
        }
        

        // 2) set vel shift
        msgCmd.cmdId = ICUBCANPROTO_POL_MB_CMD__SET_VEL_SHIFT;
        res = eo_appCanSP_SendCmd(p->cfg.appCanSP_ptr, &canLoc, msgCmd, (void*)&applCfg->srvcfg.measConvCfg.jointvelocityShift);
        if(eores_OK != res)
        {
            return(res);
        }

            
        // 3) set estim vel shift
        msgCmd.cmdId = ICUBCANPROTO_POL_MB_CMD__SET_SPEED_ESTIM_SHIFT;
        res = eo_appCanSP_SendCmd(p->cfg.appCanSP_ptr, &canLoc, msgCmd, (void*)&applCfg->srvcfg.measConvCfg.jointVelocityEstimationShift);
        if(eores_OK != res)
        {
            return(res);
        }
    }
    
    //4) set disable control loop
    msgCmd.cmdId = ICUBCANPROTO_POL_MB_CMD__CONTROLLER_IDLE;
    res = eo_appCanSP_SendCmd(p->cfg.appCanSP_ptr, &canLoc, msgCmd, NULL);
    if(eores_OK != res)
    {
        return(res);
    }
        
    return(eores_OK);
}


static eOresult_t s_eo_appTheCanBrdsMng_Config1FOC(EOappTheCanBrdsMng *p, eo_emsCanNetTopo_boardTopoInfo_t *board_ptr)
{
    eOresult_t                              res;
    eo_appCanSP_canLocation                 canLoc;
//    eo_emsCanNetTopo_motorAdditionalInfo_t  *addInfo_ptr;
    eo_icubCanProto_msgCommand_t    msgCmd = 
    {
        EO_INIT(.class) eo_icubCanProto_msgCmdClass_pollingMotorBoard,
        EO_INIT(.cmdId) 0
    };

//     eOmc_i2tParams_t i2tParams = 
//     {
//         EO_INIT(.tresh)  0xAA,
//         EO_INIT(.time)   0xBB
//     };
    
#ifdef _USE_PROTO_TEST_

    eOmc_motor_config_t cfg =
    { 
        EO_INIT(.pidcurrent)
        {
            EO_INIT(.kp)                    0x0BB8,
            EO_INIT(.ki)                    0x1388,
            EO_INIT(.kd)                    0x0000,
            EO_INIT(.limitonintegral)       0X4444,
            EO_INIT(.limitonoutput)         0x5555,
            EO_INIT(.scale)                 0x0,
            EO_INIT(.offset)                0x6666,
            EO_INIT(.filler03)              {0xf1, 0xf2, 0xf3}
        },
        EO_INIT(.maxvelocityofmotor)        0xAA,
        EO_INIT(.maxcurrentofmotor)         0xBB,
        EO_INIT(.des02FORmstatuschamaleon04)   {0}
    };
#endif
    canLoc.emscanport = board_ptr->emscanport;
    canLoc.canaddr = board_ptr->canaddr;

    /* use default i2t value in 2FOC firmware

    // 1) set i2t params
    msgCmd.cmdId = ICUBCANPROTO_POL_MB_CMD__SET_I2T_PARAMS;
    res = eo_appCanSP_SendCmd(p->cfg.appCanSP_ptr, &canLoc, msgCmd, &i2tParams);
    if(eores_OK != res)
    {
        return(res);
    }

    */    

    //2) set disable control loop
    msgCmd.cmdId = ICUBCANPROTO_POL_MB_CMD__CONTROLLER_IDLE;
    res = eo_appCanSP_SendCmd(p->cfg.appCanSP_ptr, &canLoc, msgCmd, NULL);
    if(eores_OK != res)
    {
        return(res);
    }

#ifdef _USE_PROTO_TEST_
    return(eo_appCanSP_ConfigMotor(p->cfg.appCanSP_ptr, 3, &cfg));
#else
    return(eores_OK);
#endif
}

static eOresult_t s_eo_appTheCanBrdsMng_ConfigSkin(EOappTheCanBrdsMng *p, eo_emsCanNetTopo_boardTopoInfo_t *board_ptr)
{
    eOresult_t res;
    uint8_t i;
    eo_appCanSP_canLocation canLoc; //here I don't use sensorcanlocation because i need jmindexId filed to set triangle id
    eo_icubCanProto_msgCommand_t msgCmd = 
    {
        EO_INIT(.class) 4,
        EO_INIT(.cmdId) ICUBCANPROTO_POL_SK_CMD__TACT_SETUP
    };

    canLoc.emscanport = board_ptr->emscanport;
//    canLoc.canaddr = board_ptr->canaddr;

    for(i=8; i<15; i++)
    {
        canLoc.canaddr = i;
        res = eo_appCanSP_SendCmd(p->cfg.appCanSP_ptr, &canLoc, msgCmd, NULL);
         if(eores_OK != res)
        {
            return(res);
        }
    }
    return(res);

}


static eOresult_t s_eo_appTheCanBrdsMng_StartBoard(EOappTheCanBrdsMng *p, eo_emsCanNetTopo_boardTopoInfo_t *board_ptr)
{
 eOresult_t res;
    
    switch(board_ptr->boardtype)
    {
        case eobrd_mc4:
        case eobrd_1foc:
        {
            res = s_eo_appTheCanBrdsMng_StartMotorboard(p, bid);
        }break;  

        case eobrd_skin:
        {
            res = eores_OK;; //nothing to do
        }break; 

        case eobrd_mais:
        {
            res = eores_OK;; //nothing to do
        }break; 

        case eobrd_strain:
        {
            res = eores_OK;; //nothing to do
        }break; 

        default:
        {
            res = eores_NOK_generic;
        }
    };
    return(res);
}

static eOresult_t s_eo_appTheCanBrdsMng_StartMotorboard(EOappTheCanBrdsMng *p, eo_emsCanNetTopo_boardTopoInfo_t *board_ptr)
{
    eOresult_t res;
    eo_appCanSP_canLocation canLoc;
    eo_icubCanProto_msgCommand_t msgCmd = 
    {
        EO_INIT(.class) eo_icubCanProto_msgCmdClass_pollingMotorBoard,
        EO_INIT(.cmdId) 0
    };

    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }

    canLoc.emscanport = board_ptr->emscanport;
    canLoc.canaddr = board_ptr->canaddr;
    msgCmd.cmdId = ICUBCANPROTO_POL_MB_CMD__ENABLE_PWM_PAD;

    res = eo_appCanSP_SendCmd(p->cfg.appCanSP_ptr, &canLoc, msgCmd, NULL);
    if(eores_OK != res)
    {
        return(res);
    }

    // 6) set controller run
    msgCmd.cmdId = ICUBCANPROTO_POL_MB_CMD__CONTROLLER_RUN;
    res = eo_appCanSP_SendCmd(p->cfg.appCanSP_ptr, &canLoc, msgCmd, NULL);
    if(eores_OK != res)
    {
        return(res);
    }

    return(eores_OK);
}


static eOresult_t s_eo_appTheCanBrdsMng_StopBoard(EOappTheCanBrdsMng *p, eo_emsCanNetTopo_boardTopoInfo_t *board_ptr)
{
    eOresult_t res;
    
    switch(board_ptr->boardtype)
    {
        case eobrd_mc4:
        case eobrd_1foc:
        {
            res = s_eo_appTheCanBrdsMng_StopMotorboard(p, board_ptr);
        }break;  

        case eobrd_skin:
        {
            res = eores_OK; //nothing to do
        }break; 

        default:
        {
            res = eores_NOK_generic;
        }
    };
    return(res);
}



static eOresult_t s_eo_appTheCanBrdsMng_StopMotorboard(EOappTheCanBrdsMng *p, eo_emsCanNetTopo_boardTopoInfo_t *board_ptr)
{
    eOresult_t res;
    eo_appCanSP_canLocation canLoc;
    eo_icubCanProto_msgCommand_t msgCmd = 
    {
        EO_INIT(.class) eo_icubCanProto_msgCmdClass_pollingMotorBoard,
        EO_INIT(.cmdId) 0
    };

    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }

    canLoc.emscanport = board_ptr->emscanport;
    canLoc.canaddr = board_ptr->canaddr;
    msgCmd.cmdId = ICUBCANPROTO_POL_MB_CMD__DISABLE_PWM_PAD;

    res = eo_appCanSP_SendCmd(p->cfg.appCanSP_ptr, &canLoc, msgCmd, NULL);
    if(eores_OK != res)
    {
        return(res);
    }

    // 6) set controller run
    msgCmd.cmdId = ICUBCANPROTO_POL_MB_CMD__CONTROLLER_IDLE;
    res = eo_appCanSP_SendCmd(p->cfg.appCanSP_ptr, &canLoc, msgCmd, NULL);
    if(eores_OK != res)
    {
        return(res);
    }

    return(eores_OK);
}

// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



