/*
 * Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
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

/* @file       eOprot_b07.c
    @brief      This file keeps ...
    @author     marco.accame@iit.it
    @date       06/06/2013
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h" 
#include "string.h"
#include "stdio.h"


#include "EOconstvector_hid.h"


#include "EoProtocolAS.h"
#include "EoProtocolMC.h"
#include "EoProtocolMN.h"





// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "eOprot_b07.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------

EO_VERIFYproposition(eoprot_b07_gasdfe, eoprot_boards_maxnumberof > eoprot_b07_boardnumber);



// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static uint16_t s_eoprot_b07_ep2index(void* p, eOnvEP8_t ep);




// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const eOnvset_EPcfg_t s_eoprot_b07_theEPcfgs[] =
{  
    {   // management
        EO_INIT(.endpoint)                          eoprot_endpoint_management,
        EO_INIT(.dummy)                             0,
        EO_INIT(.epram_sizeof)                      sizeof(eOprot_b07_management_t),
        EO_INIT(.fptr_ram_initialise)               eoprot_fun_INITIALISE_mn//,
//        EO_INIT(.protif)                            (eOnvset_protocol_Interface_t*)&eoprot_eonvset_Interface
    },        

    {   // motion-control         
        EO_INIT(.endpoint)                          eoprot_endpoint_motioncontrol,
        EO_INIT(.dummy)                             0,
        EO_INIT(.epram_sizeof)                      sizeof(eOprot_b07_motioncontrol_t),
        EO_INIT(.fptr_ram_initialise)               eoprot_fun_INITIALISE_mc//,
//        EO_INIT(.protif)                            (eOnvset_protocol_Interface_t*)&eoprot_eonvset_Interface       
    },
 
    {   // analog-sensors         
        EO_INIT(.endpoint)                          eoprot_endpoint_analogsensors,
        EO_INIT(.dummy)                             0,
        EO_INIT(.epram_sizeof)                      sizeof(eOprot_b07_analogsensors_t),
        EO_INIT(.fptr_ram_initialise)               eoprot_fun_INITIALISE_as//,
//        EO_INIT(.protif)                            (eOnvset_protocol_Interface_t*)&eoprot_eonvset_Interface        
    }  
    
};  EO_VERIFYsizeof(s_eoprot_b07_theEPcfgs, sizeof(eOnvset_EPcfg_t)*(eoprot_b07_endpoints_numberof));



static const EOconstvector s_eoprot_b07_constvectofEPcfg = 
{
    EO_INIT(.size)                  sizeof(s_eoprot_b07_theEPcfgs)/sizeof(const eOnvset_EPcfg_t),
    EO_INIT(.item_size)             sizeof(eOnvset_EPcfg_t),
    EO_INIT(.item_array_data)       s_eoprot_b07_theEPcfgs
};



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables
// --------------------------------------------------------------------------------------------------------------------

const eOnvset_DEVcfg_t eoprot_b07_nvsetDEVcfg =
{
    EO_INIT(.boardnum)                  eoprot_b07_boardnumber,
    EO_INIT(.dummy)                     {0, 0, 0},
    EO_INIT(.param)                     NULL,
    EO_INIT(.fptr_device_initialise)    eoprot_b07_Initialise,     
    EO_INIT(.vectorof_epcfg)            &s_eoprot_b07_constvectofEPcfg,
    EO_INIT(.fptr_ep2indexofepcfg)      s_eoprot_b07_ep2index
};



const uint8_t eoprot_b07_mn_entities_numberofeach[eomn_entities_numberof] = 
{ 
    eoprot_b07_mn_comms_numberof, 
    eoprot_b07_mn_appls_numberof,
    eoprot_b07_mn_infos_numberof
};

const uint8_t eoprot_b07_mc_entities_numberofeach[eomc_entities_numberof] = 
{ 
    eoprot_b07_mc_joints_numberof, 
    eoprot_b07_mc_motors_numberof, 
    eoprot_b07_mc_controllers_numberof
};

const uint8_t eoprot_b07_as_entities_numberofeach[eoas_entities_numberof] = 
{ 
    eoprot_b07_as_strains_numberof, 
    eoprot_b07_as_maises_numberof,
    eoprot_b07_as_extorque_numberof
};



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern eOresult_t eoprot_b07_Initialise(void* p, eObool_t islocal)
{
    // must initialise the mc, the mn, the ...
    
    eoprot_config_endpoint_entities(eoprot_b07_boardnumber, eoprot_endpoint_management, eoprot_b07_mn_entities_numberofeach);
    eoprot_config_endpoint_entities(eoprot_b07_boardnumber, eoprot_endpoint_motioncontrol, eoprot_b07_mc_entities_numberofeach);
    eoprot_config_endpoint_entities(eoprot_b07_boardnumber, eoprot_endpoint_analogsensors, eoprot_b07_as_entities_numberofeach);
    
    
    if(eobool_true == islocal)
    {
        eoprot_config_board_local(eoprot_b07_boardnumber);
        eoprot_config_proxied_variables(eoprot_b07_boardnumber, eoprot_b07_isvariableproxied);
    }
    
    return(eores_OK);
}


extern eObool_t eoprot_b07_isvariableproxied(eOnvID32_t id)
{
    // in here, if a variable is proxied (the joints in the ems 2 board in the lower arm, for instance) ..
    // the function must return eobool_true for that id
    return(eobool_false);   // no variable is proxied
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


EO_VERIFYproposition(s_eoprot_b07_mn_val, 0 == eoprot_endpoint_management);
EO_VERIFYproposition(s_eoprot_b07_mc_val, 1 == eoprot_endpoint_motioncontrol);
EO_VERIFYproposition(s_eoprot_b07_as_val, 2 == eoprot_endpoint_analogsensors);
EO_VERIFYproposition(s_eoprot_b07_sk_val, 3 == eoprot_endpoint_skin);

static uint16_t s_eoprot_b07_ep2index(void* p, eOnvEP8_t ep)
{    
    if(ep < eoprot_b07_endpoints_numberof)
    {
        return(ep);
    }
    return(EOK_uint16dummy);
}






// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



