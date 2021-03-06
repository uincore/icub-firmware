/*
 * Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
 * Author:  Valentina Gaggero, Marco Accame
 * email:   valentina.gaggero@iit.it, marco.accame@iit.it
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

// - include guard ----------------------------------------------------------------------------------------------------

#ifndef _HAL_SPIENCODER_HID_H_
#define _HAL_SPIENCODER_HID_H_


/* @file       hal_spiencoder_hid.h
    @brief      This header file implements hidden interface to a encoder
    @author     marco.accame@iit.it, valentina.gaggero@iit.it
    @date       02/07/2013
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "hal_common.h"
#include "hal_mux.h"


// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "hal_spiencoder.h"



// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section


// - definition of the hidden struct implementing the object ----------------------------------------------------------

typedef struct
{
    hal_spi_t               spiid;          /**< which spi port is used for each encoder */
    hal_mux_t               muxid;          /**< which mux id is used for each encoder */
    hal_mux_sel_t           muxsel;         /**< which mux selection is used of the mux port */    
} hal_spiencoder_spimap_t;


typedef struct
{
    uint32_t                    supportedmask;
    uint32_t                    spimaxspeed;                    // in hz
    hal_spiencoder_spimap_t     spimap[hal_spiencoders_number];
    hal_spiencoder_stream_map_t streammap;
} hal_spiencoder_boardconfig_t;


// - declaration of extern hidden variables ---------------------------------------------------------------------------

extern const hal_spiencoder_boardconfig_t hal_spiencoder__theboardconfig;

// - declaration of extern hidden functions ---------------------------------------------------------------------------



#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




