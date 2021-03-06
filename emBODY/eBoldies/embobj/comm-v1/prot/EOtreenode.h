/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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

// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTREENODE_H_
#define _EOTREENODE_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @file       EOtreenode.h
    @brief      This header file implements public interface to a node which contains a nv
    @author     marco.accame@iit.it
    @date       04/20/2011
**/

/** @defgroup eo_nvnode Object EOtreenode
    The EOtreenode object contains a EOnv plus its relation with other netvars inside the tree of
    the network variables. It is not created but loaded as a constant object from a configuration mapped in ROM.
     
    @{        
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
//#include "EOnv.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
 

// - declaration of public user-defined types -------------------------------------------------------------------------    



/** @typedef    typedef const struct EOtreenode_hid EOtreenode
    @brief      EOtreenode is an opaque struct. It is used to implement data abstraction for the  
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef const struct EOtreenode_hid EOtreenode;



    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------


// - declaration of extern public functions ---------------------------------------------------------------------------
 


///** @fn         extern EOnv * eo_treemode_hid_New(uint8_t fun, uint8_t typ, uint32_t otherthingsmaybe)
//    @brief      Creates a new nv object. 
//    @return     The pointer to the required object.
// **/

extern uint16_t eo_treenode_GetIndex(EOtreenode *node);

extern void* eo_treenode_GetData(EOtreenode *node);

extern uint8_t eo_treenode_GetNumberOfChilden(EOtreenode *node);

//extern uint16_t eo_treenode_GetIndexOfChild(EOtreenode *node, uint8_t childpos);

extern EOtreenode* eo_treenode_GetChild(EOtreenode *node, uint8_t childpos);

extern eObool_t eo_treenode_isLeaf(EOtreenode *node);


/** @}            
    end of group eo_nvnode 
 **/

#ifdef __cplusplus
}       // closing brace for extern "C"
#endif 

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

