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

// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHECANMAPPING_H_
#define _EOTHECANMAPPING_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @file       EOtheCANmapping.h
	@brief      This header file implements public interface to the singleton which tells how the can boards are mapped onto the ETH board
 	@author     marco.accame@iit.it
	@date       12 mar 2015
 **/

/** @defgroup eo_thecanmapocol Singleton EOtheCANmapping 
    
    It manages can protocol in icub.  
  
    @{		
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EoProtocol.h"
#include "EoBoards.h"
#include "EOconstvector.h"
#include "EOconstarray.h"


// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
 
// - declaration of public user-defined types ------------------------------------------------------------------------- 

/** @typedef    typedef struct EOtheCANmapping_hid EOtheCANmapping
    @brief      EOtheCANmapping is an opaque struct.
 **/  
typedef struct EOtheCANmapping_hid EOtheCANmapping;

enum 
{  
    eocanmap_joints_maxnumberof     = 12, 
    eocanmap_motors_maxnumberof     = eocanmap_joints_maxnumberof, 
    eocanmap_strains_maxnumberof    = 1, 
    eocanmap_maises_maxnumberof     = 1,
    eocanmap_inertials_maxnumberof  = 1,
    eocanmap_skins_maxnumberof      = 2
};
    
enum 
{  
    eocanmap_joint_index_boards_maxnumberof     = 1,
    eocanmap_motor_index_boards_maxnumberof     = 1,
    eocanmap_strain_index_boards_maxnumberof    = 1,
    eocanmap_mais_index_boards_maxnumberof      = 1,
    eocanmap_inertial_index_boards_maxnumberof  = 1,
    eocanmap_skin_index_boards_maxnumberof      = 8     // it tells that we can use up to 8 can boards to represent a single skin index
};


// used only as internal representation for eOcanmap_board_t::indexofentity[] 
typedef enum 
{
    entindex00 =  0, entindex01 =  1, entindex02 =  2, entindex03 =  3, entindex04 =  4, 
    entindex05 =  5, entindex06 =  6, entindex07 =  7, entindex08 =  8, entindex09 =  9, 
    entindex10 = 10, entindex11 = 11, entindex12 = 12, entindex13 = 13, entindex14 = 14, 
    entindexNONE = 255    
} eOcanmap_entityindex_t; 


/**	@typedef    typedef struct eOcanmap_board_t 
 	@brief      Contains the definition of a can board with properties and the index of the entity it holds. 
 **/
typedef struct
{
    eObrd_canproperties_t       props;
    uint8_t                     indexofentity[2];   /**< at most two entities per board. use eObrd_caninsideindex_t for addressing array and use eOcanmap_entityindex_t as its value. */
} eOcanmap_board_t;             EO_VERIFYsizeof(eOcanmap_board_t, 6); 


/**	@typedef    typedef struct eOcanmap_board_extended_t 
 	@brief      Contains the definition of a can board and also the information detected over can discovery. 
 **/
typedef struct
{
    eOcanmap_board_t            board;
    eObrd_info_t                detected;
} eOcanmap_board_extended_t;    EO_VERIFYsizeof(eOcanmap_board_extended_t, 12); 


/**	@typedef    typedef struct eOcanmap_entitydescriptor_t 
 	@brief      Contains the description of an entity with a given index mapped onto a can board location
 **/
typedef struct
{
    eObrd_canlocation_t             location;
    eOcanmap_entityindex_t          index;
} eOcanmap_entitydescriptor_t;


/**	@typedef    typedef struct eOcanmap_cfg_t 
 	@brief      Contains the configuration for the EOtheCANmapping. 
 **/
typedef struct
{
    uint32_t            nothingsofar;
} eOcanmap_cfg_t;


enum { eocanmap_maxlocations = 32 };

///**	@typedef    typedef struct eOcanmap_arrayof_locations_t 
// 	@brief      Contains the compact description of all the board locations which have been loaded into EOtheCAN mapping object.
//                The description is an array of locations. At most there are 28 boards (14 boards per can bus) because addresses 0 
//                and 15 are not used. We however use 32.
// **/
//typedef struct
//{
//    eOarray_head_t      head;
//    eObrd_canlocation_t data[eocanmap_maxlocations];   
//} eOcanmap_arrayof_locations_t;
    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------



// - declaration of extern public functions ---------------------------------------------------------------------------



/** @fn         EOtheCANmapping * eo_canmap_Initialise(const eOcanmap_cfg_t *canmapcfg)
    @brief      Initialise the EOtheCANmapping singleton 
    @arg        canmapcfg       The configuration. NULL is OK.
    @return     The handle to the EOtheCANmapping
 **/
extern EOtheCANmapping * eo_canmap_Initialise(const eOcanmap_cfg_t *canmapcfg);


/** @fn         EOtheCANmapping * eo_canmap_GetHandle(void)
    @brief      Retrieve the EOtheCANmapping singleton 
    @return     The handle to the EOtheCANmapping
 **/
extern EOtheCANmapping * eo_canmap_GetHandle(void);


/** @fn         eOresult_t eo_canmap_LoadBoards(EOtheCANmapping *p,  EOconstvector *vectorof_boardprops)
    @brief      Loads some boards onto the object. The function can be called only once with a vector containing all the boards,
                or it can be called multiple times to load some board at a time. if the same board is loaded multiple times, the
                last loaded wins. 
    @param      p                       The handle to the EOtheCANmapping                
    @param      vectorof_boardprops     A const vector of eObrd_canproperties_t, one item per board                 
    @return     eores_OK if successfull.
 **/
extern eOresult_t eo_canmap_LoadBoards(EOtheCANmapping *p,  EOconstvector *vectorof_boardprops);



extern eOresult_t eo_canmap_UnloadBoards(EOtheCANmapping *p,  EOconstvector *vectorof_boardprops);


/** @fn         eOresult_t eo_canmap_ConfigEntity(EOtheCANmapping *p,  eOprotEndpoint_t ep, eOprotEntity_t entity, EOconstvector *vectorof_entitydescriptors)
    @brief      After the boards are loaded, it configures an entity so that it is mapped into the boards. The function can be called only once with a vector containing 
                all the descriptors for the entity or it can be called multiple times to load some descriptors at a time. 
                As an example, suppose you want to load the 4 joints with indices 0, 1, 2, and 3 into 2foc boards on can1 with addresses 5, 6, 8, 10. 
                all at internal index 0. you can call this function with a vector of {.index = 0, .loc.can = 0, .loc.adr = 5, .loc.internalindx = 0} etc.                                 
    @param      p           The handle to the EOtheCANmapping
    @param      ep          the endpoint of the entity .... no management 
    @param      entity      the entity   
    @param      vectorof_entitydescriptors      the vector of eOcanmap_entitydescriptor_t items
    @return     eores_OK if successfull.
 **/
extern eOresult_t eo_canmap_ConfigEntity(EOtheCANmapping *p,  eOprotEndpoint_t ep, eOprotEntity_t entity, EOconstvector *vectorof_entitydescriptors);

extern eOresult_t eo_canmap_DeconfigEntity(EOtheCANmapping *p,  eOprotEndpoint_t ep, eOprotEntity_t entity, EOconstvector *vectorof_entitydescriptors);

/** @fn         const eOcanmap_board_extended_t * eo_canmap_GetBoard(EOtheCANmapping *p, eObrd_canlocation_t loc)
    @brief      get a board given its can location, for read only purposes.        
    @param      p           The handle to the EOtheCANmapping
    @param      loc         teh can location
    @return     a pointer to the board, or NULL if no board is found at that location
**/
extern const eOcanmap_board_extended_t * eo_canmap_GetBoard(EOtheCANmapping *p, eObrd_canlocation_t loc);


extern eObrd_cantype_t eo_canmap_GetBoardType(EOtheCANmapping *p, eObrd_canlocation_t bloc);


/** @fn         eOresult_t eo_canmap_BoardSetDetected(EOtheCANmapping *p, eObrd_canlocation_t loc, eObrd_info_t *detected)
    @brief      change the fw version etc on a board. we read it from can and then we set it with this function       
    @param      p           The handle to the EOtheCANmapping
    @param      loc         teh can location
    @param      detected    the detected type and versions which we want to set inside the object
    @return     eores_OK if successfull.
**/
extern eOresult_t eo_canmap_BoardSetDetected(EOtheCANmapping *p, eObrd_canlocation_t loc, eObrd_info_t *detected);



/** @fn         eOprotIndex_t eo_canmap_GetEntityIndex(EOtheCANmapping *p, eObrd_canlocation_t loc)
    @brief      i get the index of the entity on the board on a given location without verifying type of board. 
                it is teh quick version of eo_canmap_GetEntityIndexExtraCheck()     
    @param      p           The handle to the EOtheCANmapping
    @param      loc         the can location
    @return     the index at the specified location or EOK_uint08dummy if no board is found.
**/
extern eOprotIndex_t eo_canmap_GetEntityIndex(EOtheCANmapping *p, eObrd_canlocation_t loc);



/** @fn         eOprotIndex_t eo_canmap_GetEntityIndexExtraCheck(EOtheCANmapping *p, eObrd_canlocation_t loc, eOprotEndpoint_t ep, eOprotEntity_t entity)
    @brief      i get the index of the entity on the board on a given location and also verify if the board type is coherent with the specified entity.
                This function is the safer version of eo_canmap_GetEntityIndex()
                For example if we want the index of a joint which we believe is in can1, address 14, internalindex 0, but then the found board is a MAIS ...
                this function returns EOK_uint08dummy, whereas eo_canmap_GetEntityIndex() would return a ... value.      
    @param      p           The handle to the EOtheCANmapping
    @param      loc         the can location
    @param      ep          the endpoint of the entity 
    @param      entity      the entity      
    @return     the index at the specified location or EOK_uint08dummy if no board is found or of the board type is not coherent with the entity type.
**/
extern eOprotIndex_t eo_canmap_GetEntityIndexExtraCheck(EOtheCANmapping *p, eObrd_canlocation_t loc, eOprotEndpoint_t ep, eOprotEntity_t entity);



/** @fn         eOresult_t eo_canmap_GetEntityLocation(EOtheCANmapping *p, eOprotID32_t id32, eObrd_canlocation_t *loc, uint8_t *numoflocs, eObrd_cantype_t *boardtype)
    @brief      it gets the location of an entity with a given index. but also tells how many can boards are dedicated to this entity (eg.g, skin uses several
                can boards), and the type of board.    
    @param      p           The handle to the EOtheCANmapping
    @param      id32        it specifies the entity. use for instance: eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, 2, eoprot_tag_none); 
    @param      loc         pointer to the wanted can location
    @param      numoflocs   if not NULL it will hold the number of can boards used by that entity (all use one except for skin whcih may use 7 or 8 or ...)
    @param      boardtype   if not NULL it will hold the type of can board used by that entity.
    @return     eores_OK if successfull.
**/
extern eOresult_t eo_canmap_GetEntityLocation(EOtheCANmapping *p, eOprotID32_t id32, eObrd_canlocation_t *loc, uint8_t *numoflocs, eObrd_cantype_t *boardtype);



// it returns a eOcanmap_arrayof_locations_t*
/** @fn         EOconstarray* eo_canmap_GetBoardLocations(EOtheCANmapping *p)
    @brief      it gets the EOconstarray with elements of type eObrd_canlocation_t of all the can boards loaded in the object.
    @param      p           The handle to the EOtheCANmapping
    @return     the array, or NULL if p is NULL.
**/
//extern EOconstarray* eo_canmap_GetBoardLocations(EOtheCANmapping *p);


/** @}            
    end of group eo_thecanmapocol  
 **/

#ifdef __cplusplus
}       // closing brace for extern "C"
#endif 

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

