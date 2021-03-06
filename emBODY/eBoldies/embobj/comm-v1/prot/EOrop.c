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

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "EOtheMemoryPool.h"
#include "EOnv_hid.h"
#include "EOtheErrorManager.h"
#include "EOnv_hid.h" 
#ifndef EODEF_DONT_USE_THE_VSYSTEM
#include "EOVtheSystem.h"
#endif


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOrop.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOrop_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------

const eOropconfig_t eok_ropconfig_basic = 
{
    EO_INIT(.confrqst)      eobool_false,
    EO_INIT(.timerqst)      eobool_false,
    EO_INIT(.plussign)      eobool_false,
    EO_INIT(.plustime)      eobool_false
};

const eOropconfiguration_t eok_ropconfiguration_basic =
{
    EO_INIT(.confrqst)      0,
    EO_INIT(.timerqst)      0,
    EO_INIT(.plussign)      0,
    EO_INIT(.plustime)      0,
    EO_INIT(.confirm)       eo_ropconf_none,
    EO_INIT(.notused)       0
};


// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static EOrop * s_eo_rop_prepare_reply(EOrop *ropin, EOrop *ropout);
static eObool_t s_eo_rop_can_surely_say_cannot_perform_on_device(EOrop *ropin);


static void s_eo_rop_received_leaf_exec_on_it(EOrop *rop_in, EOrop *rop_o);
static void s_eo_rop_received_nonleaf_exec_on_it(EOrop *rop_in, EOrop *rop_o);

static void s_eo_rop_received_nonleaf_descend_to_leaves_and_exec(EOrop *rop_in, EOtreenode *nvtreenoderoot, EOrop *rop_o);
static void s_eo_rop_received_nonleaf_but_reached_leaf_and_exec_on_it(EOrop *rop_in, EOtreenode *nvtreeleaf, EOrop *rop_o);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const char s_eobj_ownname[] = "EOrop";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


 
extern EOrop* eo_rop_New(uint16_t capacity)
{
    EOrop *retptr = NULL;    

    // i get the memory for the object
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOrop), 1);

    retptr->capacityofstreamdatafield = capacity;

    if(0 != capacity)
    {
        retptr->stream.data = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_08bit, sizeof(uint8_t), capacity);
    }

    eo_rop_Reset(retptr);
    
    return(retptr);
}


extern eOresult_t eo_rop_Reset(EOrop *p)
{
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }

#define EO_ROP_OPTIMISE_RESET

    p->curindexofstreamdatafield    = 0; // but not capacity !!!


#ifdef EO_ROP_OPTIMISE_RESET
//    memset(&p->aboutip, 0, sizeof(EOrop_aboutip_hid));
#else     
//    p->aboutip.ipaddr               = 0;
//    p->aboutip.ipport               = 0;
#endif

#ifdef EO_ROP_OPTIMISE_RESET
    memset(&p->tmpdata, 0, sizeof(EOrop_tmpdata_t));
#else

    p->tmpdata.nvscfg              = NULL;
    p->tmpdata.nvownership         = eo_nv_ownership_local;
    p->tmpdata.ondevindex          = 0; 
    p->tmpdata.onendpointindex     = 0;     
    p->tmpdata.nvtreenoderoot      = NULL; 
    eo_nv_Clear(&p->netvar);

#endif

#ifdef EO_ROP_OPTIMISE_RESET
    // acces to un-uligned fields is not ok. but head is 8 bytes ....
    *((uint64_t*)(&(p->stream.head))) = 0;
#else
    p->stream.head.ctrl.confinfo           = eo_ropconf_none;
    p->stream.head.ctrl.plustime           = 0;
    p->stream.head.ctrl.plussign           = 0;
    p->stream.head.ctrl.rqsttime           = 0;
    p->stream.head.ctrl.rqstconf           = 0;
    p->stream.head.ctrl.userdefn           = 0;
    p->stream.head.endp                    = 0;
    p->stream.head.ropc                    = eo_ropcode_none;
    p->stream.head.nvid                    = 0;
    p->stream.head.dsiz                    = 0;

    memset(p->stream.data, 0, p->aboutdata.capacity);

#endif

    p->stream.sign          = EOK_uint32dummy;
    p->stream.time          = EOK_uint64dummy;

    return(eores_OK);
}


// this function operates on a given subtree in depth-first
extern eOresult_t eo_rop_Process(EOrop *p, EOrop *replyrop) 
{
    EOrop *rop_o = NULL;

    if((NULL == p) || (NULL == replyrop))
    {
        return(eores_NOK_nullpointer);
    }

    // reset the data progressive index of p. it is used to navigate the data field when it contains tree-nvs
    p->curindexofstreamdatafield = 0;

    // reset the replyrop
    eo_rop_Reset(replyrop);

    // verify if we need a reply rop and prepare it. if we need it, then rop_o is not NULL
    rop_o = s_eo_rop_prepare_reply(p, replyrop);

    // also ... filter out the operations that we cannot surely perform, such as a write on a ro var etc
    // but also any operation on a not-existing nvtreenoderoot
    if(eobool_true == s_eo_rop_can_surely_say_cannot_perform_on_device(p))
    {
        return(eores_OK);
    }

    // very well indeed .. we are in here, then we may perform a rop ... lets see.

#if !defined(EO_NV_DONT_USE_ONROPRECEPTION)    
    eo_nv_hid_OnBefore_ROP(&p->netvar, (eOropcode_t)p->stream.head.ropc, p->stream.time, p->sign);
#endif
    
    
    // per i leaf si chiama s_eo_rop_received_leaf_exec_on_it() direttamente.
    // per i non leaf che sono operabili (non MIX) si chiama una s_eo_rop_received_nonleaf_exec_on_it() che usa la piena capacita' del nodo (anche se c'e' un ARRAY sotto!!)
    // per i non leaf non MIX ma che non richiedono una scrittura chiamo s_eo_rop_received_nonleaf_exec_on_it()           
    // per i non leaf che sono FUN_mix ed hanno una scrittura allora si scende con s_eo_rop_received_nonleaf_descend_to_leaves_and_exec()
    // questo ultimo e' l'unuico caso in cui si gestiscono i dati a pezzetti.
    // ATTENZIONE: un nodo non-leaf deve essere FUN_mix a meno che: (a) tutti i nodi sotto siano con stessa FUN_xxx    

    if(eobool_true == p->netvar.isleaf)
    {   // leaf
        s_eo_rop_received_leaf_exec_on_it(p, rop_o);
    }
    else if(eo_nv_FUN_mix != eo_nv_GetFUN(&p->netvar))
    {   // non leaf but homeogeneous type, thus we can treat all the subtree in the same mode.
        s_eo_rop_received_nonleaf_exec_on_it(p, rop_o);
    }
    else if((eo_ropcode_set != p->stream.head.ropc) && (eo_ropcode_rst != p->stream.head.ropc))
    {   // non leaf and non-homogeneous, but we dont risk writing any read-only data, thus we can read all the subtree in teh same mode.
        s_eo_rop_received_nonleaf_exec_on_it(p, rop_o);
    }
    else
    {   // damn ... we need to descend the tree and operate on each leaf 
        s_eo_rop_received_nonleaf_descend_to_leaves_and_exec(p, p->netvar.treenode, rop_o);
    }

#if !defined(EO_NV_DONT_USE_ONROPRECEPTION)
    eo_nv_hid_OnAfter_ROP(&p->netvar, (eOropcode_t)p->stream.head.ropc, p->stream.time, p->sign);
#endif

    return(eores_OK);
}


extern eOropcode_t eo_rop_GetROPcode(EOrop *p)
{
    if(NULL == p)
    {
        return(eo_ropcode_none);
    }

    return((eOropcode_t)p->stream.head.ropc);
}

extern uint8_t* eo_rop_GetROPdata(EOrop *p)
{
    if(NULL == p)
    {
        return(NULL);
    }

    return(p->stream.data);
}

extern uint16_t eo_rop_ComputeSize(eOropconfig_t ropcfg, eOropcode_t ropc, uint16_t sizeofdata)
{
    uint16_t size = sizeof(eOrophead_t);
    eOrophead_t rophead = 
    {
        EO_INIT(.ctrl)  0,
        EO_INIT(.ropc)  ropc,
        EO_INIT(.endp)  0,
        EO_INIT(.nvid)  0,
        EO_INIT(.dsiz)  sizeofdata        
    };

    if( (eo_ropcode_none == ropc) || (eo_ropcode_usr == ropc) )
    {
        return(0);      
    }
    

    
    rophead.ctrl.ffu        = 0;
    rophead.ctrl.confinfo   = eo_ropconf_none;
    rophead.ctrl.plustime   = (eobool_true == ropcfg.plustime) ? (1) : (0);
    rophead.ctrl.plussign   = (eobool_true == ropcfg.plussign) ? (1) : (0);
    rophead.ctrl.rqsttime   = (eobool_true == ropcfg.timerqst) ? (1) : (0);
    rophead.ctrl.rqstconf   = (eobool_true == ropcfg.confrqst) ? (1) : (0);
    rophead.ctrl.userdefn   = 0;
    
    
    if(eobool_true == eo_rop_hid_DataField_is_Required(&rophead))
    {
        size += eo_rop_hid_DataField_EffectiveSize(rophead.dsiz);
    }

    if(1 == rophead.ctrl.plussign)
    {
        size += 4;
    }

    if(1 == rophead.ctrl.plustime)
    {
        size+= 8;
    }

    return(size);    
    
}





// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------

extern EOnv* eo_rop_hid_NV_Get(EOrop *p)
{
    if(NULL == p)
    {
        return(NULL);
    }

    return(&p->netvar);
}

// used when the rop is already built or when we already have a valid head from a stream
extern eObool_t eo_rop_hid_DataField_is_Present(const eOrophead_t *head)
{
    return( (0 != head->dsiz) ? (eobool_true) : (eobool_false));
}

// used when it is necessary to build a rop to transmit and teh dsiz field is not yet assigned
extern eObool_t eo_rop_hid_DataField_is_Required(const eOrophead_t *head)
{
    eObool_t ret = eobool_false;


    if((eo_ropconf_none == head->ctrl.confinfo) && 
       ((eo_ropcode_set == head->ropc) || (eo_ropcode_say == head->ropc) || (eo_ropcode_sig == head->ropc))
      )
    {
        ret = eobool_true;
    }

    return(ret);
}

// normal commands
// a simple node who only knows about its own netvars must use eo_nv_ownership_local
// when receives ask<>, set<>, rst<>, upd<>.
// a smart node who receives say<> and sig<> must search into eo_nv_ownership_remote.

// normal commands
// a simple node who only knows about its own netvars must use eo_nv_ownership_local
// when receives ask<>, set<>, rst<>, upd<>.
// a smart node who receives say<> and sig<> must search into eo_nv_ownership_remote.

// ack/nak commands
// the simple node just process: ack-nak-sig<>. 
// the smart node can also process: nak-ask<>, ack-nak-set<>, ack-nak-rst<>, ack-nak-upd<>

extern eOnvOwnership_t eo_rop_hid_GetOwnership(eOropcode_t ropc, eOropconfinfo_t confinfo, eOropDirection direction)
{
    eOnvOwnership_t ownership = eo_nv_ownership_local;

    // set direction for received direction
    if((eo_ropcode_ask == ropc) || (eo_ropcode_set == ropc) || (eo_ropcode_rst == ropc) || (eo_ropcode_upd == ropc))
    {
        ownership = eo_nv_ownership_local;
    }
    else // say, sig, 
    {
        ownership = eo_nv_ownership_remote;
    }

    // if direction is outgoing we exchange
    if(eo_rop_dir_outgoing == direction)
    {
        ownership = (eo_nv_ownership_local == ownership) ? (eo_nv_ownership_remote) : (eo_nv_ownership_local);
    }

    // if the ropc is a confirmation (ack/nak) we exchange again
    if(eo_ropconf_none != confinfo)
    {
        ownership = (eo_nv_ownership_local == ownership) ? (eo_nv_ownership_remote) : (eo_nv_ownership_local);
    }


    return(ownership);

}


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------



static EOrop * s_eo_rop_prepare_reply(EOrop *ropin, EOrop *ropout)
{
    // in here we fill the reply rop.

    // if the nvtreenoderoot is NULL (not existing) we reply only if ctrl.rqstconf is 1.    
    // if the nv is an existing one we reply only if (a) we have a ask or if (b) the ctrl.rqstconf is 1. 
    // in case of (a) we reply with a say.
    // in case of (b) we may reply with a nak/ack. ack if the rop can be performed, nak if it cannot be performed (or nv does not exist) 
    
    EOrop *r = NULL;

    if(NULL == ropin->netvar.treenode)
    {
        // we did not find a netvar w/ that nvid but if we have been asked for a confirmation we send a nak
        if(1 == ropin->stream.head.ctrl.rqstconf)
        {
            r = ropout;
            r->stream.head.ctrl.confinfo = eo_ropconf_nak;
            r->stream.head.endp = ropin->stream.head.endp;
            r->stream.head.ropc = ropin->stream.head.ropc;
            r->stream.head.dsiz = 0;
        }
        else
        {
            r = NULL;
        }

    }
    else if(eo_ropcode_ask == ropin->stream.head.ropc)
    {
        // we surely send a say
        r = ropout;
        r->stream.head.ropc = eo_ropcode_say;

        if(1 == ropin->stream.head.ctrl.rqstconf)
        {   // by default we assign a nak, but we SHALL transform it in a none if we process the ask later on
            r->stream.head.ctrl.confinfo = eo_ropconf_nak;  
        }
    }
    else if(1 == ropin->stream.head.ctrl.rqstconf)
    {
        // we may send a ack or a nak, depending on success of operation (insuccess is write on ro netvars). 
        // the matter is made more difficult for pkd variables of mix type (not leaves) which may contain ro and rw netvars.
        // for this reason we assign a nak ropcode, which we SHALL transform in a ack if at least one operation is successful.
        r = ropout;
        r->stream.head.ctrl.confinfo = eo_ropconf_nak;
        r->stream.head.ropc = ropin->stream.head.ropc;
    }

    // now complete the reply
    if(NULL != r)
    {
        // maybe we dont need to fill the tmpdata fields
        r->tmpdata.nvscfg              = ropin->tmpdata.nvscfg;
        r->tmpdata.ondevindex          = ropin->tmpdata.ondevindex;
        r->tmpdata.onendpointindex     = ropin->tmpdata.onendpointindex;
        
        // but we surely assign the same netvar, as the reply is about the very same netvar
        memcpy(&r->netvar, &ropin->netvar, sizeof(EOnv));


        //r->head.ctrl.userdef = 0;
        //r->head.ctrl.rqstconf = 0;
        //r->head.ctrl.rqsttime= 0;
        r->stream.head.ctrl.plussign = ropin->stream.head.ctrl.plussign;
        r->stream.head.ctrl.plustime = ropin->stream.head.ctrl.rqsttime;
        
        r->stream.head.endp = ropin->stream.head.endp;
        r->stream.head.dsiz = 0; // if it must be non-zero then some function shall fill it
        r->stream.head.nvid = ropin->stream.head.nvid;

        // nvid was filled earlier in this function
        // data will be filled by some other function
        r->stream.sign = (1 == r->stream.head.ctrl.plussign) ? (ropin->stream.sign) : (EOK_uint32dummy);
#ifndef EODEF_DONT_USE_THE_VSYSTEM
        r->stream.time = (1 == r->stream.head.ctrl.plustime) ? (eov_sys_LifeTimeGet(eov_sys_GetHandle())) : (EOK_uint64dummy);
#else
        r->stream.time = (1 == r->stream.head.ctrl.plustime) ? (0xf1f2f3f4f5f6f7f8) : (EOK_uint64dummy);
#endif
    }


    return(r);
}



static eObool_t s_eo_rop_can_surely_say_cannot_perform_on_device(EOrop *ropin)
{
    // we surely cannot do: 
    // (1) operations on a NULL nvtreenoderoot
    // (2) a write into a ro: inp or con
    // (3) an update to what is not updateable: con, cfg, beh.
    // on mix (but in general on not leaves) we cannot say. we also DO NOT filter out the sig/say operations

    eObool_t ret = eobool_false;

    // if the nvtreenoderoot was not found ... we surely cannot operate on that
    if(NULL == ropin->netvar.treenode)
    {
        return(eobool_true);
    }

    // on mix functionalities (but more in general on not leaves) we cannot say.
    //if(eobool_false == eo_treenode_isLeaf(ropin->tmpdata.nvtreenoderoot))
    if(eobool_false == ropin->netvar.isleaf)
    {
        return(eobool_false);
    }

    switch(ropin->stream.head.ropc)
    {
        case eo_ropcode_set:
        case eo_ropcode_rst:
        {
            ret = (eobool_false == eo_nv_hid_isWritable(&ropin->netvar)) ? eobool_true : eobool_false;
        } break;

        case eo_ropcode_upd:
        {
            ret = (eobool_false == eo_nv_hid_isUpdateable(&ropin->netvar)) ? eobool_true : eobool_false;
        }  break;

        default:
        {
        } break;
    }


    return(ret);
}


static void s_eo_rop_received_nonleaf_descend_to_leaves_and_exec(EOrop *rop_in, EOtreenode *nvtreenoderoot, EOrop *rop_o)
{
    uint8_t i = 0;
    uint8_t numberofchildren = eo_treenode_GetNumberOfChilden(nvtreenoderoot);

   
    // if it is a leaf nvtreenoderoot ... exec the ropcode on actual data
    if(0 == numberofchildren)
    {
        s_eo_rop_received_nonleaf_but_reached_leaf_and_exec_on_it(rop_in, nvtreenoderoot, rop_o);
    }
    else
    {
        // go down the tree
        for(i=0; i<numberofchildren; i++)
        {
            // acemor says: recursion is wild
            s_eo_rop_received_nonleaf_descend_to_leaves_and_exec(rop_in, eo_treenode_GetChild(nvtreenoderoot, i), rop_o);
        }
    }
} 



// on a received rop ...
static void s_eo_rop_received_nonleaf_but_reached_leaf_and_exec_on_it(EOrop *rop_in, EOtreenode *nvtreeleaf, EOrop *rop_o)
{
    eOresult_t res = eores_NOK_generic;
    const uint8_t *source = NULL;
    uint8_t *destin = NULL;
    uint16_t size = 0;
    uint16_t capacity = 0;
    EOnv *leaf = NULL;

    if(nvtreeleaf == rop_in->netvar.treenode)
    {
        // we use the already computed netvar
        eo_errman_Error(eo_errman_GetHandle(), eo_errortype_fatal, s_eobj_ownname, "illegal path");
        leaf = &rop_in->netvar;
    }
    else
    {
        // we need to retrieve the leaf
        leaf = eo_nvscfg_GetNV(rop_in->tmpdata.nvscfg, rop_in->tmpdata.ondevindex, rop_in->tmpdata.onendpointindex, eo_treenode_GetIndex(nvtreeleaf), nvtreeleaf, NULL);
    }

    switch(rop_in->stream.head.ropc)
    {
        case eo_ropcode_set:
        case eo_ropcode_rst:
        {   // the receiver owns the nv locally

            // in here we manage operations which ask to write a value into the netvar.
            // the value can be contained in the data field of the remote operation (set)
            // or can be taken from a constant local memory location (rst).
            // the destination is always a ram location and in addition can also be an eeprom location.
            // the writing operation is done only if the netvar has writeable properties.
            // we have a reply rop only if there is an explicit ack/nak request
            // also ... for updateable netvars (in our case vars out), we also call the update function to propagate
            // the value to the peripheral


            // get the source and increment its index. also get the size of the the source.
            // the size is always the capacity stored in the leaf except for set when typ is arr-m, where m is
            // stored in the first two bytes of payload and size is 2 + m.
            // dont use rop_in->datasize as it does not work for nested data

            if(eo_ropcode_rst == rop_in->stream.head.ropc)
            {   // rst

                // just reset the leaf without forcing anything.
                res = eo_nv_ResetTS(leaf, eobool_false, eo_nv_upd_ifneeded, rop_in->stream.time, rop_in->stream.sign); //eObool_t forcerst,           
            
                // and dont increment the progressive index of rop_in because reset does not have any payload
            }
            else 
            {   // set
                
                // get the data to be set and its size.
                source = rop_in->stream.data + rop_in->curindexofstreamdatafield;

                // set the leaf to the value without forcing anything
                res = eo_nv_SetTS(leaf, source, eobool_false, eo_nv_upd_ifneeded, rop_in->stream.time, rop_in->stream.sign); //eObool_t forceset,

                // and increment the progressive index even if the source was not written to destination
                // (the reason is that the sender can fill the packet with data that can be read-only)
                // we use capacity because it is always equal to size except for teh case of arrays.
                // but if arrays are transmitted inside a MIX, then they are copied entirely in their capacity
                capacity = eo_nv_Capacity(leaf);
                rop_in->curindexofstreamdatafield += capacity;
            }


            // if any set or rst was succesful, ... fill a possible reply 

            if(eores_OK == res)  
            {                
                // mark the ropcode of the reply (if any) to be ack
                // for pkd data of type mix, we send an ack if at least one netvar is writeable.
                if((1 == rop_in->stream.head.ctrl.rqstconf) && (NULL != rop_o))
                {
                    rop_o->stream.head.ctrl.confinfo = eo_ropconf_ack;
                } 
            }

        } break;


        case eo_ropcode_upd:
        {   // the receiver owns the nv locally

            // in here we manage an operation which ask to refresh the ram value of the netvar (if input) or to
            // move its value to the peripheral (if ouput).
            // we can do that only if the netvar is updateable: input or output.
            // for an input variable this operation means to get the value of the peripheral and copy into ram value
            // for an output variable it is instead required to refresh the value of the peripheral using the ram value.
            // these operations are done using the update() function, to which we give every responsibility
            // we have a reply rop only if there is an explicit ack/nak request.

            res = eo_nv_UpdateTS(leaf, rop_in->stream.time, rop_in->stream.sign);

            if(eores_OK == res)
            {
                if((1 == rop_in->stream.head.ctrl.rqstconf) && (NULL != rop_o))
                {
                    rop_o->stream.head.ctrl.confinfo = eo_ropconf_ack;
                } 
            }

        } break; 
        
        
        case eo_ropcode_ask:
        {   // the receiver owns the nv locally
            
            // in here we manage operations which ask to retrieve the value of the netvar and to create a reply rop.
            // the value for reply rop is taken from ram



            // we use datasize and not the progressive index because by doing so we update the rop_o as soon as we write
            destin = rop_o->stream.data + rop_o->stream.head.dsiz;

            eo_nv_Get(leaf, eo_nv_strg_volatile, destin, &size);

            // datasize is incremented by capacity. 
            rop_o->stream.head.dsiz += eo_nv_Capacity(leaf);


            // never mark it as a nak or ack, even if we received the confirmation request.
            // the confirmation is the reply message itself 
            rop_o->stream.head.ctrl.confinfo = eo_ropconf_none;

        } break; 
                     

        case eo_ropcode_sig:
        case eo_ropcode_say:
        {   // the receiver owns the nv remotely.


            // in here we need to copy the received data into the volatile data of the netvar.
            // then, the processing of such data is done in the appropriate update(), fn_update() or fn_after_rop() function 
    


            // force write also if an input, force update. we never do storage on eeprom just because remote nvs shall not
            // have valid eeprom addresses
            source = rop_in->stream.data + rop_in->curindexofstreamdatafield;
            eo_nv_remoteSetTS(leaf, source, eo_nv_upd_always, rop_in->stream.time, rop_in->stream.sign);
            //eo_nv_Set(leaf, source, eobool_true, eo_nv_upd_always);

            // i increment the progressive index of the rop_in
            //size = eo_nv_Size(leaf, source);
            rop_in->curindexofstreamdatafield += eo_nv_Capacity(leaf);


            // in here we manage the case in which the sig message that we have received required an ack.
            if((1 == rop_in->stream.head.ctrl.rqstconf) && (NULL != rop_o) && (eo_ropcode_sig == rop_in->stream.head.ropc))
            {
                rop_o->stream.head.ctrl.confinfo = eo_ropconf_ack;
            } 
          

        } break;

                  
        default:
        {
        } break;
    }


}

static void s_eo_rop_received_nonleaf_exec_on_it(EOrop *rop_in, EOrop *rop_o)
{
    s_eo_rop_received_leaf_exec_on_it(rop_in, rop_o);
}

static void s_eo_rop_received_leaf_exec_on_it(EOrop *rop_in, EOrop *rop_o)
{
    eOresult_t res = eores_NOK_generic;
    const uint8_t *source = NULL;
    uint8_t *destin = NULL;
    uint16_t size = 0;
    EOnv *thenv = NULL;

    thenv = &rop_in->netvar;


    switch(rop_in->stream.head.ropc)
    {
        case eo_ropcode_set:
        case eo_ropcode_rst:
        {   // the receiver owns the nv locally

            // in here we manage operations which ask to write a value into the netvar.
            // the value can be contained in the data field of the remote operation (set)
            // or can be taken from a constant local memory location (rst).
            // the destination is always a ram location and in addition can also be an eeprom location.
            // the writing operation is done only if the netvar has writeable properties.
            // we have a reply rop only if there is an explicit ack/nak request
            // also ... for updateable netvars (in our case vars out), we also call the update function to propagate
            // the value to the peripheral


            // get the source and increment its index. also get the size of the the source.
            // the size is always the capacity stored in the thenv except for set when typ is arr-m, where m is
            // stored in the first two bytes of payload and size is 2 + m.
            // dont use rop_in->datasize as it does not work for nested data

            if(eo_ropcode_rst == rop_in->stream.head.ropc)
            {   // rst

                // just reset the thenv without forcing anything.
                res = eo_nv_ResetTS(thenv, eobool_false, eo_nv_upd_ifneeded, rop_in->stream.time, rop_in->stream.sign); //eObool_t forcerst,           
            
                // and dont increment the progressive index of rop_in because reset does not have any payload
            }
            else 
            {   // set
                
                // get the data to be set and its size.
                source = rop_in->stream.data;

                // set the thenv to the value without forcing anything
                res = eo_nv_SetTS(thenv, source, eobool_false, eo_nv_upd_ifneeded, rop_in->stream.time, rop_in->stream.sign); //eObool_t forceset,

                // and increment the progressive index even if the source was not written to destination
                // (the reason is that the sender can fill the packet with data that can be read-only)
//                size = eo_nv_Size(thenv, source);
//                rop_in->aboutdata.index += size;
            }


            // if any set or rst was succesful, ... fill a possible reply 

            if(eores_OK == res)  
            {                
                // mark the ropcode of the reply (if any) to be ack
                // for pkd data of type mix, we send an ack if at least one netvar is writeable.
                if((1 == rop_in->stream.head.ctrl.rqstconf) && (NULL != rop_o))
                {
                    rop_o->stream.head.ctrl.confinfo = eo_ropconf_ack;
                } 
            }

        } break;


        case eo_ropcode_upd:
        {   // the receiver owns the nv locally

            // in here we manage an operation which ask to refresh the ram value of the netvar (if input) or to
            // move its value to the peripheral (if ouput).
            // we can do that only if the netvar is updateable: input or output.
            // for an input variable this operation means to get the value of the peripheral and copy into ram value
            // for an output variable it is instead required to refresh the value of the peripheral using the ram value.
            // these operations are done using the update() function, to which we give every responsibility
            // we have a reply rop only if there is an explicit ack/nak request.

            res = eo_nv_Update(thenv);

            if(eores_OK == res)
            {
                if((1 == rop_in->stream.head.ctrl.rqstconf) && (NULL != rop_o))
                {
                    rop_o->stream.head.ctrl.confinfo = eo_ropconf_ack;
                } 
            }

        } break; 
        
        
        case eo_ropcode_ask:
        {   // the receiver owns the nv locally
            
            // in here we manage operations which ask to retrieve the value of the netvar and to create a reply rop.
            // the value for reply rop is taken from ram



            // we use datasize and not the progressive index because by doing so we update the rop_o as soon as we write
            destin = rop_o->stream.data;

            eo_nv_Get(thenv, eo_nv_strg_volatile, destin, &size);

            // datasize is incremented by size. 
            // size is always equal to the capacity except for netvar of type arr-m,
            // for which m is contained in the first two bytes of the data and size is 2 + m. 
            rop_o->stream.head.dsiz += size;


            // never mark it as a nak or ack, even if we received the confirmation request.
            // the confirmation is the reply message itself 
            rop_o->stream.head.ctrl.confinfo = eo_ropconf_none;

        } break; 
                     

        case eo_ropcode_sig:
        case eo_ropcode_say:
        {   // the receiver owns the nv remotely.


            // in here we need to copy the received data into the volatile data of the netvar.
            // then, the processing of such data is done in the appropriate update(), fn_update() or fn_after_rop() function 
    


            // force write also if an input, force update. we never do storage on eeprom just because remote nvs shall not
            // have valid eeprom addresses
            source = rop_in->stream.data;
            eo_nv_remoteSetTS(thenv, source, eo_nv_upd_always, rop_in->stream.time, rop_in->stream.sign);

            // i increment the progressive index of the rop_in
//            size = eo_nv_Size(thenv, source);
//            rop_in->aboutdata.index += size;


            // in here we manage the case in which the sig message that we have received required an ack.
            if((1 == rop_in->stream.head.ctrl.rqstconf) && (NULL != rop_o) && (eo_ropcode_sig == rop_in->stream.head.ropc))
            {
                rop_o->stream.head.ctrl.confinfo = eo_ropconf_ack;
            } 
          

        } break;

                  
        default:
        {
        } break;
    }


}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




