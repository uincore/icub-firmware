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

/* @file       eOcfg_nvsEP_mc_any_con_jxxmacro.c
    @brief      This file keeps constant configuration for ...
    @author     marco.accame@iit.it
    @date       04/06/2012
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h" 
#include "string.h"
#include "stdio.h"

#include "EoMotionControl.h"
#include "eOcfg_nvsEP_mc_any_con_jxx.h"    
#include "eOcfg_nvsEP_mc_any_con_jxxdefault.h"
      




// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

//#include "eOcfg_nvsEP_mc_any_con_jxxmacro.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------

// the first xxname() does not works.
// see why at http://gcc.gnu.org/onlinedocs/cpp/Argument-Prescan.html#Argument-Prescan
//#define MACRO_NAMEOFVARIABLE(pstr, jstr, var)            eo_cfg_nvsEP_mc ## pstr ## jstr ## var 

#define xx0nameofvariable(pstr, jstr, var)                  eo_cfg_nvsEP_mc ## pstr ## jstr ## var
#define MACRO_NAMEOFVARIABLE(pstr, jstr, var)               xx0nameofvariable(pstr, jstr, var)

#define xx0verifysizeof(id, sname, ssize)                   typedef uint8_t GUARD##id##sname[ ( ssize == sizeof(sname) ) ? (1) : (0)];
#define MACRO_VERIFYSIZEOF(id, sname, ssize)                xx0verifysizeof(id, sname, ssize)

#define xx0getnvid(extprefix, postfix, j)                   extprefix##postfix(j)
#define MACRO_GETNVID(extprefix, postfix, j)                xx0getnvid(extprefix, postfix, j)



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------



#define OFFSETof_jconfig                                        (JMACRO_JOFF)
#define CAPACITY_jconfig                                        sizeof(eOmc_joint_config_t)
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jconfig) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jconfig, JMACRO_JNUM),
    EO_INIT(.capacity)  CAPACITY_jconfig,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.config,
    EO_INIT(.offset)    OFFSETof_jconfig,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig)
};
#define OFFSETafter_jconfig                                     (OFFSETof_jconfig+CAPACITY_jconfig)


#define OFFSETof_jconfig__pidposition                           (OFFSETof_jconfig)
#define CAPACITY_jconfig__pidposition                           sizeof(eOmc_PID_t)
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jconfig__pidposition) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jconfig__pidposition, JMACRO_JNUM), 
    EO_INIT(.capacity)  CAPACITY_jconfig__pidposition,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.config.pidposition,
    EO_INIT(.offset)    OFFSETof_jconfig__pidposition,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__pidposition),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__pidposition)
};
#define OFFSETafter_jconfig__pidposition                        (OFFSETof_jconfig__pidposition+CAPACITY_jconfig__pidposition)


#define OFFSETof_jconfig__pidvelocity                           (OFFSETafter_jconfig__pidposition)
#define CAPACITY_jconfig__pidvelocity                           sizeof(eOmc_PID_t)
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jconfig__pidvelocity) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jconfig__pidvelocity, JMACRO_JNUM),       
    EO_INIT(.capacity)  CAPACITY_jconfig__pidvelocity,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.config.pidvelocity,
    EO_INIT(.offset)    OFFSETof_jconfig__pidvelocity,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__pidvelocity),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__pidvelocity)
};
#define OFFSETafter_jconfig__pidvelocity                        (OFFSETof_jconfig__pidvelocity+CAPACITY_jconfig__pidvelocity)


#define OFFSETof_jconfig__pidtorque                             (OFFSETafter_jconfig__pidvelocity)
#define CAPACITY_jconfig__pidtorque                             sizeof(eOmc_PID_t)
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jconfig__pidtorque) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jconfig__pidtorque, JMACRO_JNUM),    
    EO_INIT(.capacity)  CAPACITY_jconfig__pidtorque,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.config.pidtorque,
    EO_INIT(.offset)    OFFSETof_jconfig__pidtorque,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__pidtorque),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__pidtorque)
};
#define OFFSETafter_jconfig__pidtorque                          (OFFSETof_jconfig__pidtorque+CAPACITY_jconfig__pidtorque)



#define OFFSETof_jconfig__impedance                             (OFFSETafter_jconfig__pidtorque)
#define CAPACITY_jconfig__impedance                             sizeof(eOmc_impedance_t)
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jconfig__impedance) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jconfig__impedance, JMACRO_JNUM),    
    EO_INIT(.capacity)  CAPACITY_jconfig__impedance,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.config.impedance,
    EO_INIT(.offset)    OFFSETof_jconfig__impedance,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__impedance),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__impedance)
};
#define OFFSETafter_jconfig__impedance                          (OFFSETof_jconfig__impedance+CAPACITY_jconfig__impedance)



#define OFFSETof_jconfig__minpositionofjoint                    (OFFSETafter_jconfig__impedance)
#define CAPACITY_jconfig__minpositionofjoint                    sizeof(eOmeas_position_t)
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jconfig__minpositionofjoint) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jconfig__minpositionofjoint, JMACRO_JNUM),    
    EO_INIT(.capacity)  CAPACITY_jconfig__minpositionofjoint,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.config.minpositionofjoint,
    EO_INIT(.offset)    OFFSETof_jconfig__minpositionofjoint,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__minpositionofjoint),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__minpositionofjoint)
};
#define OFFSETafter_jconfig__minpositionofjoint                (OFFSETof_jconfig__minpositionofjoint+CAPACITY_jconfig__minpositionofjoint)


#define OFFSETof_jconfig__maxpositionofjoint                    (OFFSETafter_jconfig__minpositionofjoint)
#define CAPACITY_jconfig__maxpositionofjoint                    sizeof(eOmeas_position_t)
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jconfig__maxpositionofjoint) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jconfig__maxpositionofjoint, JMACRO_JNUM),
    EO_INIT(.capacity)  CAPACITY_jconfig__maxpositionofjoint,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.config.maxpositionofjoint,
    EO_INIT(.offset)    OFFSETof_jconfig__maxpositionofjoint,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__maxpositionofjoint),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__maxpositionofjoint)
};
#define OFFSETafter_jconfig__maxpositionofjoint                (OFFSETof_jconfig__maxpositionofjoint+CAPACITY_jconfig__maxpositionofjoint)




#define OFFSETof_jconfig__velocitysetpointtimeout              (OFFSETafter_jconfig__maxpositionofjoint)
#define CAPACITY_jconfig__velocitysetpointtimeout              sizeof(eOmeas_time_t)
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jconfig__velocitysetpointtimeout) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jconfig__velocitysetpointtimeout, JMACRO_JNUM),
    EO_INIT(.capacity)  CAPACITY_jconfig__velocitysetpointtimeout,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.config.velocitysetpointtimeout,
    EO_INIT(.offset)    OFFSETof_jconfig__velocitysetpointtimeout,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__velocitysetpointtimeout),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__velocitysetpointtimeout)
};
#define OFFSETafter_jconfig__velocitysetpointtimeout            (OFFSETof_jconfig__velocitysetpointtimeout+CAPACITY_jconfig__velocitysetpointtimeout)


#define OFFSETof_jconfig__holder01FFU00            (OFFSETafter_jconfig__velocitysetpointtimeout)
#define CAPACITY_jconfig__holder01FFU00            (sizeof(eOenum08_t))
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jconfig__holder01FFU00) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jconfig__holder01FFU00, JMACRO_JNUM),    
    EO_INIT(.capacity)  CAPACITY_jconfig__holder01FFU00,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.config.holder01FFU00,
    EO_INIT(.offset)    OFFSETof_jconfig__holder01FFU00,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__holder01FFU00),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__holder01FFU00)
};
#define OFFSETafter_jconfig__holder01FFU00          (OFFSETof_jconfig__holder01FFU00+CAPACITY_jconfig__holder01FFU00)


#define OFFSETof_jconfig__motionmonitormode            (OFFSETafter_jconfig__holder01FFU00)
#define CAPACITY_jconfig__motionmonitormode            (sizeof(eOenum08_t))
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jconfig__motionmonitormode) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jconfig__motionmonitormode, JMACRO_JNUM),    
    EO_INIT(.capacity)  CAPACITY_jconfig__motionmonitormode,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.config.motionmonitormode,
    EO_INIT(.offset)    OFFSETof_jconfig__motionmonitormode,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__motionmonitormode),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__motionmonitormode)
};
#define OFFSETafter_jconfig__motionmonitormode          (OFFSETof_jconfig__motionmonitormode+CAPACITY_jconfig__motionmonitormode)


#define OFFSETof_jconfig__encoderconversionfactor            (OFFSETafter_jconfig__motionmonitormode)
#define CAPACITY_jconfig__encoderconversionfactor            (sizeof(eOutil_emulfloat32_t))
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jconfig__encoderconversionfactor) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jconfig__encoderconversionfactor, JMACRO_JNUM),    
    EO_INIT(.capacity)  CAPACITY_jconfig__encoderconversionfactor,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.config.encoderconversionfactor,
    EO_INIT(.offset)    OFFSETof_jconfig__encoderconversionfactor,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__encoderconversionfactor),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__encoderconversionfactor)
};
#define OFFSETafter_jconfig__encoderconversionfactor          (OFFSETof_jconfig__encoderconversionfactor+CAPACITY_jconfig__encoderconversionfactor)

#define OFFSETof_jconfig__encoderconversionoffset            (OFFSETafter_jconfig__encoderconversionfactor)
#define CAPACITY_jconfig__encoderconversionoffset            (sizeof(eOutil_emulfloat32_t))
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jconfig__encoderconversionoffset) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jconfig__encoderconversionoffset, JMACRO_JNUM),    
    EO_INIT(.capacity)  CAPACITY_jconfig__encoderconversionoffset,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.config.encoderconversionoffset,
    EO_INIT(.offset)    OFFSETof_jconfig__encoderconversionoffset,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__encoderconversionoffset),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__encoderconversionoffset)
};
#define OFFSETafter_jconfig__encoderconversionoffset          (OFFSETof_jconfig__encoderconversionoffset+CAPACITY_jconfig__encoderconversionoffset)


#define OFFSETof_jconfig__des02FORjstatuschamaleon04            (OFFSETafter_jconfig__encoderconversionoffset)
#define CAPACITY_jconfig__des02FORjstatuschamaleon04            (2*sizeof(eOutil_chameleon_descr_t))
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jconfig__des02FORjstatuschamaleon04) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jconfig__des02FORjstatuschamaleon04, JMACRO_JNUM),    
    EO_INIT(.capacity)  CAPACITY_jconfig__des02FORjstatuschamaleon04,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.config.des02FORjstatuschamaleon04,
    EO_INIT(.offset)    OFFSETof_jconfig__des02FORjstatuschamaleon04,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__des02FORjstatuschamaleon04),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__des02FORjstatuschamaleon04)
};
#define OFFSETafter_jconfig__des02FORjstatuschamaleon04          (OFFSETof_jconfig__des02FORjstatuschamaleon04+CAPACITY_jconfig__des02FORjstatuschamaleon04)


#define OFFSETof_jconfig__holder01FFU01            (OFFSETafter_jconfig__des02FORjstatuschamaleon04)
#define CAPACITY_jconfig__holder01FFU01            (1)
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jconfig__holder01FFU01) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jconfig__holder01FFU01, JMACRO_JNUM),    
    EO_INIT(.capacity)  CAPACITY_jconfig__holder01FFU01,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.config.holder01FFU01,
    EO_INIT(.offset)    OFFSETof_jconfig__holder01FFU01,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__holder01FFU01),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__holder01FFU01)
};
#define OFFSETafter_jconfig__holder01FFU01          (OFFSETof_jconfig__holder01FFU01+CAPACITY_jconfig__holder01FFU01)


#define OFFSETof_jconfig__holder01FFU02            (OFFSETafter_jconfig__holder01FFU01)
#define CAPACITY_jconfig__holder01FFU02            (1)
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jconfig__holder01FFU02) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jconfig__holder01FFU02, JMACRO_JNUM),    
    EO_INIT(.capacity)  CAPACITY_jconfig__holder01FFU02,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.config.holder01FFU02,
    EO_INIT(.offset)    OFFSETof_jconfig__holder01FFU02,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__holder01FFU02),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__holder01FFU02)
};
#define OFFSETafter_jconfig__holder01FFU02          (OFFSETof_jconfig__holder01FFU02+CAPACITY_jconfig__holder01FFU02)

#define OFFSETof_jconfig__holder02FFU03            (OFFSETafter_jconfig__holder01FFU02)
#define CAPACITY_jconfig__holder02FFU03            (2)
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jconfig__holder02FFU03) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jconfig__holder02FFU03, JMACRO_JNUM),    
    EO_INIT(.capacity)  CAPACITY_jconfig__holder02FFU03,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.config.holder02FFU03,
    EO_INIT(.offset)    OFFSETof_jconfig__holder02FFU03,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__holder02FFU03),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__holder02FFU03)
};
#define OFFSETafter_jconfig__holder02FFU03          (OFFSETof_jconfig__holder02FFU03+CAPACITY_jconfig__holder02FFU03)

#define OFFSETof_jconfig__holder02FFU04            (OFFSETafter_jconfig__holder02FFU03)
#define CAPACITY_jconfig__holder02FFU04            (2)
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jconfig__holder02FFU04) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jconfig__holder02FFU04, JMACRO_JNUM),    
    EO_INIT(.capacity)  CAPACITY_jconfig__holder02FFU04,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.config.holder02FFU04,
    EO_INIT(.offset)    OFFSETof_jconfig__holder02FFU04,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__holder02FFU04),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jconfig__holder02FFU04)
};
#define OFFSETafter_jconfig__holder02FFU04          (OFFSETof_jconfig__holder02FFU04+CAPACITY_jconfig__holder02FFU04)


//EO_VERIFYproposition(xxx, (OFFSETafter_jconfig__motionmonitormode-JMACRO_JOFF) == (16+16+16+12+4+4+2+1+1) );

// guard on alignement of variables. if it doesnt compile then ... there are holes
MACRO_VERIFYSIZEOF(JMACRO_JNUM, eOmc_joint_config_t, OFFSETafter_jconfig__holder02FFU04-JMACRO_JOFF);



#define OFFSETof_jstatus                                        (OFFSETafter_jconfig)
#define CAPACITY_jstatus                                        sizeof(eOmc_joint_status_t)
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jstatus) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jstatus, JMACRO_JNUM),
    EO_INIT(.capacity)  CAPACITY_jstatus,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.status,
    EO_INIT(.offset)    OFFSETof_jstatus,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jstatus),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jstatus)
};
#define OFFSETafter_jstatus                                     (OFFSETof_jstatus+CAPACITY_jstatus)


#define OFFSETof_jstatus__basic                                 (OFFSETafter_jconfig)
#define CAPACITY_jstatus__basic                                 sizeof(eOmc_joint_status_basic_t)
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jstatus__basic) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jstatus__basic, JMACRO_JNUM),
    EO_INIT(.capacity)  CAPACITY_jstatus__basic,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.status.basic,
    EO_INIT(.offset)    OFFSETof_jstatus__basic,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jstatus__basic),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jstatus__basic)
};
#define OFFSETafter_jstatus__basic                                     (OFFSETof_jstatus__basic+CAPACITY_jstatus__basic)

#define OFFSETof_jstatus__ofpid                                 (OFFSETafter_jstatus__basic)
#define CAPACITY_jstatus__ofpid                                 sizeof(eOmc_joint_status_ofpid_t)
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jstatus__ofpid) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jstatus__ofpid, JMACRO_JNUM),
    EO_INIT(.capacity)  CAPACITY_jstatus__ofpid,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.status.ofpid,
    EO_INIT(.offset)    OFFSETof_jstatus__ofpid,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jstatus__ofpid),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jstatus__ofpid)
};
#define OFFSETafter_jstatus__ofpid                                     (OFFSETof_jstatus__ofpid+CAPACITY_jstatus__ofpid)


#define OFFSETof_jstatus__chamaleon04                                 (OFFSETafter_jstatus__ofpid)
#define CAPACITY_jstatus__chamaleon04                                 (4)
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jstatus__chamaleon04) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jstatus__chamaleon04, JMACRO_JNUM),
    EO_INIT(.capacity)  CAPACITY_jstatus__chamaleon04,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.status.chamaleon04,
    EO_INIT(.offset)    OFFSETof_jstatus__chamaleon04,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jstatus__chamaleon04),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jstatus__chamaleon04)
};
#define OFFSETafter_jstatus__chamaleon04                                     (OFFSETof_jstatus__chamaleon04+CAPACITY_jstatus__chamaleon04)


// guard on alignement of variables. if it doesnt compile then ... the compiler has surely inserted some holes
//MACRO_VERIFYSIZEOF(JMACRO_JNUM, eOmc_joint_status_t, OFFSETafter_jstatus-OFFSETafter_jconfig__filler04-JMACRO_JOFF);

#define OFFSETof_jinputs                                        (OFFSETafter_jstatus)
#define CAPACITY_jinputs                                        sizeof(eOmc_joint_inputs_t)
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jinputs) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jinputs, JMACRO_JNUM),
    EO_INIT(.capacity)  CAPACITY_jinputs,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.inputs,
    EO_INIT(.offset)    OFFSETof_jinputs,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jinputs),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jinputs)
};
#define OFFSETafter_jinputs                                     (OFFSETof_jinputs+CAPACITY_jinputs)


#define OFFSETof_jinputs__externallymeasuredtorque                  (OFFSETafter_jstatus)
#define CAPACITY_jinputs__externallymeasuredtorque              sizeof(eOmeas_torque_t)
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jinputs__externallymeasuredtorque) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jinputs__externallymeasuredtorque, JMACRO_JNUM),
    EO_INIT(.capacity)  CAPACITY_jinputs__externallymeasuredtorque,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.inputs.externallymeasuredtorque,
    EO_INIT(.offset)    OFFSETof_jinputs__externallymeasuredtorque,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jinputs__externallymeasuredtorque),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jinputs__externallymeasuredtorque)
};
#define OFFSETafter_jinputs__externallymeasuredtorque           (OFFSETof_jinputs__externallymeasuredtorque+CAPACITY_jinputs__externallymeasuredtorque)


#define OFFSETof_jinputs__holder02FFU01                  (OFFSETafter_jinputs__externallymeasuredtorque)
#define CAPACITY_jinputs__holder02FFU01              (2)
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jinputs__holder02FFU01) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jinputs__holder02FFU01, JMACRO_JNUM),
    EO_INIT(.capacity)  CAPACITY_jinputs__holder02FFU01,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.inputs.holder02FFU01,
    EO_INIT(.offset)    OFFSETof_jinputs__holder02FFU01,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jinputs__holder02FFU01),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jinputs__holder02FFU01)
};
#define OFFSETafter_jinputs__holder02FFU01           (OFFSETof_jinputs__holder02FFU01+CAPACITY_jinputs__holder02FFU01)


#define OFFSETof_jinputs__holder04FFU02                  (OFFSETafter_jinputs__holder02FFU01)
#define CAPACITY_jinputs__holder04FFU02              (4)
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jinputs__holder04FFU02) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jinputs__holder04FFU02, JMACRO_JNUM),
    EO_INIT(.capacity)  CAPACITY_jinputs__holder04FFU02,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.inputs.holder04FFU02,
    EO_INIT(.offset)    OFFSETof_jinputs__holder04FFU02,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jinputs__holder04FFU02),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jinputs__holder04FFU02)
};
#define OFFSETafter_jinputs__holder04FFU02           (OFFSETof_jinputs__holder04FFU02+CAPACITY_jinputs__holder04FFU02)

//#define OFFSETafter_jinputs                          OFFSETafter_jinputs__holder04FFU02

#define OFFSETof_jcmmnds__calibration                  (OFFSETafter_jinputs)
#define CAPACITY_jcmmnds__calibration                   (sizeof(eOmc_calibrator_t))
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jcmmnds__calibration) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jcmmnds__calibration, JMACRO_JNUM),
    EO_INIT(.capacity)  CAPACITY_jcmmnds__calibration,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.cmmnds.calibration,
    EO_INIT(.offset)    OFFSETof_jcmmnds__calibration,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jcmmnds__calibration),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jcmmnds__calibration)
};
#define OFFSETafter_jcmmnds__calibration           (OFFSETof_jcmmnds__calibration+CAPACITY_jcmmnds__calibration)



#define OFFSETof_jcmmnds__setpoint                  (OFFSETafter_jcmmnds__calibration)
#define CAPACITY_jcmmnds__setpoint                   (sizeof(eOmc_setpoint_t))
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jcmmnds__setpoint) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jcmmnds__setpoint, JMACRO_JNUM),
    EO_INIT(.capacity)  CAPACITY_jcmmnds__setpoint,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.cmmnds.setpoint,
    EO_INIT(.offset)    OFFSETof_jcmmnds__setpoint,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jcmmnds__setpoint),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jcmmnds__setpoint)
};
#define OFFSETafter_jcmmnds__setpoint           (OFFSETof_jcmmnds__setpoint+CAPACITY_jcmmnds__setpoint)


#define OFFSETof_jcmmnds__stoptrajectory                  (OFFSETafter_jcmmnds__setpoint)
#define CAPACITY_jcmmnds__stoptrajectory                   (sizeof(eObool_t))
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jcmmnds__stoptrajectory) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jcmmnds__stoptrajectory, JMACRO_JNUM),
    EO_INIT(.capacity)  CAPACITY_jcmmnds__stoptrajectory,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.cmmnds.stoptrajectory,
    EO_INIT(.offset)    OFFSETof_jcmmnds__stoptrajectory,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jcmmnds__stoptrajectory),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jcmmnds__stoptrajectory)
};
#define OFFSETafter_jcmmnds__stoptrajectory           (OFFSETof_jcmmnds__stoptrajectory+CAPACITY_jcmmnds__stoptrajectory)


#define OFFSETof_jcmmnds__controlmode                  (OFFSETafter_jcmmnds__stoptrajectory)
#define CAPACITY_jcmmnds__controlmode                   (1)
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jcmmnds__controlmode) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jcmmnds__controlmode, JMACRO_JNUM),
    EO_INIT(.capacity)  CAPACITY_jcmmnds__controlmode,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.cmmnds.controlmode,
    EO_INIT(.offset)    OFFSETof_jcmmnds__controlmode,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jcmmnds__controlmode),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jcmmnds__controlmode)
};
#define OFFSETafter_jcmmnds__controlmode           (OFFSETof_jcmmnds__controlmode+CAPACITY_jcmmnds__controlmode)


#define OFFSETof_jcmmnds__holder01FFU02                  (OFFSETafter_jcmmnds__controlmode)
#define CAPACITY_jcmmnds__holder01FFU02                   (1)
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jcmmnds__holder01FFU02) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jcmmnds__holder01FFU02, JMACRO_JNUM),
    EO_INIT(.capacity)  CAPACITY_jcmmnds__holder01FFU02,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.cmmnds.holder01FFU02,
    EO_INIT(.offset)    OFFSETof_jcmmnds__holder01FFU02,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jcmmnds__holder01FFU02),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jcmmnds__holder01FFU02)
};
#define OFFSETafter_jcmmnds__holder01FFU02           (OFFSETof_jcmmnds__holder01FFU02+CAPACITY_jcmmnds__holder01FFU02)

#define OFFSETof_jcmmnds__holder01FFU03                  (OFFSETafter_jcmmnds__holder01FFU02)
#define CAPACITY_jcmmnds__holder01FFU03                   (1)
EOnv_con_t MACRO_NAMEOFVARIABLE(JMACRO_PSTR, JMACRO_JSTR, _jcmmnds__holder01FFU03) =
{   
    EO_INIT(.id)        MACRO_GETNVID(JMACRO_EXTERNALPREFIX_GETID, _jcmmnds__holder01FFU03, JMACRO_JNUM),
    EO_INIT(.capacity)  CAPACITY_jcmmnds__holder01FFU03,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mc_any_con_jxxdefault_defaultvalue.cmmnds.holder01FFU03,
    EO_INIT(.offset)    OFFSETof_jcmmnds__holder01FFU03,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jcmmnds__holder01FFU03),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mc_any_con_jxx_NVFUNTYP_jcmmnds__holder01FFU03)
};
#define OFFSETafter_jcmmnds__holder01FFU03           (OFFSETof_jcmmnds__holder01FFU03+CAPACITY_jcmmnds__holder01FFU03)


// guard on alignement of variables. if it doesnt compile then ... the compiler has surely inserted some holes
MACRO_VERIFYSIZEOF(JMACRO_JNUM, eOmc_joint_t, OFFSETafter_jcmmnds__holder01FFU03-JMACRO_JOFF);



#undef JMACRO_PSTR 
#undef JMACRO_JSTR 
#undef JMACRO_PNUM
#undef JMACRO_JNUM  
#undef JMACRO_JOFF

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------







// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------
