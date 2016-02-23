#ifndef MC_CONTROLLER___
#define MC_CONTROLLER___

#include "EoCommon.h"

#include "EOemsControllerCfg.h"

#include "Joint.h"
#include "Motor.h"
#include "AbsEncoder.h"
#include "Pid.h"
#include "JointSet.h"

typedef struct //Controller
{
    uint8_t nJoints;
    uint8_t nSets;
    
    JointSet *jointSet;
    
    uint8_t* set_dim;
    
    uint8_t** jos;
    uint8_t** mos;
    uint8_t** eos;
    
    uint8_t *j2s;
    uint8_t *m2s;
    uint8_t *e2s;
    
    Motor *motor;
    Joint *joint;
    
    float **Jjm;
    float **Jmj;
    float **Jje;
    
    AbsEncoder **absEncoder;
} Controller;

extern Controller* Controller_new(uint8_t nJoints); //

extern void Controller_init(Controller* o); //

extern void Controller_config_joint(Controller* o, int j, eOmc_joint_config_t* config); //
extern void Controller_config_motor(Controller* o, int m, uint8_t hardware_type, uint8_t motor_control_type, eOmc_motor_config_t* config); //
extern void Controller_config_absEncoder(Controller* o, uint8_t j, int32_t resolution, int16_t spike_limit); //
extern void Controller_config_Jjm(Controller* o, float **Jjm); //
extern void Controller_config_Jje(Controller* o, float **Jje); //

extern void Controller_update_joint_torque_fbk(Controller* o, uint8_t j, CTRL_UNITS trq_fbk); //
extern void Controller_update_absEncoder_fbk(Controller* o, uint8_t j, int32_t position, uint8_t error_mask); //

extern int32_t Controller_get_absEncoder(Controller* o, uint8_t j); //

extern void Controller_do(Controller* o); //

extern BOOL Controller_set_control_mode(Controller* o, uint8_t j, uint8_t control_mode);
extern void Controller_set_interaction_mode(Controller* o, uint8_t j, uint8_t interaction_mode);

#endif
