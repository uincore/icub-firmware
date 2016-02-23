#ifndef MC_JOINT_SET___
#define MC_JOINT_SET___

#include "EoCommon.h"

#include "EOemsControllerCfg.h"

#include "Joint.h"
#include "Motor.h"
#include "AbsEncoder.h"
#include "Pid.h"

typedef struct // JointSet
{
    uint8_t* pN;
    
    uint8_t* joints_of_set;
    uint8_t* motors_of_set;
    uint8_t* encoders_of_set;
    
    Joint* joint;
    Motor* motor;

    float** Jjm;
    float** Jmj;
    float** Jje;
    
    AbsEncoder** absEncoder;
 
    uint8_t control_mode;
    uint8_t interaction_mode;
    
    BOOL pos_control_active;
    BOOL trq_control_active;
    BOOL can_do_trq_ctrl;
    
    uint8_t motor_control_type;
    
    BOOL is_calibrated;
} JointSet;

extern JointSet* JointSet_new(uint8_t n); //

extern void JointSet_init(JointSet* o); //

extern void JointSet_config //
(
    JointSet* o,
    uint8_t* pN,
    uint8_t* joints_of_set,
    uint8_t* motors_of_set,
    uint8_t* encoders_of_set,
    Joint* joint,
    Motor* motor,
    float** Jjm,
    float** Jmj,
    float** Jje,
    AbsEncoder **absEncoder
);

extern void JointSet_do_odometry(JointSet* o); //
extern BOOL JointSet_do_check_faults(JointSet* o); //
extern void JointSet_do_control(JointSet* o); //

extern void JointSet_do(JointSet* o);
    
extern BOOL JointSet_set_control_mode(JointSet* o, uint8_t control_mode);
extern void JointSet_set_interaction_mode(JointSet* o, uint8_t interaction_mode);

#endif
