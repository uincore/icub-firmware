#ifndef MC_CONTROLLER___
#define MC_CONTROLLER___

#include "EoCommon.h"

#include "EOemsControllerCfg.h"

#include "Joint.h"
#include "Motor.h"
#include "AbsEncoder.h"
#include "Pid.h"
#include "JointSet.h"

typedef struct //MController
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
    
    uint8_t part_type;
    uint8_t actuation_type;
    
    AbsEncoder **absEncoder;
} MController;

extern MController* MController_new(uint8_t nJoints); //

extern void MController_init(void); //

extern void MController_config_board(uint8_t board_type, uint8_t actuation_type);

extern void MController_config_joint(int j, eOmc_joint_config_t* config); //
extern void MController_config_motor(int m, uint8_t hardware_type, uint8_t motor_control_type, eOmc_motor_config_t* config); //
extern void MController_config_absEncoder(uint8_t j, int32_t resolution, int16_t spike_limit); //
extern void MController_config_Jjm(float **Jjm); //
extern void MController_config_Jje(float **Jje); //

extern void MController_update_motor_state_fbk(uint8_t m, void* state);
extern void MController_update_joint_torque_fbk(uint8_t j, CTRL_UNITS trq_fbk); //
extern void MController_update_absEncoder_fbk(uint8_t e, int32_t position); //
extern void MController_invalid_absEncoder_fbk(uint8_t e, uint8_t error_flags);
extern void MController_timeout_absEncoder_fbk(uint8_t e);

extern int32_t MController_get_absEncoder(uint8_t j); //

extern void MController_do(void); //

extern BOOL MController_set_control_mode(uint8_t j, eOmc_controlmode_command_t control_mode);
extern void MController_set_interaction_mode(uint8_t j, eOmc_interactionmode_t interaction_mode);

#if 0
/** @typedef    typedef struct eOmc_calibrator32_t
    @brief      eOmc_calibrator32_t specifies a calibrator with type and parameters for teh new definition of measures
 **/
typedef struct                  // size is 1+3+4*4 = 20
{
    eOenum08_t                  type;                               /**< use eOmc_calibration_type_t */
    uint8_t                     filler03[3];
    union
    {
        uint32_t                                                any[6];
        eOmc_calibrator_params_type0_hard_stops_t               type0;
        eOmc_calibrator_params_type1_abs_sens_analog_t          type1;
        eOmc_calibrator_params_type2_hard_stops_diff_t          type2;
        eOmc_calibrator_params_type3_abs_sens_digital_t         type3;
        eOmc_calibrator_params_type4_abs_and_incremental_t      type4;
        eOmc_calibrator_params_type5_hard_stops_mc4plus_t       type5;
        eOmc_calibrator_params_type6_mais_t                     type6;
        eOmc_calibrator_params_type7_hall_sensor_t              type7;
        eOmc_calibration_type8_adc_and_incr_mc4plus_t           type8;
    } params;                                                       /**< the params of the calibrator */   
} eOmc_calibrator32_t;           EO_VERIFYsizeof(eOmc_calibrator32_t, 28);

typedef eOmc_calibrator32_t eOmc_calibrator_t;
#endif

extern void MController_calibrate_encoder(uint8_t e, eOmc_calibrator_t *calibrator);

extern void MController_go_idle(void);

////////////////////////////////////////////////////////////////////////
//extern void MController_get_motor_control_state(uint8_t m, uint8_t* control_state, uint8_t* control_state_req);
extern uint32_t MController_get_motor_fault_mask(uint8_t m);
extern void MController_get_joint_state(int j, eOmc_joint_status_t* joint_state);
extern void MController_get_pid_state(int j, eOmc_joint_status_ofpid_t* pid_state, BOOL decoupled_pwm);
extern void MController_get_motor_state(int m, eOmc_motor_status_t* motor_status);
extern void MController_update_motor_pos_fbk(int m, int32_t position);
extern void MController_update_motor_current_fbk(int m, int16_t current);
extern void MController_config_motor_friction(int m, eOmc_motor_params_t* friction); //
extern void MController_config_joint_impedance(int j, eOmc_impedance_t* impedance); //
extern void MController_config_pos_pid(int j, eOmc_PID_t *pid_conf); //
extern void MController_config_trq_pid(int m, eOmc_PID_t *pid_conf); //
extern void MController_config_joint_pos_limits(int j, int32_t pos_min, int32_t pos_max);
extern void MController_config_joint_vel_ref_timeout(int j, int32_t timeout_ms);
extern BOOL MController_set_joint_pos_ref(int j, CTRL_UNITS pos_ref, CTRL_UNITS vel_ref);
extern BOOL MController_set_joint_vel_ref(int j, CTRL_UNITS vel_ref, CTRL_UNITS acc_ref);
extern BOOL MController_set_joint_pos_raw(int j, CTRL_UNITS pos_ref);
extern BOOL MController_set_joint_trq_ref(int j, CTRL_UNITS trq_ref);
extern BOOL MController_set_joint_out_ref(int j, CTRL_UNITS out_ref);
extern void MController_stop_joint(int j);
extern void MController_config_motor_gearbox_ratio(int m, int32_t gearbox_ratio);
extern void empty_fake_MController_config_motor_encoder(int m, int32_t resolution);
extern int16_t MController_config_motor_pwm_limit(int m, int16_t pwm_limit);
extern void MController_update_motor_odometry_fbk_can(int m, void* data);
extern void MController_set_motor_overcurrent_fault(int m);
////////////////////////////////////////////////////////////////////////

#endif
