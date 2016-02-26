#include "Joint.h"

#include "EOtheCANprotocol.h"

Joint* Joint_new(uint8_t n)
{
    Joint* o = NEW(Joint, n);
    
    for (int i=0; i<n; ++i)
    {
        Joint_init(o+i);
    }
    
    return o;
}

void Joint_init(Joint* o)
{
    o->pos_min = ZERO;
    o->pos_max = ZERO;
    o->vel_max = ZERO;
    o->acc_max = ZERO;
    
    o->output_lim = ZERO;
    
    //o->trq_max = ZERO;
    //o->out_max = ZERO;
    
    o->pos_fbk = ZERO;
    o->vel_fbk = ZERO;
    o->trq_fbk = ZERO;
    
    o->motor_pos_fbk = ZERO;
    o->motor_vel_fbk = ZERO;
    
    o->out_ref = ZERO;
    o->pos_ref = ZERO;
    o->vel_ref = ZERO;
    o->acc_ref = ZERO;
    o->trq_ref = ZERO;
    
    o->vel_dir_ref = ZERO;
    
    o->pos_err = ZERO;
    o->vel_err = ZERO;
    o->trq_err = ZERO;
    
    o->output = ZERO;
    
    o->tcKoffset = ZERO;
    o->tcKstiff  = ZERO;
    o->tcKdamp   = ZERO;
    
    o->scKpos   = ZERO;
    o->scKvel   = ZERO;
    o->scKstill = ZERO;
    
    WatchDog_init(&o->trq_fbk_wdog);
    WatchDog_init(&o->vel_ref_wdog);
    
    WatchDog_set_base_time_msec(&o->trq_fbk_wdog, DEFAULT_WATCHDOG_TIME_MSEC);
    WatchDog_set_base_time_msec(&o->vel_ref_wdog, DEFAULT_WATCHDOG_TIME_MSEC);
    
    WatchDog_rearm(&o->trq_fbk_wdog);
    WatchDog_rearm(&o->vel_ref_wdog);
    
    Trajectory_init(&o->trajectory, 0, 0, 0);
    
    PID_init(&o->posPID);
    
    o->control_mode = eomc_controlmode_notConfigured;
    o->interaction_mode = eOmc_interactionmode_stiff;
    
    o->pos_control_active = FALSE;
    o->trq_control_active = FALSE;
    
    o->motor_control_type = PWM_CONTROLLED_MOTOR;
    
    o->can_do_trq_ctrl = TRUE;
    
    o->pushing_limit = FALSE;
    
    //SpeedController_init(o->speedController);
}

void Joint_config(Joint* o, eOmc_joint_config_t* config)
{
    PID_config(&o->posPID, &config->pidposition);
    
    o->scKpos   = config->pidposition.kp;
    o->scKvel   = config->pidposition.kd;
    o->scKstill = config->pidposition.ki;
    
    o->pos_min = config->limitsofjoint.min;
    o->pos_max = config->limitsofjoint.max;
    o->vel_max = config->maxvelocityofjoint;
    o->acc_max = 65535;
    
    o->tcKstiff  = 0.001f*(CTRL_UNITS)(config->impedance.stiffness);
    o->tcKdamp   = 0.001f*CTRL_LOOP_FREQUENCY*(CTRL_UNITS)(config->impedance.damping);
    o->tcKoffset = config->impedance.offset;
    
    o->Kadmitt = (o->tcKstiff == ZERO) ? ZERO: (1.0f/o->tcKstiff); 
    
    Trajectory_config_limits(&o->trajectory, o->pos_min, o->pos_max, o->vel_max, o->acc_max);
    
    WatchDog_set_base_time_msec(&o->vel_ref_wdog, config->velocitysetpointtimeout);
    WatchDog_rearm(&o->vel_ref_wdog);
    
    /////#warning *** MCR joint output max (openloop) missing ***
    //o->out_max = config->
    /////#warning *** MCR joint torque max (openloop) missing ***
    //o->trq_max = config->
    /////#warning *** MCR joint admittance missing ***
    o->Kadmitt = ZERO;
}

void Joint_destroy(Joint* o)
{
    DELETE(o);
}

void Joint_motion_reset(Joint *o)
{
    PID_reset(&o->posPID);
    
    Trajectory_stop(&o->trajectory, o->pos_fbk);
        
    //Watchdog_disarm(&o->vel_ref_wdog);
    
    o->pos_ref = o->pos_fbk;
    o->vel_ref = ZERO;
    o->acc_ref = ZERO;
    o->out_ref = ZERO;
    o->trq_ref = o->trq_fbk;
       
    o->vel_dir_ref = ZERO;
    
    o->pos_err = ZERO;
    o->trq_err = ZERO;
    o->vel_err = ZERO;
    
    o->output = ZERO;
}

BOOL Joint_set_control_mode(Joint* o, eOmc_controlmode_command_t control_mode)
{
    if (o->control_mode == control_mode) return TRUE;
    
    if (o->control_mode == eomc_controlmode_notConfigured) return FALSE;
        
    if (o->control_mode == eomc_controlmode_calib) return FALSE;    
    
    if (o->control_mode == eomc_controlmode_hwFault)
    {
        if (control_mode == eomc_controlmode_cmd_force_idle)
        {
            Joint_clear_faults(o);
        }
        else
        {
            return FALSE;
        }
    }
    
    switch (control_mode)
    {
        case eomc_controlmode_cmd_force_idle:
            control_mode = eomc_controlmode_cmd_idle;
        case eomc_controlmode_cmd_idle:
        case eomc_controlmode_cmd_mixed:
        case eomc_controlmode_cmd_velocity:
        case eomc_controlmode_cmd_position:
        case eomc_controlmode_cmd_direct:
            break;
                
        case eomc_controlmode_cmd_openloop:
            if (o->motor_control_type != PWM_CONTROLLED_MOTOR) return FALSE;
            break;

        case eomc_controlmode_cmd_torque:
            if (!o->can_do_trq_ctrl) return FALSE;
            break;
            
        default:
            return FALSE;
    }
    
    o->control_mode = (eOmc_controlmode_t)control_mode;
    
    Joint_motion_reset(o);
    
    return TRUE;
}

#if 0
typedef struct // Joint
{    
    WatchDog trq_fbk_wdog;
    WatchDog vel_ref_wdog;
} Joint;
#endif

BOOL Joint_set_interaction_mode(Joint* o, eOmc_interactionmode_t interaction_mode)
{
    if (o->interaction_mode == interaction_mode) return TRUE;
    
    if (!o->can_do_trq_ctrl)
    {
        if (interaction_mode == eOmc_interactionmode_compliant)
        {
            return FALSE;
        }
    }
    
    o->interaction_mode = interaction_mode;
    
    if (interaction_mode == eOmc_interactionmode_stiff)
    {
        Joint_motion_reset(o);
    }
    
    return TRUE;
}

void Joint_update_odometry_fbk(Joint* o, CTRL_UNITS pos_fbk, CTRL_UNITS vel_fbk)
{
    o->pos_fbk = pos_fbk;
    o->vel_fbk = vel_fbk;
}

void Joint_update_torque_fbk(Joint* o, CTRL_UNITS trq_fbk)
{
    if (trq_fbk == o->trq_fbk) return;
    
    o->trq_fbk = trq_fbk;
    
    WatchDog_rearm(&o->trq_fbk_wdog);
}

BOOL Joint_check_faults(Joint* o)
{
    BOOL fault = FALSE; 
    
    if (WatchDog_check_expired(&o->trq_fbk_wdog))
    {
        o->trq_fbk = ZERO;
        o->trq_ref = ZERO;
        o->trq_err = ZERO;
        
        if (o->trq_control_active)
        {
            fault = TRUE;
        }
    }
    
    return fault;
}
 
extern void Joint_clear_faults(Joint* o)
{
    o->fault_state_mask = 0;
}

int8_t Joint_check_limits(Joint* o)
{
    if (o->pos_fbk <= o->pos_min) return -1;
    if (o->pos_fbk >= o->pos_max) return  1;
    
    return 0;
}

int8_t Joint_pushing_limit(Joint* o)
{
    return o->pushing_limit;
}

CTRL_UNITS Joint_do_pwm_control(Joint* o)
{
    o->pushing_limit = FALSE;
    
    switch (o->control_mode)
    {
        case eomc_controlmode_openloop:
        {
            o->pos_err = o->pos_ref = ZERO;
            o->vel_err = o->vel_ref = ZERO;
            o->trq_err = o->trq_ref = ZERO;
        
            o->output = o->out_ref;
            break;
        }
        case eomc_controlmode_torque:
        {
            o->pos_err = o->pos_ref = ZERO;
            o->vel_err = o->vel_ref = ZERO;
            o->trq_err = o->trq_ref - o->trq_fbk;
        
            o->output = o->trq_ref;
        
            if (o->pos_fbk <= o->pos_min) 
            {
                o->output_lim = PID_do_out(&o->posPID, o->pos_min-o->pos_fbk);
                o->pushing_limit = -1;
            }
            else if (o->pos_fbk >= o->pos_max)
            {
                o->output_lim = PID_do_out(&o->posPID, o->pos_max-o->pos_fbk);
                o->pushing_limit =  1;
            }
            else
            {
                o->output_lim = ZERO;
            }
        
            break;
        }
        case eomc_controlmode_velocity:
        case eomc_controlmode_mixed:
        {
            if (WatchDog_check_expired(&o->vel_ref_wdog))
            {
                Trajectory_velocity_stop(&o->trajectory);
            }
        }
        case eomc_controlmode_position:
        case eomc_controlmode_direct:
        {    
            Trajectory_do_step(&o->trajectory, &o->pos_ref, &o->vel_ref, &o->acc_ref);
        
            CTRL_UNITS pos_err_old = o->pos_err;
        
            o->pos_err = o->pos_ref - o->pos_fbk;
            o->vel_err = o->vel_ref - o->vel_fbk;
        
            if (o->interaction_mode == eOmc_interactionmode_stiff)
            {
                o->trq_err = o->trq_ref = ZERO;
                
                o->output = PID_do_out(&o->posPID, o->pos_err);
            }
            else
            {
                o->trq_ref = o->tcKoffset + o->tcKstiff*o->pos_err + o->tcKdamp*(o->pos_err - pos_err_old);
                o->trq_err = o->trq_ref - o->trq_fbk;
                
                o->output = o->trq_ref;
                
                if (o->pos_fbk <= o->pos_min) 
                {
                    o->output_lim = PID_do_out(&o->posPID, o->pos_min-o->pos_fbk);
                    o->pushing_limit = -1;
                }
                else if (o->pos_fbk >= o->pos_max)
                {
                    o->output_lim = PID_do_out(&o->posPID, o->pos_max-o->pos_fbk);
                    o->pushing_limit =  1;
                }
                else
                {
                    o->output_lim = ZERO;
                }
            }
            
            break;
        }    
        default:
        {
            o->pos_err = o->pos_ref = ZERO;
            o->vel_err = o->vel_ref = ZERO;
            o->trq_err = o->trq_ref = ZERO;
            
            o->output = ZERO;
            break;
        }
    }
    
    return o->output;
}

CTRL_UNITS Joint_do_vel_control(Joint* o)
{
    o->pushing_limit = FALSE;
    
    switch (o->control_mode)
    { 
        case eomc_controlmode_torque:
        {
            o->pos_err = o->pos_ref = ZERO;
            o->vel_err = o->vel_ref = ZERO;
            o->trq_err = o->trq_ref - o->trq_fbk;
        
            if (o->pos_fbk <= o->pos_min) 
            {
                o->pushing_limit = -1;
                o->vel_ref = o->scKstill * (o->pos_min - o->pos_fbk);        
            }
            else if (o->pos_fbk >= o->pos_max)
            {
                o->pushing_limit =  1;
                o->vel_ref = o->scKstill * (o->pos_max - o->pos_fbk);
            }
            else
            {
                o->vel_ref = -o->Kadmitt * o->trq_err;
            }
                
            o->output = o->vel_ref;

            break;
        }
        case eomc_controlmode_velocity:
        case eomc_controlmode_mixed:
        {            
            if (WatchDog_check_expired(&o->vel_ref_wdog))
            {
                Trajectory_velocity_stop(&o->trajectory);
            }
        }
        case eomc_controlmode_position:
        case eomc_controlmode_direct:
        {    
            Trajectory_do_step(&o->trajectory, &o->pos_ref, &o->vel_ref, &o->acc_ref);
        
            //CTRL_UNITS pos_err_old = o->pos_err;
        
            o->pos_err = o->pos_ref - o->pos_fbk;
            o->vel_err = o->vel_ref - o->vel_fbk;
        
            if (o->interaction_mode == eOmc_interactionmode_stiff)
            {
                o->trq_err = o->trq_ref = ZERO;
                
                if (o->control_mode == eomc_controlmode_direct)
                {
                    o->vel_ref = o->scKpos*o->pos_err + o->vel_dir_ref;
                }
                else
                {
                    if (Trajectory_is_done(&o->trajectory))
                    {
                        o->vel_ref = o->scKstill*o->pos_err;
                    }
                    else
                    {
                        o->vel_ref += o->scKpos*o->pos_err;
                    }
                }
                
                o->output = o->vel_ref;
            }
            else
            {
                if (o->pos_fbk <= o->pos_min) 
                {
                    o->pushing_limit = -1;
                    o->vel_ref = o->scKstill * (o->pos_min - o->pos_fbk);
                    
                }
                else if (o->pos_fbk >= o->pos_max)
                {
                    o->pushing_limit =  1;
                    o->vel_ref = o->scKstill * (o->pos_max - o->pos_fbk);
                }
                else
                {
                    o->vel_ref += o->scKpos * (o->pos_err + o->Kadmitt * o->trq_fbk);
                }
                
                o->output = o->vel_ref;
            }
            
            break;
        }    
        default:
        {
            o->pos_err = o->pos_ref = ZERO;
            o->vel_err = o->vel_ref = ZERO;
            o->trq_err = o->trq_ref = ZERO;
            
            o->output = ZERO;
            break;
        }
    }
    
    return o->output;
}

void Joint_set_impedance(Joint* o, eOmc_impedance_t* impedance)
{
    o->tcKstiff  = 0.001f*(CTRL_UNITS)(impedance->stiffness);
    o->tcKdamp   = 0.001f*CTRL_LOOP_FREQUENCY*(CTRL_UNITS)(impedance->damping);
    o->tcKoffset = impedance->offset;
    
    if (o->tcKstiff != 0.0f)
    {
        o->Kadmitt = 1.0f/o->tcKstiff; 
    }
    else
    {
        o->Kadmitt = 0.0f;
    }
}

void Joint_get_impedance(Joint* o, eOmc_impedance_t* impedance)
{
    impedance->stiffness = (eOmeas_stiffness_t)(1000.0f*o->tcKstiff);
    impedance->damping   = (eOmeas_damping_t)(1000.0f*CTRL_LOOP_PERIOD*o->tcKdamp);
    impedance->offset    = (eOmeas_torque_t)(o->tcKoffset);
}

void Joint_get_state(Joint* o, eOmc_joint_status_t* joint_state)
{
    joint_state->core.modes.interactionmodestatus    = o->interaction_mode;
    joint_state->core.modes.controlmodestatus        = o->control_mode;
    joint_state->core.modes.ismotiondone             = Trajectory_is_done(&o->trajectory);
    joint_state->core.measures.meas_position         = o->pos_fbk;           
    joint_state->core.measures.meas_velocity         = o->vel_fbk;        
    joint_state->core.measures.meas_acceleration     = 0; // not implemented      
    joint_state->core.measures.meas_torque           = o->trq_fbk;
}

BOOL Joint_get_pid_state(Joint* o, eOmc_joint_status_ofpid_t* pid_state)
{
    BOOL trq_active = FALSE;
    
    if (o->pos_control_active)
    {
        pid_state->complpos.refpos = o->pos_ref;
        pid_state->complpos.errpos = o->pos_err;
    }
    else
    {
        pid_state->complpos.refpos = 0;
        pid_state->complpos.errpos = 0;
    }
    
    if (o->trq_control_active)
    {
        trq_active = TRUE;
        
        pid_state->complpos.reftrq = o->trq_ref;
        pid_state->complpos.errtrq = o->trq_err;
    }
    else
    {
        pid_state->complpos.reftrq = 0;
        pid_state->complpos.errtrq = 0;
        
        pid_state->complpos.output = o->output;
    }
    
    return trq_active;
}
