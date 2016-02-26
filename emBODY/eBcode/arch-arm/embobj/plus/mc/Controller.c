#include "EoCommon.h"

#include "EOtheMemoryPool.h"

#include "EOemsControllerCfg.h"

#include "Joint.h"
#include "Motor.h"
#include "AbsEncoder.h"
#include "Pid.h"

#include "Controller.h"

MController* smc = NULL;

static char invert_matrix(float** M, float** I, char n);
static void Controller_config_motor_set(MController* o);
static void Controller_config_encoder_set(MController* o);

MController* Controller_new(uint8_t nJoints) //
{
    if (!smc) smc = NEW(MController, 1);
    
    MController* o = smc;

    if (!o) return NULL;
    
    o->nJoints = nJoints;
    o->nSets = nJoints;
    
    o->jointSet = JointSet_new(nJoints);
    
    o->set_dim = NEW(uint8_t, nJoints);
    
    o->jos = NEW(uint8_t*, nJoints);
    o->mos = NEW(uint8_t*, nJoints);
    o->eos = NEW(uint8_t*, nJoints);
    
    o->j2s = NEW(uint8_t, nJoints);
    o->m2s = NEW(uint8_t, nJoints);
    o->e2s = NEW(uint8_t, nJoints);
    
    o->joint = Joint_new(nJoints);
    o->motor = Motor_new(nJoints); 
    
    o->Jjm = NEW(float*, nJoints);
    o->Jmj = NEW(float*, nJoints);
    o->Jje = NEW(float*, nJoints);
    
    o->absEncoder = NEW(AbsEncoder*, nJoints);
    
    for (int i=0; i<nJoints; ++i)
    {
        o->jos[i] = NEW(uint8_t, nJoints);
        o->mos[i] = NEW(uint8_t, nJoints);
        o->eos[i] = NEW(uint8_t, nJoints);
        
        o->Jjm[i] = NEW(float, nJoints);
        o->Jmj[i] = NEW(float, nJoints);
        o->Jje[i] = NEW(float, nJoints);
        
        o->absEncoder[i] = NULL;
        
        ////////////////////////
        
        JointSet_config
        (
            o->jointSet+i,
            o->set_dim+i,
            o->jos[i],
            o->mos[i],
            o->eos[i],
            o->joint, 
            o->motor, 
            o->Jjm,
            o->Jmj,
            o->Jje,
            o->absEncoder
        );
    }
    
    Controller_init();
        
    return o;
}

void Controller_init() //
{
    MController *o = smc;
    
    if (!o) return;
    
    o->nSets = o->nJoints;
    
    for (int i=0; i<o->nJoints; ++i)
    {
        o->absEncoder[i] = NULL;
        
        o->j2s[i] = i;
        o->m2s[i] = i;
        o->e2s[i] = i;
        
        o->set_dim[i] = 1;
        
        o->jos[i][0] = i;
        o->mos[i][0] = i;
        o->eos[i][0] = i;
        
        for (int k=0; k<o->nJoints; ++k)
        {
            o->Jjm[i][k] = o->Jmj[i][k] = o->Jje[i][k] = (i==k?1.0f:0.0f);
        }
    }
}

void Controller_config_board(uint8_t part_type, uint8_t actuation_type)
{
    MController *o = smc;
    
    o->part_type      = part_type;
    o->actuation_type = actuation_type;
}

void Controller_config_joint(int j, eOmc_joint_config_t* config) //
{
    MController *o = smc;
    
    Joint_config(o->joint+j, config);
    
    Motor_config_trqPID(o->motor+j, &(config->pidtorque));
    Motor_config_filter(o->motor+j, config->tcfiltertype);
    Motor_config_friction(o->motor+j, config->motor_params.bemf_value, config->motor_params.ktau_value);
    
    switch(config->jntEncoderType)
    {
        case eomc_encoder_AEA:
            o->absEncoder[j] = AbsEncoder_new(1);
            AbsEncoder_config(o->absEncoder[j], j, config->jntEncoderResolution, AEA_DEFAULT_SPIKE_LIMIT);
            break;
        
        default:
            AbsEncoder_destroy(o->absEncoder[j]);
            o->absEncoder[j] = NULL;
            break;
    }
}

void Controller_config_motor(int m, uint8_t hardware_type, uint8_t motor_control_type, eOmc_motor_config_t* config) //
{
    MController *o = smc;
    
    Motor_config(o->motor+m, m, hardware_type, motor_control_type, config);
}

void Controller_config_absEncoder(uint8_t j, int32_t resolution, int16_t spike_limit) //
{
    MController *o = smc;
    
    if (!o->absEncoder[j]) o->absEncoder[j] = AbsEncoder_new(1);
    
    AbsEncoder_config(o->absEncoder[j], j, resolution, spike_limit);
}

void Controller_config_Jjm(float **Jjm) //
{
    MController *o = smc;
    
    int N = o->nJoints;
    
    for (int j=0; j<N; ++j)
    {
        for (int m=0; m<N; ++m)
        {
            o->Jjm[j][m] = Jjm[j][m];
        }
    }
    
    invert_matrix(Jjm, o->Jmj, N);

    for (int m=0; m<N; ++m)
    {
        for (int j1=0; j1<N; ++j1)
        {
            for (int j2=0; j2<N; ++j2) if (j1!=j2)
            {
                if (Jjm[j1][m] != 0.0f && Jjm[j2][m] != 0.0f)
                {
                    if (o->j2s[j2] > o->j2s[j1])
                    {
                        int s = o->j2s[j2];

                        for (int j=0; j<N; ++j)
                        {
                            if (o->j2s[j] == s) o->j2s[j] = o->j2s[j1]; 
                        }
                    }
                }
            }
        }
    }
    
    for (int s=0; s<N; ++s)
    {   
        o->set_dim[s] = 0;
    }
    
    for (int j=0; j<N; ++j)
    {
        int s = o->j2s[j];

        o->jos[s][o->set_dim[s]++] = j;
    }

    o->nSets = 0;
    
    for (int s=0; s<N; ++s)
    {
        if (o->set_dim[s])
        {
            o->set_dim[o->nSets] = o->set_dim[s];
            
            for (int j=0; j<o->set_dim[s]; ++j)
            {
                o->jos[o->nSets][j] = o->jos[s][j];
            }

            ++o->nSets;
        }
    }
    
    Controller_config_motor_set(o);
    Controller_config_encoder_set(o);
}

void Controller_config_Jje(float **Jje) //
{
    MController *o = smc;
    
    int N = o->nJoints;

    for (int j=0; j<N; ++j)
    { 
        for (int e=0; e<N; ++e)
        {
            o->Jje[j][e] = Jje[j][e];
        }
    }
        
    for (int e=0; e<N; ++e)
    {
        for (int j1=0; j1<N; ++j1)
        {
            for (int j2=0; j2<N; ++j2) if (j1!=j2)
            {
                if (Jje[j1][e] != 0.0f && Jje[j2][e] != 0.0f)
                {
                    if (o->j2s[j2] > o->j2s[j1])
                    {
                        int s = o->j2s[j2];

                        for (int j=0; j<N; ++j)
                        {
                            if (o->j2s[j] == s) o->j2s[j] = o->j2s[j1];
                        }
                    }
                }
            }
        }
    }
    
    for (int s=0; s<N; ++s)
    {   
        o->set_dim[s] = 0;
    }
    
    for (int j=0; j<N; ++j)
    {
        int s = o->j2s[j];

        o->jos[s][o->set_dim[s]++] = j;
    }
    
    for (int s=0; s<N; ++s)
    {
        if (o->set_dim[s])
        {
            o->set_dim[o->nSets] = o->set_dim[s];
            
            for (int j=0; j<o->set_dim[s]; ++j)
            {
                o->jos[o->nSets][j] = o->jos[s][j];
            }

            ++o->nSets;
        }
    }

    Controller_config_motor_set(o);
    Controller_config_encoder_set(o);
}

void Controller_update_joint_torque_fbk(uint8_t j, CTRL_UNITS trq_fbk) //
{
    Joint_update_torque_fbk(smc->joint+j, trq_fbk);
}

void Controller_update_absEncoder_fbk(uint8_t e, int32_t position) //
{
    AbsEncoder_update(smc->absEncoder[e], position);
}

void Controller_update_motor_state_fbk(uint8_t m, void* state)
{
    Motor_update_state_fbk(smc->motor+m, state);
}

void Controller_invalid_absEncoder_fbk(uint8_t e, uint8_t error_flags) //
{
    AbsEncoder_invalid(smc->absEncoder[e], error_flags);
}

void Controller_timeout_absEncoder_fbk(uint8_t e) //
{
    AbsEncoder_timeout(smc->absEncoder[e]);
}

int32_t Controller_get_absEncoder(uint8_t j)
{
    return smc->absEncoder[j]->distance;
}

void Controller_do()
{
    for (int s=0; s<smc->nSets; ++s)
    {
        JointSet_do_odometry(smc->jointSet+s);
    }
}

BOOL Controller_set_control_mode(uint8_t j, eOmc_controlmode_command_t control_mode) //
{
    return JointSet_set_control_mode(smc->jointSet+smc->j2s[j], control_mode);
}    

void Controller_set_interaction_mode(uint8_t j, eOmc_interactionmode_t interaction_mode) //
{
    JointSet_set_interaction_mode(smc->jointSet+smc->j2s[j], interaction_mode);
} 

// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static void Controller_config_motor_set(MController *o)
{
    int N = o->nJoints;
    
    uint8_t set_dim[MAX_PER_BOARD];
    
    for (int s=0; s<o->nSets; ++s)
    {
        for (int i=0; i<o->set_dim[s]; ++i)
        {
            int j = o->jos[s][i];

            //o->j2s[j] = s;

            for (int m=0; m<N; ++m)
            {
                if (o->Jjm[j][m] != 0.0f)
                {
                    o->m2s[m] = s;
                }
            }
        }
        
        set_dim[s] = 0;
    }

    for (int m=0; m<N; ++m)
    {
        int s = o->m2s[m];

        o->mos[s][set_dim[s]++] = m;
    }
}

static void Controller_config_encoder_set(MController *o)
{
    int N = o->nJoints;
    
    uint8_t set_dim[MAX_PER_BOARD];
    
    for (int s=0; s<o->nSets; ++s)
    {
        for (int i=0; i<o->set_dim[s]; ++i)
        {
            int j = o->jos[s][i];

            //o->j2s[j] = s;

            for (int e=0; e<N; ++e)
            {
                if (o->Jje[j][e] != 0.0f)
                {
                    o->e2s[e] = s;
                }
            }
        }
        
        set_dim[s] = 0;
    }

    for (int e=0; e<N; ++e)
    {
        int s = o->e2s[e];

        o->eos[s][set_dim[s]++] = e;
    }
}

#define FOR(i) for (int i=0; i<n; ++i)
#define SCAN(r,c) FOR(r) FOR(c)

static char invert_matrix(float** M, float** I, char n)
{
    float B[MAX_PER_BOARD][MAX_PER_BOARD];
    SCAN(r,c) { B[r][c]=M[r][c]; I[r][c]=0.0f; }
    FOR(i) I[i][i]=1.0f;
    
    for (int r=0; r<n-1; ++r)
    {
        int pivot=-1;
        float max2=0.0f;
        
        for (int d=r; d<n; ++d)
        {
            float m2=B[d][r]*B[d][r];

            if (m2>max2)
            {
                max2=m2;
                pivot=d;
            }
        }

        if (pivot==-1)
        {
            return 0;
        }

        if (pivot!=r)
        {
            FOR(c)
            {
                float tb=B[r][c]; B[r][c]=B[pivot][c]; B[pivot][c]=tb;
                float ti=I[r][c]; I[r][c]=I[pivot][c]; I[pivot][c]=ti;
            }
        }

        float P=-1.0f/B[r][r];

        for (int rr=r+1; rr<n; ++rr)
        {
            float D=P*B[rr][r]; 

            FOR(c)
            {
                B[rr][c]+=D*B[r][c];
                I[rr][c]+=D*I[r][c];
            }
        }
    }

    for (int r=n-1; r>0; --r)
    {
        float P=-1.0f/B[r][r];

        for (int rr=r-1; rr>=0; --rr)
        {
            float D=P*B[rr][r]; 

            FOR(c)
            {
                B[rr][c]+=D*B[r][c];
                I[rr][c]+=D*I[r][c];
            }
        }
    }

    FOR(r)
    {
        float D=1.0f/B[r][r];

        FOR(c) I[r][c]*=D;
    }
    
    return 1;
}

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

void Controller_calibrate(uint8_t e, eOmc_calibrator_t *calibrator)
{
    JointSet_calibrate(smc->jointSet+smc->e2s[e], e, calibrator);
}

void Controller_go_idle(void)
{
    for (uint8_t s=0; s<smc->nSets; ++s)
    {
        JointSet_set_control_mode(smc->jointSet+s, eomc_controlmode_cmd_force_idle);
    }
}

////////////////////////////////////////////////////////////////////////////////

/*
void Controller_get_motor_control_state(uint8_t m, uint8_t* control_state, uint8_t* control_state_req)
{
    Motor_get_control_state(smc->motor+m, control_state, control_state_req);
}
*/

uint32_t Controller_get_motor_fault_mask(uint8_t m)
{
    return Motor_get_fault_mask(smc->motor+m);
}

void Controller_get_joint_state(uint8_t j, eOmc_joint_status_t* joint_state)
{
    Joint_get_state(smc->joint+j, joint_state);
}


void Controller_get_pid_state(uint8_t j, eOmc_joint_status_ofpid_t* pid_state, BOOL decoupled_pwm)
{    
    if (Joint_get_pid_state(smc->joint+j, pid_state))
    {
        Motor_get_pid_state(smc->motor+j, pid_state);
    }
    
    if (decoupled_pwm)
    {
        pid_state->complpos.output = smc->motor[j].output;
    }
}

void Controller_get_motor_state(uint8_t m, eOmc_motor_status_t* motor_status)
{
    Motor_get_state(smc->motor+m, motor_status);
}

void Controller_update_motor_pos_fbk(uint8_t m, int32_t position)
{
    Motor_update_pos_fbk(smc->motor+m, position);
}

void Controller_update_motor_current_fbk(uint8_t m, int16_t current)
{
    Motor_update_current_fbk(smc->motor+m, current);
}

////////////////////////////////////////////////////////////////////////////////
