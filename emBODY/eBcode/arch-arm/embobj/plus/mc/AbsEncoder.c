#include "EoCommon.h"

#include "EoError.h"
#include "EOtheErrorManager.h"

#include "EOMtheEMSrunner.h"

#include "EOemsControllerCfg.h"

#include "AbsEncoder.h"

/////////////////////////////////////////////////////////
// AbsEncoder

AbsEncoder* AbsEncoder_new(uint8_t n)
{
    AbsEncoder *o = NEW(AbsEncoder, n);
    
    for (uint8_t i=0; i<n; ++i)
    {
        AbsEncoder_init(&(o[i]));
    }
    
    return o;
}

void AbsEncoder_init(AbsEncoder* o)
{
    ////////////////////// set by config
    
    o->ID = 0;
    
    o->spike_limit = 0;
    
    o->sign = 0;
    
    ////////////////////// set by calibrate
    
    o->offset = 0;
    
    //////////////////////
    
    o->distance = 0;
    o->position_last = 0;
    o->position_sure = 0;
    
    o->velocity = 0;

    o->delta = 0;

    o->invalid_fault_cnt = 0;
    o->timeout_fault_cnt = 0;
    
    o->spikes_count = 0;
    
    o->valid_first_data_cnt = 0;
    
    o->faults.fault_mask = FALSE;
    
    o->state.bits.not_configured  = TRUE;
    o->state.bits.not_calibrated  = TRUE;
    o->state.bits.not_initialized = TRUE;
}

void AbsEncoder_destroy(AbsEncoder* o)
{
    DELETE(o);
}

void AbsEncoder_config(AbsEncoder* o, uint8_t ID, int32_t resolution, int16_t spike_limit)
{
    o->ID = ID;
    
    o->spike_limit = spike_limit;//7*(65536L/resolution);
    
    o->sign = resolution > 0 ? 1 : -1;
    
    o->state.bits.not_configured = FALSE;    
}

void AbsEncoder_calibrate(AbsEncoder* o, int32_t offset)
{
    o->offset = offset;
    
    o->state.bits.not_calibrated = FALSE; 
}

int32_t AbsEncoder_position(AbsEncoder* o)
{
    return o->sign*o->distance;
}

int32_t AbsEncoder_velocity(AbsEncoder* o)
{
    return o->sign*o->velocity;
}

void AbsEncoder_posvel(AbsEncoder* o, int32_t* position, int32_t* velocity)
{
    *position = o->sign*o->distance;
    *velocity = o->sign*o->velocity;
}



static void AbsEncoder_position_init(AbsEncoder* o, int16_t position)
{
    if (!o) return;
    
    if (!o->valid_first_data_cnt)
    {
        o->position_last = position;
    }
    
    if (o->position_last != position)
    {
        o->valid_first_data_cnt = 0;
        
        return;
    }
    
    if (++o->valid_first_data_cnt >= 3)
    {
        o->position_last = position;
        o->position_sure = position;

        o->distance = position;
        
        o->velocity = 0;
        
        o->delta = 0;

        o->valid_first_data_cnt = 0;
        
        o->state.bits.not_initialized = FALSE;
    }
}

void AbsEncoder_timeout(AbsEncoder* o)
{
    if (!o) return;
    
    if (o->state.bits.not_configured) return;
    
    if (o->state.bits.not_calibrated) return;
    
    if (o->timeout_fault_cnt > ENCODER_TIMEOUT_COUNTER)
    {
        o->faults.fault_bits.timeout_fault = TRUE;
    }
    else
    {
        ++o->timeout_fault_cnt;
    }
    
    o->valid_first_data_cnt = 0;
}

void AbsEncoder_invalid(AbsEncoder* o, uint8_t error_flags)
{
    if (!o) return;
    
    if (o->state.bits.not_configured) return;
    
    if (o->state.bits.not_calibrated) return;

    if (o->invalid_fault_cnt > ENCODER_INVALID_COUNTER)
    {
        o->faults.fault_bits.invalid_data_fault = TRUE;
    }
    else
    {
        ++o->invalid_fault_cnt;
    }
    
    o->valid_first_data_cnt = 0;
}

int32_t AbsEncoder_update(AbsEncoder* o, int16_t position)
{
    if (!o) return 0;
        
    if (o->state.bits.not_configured) return 0;
    
    if (o->state.bits.not_calibrated) return 0;
    
    position -= o->offset;
        
    o->invalid_fault_cnt = 0;
    o->timeout_fault_cnt = 0;
    
    if (o->state.bits.not_initialized)
    {
        AbsEncoder_position_init(o, position);
        
        o->velocity = 0;
        
        return o->sign*o->distance;
    }
    
    int16_t check = position - o->position_last;
        
    o->position_last = position;

    if (-o->spike_limit <= check && check <= o->spike_limit)
    {
        int16_t delta = position - o->position_sure;
            
        if (delta)
        {
            o->position_sure = position;
                
            o->delta = delta;
                
            o->distance += delta;
                
            o->velocity = (7*o->velocity + ((int32_t)CTRL_LOOP_FREQUENCY)*delta) >> 3;
        }
        else
        {
            o->velocity = (7*o->velocity) >> 3;
        }
    }
    else
    {
        o->spikes_count++;
       
        o->velocity = (7*o->velocity) >> 3;
    }
        
    //every second
    
    eOemsrunner_diagnosticsinfo_t* runner_info = eom_emsrunner_GetDiagnosticsInfoHandle(eom_emsrunner_GetHandle());
    
    if ((runner_info->numberofperiods % 1000) == 0)
    {
        if (o->spikes_count > 0)
        {                
            //message "spike encoder error"
            eOerrmanDescriptor_t descriptor = {0};
            descriptor.par16 = o->ID;           
            descriptor.par64 = o->spikes_count;
            descriptor.sourcedevice = eo_errman_sourcedevice_localboard;
            descriptor.sourceaddress = 0;
            descriptor.code = eoerror_code_get(eoerror_category_MotionControl, eoerror_value_MC_aea_abs_enc_spikes);
            eo_errman_Error(eo_errman_GetHandle(), eo_errortype_warning, NULL, NULL, &descriptor);
                
            o->spikes_count = 0;
        }
    }
    
    return o->sign*o->distance;
}

BOOL AbsEncoder_is_ok(AbsEncoder* o)
{
    return (!o->state.not_ready) && (!o->faults.fault_mask);
}


BOOL AbsEncoder_is_calibrated(AbsEncoder* o)
{    
    return !o->state.bits.not_calibrated;
}

BOOL AbsEncoder_check_faults(AbsEncoder* o)
{
    if (!o) return FALSE;
        
    return o->faults.fault_mask != 0;
}

void AbsEncoder_clear_faults(AbsEncoder* o)
{
    o->faults.fault_mask = 0;
}


// AbsEncoder
/////////////////////////////////////////////////////////