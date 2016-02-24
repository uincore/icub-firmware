#ifndef MC_ABS_ENCODER_H___
#define MC_ABS_ENCODER_H___

#include "EoCommon.h"

#include "EOEmsControllerCfg.h"

/////////////////////////////////////////////////////////
// AbsEncoder

typedef struct //AbsEncoder
{
    int32_t distance;
    int16_t position_last;
    int16_t position_sure;
    
    int32_t velocity;

    int32_t sign;
    int16_t offset;

    int16_t delta;

    uint16_t invalid_fault_cnt;
    uint16_t timeout_fault_cnt;
    
    uint16_t spikes_count;

    int8_t ID;
    int8_t valid_first_data_cnt;
    
    int16_t spike_limit;

    union
    {
        struct
        {
            unsigned not_configured  :1;
            unsigned not_calibrated  :1;
            unsigned not_initialized :1;
            unsigned unused          :5;
        } bits;
        
        uint8_t not_ready;
    } state;
    
    union
    {
        struct
        {
            unsigned timeout_fault      :1;
            unsigned invalid_data_fault :1;
            unsigned invalid_reading    :1;
            unsigned parity_error       :1;
            unsigned spikes             :1;
            unsigned unused             :3;
        } fault_bits;
        
        uint8_t fault_mask;
        
    } faults;
} AbsEncoder;

extern AbsEncoder* AbsEncoder_new(uint8_t n);
extern void AbsEncoder_init(AbsEncoder* o);
extern void AbsEncoder_destroy(AbsEncoder* o);
extern void AbsEncoder_config(AbsEncoder *o, uint8_t ID, int32_t resolution, int16_t spike_limit);
extern void AbsEncoder_calibrate(AbsEncoder* o, int32_t offset);

extern int32_t AbsEncoder_update(AbsEncoder* o, int16_t position);
extern void AbsEncoder_invalid(AbsEncoder* o, uint8_t error_flags);
extern void AbsEncoder_timeout(AbsEncoder* o);

extern int32_t AbsEncoder_position(AbsEncoder* o);
extern int32_t AbsEncoder_velocity(AbsEncoder* o);
extern void AbsEncoder_posvel(AbsEncoder* o, int32_t* position, int32_t* velocity);

extern BOOL AbsEncoder_is_ok(AbsEncoder* o);
extern BOOL AbsEncoder_is_calibrated(AbsEncoder* o);
extern BOOL AbsEncoder_check_faults(AbsEncoder* o);
extern void AbsEncoder_clear_faults(AbsEncoder* o);

#endif

// AbsEncoder
/////////////////////////////////////////////////////////
