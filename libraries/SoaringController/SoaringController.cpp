#include <AP_HAL.h>
#include "SoaringController.h"

extern const AP_HAL::HAL& hal;


// ArduSoar parameters
const AP_Param::GroupInfo SoaringController::var_info[] PROGMEM = {
    // @Param: SOAR_ACTIVE
    // @DisplayName: Is the soaring mode active or not
    // @Description: Toggles the soaring mode on and off
    // @Units: boolean
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("ACTIVE", 0, SoaringController, soar_active, 0.0f),
     
    // @Param: THERMAL_VSPEED
    // @DisplayName: Vertical v-speed
    // @Description: Rate of climb to trigger themalling speed
    // @Units: m/s
    // @Range: 0 10
    // @User: Advanced 
    AP_GROUPINFO("VSPEED", 1, SoaringController, thermal_vspeed, 50.0f),
    
    // @Param: THERMAL_Q
    // @DisplayName: Process noise
    // @Description: Standard deviation of noise in process
    // @Units: 
    // @Range: 0 10
    // @User: Advanced 
    AP_GROUPINFO("Q", 2, SoaringController, thermal_q, 0.01f),
    
    // @Param: THERMAL_R
    // @DisplayName: Measurement noise
    // @Description: Standard deviation of noise in measurement
    // @Units: 
    // @Range: 0 10
    // @User: Advanced 
    AP_GROUPINFO("R", 3, SoaringController, thermal_r, 0.45f),
    
    AP_GROUPEND
};
    
void SoaringController::get_target(Location &wp)
{
    wp=_prev_update_location;
    location_offset(wp,ekf.X[2],ekf.X[3]);
}
bool SoaringController::suppress_throttle()
{
    return false;
}

bool SoaringController::check_thermal_criteria()
{
    return(soar_active && (( hal.scheduler->millis() - _cruise_start_time_ms ) > MIN_CRUISE_TIME_MS) && _vario_reading > thermal_vspeed);
}
bool SoaringController::check_cruise_criteria()
{
    float thermalability = (ekf.X[0]*exp(-pow(_loiter_rad/ekf.X[1],2)))-EXPECTED_THERMALLING_SINK; 
    if (soar_active && (hal.scheduler->millis()-_thermal_start_time_ms) > MIN_THERMAL_TIME_MS && thermalability < McCready(_alt)) {
        hal.console->printf_P(PSTR("Thermal weak, recommend quitting: W %f R %f th %f alt %f Mc %f\n"),ekf.X[0],ekf.X[1],thermalability,_alt,McCready(_alt));
        return true;
    }
    return false;
}
void SoaringController::init_thermalling()
{
    //Calc filter matrices - so that changes to parameters can be updated by switching in and out of thermal mode
    float r[1][1] = {{pow(thermal_r,2)}};
    float cov_q = pow(thermal_q,2); //State covariance
    float q[N][N] = {{cov_q, 0, 0, 0},{0, cov_q, 0, 0},{0,0, cov_q,0},{0,0,0,cov_q}};
         
    // New state vector filter will be reset. Thermal location is placed 10m in front of a/c 
    float xr[] = {INITIAL_THERMAL_STRENGTH, INITIAL_THERMAL_RADIUS, THERMAL_DISTANCE_AHEAD*cos(_ahrs.yaw), THERMAL_DISTANCE_AHEAD*sin(_ahrs.yaw)};      
    // Also reset covariance matrix p so filter is not affected by previous data       
    ekf.reset(xr,p,q,r);
    _ahrs.get_position(_prev_update_location);
    _prev_update_time = hal.scheduler->millis();
    _thermal_start_time_ms = hal.scheduler->millis();
}
   
void SoaringController::init_cruising()
{
    _cruise_start_time_ms = hal.scheduler->millis();
}

void SoaringController::update_thermalling(float loiter_radius)
{
    
    _loiter_rad = loiter_radius;
    struct Location current_loc;
    _ahrs.get_position(current_loc);
    if (_new_data) {
        float dx = get_offset_north(_prev_update_location, current_loc);  // get distances from previous update
        float dy = get_offset_east(_prev_update_location, current_loc);

        if (0) {
            // Wind correction currently unused
            //Vector3f wind = _ahrs.wind_estimate();
            //dx += wind.x * (millis()-_prev_update_time)/1000.0;
            //dy += wind.y * (millis()-_prev_update_time)/1000.0;
        }

        if (0) {
            // Print filter info for debugging
            //hal.console->printf_P(PSTR("%f %f %f "),netto_rate, dx, dy);

            int i;
            for (i=0;i<4;i++) {
                hal.console->printf_P(PSTR("%e "),ekf.P[i][i]);
            }
            for (i=0;i<4;i++) {
                hal.console->printf_P(PSTR("%e "),ekf.X[i]);
            }
            //hal.console->printf_P(PSTR("%ld %ld %f %f %f %f\n"),current_loc.lng, current_loc.lat, airspeed.get_airspeed(), alt, ahrs.roll, climb_rate_unfilt);
        }
        else {
             // write log - save the data.
            log_tuning.time_ms = hal.scheduler->millis();
            log_tuning.netto_rate = _vario_reading;
            log_tuning.dx = dx;
            log_tuning.dy = dy;
            log_tuning.x0 = ekf.X[0];
            log_tuning.x1 = ekf.X[1];
            log_tuning.x2 = ekf.X[2];
            log_tuning.x3 = ekf.X[3];
            log_tuning.lat = current_loc.lat;
            log_tuning.lng = current_loc.lng;
            log_tuning.alt = _alt;
        }
        log_data(); 
        ekf.update(_vario_reading,dx, dy);                              // update the filter
         
        _prev_update_location = current_loc;      // save for next time
        _prev_update_time = hal.scheduler->millis();
        _new_data = false;
    }
    else {
    }
}
void SoaringController::update_cruising()
{
    // Do nothing
}
void SoaringController::update_vario()
{   
    Location current_loc;
    _ahrs.get_position(current_loc);
    _alt = current_loc.alt/100.0f;
    if (!(_alt==_last_alt)) {
    // Both filtered total energy rates and unfiltered are computed for the thermal switching logic and the EKF
        float aspd;
        float roll = _ahrs.roll;
        if (!_ahrs.airspeed_estimate(&aspd)) {
            aspd = 0.5f*(aparm.airspeed_min+aparm.airspeed_max); //
        }
        float total_E = _alt+0.5*aspd*aspd/GRAVITY_MSS;                                                                 // Work out total energy                                                 // Feed into the derivative filter
        float sinkrate = correct_netto_rate(0.0f, (roll+_last_roll)/2, (aspd+_last_aspd)/2);                            // Compute still-air sinkrate
        _vario_reading = (total_E - _last_total_E)*1000.0/(hal.scheduler->millis()-_prev_vario_update_time) + sinkrate; // Unfiltered netto rate
        _filtered_vario_reading = 0.9048*_filtered_vario_reading + 0.0952*_vario_reading;                               // Apply low pass 2sec timeconst filter for noise
        _last_alt = _alt;                                       // Store variables
        _last_roll=roll;
        _last_aspd = aspd;
        _last_total_E = total_E;
        _prev_vario_update_time = hal.scheduler->millis();
        _new_data=true;
    }
}

float SoaringController::correct_netto_rate(float climb_rate, float phi, float aspd) {
    // Remove aircraft sink rate
    float CL0;  // CL0 = 2*W/(rho*S*V^2)
    float C1;   // C1 = CD0/CL0
    float C2;   // C2 = CDi0/CL0 = B*CL0
    float netto_rate;
    float cosphi;
    CL0 = CL_FACTOR/(aspd*aspd);  
    C1 = CD0/CL0;  // constant describing expected angle to overcome zero-lift drag
    C2 = B*CL0;    // constant describing expected angle to overcome lift induced drag at zero bank

    cosphi = (1 - phi*phi/2); // first two terms of mclaurin series for cos(phi)
    netto_rate = climb_rate + aspd*(C1 + C2/(cosphi*cosphi));  // effect of aircraft drag removed

    //Remove acceleration effect - needs to be tested.
    //float temp_netto = netto_rate;
    //float dVdt = SpdHgt_Controller->get_VXdot();
    //netto_rate = netto_rate + aspd*dVdt/GRAVITY_MSS;
    //hal.console->printf_P(PSTR("%f %f %f %f\n"),temp_netto,dVdt,netto_rate,barometer.get_altitude());
    return netto_rate;
}
 
float SoaringController::McCready(float alt) {
    float XP[] = {500, 3000};
    float YP[] = {0, 4};
    int n = 2;
    // Linear interpolation (without extrap)
    if (alt<=XP[0]) { 
        return YP[0];
    }
    else if (alt>=XP[n-1]){
        return YP[n-1];
    }
    else {
        for (int i=0;i<n;i++) {
            if (alt>=XP[i]) {
                return (((alt-XP[i]) * (YP[i+1]-YP[i]) /(XP[i+1]-XP[i])) + YP[i]);
            }     
        }
    }
    return -1.0; // never happens   
 }
        
// log the contents of the log_tuning structure to dataflash
void SoaringController::log_data()
{
    log_tuning.head1 = HEAD_BYTE1;
    log_tuning.head2 = HEAD_BYTE2;
    log_tuning.msgid = _msgid;
    _dataflash->WriteBlock(&log_tuning, sizeof(log_tuning));
}

    
    
    