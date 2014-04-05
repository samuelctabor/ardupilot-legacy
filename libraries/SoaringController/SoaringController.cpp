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
    AP_GROUPINFO("Q", 2, SoaringController, thermal_q, 0.1f),
    
    // @Param: THERMAL_R
    // @DisplayName: Measurement noise
    // @Description: Standard deviation of noise in measurement
    // @Units: 
    // @Range: 0 10
    // @User: Advanced 
    AP_GROUPINFO("R", 3, SoaringController, thermal_r, 0.1f),
    
    AP_GROUPEND
};
    
void SoaringController::get_target(Location &wp)
{
    wp=prev_update_location;
    location_offset(wp,ekf.X[2],ekf.X[3]);
}
bool SoaringController::suppress_throttle()
{
    return false;
}

bool SoaringController::check_thermal_criteria()
{
    hal.console->printf_P(PSTR("vario %f\n"),_vario_reading);
    return (_vario_reading > thermal_vspeed);
}
bool SoaringController::check_cruise_criteria()
{
    float thermalability = (ekf.X[0]*exp(-pow(_loiter_rad/ekf.X[1],2)))-EXPECTED_THERMALLING_SINK; 

    //hal.console->printf_P(PSTR("Thermal weak, recommend quitting: W %f R %f th %f alt %f Mc %f\n"),ekf.X[0],ekf.X[1],thermalability,rel_alt,McCready(rel_alt));
    hal.console->printf_P(PSTR("Thermalability %f McC %f \n"),thermalability, McCready(_alt));
   
    return (thermalability < McCready(_alt));
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
    _ahrs.get_position(prev_update_location);
    //_last_alt = prev_update_location.alt/100.0f;
    prev_update_time = hal.scheduler->millis();
}
   
void SoaringController::init_cruising()
{
    //requested_flight_mode = AUTO;
}

void SoaringController::update_thermalling(float loiter_radius)
{
    hal.console->printf_P(PSTR("Updating thermalling, vario %f\n"),_vario_reading);
    _loiter_rad = loiter_radius;
    struct Location current_loc;
    _ahrs.get_position(current_loc);
    if (!(current_loc.lat==prev_update_location.lat)) {
        float dx = get_offset_north(prev_update_location, current_loc);  // get distances from previous update
        float dy = get_offset_east(prev_update_location, current_loc);

        if (0) {
            // Wind correction currently unused
            //Vector3f wind = _ahrs.wind_estimate();
            //dx += wind.x * (millis()-prev_update_time)/1000.0;
            //dy += wind.y * (millis()-prev_update_time)/1000.0;
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
         
        ekf.update(_vario_reading,dx, dy);                              // update the filter
         
        prev_update_location = current_loc;      // save for next time
        prev_update_time = hal.scheduler->millis();
    }
}
void SoaringController::update_cruising()
{
    // Do nothing
}
void SoaringController::update_vario()
{   
    //hal.console->printf_P(PSTR("Updating vario . . .\n"));
    Location current_loc;
    _ahrs.get_position(current_loc);
    _alt = current_loc.alt/100.0f;
    //hal.console->printf_P(PSTR("    new alt %f last %f %u\n"),_alt, _last_alt, _alt==_last_alt);
    if (!(_alt==_last_alt)) {
        float dhdt = 1000.0*(_alt - _last_alt)/(hal.scheduler->millis()-_prev_vario_update_time);
        // Correct for aircraft sink
        float aspd;
        
        if (!_ahrs.airspeed_estimate(&aspd)) {
            aspd = 0.5f*(aparm.airspeed_min+aparm.airspeed_max); //
        }
        _vario_reading = correct_netto_rate(dhdt, _ahrs.roll, aspd);
        //hal.console->printf_P(PSTR("Vario    %f %f\n"),dhdt,_vario_reading);
        _last_alt = _alt;
        _prev_vario_update_time = hal.scheduler->millis();
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
void SoaringController::log_data(DataFlash_Class &dataflash, uint8_t msgid)
{
    log_tuning.head1 = HEAD_BYTE1;
    log_tuning.head2 = HEAD_BYTE2;
    log_tuning.msgid = msgid;
	dataflash.WriteBlock(&log_tuning, sizeof(log_tuning));
}

    
    
    