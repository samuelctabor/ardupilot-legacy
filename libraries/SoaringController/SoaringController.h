/* 
Soaring Controller class by Samuel Tabor
Provides a layer between the thermal centring algorithm and the main code for managing navigation targets, data logging, tuning parameters, algorithm inputs and eventually other soaring strategies such as speed-to-fly. AP_TECS libary used for reference.
*/
#ifndef SoaringController_h
#define SoaringController_h

#include <AP_AHRS.h>
#include <AP_Param.h>
#include <DataFlash.h>
#include "MatrixMath.h"
#include "math.h"
#include "ExtendedKalmanFilter.h"
#include <AP_SpdHgtControl.h>
#define EXPECTED_THERMALLING_SINK 0.7
#define INITIAL_THERMAL_STRENGTH 2.0
#define INITIAL_THERMAL_RADIUS 150.0
#define INITIAL_STRENGTH_COVARIANCE 0.000049
#define INITIAL_RADIUS_COVARIANCE 2500.0
#define INITIAL_POSITION_COVARIANCE 300.0
#define ASPD_FILT 0.05
#define TE_FILT 0.03
class SoaringController
{
   

  ExtendedKalmanFilter ekf;
  AP_AHRS &_ahrs;
  const AP_Vehicle::FixedWing &aparm;
  

  
  AP_SpdHgtControl *&_spdHgt;
  float p[N][N] = {{INITIAL_STRENGTH_COVARIANCE, 0,                         0,                           0},
                 {0,                           INITIAL_RADIUS_COVARIANCE, 0,                           0},
                 {0,                           0,                         INITIAL_POSITION_COVARIANCE, 0},
                 {0,                           0,                         0,                           INITIAL_POSITION_COVARIANCE }}; //Covariance matrix

 
 // Keep track of the waypoint so we can restore after coming out of thermal mode.
 struct Location _prev_next_wp;
 
 //Store aircraft location at last update
 struct Location _prev_update_location;
 
 // store time thermal was entered for hysteresis
 unsigned long _thermal_start_time_ms;
 
  // store time cruise was entered for hysteresis
 unsigned long _cruise_start_time_ms;
 
 // store time of last update
 unsigned long _prev_update_time;
 
  // store time of last update
 unsigned long _prev_vario_update_time;
 //gcs_send_text_P(SEVERITY_LOW, PSTR("Soar initialisation complete"));
 
 float _vario_reading;
 float _filtered_vario_reading;
 float _last_alt;
 float _alt;
 float _last_aspd;
 float _last_roll;
 float _last_total_E;
 bool _new_data;
 float _loiter_rad; // Loiter radius passed in
 bool _throttle_suppressed;
 float _aspd_filt;
 uint8_t _msgid;
 uint8_t _msgid2;
 DataFlash_Class* _dataflash;
 float correct_netto_rate(float climb_rate, float phi, float aspd);
 float McCready(float alt);
 protected:
  AP_Int8 soar_active;
  AP_Float thermal_vspeed;
  AP_Float thermal_q1;
  AP_Float thermal_q2;
  AP_Float thermal_r;
  AP_Float thermal_distance_ahead;
  AP_Int16 min_thermal_s;
  AP_Int16 min_cruise_s;
  AP_Float polar_CD0;
  AP_Float polar_B;
  AP_Float polar_K;
  AP_Float alt_max;
  AP_Float alt_min;
  AP_Float alt_cutoff;
  
  public:
  SoaringController(AP_AHRS &ahrs, AP_SpdHgtControl *&spdHgt, const AP_Vehicle::FixedWing &parms, DataFlash_Class* dataflash, uint8_t msgid, uint8_t msgid2) :
    _ahrs(ahrs),
    aparm(parms),
    _spdHgt(spdHgt),
    _dataflash(dataflash),
    _msgid(msgid),
    _msgid2(msgid2)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }
    
    
  // this supports the TECS_* user settable parameters
  static const struct AP_Param::GroupInfo var_info[];
  void get_target(Location & wp);
  bool suppress_throttle();
  bool get_throttle_suppressed();
  //void log_data(DataFlash_Class &dataflash, uint8_t msgid);
  void log_data();
  bool check_thermal_criteria();
  bool check_cruise_criteria();
  void init_thermalling();
  void init_cruising();
  void update_thermalling(float loiter_radius);
  void update_cruising();
  bool is_active();
  // Soaring log structure
struct PACKED log_Thermal_Tuning {
	LOG_PACKET_HEADER;		
	uint32_t time_ms;
	float netto_rate;
	float dx;
	float dy;
	float x0;
	float x1;		
	float x2;		
	float x3;		
	uint32_t lat;		
	uint32_t lng;		
	float alt;
    float dx_w;
    float dy_w;
	} log_tuning;
    
    struct PACKED log_Vario_Tuning {
	LOG_PACKET_HEADER;		
	uint32_t time_ms;
	float aspd_raw;
    float aspd_filt;
	float alt;
	float roll;
	float raw;
	float filt;		
	} log_vario_tuning;
void update_vario();

  
                 
};
#define THML_LOG_FORMAT(msg) { msg, sizeof(SoaringController::log_tuning),	\
							   "THML",  "IfffffffLLfff",     "TimeMS,nettorate,dx,dy,x0,x1,x2,x3,lat,lng,alt,dx_w,dy_w" }
#define VARIO_LOG_FORMAT(msg) { msg, sizeof(SoaringController::log_vario_tuning),	\
							   "VAR",  "Iffffff",     "TimeMS,aspd_raw,aspd_filt,alt,roll,raw,filt" }


#endif
