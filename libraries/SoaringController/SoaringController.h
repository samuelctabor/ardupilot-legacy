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
#define N 4
#define THERMAL_DISTANCE_AHEAD 50.0
#define EXPECTED_THERMALLING_SINK 0.9
#define CD0 0.0283
#define B 0.04
#define CL_FACTOR 109.5  // 109.5 = 2*W/(rho*S)=2*6.5*9.81/(1.2*0.98)
#define INITIAL_THERMAL_STRENGTH 2.0
#define INITIAL_THERMAL_RADIUS 300.0
#define INITIAL_STRENGTH_COVARIANCE 2.0
#define INITIAL_RADIUS_COVARIANCE 2500.0
#define INITIAL_POSITION_COVARIANCE 300.0

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
 struct Location prev_next_wp;
 
 //Store aircraft location at last update
 struct Location prev_update_location;
 
 // store time thermal was entered for hysteresis
 unsigned long thermal_start_time_ms;
 
  // store time cruise was entered for hysteresis
 unsigned long cruise_start_time_ms;
 
 // store time of last update
 unsigned long prev_update_time;
 
  // store time of last update
 unsigned long _prev_vario_update_time;
 //gcs_send_text_P(SEVERITY_LOW, PSTR("Soar initialisation complete"));
 
 float _vario_reading;
 float _last_alt;
 float _alt;
 bool _new_data;
 float _loiter_rad; // Loiter radius passed in
 uint8_t _msgid;
 DataFlash_Class* _dataflash;
 float correct_netto_rate(float climb_rate, float phi, float aspd);
 float McCready(float alt);
 protected:
   AP_Float soar_active;
  AP_Float thermal_vspeed;
  AP_Float thermal_q;
  AP_Float thermal_r;
  public:
  SoaringController(AP_AHRS &ahrs, AP_SpdHgtControl *&spdHgt, const AP_Vehicle::FixedWing &parms, DataFlash_Class* dataflash, uint8_t msgid) :
    _ahrs(ahrs),
    aparm(parms),
    _spdHgt(spdHgt),
    _dataflash(dataflash),
    _msgid(msgid)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }
    
    
  // this supports the TECS_* user settable parameters
  static const struct AP_Param::GroupInfo var_info[];
  void get_target(Location & wp);
  bool suppress_throttle();
  //void log_data(DataFlash_Class &dataflash, uint8_t msgid);
  void log_data();
  bool check_thermal_criteria();
  bool check_cruise_criteria();
  void init_thermalling();
  void init_cruising();
  void update_thermalling(float loiter_radius);
  void update_cruising();
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
	} log_tuning;
void update_vario();

  
                 
};
#define THML_LOG_FORMAT(msg) { msg, sizeof(SoaringController::log_tuning),	\
							   "THML",  "IfffffffLLf",     "TimeMS,nettorate,dx,dy,x0,x1,x2,x3,lat,lng,alt" }


#endif
