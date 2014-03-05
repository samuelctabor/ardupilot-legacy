 /// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/*
 *  ArduSoar support functions
 *
 *  Peter Braswell 
 * 
 *  This firmware is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 */
#include <MatrixMath.h>
#include <ExtendedKalmanFilter.h>

#define N 4
#define MIN_THERMAL_TIME_MS  60000
#define MIN_CRUISE_TIME_MS  120000
#define RUN_FILTER 1
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

//gcs_send_text_P(SEVERITY_LOW, PSTR("In soar code, initialising variables"));

float p[N][N] = {{INITIAL_STRENGTH_COVARIANCE, 0,                         0,                           0},
                 {0,                           INITIAL_RADIUS_COVARIANCE, 0,                           0},
                 {0,                           0,                         INITIAL_POSITION_COVARIANCE, 0},
                 {0,                           0,                         0,                           INITIAL_POSITION_COVARIANCE }}; //Covariance matrix

ExtendedKalmanFilter ekf;  



 // Keep track of the previous flight mode so we can transition back
 // When we come out of thermal mode.
 static FlightMode previous_control_mode;
 
 // Keep track of the waypoint so we can restore after coming out of thermal mode.
 static struct Location prev_next_wp;
 
 //Store aircraft location at last update
 static struct Location prev_update_location;
 
 // store time thermal was entered for hysteresis
 unsigned long thermal_start_time_ms;
 
  // store time cruise was entered for hysteresis
 unsigned long cruise_start_time_ms;
 
 // store time of last update
 unsigned long prev_update_time;
 //gcs_send_text_P(SEVERITY_LOW, PSTR("Soar initialisation complete"));
 
 float last_alt;
 

 // Check to see if see if we should be thermalling
 static FlightMode thermal(FlightMode current_control_mode) {

   FlightMode calculated_control_mode = current_control_mode;
   
   if( g.soar_active == 1 ) {
       if ((read_netto_rate(read_climb_rate()) > g.thermal_vspeed ) && (( millis()- cruise_start_time_ms ) > MIN_CRUISE_TIME_MS )) {  
       hal.console->printf_P(PSTR("Thermal detected, entering loiter\n"));
       previous_control_mode = current_control_mode;
       
       prev_next_wp = next_WP;
       
       next_WP = current_loc; // filter offsets based on ac location
         
       prev_update_location = current_loc; // needed to see how far the move the thermal relative to the a/c
       
       calculated_control_mode =  LOITER;
       
       thermal_start_time_ms = millis();
        
       //Calc filter matrices - so that changes to parameters can be updated by switching in and out of thermal mode
       float r[1][1] = {{pow(g.thermal_r,2)}};
       float cov_q = pow(g.thermal_q,2); //State covariance
       float q[N][N] = {{cov_q, 0, 0, 0},{0, cov_q, 0, 0},{0,0, cov_q,0},{0,0,0,cov_q}};
             
       // New state vector filter will be reset. Thermal location is placed 10m in front of a/c 
       float xr[] = {INITIAL_THERMAL_STRENGTH, INITIAL_THERMAL_RADIUS, THERMAL_DISTANCE_AHEAD*cos(ahrs.yaw), THERMAL_DISTANCE_AHEAD*sin(ahrs.yaw)};      
       // Also reset covariance matrix p so filter is not affected by previous data       
       ekf.reset(xr,p,q,r);
       
       location_offset(next_WP, ekf.X[2], ekf.X[3]); //place waypoint to reflect filter state
       last_alt = barometer.get_altitude();  // So that the first delta is not nonsense
       prev_update_location = current_loc;                                // save for next time
       prev_update_time = millis();
     }
   }
   
   return calculated_control_mode;
   
 }
 
 // Check to see if we've topped out of a thermal and 
 // Should transition to cruise (or rather the previous control mode).
 static FlightMode cruise(FlightMode current_control_mode) {

   FlightMode calculated_control_mode = current_control_mode;  // default  behaviour is to keep current mode
 
   if ( g.soar_active == 1 ) {

     float thermalability = (ekf.X[0]*exp(-pow(g.loiter_radius/ekf.X[1],2)))-EXPECTED_THERMALLING_SINK; 
     
     if (current_loc.lat == prev_update_location.lat) {
       // Bail out if new new data has come in from simulator/GPS
       return calculated_control_mode;
     }
     
      
     // Compute unfiltered climb rate
     float alt = barometer.get_altitude();
     
     float rel_alt = alt - (float)home.alt/100.0;
     
     if (alt==last_alt) {
       return calculated_control_mode;
     }
       
     float climb_rate_unfilt = 1000.0*(alt - last_alt)/(millis()-prev_update_time);
     // Correct for aircraft sink
     float netto_rate = read_netto_rate(climb_rate_unfilt);
     
     if ((thermalability < McCready(rel_alt)) && ((millis()-thermal_start_time_ms) > MIN_THERMAL_TIME_MS)) {
       // Exit as soon as thermal state estimate deteriorates
       hal.console->printf_P(PSTR("Thermal weak, reentering previous mode: W %f R %f th %f alt %f Mc %f\n"),ekf.X[0],ekf.X[1],thermalability,rel_alt,McCready(rel_alt));
       calculated_control_mode =  previous_control_mode;
       next_WP = prev_next_wp;    // continue to the waypoint being used before thermal mode
       cruise_start_time_ms = millis();
     }
     else {
       // still in thermal - need to update the wp location and update the filter according to new measurement

       float dx = get_offset_north(prev_update_location, current_loc);  // get distances from previous update
       float dy = get_offset_east(prev_update_location, current_loc);
       
       if (0) {
         // Wind correction currently unused
         //Vector3f wind = ahrs.wind_estimate();
         //dx += wind.x * (millis()-prev_update_time)/1000.0;
         //dy += wind.y * (millis()-prev_update_time)/1000.0;
       }
       
       if (0) {
         // Print filter info for debugging
         hal.console->printf_P(PSTR("%f %f %f "),netto_rate, dx, dy);
         
         int i;
         for (i=0;i<4;i++) {
             hal.console->printf_P(PSTR("%e "),ekf.P[i][i]);
         }
         for (i=0;i<4;i++) {
           hal.console->printf_P(PSTR("%e "),ekf.X[i]);
         }
         hal.console->printf_P(PSTR("%ld %ld %f %f %f %f\n"),current_loc.lng, current_loc.lat, airspeed.get_airspeed(), alt, ahrs.roll, climb_rate_unfilt);
       }
       
       ekf.update(netto_rate,dx, dy);                              // update the filter
       
       next_WP = current_loc; // as filter estimate is based on offset from current location
       location_offset(next_WP, ekf.X[2], ekf.X[3]); //update the WP
       
       prev_update_location = current_loc;      // save for next time
       prev_update_time = millis();
       last_alt = alt;
     }
   }
   return calculated_control_mode;
 }
 
 static float read_netto_rate(float climb_rate) {
   // Remove aircraft sink rate
   float aspd;
   float CL0;  // CL0 = 2*W/(rho*S*V^2)
   float C1;   // C1 = CD0/CL0
   float C2;   // C2 = CDi0/CL0 = B*CL0
   float netto_rate;
   float phi;
   float cosphi;
   aspd = airspeed.get_airspeed();
   CL0 = CL_FACTOR/(aspd*aspd);  
   C1 = CD0/CL0;  // constant describing expected angle to overcome zero-lift drag
   C2 = B*CL0;    // constant describing expected angle to overcome lift induced drag at zero bank
  
   phi = ahrs.roll; //approximately bank angle
   cosphi = (1 - phi*phi/2); // first two terms of mclaurin series for cos(phi)
   netto_rate = climb_rate + aspd*(C1 + C2/(cosphi*cosphi));  // effect of aircraft drag removed
   
   //Remove acceleration effect - needs to be tested.
   float temp_netto = netto_rate;
   float dVdt = SpdHgt_Controller->get_VXdot();
   netto_rate = netto_rate + aspd*dVdt/GRAVITY_MSS;
   hal.console->printf_P(PSTR("%f %f %f %f\n"),temp_netto,dVdt,netto_rate,barometer.get_altitude());
   return netto_rate;
 }
 
 static float McCready(float alt) {
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
 
