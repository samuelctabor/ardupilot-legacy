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
#define MIN_THERMAL_TIME_MS  60000
#define MIN_CRUISE_TIME_MS  120000

 // Keep track of the previous flight mode so we can transition back
 // When we come out of thermal mode.
 static FlightMode previous_control_mode;
 
 // Keep track of the waypoint so we can restore after coming out of thermal mode.
 static struct Location prev_next_wp;
 
 // store time thermal was entered for hysteresis
 unsigned long thermal_start_time_ms;
 
  // store time cruise was entered for hysteresis
 unsigned long cruise_start_time_ms;
 
 // Check to see if see if we should be thermalling
 static FlightMode thermal(FlightMode current_control_mode) {
   soaring_controller.update_vario();
   soaring_controller.update_cruising();
   FlightMode calculated_control_mode = current_control_mode;

   if ((( millis()- cruise_start_time_ms ) > MIN_CRUISE_TIME_MS) && soaring_controller.check_thermal_criteria()) {
         
       hal.console->printf_P(PSTR("Thermal detected, entering loiter\n"));
       soaring_controller.init_thermalling();
       previous_control_mode = current_control_mode;
       
       prev_next_wp = next_WP_loc;
       calculated_control_mode =  LOITER;
       thermal_start_time_ms = millis();
   }
   return calculated_control_mode;
 }
 
 // Check to see if we've topped out of a thermal and 
 // Should transition to cruise (or rather the previous control mode).
 static FlightMode cruise(FlightMode current_control_mode) {
   soaring_controller.update_vario();
   soaring_controller.update_thermalling(g.loiter_radius);
   FlightMode calculated_control_mode = current_control_mode;  // default  behaviour is to keep current mode

   if ( soaring_controller.check_cruise_criteria() && (millis()-thermal_start_time_ms) > MIN_THERMAL_TIME_MS) {
     // Exit as soon as thermal state estimate deteriorates
     calculated_control_mode =  previous_control_mode;
     next_WP_loc = prev_next_wp;    // continue to the waypoint being used before thermal mode
     cruise_start_time_ms = millis();
   }
   else {
     // still in thermal - need to update the wp location and update the filter according to new measurement
     soaring_controller.get_target(next_WP_loc); 
   }
   return calculated_control_mode;
 }
 
 
 
