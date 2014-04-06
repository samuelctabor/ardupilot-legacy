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

 // Keep track of the previous flight mode so we can transition back
 // When we come out of thermal mode.
 static FlightMode previous_control_mode;
 
 static void update_soaring() {
   soaring_controller.update_vario();
   switch (control_mode)
   {
     case AUTO:
     case FLY_BY_WIRE_B:
       // Test for switch into thermalling mode
       soaring_controller.update_cruising();

       if (soaring_controller.check_thermal_criteria()) {
         hal.console->printf_P(PSTR("Thermal detected, entering loiter\n"));
         soaring_controller.init_thermalling();
         previous_control_mode = control_mode;
         set_mode(LOITER);
         soaring_controller.get_target(next_WP_loc); // ahead on flight path
       }
       break;
     case LOITER:
       // Update thermal estimate abd check for switch back to AUTO
       soaring_controller.update_thermalling(g.loiter_radius);  // Update estimate

       if (soaring_controller.check_cruise_criteria()) {
         // Exit as soon as thermal state estimate deteriorates
         soaring_controller.init_cruising();
         set_mode(previous_control_mode);
       }
       else {
         // still in thermal - need to update the wp location
         soaring_controller.get_target(next_WP_loc); 
       }
       break;
   }
 }
