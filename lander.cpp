// Mars lander simulator
// Version 1.11
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"
#include <math.h>

double k = GRAVITY * MARS_MASS;   // G*M in gravity calc. 
double r_mars = MARS_RADIUS;
double lander_const = M_PI * LANDER_SIZE * LANDER_SIZE * DRAG_COEF_LANDER; //area * drag coeff
double parachute_const = 5.0 * 4.0 * LANDER_SIZE * LANDER_SIZE * DRAG_COEF_CHUTE; //area * drag coeff
double Kh = 0.02;
double Kp = 0.88;
double delta = 0;
double Pout;
// in lander.h, FUEL_RATE_AT_MAX_THRUST  = 0.5 by default. changed to 0 for debugging, and infinite fuel


void autopilot (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{
    Pout = -Kp * (0.5 + Kh * (position.abs() - r_mars) + velocity * (position.norm()));
    if (Pout <= -delta) {
        throttle = 0;
    }
    else 
        if (Pout < 1 - delta) {
            throttle = delta + Pout;

        
    }
    else {
        throttle = 1;
    }
	cout << "Pout: " << Pout << " Throttle: " << throttle << endl;
}



void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's position The time step is delta_t (global variable).
{
  // INSERT YOUR CODE HERE
  // verlet intergator

  static vector3d previous_position;
  vector3d new_position;
  vector3d thrust = thrust_wrt_world();
  vector3d acceleration;

  
  double mag_pos = position.abs();
  double mag_vel = velocity.abs();
  vector3d drag = (0.5) * atmospheric_density(position) * lander_const * mag_vel * velocity;
  if (parachute_status == DEPLOYED) {
      drag += (0.5) * atmospheric_density(position) * parachute_const * mag_vel * velocity;
  }
  double lander_mass_current = UNLOADED_LANDER_MASS + fuel * FUEL_CAPACITY*FUEL_DENSITY;
  acceleration = (thrust - (k / pow(mag_pos, 3)) * position * lander_mass_current - drag) / lander_mass_current;

  if (simulation_time == 0.0) {

      //do an Euler update for the first iteration 
      new_position = position + delta_t * velocity;
      velocity = velocity + delta_t * acceleration;
  }
  else {
      //do a Verlet update on all subsequent iterations 
	  new_position = 2 * position - previous_position + delta_t * delta_t * acceleration;
	  velocity = (new_position - previous_position) / (2 * delta_t);
  }


   previous_position = position;

   position = new_position;

  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled) autopilot();

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude) attitude_stabilization();
}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "";
  scenario_description[7] = "";
  scenario_description[8] = "";
  scenario_description[9] = "";

  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 6:
    break;

  case 7:
    break;

  case 8:
    break;

  case 9:
    break;

  }
}
