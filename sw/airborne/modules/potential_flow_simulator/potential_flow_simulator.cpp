/*
 * Copyright (C) Sunyou Hwang <S.Hwang-1@tudelft.nl>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/potential_flow_simulator/potential_flow_simulator.c"
 * @author Sunyou Hwang <S.Hwang-1@tudelft.nl>
 * Potential flow simulator for orographic soaring (Never Landing Drone project)

 */


#include <cstdio>
#include <cstdlib>
#include <string>
#include <iostream>
#include <sys/time.h>

extern "C" {
#include "modules/potential_flow_simulator/potential_flow_simulator.h"
#include "state.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_geodetic_double.h"
#include "subsystems/datalink/telemetry.h"
#include "nps_atmosphere.h"
}

using namespace std;

// define radius of the obstacle - TODO: building and boat?
#ifndef PF_OBSTACLE_RADIUS
#define PF_OBSTACLE_RADIUS 20
#endif
// surface roughness
#ifndef PF_SURFACE_ROUGHNESS
#define PF_SURFACE_ROUGHNESS 0.05
#endif
// Initial obstacle direction (starting from North, in degrees)
#ifndef PF_INITIAL_OBSTACLE_DIR
#define PF_INITIAL_OBSTACLE_DIR 0
#endif
// type of the obstacle (0: boat, 1: hill, 2: building, ...)
// for boat, use ground gps. for others, set heading angle and position (center point)
#ifndef PF_OBSTACLE_TYPE
#define PF_OBSTACLE_TYPE 1
#endif
#ifndef PF_OBSTACLE_POSITION_X
#define PF_OBSTACLE_POSITION_X 50
#endif
#ifndef PF_OBSTACLE_POSITION_Y
#define PF_OBSTACLE_POSITION_Y 0
#endif
#ifndef PF_OBSTACLE_POSITION_Z
#define PF_OBSTACLE_POSITION_Z 0
#endif
// reference wind velocity: x dir in the obstacle's body frame
#ifndef PF_REF_WIND_VEL
#define PF_REF_WIND_VEL 10
#endif
// whether use ground gps or not
#ifndef PF_USE_GROUND_GPS
#define PF_USE_GROUND_GPS FALSE
#endif
// set wind to nps environment?
#ifndef PF_SET_WIND_NPS_ENV
#define PF_SET_WIND_NPS_ENV FALSE
#endif

// other constants
#define HEADING_QUEUE_SIZE 10

static struct LlaCoor_i ground_lla; // lla coordinates received by the GPS message
struct UtmCoor_d ground_utm, ground_utm_old;
float ground_heading;

//struct FloatVect3 calc_relative_position(struct UtmCoor_f *utm_vehicle, struct UtmCoor_f *utm_ground);
//struct LlaCoor_d *to_lla_d(struct LlaCoor_f *_lla_f);
//struct FloatVect3 rotate_frame(struct FloatVect3 *point, float theta);
#if PF_USE_GROUND_GPS
void parse_ground_gps(uint8_t *buf);
void follow_me_set_heading(void);
float average_heading(float diffx, float diffy);
#endif
struct FloatVect3 compute_potential_flow(struct FloatVect3 rel_dist_v3f, float ref_wind_speed);


inline struct FloatVect3 calc_relative_position_utm(struct UtmCoor_f *utm_vehicle, struct UtmCoor_f *utm_ground){
    struct FloatVect3 rel_pos;
    rel_pos.x = utm_ground->east - utm_vehicle->east;
    rel_pos.y = utm_ground->north - utm_vehicle->north;
    rel_pos.z = utm_ground->alt - utm_vehicle->alt;
    return rel_pos;
}
inline struct FloatVect3 calc_relative_position_v3(struct FloatVect3 *utm_vehicle, struct FloatVect3 *utm_ground){
    struct FloatVect3 rel_pos;
    rel_pos.x = utm_ground->x - utm_vehicle->x;
    rel_pos.y = utm_ground->y - utm_vehicle->y;
    rel_pos.z = utm_ground->z - utm_vehicle->z;
    return rel_pos;
}

inline struct FloatVect3 rotate_frame(struct FloatVect3 *point, float theta){
    // Create return Vector for function
    struct FloatVect3 transformation;

    // Rotate point
    transformation.x = cosf(theta)*point->x + sinf(theta)*point->y;
    transformation.y = -sinf(theta)*point->x + cosf(theta)*point->y;
    transformation.z = point->z;

    // Return
    return transformation;
}

inline struct LlaCoor_d to_lla_d(struct LlaCoor_f *_lla_f) {
    struct LlaCoor_d _lla_d;
    _lla_d.lat = _lla_f->lat;
    _lla_d.lon = _lla_f->lon;
    _lla_d.alt = _lla_f->alt;
    return _lla_d;
}

// utm: ENU -> NED
inline struct FloatVect3 utm_to_v3(struct UtmCoor_d *_utm_d) {
    struct FloatVect3 _fv3;
    _fv3.x = _utm_d->north;
    _fv3.y = _utm_d->east;
    _fv3.z = -_utm_d->alt;
    return _fv3;
}



/// three functions below here are from follow_me_controller
// void follow_me_compute_wp(void);
// Function that is executed each time the GROUND_GPS message is received
#if PF_USE_GROUND_GPS
void parse_ground_gps(uint8_t *buf){
    if(DL_GROUND_GPS_ac_id(buf) != AC_ID)
        return;

    // Save the received values
    ground_lla.lat = DL_GROUND_GPS_lat(buf);
    ground_lla.lon = DL_GROUND_GPS_lon(buf);
    ground_lla.alt = DL_GROUND_GPS_alt(buf);

    // ground_speed = DL_GROUND_GPS_speed(buf);
    // ground_climb = DL_GROUND_GPS_climb(buf);
    // ground_course = DL_GROUND_GPS_course(buf);
    old_ground_timestamp = ground_timestamp;
    ground_timestamp = DL_GROUND_GPS_timestamp(buf);
    // fix_mode = DL_GROUND_GPS_mode(buf);


    /// convert lla to utm for calculating relative position btw the ground station and vehicle
    struct LlaCoor_f _lla;
    _lla.lat = RadOfDeg((float)(ground_lla.lat / 1e7));     // utm2lla requires rad & meters
    _lla.lon = RadOfDeg((float)(ground_lla.lon / 1e7));
    _lla.alt = ((float)(ground_lla.alt))/1000.;

    utm_of_lla_d(&ground_utm, &_lla); // east north alt;

    follow_me_set_heading();    // calc heading
}

//int counter_heading = 0;
void follow_me_set_heading(void){
    // Obtain follow me heading based on position
//    counter_heading++;
//    if (counter_heading == heading_calc_counter){
//        counter_heading = 0;
        float diff_x;
        float diff_y;
        // This is probably used in order to skip the first loop as ground_utm_old is set to 0 initially
        if (ground_utm_old.north != 0 && ground_utm_old.east != 0){
            diff_y = ground_utm.north - ground_utm_old.north;
            diff_x = ground_utm.east - ground_utm_old.east;
        } else {
            diff_x = 0;
            diff_y = 0;
        }

        // Obtain average heading over this new distance
        // Note atan2 gives results between -180 and 180
        ground_heading = average_heading(diff_x, diff_y);
        ground_utm_old = ground_utm_new;
//    }
}

// Calculate the average gps heading in order to predict where the boat is going
// This has to be done by summing up the difference in x and difference in y in order to obtain a vector addition
// The use of vectors makes it possible to also calculate the average over for example 359, 0 and 1 degree
float all_diff_x[HEADING_QUEUE_SIZE]={0};
float all_diff_y[HEADING_QUEUE_SIZE]={0};
// Parameter which keeps track of the value that needs to be replaced
uint8_t heading_queue_idx = 0;
//function definition
float average_heading(float diffx, float diffy)
{
    float Sum_x = 0;
    float Sum_y = 0;

    all_diff_x[heading_queue_idx] = diffx;
    all_diff_y[heading_queue_idx] = diffy;

    heading_queue_idx++;
    heading_queue_idx %= HEADING_QUEUE_SIZE;

    for (int i=0; i<HEADING_QUEUE_SIZE; i++){
        Sum_x = Sum_x + all_diff_x[i];
        Sum_y = Sum_y + all_diff_y[i];
    }

    // Check for condition in which we are not moving
    // In case we are not moving keep the current heading
    if ((fabs(Sum_x) < 4) && (fabs(Sum_y) < 4)){
        stationary_ground = 1;
        return default_heading;
    } else {
        stationary_ground = 0;
        float heading = 0.0;
        // First check cases which divide by 0
        if (Sum_y == 0.0){
            if (Sum_x > 0.0){
                heading = 90.0;
            } else if (Sum_x < 0.0){
                heading = -90.0;
            }
        } else {
            heading = atan2(Sum_x, Sum_y)*180.0/M_PI;  // returns value between -180 and 180 (at least no other values have been found yet)
        }
        return heading;
    }
}
#endif


/// the actual potential flow calculation
struct FloatVect3 compute_potential_flow(struct FloatVect3 rel_dist_v3f, float ref_wind_speed){
    float _y = rel_dist_v3f.y;
    float _z = rel_dist_v3f.z;

    float theta = atan2(-_z, _y);
    float r = sqrt(_y*_y+_z*_z);

    float R_ridge = PF_OBSTACLE_RADIUS;

    float u_r = (1 - ((R_ridge*R_ridge)/(r*r))) * ref_wind_speed * cos(theta);
    float u_th = -(1 + ((R_ridge*R_ridge)/(r*r))) * ref_wind_speed * sin(theta);

//    cout << theta << ", " << u_r << ", " << u_th << endl;

    float z_circle = -0.01;
    if (R_ridge*R_ridge - _y*_y > 0){       // what is this for?
        z_circle = -1*sqrt(R_ridge*R_ridge - _y*_y);
    }

//    cout << "; " << (_z - z_circle) << "< " << PF_SURFACE_ROUGHNESS << "< " << endl;
    // 2d potential flow, so there is no x component (x:east)
    float mult_factor = (logf(-(_z - z_circle) / PF_SURFACE_ROUGHNESS)) / (logf(-(-70 - z_circle) / PF_SURFACE_ROUGHNESS));

//    cout << z_circle << ", " << mult_factor << endl;
    mult_factor = 1.0;  //TODO: temp

    struct FloatVect3 _wind_vel;
    _wind_vel.x = 0;
    _wind_vel.y = (cosf(theta) * u_r) - (sinf(theta) * u_th*mult_factor);
    _wind_vel.z = (sinf(theta) * u_r) + (cosf(theta) * u_th)*mult_factor;
    // TODO: check mult_factor. what is it for?

    return _wind_vel;
}

// init
void init_potential_flow_simulator(void)
{
    // your init code here

    // init ground utm & heading
    // init ref wind velocity

//    if (PF_USE_GROUND_GPS == 0) {
    ground_utm.east = 0;
    ground_utm.north = 0;
    ground_utm.alt = 0;
    ground_heading = 0;

//    cout << "init PF" << endl;

//    } /// move this to init. if it is not using ground gps, then do not update

}

void potential_flow_simulator_periodic(void)
{
    // Should I use mutex?
//    cout << "periodic PF" << endl;

    /// check if I use ground gps and already have the ground position
    // if(bit_is_set(ground_utm, something)) {}

    /// retrieve vehicle position & convert it to utm_ds
    // why lla->utm??: for accuracy. but IDK whether I really need it..
    struct LlaCoor_f *vehicle_position_lla_f = stateGetPositionLla_f();
//    cout << vehicle_position_lla_f->lon << ", " << vehicle_position_lla_f->lat << endl;


    struct LlaCoor_d vehicle_position_lla_d = to_lla_d(vehicle_position_lla_f);
//    cout << "lla PF" << endl;
//    cout << vehicle_position_lla_d.lon << ", " << vehicle_position_lla_d.lat << ", " << vehicle_position_lla_d.alt << endl;

    struct UtmCoor_d vehicle_position_utm;
//    cout << "utm PF1" << endl;

    utm_of_lla_d(&vehicle_position_utm, &vehicle_position_lla_d);
//    cout << "utm PF2" << endl;

    struct FloatVect3 vehicle_position_v3 = utm_to_v3(&vehicle_position_utm);

//    cout << "periodic PF 2" << endl;

    // heading
    float vehicle_heading = stateGetNedToBodyEulers_f()->psi*180/M_PI;
//  if (PF_USE_GROUND_GPS) { // no need to calc again..
//      ground_heading = get_ground_heading();
//  }
//    cout << "heading: " << vehicle_heading << endl;

    /// then I have ground station position and vehicle position both in UTM
    /// AND IN OBSTACLE's body frame!!! !!! !!!
    struct FloatVect3 vehicle_position_in_ground_body_frame =
            rotate_frame(&vehicle_position_v3, ground_heading-vehicle_heading);

//    cout << vehicle_position_in_ground_body_frame.x << ", " << vehicle_position_in_ground_body_frame.y << ", "
//    << vehicle_position_in_ground_body_frame.z << endl;

    struct FloatVect3 ground_position_v3_enu = utm_to_v3(&ground_utm);

//    cout << ground_position_v3_enu.x << ", " << ground_position_v3_enu.y << ", " << ground_position_v3_enu.z << endl;

    struct FloatVect3 ground_position_v3 = rotate_frame(&ground_position_v3_enu, ground_heading);

//    cout << ground_position_v3.x << ", " << ground_position_v3.y << ", " << ground_position_v3.z << endl;

    /// calculate relative distance.. we assume that ground gps position is the obstacle position.
    // TODO: if not using ground gps, this does not make much sense.. bc ground position is in local, not utm.
    // TODO: so, set gp in utm, or add some conditions..
    struct FloatVect3 dist_from_obstacle_to_vehicle =
            calc_relative_position_v3(&vehicle_position_in_ground_body_frame, &ground_position_v3);
//    cout << "periodic PF 3" << endl;

//    cout << dist_from_obstacle_to_vehicle.x << ", " << dist_from_obstacle_to_vehicle.y << ", " << dist_from_obstacle_to_vehicle.z << endl;


    /// WIND. IN. NED.
//    float state_wind_x = stateGetHorizontalWindspeed_f()->x; // north
//    float state_wind_y = stateGetHorizontalWindspeed_f()->y; // east
//struct FloatVect3 windspeed_v3f = stateGetWindspeed_f();

    /// should be a user input or a measurement
//    float ref_wind_vel = PF_REF_WIND_VEL;

    // x, y ; z does not need to be converted to the body frame bc it is alt

    /// potential flow calculation: obstacle's body frame; wind velocity (x, z) and reference wind vel (x dir)
    struct FloatVect3 wind_vel_v3f = compute_potential_flow(dist_from_obstacle_to_vehicle, PF_REF_WIND_VEL);

//    cout << wind_vel_v3f.x << ", " << wind_vel_v3f.y << ", " << wind_vel_v3f.z << endl;

    struct FloatVect3 wind_in_ned = rotate_frame(&wind_vel_v3f, -ground_heading);
//    cout << "periodic PF 4" << endl;

    struct FloatVect2 hor_wind;
    hor_wind.x = wind_in_ned.x;
    hor_wind.y = wind_in_ned.y;

    float ver_wind = wind_in_ned.z;

//    cout << hor_wind.x << ", " << hor_wind.y << ", " << ver_wind << endl;

    // Set wind speed (state)
    stateSetHorizontalWindspeed_f(&hor_wind);
    stateSetVerticalWindspeed_f(ver_wind);

    // set wind speed as environment
    if (PF_SET_WIND_NPS_ENV){
        nps_atmosphere_set_wind_ned((double)wind_in_ned.x, (double)wind_in_ned.y, (double)wind_in_ned.z);
    }
}


/*
 *
 * struct FloatVect3 compute_wind_field(void){
    // Initial Parameters for wind field computation
	float R_ridge = 10.0; // Height of the hill in m
	float U_inf = 15.0; // Wind velocity at infinity in m/s
	float a = 10; // Defines loci x-position
	float x_stag = 14; // m, defines a-axis of standard oval
	float b = sqrt(x_stag*x_stag - a*a);

	struct FloatVect3 dist_from_boat = compute_state();
	struct FloatVect3 wind_vector;
	wind_vector.x = 0;

	// Start of computation
    float m = M_PI*U_inf/a*(x_stag*x_stag - a*a);
    dist_y_plus_a = dist_from_boat.y + a;
    dist_z_plus_a = dist_from_boat.z + a;
    dist_y_minus_a = dist_from_boat.y - a;
    dist_z_minus_a = dist_from_boat.z - a;

    wind_vector.y = U_inf + m / (2 * M_PI) * (dist_y_plus_a / (dist_y_plus_a * dist_y_plus_a + dist_from_boat.z * dist_from_boat.z) - (dist_y_minus_a) / (dist_y_minus_a*dist_y_minus_a + dist_from_boat.z*dist_from_boat.z));
    wind_vector.z = -(m * dist_from_boat.z) / (2 * M_PI) * (1 / (dist_y_plus_a*dist_y_plus_a + dist_from_boat.z*dist_from_boat.z) - 1 / (dist_y_minus_a*dist_y_minus_a + dist_from_boat.z*dist_from_boat.z));


    wind_vector.y = U_inf + m / (2 * M_PI) * ((dist_from_boat.y + a) / ((dist_from_boat.y+0.001 + a) * (dist_from_boat.y+0.001 + a) + dist_from_boat.z * dist_from_boat.z) - (dist_from_boat.y+0.001 - a) / ((dist_from_boat.y+0.001 - a)*(dist_from_boat.y+0.001 - a) + dist_from_boat.z*dist_from_boat.z));
    wind_vector.z = -(m * dist_from_boat.z) / (2 * M_PI) * (1 / ((dist_from_boat.y+0.001 + a)*(dist_from_boat.y+0.001 + a) + dist_from_boat.z*dist_from_boat.z) - 1 / ((dist_from_boat.y+0.001 - a)*(dist_from_boat.y+0.001 - a) + dist_from_boat.z*dist_from_boat.z));

	float ellipse_eq = (dist_from_boat.y * dist_from_boat.y) / (x_stag * x_stag) + (dist_from_boat.z * dist_from_boat.z) / (b * b);

	if (ellipse_eq < 1){
	    wind_vector.x = 0;
	    wind_vector.y = 0;
	    wind_vector.z = 0;
	    return wind_vector;
	}

    if (dist_from_boat.y < x_stag){
	    float z_ellipse = -b / x_stag * sqrt(x_stag * x_stag- dist_from_boat.y * dist_from_boat.y);

	    // v_x *= ((z_i - z_ellipse)/-20)**(1/7)
	    // v_z *= ((z_i - z_ellipse) / -20) ** (1 / 7)

	    float mult_factor = (log(-(dist_from_boat.z - z_ellipse))/0.05)/(log(-(-20 - z_ellipse))/0.05);
		if (mult_factor){
            wind_vector.y *= mult_factor;
	        wind_vector.z *= mult_factor;
		} else {
	        if (dist_from_boat.z < 0){
	            z_ellipse = -0.01;
	            mult_factor = (log(-(dist_from_boat.z - z_ellipse)) / 0.05) / (log(-(-20 - z_ellipse)) / 0.05);
	            wind_vector.y *= mult_factor;
	            wind_vector.z *= mult_factor;
	        } else {
	            wind_vector.x = 0;
	            wind_vector.y = 0;
	            wind_vector.z = 0;
	        }
		}
    }
    return wind_vector;
}
 *
 */
