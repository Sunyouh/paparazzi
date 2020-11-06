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

/** @file "modules/gazebo_camera/gazebo_camera.cpp"
 * @author Sunyou Hwang <S.Hwang-1@tudelft.nl>
 * Camera module for gazebo simulation
 * Set vehicle pose to the gazebo model and retrieve an image from camera
 */


#include <cstdio>
#include <cstdlib>
#include <string>
#include <iostream>
#include <sys/time.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <sdf/sdf.hh>

#include <gazebo/gazebo_config.h>

extern "C" {
#include "state.h"
#include "modules/gazebo_camera/gazebo_camera.h"
#include "nps_fdm.h"
}


// Copied from nps_fdm_gazebo.cpp & modified
// Common functions to use Gazebo

using namespace std;

#ifndef NPS_GAZEBO_WORLD
#define NPS_GAZEBO_WORLD "empty.world"
#endif
#ifndef NPS_GAZEBO_AC_NAME
#define NPS_GAZEBO_AC_NAME "simple_fixedwing_w_camera"
#endif
#ifndef NPS_GAZEBO_SCALE
#define NPS_GAZEBO_SCALE 0.2
#endif

// Add video handling functions if req'd.
#if NPS_SIMULATE_VIDEO
extern "C" {
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/video_thread_nps.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "mcu_periph/sys_time.h"
//#include "boards/bebop/mt9f002.h"
//#include "boards/bebop/mt9v117.h"
}

static void init_gazebo_video(void);
static void gazebo_read_video(void);
static void read_image(
  struct image_t *img,
  gazebo::sensors::CameraSensorPtr cam);
struct gazebocam_t {
  gazebo::sensors::CameraSensorPtr cam;
  gazebo::common::Time last_measurement_time;
};
static struct gazebocam_t gazebo_cams[VIDEO_THREAD_MAX_CAMERAS] =
{ { NULL, 0 } };

#endif // NPS_SIMULATE_VIDEO


/// Holds all necessary NPS FDM state information
//struct NpsFdm fdm;

// Pointer to Gazebo data
static bool gazebo_initialized = false;
static gazebo::physics::ModelPtr model = NULL;

// Get contact sensor
static gazebo::sensors::ContactSensorPtr ct;

// Helper functions
static void init_gazebo(void);
static void gazebo_read(void);
static void gazebo_write(void);

// For heading fix
//static float heading_offset = 0.0;

// Conversion routines
inline struct EcefCoor_d to_pprz_ecef(ignition::math::Vector3d ecef_i)
{
    struct EcefCoor_d ecef_p;
    ecef_p.x = ecef_i.X();
    ecef_p.y = ecef_i.Y();
    ecef_p.z = ecef_i.Z();
    return ecef_p;
}

inline struct NedCoor_d to_pprz_ned(ignition::math::Vector3d global)
{
    struct NedCoor_d ned;
    ned.x = global.Y();
    ned.y = global.X();
    ned.z = -global.Z();
    return ned;
}

inline struct LlaCoor_d to_pprz_lla(ignition::math::Vector3d lla_i)
{
    struct LlaCoor_d lla_p;
    lla_p.lat = lla_i.X();
    lla_p.lon = lla_i.Y();
    lla_p.alt = lla_i.Z();
    return lla_p;
}

inline struct DoubleVect3 to_pprz_body(ignition::math::Vector3d body_g)
{
    struct DoubleVect3 body_p;
    body_p.x = body_g.X();
    body_p.y = -body_g.Y();
    body_p.z = -body_g.Z();
    return body_p;
}

inline struct DoubleRates to_pprz_rates(ignition::math::Vector3d body_g)
{
    struct DoubleRates body_p;
    body_p.p = body_g.X();
    body_p.q = -body_g.Y();
    body_p.r = -body_g.Z();
    return body_p;
}

inline struct DoubleEulers to_pprz_eulers(ignition::math::Quaterniond quat)
{
    struct DoubleEulers eulers;
    eulers.psi = -quat.Yaw();
    eulers.theta = -quat.Pitch();
    eulers.phi = quat.Roll();
    return eulers;
}

inline struct DoubleEulers to_global_pprz_eulers(ignition::math::Quaterniond quat)
{
    struct DoubleEulers eulers;
    eulers.psi = -quat.Yaw() - M_PI / 2;
    eulers.theta = -quat.Pitch();
    eulers.phi = quat.Roll();
    return eulers;
}

inline struct DoubleVect3 to_pprz_ltp(ignition::math::Vector3d xyz)
{
    struct DoubleVect3 ltp;
    ltp.x = xyz.Y();
    ltp.y = xyz.X();
    ltp.z = -xyz.Z();
    return ltp;
}

/// pprz struct to gzb vector3
inline ignition::math::Vector3d to_gazebo_3d(EcefCoor_d ecef_i)
{
    ignition::math::Vector3d pos_gzb;
    pos_gzb.X(ecef_i.x);
    pos_gzb.Y(ecef_i.y);
    pos_gzb.Z(ecef_i.z);
    return pos_gzb;
}

inline ignition::math::Vector3d to_gazebo_3d(NedCoor_d pos_pprz_d){
    ignition::math::Vector3d pos_gzb;
    pos_gzb.X(pos_pprz_d.x);
    pos_gzb.Y(pos_pprz_d.y);
    pos_gzb.Z(pos_pprz_d.z);
    return pos_gzb;
}

inline ignition::math::Vector3d to_gazebo_3d(EnuCoor_f pos_pprz_f){
    ignition::math::Vector3d pos_gzb;
    pos_gzb.X(pos_pprz_f.x);
    pos_gzb.Y(pos_pprz_f.y);
    pos_gzb.Z(pos_pprz_f.z);
    return pos_gzb;
}

inline ignition::math::Vector3d to_gazebo_3d(LlaCoor_d pos_pprz_d)
{
    ignition::math::Vector3d pos_gzb;
    pos_gzb.X(pos_pprz_d.lat);
    pos_gzb.Y(pos_pprz_d.lon);
    pos_gzb.Z(pos_pprz_d.alt);
    return pos_gzb;
}

inline ignition::math::Quaterniond to_gazebo_quat(DoubleEulers pprz_euler){
    ignition::math::Quaterniond quat_gzb;
//    quat_gzb.Euler(-pprz_euler.theta, pprz_euler.phi, -pprz_euler.psi);
    quat_gzb.Euler(pprz_euler.phi, pprz_euler.theta, pprz_euler.psi);

//    quat_gzb.Yaw(pprz_euler.psi);
//    quat_gzb.Pitch(pprz_euler.theta);
//    quat_gzb.Roll(pprz_euler.phi);
    return quat_gzb;
}

// External functions, interface with Paparazzi's NPS as declared in nps_fdm.h

/**
 * Update the simulation state.
 * @param launch
 * @param act_commands
 * @param commands_nb
 */
//void nps_fdm_run_step(
//        bool launch __attribute__((unused)),
//        double *act_commands,
//        int commands_nb)
//{
//    // Initialize Gazebo if req'd.
//    // Initialization is peformed here instead of in nps_fdm_init because:
//    // - Video devices need to added at this point. Video devices have not been
//    //   added yet when nps_fdm_init is called.
//    // - nps_fdm_init runs on a different thread then nps_fdm_run_step, which
//    //   causes problems with Gazebo.
//    if (!gazebo_initialized) {
//        init_gazebo();
//        gazebo_read();
//#if NPS_SIMULATE_VIDEO
//        init_gazebo_video();
//#endif
//        gazebo_initialized = true;
//    }
//
//    // Update the simulation for a single timestep.
//    gazebo::runWorld(model->GetWorld(), 1);
//    gazebo::sensors::run_once();
//    gazebo_write(act_commands, commands_nb);
//    gazebo_read();
//#if NPS_SIMULATE_VIDEO
//    gazebo_read_video();
//#endif
//
//}


// Internal functions
/**
 * Set up a Gazebo server.
 *
 * Starts a Gazebo server, adds conf/simulator/gazebo/models to its model path
 * and loads the world specified by NPS_GAZEBO_WORLD.
 *
 * This function also obtaines a pointer to the aircraft model, named
 * NPS_GAZEBO_AC_NAME ('paparazzi_uav' by default). This pointer, 'model',
 * is used to read the state and write actuator commands in gazebo_read and
 * _write.
 */
static void init_gazebo(void)
{
    string gazebo_home = "/conf/simulator/gazebo/";
    string pprz_home(getenv("PAPARAZZI_HOME"));
    string gazebodir = pprz_home + gazebo_home;
    cout << "Gazebo directory: " << gazebodir << endl;

    if (getenv("ROS_MASTER_URI")) {
        // Launch with ROS support
        cout << "Add ROS plugins... ";
        gazebo::addPlugin("libgazebo_ros_paths_plugin.so");
        gazebo::addPlugin("libgazebo_ros_api_plugin.so");
        cout << "ok" << endl;
    }

    if (!gazebo::setupServer(0, NULL)) {
        cout << "Failed to start Gazebo, exiting." << endl;
        std::exit(-1);
    }

    cout << "Add Paparazzi paths: " << gazebodir << endl;
    gazebo::common::SystemPaths::Instance()->AddModelPaths(gazebodir + "models/");
    sdf::addURIPath("model://", gazebodir + "models/");
    sdf::addURIPath("world://", gazebodir + "world/");

    cout << "Add TU Delft paths: " << pprz_home + "/sw/ext/tudelft_gazebo_models/" << endl;
    gazebo::common::SystemPaths::Instance()->AddModelPaths(pprz_home + "/sw/ext/tudelft_gazebo_models/models/");
    sdf::addURIPath("model://", pprz_home + "/sw/ext/tudelft_gazebo_models/models/");
    sdf::addURIPath("world://", pprz_home + "/sw/ext/tudelft_gazebo_models/world/");

    // get vehicles
    string vehicle_uri = "model://" + string(NPS_GAZEBO_AC_NAME) + "/" + string(NPS_GAZEBO_AC_NAME) + ".sdf";
    string vehicle_filename = sdf::findFile(vehicle_uri, false);
    if (vehicle_filename.empty()) {
        cout << "ERROR, could not find vehicle " + vehicle_uri << endl;
        std::exit(-1);
    }
    cout << "Load vehicle: " << vehicle_filename << endl;
    sdf::SDFPtr vehicle_sdf(new sdf::SDF());
    sdf::init(vehicle_sdf);
    if (!sdf::readFile(vehicle_filename, vehicle_sdf)) {
        cout << "ERROR, could not read vehicle " + vehicle_filename << endl;
        std::exit(-1);
    }

    // add or set up sensors before the vehicle gets loaded
#if NPS_SIMULATE_VIDEO
    // Cameras
  sdf::ElementPtr link = vehicle_sdf->Root()->GetFirstElement()->GetElement("link");
  while (link) {
      // TODO: Set image w/h, rate and intrinsic
//    if (link->Get<string>("name") == "front_camera" && link->GetElement("sensor")->Get<string>("name") == "mt9f002") {
//      if (NPS_MT9F002_SENSOR_RES_DIVIDER != 1) {
//        int w = link->GetElement("sensor")->GetElement("camera")->GetElement("image")->GetElement("width")->Get<int>();
//        int h = link->GetElement("sensor")->GetElement("camera")->GetElement("image")->GetElement("height")->Get<int>();
//        int env = link->GetElement("sensor")->GetElement("camera")->GetElement("lens")->GetElement("env_texture_size")->Get<int>();
//        link->GetElement("sensor")->GetElement("camera")->GetElement("image")->GetElement("width")->Set(w / NPS_MT9F002_SENSOR_RES_DIVIDER);
//        link->GetElement("sensor")->GetElement("camera")->GetElement("image")->GetElement("height")->Set(h / NPS_MT9F002_SENSOR_RES_DIVIDER);
//        link->GetElement("sensor")->GetElement("camera")->GetElement("lens")->GetElement("env_texture_size")->Set(env / NPS_MT9F002_SENSOR_RES_DIVIDER);
//      }
//      if (MT9F002_TARGET_FPS){
//        int fps = Min(MT9F002_TARGET_FPS, link->GetElement("sensor")->GetElement("update_rate")->Get<int>());
//        link->GetElement("sensor")->GetElement("update_rate")->Set(fps);
//      }
//    } else if  (link->Get<string>("name") == "bottom_camera" && link->GetElement("sensor")->Get<string>("name") == "mt9v117") {
//      if (MT9V117_TARGET_FPS){
//        int fps = Min(MT9V117_TARGET_FPS, link->GetElement("sensor")->GetElement("update_rate")->Get<int>());
//        link->GetElement("sensor")->GetElement("update_rate")->Set(fps);
//      }
//    }
    link = link->GetNextElement("link");
  }
#endif

    // get world
    string world_uri = "world://" + string(NPS_GAZEBO_WORLD);
    string world_filename = sdf::findFile(world_uri, false);
    if (world_filename.empty()) {
        cout << "ERROR, could not find world " + world_uri << endl;
        std::exit(-1);
    }
    cout << "Load world: " << world_filename << endl;
    sdf::SDFPtr world_sdf(new sdf::SDF());
    sdf::init(world_sdf);
    if (!sdf::readFile(world_filename, world_sdf)) {
        cout << "ERROR, could not read world " + world_filename << endl;
        std::exit(-1);
    }

    // add vehicles
    world_sdf->Root()->GetFirstElement()->InsertElement(vehicle_sdf->Root()->GetFirstElement());

    world_sdf->Write(pprz_home + "/var/gazebo.world");

    gazebo::physics::WorldPtr world = gazebo::loadWorld(pprz_home + "/var/gazebo.world");
    if (!world) {
        cout << "Failed to open world, exiting." << endl;
        std::exit(-1);
    }

    cout << "Get pointer to aircraft: " << NPS_GAZEBO_AC_NAME << endl;
    model = world->ModelByName(NPS_GAZEBO_AC_NAME);
    if (!model) {
        cout << "Failed to find '" << NPS_GAZEBO_AC_NAME << "', exiting."
             << endl;
        std::exit(-1);
    }

    cout << "Gazebo initialized successfully!" << endl;
}

/**
 * Read Gazebo's simulation state and store the results in the fdm struct used
 * by NPS.
 *
 * Not all fields are filled at the moment, as some of them are unused by
 * paparazzi (see comments) and others are not available in Gazebo 7
 * (atmosphere).
 */
static void gazebo_read(void)
{
    static ignition::math::Vector3d vel_prev;
    static double time_prev;

    gazebo::physics::WorldPtr world = model->GetWorld();
    ignition::math::Pose3d pose = model->WorldPose(); // In LOCAL xyz frame
    ignition::math::Vector3d vel = model->WorldLinearVel();
    ignition::math::Vector3d ang_vel = model->WorldAngularVel();
    gazebo::common::SphericalCoordinatesPtr sphere = world->SphericalCoords();
    ignition::math::Quaterniond local_to_global_quat(0, 0, -sphere->HeadingOffset().Radian());

    /* Fill FDM struct */
    fdm.time = world->SimTime().Double();

    // Find world acceleration by differentiating velocity
    // model->GetWorldLinearAccel() does not seem to take the velocity_decay into account!
    // Derivation of the velocity also follows the IMU implementation of Gazebo itself:
    // https://bitbucket.org/osrf/gazebo/src/e26144434b932b4b6a760ddaa19cfcf9f1734748/gazebo/sensors/ImuSensor.cc?at=default&fileviewer=file-view-default#ImuSensor.cc-370
    double dt = fdm.time - time_prev;
    ignition::math::Vector3d accel = (vel - vel_prev) / dt;
    vel_prev = vel;
    time_prev = fdm.time;
}

/**
 * Write position and attitude to Gazebo.
 * pose comes from fdm.JSBsim
 */
static void gazebo_write(void)
{
    // get model link & set pose
    gazebo::physics::WorldPtr world = model->GetWorld();
    gazebo::physics::LinkPtr link = model->GetLink("body");     // TODO: receive the link name from xml file
//    fdm.lla_pos = to_pprz_lla(sphere->PositionTransform(pose.Pos(), gazebo::common::SphericalCoordinates::LOCAL,
//                                                        gazebo::common::SphericalCoordinates::SPHERICAL));

    // lla to local
//    sphere->PositionTransform(fdm.lla_pos,gazebo::common::SphericalCoordinates::SPHERICAL,
//            gazebo::common::SphericalCoordinates::LOCAL);
//    (fdm.ltp_to_body_quat);

    ignition::math::Pose3d setPose;
//    gazebo::common::SphericalCoordinatesPtr sphere;

    gazebo::common::SphericalCoordinatesPtr sphere = world->SphericalCoords();
//    ignition::math::Quaterniond pos_rot_quat(0, 0, -M_PI/2);
//    ignition::math::Quaterniond att_rot_quat(0, -M_PI, -M_PI/2);
    ignition::math::Quaterniond rot_x(M_PI, 0, 0);
    ignition::math::Quaterniond rot_y(0, M_PI, 0);
    ignition::math::Quaterniond rot_z(0 , 0, M_PI);

//    struct EnuCoor_f pos_enu = *stateGetPositionEnu_f();

//    pprz_pos = stateGetPositionEnu_f();


    // TODO: check multi-rotor coordinates
    // TODO: Clean up & check naming

/// lla
//    ignition::math::Vector3d gzb_lla_pos = to_gazebo_3d(fdm.lla_pos);
//    ignition::math::Vector3d gzb_lla_to_local = sphere->PositionTransform(gzb_lla_pos,
//            gazebo::common::SphericalCoordinates::SPHERICAL, gazebo::common::SphericalCoordinates::LOCAL);
//    ignition::math::Vector3d rot_gzb_pos = gzb_lla_to_local;
//
//    cout << "" << endl;
//    cout << "fdm_lla: " << fdm.lla_pos.lon << ", " << fdm.lla_pos.lat << ", " << fdm.lla_pos.alt << endl;
//    cout << "gzb_lla: " << gzb_lla_to_local.X() << ", " << gzb_lla_to_local.Y() << ", " << gzb_lla_to_local.Z() << endl;
//    cout << "rot_lla: " << rot_gzb_pos.X() << ", " << rot_gzb_pos.Y() << ", " << rot_gzb_pos.Z() << endl;

/// ecef
//    ignition::math::Vector3d gzb_position = to_gazebo_3d(fdm.ecef_pos);
//    ignition::math::Vector3d ecef_pos = sphere->PositionTransform(gzb_position,
//            gazebo::common::SphericalCoordinates::ECEF, gazebo::common::SphericalCoordinates::LOCAL);
//    ignition::math::Vector3d rot_gzb_pos = rot_y.RotateVector(ecef_pos);
//    rot_gzb_pos.Z(-rot_gzb_pos.Z());
//
//    cout << "" << endl;
//    cout << "gzb_ecef: " << ecef_pos.X() << ", " << ecef_pos.Y() << ", " << ecef_pos.Z() << endl;
//    cout << "fdm_ecef: " << fdm.ecef_pos.x << ", " << fdm.ecef_pos.y << ", " << fdm.ecef_pos.z << endl;
//    cout << "rot_ecef: " << rot_gzb_pos.X() << ", " << rot_gzb_pos.Y() << ", " << rot_gzb_pos.Z() << endl;


/// ltp
    ignition::math::Vector3d gzb_ltp_pos = to_gazebo_3d(fdm.ltpprz_pos);
    ignition::math::Vector3d gzb_ltp_to_local = gzb_ltp_pos;
    ignition::math::Vector3d rot_gzb_pos = rot_x.RotateVector(gzb_ltp_to_local);

    // TODO: scale factor
    ignition::math::Vector3d z_fix_rot_gzb_pos(NPS_GAZEBO_SCALE*rot_gzb_pos.X(),
                                               NPS_GAZEBO_SCALE*rot_gzb_pos.Y(), NPS_GAZEBO_SCALE*rot_gzb_pos.Z());


    /// for multi-rotors, will check it later (coordinate is different)
//        ignition::math::Vector3d gzb_ltp_to_local = sphere->PositionTransform(gzb_ltp_pos,
//            gazebo::common::SphericalCoordinates::GLOBAL, gazebo::common::SphericalCoordinates::LOCAL);
//        ignition::math::Vector3d rot_gzb_pos = rot_z.RotateVector(gzb_ltp_to_local);
//    ignition::math::Vector3d z_fix_rot_gzb_pos(0.1*rot_gzb_pos.X(), 0.1*rot_gzb_pos.Y(), -0.1*rot_gzb_pos.Z());

//    cout << "" << endl;
//    cout << "fdm_ltp: " << fdm.ltpprz_pos.x << ", " << fdm.ltpprz_pos.y << ", " << fdm.ltpprz_pos.z << endl;
//    cout << "gzb_lcl: " << gzb_ltp_to_local.X() << ", " << gzb_ltp_to_local.Y() << ", " << gzb_ltp_to_local.Z() << endl;
//    cout << "rot_ltp: " << z_fix_rot_gzb_pos.X() << ", " << z_fix_rot_gzb_pos.Y() << ", " << z_fix_rot_gzb_pos.Z() << endl;


    /// attitude
    ignition::math::Pose3d pose = model->WorldPose(); // In LOCAL xyz frame
//    gazebo::common::SphericalCoordinatesPtr sphere = world->SphericalCoords();

    ignition::math::Quaterniond local_to_global_quat(0, 0, sphere->HeadingOffset().Radian());
    ignition::math::Quaterniond fdm_quat = to_gazebo_quat(fdm.ltp_to_body_eulers);
//    ignition::math::Quaterniond inv_quat = local_to_global_quat.Inverse();
//    ignition::math::Quaterniond gzb_quat = local_to_global_quat*fdm_quat;
//    ignition::math::Quaterniond gzb_quat(0, 0, 0);

//    ignition::math::Vector3d rot_gzb_pos = pos_rot_quat.RotateVector(gzb_position);
//    ignition::math::Vector3d rot_gzb_pos = rot_x.RotateVector(gzb_position);
//    ignition::math::Vector3d rot_gzb_pos = rot_z.RotateVector(gzb_position);
//    ignition::math::Vector3d rot_gzb_pos = gzb_position;

    ignition::math::Quaterniond pose_rot = pose.Rot();
    DoubleEulers gzb_to_pprz_euler = to_global_pprz_eulers(local_to_global_quat * pose.Rot());
    ignition::math::Quaterniond heading_fix_rot = local_to_global_quat.Inverse() * fdm_quat;

    ignition::math::Quaterniond gzb_quat = rot_z*rot_y*fdm_quat;
//    ignition::math::Quaterniond fix_inv_pitch(gzb_quat.Roll(), -gzb_quat.Pitch(), gzb_quat.Yaw());

//    cout << "" << endl;
//    cout << "pprz_ltp_b_euler: " << fdm.ltp_to_body_eulers.phi << ", " << fdm.ltp_to_body_eulers.theta << ", " << fdm.ltp_to_body_eulers.psi << endl;
//    cout << "gzb_raw_euler: " << pose_rot.Roll() << ", " << pose_rot.Pitch() << ", " << pose_rot.Yaw() << endl;
//    cout << "gzb_to_pprz_euler: " << gzb_to_pprz_euler.phi << ", " << gzb_to_pprz_euler.theta << ", " << gzb_to_pprz_euler.psi << endl;
//    cout << "heading_fix_euler: " << heading_fix_rot.Roll() << ", " << heading_fix_rot.Pitch() << ", " << heading_fix_rot.Yaw() << endl;
//    cout << "" << endl;

//    ignition::math::Pose3d pose = model->WorldPose(); // In LOCAL xyz frame
//    ignition::math::Vector3d pose_pos = pose.Pos();
//    ignition::math::Vector3d pose_pos(0, 0, 1);

//    LlaCoor_d gzb_lla_pos = to_pprz_lla(sphere->PositionTransform(pose_pos, gazebo::common::SphericalCoordinates::LOCAL,
//                                                        gazebo::common::SphericalCoordinates::SPHERICAL));
//    ignition::math::Vector3d gzb_lla_position = to_gazebo_3d(gzb_lla_pos);
//
//    cout << "" << endl;
//    cout << "fdm_lla: " << gzb_position.X() << ", " << gzb_position.Y() << ", " << gzb_position.Z() << endl;
//    cout << "local_lla: " << rot_gzb_pos.X() << ", " << rot_gzb_pos.Y() << ", " << rot_gzb_pos.Z() << endl;
//    cout << "" << endl;
//    cout << "gzb_xyz: " << pose_pos.X() << ", " << pose_pos.Y() << ", " << pose_pos.Z() << endl;
//    cout << "fdm_lla: " << fdm.lla_pos.lat << ", " << fdm.lla_pos.lon << ", " << fdm.lla_pos.alt << endl;
//    cout << "gzb_lla: " << gzb_lla_position.X() << ", " << gzb_lla_position.Y() << ", " << gzb_lla_position.Z() << endl;
//
//    EcefCoor_d  ecef_pos;
//    NedCoor_d ltpprz_pos;
//
//    EcefCoor_d ecef_gzb = to_pprz_ecef(sphere->PositionTransform(pose_pos, gazebo::common::SphericalCoordinates::LOCAL,
//                                                                 gazebo::common::SphericalCoordinates::ECEF));
//    ignition::math::Vector3d gzb_ecef_pos = to_gazebo_3d(ecef_gzb);

//
//    NedCoor_d ned_gzb = to_pprz_ned(sphere->PositionTransform(pose_pos, gazebo::common::SphericalCoordinates::LOCAL,
//                                                                 gazebo::common::SphericalCoordinates::GLOBAL));
//    ignition::math::Vector3d gzb_ned_pos = to_gazebo_3d(ned_gzb);
//    cout << "gzb_ecef: " << gzb_ned_pos.X() << ", " << gzb_ned_pos.Y() << ", " << gzb_ned_pos.Z() << endl;
//    cout << "fdm_ecef: " << fdm.ltpprz_pos.x << ", " << fdm.ltpprz_pos.y << ", " << fdm.ltpprz_pos.z << endl;

//    ignition::math::Quaterniond gzb_quat = to_gazebo_quat(local_to_global_quat.RotateVectorReverse(fdm.ltp_to_body_eulers));

//    cout << "fdm_lla: " << fdm.lla_pos.lat << ", " << fdm.lla_pos.lon << ", " << fdm.lla_pos.alt << endl;
//    cout << "gzb_pos: " << gzb_position.X() << ", " << gzb_position.Y() << ", " << gzb_position.Z() << endl;

//    ignition::math::Pose3d world_pose = link->WorldPose();
//    ignition::math::Vector3d w_position = world_pose.Pos();
//    cout << "wld_pos: " << w_position.X() << ", " << w_position.Y() << ", " << w_position.Z() << endl;

    setPose = ignition::math::Pose3d(z_fix_rot_gzb_pos, gzb_quat);
//    setPose = ignition::math::Pose3d(gzb_position, gzb_quat);

    model->SetRelativePose(setPose);        // pose 3d; x,y,z, qw,qx,qy,qz
}

#if NPS_SIMULATE_VIDEO
/**
 * Set up cameras.
 *
 * This function finds the video devices added through add_video_device
 * (sw/airborne/modules/computer_vision/cv.h). The camera links in the Gazebo AC
 * model should have the same name as the .dev_name field in the corresponding
 * video_config_t struct stored in 'cameras[]' (computer_vision/
 * video_thread_nps.h). Pointers to Gazebo's cameras are stored in gazebo_cams
 * at the same index as their 'cameras[]' counterpart.
 *
 * The video_config_t parameters are updated using the values provided by
 * Gazebo. This should simplify the use of different UAVs with different camera
 * setups.
 */
static void init_gazebo_video(void)
{
  gazebo::sensors::SensorManager *mgr = gazebo::sensors::SensorManager::Instance();

  cout << "Initializing cameras..." << endl;
  // Loop over cameras registered in video_thread_nps
  for (int i = 0; i < VIDEO_THREAD_MAX_CAMERAS && cameras[i] != NULL; ++i) {
    // Find link in gazebo model
    cout << "Setting up '" << cameras[i]->dev_name << "'... ";
    gazebo::physics::LinkPtr link = model->GetLink(cameras[i]->dev_name);
    if (!link) {
      cout << "ERROR: Link '" << cameras[i]->dev_name << "' not found!"
           << endl;
      ;
      continue;
    }
    // Get a pointer to the sensor using its full name
    if (link->GetSensorCount() != 1) {
      cout << "ERROR: Link '" << link->GetName()
           << "' should only contain 1 sensor!" << endl;
      continue;
    }
    string name = link->GetSensorName(0);
    gazebo::sensors::CameraSensorPtr cam = static_pointer_cast
                                           < gazebo::sensors::CameraSensor > (mgr->GetSensor(name));

//    gazebo::sensors::SensorPtr genericSensor = gazebo::sensors::get_sensor("front_camera");
//    if (!genericSensor) {
//        cout << "Could not get the sensor " << name << endl;
//        continue;
//    }
//    gazebo::sensors::CameraSensorPtr cam = std::static_pointer_cast<gazebo::sensors::CameraSensor>(genericSensor);

    if (!cam) {
      cout << "ERROR: Could not get pointer to '" << name << "'!" << endl;
      continue;
    }
    // Activate sensor
    cam->SetActive(true);

    // Add to list of cameras
    gazebo_cams[i].cam = cam;
    gazebo_cams[i].last_measurement_time = cam->LastMeasurementTime();

    // set default camera settings
    // Copy video_config settings from Gazebo's camera
    cameras[i]->output_size.w = cam->ImageWidth();
    cameras[i]->output_size.h = cam->ImageHeight();
    cameras[i]->sensor_size.w = cam->ImageWidth();
    cameras[i]->sensor_size.h = cam->ImageHeight();
    cameras[i]->crop.w = cam->ImageWidth();
    cameras[i]->crop.h = cam->ImageHeight();
    cameras[i]->fps = 0;
    cameras[i]->camera_intrinsics.focal_x = cameras[i]->output_size.w / 2.0f;
    cameras[i]->camera_intrinsics.center_x = cameras[i]->output_size.w / 2.0f;
    cameras[i]->camera_intrinsics.focal_y = cameras[i]->output_size.h / 2.0f;
    cameras[i]->camera_intrinsics.center_y = cameras[i]->output_size.h / 2.0f;

    cout << "ok" << endl;
  }
}

/**
 * Read camera images.
 *
 * Polls gazebo cameras. If the last measurement time has been updated, a new
 * frame is available. This frame is converted to Paparazzi's UYVY format
 * and passed to cv_run_device which runs the callbacks registered by various
 * modules.
 */
static void gazebo_read_video(void)
{
  for (int i = 0; i < VIDEO_THREAD_MAX_CAMERAS; ++i) {
    gazebo::sensors::CameraSensorPtr &cam = gazebo_cams[i].cam;
    // Skip unregistered or unfound cameras
    if (cam == NULL) { continue; }
    // Skip if not updated
    // Also skip when LastMeasurementTime() is zero (workaround)
    if ((cam->LastMeasurementTime() - gazebo_cams[i].last_measurement_time).Float() < 0.005
        || cam->LastMeasurementTime() == 0) { continue; }
    // Grab image, convert and send to video thread
    struct image_t img;
    read_image(&img, cam);

#if NPS_DEBUG_VIDEO
    cv::Mat RGB_cam(cam->ImageHeight(), cam->ImageWidth(), CV_8UC3, (uint8_t *)cam->ImageData());
    cv::cvtColor(RGB_cam, RGB_cam, cv::COLOR_RGB2BGR);
    cv::namedWindow(cameras[i]->dev_name, cv::WINDOW_AUTOSIZE);  // Create a window for display.
    cv::imshow(cameras[i]->dev_name, RGB_cam);
    cv::waitKey(1);
#endif

    cv_run_device(cameras[i], &img);
    // Free image buffer after use.
    image_free(&img);
    // Keep track of last update time.
    gazebo_cams[i].last_measurement_time = cam->LastMeasurementTime();
  }
}

/**
 * Read Gazebo image and convert.
 *
 * Converts the current camera frame to the format used by Paparazzi. This
 * includes conversion to UYVY. Gazebo's simulation time is used for the image
 * timestamp.
 *
 * @param img
 * @param cam
 */
static void read_image(struct image_t *img, gazebo::sensors::CameraSensorPtr cam)
{
    image_create(img, cam->ImageWidth(), cam->ImageHeight(), IMAGE_YUV422);

  // Convert Gazebo's *RGB888* image to Paparazzi's YUV422
  const uint8_t *data_rgb = cam->ImageData();
  uint8_t *data_yuv = (uint8_t *)(img->buf);
  for (int x_yuv = 0; x_yuv < img->w; ++x_yuv) {
    for (int y_yuv = 0; y_yuv < img->h; ++y_yuv) {
      int x_rgb = x_yuv;
      int y_rgb = y_yuv;

      int idx_rgb = 3 * (cam->ImageWidth() * y_rgb + x_rgb);
      int idx_yuv = 2 * (img->w * y_yuv + x_yuv);
      int idx_px = img->w * y_yuv + x_yuv;
      if (idx_px % 2 == 0) { // Pick U or V
        data_yuv[idx_yuv] = - 0.148 * data_rgb[idx_rgb]
                            - 0.291 * data_rgb[idx_rgb + 1]
                            + 0.439 * data_rgb[idx_rgb + 2] + 128; // U
      } else {
        data_yuv[idx_yuv] =   0.439 * data_rgb[idx_rgb]
                              - 0.368 * data_rgb[idx_rgb + 1]
                              - 0.071 * data_rgb[idx_rgb + 2] + 128; // V
      }
      data_yuv[idx_yuv + 1] =   0.257 * data_rgb[idx_rgb]
                                + 0.504 * data_rgb[idx_rgb + 1]
                                + 0.098 * data_rgb[idx_rgb + 2] + 16; // Y
    }
  }
  // Fill miscellaneous fields
  gazebo::common::Time ts = cam->LastMeasurementTime();
  img->ts.tv_sec = ts.sec;
  img->ts.tv_usec = ts.nsec / 1000.0;
  img->pprz_ts = ts.Double() * 1e6;
  img->buf_idx = 0; // unused
}
#endif

#pragma GCC diagnostic pop // Ignore -Wdeprecated-declarations



    /**
     * Main functions called from xml
     */
void gazebo_camera_init(void)
{
    // Initialize gazebo server & video, set gazebo model pointer
    init_gazebo();

    // TODO: Check if this work as expected (was in fdm_run loop originally)
#if NPS_SIMULATE_VIDEO
    init_gazebo_video();
#endif
    gazebo_initialized = true;
}

void gazebo_camera_periodic(void)
{
  // freq = 30.0 Hz can be changed later

  if (!gazebo_initialized) {
        init_gazebo();
        gazebo_read();
#if NPS_SIMULATE_VIDEO
        init_gazebo_video();
#endif
        gazebo_initialized = true;
    }
//    cout << "Periodic..." << endl;

//    gazebo::physics::WorldPtr world = model->GetWorld();
//    gazebo::runWorld(world, 5);
//    cout << "run world.." << endl;

    // TODO: check the 'timestep'
    // Update the simulation for a single timestep.
    gazebo::runWorld(model->GetWorld(), 1); // Single timestep?? Should I do something for sync?
//    gazebo::sensors::run_once(); // Is this necessary?

    // Write position and attitude of the vehicle on gazebo
    gazebo_write();

    // We don't need to read any of sensor measurement
    //    gazebo_read();
#if NPS_SIMULATE_VIDEO
    gazebo_read_video();
#endif

}


