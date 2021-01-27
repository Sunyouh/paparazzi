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
 * The actual simulation is running on NPS FDM (e.g. JSBSIM)
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
#ifndef NPS_DEBUG_VIDEO
#define NPS_DEBUG_VIDEO 0
#endif

#if NPS_DEBUG_VIDEO
// Opencv tools
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
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

// Helper functions
static void init_gazebo(void);
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
inline ignition::math::Vector3d to_gazebo_position(EcefCoor_d ecef_i)
{
    ignition::math::Vector3d pos_gzb;
    pos_gzb.X(ecef_i.x);
    pos_gzb.Y(ecef_i.y);
    pos_gzb.Z(ecef_i.z);
    return pos_gzb;
}

inline ignition::math::Vector3d to_gazebo_position(NedCoor_f *pos_pprz_f){
    ignition::math::Vector3d pos_gzb;
    pos_gzb.X(pos_pprz_f->x);
    pos_gzb.Y(pos_pprz_f->y);
    pos_gzb.Z(pos_pprz_f->z);
    return pos_gzb;
}

inline ignition::math::Vector3d to_gazebo_position(EnuCoor_f pos_pprz_f){
    ignition::math::Vector3d pos_gzb;
    pos_gzb.X(pos_pprz_f.x);
    pos_gzb.Y(pos_pprz_f.y);
    pos_gzb.Z(pos_pprz_f.z);
    return pos_gzb;
}

inline ignition::math::Vector3d to_gazebo_position(LlaCoor_d pos_pprz_d)
{
    ignition::math::Vector3d pos_gzb;
    pos_gzb.X(pos_pprz_d.lat);
    pos_gzb.Y(pos_pprz_d.lon);
    pos_gzb.Z(pos_pprz_d.alt);
    return pos_gzb;
}

inline ignition::math::Quaterniond to_gazebo_quat(FloatEulers *pprz_euler){
    ignition::math::Quaterniond quat_gzb;
    quat_gzb.Euler(pprz_euler->phi, pprz_euler->theta, pprz_euler->psi);
    return quat_gzb;
}

// External functions, interface with Paparazzi's NPS as declared in nps_fdm.h



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
//    cout << "init GZB" << endl;

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
//    cout << "init GZB 2" << endl;

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
//    cout << "init GZB 3" << endl;

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
//    cout << "init GZB 4" << endl;

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
//    cout << "init GZB 5" << endl;

    // Initialize sensors
    gazebo::sensors::run_once(true);
    gazebo::sensors::run_threads();
    gazebo::runWorld(world, 1);
    cout << "Sensors initialized..." << endl;

    cout << "Gazebo initialized successfully!" << endl;
}

/**
 * Write position and attitude to Gazebo.
 * pose comes from fdm.JSBsim (could be any other fdm)
 */
static void gazebo_write(void)
{
    // get model link & set pose
    gazebo::physics::WorldPtr world = model->GetWorld();
    gazebo::physics::LinkPtr link = model->GetLink("body");     // TODO: receive the link name from xml file

    ignition::math::Quaterniond rot_x(M_PI, 0, 0);
    ignition::math::Quaterniond rot_y(0, M_PI, 0);
    ignition::math::Quaterniond rot_z(0 , 0, M_PI);


    // TODO: check multi-rotor coordinate

    struct NedCoor_f *_ltp_pos = stateGetPositionNed_f();
//    VECT3_COPY(_ltp_pos, fdm.ltpprz_pos);
    ignition::math::Vector3d pos_v3d = to_gazebo_position(_ltp_pos);
    ignition::math::Vector3d rot_gzb_pos = rot_x.RotateVector(pos_v3d);

    ignition::math::Vector3d scaled_rot_gzb_pos(NPS_GAZEBO_SCALE*rot_gzb_pos.X(),
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
//    ignition::math::Pose3d pose = model->WorldPose(); // In LOCAL xyz frame
//    gazebo::common::SphericalCoordinatesPtr sphere = world->SphericalCoords();

    struct FloatEulers *_ltp_eulers = stateGetNedToBodyEulers_f();
//    EULERS_COPY(_ltp_eulers, fdm.ltp_to_body_eulers);

    ignition::math::Quaterniond fdm_quat = to_gazebo_quat(_ltp_eulers);
    ignition::math::Quaterniond gzb_quat = rot_z*rot_y*fdm_quat;

    ignition::math::Pose3d setPose;
    setPose = ignition::math::Pose3d(scaled_rot_gzb_pos, gzb_quat);

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


    /**
     * Main functions called from xml
     */
void gazebo_camera_init(void)
{
    // Do nothing...
    // Gazebo initialization should be in the main thread, not here.
    // If it runs in a different thread, sensors::run_once will throw a silent exception :(
}

void gazebo_camera_periodic(void)
{
  // freq = 30.0 Hz can be changed later
//    cout << "periodic GZB" << endl;

  if (!gazebo_initialized) {
        init_gazebo();
#if NPS_SIMULATE_VIDEO
        init_gazebo_video();
#endif
        gazebo_initialized = true;
    }

//    cout << "periodic GZB 2" << endl;

    // Update the simulation for a single timestep.
    gazebo::runWorld(model->GetWorld(), 1); // 1 == iteration
//    cout << "runworld GZB" << endl;

    gazebo::sensors::run_once(); // Is this necessary? YES, indeed!
//    cout << "sensors GZB" << endl;

    // Write position and attitude of the vehicle on gazebo
    gazebo_write();
//    cout << "wirte GZB" << endl;


#if NPS_SIMULATE_VIDEO
    gazebo_read_video();
#endif

}


#pragma GCC diagnostic pop // Ignore -Wdeprecated-declarations
