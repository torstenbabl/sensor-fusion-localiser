/// @brief VIO + GPS
/// @author Torsten Babl

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <ostream>
#include <utility>
	

#include "main.h"
#include "geodetic_conv.hpp"

#include "gflags/gflags.h"

#include "sfl/types/GpsReading.h"
#include "sfl/types/RosTime.h"
#include "sfl/types/MetricUnits.h"
#include "sfl/utilities/readRosImuFromCsv.h"
#include "sfl/utilities/readRosTimedGpsFromCsv.h"
#include "sfl/utilities/readVoTransformsFromCsv.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Quaternion.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/inference/Symbol.h>

using sfl::utilities::readRosImuFromCsv;
using sfl::utilities::readVoTransformFromCsv;
using sfl::utilities::RosTimedImu;
using sfl::utilities::RosTimedT3D;
using sfl::types::RosTime;
using sfl::types::GpsReading;
using sfl::types::Meters;
using std::filesystem::path;
using sfl::utilities::RosTimedGps;
using sfl::utilities::readRosTimedGpsFromCsv;
using namespace gtsam;

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

char const* IMU_FILENAME = "imu.csv";
char const* VO_FILENAME = "vo.csv";
char const* GPS_FILENAME = "gps.csv";
uint const IMU_PER_VO = 20U;
double const IMU_PERIOD_SEC = 1.0 / 200.0;
Rot3 const VO_R_ROBOT = Rot3::Quaternion(-0.5, -0.5, 0.5, 0.5);
Pose3 const VO_T_ROBOT(VO_R_ROBOT, Point3());

Eigen::Matrix3d NED_R_ENU = (Eigen::Matrix3d() << 0, 1, 0,
                                                  1, 0, 0,
                                                  0, 0, -1).finished();

boost::shared_ptr<PreintegratedCombinedMeasurements::Params> imuParams() {
  // We use the sensor specs to build the noise model for the IMU factor.
  /// @todo Populate microstrain intrinsics
  double accel_noise_sigma = 0.0002276846771973415 * sqrt(200);
  double gyro_noise_sigma = 0.00010426654551109309 * sqrt(200);
  double accel_bias_rw_sigma = 6.706087584689249e-06 ;
  double gyro_bias_rw_sigma = 3.5636559575381104e-06 ;
  Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma, 2);
  Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma, 2);
  Matrix33 integration_error_cov =
      I_3x3 * 1e-8;  // error committed in integrating position from velocities
  Matrix33 bias_acc_cov = I_3x3 * pow(accel_bias_rw_sigma, 2);
  Matrix33 bias_omega_cov = I_3x3 * pow(gyro_bias_rw_sigma, 2);
  Matrix66 bias_acc_omega_init =
      I_6x6 * 1e-5;  // error in the bias used for preintegration

  auto p = PreintegratedCombinedMeasurements::Params::MakeSharedU(9.81);
  // PreintegrationBase params:
  p->accelerometerCovariance =
      measured_acc_cov;  // acc white noise in continuous
  p->integrationCovariance =
      integration_error_cov;  // integration uncertainty continuous
  // should be using 2nd order integration
  // PreintegratedRotation params:
  p->gyroscopeCovariance =
      measured_omega_cov;  // gyro white noise in continuous
  // PreintegrationCombinedMeasurements params:
  p->biasAccCovariance = bias_acc_cov;      // acc bias in continuous
  p->biasOmegaCovariance = bias_omega_cov;  // gyro bias in continuous
  p->biasAccOmegaInt = bias_acc_omega_init;

  return p;
}

// Define command-line flags
DEFINE_string(datadir, "/home/torsten/Documents/masters/citrus_data_set/04_13D_Jackal/", "Directory containing matching vo.csv, imu.csv files");
DEFINE_bool(use_isam2, false, "Use iSAM2 as the optimiser, otherwise LevenbergMarquardtOptimizer is used.");

Pose3 csvVoToPose3(RosTimedT3D const* vo);

/// @brief Calculate the heading of the first movement of the gpsIterator
/// @details Advances gpsIterator to the point associated with the returned rotation
/// @return Rot3 of the rotation within ENU
std::pair<Rot3, std::vector<RosTimedGps>::iterator> initialRotationInEnu(
  std::vector<RosTimedGps>& gpsVector ///< [in,out] vector of GpsReadings from which to calulate the inital heading.
);

int main(int argc, char* argv[])
{
  // Parse command-line flags
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if (!checkFilesExist(FLAGS_datadir))
  { std::cerr << "datadir does not exist" << std::endl;
    exit(1);
  }

  path const voPath = path(FLAGS_datadir) / VO_FILENAME;
  path const imuPath = path(FLAGS_datadir) / IMU_FILENAME;
  path const gpsPath = path(FLAGS_datadir) / GPS_FILENAME;

  std::vector<RosTimedT3D> rawVoTransforms = readVoTransformFromCsv(voPath);
  std::vector<RosTimedImu> rawImuSamples = readRosImuFromCsv(imuPath);
  std::vector<RosTimedGps> rawGpsReadings = readRosTimedGpsFromCsv(gpsPath);

  std::vector<RosTimedT3D>::iterator voIterator = rawVoTransforms.begin();
  std::vector<RosTimedImu>::iterator imuIterator = rawImuSamples.begin(); 
  //std::vector<RosTimedGps>::iterator gpsIterator = rawGpsReadings.begin(); 

  // Global coordinate system is xyz: East, North, Up. Origin point will be first point in 
  Rot3 initialGpsRotEnu;
  std::vector<RosTimedGps>::iterator gpsIterator;

  std::tie(initialGpsRotEnu, gpsIterator) = initialRotationInEnu(rawGpsReadings);

  std::cerr << "Starting heading (z rot in rad): " << initialGpsRotEnu.xyz().z() << std::endl;
  std::cerr << "Starting rotation: " << initialGpsRotEnu << std::endl;

  // Align IMU to VO iterator in time
  while (gpsIterator->time > voIterator->time)
  {
    voIterator++;
  }

  // Point zero timestamp
  std::vector<RosTime> keyFrameTimes;
  keyFrameTimes.push_back(voIterator->time);

  // Set ENU (0, 0) coordinate
  GeodeticConverter gpsCoordConverter(gpsIterator->data);
  std::cerr << "Set GPS origin at lat, lon: " << gpsIterator->data.latitude << " " << gpsIterator->data.longitude << std::endl;

  // iSAM2
  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  std::shared_ptr<ISAM2> isam2 = std::make_shared<ISAM2>(parameters);

  // Assemble initial quaternion through GTSAM constructor
  // ::quaternion(w,x,y,z);
  Rot3 prior_rotation = initialGpsRotEnu; // Critical for GPS factors
  Point3 prior_point(0, 0, 0);
  Pose3 prior_pose(prior_rotation, prior_point);
  Vector3 prior_velocity(0, 0, 0); // assume starting from rest
  imuBias::ConstantBias prior_imu_bias;  // assume zero initial bias

  Values initial_values;
  int correction_count = 0;
  initial_values.insert(X(correction_count), prior_pose);
  initial_values.insert(V(correction_count), prior_velocity);
  initial_values.insert(B(correction_count), prior_imu_bias);

  // Assemble prior noise model and add it the graph.`
  auto pose_noise_model = noiseModel::Diagonal::Sigmas(
      (Vector(6) << 0.01, 0.01, 0.01, 0.05, 0.05, 0.05)
          .finished());  // rad,rad,rad,m, m, m
  auto velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);  // m/s
  auto bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);

  NonlinearFactorGraph graph;
  graph.addPrior(X(correction_count), prior_pose, pose_noise_model);
  graph.addPrior(V(correction_count), prior_velocity, velocity_noise_model);
  graph.addPrior(B(correction_count), prior_imu_bias, bias_noise_model);

  auto params = imuParams();

  std::shared_ptr<PreintegrationType> preintegrated =
    std::make_shared<PreintegratedCombinedMeasurements>(params, prior_imu_bias);

  assert(preintegrated);

  // Store previous state for imu integration and latest predicted outcome.
  NavState prev_state(prior_pose, prior_velocity);
  NavState prop_state = prev_state;
  imuBias::ConstantBias prev_bias = prior_imu_bias;

  // VO kf factor noise
  double const VO_NOISE_COV = 0.05;
  auto vo_noise = noiseModel::Diagonal::Sigmas(
    (Vector6() << VO_NOISE_COV, VO_NOISE_COV, VO_NOISE_COV, VO_NOISE_COV, VO_NOISE_COV, VO_NOISE_COV).finished());

  // Constant diagonal covariance given by publichers of citrus dataset
  auto gpsNoise = noiseModel::Diagonal::Sigmas(
    (Vector3() << 0.0049, 0.0049, 0.01).finished());

  std::cout << "correction_count,key_index,t_x,t_y,t_z,v_x,v_y,v_z,r_x,r_y,r_z,r_w,sec,nanosec" << std::endl;

  // Align IMU to VO iterator in time
  while (voIterator->time > imuIterator->time)
  {
    imuIterator++;
  }

  Pose3 voSinceLastKeyFrame; // Cumulative vo transform between optimisation points
  // Iterate over data until exhausted
  for (uint voCounter;;voCounter++)
  {
    
    RosTimedT3D* currentVo = voIterator++.base();
    Pose3 robotFrameVo = csvVoToPose3(currentVo);
    //std::cout << voCounter << ":" << robotFrameVo.x() << ","  << robotFrameVo.y() << "," << robotFrameVo.z() << std::endl;
    voSinceLastKeyFrame = voSinceLastKeyFrame * robotFrameVo;

    //std::cerr << "VO - IMU time: " << static_cast<int>(currentVo->time.seconds) - static_cast<int>(imuIterator->time.seconds) << " nanosec: " << static_cast<int>(currentVo->time.nanoseconds) - static_cast<int>(imuIterator->time.nanoseconds) << std::endl;
    //std::cerr << "Accumulated VO: " << voSinceLastKeyFrame << std::endl;
    
    
    // Add all IMU readings up to last VO time to preintegration
    // Too much jitter to find all samples between times. Pop a fixed amount of imu samples. 20 per VO transform

    for (uint i = 0; i < IMU_PER_VO; i++)
    {
      auto rawLa = imuIterator->data.acceleration;
      auto rawAv = imuIterator->data.angularVelocity;
      Vector3 la(-rawLa[0], -rawLa[1], rawLa[2]);
      Vector3 av(-rawAv[0], -rawAv[1], rawAv[2]);
      preintegrated->integrateMeasurement(la, av, IMU_PERIOD_SEC);
      imuIterator++;
    }

    // Key frame event
    if (voCounter % 25 == 0){
      correction_count++;
      std::cerr << "Correction count: " << correction_count << std::endl;
      keyFrameTimes.push_back(currentVo->time);

      // Decide when to add a GPS factor
      if (correction_count % 72 == 0)
      {
        // Get closest GPS factor to VO time
        while (currentVo->time > gpsIterator->time)
        {
          gpsIterator++;
          if (gpsIterator == rawGpsReadings.end())
          {
            std::cerr << "GPS vector exhausted" << std::endl;
            break;
          }
        }

        Vector3 gpsEnu = gpsCoordConverter.relativeENUto(gpsIterator->data);

        GPSFactor gnss_factor(X(correction_count), gpsEnu, gpsNoise);
        graph.add(gnss_factor);

        std::cerr << "Adding GPS factor at lat, lon: " << gpsIterator->data.latitude << " " << gpsIterator->data.longitude << std::endl;
        std::cerr << "Adding GPS factor at ENU: " << gpsEnu.transpose() << std::endl;
      }

      // Add the VO factor
      BetweenFactor<Pose3> vo_factor(X(correction_count - 1), X(correction_count), voSinceLastKeyFrame,vo_noise);
      graph.add(vo_factor);

      // Add IMU factor
      auto preint_imu = dynamic_cast<const PreintegratedCombinedMeasurements&>(*preintegrated);
      CombinedImuFactor imu_factor(X(correction_count - 1), V(correction_count - 1),
                          X(correction_count), V(correction_count),
                          B(correction_count - 1), B(correction_count), 
                          preint_imu);
      graph.add(imu_factor);

      //imu_factor.print();

      // Propagate nav state
      prop_state = preintegrated->predict(prev_state, prev_bias);
      initial_values.insert(X(correction_count), prop_state.pose());
      initial_values.insert(V(correction_count), prop_state.v());
      initial_values.insert(B(correction_count), prev_bias);

      // Optimise and print factor graph to stdout
      auto start_wall_time = std::chrono::high_resolution_clock::now();
      Values result;
      if (FLAGS_use_isam2)
      {
        // iSAM2
        isam2->update(graph, initial_values);
        result = isam2->calculateEstimate();
        // Reset graph and initial estimate for next iteration
        graph.resize(0);
        initial_values.clear();
      }
      else
      {
        // LevenbergMarquardtOptimizer
        LevenbergMarquardtParams params;
        //params.setVerbosityLM("SUMMARY");
        LevenbergMarquardtOptimizer optimizer(graph, initial_values, params);
        result = optimizer.optimize();
      }
      auto end_wall_time = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> duration = end_wall_time - start_wall_time;
      std::cerr << "Optimisation time (ms): " << duration << std::endl;

      prev_state = NavState(result.at<Pose3>(X(correction_count)),result.at<Vector3>(V(correction_count)));
      prev_bias = result.at<imuBias::ConstantBias>(B(correction_count));

      // Reset all necessary
      voSinceLastKeyFrame = Pose3();
      preintegrated->resetIntegrationAndSetBias(prev_bias);

      // Print graph to stdout
      for (int i = 0; i <= correction_count; i++)
      {
        std::cout << correction_count << ",";
        std::cout << i << ",";
        Pose3 pose = result.at(X(i)).cast<Pose3>();
        Vector3 vel = result.at<Vector3>(V(i));
        std::cout << pose.x() << "," << pose.y() << "," << pose.z() << "," << vel.x() << "," << vel.y() << "," << vel.z() << ",";
        Quaternion rot_q = pose.rotation().toQuaternion();
        std::cout << rot_q.x() << "," << rot_q.y() << "," << rot_q.z() << "," << rot_q.w() << ",";
        std::cout << keyFrameTimes.at(i).seconds << "," << keyFrameTimes.at(i).nanoseconds << std::endl;
      }
    }

    // Decide if keyframe condition is reached
    //  Add preintegrated IMU and VO factor to graph
    //  Optimise graph and output
    //  Do housekeeping and reset needed

    if (imuIterator == rawImuSamples.end() || voIterator == rawVoTransforms.end())
    {
      std::cerr << "Files exhausted" << std::endl;
      break;
    }
  }


}

bool checkFilesExist(path datadir)
{
  if (!std::filesystem::exists(datadir))
  {
    std::cerr << "datadir does not exist" << std::endl;
    return false;
  }

  path voPath = datadir / VO_FILENAME;
  if (!std::filesystem::exists(voPath) || !std::filesystem::is_regular_file(voPath))
  {
    std::cerr << voPath << " does not exist" << std::endl;
    return false;
  }

  path imuPath = datadir / IMU_FILENAME;
  if (!std::filesystem::exists(imuPath) || !std::filesystem::is_regular_file(imuPath))
  {
    std::cerr << imuPath << " does not exist" << std::endl;
    return false;
  }

  return true;
}

Pose3 csvVoToPose3(RosTimedT3D const* vo)
{
  Rot3 rotation = Rot3::Quaternion(vo->data.rotation.w, vo->data.rotation.i, vo->data.rotation.j, vo->data.rotation.k);
  Point3 translation(vo->data.translation[0], vo->data.translation[1], vo->data.translation[2]);
  return  VO_T_ROBOT * Pose3(rotation, translation) * VO_T_ROBOT.inverse();
}

std::pair<Rot3, std::vector<RosTimedGps>::iterator> initialRotationInEnu(
  std::vector<RosTimedGps>& gpsVector
)
{
  uint const LOOK_AHEAD = 10;
  Meters const DISTANCE_THRESHHOLD = 0.5;
  // Look 10 readings ahead
  // Check translation in N, E
  // If robot has moved more than 0.5m, calculate rot3
  // set iterator to start point of translation

  std::vector<RosTimedGps>::iterator gpsIterator = gpsVector.begin();

  Rot3 returnRot;
  uint infoCounter = 0;

  for (;gpsIterator + LOOK_AHEAD != gpsVector.end();gpsIterator++)
  {
    infoCounter++;
    const GpsReading startPoint = gpsIterator->data;
    const GpsReading pointAhead = (gpsIterator + LOOK_AHEAD)->data;

    GeodeticConverter converter(startPoint);

    auto translation = converter.relativeNEDto(pointAhead);

    const Meters distance = hypot(translation.x(), translation.y());

    if (distance >= DISTANCE_THRESHHOLD)
    {
      // Calculate rotation and return
      translation.z() = 0;
      translation = translation.normalized();
      returnRot = Rot3::ClosestTo((Matrix(3, 3) << translation.y(),  -translation.x(), 0,
                                                           translation.x(), translation.y(), 0,
                                                           0,                0,               1).finished());

      break;
    }
  }
  std::cerr << "Initial rotation: Number of GPS readings advanced: " << infoCounter << std::endl;
  return {returnRot, gpsIterator};

}
