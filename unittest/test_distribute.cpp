#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include "sot/core/parameter-server.hh"
#include "sot/talos_balance/distribute-wrench.hh"

using namespace dynamicgraph::sot;
using namespace dynamicgraph::sot::talos_balance;

int main (int , char** )
{
  // --- General ---
  std::cout << "--- General ---" << std::endl;

  const double dt = 0.001;
  const std::string robot_name = "robot";

  Eigen::VectorXd q(38);

  q <<  0.0,  0.0,  1.018213,  0.0,  0.0, 0.0, 1.0,                     //Free flyer
        0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.0,                  //Left Leg
        0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.0,                  //Right Leg
        0.0,  0.006761,                                                    //Chest
        0.25847 ,  0.173046, -0.0002, -0.525366, 0.0, -0.0,  0.1, -0.005,  //Left Arm
       -0.25847 , -0.173046, 0.0002  , -0.525366, 0.0,  0.0,  0.1,-0.005,  //Right Arm
        0.,  0.;                                                           //Head

  std::cout << "q: " << q.transpose() << std::endl;

  //from rospkg import RosPack
  //rospack = RosPack()
  //urdfPath = rospack.get_path('talos_data')+"/urdf/talos_reduced.urdf"
  //urdfDir = [rospack.get_path('talos_data')+"/../"]

  std::string urdfPath = "/opt/openrobots/share/talos_data/urdf/talos_reduced.urdf";

  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdfPath, pinocchio::JointModelFreeFlyer(), model);
  pinocchio::Data data(model);
  Eigen::Vector3d com = pinocchio::centerOfMass(model,data,q);
  pinocchio::updateFramePlacements(model,data);
  double m = data.mass[0];
  //com[1] = 0. # ensure perfect symmetry

  std::cout << "com: " << com.transpose() << std::endl;

  const std::string leftName = "leg_left_6_joint";
  const pinocchio::FrameIndex leftId = model.getFrameId(leftName);
  pinocchio::SE3 leftPos  = data.oMf[leftId];
  std::cout << leftName << ": " << leftId << std::endl;
  std::cout << leftPos << std::endl;

  const std::string rightName = "leg_right_6_joint";
  const pinocchio::FrameIndex rightId = model.getFrameId(rightName);
  pinocchio::SE3 rightPos = data.oMf[rightId];
  // Eigen::Vector3d pR = leftPos.translation // ensure perfect symmetry
  // pR[1] = -pR[1]
  // rightPos.translation = pR
  std::cout << rightName << ": " << rightId << std::endl;
  std::cout << rightPos << std::endl;

  double g = 9.81;
  double fz = m*g;
  Eigen::Vector3d force(0.0, 0.0, fz);
  double lx = com[0];
  double tauy = -fz*lx;
  Eigen::Vector3d moment(0.0, tauy,   0.0);
  Eigen::VectorXd wrench(6);
  wrench << force, moment;

  std::cout << "total wrench: " << wrench.transpose() << std::endl;

  // --- Parameter server ---
  std::cout << "--- Parameter server ---" << std::endl;

  ParameterServer param_server("param_server");
  
  param_server.init(dt, urdfPath, robot_name);

  // Set the map from joint name to joint ID
  // for key in conf.mapJointNameToID:
  //      param_server.setNameToId(key, conf.mapJointNameToID[key]);

//    # Set the map joint limits for each id
//    for key in conf.mapJointLimits:
//        param_server.setJointLimitsFromId(key, conf.mapJointLimits[key][0],
//                                          conf.mapJointLimits[key][1])

//    # Set the force limits for each id
//    for key in conf.mapForceIdToForceLimits:
//        param_server.setForceLimitsFromId(key, tuple(conf.mapForceIdToForceLimits[key][0]),
//                                          tuple(conf.mapForceIdToForceLimits[key][1]))

    // Set the force sensor id for each sensor name
//    for key in conf.mapNameToForceId:
//        param_server.setForceNameToForceId(key, conf.mapNameToForceId[key])

//    # Set the map from the urdf joint list to the sot joint list
//    param_server.setJointsUrdfToSot(conf.urdftosot)

//    # Set the foot frame name
//    for key in conf.footFrameNames:
//        param_server.setFootFrameName(key, conf.footFrameNames[key])

//    # Set IMU hosting joint name
//    param_server.setImuJointName(conf.ImuJointName)

  Eigen::VectorXd rightFootSensorXYZ(3);
  rightFootSensorXYZ << 0.0,0.0,-0.085;
  param_server.setRightFootForceSensorXYZ(rightFootSensorXYZ);

  Eigen::VectorXd rightFootSoleXYZ(3);
  rightFootSoleXYZ << 0.0,0.0,-0.105;

  param_server.setRightFootSoleXYZ(rightFootSoleXYZ);

  // --- DistributeWrench ---
  std::cout << "--- DistributeWrench ---" << std::endl;

  DistributeWrench distribute("distribute");

//    distribute.phase.value = 0
//    distribute.rho.value = 0.5

//    distribute.setMinPressure(conf.minPressure)
//    distribute.frictionCoefficient.value = conf.frictionCoefficient
//    distribute.wSum.value = conf.wSum
//    distribute.wNorm.value = conf.wNorm
//    distribute.wRatio.value = conf.wRatio
//    distribute.wAnkle.value = conf.wAnkle

//    distribute.set_right_foot_sizes(conf.RIGHT_FOOT_SIZES)
//    distribute.set_left_foot_sizes(conf.LEFT_FOOT_SIZES)










//distribute.q.value = halfSitting
//distribute.wrenchDes.value = wrench

//distribute.init(robot_name)

//# --- Wrench distribution ---
//print()
//print("--- Wrench distribution ---")
//distribute.phase.value = 0

//forceLeft  = [0.0, 0.0, fz/2]
//forceRight = [0.0, 0.0, fz/2]
//ly = float(leftPos.translation[1])
//taux = fz*ly/2
//wrenchLeft  = forceLeft  + [ taux, tauy/2, 0.0]
//wrenchRight = forceRight + [-taux, tauy/2, 0.0]

//lx = float(com[0]-leftPos.translation[0])
//tauy = -fz*lx/2
//ankleWrenchLeft  = forceLeft  + [0.0, tauy, 0.0]
//ankleWrenchRight = forceRight + [0.0, tauy, 0.0]

//print( "expected global wrench: %s" % str(wrench) )
//print( "expected global left wrench: %s"  % str(wrenchLeft) )
//print( "expected global right wrench: %s" % str(wrenchRight) )
//print( "expected ankle left wrench: %s"  % str(ankleWrenchLeft) )
//print( "expected ankle right wrench: %s" % str(ankleWrenchRight) )

//copLeft  = [float(com[0] - leftPos.translation[0]),  0., 0.]
//copRight = [float(com[0] - rightPos.translation[0]), 0., 0.]

//print( "expected sole left CoP: %s"  % str(copLeft) )
//print( "expected sole right CoP: %s" % str(copRight) )
//print()

//distribute.zmpRef.recompute(0)

//print( "resulting global wrench: %s" % str(distribute.wrenchRef.value) )
//assertApprox(wrench,distribute.wrenchRef.value,2)
//print( "resulting global left wrench: %s"  % str(distribute.wrenchLeft.value) )
//assertApprox(wrenchLeft,distribute.wrenchLeft.value,3)
//print( "resulting global right wrench: %s" % str(distribute.wrenchRight.value) )
//assertApprox(wrenchRight,distribute.wrenchRight.value,3)

//distribute.ankleWrenchLeft.recompute(0)
//distribute.ankleWrenchRight.recompute(0)

//print( "resulting ankle left wrench: %s"  % str(distribute.ankleWrenchLeft.value) )
//assertApprox(ankleWrenchLeft,distribute.ankleWrenchLeft.value,3)
//print( "resulting ankle right wrench: %s" % str(distribute.ankleWrenchRight.value) )
//assertApprox(ankleWrenchRight,distribute.ankleWrenchRight.value,3)

//distribute.copLeft.recompute(0)
//distribute.copRight.recompute(0)

//print( "resulting sole left CoP: %s"  % str(distribute.copLeft.value) )
//assertApprox(copLeft,distribute.copLeft.value,3)
//print( "resulting sole right CoP: %s" % str(distribute.copRight.value) )
//assertApprox(copRight,distribute.copRight.value,3)

//distribute.emergencyStop.recompute(0)
//stop = distribute.emergencyStop.value
//np.testing.assert_equal(stop,0)

//# --- Wrench saturation (left) ---
//print()
//print("--- Wrench saturation ---")
//print('NOTE: "predicted" wrench values are not accurate due to the foot saturation and as such they are not checked.')
//print("CoP values are predicted under the assumption that they are at the foot border and as such they are checked.")

//# --- Wrench saturation (left) ---
//print()
//print("--- Wrench saturation (left) ---")
//distribute.phase.value = 1
//distribute.phase.time = 1

//wrenchLeft  = wrench
//ankleWrenchLeft  = list(leftPos.actInv(pin.Force(np.matrix(wrenchLeft).T)).vector.flat)

//print( "expected global wrench: %s" % str(wrench) )
//print( "expected global left wrench: %s"  % str(wrenchLeft) )
//print( "expected ankle left wrench: %s"  % str(ankleWrenchLeft) )

//copLeft  = [float(com[0] - leftPos.translation[0]),  distribute_conf.RIGHT_FOOT_SIZES[3], 0.]

//print( "expected sole left CoP: %s"  % str(copLeft) )
//print()

//distribute.zmpRef.recompute(1)

//print( "resulting global wrench: %s" % str(distribute.wrenchRef.value) )
//#assertApprox(wrench,distribute.wrenchRef.value,2)
//print( "resulting global left wrench: %s"  % str(distribute.wrenchLeft.value) )
//#assertApprox(wrenchLeft,distribute.wrenchLeft.value,3)

//distribute.ankleWrenchLeft.recompute(1)

//print( "resulting ankle left wrench: %s"  % str(distribute.ankleWrenchLeft.value) )
//#assertApprox(ankleWrenchLeft,distribute.ankleWrenchLeft.value,3)

//distribute.copLeft.recompute(1)
//distribute.copRight.recompute(1)

//print( "resulting sole left CoP: %s"  % str(distribute.copLeft.value) )
//assertApprox(copLeft,distribute.copLeft.value,3)

//distribute.emergencyStop.recompute(0)
//stop = distribute.emergencyStop.value
//np.testing.assert_equal(stop,0)

//# --- Wrench saturation (right) ---
//print()
//print("--- Wrench saturation (right) ---")
//distribute.phase.value = -1
//distribute.phase.time = 2

//wrenchRight = wrench
//ankleWrenchRight = list(rightPos.actInv(pin.Force(np.matrix(wrenchRight).T)).vector.flat)

//print( "expected global wrench: %s" % str(wrench) )
//print( "expected global right wrench: %s" % str(wrenchRight) )
//print( "expected ankle right wrench: %s" % str(ankleWrenchRight) )

//copRight = [float(com[0] - rightPos.translation[0]),  distribute_conf.RIGHT_FOOT_SIZES[2], 0.]

//print( "expected sole right CoP: %s" % str(copRight) )
//print()

//distribute.zmpRef.recompute(2)

//print( "resulting global wrench: %s" % str(distribute.wrenchRef.value) )
//#assertApprox(wrench,distribute.wrenchRef.value,2)
//print( "resulting global right wrench: %s" % str(distribute.wrenchRight.value) )
//#assertApprox(wrenchRight,distribute.wrenchRight.value,3)

//distribute.ankleWrenchRight.recompute(2)

//print( "resulting ankle right wrench: %s" % str(distribute.ankleWrenchRight.value) )
//#assertApprox(ankleWrenchRight,distribute.ankleWrenchRight.value,3)

//distribute.copLeft.recompute(2)
//distribute.copRight.recompute(2)

//print( "resulting sole right CoP: %s" % str(distribute.copRight.value) )
//assertApprox(copRight,distribute.copRight.value,3)

//distribute.emergencyStop.recompute(0)
//stop = distribute.emergencyStop.value
//np.testing.assert_equal(stop,0)

}
