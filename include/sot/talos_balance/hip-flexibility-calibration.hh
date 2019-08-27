/*
 * Copyright 2018, Gepetto team, LAAS-CNRS
 *
 * This file is part of sot-talos-balance.
 * sot-talos-balance is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-talos-balance is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-talos-balance.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __sot_talos_balance_hip_flexibility_calibration_H__
#define __sot_talos_balance_hip_flexibility_calibration_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(hip_flexibility_calibration_EXPORTS)
#define HIPFLEXIBILITYCALIBRATION_EXPORT __declspec(dllexport)
#else
#define HIPFLEXIBILITYCALIBRATION_EXPORT __declspec(dllimport)
#endif
#else
#define HIPFLEXIBILITYCALIBRATION_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic-graph/signal-helper.h>

#include <map>
#include <sot/core/robot-utils.hh>
#include "boost/assign.hpp"

namespace dynamicgraph {
namespace sot {
namespace talos_balance {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class HIPFLEXIBILITYCALIBRATION_EXPORT HipFlexibilityCalibration
    : public ::dynamicgraph::Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* --- CONSTRUCTOR ---- */
  HipFlexibilityCalibration(const std::string& name);

  /* --- SIGNALS --- */
  /// \brief  Desired joint configuration of the robot
  DECLARE_SIGNAL_IN(q_des, dynamicgraph::Vector);
  /// \brief  Current torque mesured at each joint
  DECLARE_SIGNAL_IN(tau, dynamicgraph::Vector);
  /// \brief Left flexibility correction for the angular computation
  DECLARE_SIGNAL_IN(K_l, double);
  /// \brief Right flexibility correction for the angular computation
  DECLARE_SIGNAL_IN(K_r, double);

  /// \brief  Angular correction of the flexibility 
  DECLARE_SIGNAL_OUT(theta_diff, dynamicgraph::Vector);
  /// \brief  Corrected desired joint configuration of the robot with flexibilityjoint configuration
  /// q_cmd = q_des + theta_diff
  DECLARE_SIGNAL_OUT(q_cmd, dynamicgraph::Vector);

  /* --- COMMANDS --- */
  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream& os) const;

  /// \brief Initialize the entity
  void init(const std::string& robotName);
  /// \brief Activate the LowPassFilter for the angular correction computation.
  void activateLowPassFilter();
  /// \brief Activate the saturation for the angular correction computation, set the value of the saturation.
  void activateAngularSaturation(const double& saturation);

 protected:
  bool m_initSucceeded;  /// true if the entity has been successfully initialized
  bool m_useLowPassFilter;
  bool m_useAngularSaturation;
  double m_theta_diff_saturation;

  RobotUtilShrPtr m_robot_util;

};  // class HipFlexibilityCalibration

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

#endif // #ifndef __sot_talos_balance_hip_flexibility_calibration_H__