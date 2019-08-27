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

#include "sot/talos_balance/hip-flexibility-calibration.hh"

#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>
#include <sot/core/stop-watch.hh>


namespace dynamicgraph {
namespace sot {
namespace talos_balance {
namespace dg = ::dynamicgraph;
using namespace dg;
using namespace dg::command;

#define PROFILE_HIPFLEXIBILITYCALIBRATION_THETADIFF_COMPUTATION \
  "HipFlexibilityCalibration: Angular correction computation   "
#define PROFILE_HIPFLEXIBILITYCALIBRATION_QCMD_COMPUTATION \
  "HipFlexibilityCalibration: Corrected joint configuration computation   "

#define JOINT_DES_SIGNALS m_q_desSIN

#define INPUT_SIGNALS m_tauSIN << m_K_rSIN << m_K_lSIN 

#define OUTPUT_SIGNALS m_theta_diffSOUT << m_q_cmdSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef HipFlexibilityCalibration EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(HipFlexibilityCalibration,
                                   "HipFlexibilityCalibration");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
HipFlexibilityCalibration::HipFlexibilityCalibration(const std::string& name)
  : Entity(name)
  , CONSTRUCT_SIGNAL_IN(q_des, dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(tau, dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(K_l, double)
  , CONSTRUCT_SIGNAL_IN(K_r, double)
  , CONSTRUCT_SIGNAL_OUT(theta_diff, dynamicgraph::Vector, INPUT_SIGNALS)
  , CONSTRUCT_SIGNAL_OUT(q_cmd, dynamicgraph::Vector, JOINT_DES_SIGNALS << m_theta_diffSOUT)
  , m_initSucceeded(false)
  , m_useLowPassFilter(false)
  , m_useAngularSaturation(false) {

  Entity::signalRegistration( JOINT_DES_SIGNALS << INPUT_SIGNALS << OUTPUT_SIGNALS );

  /* Commands. */
  addCommand("init", makeCommandVoid1(*this, 
                                      &HipFlexibilityCalibration::init,
                                      docCommandVoid1("Initialize the entity.", "Robot name")));

  addCommand("activateLowPassFilter", makeCommandVoid0(*this, 
                                      &HipFlexibilityCalibration::activateLowPassFilter,
                                      docCommandVoid0("Activate the LowPassFilter for the angular correction computation.")));

  addCommand("activateAngularSaturation", makeCommandVoid1(*this, 
                                      &HipFlexibilityCalibration::activateAngularSaturation,
                                      docCommandVoid1("Activate the saturation for the angular correction computation.",
                                                      "Value of the saturation")));
}

/* --- COMMANDS ---------------------------------------------------------- */

void HipFlexibilityCalibration::init(const std::string& robotName) {
  if (!m_q_desSIN.isPlugged())
    return SEND_MSG("Init failed: signal q_des is not plugged", MSG_TYPE_ERROR);
  if (!m_tauSIN.isPlugged())
    return SEND_MSG("Init failed: signal tau is not plugged", MSG_TYPE_ERROR);
  if (!m_K_rSIN.isPlugged())
    return SEND_MSG("Init failed: signal K_r is not plugged", MSG_TYPE_ERROR);
  if (!m_K_lSIN.isPlugged())
    return SEND_MSG("Init failed: signal K_l is not plugged", MSG_TYPE_ERROR);

  std::string robotName_nonconst(robotName);

  if (!isNameInRobotUtil(robotName_nonconst)) {
    SEND_MSG("You should have a robotUtil pointer initialized before", MSG_TYPE_ERROR);
  } else {
    m_robot_util = getRobotUtil(robotName_nonconst);
    std::cerr << "m_robot_util:" << m_robot_util << std::endl;
  }

  m_initSucceeded = true;
}

void HipFlexibilityCalibration::activateLowPassFilter() {
  m_useLowPassFilter = true;
}

void HipFlexibilityCalibration::activateAngularSaturation(const double& saturation) {
  m_useAngularSaturation = true;
  m_theta_diff_saturation = saturation;
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_OUT_FUNCTION(theta_diff, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal theta_diff before initialization!");
    return s;
  }

  getProfiler().start(PROFILE_HIPFLEXIBILITYCALIBRATION_THETADIFF_COMPUTATION);

  const Vector &tau = m_tauSIN(iter);
  const double &K_r = m_K_rSIN(iter);
  const double &K_l = m_K_lSIN(iter);

  if(s.size() != tau.size())
    s.setZero(tau.size());

  double mod_tau_l = tau[1]/K_l; // torque/flexibility of left hip (roll) 
  double mod_tau_r = tau[7]/K_r; // torque/flexibility of right hip (roll) 
  // std::cout << "mod_tau_l: " << mod_tau_l << std::endl;
  // std::cout << "mod_tau_r: " << mod_tau_r << std::endl;

  if (m_useLowPassFilter){
    std::cout << "useLowPassFilter" << std::endl;
    double RC = 1.0; // time constant
    double alpha = 1/(1+RC); // smoothing factor: alpha = dt/(RC+dt) so here 1/(1+RC)=0.5
    mod_tau_l = alpha * mod_tau_l;
    mod_tau_r = alpha * mod_tau_r;
  }
  
  if (m_useAngularSaturation){    
    std::cout << "useAngularSaturation" << std::endl;
    if (mod_tau_l > m_theta_diff_saturation){
      mod_tau_l = m_theta_diff_saturation;
      // std::cout << "mod_tau_l: " << mod_tau_l << std::endl;
    }
    else if (mod_tau_l < -m_theta_diff_saturation){
      mod_tau_l = -m_theta_diff_saturation;
      // std::cout << "mod_tau_l: " << mod_tau_l << std::endl;
    }

    if (mod_tau_r > m_theta_diff_saturation){
      mod_tau_r = m_theta_diff_saturation;
      // std::cout << "mod_tau_r: " << mod_tau_r << std::endl;
    }
    else if (mod_tau_r < -m_theta_diff_saturation){
      mod_tau_r = -m_theta_diff_saturation;
      // std::cout << "mod_tau_r: " << mod_tau_r << std::endl;
    }
  }

  s[1] = mod_tau_l;
  s[7] = mod_tau_r;

  getProfiler().stop(PROFILE_HIPFLEXIBILITYCALIBRATION_THETADIFF_COMPUTATION);
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(q_cmd, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal q_cmd before initialization!");
    return s;
  }

  getProfiler().start(PROFILE_HIPFLEXIBILITYCALIBRATION_QCMD_COMPUTATION);

  const Vector &q_des = m_q_desSIN(iter);
  const Vector &theta_diff = m_theta_diffSOUT(iter);

  if(s.size() != q_des.size())
    s.resize(q_des.size());

  s = q_des + theta_diff;

  getProfiler().stop(PROFILE_HIPFLEXIBILITYCALIBRATION_QCMD_COMPUTATION);

  return s;
}


/* ------------------------------------------------------------------- */
/* --- ENTITY INHERITANCE -------------------------------------------- */
/* ------------------------------------------------------------------- */

void HipFlexibilityCalibration::display(std::ostream& os) const {
  os << "HipFlexibilityCalibration " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {}
}

} // namespace talos_balance
} // namespace sot
} // namespace dynamicgraph

