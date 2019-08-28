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

#include "sot/talos_balance/hip-flexibility-compensation.hh"

#include <limits>
#include <math.h>
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

#define PROFILE_HIPFLEXIBILITYCOMPENSATION_DELTAQ_COMPUTATION \
  "HipFlexibilityCompensation: Angular correction computation   "
#define PROFILE_HIPFLEXIBILITYCOMPENSATION_QCMD_COMPUTATION \
  "HipFlexibilityCompensation: Corrected joint configuration computation   "

#define JOINT_DES_SIGNALS m_q_desSIN

#define INPUT_SIGNALS m_tauSIN << m_K_rSIN << m_K_lSIN 

#define OUTPUT_SIGNALS m_delta_qSOUT << m_q_cmdSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef HipFlexibilityCompensation EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(HipFlexibilityCompensation,
                                   "HipFlexibilityCompensation");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
HipFlexibilityCompensation::HipFlexibilityCompensation(const std::string& name)
  : Entity(name)
  , CONSTRUCT_SIGNAL_IN(q_des, dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(tau, dynamicgraph::Vector)
  , CONSTRUCT_SIGNAL_IN(K_l, double)
  , CONSTRUCT_SIGNAL_IN(K_r, double)
  , CONSTRUCT_SIGNAL_OUT(delta_q, dynamicgraph::Vector, INPUT_SIGNALS) 
  , CONSTRUCT_SIGNAL_OUT(q_cmd, dynamicgraph::Vector, JOINT_DES_SIGNALS << m_delta_qSOUT)
  , m_initSucceeded(false) 
  , m_lowPassFilterFrequency(0)
  , m_delta_q_saturation(1e6){

  Entity::signalRegistration( JOINT_DES_SIGNALS << INPUT_SIGNALS << OUTPUT_SIGNALS );

  /* Commands. */
  addCommand("init", makeCommandVoid2(*this, 
                                      &HipFlexibilityCompensation::init,
                                      docCommandVoid2("Initialize the entity.", 
                                                      "Robot time step",
                                                      "Robot name")));

  addCommand("setLowPassFilterFrequency", makeCommandVoid1(*this, 
                                          &HipFlexibilityCompensation::setLowPassFilterFrequency,
                                          docCommandVoid1("Set the LowPassFilter frequency for the angular correction computation.",
                                                          "Value of the frequency")));

  addCommand("setAngularSaturation", makeCommandVoid1(*this, 
                                     &HipFlexibilityCompensation::setAngularSaturation,
                                     docCommandVoid1("Set the saturation for the angular correction computation.",
                                                     "Value of the saturation")));

  addCommand("getLowPassFilterFrequency", makeDirectGetter(*this, &m_lowPassFilterFrequency,
             docDirectGetter("Get the current value of the LowPassFilter frequency.", "frequency (double)")));
  
  addCommand("getAngularSaturation", makeDirectGetter(*this, &m_delta_q_saturation,
             docDirectGetter("Get the current value of the Angular Saturation.", "saturation (double)")));
}

/* --- COMMANDS ---------------------------------------------------------- */

void HipFlexibilityCompensation::init(const double &dt, const std::string& robotName) {
  if (!m_q_desSIN.isPlugged())
    return SEND_MSG("Init failed: signal q_des is not plugged", MSG_TYPE_ERROR);
  if (!m_tauSIN.isPlugged())
    return SEND_MSG("Init failed: signal tau is not plugged", MSG_TYPE_ERROR);
  if (!m_K_rSIN.isPlugged())
    return SEND_MSG("Init failed: signal K_r is not plugged", MSG_TYPE_ERROR);
  if (!m_K_lSIN.isPlugged())
    return SEND_MSG("Init failed: signal K_l is not plugged", MSG_TYPE_ERROR);

  m_dt = dt;
  std::string robotName_nonconst(robotName);

  if (!isNameInRobotUtil(robotName_nonconst)) {
    SEND_MSG("You should have a robotUtil pointer initialized before", MSG_TYPE_ERROR);
  } else {
    m_robot_util = getRobotUtil(robotName_nonconst);
    std::cerr << "m_robot_util:" << m_robot_util << std::endl;
  }

  m_initSucceeded = true;
  const Vector& q_des = m_q_desSIN.accessCopy();
  m_previous_delta_q.resize(q_des.size());
  m_previous_delta_q.setZero();
}

void HipFlexibilityCompensation::setLowPassFilterFrequency(const double& frequency) {
  m_lowPassFilterFrequency = frequency;
}

void HipFlexibilityCompensation::setAngularSaturation(const double& saturation) {
  m_delta_q_saturation = saturation;
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_OUT_FUNCTION(delta_q, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal delta_q before initialization!");
    return s;
  }

  getProfiler().start(PROFILE_HIPFLEXIBILITYCOMPENSATION_DELTAQ_COMPUTATION);

  const Vector &tau = m_tauSIN(iter);
  const double &K_r = m_K_rSIN(iter);
  const double &K_l = m_K_lSIN(iter);

  if(s.size() != tau.size())
    s.setZero(tau.size());

  for (unsigned int i=0; i<tau.size(); i++){
    if (i==1){
      s[i] = tau[i]/K_l; // torque/flexibility of left hip (roll) 
    }
    else if (i==7){
      s[i] = tau[i]/K_r; // torque/flexibility of right hip (roll) 
    }
    else {
      double inf = std::numeric_limits<double>::infinity();
      s[i] = tau[i]/inf; // no flexibility for other joints
    }
  }

  // Low pass filter
  // delta_q = alpha * previous_delta_q(-1) + (1-alpha) * delta_q_des
  double alpha = exp(- m_dt * m_lowPassFilterFrequency);
  s = alpha * m_previous_delta_q + s * (1 - alpha);
  m_previous_delta_q = s;

  // Angular Saturation
  // left hip
  if (s[1] > m_delta_q_saturation){
    s[1] = m_delta_q_saturation;
  }
  else if (s[1] < -m_delta_q_saturation){
    s[1] = -m_delta_q_saturation;
  }
  // right hip
  if (s[7] > m_delta_q_saturation){
    s[7] = m_delta_q_saturation;
  }
  else if (s[7] < -m_delta_q_saturation){
    s[7] = -m_delta_q_saturation;
  }

  getProfiler().stop(PROFILE_HIPFLEXIBILITYCOMPENSATION_DELTAQ_COMPUTATION);
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(q_cmd, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal q_cmd before initialization!");
    return s;
  }

  getProfiler().start(PROFILE_HIPFLEXIBILITYCOMPENSATION_QCMD_COMPUTATION);

  const Vector &q_des = m_q_desSIN(iter);
  const Vector &delta_q = m_delta_qSOUT(iter);

  if(s.size() != q_des.size())
    s.resize(q_des.size());

  s = q_des + delta_q;

  getProfiler().stop(PROFILE_HIPFLEXIBILITYCOMPENSATION_QCMD_COMPUTATION);

  return s;
}


/* ------------------------------------------------------------------- */
/* --- ENTITY INHERITANCE -------------------------------------------- */
/* ------------------------------------------------------------------- */

void HipFlexibilityCompensation::display(std::ostream& os) const {
  os << "HipFlexibilityCompensation " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {}
}

} // namespace talos_balance
} // namespace sot
} // namespace dynamicgraph

