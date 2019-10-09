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

#define PROFILE_HIPFLEXIBILITYCOMPENSATION_TAUFILT_COMPUTATION \
  "HipFlexibilityCompensation: Torque filter computation   "
#define PROFILE_HIPFLEXIBILITYCOMPENSATION_DELTAQ_COMPUTATION \
  "HipFlexibilityCompensation: Angular correction computation   "
#define PROFILE_HIPFLEXIBILITYCOMPENSATION_QCMD_COMPUTATION \
  "HipFlexibilityCompensation: Corrected joint configuration computation   "

#define JOINT_DES_SIGNALS m_q_desSIN

#define INPUT_SIGNALS m_tauSIN << m_K_rSIN << m_K_lSIN //<< m_K_dSIN

#define OUTPUT_SIGNALS m_tau_filtSOUT << m_delta_qSOUT << m_q_cmdSOUT

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
  // , CONSTRUCT_SIGNAL_IN(K_d, double)
  // , CONSTRUCT_SIGNAL_OUT(tau_dot, dynamicgraph::Vector, m_tauSIN)
  , CONSTRUCT_SIGNAL_OUT(tau_filt, dynamicgraph::Vector, m_tauSIN)
  , CONSTRUCT_SIGNAL_OUT(delta_q, dynamicgraph::Vector, INPUT_SIGNALS << m_tau_filtSOUT) 
  , CONSTRUCT_SIGNAL_OUT(q_cmd, dynamicgraph::Vector, JOINT_DES_SIGNALS << m_delta_qSOUT) //m_K_dSIN 
  , m_initSucceeded(false) 
  , m_angularLowPassFilterFrequency(1)
  , m_torqueLowPassFilterFrequency(1)
  , m_delta_q_saturation(0.0034)
  , m_rate_limiter(1.0){

  Entity::signalRegistration( JOINT_DES_SIGNALS << INPUT_SIGNALS << OUTPUT_SIGNALS );

  /* Commands. */
  addCommand("init", makeCommandVoid2(*this, 
                                      &HipFlexibilityCompensation::init,
                                      docCommandVoid2("Initialize the entity.", 
                                                      "Robot time step",
                                                      "Robot name")));

  addCommand("setAngularLowPassFilterFrequency", makeCommandVoid1(*this, 
                                                 &HipFlexibilityCompensation::setAngularLowPassFilterFrequency,
                                                 docCommandVoid1("Set the LowPassFilter frequency for the angular correction computation.",
                                                                 "Value of the frequency")));
  addCommand("setTorqueLowPassFilterFrequency", makeCommandVoid1(*this, 
                                                &HipFlexibilityCompensation::setTorqueLowPassFilterFrequency,
                                                docCommandVoid1("Set the LowPassFilter frequency for the torque derivative computation.",
                                                                "Value of the frequency")));
  addCommand("setAngularSaturation", makeCommandVoid1(*this, 
                                     &HipFlexibilityCompensation::setAngularSaturation,
                                     docCommandVoid1("Set the saturation for the angular correction computation.",
                                                     "Value of the saturation")));
  addCommand("setRateLimiter", makeCommandVoid1(*this, 
                                     &HipFlexibilityCompensation::setRateLimiter,
                                     docCommandVoid1("Set the rate for the rate limiter of delta_q.",
                                                     "Value of the limiter")));

  addCommand("getAngularLowPassFilterFrequency", makeDirectGetter(*this, &m_angularLowPassFilterFrequency,
             docDirectGetter("Get the current value of the angular LowPassFilter frequency.", "frequency (double)")));

  addCommand("getTorqueLowPassFilterFrequency", makeDirectGetter(*this, &m_torqueLowPassFilterFrequency,
             docDirectGetter("Get the current value of the torque LowPassFilter frequency.", "frequency (double)")));
  
  addCommand("getAngularSaturation", makeDirectGetter(*this, &m_delta_q_saturation,
             docDirectGetter("Get the current value of the Angular Saturation.", "saturation (double)")));

  addCommand("getRateLimiter", makeDirectGetter(*this, &m_rate_limiter,
             docDirectGetter("Get the current value of the rate limiter.", "rate (double)")));
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
  // if (!m_K_dSIN.isPlugged())
  //   return SEND_MSG("Init failed: signal K_d is not plugged", MSG_TYPE_ERROR);

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
  const Vector& tau = m_tauSIN.accessCopy();
  m_previous_delta_q.resize(q_des.size());
  m_previous_delta_q.setZero();
  m_previous_tau.resize(tau.size());
  m_previous_tau.setZero();
  // m_previous_tau_dot.resize(tau.size());
  // m_previous_tau_dot.setZero();  
  // m_previous_q.resize(q_des.size());
  // m_previous_q.setZero();
}

void HipFlexibilityCompensation::setAngularLowPassFilterFrequency(const double& frequency) {
  m_angularLowPassFilterFrequency = frequency;
}

void HipFlexibilityCompensation::setTorqueLowPassFilterFrequency(const double& frequency) {
  m_torqueLowPassFilterFrequency = frequency;
}

void HipFlexibilityCompensation::setAngularSaturation(const double& saturation) {
  m_delta_q_saturation = saturation;
}

void HipFlexibilityCompensation::setRateLimiter(const double& rate) {
  m_rate_limiter = rate;
}

Vector HipFlexibilityCompensation::lowPassFilter(const double& frequency, const Vector& signal, Vector& previous_signal){
  // delta_q = alpha * previous_delta_q(-1) + (1-alpha) * delta_q_des
  double alpha = exp(- m_dt * 2 * M_PI * frequency);
  std::cout << "alpha : " << alpha << std::endl;
  Vector output = alpha * previous_signal + signal * (1 - alpha);
  return output;
}

void HipFlexibilityCompensation::rateLimiter(const Vector& signal, Vector& previous_signal, Vector& output){
  Vector rate = (signal - previous_signal)/m_dt;  
  // Falling slew rate = - Rising slew rate  = - m_rate_limiter
  for (unsigned int i=0; i<signal.size(); i++){
    if (rate[i] > m_rate_limiter){
      output[i] = m_dt * m_rate_limiter + previous_signal[i];
    } else if (rate[i] < -m_rate_limiter){
      output[i] = m_dt * (-m_rate_limiter) + previous_signal[i];
    } else {
      output[i] = signal[i];
    }
  }
}
/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

// DEFINE_SIGNAL_OUT_FUNCTION(tau_dot, dynamicgraph::Vector) {
//   if (!m_initSucceeded) {
//     SEND_WARNING_STREAM_MSG("Cannot compute signal tau_dot before initialization!");
//     return s;
//   }

//   getProfiler().start(PROFILE_HIPFLEXIBILITYCOMPENSATION_TAUDOT_COMPUTATION);

//   const Vector& tau = m_tauSIN(iter);

//   if(s.size() != tau.size())
//     s.resize(tau.size());

//   if (iter < 5){
//     s.setZero();
//     m_previous_tau_dot = s;
//   } else {    
//     s = (tau - m_previous_tau)/m_dt;    
//     // Low pass filter  
//     s = lowPassFilter(m_torqueLowPassFilterFrequency, s, m_previous_tau_dot);
//   }     
//   m_previous_tau_dot = s;
//   m_previous_tau = tau;  
//   getProfiler().stop(PROFILE_HIPFLEXIBILITYCOMPENSATION_TAUDOT_COMPUTATION);

//   return s;
// }
// 
DEFINE_SIGNAL_OUT_FUNCTION(tau_filt, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal tau_filt before initialization!");
    return s;
  }

  getProfiler().start(PROFILE_HIPFLEXIBILITYCOMPENSATION_TAUFILT_COMPUTATION);

  const Vector& tau = m_tauSIN(iter);

  if(s.size() != tau.size())
    s.resize(tau.size());
  
  if (iter < 5){
    s = tau;
  } else {
    // Low pass filter  
    s = lowPassFilter(m_torqueLowPassFilterFrequency, tau, m_previous_tau);     
  }
  m_previous_tau = tau;  
  getProfiler().stop(PROFILE_HIPFLEXIBILITYCOMPENSATION_TAUFILT_COMPUTATION);

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(delta_q, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal delta_q before initialization!");
    return s;
  }

  getProfiler().start(PROFILE_HIPFLEXIBILITYCOMPENSATION_DELTAQ_COMPUTATION);

  const Vector &tau = m_tau_filtSOUT(iter);
  double K_r = m_K_rSIN(iter);
  double K_l = m_K_lSIN(iter);
  
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
      // double inf = std::numeric_limits<double>::infinity();
      s[i] = 0.0; //tau[i]/inf; // no flexibility for other joints
    }
  }
  // // Low pass filter
  // s = lowPassFilter(m_angularLowPassFilterFrequency, s, m_previous_delta_q);
  // m_previous_delta_q = s;
  
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
  // const double &K_d = m_K_dSIN(iter);
  // const Vector &tau_dot = m_tau_dotSOUT(iter);
  const Vector &delta_q = m_delta_qSOUT(iter);

  if(s.size() != q_des.size())
    s.resize(q_des.size());

  Vector limitedSignal;
  limitedSignal.resize(delta_q.size());
  if (iter < 5){
    s = q_des;
  } else {
    rateLimiter(delta_q, m_previous_delta_q, limitedSignal);
    s = q_des + limitedSignal;
  }
  m_previous_delta_q = delta_q;

  // tempSignal[1] = tempSignal[1] - K_d * tau_dot[1];  
  // tempSignal[7] = tempSignal[7] - K_d * tau_dot[7];

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

