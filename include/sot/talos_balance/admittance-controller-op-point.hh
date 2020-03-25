/*
 * Copyright 2019
 *
 * LAAS-CNRS
 *
 * Noëlie Ramuzat
 * This file is part of sot-talos-balance.
 * See license file.
 */

#ifndef __sot_talos_balance_admittance_controller_op_point_H__
#define __sot_talos_balance_admittance_controller_op_point_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#  if defined(admittance_controller_op_point_EXPORTS)
#    define ADMITTANCECONTROLLEROPPOINT_EXPORT __declspec(dllexport)
#  else
#    define ADMITTANCECONTROLLEROPPOINT_EXPORT __declspec(dllimport)
#  endif
#else
#  define ADMITTANCECONTROLLEROPPOINT_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic-graph/signal-helper.h>

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

#include <sot/core/matrix-geometry.hh>

namespace dynamicgraph
{
namespace sot
{
namespace talos_balance
{

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/**
 * @brief  Admittance controller for an operational point wrt to a force sensor. 
 *         It can be a point of the model (hand) or not (created operational point: an object in the hand of the robot)
 *         Which is closed to a force sensor (for instance the right or left wrist ft)
 *
 *  This entity computes a velocity reference for an operational point based 
 *  on the force error in the world frame :
 *  w_dq = integral(Kp(w_forceDes-w_force)) + Kd (w_dq)
 *
 */
class ADMITTANCECONTROLLEROPPOINT_EXPORT AdmittanceControllerOpPoint
    : public ::dynamicgraph::Entity
{
  DYNAMIC_GRAPH_ENTITY_DECL();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* --- CONSTRUCTOR ---- */
  AdmittanceControllerOpPoint(const std::string &name);

  /* --- SIGNALS --- */
  /// \brief  Gain (6d) for the integration of the error on the force
  DECLARE_SIGNAL_IN(Kp, dynamicgraph::Vector);
  /// \brief  Derivative gain (6d) for the error on the force
  DECLARE_SIGNAL_IN(Kd, dynamicgraph::Vector);
  /// \brief  Value of the saturation to apply on the velocity output
  DECLARE_SIGNAL_IN(dqSaturation, dynamicgraph::Vector);
  /// \brief  6d force given by the sensor in its local frame
  DECLARE_SIGNAL_IN(force, dynamicgraph::Vector);
  /// \brief  6d desired force of the end-effector in the world frame
  DECLARE_SIGNAL_IN(w_forceDes, dynamicgraph::Vector);
  /// \brief  Current position (matrixHomogeneous) of the given operational point
  DECLARE_SIGNAL_IN(opPose, dynamicgraph::sot::MatrixHomogeneous);
  /// \brief  Current position (matrixHomogeneous) of the given force sensor
  DECLARE_SIGNAL_IN(sensorPose, dynamicgraph::sot::MatrixHomogeneous);

  /// \brief  6d force given by the sensor in the world frame
  DECLARE_SIGNAL_INNER(w_force, dynamicgraph::Vector);
  /// \brief  Internal intergration computed in the world frame
  DECLARE_SIGNAL_INNER(w_dq, dynamicgraph::Vector);

  /// \brief  Velocity reference for the end-effector in the local frame
  DECLARE_SIGNAL_OUT(dq, dynamicgraph::Vector);

  /* --- COMMANDS --- */
  /**
   * @brief      Initialize the entity
   *
   * @param[in]  dt  Time step of the control
   */
  void init(const double &dt);

  /**
   * @brief      Reset the velocity
   */
  void resetDq();

  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream &os) const;

protected:
  /// Dimension of the force signals and of the output
  int m_n;
  /// True if the entity has been successfully initialized
  bool m_initSucceeded;
  /// Internal state
  dynamicgraph::Vector m_w_dq;
  /// Time step of the control
  double m_dt;
  // Weight of the end-effector
  double m_mass;
  
}; // class AdmittanceControllerOpPoint

} // namespace talos_balance
} // namespace sot
} // namespace dynamicgraph

#endif // #ifndef __sot_talos_balance_admittance_controller_op_point_H__
