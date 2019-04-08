/*
 * Copyright 2019
 *
 * LAAS-CNRS
 *
 * Fanny Risbourg 
 * This file is part of sot-talos-balance.
 * See license file.
 */

#ifndef __sot_talos_balance_admittance_controller_end_effector_H__
#define __sot_talos_balance_admittance_controller_end_effector_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (admittance_controller_end_effector_EXPORTS)
#    define ADMITTANCECONTROLLERENDEFFECTOR_EXPORT __declspec(dllexport)
#  else
#    define ADMITTANCECONTROLLERENDEFFECTOR_EXPORT __declspec(dllimport)
#  endif
#else
#  define ADMITTANCECONTROLLERENDEFFECTOR_EXPORT
#endif


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic-graph/signal-helper.h>
#include <map>
#include "boost/assign.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include "pinocchio/spatial/se3.hpp"
#include <sot/core/robot-utils.hh>
#include <pinocchio/algorithm/kinematics.hpp>


namespace dynamicgraph {
  namespace sot {
    namespace talos_balance {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */

      /**
       * @brief      Admittance controller for an upper body end-effector (right or left wrist
       *
       *  This entity computes a velocity reference for an end-effector based on the force error in the world frame :
       *  dqRef = integral( Kp(forceDes-forceWorldFrame) )
       *
       */
      class ADMITTANCECONTROLLERENDEFFECTOR_EXPORT AdmittanceControllerEndEffector
          : public ::dynamicgraph::Entity
      {
      DYNAMIC_GRAPH_ENTITY_DECL();

      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /* --- CONSTRUCTOR ---- */
        AdmittanceControllerEndEffector(const std::string & name);

        /* --- SIGNALS --- */
        /// \brief     Gain (6d) for the integration of the error on the force
        DECLARE_SIGNAL_IN(Kp, dynamicgraph::Vector);
        /// \brief     6d force given by the sensor in its local frame
        DECLARE_SIGNAL_IN(force, dynamicgraph::Vector);
        /// \brief     6d desired force of the end-effector in the world frame
        DECLARE_SIGNAL_IN(forceDes, dynamicgraph::Vector);
        /// \brief     Current joint configuration of the robot
        DECLARE_SIGNAL_IN(jointPosition, dynamicgraph::Vector);

        /// \brief     Velocity reference for the end-effector computed by the controller
      DECLARE_SIGNAL_OUT(dqRef, dynamicgraph::Vector);

        /// \brief     6d force given by the sensor in he global frame
      DECLARE_SIGNAL_INNER(forceWorldFrame, dynamicgraph::Vector);

        /* --- COMMANDS --- */
        /**
        * @brief      Initialize the entity
        *
        * @param[in]  dt  Time step of the control
        * @param[in]  sensorFrameName  Name of the force sensor of the end-effector used in the pinocchio model
        */
        void init(const double & dt, const std::string & sensorFrameName);
        /**
         * @brief      Reset the velocity
         */
        void reset_dq();

        /* --- ENTITY INHERITANCE --- */
        virtual void display( std::ostream& os ) const;

      protected:
        /// Dimension of the force signals and of the output
        int                    m_n;
        /// True if the entity has been successfully initialized
        bool                   m_initSucceeded;
        /// Internal state
        dynamicgraph::Vector   m_dq;
        /// Time step of the control
        double                 m_dt;

        /// Robot Util instance to get the sensor frame
        RobotUtilShrPtr             m_robot_util;
        /// Pinocchio robot model
        pinocchio::Model             m_model;
        /// Pinocchio robot data
        pinocchio::Data             *m_data;
        /// Force sensor frame placement wrt the parent frame
        pinocchio::SE3               m_sensorFrame;
        /// Id of the parent joint of the force sensor frame
        pinocchio::JointIndex        m_parentId;
        /// robot configuration according to pinocchio convention
        dynamicgraph::Vector   m_q;

      }; // class AdmittanceControllerEndEffector

    }    // namespace talos_balance
  }      // namespace sot
}        // namespace dynamicgraph



#endif // #ifndef __sot_talos_balance_admittance_controller_end_effector_H__
