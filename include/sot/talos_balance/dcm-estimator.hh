/*
 * Copyright 2019, 
 * LAAS-CNRS,
 * Gepetto team 
 *
 * This file is part of sot-talos-balance.
 * See license file 
 */

#ifndef __sot_talos_balance_dcm_estimator_H__
#define __sot_talos_balance_dcm_estimator_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (dcm_estimator_EXPORTS)
#    define DCMESTIMATOR_EXPORT __declspec(dllexport)
#  else
#    define DCMESTIMATOR_EXPORT __declspec(dllimport)
#  endif
#else
#  define DCMESTIMATOR_EXPORT
#endif


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <sot/core/signal-helper.hh>
#include <sot/core/matrix-geometry.hh>
#include <sot/core/logger.hh>
#include <map>
#include "boost/assign.hpp"
#include <boost/math/distributions/normal.hpp> // for normal_distribution

/* Pinocchio */
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <sot/core/robot-utils.hh>

namespace dynamicgraph 
{
  namespace sot 
  {
    namespace talos_balance 
    {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */

      class DCMESTIMATOR_EXPORT DcmEstimator
	                         :public::dynamicgraph::Entity
      {
        typedef se3::SE3 SE3;
        typedef Eigen::Vector2d Vector2;
        typedef Eigen::Vector3d Vector3;
        typedef Eigen::Vector4d Vector4;
        typedef Eigen::Vector6d Vector6;
        typedef Eigen::Vector7d Vector7;
        typedef Eigen::Matrix3d Matrix3;
        typedef boost::math::normal normal;

        DYNAMIC_GRAPH_ENTITY_DECL();

      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /* --- CONSTRUCTOR ---- */
        DcmEstimator(const std::string & name );

        void init(const double & dt, const std::string& urdfFile);
        void test_command(const int& test_int);
        /* --- SIGNALS --- */
        DECLARE_SIGNAL_IN(q,   dynamicgraph::Vector);

        DECLARE_SIGNAL_OUT(c,  dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(dc, dynamicgraph::Vector);

        /* --- COMMANDS --- */
        /* --- ENTITY INHERITANCE --- */
        virtual void display( std::ostream& os ) const;

        void sendMsg(const std::string& msg, MsgType t=MSG_TYPE_INFO, const char* file="", int line=0)
        {
          getLogger().sendMsg("[DcmEstimator-"+name+"] "+msg, t, file, line);
        }

      protected:
        bool      m_initSucceeded;            /// true if the entity has been successfully initialized
        RobotUtil*   m_robot_util;
        se3::Data         *m_data;            /// Pinocchio robot data
        Eigen::VectorXd   m_q_pin;            /// robot configuration according to pinocchio convention
        Eigen::VectorXd   m_v_pin;            /// robot velocities according to pinocchio convention
        Vector3        m_last_vel;
        double               m_dt;            /// sampling time step
        se3::Model        m_model;            /// Pinocchio robot model

      }; // class DCMEstimator

    }    // namespace talos_balance
  }      // namespace sot
}        // namespace dynamicgraph



#endif // #ifndef __sot_talos_balance_dcm_estimator_H__
