/*
 * Copyright 2019
 * LAAS-CNRS
 * F. Bailly
 * T. Flayol
 * F. Risbourg
 */

#ifndef __sot_talos_balance_ft_wrist_calibration_H__
#define __sot_talos_balance_ft_wrist_calibration_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (__sot_talos_balance_ft_wrist_calibration_H__)
#    define SOTFTWRISTCALIBRATION_EXPORT __declspec(dllexport)
#  else
#    define SOTFTWRISTCALIBRATION_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTFTWRISTCALIBRATION_EXPORT
#endif


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic-graph/signal-helper.h>
#include <dynamic-graph/real-time-logger.h>
#include <sot/core/matrix-geometry.hh>
#include <sot/core/robot-utils.hh>
#include <map>
#include "boost/assign.hpp"


#include <sot/talos_balance/robot/robot-wrapper.hh>

namespace dynamicgraph {
  namespace sot {
    namespace talos_balance {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */

      class SOTFTWRISTCALIBRATION_EXPORT FtWristCalibration
                     :public::dynamicgraph::Entity
      {
        //typedef FtCalibration EntityClassName;
        DYNAMIC_GRAPH_ENTITY_DECL();

      public:
        /* --- CONSTRUCTOR ---- */
        FtWristCalibration( const std::string & name);
        /// Initialize
        void init(const std::string & robotRef);

        /* --- SIGNALS --- */
        DECLARE_SIGNAL_IN(rightWristForceIn,  dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(leftWristForceIn,   dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(rightWristForceOut, dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(leftWristForceOut, dynamicgraph::Vector);

        /* --- COMMANDS --- */

        /// Commands for setting the hand weight
        void setRightHandWeight(const Vector &rightW);
        void setLeftHandWeight(const  Vector  &leftW);
        
        /// Command to calibrate the wrist sensors when the robot is in half sitting with the hands aligned
        void calibrateWristSensor();

        void displayRobotUtil();

        /* --- ENTITY INHERITANCE --- */
        virtual void display( std::ostream& os ) const;
        
        /* --- TYPEDEFS ---- */
        typedef Eigen::Matrix<double, 6, 1>    Vector6d;

      protected:
        RobotUtilShrPtr m_robot_util;
        int m_rightCalibrationIter = -2; /// Number of iteration righ for calibration (-2= not calibrated, -1=caibration done)
        int m_leftCalibrationIter =  -2; /// Number of iteration left for calibration (-2= not cailbrated, -1=caibration done)
        Vector6d m_right_FT_offset; /// Offset or bias to be removed from Right FT sensor
        Vector6d m_left_FT_offset;  /// Offset or bias to be removed from Left FT sensor
        Vector6d m_right_FT_offset_calibration_sum;  /// Variable used durring average computation of the offset
        Vector6d m_left_FT_offset_calibration_sum;  /// Variable used durring average computation of the offset
        bool    m_initSucceeded;    /// true if the entity has been successfully initialized
        Vector6d m_rightHandWeight;  // weight of the right hand after the ft sensor
        Vector6d m_leftHandWeight;   // weight of the left hand after the ft sensor

      }; // class FtWristCalibration

    }    // namespace talos_balance
  }      // namespace sot
}        // namespace dynamicgraph



#endif // #ifndef __sot_talos_balance_ft_wrist_calibration_H__
