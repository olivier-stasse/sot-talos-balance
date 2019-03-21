/*
 * Copyright 2019
 * LAAS-CNRS
 * F. Bailly
 */

#ifndef __sot_torque_control_ft_calibration_H__
#define __sot_torque_control_ft_calibration_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (__sot_talos_balance_ft_calibration_H__)
#    define SOTFtCalibration_EXPORT __declspec(dllexport)
#  else
#    define SOTFtCalibration_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTPFtCalibration_EXPORT
#endif


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic-graph/signal-helper.h>
#include <sot/core/matrix-geometry.hh>
#include <sot/core/robot-utils.hh>
#include <map>
#include "boost/assign.hpp"


#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <sot/talos_balance/robot/robot-wrapper.hh>

namespace dynamicgraph {
  namespace sot {
    namespace talos_balance {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */

      class SOTFtCalibration_EXPORT FtCalibration
        :public::dynamicgraph::Entity
      {
        typedef FtCalibration EntityClassName;
        DYNAMIC_GRAPH_ENTITY_DECL();

      public:
        /* --- CONSTRUCTOR ---- */
        FtCalibration( const std::string & name);

	/// Initialize
        void init(const std::string & robotRef);

        /* --- SIGNALS --- */
        /* --- SIGNALS --- */
        DECLARE_SIGNAL_IN(force_in,  dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(force_out, dynamicgraph::Vector);

        /* --- COMMANDS --- */

	      /// Commands for setting the feet weight
	      void setRightFootWeight(const dynamicgraph::Vector &rightW);
	      void setLeftFootWeight(const dynamicgraph::Vector &leftW);

	      void displayRobotUtil();


        /* --- ENTITY INHERITANCE --- */
        virtual void display( std::ostream& os ) const;

      protected:
        RobotUtil *                   m_robot_util;
        bool    m_initSucceeded;    /// true if the entity has been successfully initialized
        dynamicgraph::Vector m_right_foot_weight  // weight of the right feet underneath the ft sensor
        dynamicgraph::Vector m_left_foot_weight   // weight of the left feet underneath the ft sensor

      }; // class FtCalibration

    }    // namespace talos_balance
  }      // namespace sot
}        // namespace dynamicgraph



#endif // #ifndef __sot_talos_balance_ft_calibration_H__
