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

#ifndef __sot_talos_balance_simple_distribute_wrench_H__
#define __sot_talos_balance_simple_distribute_wrench_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (position_controller_EXPORTS)
#    define SIMPLE_DISTRIBUTE_WRENCH_EXPORT __declspec(dllexport)
#  else
#    define SIMPLE_DISTRIBUTE_WRENCH_EXPORT __declspec(dllimport)
#  endif
#else
#  define SIMPLE_DISTRIBUTE_WRENCH_EXPORT
#endif


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic-graph/signal-helper.h>

#include <map>
#include "boost/assign.hpp"
#include <sot/core/robot-utils.hh>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace dynamicgraph {
  namespace sot {
    namespace talos_balance {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */

      class SIMPLE_DISTRIBUTE_WRENCH_EXPORT SimpleDistributeWrench
	                         : public ::dynamicgraph::Entity
      {
        DYNAMIC_GRAPH_ENTITY_DECL();

      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /* --- CONSTRUCTOR ---- */
        SimpleDistributeWrench( const std::string & name );

        void init(const std::string& robotName);

        /* --- SIGNALS --- */
        DECLARE_SIGNAL_IN(wrenchDes,  dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(q,  dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(rho, double);

        DECLARE_SIGNAL_INNER(kinematics_computations, int);
        DECLARE_SIGNAL_INNER(wrenches,  int);

        DECLARE_SIGNAL_OUT(wrenchLeft, dynamicgraph::Vector);
//        DECLARE_SIGNAL_OUT(copLeft, dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(wrenchRight, dynamicgraph::Vector);
//        DECLARE_SIGNAL_OUT(copRight, dynamicgraph::Vector);

        DECLARE_SIGNAL_OUT(wrenchRef, dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(zmpRef, dynamicgraph::Vector);

        /* --- COMMANDS --- */
        /* --- ENTITY INHERITANCE --- */
        virtual void display( std::ostream& os ) const;

//        dynamicgraph::Vector computeCoP(const dynamicgraph::Vector & wrench, const MatrixHomogeneous & pose) const;

      protected:
        bool  m_initSucceeded;    /// true if the entity has been successfully initialized
        pinocchio::Model m_model;       /// Pinocchio robot model
        pinocchio::Data  m_data;        /// Pinocchio robot data
        RobotUtilShrPtr  m_robot_util;

        dynamicgraph::Vector m_wrenchLeft;
        dynamicgraph::Vector m_wrenchRight;
      }; // class SimpleDistributeWrench

    }    // namespace talos_balance
  }      // namespace sot
}        // namespace dynamicgraph



#endif // #ifndef __sot_talos_balance_simple_distribute_wrench_H__