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

#include "sot/talos_balance/simple-distribute-wrench.hh"

#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>

#include <sot/core/stop-watch.hh>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace dynamicgraph
{
  namespace sot
  {
    namespace talos_balance
    {
      namespace dg = ::dynamicgraph;
      using namespace dg;
      using namespace dg::command;

//Size to be aligned                                      "-------------------------------------------------------"
#define PROFILE_SIMPLE_DISTRIBUTE_WRENCH_KINEMATICS_COMPUTATIONS "SimpleDistributeWrench: kinematics computations              "
#define PROFILE_SIMPLE_DISTRIBUTE_WRENCH_WRENCHES_COMPUTATIONS   "SimpleDistributeWrench: wrenches computations                "

#define INPUT_SIGNALS     m_wrenchDesSIN << m_qSIN << m_rhoSIN

#define INNER_SIGNALS m_kinematics_computations << m_wrenches

#define OUTPUT_SIGNALS m_wrenchLeftSOUT << m_wrenchRightSOUT << m_wrenchRefSOUT << m_zmpRefSOUT

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef SimpleDistributeWrench EntityClassName;

      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SimpleDistributeWrench,
                                         "SimpleDistributeWrench");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      SimpleDistributeWrench::SimpleDistributeWrench(const std::string& name)
                      : Entity(name)
                      , CONSTRUCT_SIGNAL_IN(wrenchDes, dynamicgraph::Vector)
                      , CONSTRUCT_SIGNAL_IN(q, dynamicgraph::Vector)
                      , CONSTRUCT_SIGNAL_IN(rho, double)
                      , CONSTRUCT_SIGNAL_INNER(kinematics_computations, int, m_qSIN)
                      , CONSTRUCT_SIGNAL_INNER(wrenches, int, m_wrenchDesSIN << m_kinematics_computationsSINNER)
                      , CONSTRUCT_SIGNAL_OUT(wrenchLeft, dynamicgraph::Vector, m_wrenchesSINNER)
                      , CONSTRUCT_SIGNAL_OUT(wrenchRight, dynamicgraph::Vector, m_wrenchesSINNER)
                      , CONSTRUCT_SIGNAL_OUT(wrenchRef, dynamicgraph::Vector, m_wrenchLeftSOUT << m_wrenchRightSOUT)
                      , CONSTRUCT_SIGNAL_OUT(zmpRef, dynamicgraph::Vector, m_wrenchRefSOUT)
                      , m_initSucceeded(false)
                      , m_model()
                      , m_data(pinocchio::Model())
      {
        Entity::signalRegistration( INPUT_SIGNALS << OUTPUT_SIGNALS );

        /* Commands. */
        addCommand("init", makeCommandVoid1(*this, &SimpleDistributeWrench::init, docCommandVoid1("Initialize the entity.","Robot name")));
      }

      void SimpleDistributeWrench::init(const std::string& robotName)
      {
        if(!m_wrenchDesSIN.isPlugged())
          return SEND_MSG("Init failed: signal wrenchDes is not plugged", MSG_TYPE_ERROR);
        if(!m_qSIN.isPlugged())
          return SEND_MSG("Init failed: signal q is not plugged", MSG_TYPE_ERROR);

        try
        {
          /* Retrieve m_robot_util informations */
          std::string localName(robotName);
          if (isNameInRobotUtil(localName))
          {
            m_robot_util = getRobotUtil(localName);
//            std::cerr << "m_robot_util:" << m_robot_util << std::endl;
          }
          else
          {
            SEND_MSG("You should have a robotUtil pointer initialized before",MSG_TYPE_ERROR);
            return;
          }

          pinocchio::urdf::buildModel(m_robot_util->m_urdf_filename, pinocchio::JointModelFreeFlyer(), m_model);
          m_data = pinocchio::Data(m_model);
        }
        catch (const std::exception& e)
        {
          std::cout << e.what();
          SEND_MSG("Init failed: Could load URDF :" + m_robot_util->m_urdf_filename, MSG_TYPE_ERROR);
          return;
        }

        m_initSucceeded = true;
      }

      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      DEFINE_SIGNAL_INNER_FUNCTION(kinematics_computations, int)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal kinematics_computations before initialization!");
          return s;
        }

        const Eigen::VectorXd & q = m_qSIN(iter);
        assert(q.size()==m_model.nq     && "Unexpected size of signal q");

        getProfiler().start(PROFILE_SIMPLE_DISTRIBUTE_WRENCH_KINEMATICS_COMPUTATIONS);

        pinocchio::framesForwardKinematics(m_model, m_data, q);

        getProfiler().stop(PROFILE_SIMPLE_DISTRIBUTE_WRENCH_KINEMATICS_COMPUTATIONS);

        return s;
      }

      DEFINE_SIGNAL_INNER_FUNCTION(wrenches, int)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal wrenches before initialization!");
          return s;
        }

        const double rho_m = 0.1;
        const double rho_M = 1.0 - rho_m;

        const Eigen::VectorXd & wrenchDes = m_wrenchDesSIN(iter);

        double rho = m_rhoSIN.isPlugged() ? m_rhoSIN(iter) : 0.5;
        if(rho<rho_m)
          rho = rho_m;
        else if(rho>rho_M)
          rho = rho_M;

        m_kinematics_computationsSINNER(iter);

        assert(wrenchDes.size()==6     && "Unexpected size of signal wrenchDes");

        getProfiler().start(PROFILE_SIMPLE_DISTRIBUTE_WRENCH_WRENCHES_COMPUTATIONS);

        // stub
        m_wrenchLeft  = wrenchDes/2;
        m_wrenchRight = wrenchDes/2;

        m_wrenchLeft[2] = (1-rho) * wrenchDes[2];
        m_wrenchRight[2] = rho * wrenchDes[2];

        getProfiler().stop(PROFILE_SIMPLE_DISTRIBUTE_WRENCH_WRENCHES_COMPUTATIONS);

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(wrenchLeft, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal wrenchLeft before initialization!");
          return s;
        }

        m_wrenchesSINNER(iter);
        s = m_wrenchLeft;
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(wrenchRight, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal wrenchRight before initialization!");
          return s;
        }

        m_wrenchesSINNER(iter);
        s = m_wrenchRight;
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(wrenchRef, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal wrenchRef before initialization!");
          return s;
        }

        const Eigen::VectorXd & wrenchLeft  = m_wrenchLeftSOUT(iter);
        const Eigen::VectorXd & wrenchRight = m_wrenchRightSOUT(iter);

        s = wrenchLeft + wrenchRight;

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(zmpRef, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal zmpRef before initialization!");
          return s;
        }

        const Eigen::VectorXd & wrenchRef  = m_wrenchRefSOUT(iter);

        const double fx = wrenchRef[0];
        const double fy = wrenchRef[1];
        const double fz = wrenchRef[2];
        const double tx = wrenchRef[3];
        const double ty = wrenchRef[4];

        double m_eps = 0.1; // temporary

        double px, py;
        if(fz >= m_eps/2)
        {
          px = -ty/fz;
          py =  tx/fz;
        }
        else
        {
          px = 0.0;
          py = 0.0;
        }
        const double pz = 0.0;

        dg::Vector zmp(3);
        zmp[0] = px;
        zmp[1] = py;
        zmp[2] = pz;

        s = zmp;

        return s;
      }


      /* --- COMMANDS ---------------------------------------------------------- */

      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      void SimpleDistributeWrench::display(std::ostream& os) const
      {
        os << "SimpleDistributeWrench " << getName();
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }

    } // namespace talos_balance
  } // namespace sot
} // namespace dynamicgraph

