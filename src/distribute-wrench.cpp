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

#include "sot/talos_balance/distribute-wrench.hh"

#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-bind.h>

#include "sot/talos_balance/utils/commands-helper.hh"
#include "sot/talos_balance/utils/stop-watch.hh"

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
#define PROFILE_DISTRIBUTE_WRENCH_KINEMATICS_COMPUTATIONS "DistributeWrench: kinematics computations              "
#define PROFILE_DISTRIBUTE_WRENCH_QP_COMPUTATIONS         "DistributeWrench: QP problem computations              "

#define INPUT_SIGNALS     m_wrenchDesSIN << m_qSIN

#define INNER_SIGNALS m_kinematics_computations << m_qp_computations

//#define OUTPUT_SIGNALS m_wrenchLeftSOUT << m_copLeftSOUT << m_wrenchRightSOUT << m_copRightSOUT << m_wrenchRefSOUT << m_zmpRefSOUT << m_emergencyStopSOUT
#define OUTPUT_SIGNALS m_wrenchLeftSOUT << m_wrenchRightSOUT << m_wrenchRefSOUT << m_zmpRefSOUT << m_emergencyStopSOUT

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef DistributeWrench EntityClassName;

      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(DistributeWrench,
                                         "DistributeWrench");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      DistributeWrench::DistributeWrench(const std::string& name)
                      : Entity(name)
                      , CONSTRUCT_SIGNAL_IN(wrenchDes, dynamicgraph::Vector)
                      , CONSTRUCT_SIGNAL_IN(q, dynamicgraph::Vector)
                      , CONSTRUCT_SIGNAL_INNER(kinematics_computations, int, m_qSIN)
                      , CONSTRUCT_SIGNAL_INNER(qp_computations, int, m_wrenchDesSIN << m_kinematics_computationsSINNER)
                      , CONSTRUCT_SIGNAL_OUT(wrenchLeft, dynamicgraph::Vector, m_qp_computationsSINNER)
//                      , CONSTRUCT_SIGNAL_OUT(copLeft, dynamicgraph::Vector, m_wrenchLeftSOUT)
                      , CONSTRUCT_SIGNAL_OUT(wrenchRight, dynamicgraph::Vector, m_qp_computationsSINNER)
//                      , CONSTRUCT_SIGNAL_OUT(copRight, dynamicgraph::Vector, m_wrenchRightSOUT)
                      , CONSTRUCT_SIGNAL_OUT(wrenchRef, dynamicgraph::Vector, m_wrenchLeftSOUT << m_wrenchRightSOUT)
                      , CONSTRUCT_SIGNAL_OUT(zmpRef, dynamicgraph::Vector, m_wrenchRefSOUT)
                      , CONSTRUCT_SIGNAL_OUT(emergencyStop, bool, m_zmpRefSOUT)
                      , m_initSucceeded(false)
                      , m_model()
                      , m_data(pinocchio::Model())
      {
        Entity::signalRegistration( INPUT_SIGNALS << OUTPUT_SIGNALS );

        /* Commands. */
        addCommand("init", makeCommandVoid1(*this, &DistributeWrench::init, docCommandVoid1("Initialize the entity.","Robot name")));
      }

      void DistributeWrench::init(const std::string& robotName)
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

        // TODO: initialize m_qp1

        m_qp2.problem(12,6,0);

        m_initSucceeded = true;
      }

/*
      dynamicgraph::Vector
      DistributeWrench::computeCoP(const dg::Vector & wrench, const MatrixHomogeneous & pose) const
      {
        const double h = pose(2,3);

        const double fx = wrench[0];
        const double fy = wrench[1];
        const double fz = wrench[2];
        const double tx = wrench[3];
        const double ty = wrench[4];

        double m_eps = 0.1; // temporary

        double px, py;
        if(fz >= m_eps/2)
        {
          px = (- ty - fx*h)/fz;
          py = (  tx - fy*h)/fz;
        }
        else
        {
          px = 0.0;
          py = 0.0;
        }
        const double pz = - h;

        dg::Vector copLocal(3);
        copLocal[0] = px;
        copLocal[1] = py;
        copLocal[2] = pz;

        dg::Vector cop = pose.linear()*copLocal + pose.translation();

        return cop;
      }
*/

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

        getProfiler().start(PROFILE_DISTRIBUTE_WRENCH_KINEMATICS_COMPUTATIONS);

        /* Compute kinematics */
//        m_q_pin.head<6>().setZero();
//        m_q_pin(6) = 1.0;
        pinocchio::framesForwardKinematics(m_model, m_data, q);

        getProfiler().stop(PROFILE_DISTRIBUTE_WRENCH_KINEMATICS_COMPUTATIONS);

        return s;
      }

      DEFINE_SIGNAL_INNER_FUNCTION(qp_computations, int)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal qp_computations before initialization!");
          return s;
        }

        const Eigen::VectorXd & wrenchDes = m_wrenchDesSIN(iter);
        m_kinematics_computationsSINNER(iter);
        assert(wrenchDes.size()==6     && "Unexpected size of signal q");

        getProfiler().start(PROFILE_DISTRIBUTE_WRENCH_QP_COMPUTATIONS);

        // stub
        Eigen::MatrixXd Q(12,12);
        Q.setIdentity();

        Eigen::VectorXd C(12);
        C.setZero();

        Eigen::MatrixXd Aeq(6,12);
        Aeq.block<6,6>(0,0).setIdentity();
        Aeq.block<6,6>(0,6).setIdentity();

        Eigen::VectorXd Beq(6);
        Beq = wrenchDes;

        Eigen::MatrixXd Aineq(0,12);

        Eigen::VectorXd Bineq(0);

        bool success = m_qp2.solve(Q, C, Aeq, Beq, Aineq, Bineq);

        m_emergency_stop_triggered = !success;

        if(!success)
        {
          SEND_WARNING_STREAM_MSG("No solution to the QP problem!");
          return s;
        }

        Eigen::VectorXd result = m_qp2.result();

        m_wrenchLeft  = result.head<6>();
        m_wrenchRight = result.tail<6>();

        getProfiler().stop(PROFILE_DISTRIBUTE_WRENCH_QP_COMPUTATIONS);

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(wrenchLeft, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal wrenchLeft before initialization!");
          return s;
        }

        m_qp_computationsSINNER(iter);
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

        m_qp_computationsSINNER(iter);
        s = m_wrenchRight;
        return s;
      }

/*
      DEFINE_SIGNAL_OUT_FUNCTION(copLeft, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal copLeft before initialization!");
          return s;
        }

        const Eigen::VectorXd & wrenchLeft  = m_wrenchLeftSOUT(iter);

        // stub
        const MatrixHomogeneous & pose = MatrixHomogeneous::Identity();

        s = computeCoP(wrenchLeft,pose);

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(copRight, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal copRight before initialization!");
          return s;
        }

        const Eigen::VectorXd & wrenchRight  = m_wrenchRightSOUT(iter);

        // stub
        const MatrixHomogeneous & pose = MatrixHomogeneous::Identity();

        s = computeCoP(wrenchRight,pose);

        return s;
      }
*/

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

        //const double fx = wrenchRef[0];
        //const double fy = wrenchRef[1];
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

      DEFINE_SIGNAL_OUT_FUNCTION(emergencyStop, bool)
      {
        const dynamicgraph::Vector & zmp = m_zmpRefSOUT(iter); // dummy to trigger zmp computation
        (void) zmp;                                            // disable unused variable warning
        s = m_emergency_stop_triggered;
        return s;
      }

      /* --- COMMANDS ---------------------------------------------------------- */

      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      void DistributeWrench::display(std::ostream& os) const
      {
        os << "DistributeWrench " << getName();
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }

    } // namespace talos_balance
  } // namespace sot
} // namespace dynamicgraph

