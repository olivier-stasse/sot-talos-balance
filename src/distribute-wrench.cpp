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

#include <iostream>

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
#define PROFILE_DISTRIBUTE_WRENCH_KINEMATICS_COMPUTATIONS "DistributeWrench: kinematics computations              "
#define PROFILE_DISTRIBUTE_WRENCH_QP_COMPUTATIONS         "DistributeWrench: QP problem computations              "

#define INPUT_SIGNALS     m_wrenchDesSIN << m_qSIN << m_rhoSIN

#define INNER_SIGNALS m_kinematics_computations << m_qp_computations

#define OUTPUT_SIGNALS m_wrenchLeftSOUT << m_copLeftSOUT << m_wrenchRightSOUT << m_copRightSOUT << m_wrenchRefSOUT << m_zmpRefSOUT << m_emergencyStopSOUT

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
                      , CONSTRUCT_SIGNAL_IN(rho, double)
                      , CONSTRUCT_SIGNAL_INNER(kinematics_computations, int, m_qSIN)
                      , CONSTRUCT_SIGNAL_INNER(qp_computations, int, m_wrenchDesSIN << m_rhoSIN << m_kinematics_computationsSINNER)
                      , CONSTRUCT_SIGNAL_OUT(wrenchLeft, dynamicgraph::Vector, m_qp_computationsSINNER)
                      , CONSTRUCT_SIGNAL_OUT(copLeft, dynamicgraph::Vector, m_wrenchLeftSOUT)
                      , CONSTRUCT_SIGNAL_OUT(wrenchRight, dynamicgraph::Vector, m_qp_computationsSINNER)
                      , CONSTRUCT_SIGNAL_OUT(copRight, dynamicgraph::Vector, m_wrenchRightSOUT)
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

        addCommand("set_right_foot_sizes",
                   makeCommandVoid1(*this, &DistributeWrench::set_right_foot_sizes,
                                    docCommandVoid1("Set the size of the right foot (pos x, neg x, pos y, neg y)",
                                                    "4d vector")));
        addCommand("set_left_foot_sizes",
                   makeCommandVoid1(*this, &DistributeWrench::set_left_foot_sizes,
                                    docCommandVoid1("Set the size of the left foot (pos x, neg x, pos y, neg y)",
                                                    "4d vector")));
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


        assert(m_model.existFrame(m_robot_util->m_foot_util.m_Left_Foot_Frame_Name));
        assert(m_model.existFrame(m_robot_util->m_foot_util.m_Right_Foot_Frame_Name));
        m_left_foot_id       = m_model.getFrameId(m_robot_util->m_foot_util.m_Left_Foot_Frame_Name);
        m_right_foot_id      = m_model.getFrameId(m_robot_util->m_foot_util.m_Right_Foot_Frame_Name);

        m_ankle_M_ftSens = pinocchio::SE3(Eigen::Matrix3d::Identity(), m_robot_util->m_foot_util.m_Right_Foot_Force_Sensor_XYZ.head<3>());
//        m_ankle_M_sole   = pinocchio::SE3(Eigen::Matrix3d::Identity(), m_robot_util->m_foot_util.m_Right_Foot_Sole_XYZ.head<3>());

        // TODO: initialize m_qp1

        m_qp2.problem(12,0,0);

        m_initSucceeded = true;
      }

      void DistributeWrench::set_right_foot_sizes(const dynamicgraph::Vector & s)
      {
        if(s.size()!=4)
          return SEND_MSG("Foot size vector should have size 4, not "+toString(s.size()), MSG_TYPE_ERROR);
        m_right_foot_sizes = s;
      }

      void DistributeWrench::set_left_foot_sizes(const dynamicgraph::Vector & s)
      {
        if(s.size()!=4)
          return SEND_MSG("Foot size vector should have size 4, not "+toString(s.size()), MSG_TYPE_ERROR);
        m_left_foot_sizes = s;
      }

      dynamicgraph::Vector
      DistributeWrench::computeCoP(const dg::Vector & wrenchGlobal, const pinocchio::SE3 & pose) const
      {
//        std::cout << "++++++++++++" << std::endl;
//        std::cout << wrenchGlobal.transpose() << std::endl;
        dg::Vector wrench = pose.actInv(pinocchio::Force(wrenchGlobal)).toVector();
//        std::cout << wrench.transpose() << std::endl;
//        std::cout << "------------" << std::endl;

        const double h = pose.translation()[2];

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
        const double pz = 0.0;

        dg::Vector cop(3);
        cop[0] = px;
        cop[1] = py;
        cop[2] = pz;

        return cop;
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
        const double & rho = m_rhoSIN(iter);
        const int & dummy = m_kinematics_computationsSINNER(iter);
        (void) dummy;

        assert(wrenchDes.size()==6     && "Unexpected size of signal q");

        getProfiler().start(PROFILE_DISTRIBUTE_WRENCH_QP_COMPUTATIONS);

        // --- COSTS

        const double wSum = 10000.0; // TODO: signal/conf
        const double wNorm = 10.0; // TODO: signal/conf
        const double wRatio = 1.0; // TODO: signal/conf
        Eigen::VectorXd wAnkle(6);
        wAnkle << 1., 1., 1e-4, 1., 1., 1e-4; // TODO: signal/conf

        // Initialize cost matrices
        Eigen::MatrixXd Q(12,12);
        Eigen::VectorXd C(12);

        // min |wrenchLeft + wrenchRight - wrenchDes|^2
        Q.topLeftCorner<6,6>().setIdentity();
        Q.topRightCorner<6,6>().setIdentity();
        Q.bottomLeftCorner<6,6>().setIdentity();
        Q.bottomRightCorner<6,6>().setIdentity();
        Q *= wSum;

        C.head<6>() = -wrenchDes;
        C.tail<6>() = -wrenchDes;
        C *= wSum;

        // min |wrenchLeft_a|^2 + |wrenchRight_a|^2
        Eigen::MatrixXd tmp = wAnkle.asDiagonal() * m_data.oMf[m_left_foot_id].inverse().toDualActionMatrix();
        tmp = tmp.transpose() * tmp * wNorm;
        Q.topLeftCorner<6,6>() += tmp;

        tmp = wAnkle.asDiagonal() * m_data.oMf[m_right_foot_id].inverse().toDualActionMatrix();
        tmp = tmp.transpose() * tmp * wNorm;
        Q.bottomRightCorner<6,6>() += tmp;

        // min |(1-rho)e_z^T*wrenchLeft_c - rho*e_z^T*wrenchLeft_c|
        const pinocchio::SE3 leftPos  = m_data.oMf[m_left_foot_id]  * m_ankle_M_ftSens;
        const pinocchio::SE3 rightPos = m_data.oMf[m_right_foot_id] * m_ankle_M_ftSens;

        Eigen::MatrixXd tmp2(1,12);
        tmp2 << (1-rho) * (  leftPos.inverse().toDualActionMatrix().row(2) ),
                 (-rho) * ( rightPos.inverse().toDualActionMatrix().row(2) );

        Q += wRatio * tmp2.transpose()*tmp2;

        // --- Equality constraints

        Eigen::MatrixXd Aeq(0,12);

        Eigen::VectorXd Beq(0);

        // --- Inequality constraints

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

      DEFINE_SIGNAL_OUT_FUNCTION(copLeft, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal copLeft before initialization!");
          return s;
        }

        const Eigen::VectorXd & wrenchLeft  = m_wrenchLeftSOUT(iter);

        // stub
        const pinocchio::SE3 pose = m_data.oMf[m_left_foot_id] * m_ankle_M_ftSens;

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

        const pinocchio::SE3 pose = m_data.oMf[m_right_foot_id] * m_ankle_M_ftSens;

        s = computeCoP(wrenchRight,pose);

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

