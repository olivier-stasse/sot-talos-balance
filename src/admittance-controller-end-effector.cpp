/*
 * Copyright 2019
 *
 * LAAS-CNRS
 *
 * Fanny Risbourg
 * This file is part of sot-talos-balance.
 * See license file.
 */

#include "sot/talos_balance/admittance-controller-end-effector.hh"
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>
#include "sot/talos_balance/utils/stop-watch.hh"

namespace dynamicgraph
{
  namespace sot
  {
    namespace talos_balance
    {
      namespace dg = ::dynamicgraph;
      using namespace dg;
      using namespace pinocchio;
      using namespace dg::command;

//Size to be aligned                                   "-------------------------------------------------------"

#define PROFILE_ADMITTANCECONTROLLERENDEFFECTOR_DQREF_COMPUTATION "AdmittanceControllerEndEffector: dqRef computation                "

#define PROFILE_ADMITTANCECONTROLLERENDEFFECTOR_FORCEWORLDFRAME_COMPUTATION "AdmittanceControllerEndEffector: forceWorldFrame computation          "

#define INPUT_SIGNALS     m_KpSIN << m_velocitySaturationSIN << m_forceSIN << m_forceDesSIN << m_jointPositionSIN

#define INNER_SIGNALS     m_forceWorldFrameSINNER

#define OUTPUT_SIGNALS    m_dqRefSOUT

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef AdmittanceControllerEndEffector EntityClassName;

      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(AdmittanceControllerEndEffector,
                                         "AdmittanceControllerEndEffector");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      AdmittanceControllerEndEffector::AdmittanceControllerEndEffector(const std::string& name)
          : Entity(name)
          , CONSTRUCT_SIGNAL_IN(Kp, dynamicgraph::Vector)
          , CONSTRUCT_SIGNAL_IN(velocitySaturation, dynamicgraph::Vector)
          , CONSTRUCT_SIGNAL_IN(force, dynamicgraph::Vector)
          , CONSTRUCT_SIGNAL_IN(forceDes, dynamicgraph::Vector)
          , CONSTRUCT_SIGNAL_IN(jointPosition, dynamicgraph::Vector)
          , CONSTRUCT_SIGNAL_OUT(dqRef, dynamicgraph::Vector, INPUT_SIGNALS << INNER_SIGNALS)
          , CONSTRUCT_SIGNAL_INNER(forceWorldFrame, dynamicgraph::Vector, m_forceSIN)
          , m_initSucceeded(false)
          , m_robot_util()
          , m_model()
          , m_sensorFrame()
          , m_parentId()
      {
        Entity::signalRegistration( INPUT_SIGNALS << INNER_SIGNALS << OUTPUT_SIGNALS );

        /* Commands. */
        addCommand("init", makeCommandVoid2(*this, &AdmittanceControllerEndEffector::init,
                                            docCommandVoid2("Initialize the entity.","time step", "sensor frame name")));
        addCommand("reset_dq", makeCommandVoid0(*this, &AdmittanceControllerEndEffector::reset_dq,
                                                docCommandVoid0("reset_dq")));
      }

      void AdmittanceControllerEndEffector::init(const double & dt, const std::string & sensorFrameName)
      {
        if(!m_KpSIN.isPlugged())
          return SEND_MSG("Init failed: signal velocitySaturation is not plugged", MSG_TYPE_ERROR);
        if(!m_velocitySaturationSIN.isPlugged())
          return SEND_MSG("Init failed: signal Kp is not plugged", MSG_TYPE_ERROR);
        if(!m_forceSIN.isPlugged())
          return SEND_MSG("Init failed: signal force is not plugged", MSG_TYPE_ERROR);
        if(!m_forceDesSIN.isPlugged())
          return SEND_MSG("Init failed: signal forceDes is not plugged", MSG_TYPE_ERROR);
        if(!m_jointPositionSIN.isPlugged())
          return SEND_MSG("Init failed: signal jointPosition is not plugged", MSG_TYPE_ERROR);

        m_n = 6;
        m_dt = dt;
        m_dq.setZero(m_n);

        try
        {
          /* Retrieve m_robot_util informations */
          std::string localName("robot");
          if (isNameInRobotUtil(localName))
          {
            m_robot_util = getRobotUtil(localName);
            std::cerr << "m_robot_util:" << m_robot_util << std::endl;
          }
          else
          {
            SEND_MSG("You should have a robotUtil pointer initialized before",MSG_TYPE_ERROR);
            return;
          }

          pinocchio::urdf::buildModel(m_robot_util->m_urdf_filename, pinocchio::JointModelFreeFlyer(), m_model);
          m_data = new pinocchio::Data(m_model);
          m_q.setZero(m_model.nq);

          pinocchio::FrameIndex sensorFrameId = m_model.getFrameId(sensorFrameName);
          assert(m_model.existFrame(sensorFrameId));

          m_parentId = m_model.frames[sensorFrameId].parent;
          m_sensorFrame = m_model.frames[sensorFrameId].placement;
        }
        catch (const std::exception& e)
        {
          std::cout << e.what();
          SEND_MSG("Init failed: Could load URDF :" + m_robot_util->m_urdf_filename, MSG_TYPE_ERROR);
          return;
        }

        m_initSucceeded = true;
      }

      void AdmittanceControllerEndEffector::reset_dq()
      {
        m_dq.setZero(m_n);
        return;
      }

      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      DEFINE_SIGNAL_INNER_FUNCTION(forceWorldFrame, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal forceWorldFrame before initialization!");
          return s;
        }

        getProfiler().start(PROFILE_ADMITTANCECONTROLLERENDEFFECTOR_FORCEWORLDFRAME_COMPUTATION);

        const Vector & forceVector = m_forceSIN(iter);
        const Vector & q = m_jointPositionSIN(iter);

        assert(forceVector.size()==m_n            && "Unexpected size of signal force");
        assert(q.size()==static_cast<dg::Index>(m_robot_util->m_nbJoints) 
			&& "Unexpected size of signal joint_positions");

        // Fill m_q with the current joint configuration and compute forward kinematics
        m_q.head<6>().setZero();
        m_q[6]= 1.;
        m_robot_util->joints_sot_to_urdf(q, m_q.tail(m_robot_util->m_nbJoints));
        pinocchio::forwardKinematics(m_model, *m_data, m_q);

        // Compute sensorPlacement
        pinocchio::SE3 parentPlacement = m_data->oMi[m_parentId];
        pinocchio::SE3 sensorPlacement =  parentPlacement * m_sensorFrame;

        pinocchio::Force force = pinocchio::Force(forceVector);
        pinocchio::Force forceWorldFrame = sensorPlacement.act(force);
        const Vector & forceWorldFrameVector = forceWorldFrame.toVector();

        s = forceWorldFrameVector;

        getProfiler().stop(PROFILE_ADMITTANCECONTROLLERENDEFFECTOR_FORCEWORLDFRAME_COMPUTATION);

        return s;
      }


      DEFINE_SIGNAL_OUT_FUNCTION(dqRef, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal dqRef before initialization!");
          return s;
        }

        getProfiler().start(PROFILE_ADMITTANCECONTROLLERENDEFFECTOR_DQREF_COMPUTATION);

        const Vector & forceDes = m_forceDesSIN(iter);
        const Vector & forceWorldFrame = m_forceWorldFrameSINNER(iter);
        const Vector & Kp = m_KpSIN(iter);
        const Vector & velocitySaturation = m_velocitySaturationSIN(iter);

        assert(forceWorldFrame.size()==m_n       && "Unexpected size of signal force");
        assert(forceDes.size()==m_n              && "Unexpected size of signal forceDes");
        assert(Kp.size()==m_n                    && "Unexpected size of signal Kp");
        assert(velocitySaturation.size()==m_n    && "Unexpected size of signal velocitySaturation");


        m_dq += Kp.cwiseProduct(forceDes - forceWorldFrame);

	for(int i = 0; i < m_n; i++)
	{
          if(m_dq[i] > velocitySaturation[i])
          {
            m_dq[i] = velocitySaturation[i];
	  }
          if(m_dq[i] < -velocitySaturation[i])
          {
            m_dq[i] = -velocitySaturation[i];
	  }
	}
        
	s = m_dq;

        getProfiler().stop(PROFILE_ADMITTANCECONTROLLERENDEFFECTOR_DQREF_COMPUTATION);

        return s;
      }

      /* --- COMMANDS ------------s---------------------------------------------- */

      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      void AdmittanceControllerEndEffector::display(std::ostream& os) const
      {
        os << "AdmittanceControllerEndEffector " << getName();
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }
    } // namespace talos_balance
  } // namespace sot
} // namespace dynamicgraph

