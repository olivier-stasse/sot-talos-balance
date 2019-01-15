/*
 * Copyright 2019, 
 * LAAS-CNRS
 * François Bailly, 
 *
 * This file is part of sot-talos-balance.
 * See license file.
 */

#include "sot/talos_balance/dcm-estimator.hh"
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include "sot/talos_balance/utils/commands-helper.hh"
#include <sot/talos_balance/utils/stop-watch.hh>
#include "pinocchio/algorithm/frames.hpp"

namespace dynamicgraph
{
  namespace sot
  {
    namespace talos_balance
    {
      namespace dg = ::dynamicgraph;
      using namespace dg;
      using namespace dg::command;
      using namespace std;
      using namespace se3;
      using boost::math::normal; // typedef provides default type is double.
 //Size to be aligned                         "-------------------------------------------------------"
     
#define PROFILE_BASE_POSITION_ESTIMATION      "base-est position estimation"
#define PROFILE_BASE_VELOCITY_ESTIMATION      "base-est velocity estimation"
#define PROFILE_BASE_KINEMATICS_COMPUTATION   "base-est kinematics computation"

#define INPUT_SIGNALS     m_qSIN
#define OUTPUT_SIGNALS    m_cSOUT << m_dcSOUT

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef DcmEstimator EntityClassName;
      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(DcmEstimator,
                                         "DcmEstimator");      
      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      DcmEstimator::DcmEstimator(const std::string& name)
        : Entity(name)
        ,CONSTRUCT_SIGNAL_IN( q,   dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_OUT(c,   dynamicgraph::Vector, m_qSIN)
        ,CONSTRUCT_SIGNAL_OUT(dc,  dynamicgraph::Vector, m_qSIN)

      {
        Entity::signalRegistration( INPUT_SIGNALS << OUTPUT_SIGNALS );

        /* Commands. */
        addCommand("init",
                   makeCommandVoid2(*this, &DcmEstimator::init,
                                    docCommandVoid2("Initialize the entity.",
                                                    "time step (double)",
                                                    "URDF file path (string)")));

        addCommand("test_command",
                           makeCommandVoid1(*this, &DcmEstimator::test_command,
                                            docCommandVoid1("Test dumb command",
                                                            "integer (int)")));
      }
     void DcmEstimator::init(const double & dt, const std::string& robotRef)
      {
        m_dt = dt;
        try
        {
          // Retrieve m_robot_util informations 
          std::string localName(robotRef);
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

          se3::urdf::buildModel(m_robot_util->m_urdf_filename,
                                se3::JointModelFreeFlyer(), m_model);

          assert(m_model.existFrame(m_robot_util->m_foot_util.m_Left_Foot_Frame_Name));
          assert(m_model.existFrame(m_robot_util->m_foot_util.m_Right_Foot_Frame_Name));
          assert(m_model.existFrame(m_robot_util->m_foot_util.m_Right_Foot_Frame_Name));
          m_q_pin.setZero(m_model.nq);
          m_q_pin[6]= 1.; // for quaternion
          m_v_pin.setZero(m_robot_util->m_nbJoints+6);

          m_last_vel.setZero(); // ?
        }
        catch (const std::exception& e)
        {
          std::cout << e.what();
          SEND_MSG("Init failed: Could load URDF :" + m_robot_util->m_urdf_filename, MSG_TYPE_ERROR);
          return;
        }
        m_data = new se3::Data(m_model);
        m_initSucceeded = true;
      }
    
     void DcmEstimator::test_command(const int& test_int)     
     {
        std::cout << "test integer:" << test_int << std::endl;
     }
      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      DEFINE_SIGNAL_OUT_FUNCTION(c, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal com before initialization!");
          return s;
        }

        s = m_q_pin;
        return s;
      }
      
      DEFINE_SIGNAL_OUT_FUNCTION(dc, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal dcom before initialization!");
          return s;
        }

        s = m_q_pin;
        return s;
      }
      void DcmEstimator::display(std::ostream& os) const
      {
        os << "DcmEstimator " << getName();
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }
    } // namespace talos_balance
  } // namespace sot
} // namespace dynamicgraph