/*
 * Copyright 2019
 * LAAS-CNRS
 * F. Bailly
 * T. Flayols
 * F. Risbourg
 */


#include <sot/talos_balance/ft-wrist-calibration.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>


#include <dynamic-graph/all-commands.h>
#include <sot/talos_balance/utils/stop-watch.hh>
#include <sot/talos_balance/utils/statistics.hh>

#define CALIB_ITER_TIME 1000 //Iteration needed for sampling and averaging the FT sensors while calibrating

using namespace sot::talos_balance;

namespace dynamicgraph
{
  namespace sot
  {
    namespace talos_balance
    {
      namespace dynamicgraph = ::dynamicgraph;
      using namespace dynamicgraph;
      using namespace dynamicgraph::command;
      using namespace dg::sot::talos_balance;

#define INPUT_SIGNALS  m_rightWristForceInSIN   << m_leftWristForceInSIN
#define OUTPUT_SIGNALS m_rightWristForceOutSOUT << m_leftWristForceOutSOUT

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef FtWristCalibration EntityClassName;

      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FtWristCalibration,
                                         "FtWristCalibration");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      FtWristCalibration::
      FtWristCalibration(const std::string& name)
        : Entity(name)
        , CONSTRUCT_SIGNAL_IN(rightWristForceIn,  dynamicgraph::Vector)
        , CONSTRUCT_SIGNAL_IN(leftWristForceIn,   dynamicgraph::Vector)
        , CONSTRUCT_SIGNAL_OUT(rightWristForceOut, dynamicgraph::Vector, m_rightWristForceInSIN)
        , CONSTRUCT_SIGNAL_OUT(leftWristForceOut,  dynamicgraph::Vector, m_leftWristForceInSIN)
        ,m_robot_util(RefVoidRobotUtil())
        ,m_initSucceeded(false)  
      {

        Entity::signalRegistration( INPUT_SIGNALS << OUTPUT_SIGNALS);

        /* Commands. */
         addCommand("init",
                   makeCommandVoid1(*this, &FtWristCalibration::init,
                                    docCommandVoid1("Initialize the entity.",
                                                    "Robot reference (string)")));
        addCommand("setRightHandWeight",
                   makeCommandVoid1(*this, &FtWristCalibration::setRightHandWeight,
                                    docCommandVoid1("Set the weight of the right hand after the sensor",
                                                    "Vector of default forces in Newton")));
        addCommand("setLeftHandWeight",
                   makeCommandVoid1(*this, &FtWristCalibration::setLeftHandWeight,
                            docCommandVoid1("Set the weight of the left hand after the sensor",
                                            "Vector of default forces in Newton")));
        addCommand("calibrateWristSensor",
                    makeCommandVoid0(*this, &FtWristCalibration::calibrateWristSensor,
                            docCommandVoid0("Calibrate the wrist sensors")));

      }

      void FtWristCalibration::init(const std::string &robotRef)
      {
        dgADD_OSTREAM_TO_RTLOG (std::cout);
        std::string localName(robotRef);
        m_initSucceeded = true;
        if (!isNameInRobotUtil(localName))
        {
            m_robot_util = createRobotUtil(localName);
        }
        else
        {
            m_robot_util = getRobotUtil(localName);
        }
        m_right_FT_offset                 << 0,0,0,0,0,0;
        m_left_FT_offset                  << 0,0,0,0,0,0;
        m_right_FT_offset_calibration_sum << 0,0,0,0,0,0;
        m_left_FT_offset_calibration_sum  << 0,0,0,0,0,0;
        SEND_MSG("Entity Initialized",MSG_TYPE_INFO);
      }


      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      DEFINE_SIGNAL_OUT_FUNCTION(rightWristForceOut, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal sum before initialization!");
          return s;
        }
        const Vector & rightWristForce = m_rightWristForceInSIN(iter);
        assert(rightWristForce.size() == 6  && "Unexpected size of signal rightWristForceIn, should be 6.");
        
        //do offset calibration if needed
        if (m_rightCalibrationIter > 0)
        {
			   m_right_FT_offset_calibration_sum += rightWristForce ;
			   m_rightCalibrationIter--;
	    	}
		    else if (m_rightCalibrationIter == 0)
		    {
         SEND_INFO_STREAM_MSG("Calibrating ft sensors...");  
			   m_right_FT_offset = m_right_FT_offset_calibration_sum / CALIB_ITER_TIME ;
         std::cout <<  m_rightHandWeight + m_right_FT_offset << std::endl;
         m_rightCalibrationIter--;
		    }
		
		    //remove offset and hand weight
		    s = rightWristForce - m_rightHandWeight - m_right_FT_offset;
        return s;
      }
      
      DEFINE_SIGNAL_OUT_FUNCTION(leftWristForceOut,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal sum before initialization!");
          return s;
        }
        const Vector & leftWristForce = m_leftWristForceInSIN(iter);
        assert(leftWristForce.size() == 6  && "Unexpected size of signal leftWristForceIn, should be 6.");
        
        //do offset calibration if needed
        if (m_leftCalibrationIter > 0)
        {
			   m_left_FT_offset_calibration_sum += leftWristForce;
			   m_leftCalibrationIter--;
	    	}
		    else if (m_leftCalibrationIter == 0)
	    	{
			   m_left_FT_offset = m_left_FT_offset_calibration_sum / CALIB_ITER_TIME ; 
         m_leftCalibrationIter--;
		    }
	    	//remove offset and hand weight
		    s = leftWristForce - m_leftHandWeight - m_left_FT_offset;
        return s;
      }
      /* --- COMMANDS ---------------------------------------------------------- */

      void FtWristCalibration::setRightHandWeight(const Vector &rightW)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot set right hand weight before initialization!");
          return;
        }
        m_rightHandWeight << rightW[0],rightW[1],rightW[2],0,0,0;
      }

      void FtWristCalibration::setLeftHandWeight(const Vector &leftW)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot set left hand weight before initialization!");
          return;
        }
        m_leftHandWeight << leftW[0],leftW[1],leftW[2],0,0,0;
      }
      
      void FtWristCalibration::calibrateWristSensor()
      {
		    SEND_WARNING_STREAM_MSG("Sampling FT sensor for offset calibration... Robot should be in the air, with horizontal hand.");
        m_rightCalibrationIter = CALIB_ITER_TIME;
        m_leftCalibrationIter = CALIB_ITER_TIME;
        m_right_FT_offset_calibration_sum << 0,0,0,0,0,0; 
        m_left_FT_offset_calibration_sum  << 0,0,0,0,0,0; 
      }
      
      /* --- PROTECTED MEMBER METHODS ---------------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */


      void FtWristCalibration::display(std::ostream& os) const
      {
        os << "FtWristCalibration "<<getName();
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }
      
    } // namespace talos_balance
  } // namespace sot
} // namespace dynamicgraph
