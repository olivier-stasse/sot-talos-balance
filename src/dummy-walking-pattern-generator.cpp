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

#include "sot/talos_balance/dummy-walking-pattern-generator.hh"

#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-bind.h>

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
      using namespace dg::command;

//Size to be aligned                                         "-------------------------------------------------------"
#define PROFILE_DUMMYWALKINGPATTERNGENERATOR_DCM_COMPUTATION "DummyWalkingPatternGenerator: dcm computation          "

#define INPUT_SIGNALS     m_omegaSIN << m_footLeftSIN << m_footRightSIN << m_comSIN << m_vcomSIN << m_acomSIN

#define INNER_SIGNALS     m_referenceFrameSINNER

#define OUTPUT_SIGNALS m_comDesSOUT << m_vcomDesSOUT << m_acomDesSOUT << m_dcmDesSOUT << m_zmpDesSOUT << m_footLeftDesSOUT << m_footRightDesSOUT

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef DummyWalkingPatternGenerator EntityClassName;

      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(DummyWalkingPatternGenerator,
                                         "DummyWalkingPatternGenerator");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      DummyWalkingPatternGenerator::DummyWalkingPatternGenerator(const std::string& name)
                      : Entity(name)
                      , CONSTRUCT_SIGNAL_IN(omega, double)
                      , CONSTRUCT_SIGNAL_IN(footLeft,  MatrixHomogeneous)
                      , CONSTRUCT_SIGNAL_IN(footRight, MatrixHomogeneous)
                      , CONSTRUCT_SIGNAL_IN(com,  dynamicgraph::Vector)
                      , CONSTRUCT_SIGNAL_IN(vcom, dynamicgraph::Vector)
                      , CONSTRUCT_SIGNAL_IN(acom, dynamicgraph::Vector)
                      , CONSTRUCT_SIGNAL_INNER(referenceFrame, MatrixHomogeneous, m_footLeftSIN << m_footRightSIN)
                      , CONSTRUCT_SIGNAL_OUT(comDes,  dynamicgraph::Vector, m_comSIN  << m_referenceFrameSINNER)
                      , CONSTRUCT_SIGNAL_OUT(vcomDes, dynamicgraph::Vector, m_vcomSIN << m_referenceFrameSINNER)
                      , CONSTRUCT_SIGNAL_OUT(acomDes, dynamicgraph::Vector, m_acomSIN << m_referenceFrameSINNER)
                      , CONSTRUCT_SIGNAL_OUT(dcmDes,  dynamicgraph::Vector, m_omegaSIN << m_comDesSOUT << m_vcomDesSOUT)
                      , CONSTRUCT_SIGNAL_OUT(zmpDes,  dynamicgraph::Vector, m_omegaSIN << m_comDesSOUT << m_acomDesSOUT)
                      , CONSTRUCT_SIGNAL_OUT(footLeftDes,  MatrixHomogeneous, m_footLeftSIN << m_referenceFrameSINNER)
                      , CONSTRUCT_SIGNAL_OUT(footRightDes, MatrixHomogeneous, m_footRightSIN << m_referenceFrameSINNER)
                      , m_initSucceeded(false)
      {
        Entity::signalRegistration( INPUT_SIGNALS << INNER_SIGNALS << OUTPUT_SIGNALS );

        /* Commands. */
        addCommand("init", makeCommandVoid1(*this, &DummyWalkingPatternGenerator::init, docCommandVoid1("Initialize the entity.","Robot name")));
      }

      dynamicgraph::Vector DummyWalkingPatternGenerator::actInv(MatrixHomogeneous m, dynamicgraph::Vector v)
      {
        return m.linear().transpose()*(v-m.translation());
      }
      
      MatrixHomogeneous DummyWalkingPatternGenerator::actInv(MatrixHomogeneous m1, MatrixHomogeneous m2)
      {
        MatrixHomogeneous res;
        res.linear() = m1.linear().transpose()*m2.linear();
        res.translation() = m1.linear().transpose()*(m2.translation()-m1.translation());
        return res;
      }

      void DummyWalkingPatternGenerator::init(const std::string& robotName)
      {
        try
        {
          /* Retrieve m_robot_util informations */
          std::string localName(robotName);
          if (isNameInRobotUtil(localName))
          {
            m_robot_util = getRobotUtil(localName);
          }
          else
          {
            SEND_ERROR_STREAM_MSG("You should have a robotUtil pointer initialized before");
            return;
          }
        }
        catch (const std::exception& e)
        {
          SEND_ERROR_STREAM_MSG("Init failed: Could load URDF :" + m_robot_util->m_urdf_filename);
          return;
        }

        m_rightFootSoleXYZ = m_robot_util->m_foot_util.m_Right_Foot_Sole_XYZ;

        m_initSucceeded = true;
      }

      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      DEFINE_SIGNAL_INNER_FUNCTION(referenceFrame, MatrixHomogeneous)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal referenceFrame before initialization!");
          return s;
        }

        const MatrixHomogeneous & footLeft = m_footLeftSIN(iter);
        const MatrixHomogeneous & footRight = m_footRightSIN(iter);

        const Vector & centerTranslation = ( footLeft.translation() + footRight.translation() )/2 + m_rightFootSoleXYZ;

        MatrixHomogeneous referenceFrame;
        referenceFrame.linear() = footRight.linear();
        referenceFrame.translation() = centerTranslation;

        s = referenceFrame;

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(comDes, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal comDes before initialization!");
          return s;
        }

        const Vector & com = m_comSIN(iter);
        const MatrixHomogeneous & referenceFrame = m_referenceFrameSINNER(iter);

        assert( com.size()==3 && "Unexpected size of signal com" );

        const Vector comDes = actInv(referenceFrame, com);

        s = comDes;

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(vcomDes, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal vcomDes before initialization!");
          return s;
        }

        const Vector & vcom = m_vcomSIN(iter);
        const MatrixHomogeneous & referenceFrame = m_referenceFrameSINNER(iter);

        assert( vcom.size()==3 && "Unexpected size of signal vcom" );

        const Vector vcomDes(referenceFrame.linear().transpose()*vcom);

        s = vcomDes;

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(acomDes, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal acomDes before initialization!");
          return s;
        }

        const Vector & acom = m_acomSIN(iter);
        const MatrixHomogeneous & referenceFrame = m_referenceFrameSINNER(iter);

        assert( acom.size()==3 && "Unexpected size of signal acom" );

        const Vector acomDes(referenceFrame.linear().transpose()*acom);

        s = acomDes;

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(dcmDes, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal dcmDes before initialization!");
          return s;
        }

        const double & omega = m_omegaSIN(iter);
        const Vector & comDes = m_comDesSOUT(iter);
        const Vector & vcomDes = m_vcomDesSOUT(iter);

        const Vector dcmDes = comDes + vcomDes/omega;

        s = dcmDes;

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(zmpDes, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal zmpDes before initialization!");
          return s;
        }

        const double & omega = m_omegaSIN(iter);
        const Vector & comDes = m_comDesSOUT(iter);
        const Vector & acomDes = m_acomDesSOUT(iter);

        Vector zmpDes = comDes - acomDes/(omega*omega);
        zmpDes[2] = 0.0;

        s = zmpDes;

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(footLeftDes, MatrixHomogeneous)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal footLeftDes before initialization!");
          return s;
        }

        const MatrixHomogeneous & footLeft = m_footLeftSIN(iter);
        const MatrixHomogeneous & referenceFrame = m_referenceFrameSINNER(iter);

        s = actInv(referenceFrame, footLeft);

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(footRightDes, MatrixHomogeneous)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal footRightDes before initialization!");
          return s;
        }

        const MatrixHomogeneous & footRight = m_footRightSIN(iter);
        const MatrixHomogeneous & referenceFrame = m_referenceFrameSINNER(iter);

        s = actInv(referenceFrame, footRight);

        return s;
      }

      /* --- COMMANDS ---------------------------------------------------------- */

      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      void DummyWalkingPatternGenerator::display(std::ostream& os) const
      {
        os << "DummyWalkingPatternGenerator " << getName();
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }
    } // namespace talos_balance
  } // namespace sot
} // namespace dynamicgraph

