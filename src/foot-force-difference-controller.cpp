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

#include "sot/talos_balance/foot-force-difference-controller.hh"

#include <sot/core/debug.hh>
#include <sot/core/stop-watch.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/all-commands.h>

namespace dynamicgraph
{
  namespace sot
  {
    namespace talos_balance
    {
      namespace dg = ::dynamicgraph;
      using namespace dg;
      using namespace dg::command;

#define INPUT_SIGNALS  m_phaseSIN << m_dfzAdmittanceSIN << m_vdcFrequencySIN << m_vdcDampingSIN << m_wrenchRightDesSIN << m_wrenchLeftDesSIN << m_wrenchRightSIN << m_wrenchLeftSIN << m_posRightDesSIN << m_posLeftDesSIN << m_posRightSIN << m_posLeftSIN

#define INNER_SIGNALS m_dz_ctrlSOUT << m_dz_posSOUT

#define OUTPUT_SIGNALS m_vRightSOUT << m_vLeftSOUT

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef FootForceDifferenceController EntityClassName;

      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FootForceDifferenceController,
                                         "FootForceDifferenceController");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      FootForceDifferenceController::FootForceDifferenceController(const std::string& name)
          : Entity(name)
          , CONSTRUCT_SIGNAL_IN(phase, int)
          , CONSTRUCT_SIGNAL_IN(dfzAdmittance, double)
          , CONSTRUCT_SIGNAL_IN(vdcFrequency, double)
          , CONSTRUCT_SIGNAL_IN(vdcDamping, double)
          , CONSTRUCT_SIGNAL_IN(wrenchRightDes, dynamicgraph::Vector)
          , CONSTRUCT_SIGNAL_IN(wrenchLeftDes, dynamicgraph::Vector)
          , CONSTRUCT_SIGNAL_IN(wrenchRight, dynamicgraph::Vector)
          , CONSTRUCT_SIGNAL_IN(wrenchLeft, dynamicgraph::Vector)
          , CONSTRUCT_SIGNAL_IN(posRightDes, MatrixHomogeneous)
          , CONSTRUCT_SIGNAL_IN(posLeftDes, MatrixHomogeneous)
          , CONSTRUCT_SIGNAL_IN(posRight, MatrixHomogeneous)
          , CONSTRUCT_SIGNAL_IN(posLeft, MatrixHomogeneous)
          , CONSTRUCT_SIGNAL_INNER(dz_ctrl, double, m_dfzAdmittanceSIN << m_vdcDampingSIN << m_wrenchRightDesSIN << m_wrenchLeftDesSIN << m_wrenchRightSIN << m_wrenchLeftSIN << m_posRightSIN << m_posLeftSIN)
          , CONSTRUCT_SIGNAL_INNER(dz_pos, double, m_vdcFrequencySIN << m_posRightDesSIN << m_posLeftDesSIN << m_posRightSIN << m_posLeftSIN)
          , CONSTRUCT_SIGNAL_OUT(vRight, dynamicgraph::Vector, m_phaseSIN << m_dz_ctrlSINNER << m_dz_posSINNER)
          , CONSTRUCT_SIGNAL_OUT(vLeft,  dynamicgraph::Vector, m_phaseSIN << m_dz_ctrlSINNER << m_dz_posSINNER)
          , m_initSucceeded(false)
      {
        Entity::signalRegistration( INPUT_SIGNALS << OUTPUT_SIGNALS );

        /* Commands. */
        addCommand("init", makeCommandVoid0(*this, &FootForceDifferenceController::init, docCommandVoid0("Initialize the entity.")));
      }

      void FootForceDifferenceController::init()
      {
        m_initSucceeded = true;
      }

      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      DEFINE_SIGNAL_INNER_FUNCTION(dz_ctrl, double)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Can't compute dz_ctrl before initialization!");
          return s;
        }

        const double & dfzAdmittance = m_dfzAdmittanceSIN(iter);
        const double & vdcDamping = m_vdcDampingSIN(iter);

        const Eigen::VectorXd & wrenchRightDes = m_wrenchRightDesSIN(iter);
        const Eigen::VectorXd & wrenchLeftDes  = m_wrenchLeftDesSIN(iter);
        const Eigen::VectorXd & wrenchRight = m_wrenchRightSIN(iter);
        const Eigen::VectorXd & wrenchLeft  = m_wrenchLeftSIN(iter);

        const MatrixHomogeneous & posRight = m_posRightSIN(iter);
        const MatrixHomogeneous & posLeft  = m_posLeftSIN(iter);

        const double RTz = posRight.translation()[2];
        const double LTz = posLeft.translation()[2];

        const double RFz_d = wrenchRightDes[2];
        const double LFz_d = wrenchLeftDes[2];

        const double RFz = wrenchRight[2];
        const double LFz = wrenchLeft[2];

        const double dz_ctrl = dfzAdmittance * ((LFz_d - RFz_d) - (LFz - RFz)) - vdcDamping * (RTz - LTz);

        s = dz_ctrl;

        return s;
      }

      DEFINE_SIGNAL_INNER_FUNCTION(dz_pos, double)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Can't compute dz_pos before initialization!");
          return s;
        }

        const double & vdcFrequency = m_vdcFrequencySIN(iter);

        const MatrixHomogeneous & posRightDes = m_posRightDesSIN(iter);
        const MatrixHomogeneous & posLeftDes  = m_posLeftDesSIN(iter);
        const MatrixHomogeneous & posRight = m_posRightSIN(iter);
        const MatrixHomogeneous & posLeft  = m_posLeftSIN(iter);

        const double RTz_d = posRightDes.translation()[2];
        const double LTz_d = posLeftDes.translation()[2];

        const double RTz = posRight.translation()[2];
        const double LTz = posLeft.translation()[2];

        const double dz_pos = vdcFrequency * ((LTz_d + RTz_d) - (LTz + RTz));

        s = dz_pos;

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(vRight, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Can't compute vRight before initialization!");
          return s;
        }
        if(s.size()!=6)
          s.resize(6);

        const double & dz_ctrl = m_dz_ctrlSINNER(iter);
        const double & dz_pos  = m_dz_posSINNER(iter);
        const int & phase = m_phaseSIN(iter);

        s.setZero(6);

        if(phase==0)
          s[2] = 0.5 * (dz_pos + dz_ctrl);

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(vLeft, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Can't compute vLeft before initialization!");
          return s;
        }
        if(s.size()!=6)
          s.resize(6);

        const double & dz_ctrl = m_dz_ctrlSINNER(iter);
        const double & dz_pos  = m_dz_posSINNER(iter);
        const int & phase = m_phaseSIN(iter);

        s.setZero(6);

        if(phase==0)
          s[2] = 0.5 * (dz_pos - dz_ctrl);

        return s;
      }

      /* --- COMMANDS ---------------------------------------------------------- */

      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      void FootForceDifferenceController::display(std::ostream& os) const
      {
        os << "FootForceDifferenceController " << getName();
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }
    } // namespace talos_balance
  } // namespace sot
} // namespace dynamicgraph

