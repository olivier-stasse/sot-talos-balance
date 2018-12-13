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

#include "sot/talos_balance/admittance-controller-single-joint.hh"

#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-bind.h>

#include "sot/talos_balance/utils/commands-helper.hh"
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

//Size to be aligned                                             "-------------------------------------------------------"
#define PROFILE_ADMITTANCECONTROLLERSINGLEJOINT_QREF_COMPUTATION "AdmittanceControllerSingleJoint: qRef computation      "

#define INPUT_SIGNALS     m_KpSIN << m_stateSIN << m_tauSIN << m_tauDesSIN

#define OUTPUT_SIGNALS m_qRefSOUT

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef AdmittanceControllerSingleJoint EntityClassName;

      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(AdmittanceControllerSingleJoint,
                                         "AdmittanceControllerSingleJoint");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      AdmittanceControllerSingleJoint::AdmittanceControllerSingleJoint(const std::string& name)
                      : Entity(name)
                      , CONSTRUCT_SIGNAL_IN(Kp, dynamicgraph::Vector)
                      , CONSTRUCT_SIGNAL_IN(state, dynamicgraph::Vector)
                      , CONSTRUCT_SIGNAL_IN(tau, dynamicgraph::Vector)
                      , CONSTRUCT_SIGNAL_IN(tauDes, dynamicgraph::Vector)
                      , CONSTRUCT_SIGNAL_OUT(qRef, dynamicgraph::Vector, INPUT_SIGNALS)
                      , m_initSucceeded(false)
      {
        Entity::signalRegistration( INPUT_SIGNALS << OUTPUT_SIGNALS );

        /* Commands. */
        addCommand("init", makeCommandVoid2(*this, &AdmittanceControllerSingleJoint::init, docCommandVoid2("Initialize the entity.","Control gains","time step")));
        addCommand("setPosition", makeCommandVoid1(*this, &AdmittanceControllerSingleJoint::setPosition, docCommandVoid1("Set initial reference position.","Initial position")));
      }

      void AdmittanceControllerSingleJoint::init(const double & dt, const unsigned & n)
      {
        if(n<1)
          return SEND_MSG("n must be at least 1", MSG_TYPE_ERROR);
        if(!m_KpSIN.isPlugged())
          return SEND_MSG("Init failed: signal Kp is not plugged", MSG_TYPE_ERROR);
        if(!m_stateSIN.isPlugged())
          return SEND_MSG("Init failed: signal state is not plugged", MSG_TYPE_ERROR);
        if(!m_tauSIN.isPlugged())
          return SEND_MSG("Init failed: signal tau is not plugged", MSG_TYPE_ERROR);
        if(!m_tauDesSIN.isPlugged())
          return SEND_MSG("Init failed: signal tauDes is not plugged", MSG_TYPE_ERROR);

        m_n = n;
        m_dt = dt;
        m_q.setZero(n);
        m_initSucceeded = true;
      }

      void AdmittanceControllerSingleJoint::setPosition(const dynamicgraph::Vector & state)
      {
        m_q = state;
      }

      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      DEFINE_SIGNAL_OUT_FUNCTION(qRef, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal qRef before initialization!");
          return s;
        }

        getProfiler().start(PROFILE_ADMITTANCECONTROLLERSINGLEJOINT_QREF_COMPUTATION);

        const Vector & tauDes = m_tauDesSIN(iter);
        const Vector & tau = m_tauSIN(iter);
        const Vector & Kp = m_KpSIN(iter);

        assert(state.size()==m_n  && "Unexpected size of signal state");
        assert(tau.size()==m_n    && "Unexpected size of signal tau");
        assert(tauDes.size()==m_n && "Unexpected size of signal tauDes");
        assert(Kp.size()==m_n     && "Unexpected size of signal Kp");

        const Vector & dqRef = Kp.cwiseProduct(tauDes-tau);

        m_q += dqRef*m_dt;

        s = m_q;

        getProfiler().stop(PROFILE_ADMITTANCECONTROLLERSINGLEJOINT_QREF_COMPUTATION);

        return s;
      }


      /* --- COMMANDS ---------------------------------------------------------- */

      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      void AdmittanceControllerSingleJoint::display(std::ostream& os) const
      {
        os << "AdmittanceControllerSingleJoint " << getName();
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }
    } // namespace talos_balance
  } // namespace sot
} // namespace dynamicgraph

