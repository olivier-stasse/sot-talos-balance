/*
 * Copyright 2015, Andrea Del Prete, LAAS-CNRS
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

#include "sot/talos_balance/example.hh"

#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-bind.h>

#include "sot/talos_balance/commands-helper.hh"
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

//Size to be aligned                    "-------------------------------------------------------"
#define PROFILE_EXAMPLE_SUM_COMPUTATION "Example: sum computation                               "

#define INPUT_SIGNALS     m_firstAddendSIN << m_secondAddendSIN

#define OUTPUT_SIGNALS m_sumSOUT

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef Example EntityClassName;

      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Example,
                                         "Example");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      Example::Example(const std::string& name)
                      : Entity(name)
                      , CONSTRUCT_SIGNAL_IN(firstAddend,  double)
                      , CONSTRUCT_SIGNAL_IN(secondAddend, double)
                      , CONSTRUCT_SIGNAL_OUT(sum,         double, INPUT_SIGNALS)
                      , m_initSucceeded(false)
      {
        Entity::signalRegistration( INPUT_SIGNALS << OUTPUT_SIGNALS );

        /* Commands. */
        addCommand("init", makeCommandVoid0(*this, &Example::init, docCommandVoid0("Initialize the entity.")));
      }

      void Example::init()
      {
        if(!m_firstAddendSIN.isPlugged())
          return SEND_MSG("Init failed: signal firstAddend is not plugged", MSG_TYPE_ERROR);
        if(!m_secondAddendSIN.isPlugged())
          return SEND_MSG("Init failed: signal secondAddend is not plugged", MSG_TYPE_ERROR);

        m_initSucceeded = true;
      }

      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      DEFINE_SIGNAL_OUT_FUNCTION(sum,double)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal sum before initialization!");
          return s;
        }

        getProfiler().start(PROFILE_EXAMPLE_SUM_COMPUTATION);

        double firstAddend  = m_firstAddendSIN(iter);
        double secondAddend = m_secondAddendSIN(iter);

        s = firstAddend + secondAddend;

        getProfiler().stop(PROFILE_EXAMPLE_SUM_COMPUTATION);

        return s;
      }


      /* --- COMMANDS ---------------------------------------------------------- */

      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      void Example::display(std::ostream& os) const
      {
        os << "Example " << getName();
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }
    } // namespace talos_balance
  } // namespace sot
} // namespace dynamicgraph

