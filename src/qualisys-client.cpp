/*
 * Copyright 2019
 * LAAS-CNRS
 * F. Bailly
 * T. Flayols
 */


#include <sot/talos_balance/qualisys-client.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

#include <dynamic-graph/all-commands.h>
#include <sot/talos_balance/utils/stop-watch.hh>
#include <sot/talos_balance/utils/statistics.hh>


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

#define INPUT_SIGNALS
#define OUTPUT_SIGNALS

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef QualisysClient EntityClassName;

      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(QualisysClient,
                                         "QualisysClient");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      QualisysClient::
      QualisysClient(const std::string& name)
        : Entity(name)
        ,m_initSucceeded(false)  
      {
        /* Commands. */
        addCommand("init",
                   makeCommandVoid0(*this, &QualisysClient::init,
                                    docCommandVoid0("Initialize the entity.")));
 
        addCommand("registerRigidBody",
                   makeCommandVoid1(*this,&QualisysClient::registerRigidBody,
                                    docCommandVoid1("Register a rigid body",
                                                    "Name of the rigid body")));
        /*addCommand("removeRB",
                   makeCommandVoid1(*this,&QualisysClient::removeRB,
                            docCommandVoid1("Remove a registered a rigid body",
                                            "Name of the rigid body to be removed")));
        addCommand("getRBList",
                   makeCommandVoid1(*this,&QualisysClient::getRBList,
                            docCommandVoid0("Displays the list of registered rigid bodies"));*/

      }

      void QualisysClient::init()
      {
        m_initSucceeded = true;
      }


      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

    dg::Vector& QualisysClient::readGenericRigidBody(const std::string RBname, dg::Vector& res, const int& time)
    {
      if(res.size()!=7)
      {
        res.resize(7);
      }
      res << 0,0,0,0,0,0,0;
      return res;
    }

      /* --- COMMANDS ---------------------------------------------------------- */

      void QualisysClient::registerRigidBody(const std::string& RBname)
      {

       dg::SignalTimeDependent< dg::Vector,int > * sig;
       boost::bind(&QualisysClient::readGenericRigidBody,this,RBname,_1,_2);
       sig = new dg::SignalTimeDependent< dg::Vector,int >(boost::bind(&QualisysClient::readGenericRigidBody,this,RBname,_1,_2),NULL,getClassName()+"("+getName()+")::output(dynamicgraph::Vector)::xyzquat_"+RBname);
       //genericSignalRefs.push_back( sig );
       signalRegistration( *sig );
      }


      /* --- PROTECTED MEMBER METHODS ---------------------------------------------------------- */


      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */


      void QualisysClient::display(std::ostream& os) const
      {
        os << "QualisysClient "<<getName();
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }
      
    } // namespace talos_balance
  } // namespace sot
} // namespace dynamicgraph
