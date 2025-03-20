/************************************************************/
/*    NAME: Aiden Mai                                       */
/*    ORGN: UMASSD                                          */
/*    FILE: BHV_SimpleMidpoint.cpp                          */
/*    DATE: 02 February 2025                                */
/*    This is a modified vesrion of BHV_Trail               */
/************************************************************/


#include <iterator>
#include <cstdlib>
#include "AOF_CutRangeCPA.h"
#include "MBUtils.h"
#include "BuildUtils.h"
#include "BHV_SimpleMidpoint.h"

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_SimpleMidpoint::BHV_SimpleMidpoint(IvPDomain domain) :
  IvPBehavior(domain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "simple_midpoint");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course,speed");

  m_comm_radius = 10;

  // Add any variables this behavior needs to subscribe for
  addInfoVars("NAV_X, NAV_Y");
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_SimpleMidpoint::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  // Get the numerical value of the param argument for convenience once
  double double_val = atof(val.c_str());
  
  // This is for handling parameters to their type. We can set what type of data is set to the parameter here.
  if((param == "m_comm_radius") {
    // Set local member variables here

    return(setNonNegDoubleOnString(m_comm_radius, val));
  }

  // If not handled above, then just return false;
  return(false);
}

//---------------------------------------------------------------
// Procedure: onSetParamComplete()
//   Purpose: Invoked once after all parameters have been handled.
//            Good place to ensure all required params have are set.
//            Or any inter-param relationships like a<b.

void BHV_SimpleMidpoint::onSetParamComplete()
{
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_SimpleMidpoint::onHelmStart()
{
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_SimpleMidpoint::onIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_SimpleMidpoint::onCompleteState()
{
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_SimpleMidpoint::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_SimpleMidpoint::onIdleToRunState()
{
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_SimpleMidpoint::onRunToIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_SimpleMidpoint::onRunState()
{
  // Part 1: Build the IvP function
  IvPFunction *ipf = 0;

  AOF_CutRangeCPA aof(m_domain);
  aof.setParam("cnlat", m_trail_pt_y);
  aof.setParam("cnlon", m_trail_pt_x);
  aof.setParam("cncrs", m_cnh);
  aof.setParam("cnspd", m_cnv);
  aof.setParam("oslat", m_osy);
  aof.setParam("oslon", m_osx);
  aof.setParam("tol",   m_time_on_leg);
  bool ok = aof.initialize();

  // Part N: Prior to returning the IvP function, apply the priority wt
  // Actual weight applied may be some value different than the configured
  // m_priority_wt, depending on the behavior author's insite.
  if(ipf)
    ipf->setPWT(m_priority_wt);

  return(ipf);
}

void BHV_SimpleMidpoint::CalculateMidpoint(){
  // Calculate midpoint between (m_cnx, m_cny) and (m_osx, m_osy)

  // if the distance between the ownship and GCS is close to or over the distance of the radius
  // Set the midpoint to the radius distance 
}
