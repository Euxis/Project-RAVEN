/************************************************************/
/*    NAME: Aiden Mai                                       */
/*    ORGN: UMASSD                                          */
/*    FILE: BHV_SimpleMidpoint.cpp                          */
/*    DATE: 02 February 2025                                */
/*    This is a modified vesrion of BHV_Trail               */
/************************************************************/


#include <iterator>
#include <cmath>
#include <cstdlib>
#include <algorithm> 
#include "AngleUtils.h"
#include "GeomUtils.h"
#include "AOF_CutRangeCPA.h"
#include "BHV_SimpleMidpoint.h"
#include "OF_Reflector.h"
#include "BuildUtils.h"
#include "MBUtils.h"
#include "ZAIC_PEAK.h"
#include "OF_Coupler.h"
#include "IvPBehavior.h"
#include "XYPoint.h"
#include "BHV_Waypoint.h"
#include "WaypointEngine.h"


using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_SimpleMidpoint::BHV_SimpleMidpoint(IvPDomain domain) :
IvPContactBehavior(domain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "simple_midpoint");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course,speed");

  m_comm_radius = 20;
  m_station_point_x = 0;
  m_station_point_y = 0;
  m_midpoint_x = 0;
  m_midpoint_y = 0;

  // Add any variables this behavior needs to subscribe for
  addInfoVars("NAV_X, NAV_Y, NAV_SPEED, NAV_HEADING, STATION_UPDATE");
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_SimpleMidpoint::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  if(IvPContactBehavior::setParam(param, val))
    return(true);

  // Get the numerical value of the param argument for convenience once
  double double_val = atof(val.c_str());
  
  // This is for handling parameters to their type. We can set what type of data is set to the parameter here.
  if(param == "comm_radius") {
    // Set local member variables here
    return(setNonNegDoubleOnString(m_comm_radius, val));
  }
  else if(param == "station_point_x"){
    return(setNonNegDoubleOnString(m_station_point_x, val));
  }
  else if(param == "station_point_y"){
    return(setNonNegDoubleOnString(m_station_point_y, val));
  }
  else if(param == "midpoint_x"){
    return(setNonNegDoubleOnString(m_midpoint_x, val));
  }
  else if(param == "midpoint_y"){
    return(setNonNegDoubleOnString(m_midpoint_y, val));
  }
  // If not handled above, then just return false;
  return(false);
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_SimpleMidpoint::onHelmStart()
{
  string alert_templ = m_update_var + "=name=$[VNAME] # contact=$[VNAME]";
  string request = "id=" + getDescriptor();
  request += ", on_flag=" + alert_templ;
  request += ",alert_range=" + doubleToStringX(m_comm_radius,1);
  request += ", cpa_range=" + doubleToStringX(m_comm_radius+5,1);
  request = augmentSpec(request, getFilterSummary());
  
  postMessage("BCM_ALERT_REQUEST", request);
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_SimpleMidpoint::onIdleState()
{
  calculateMidpoint();
  updateMidpointDistance();
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_SimpleMidpoint::onRunState()
{
  if(!platformUpdateOK())
    return(0);

  

  // m_cnh =angle360(m_cnh);  

  // // Part 1: Build the IvP function
  // IvPFunction *ipf = 0;

  // double head_x = cos(headingToRadians(m_cnh));
  // double head_y = sin(headingToRadians(m_cnh));

  // double distance = updateMidpointDistance();
  // bool outside = distance > m_comm_radius;

  // cn = contact
  // os = ownship
  AOF_CutRangeCPA aof(m_domain);
  aof.setParam("cnlat", m_cnx);
  aof.setParam("cnlon", m_cny);
  aof.setParam("cncrs", m_cnh);
  aof.setParam("cnspd", m_cnv);
  aof.setParam("oslat", m_osy);
  aof.setParam("oslon", m_osx);
  aof.setParam("tol",   m_time_on_leg);
  bool ok = aof.initialize();
  
  if(!ok) {
  postWMessage("Error in initializing AOF_CutRangeCPA.");
  return(0);
  }

  calculateMidpoint();
  postViewableMidpoint();
      
  // OF_Reflector reflector(&aof);
  // reflector.create(m_build_info);
  // if(!reflector.stateOK())
  //   postWMessage(reflector.getWarnings());
  // else
  //   ipf = reflector.extractIvPFunction();

  // double ahead_by = head_x*(m_osx-m_midpoint_x)+head_y*(m_osy-m_midpoint_y) ;
  // //bool ahead = (ahead_by > 0);
  
  // // head toward point nm_radius ahead of trail point
  // double ppx = head_x*m_comm_radius+m_midpoint_x;
  // double ppy = head_y*m_comm_radius+m_midpoint_y;
  // double distp=hypot((ppx-m_osx), (ppy-m_osy));
  // double bear_x = (head_x*m_comm_radius+m_midpoint_x)/distp;
  // double bear_y = (head_y*m_comm_radius+m_midpoint_y)/distp;
  // double modh = radToHeading(atan2(bear_y,bear_x));
  
  // postIntMessage("TRAIL_HEADING", modh);
  
  // ZAIC_PEAK hdg_zaic(m_domain, "course");
  
  // // summit, pwidth, bwidth, delta, minutil, maxutil
  // hdg_zaic.setParams(modh, 30, 150, 50, 0, 100);
  // hdg_zaic.setValueWrap(true);
  
  // IvPFunction *hdg_ipf = hdg_zaic.extractIvPFunction();
  
  // // If ahead, reduce speed proportionally
  // // if behind, increaase speed proportionally
  
  // double modv = m_cnv * (1 - 0.5*ahead_by/m_comm_radius);
  
  //if(modv < 0 || !m_extrapolate) modv = 0;
  
  // // snap to one decimal precision to reduce excess postings.
  // double snapped_modv = snapToStep(modv, 0.1);
  // postMessage("TRAIL_SPEED", snapped_modv);
  
  // ZAIC_PEAK spd_zaic(m_domain, "speed");
  
  // spd_zaic.setSummit(modv);
  // spd_zaic.setPeakWidth(0.1);
  // spd_zaic.setBaseWidth(2.0);
  // spd_zaic.setSummitDelta(50.0); 
  
  // the following creates 0 desired speed. HS 032708
  //      spd_zaic.addSummit(modv, 0, 2.0, 10, 0, 25);
  // 	  spd_zaic.setValueWrap(true);
  
  // IvPFunction *spd_ipf = spd_zaic.extractIvPFunction();
  
  // OF_Coupler coupler;
  // ipf = coupler.couple(hdg_ipf, spd_ipf);

  ZAIC_PEAK zaic(m_domain, "depth");
  zaic.setSummit(1);
  zaic.setBaseWidth(1);
  zaic.setPeakWidth(1);
  zaic.setSummitDelta(1);

  IvPFunction *ipf = zaic.extractIvPFunction();
  if(ipf)
    ipf->setPWT(m_priority_wt);
  else 
    postEMessage("Unable to generate simple-depth IvP function");

  string zaic_warnings = zaic.getWarnings();
  if(zaic_warnings != "")
    postWMessage(zaic_warnings);

  return(ipf);
}

/// @brief Posts midpoint on pMarineViewer
void BHV_SimpleMidpoint::postViewableMidpoint()
{
  XYPoint m_trail_point;
  m_trail_point.set_vertex(m_midpoint_x, m_midpoint_y);
  m_trail_point.set_label("midpoint");
  m_trail_point.set_active(true);

  m_trail_point.set_vertex_size(8);
  m_trail_point.set_vertex_color("gray50");
  m_trail_point.set_time(getBufferCurrTime());

  string spec = m_trail_point.get_spec();
  postMessage("VIEW_POINT", spec);
}

// Get distance between midpoint and station point
double BHV_SimpleMidpoint::updateMidpointDistance(){
  double distance = distPointToPoint(m_midpoint_x, m_midpoint_y, m_station_point_x, m_station_point_y);
  postIntMessage("DISTANCE", distance);
  return distance;
}

/// @brief Calculate the midpoint based on GCS and contact vehicle location. Will clamp to radius if distance between GCS and contact exceeds comm radius.
void BHV_SimpleMidpoint::calculateMidpoint(){

  // Calculate midpoint between GCS and contact vehicle
  m_midpoint_x = (m_station_point_x + m_cnx) / 2;
  m_midpoint_y = (m_station_point_y + m_cny) / 2;

  // Get the distance between midpoint and station
  double midpoint_dist = updateMidpointDistance();

  // Get direction vector from station to midpoint
  double direction_x = m_midpoint_x - m_station_point_x;
  double direction_y = m_midpoint_y - m_station_point_y;

  // Calculate normalized direction vector
  double magnitude = sqrt(pow(direction_x, 2) + pow(direction_y, 2));
  double direction_x_normalized = direction_x / magnitude;
  double direction_y_normalized = direction_y / magnitude;

  // If the distance between midpoint and GCS is larger than
  // the comms radius:
  if(midpoint_dist >= m_comm_radius){
    // Clamp the midpoint to the radius
    m_midpoint_x = m_station_point_x + direction_x_normalized * m_comm_radius;
    m_midpoint_y = m_station_point_y + direction_y_normalized * m_comm_radius; 
  }
  cout << "midpoint calculated" << endl;

}
