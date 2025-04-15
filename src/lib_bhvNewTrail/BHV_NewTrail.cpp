/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: BHV_NewTrail.cpp                                        */
/*    DATE: Jul 3rd 2005 Sunday morning at Brueggers             */
/*                                                               */
/* This file is part of MOOS-IvP                                 */
/*                                                               */
/* MOOS-IvP is free software: you can redistribute it and/or     */
/* modify it under the terms of the GNU General Public License   */
/* as published by the Free Software Foundation, either version  */
/* 3 of the License, or (at your option) any later version.      */
/*                                                               */
/* MOOS-IvP is distributed in the hope that it will be useful,   */
/* but WITHOUT ANY WARRANTY; without even the implied warranty   */
/* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See  */
/* the GNU General Public License for more details.              */
/*                                                               */
/* You should have received a copy of the GNU General Public     */
/* License along with MOOS-IvP.  If not, see                     */
/* <http://www.gnu.org/licenses/>.                               */
/*****************************************************************/
#include <cmath>
#include <cstdlib>
#include "AngleUtils.h"
#include "GeomUtils.h"
#include "AOF_CutRangeCPA.h"
#include "BHV_NewTrail.h"
#include "OF_Reflector.h"
#include "BuildUtils.h"
#include "MBUtils.h"
#include "ZAIC_PEAK.h"
#include "OF_Coupler.h"

using namespace std;

//-----------------------------------------------------------
// Constructor()

BHV_NewTrail::BHV_NewTrail(IvPDomain gdomain) : IvPContactBehavior(gdomain)
{
  this->setParam("descriptor", "(d)trail");
  this->setParam("build_info", "uniform_piece=discrete@course:3,speed:2");
  this->setParam("build_info", "uniform_grid =discrete@course:9,speed:6");
  
  m_domain = subDomain(m_domain, "course,speed");
  
  m_trail_range    = 50;
  m_trail_angle    = 180;
  m_radius         = 5;
  m_nm_radius      = 20;
  m_pwt_outer_dist = 0;
  m_angle_relative = true; // as opposed to angle being absolute
  m_time_on_leg    = 60;
  m_trail_pt_x     = 0;
  m_trail_pt_y     = 0;
    
  m_proxy_distance_exceeded = false; //TEST
    
  m_connections_ok = false; //TEST

  m_post_trail_dist_on_idle = true;

  m_no_alert_request = true;
  m_min_trail_range  = 10;
  
  addInfoVars("NAV_X, NAV_Y, NAV_SPEED, NAV_HEADING");
    //TEST
  addInfoVars("RETURN, MODE, TRAIL, DEPLOY");
}

//-----------------------------------------------------------
// Procedure: setParam()
//
//    trail_range: desired range to the vehicle trailed.
//    trail_angle: desired angle to the vehicle trailed.
//         radius: distance to the desired trailing point within
//                 which the behavior is "shadowing".
//      nm_radius: If within this and heading ahead of target slow down
// pwt_outer_dist: contact range outside which priority is zero.

bool BHV_NewTrail::setParam(string param, string param_val)
{
  if(IvPContactBehavior::setParam(param, param_val))
    return(true);
  
  double dval = atof(param_val.c_str());

  if(param == "nm_radius")
    return(setNonNegDoubleOnString(m_nm_radius, param_val));
  else if(param == "no_alert_request")
    return(setBooleanOnString(m_no_alert_request, param_val));
  else if(param == "post_trail_dist_on_idle")
    return(setBooleanOnString(m_post_trail_dist_on_idle, param_val));
  else if(param == "pwt_outer_dist")
    return(setNonNegDoubleOnString(m_pwt_outer_dist, param_val));
  else if(param == "radius")
    return(setNonNegDoubleOnString(m_radius, param_val));
  else if(param == "trail_range")
    return(setNonNegDoubleOnString(m_trail_range, param_val));
  else if(param == "mod_trail_range")
    return(handleConfigModTrailRange(dval));
  else if(param == "mod_trail_range_pct")
    return(handleConfigModTrailRangePct(dval));
  else if(param == "trail_angle") {
    if(isNumber(param_val)) {
      m_trail_angle = angle180(dval);
      return(true);
    }
  }
  else if(param == "trail_angle_type") {
    param_val = tolower(param_val);
    if(param_val == "absolute")
      m_angle_relative = false;
    else if(param_val == "relative")
      m_angle_relative = true;
    else
      return(false);
    return(true);
  }
  
  return(false);
}

//-----------------------------------------------------------
// Procedure: handleConfigModTrailRange()

bool BHV_NewTrail::handleConfigModTrailRange(double mod)
{
  m_trail_range += mod;
  if(m_trail_range <= m_min_trail_range)
    m_trail_range = m_min_trail_range;
  return(true);
}

//-----------------------------------------------------------
// Procedure: handleConfigModTrailRangePct()
//      Note: Given pct in the range of (0, inf).
//            Fifty percent is given as 50, not 0.5

bool BHV_NewTrail::handleConfigModTrailRangePct(double pct_100)
{
  if(pct_100 <= 0)
    return(false);

  double pct = pct_100 / 100.0;

  m_trail_range *= pct;
  if(m_trail_range <= m_min_trail_range)
    m_trail_range = m_min_trail_range;
  return(true);
}

//-----------------------------------------------------------
// Procedure: onHelmStart()
//      Note: This function is called when the helm starts, even if,
//            especially if, the behavior is just a template at start
//            time to be spawned later.
//      Note: An alert request will be sent to the contact manager if
//            the behavior is configured with templating enabled, and
//            an updates variable has been provided.  In the rare case
//            that the above is true but the user still does not want
//            an alert request generated, this can be done by setting
//            m_no_alert_request to true.

void BHV_NewTrail::onHelmStart()
{
  if(m_no_alert_request || (m_update_var == "") || !m_dynamically_spawnable)
    return;

  string alert_templ = m_update_var + "=name=$[VNAME] # contact=$[VNAME]";
  string request = "id=" + getDescriptor();
  request += ", on_flag=" + alert_templ;
  request += ",alert_range=" + doubleToStringX(m_pwt_outer_dist,1);
  request += ", cpa_range=" + doubleToStringX(m_pwt_outer_dist+5,1);
  request = augmentSpec(request, getFilterSummary());
  
  postMessage("BCM_ALERT_REQUEST", request);
}

//-----------------------------------------------------------
// Procedure: onRunState()

//IvPFunction *BHV_NewTrail::onRunState()
//{
//  if(!platformUpdateOK())
//    return(0);
//    
//    
//
////------------------------------------------------------------------------------------------------------------------TEST
//  bool ok_proxy_x, ok_proxy_y;
//  double proxy_x = getLedgerInfoDbl("shoresideproxy", "x", ok_proxy_x);
//  double proxy_y = getLedgerInfoDbl("shoresideproxy", "y", ok_proxy_y);
//  if(ok_proxy_x && ok_proxy_y) {
//      double dist_to_proxy = hypot(m_osx - proxy_x, m_osy - proxy_y);
//      if(dist_to_proxy >= 90) {
//          setComplete(); // End the behavior
//          return(0);
//      }
//  }
////----------------------------------------------------------------------------------------------------------------
//  
//  // Added Aug11,2008 on GLINT to handle sync AUV multistatic - mikerb
//  if(m_extrapolate && m_extrapolator.isDecayMaxed())
//    return(0);
//
//  calculateTrailPoint();
//  postViewableTrailPoint();
//    
//    
//  // double adjusted_angle = angle180(m_cnh + m_trail_angle);
//  // projectPoint(adjusted_angle, m_trail_range, m_cnx,
//  //              m_cny, m_trail_pt_x, m_trail_pt_y);
//
//  // Calculate the relevance first. If zero-relevance, we won't
//  // bother to create the objective function.
//  double relevance = getRelevance();
//
//  postRepeatableMessage("TRAIL_RELEVANCE", relevance);
//  
//  m_cnh =angle360(m_cnh);
//
//  postRepeatableMessage("TRAIL_CTR", 1);
//
//  if(relevance <= 0) {
//    postMessage("PURSUIT", 0);
//    return(0);
//  }
//  
//  postMessage("PURSUIT", 1);
//  
//  IvPFunction *ipf = 0;
//  double head_x = cos(headingToRadians(m_cnh));
//  double head_y = sin(headingToRadians(m_cnh));
//  
//  double distance = updateTrailDistance();
//  bool   outside = (distance > m_radius);
// 
//  if(outside) {
//    if(distance > m_nm_radius) {  // Outside nm_radius
//      postMessage("REGION", "Outside nm_radius");
//      
//      AOF_CutRangeCPA aof(m_domain);
//      aof.setParam("cnlat", m_trail_pt_y);
//      aof.setParam("cnlon", m_trail_pt_x);
//      aof.setParam("cncrs", m_cnh);
//      aof.setParam("cnspd", m_cnv);
//      aof.setParam("oslat", m_osy);
//      aof.setParam("oslon", m_osx);
//      aof.setParam("tol",   m_time_on_leg);
//      bool ok = aof.initialize();
//      
//      if(!ok) {
//    postWMessage("Error in initializing AOF_CutRangeCPA.");
//    return(0);
//      }
//      
//      OF_Reflector reflector(&aof);
//      reflector.create(m_build_info);
//      if(!reflector.stateOK())
//    postWMessage(reflector.getWarnings());
//      else
//    ipf = reflector.extractIvPFunction();
//    }
//    else { // inside nm_radius
//      postMessage("REGION", "Inside nm_radius");
//      
//      double ahead_by = head_x*(m_osx-m_trail_pt_x)+head_y*(m_osy-m_trail_pt_y) ;
//      //bool ahead = (ahead_by > 0);
//      
//      // head toward point nm_radius ahead of trail point
//      double ppx = head_x*m_nm_radius+m_trail_pt_x;
//      double ppy = head_y*m_nm_radius+m_trail_pt_y;
//      double distp=hypot((ppx-m_osx), (ppy-m_osy));
//      double bear_x = (head_x*m_nm_radius+m_trail_pt_x-m_osx)/distp;
//      double bear_y = (head_y*m_nm_radius+m_trail_pt_y-m_osy)/distp;
//      double modh = radToHeading(atan2(bear_y,bear_x));
//      
//      postIntMessage("TRAIL_HEADING", modh);
//      
//      ZAIC_PEAK hdg_zaic(m_domain, "course");
//      
//      // summit, pwidth, bwidth, delta, minutil, maxutil
//      hdg_zaic.setParams(modh, 30, 150, 50, 0, 100);
//      hdg_zaic.setValueWrap(true);
//      
//      IvPFunction *hdg_ipf = hdg_zaic.extractIvPFunction();
//      
//      // If ahead, reduce speed proportionally
//      // if behind, increaase speed proportionally
//      
//      double modv = m_cnv * (1 - 0.5*ahead_by/m_nm_radius);
//      
//      if(modv < 0 || !m_extrapolate)
//    modv = 0;
//      
//      // snap to one decimal precision to reduce excess postings.
//      double snapped_modv = snapToStep(modv, 0.1);
//      postMessage("TRAIL_SPEED", snapped_modv);
//      
//      ZAIC_PEAK spd_zaic(m_domain, "speed");
//      
//      spd_zaic.setSummit(modv);
//      spd_zaic.setPeakWidth(0.1);
//      spd_zaic.setBaseWidth(2.0);
//      spd_zaic.setSummitDelta(50.0);
//      
//      // the following creates 0 desired speed. HS 032708
//      //      spd_zaic.addSummit(modv, 0, 2.0, 10, 0, 25);
//      //      spd_zaic.setValueWrap(true);
//      
//      IvPFunction *spd_ipf = spd_zaic.extractIvPFunction();
//      
//      OF_Coupler coupler;
//      ipf = coupler.couple(hdg_ipf, spd_ipf);
//    }
//  }
//  else {
//    postMessage("REGION", "Inside radius");
//    ZAIC_PEAK hdg_zaic(m_domain, "course");
//    
//    // summit, pwidth, bwidth, delta, minutil, maxutil
//    hdg_zaic.setParams(m_cnh, 30, 150, 50, 0, 100);
//    hdg_zaic.setValueWrap(true);
//    
//    IvPFunction *hdg_ipf = hdg_zaic.extractIvPFunction();
//    
//    ZAIC_PEAK spd_zaic(m_domain, "speed");
//    
//    // If inside radius and ahead, reduce speed a little
//    double modv=m_cnv;
//    //      if (ahead)
//    //    modv = m_cnv - 0.1;
//    
//    if(modv < 0 || !m_extrapolate)
//      modv = 0;
//    
//    postMessage("TRAIL_SPEED", modv);
//    
//    // summit, pwidth, bwidth, delta, minutil, maxutil
//    spd_zaic.setParams(modv, 0.1, 2.0, 50, 0, 100);
//      
//      
//    
//    // the following creates 0 desired speed. HS 032708
//    //      spd_zaic.addSummit(modv, 0, 2.0, 10, 0, 25);
//    //      spd_zaic.setValueWrap(true);
//    
//    IvPFunction *spd_ipf = spd_zaic.extractIvPFunction();
//    
//    OF_Coupler coupler;
//    ipf = coupler.couple(hdg_ipf, spd_ipf);
//  }
//  
//  if(ipf) {
//    ipf->getPDMap()->normalize(0.0, 100.0);
//    ipf->setPWT(relevance * m_priority_wt);
//  }
//  
//  return(ipf);
//}
//----------------------------------------------------------------------------------------------------------old
IvPFunction *BHV_NewTrail::onRunState()
{
  if(!platformUpdateOK())
    return(0);
    
    

//------------------------------------------------------------------------------------------------------------------TEST
  bool ok_proxy_x, ok_proxy_y;
  double proxy_x = getLedgerInfoDbl("shoresideproxy", "x", ok_proxy_x);
  double proxy_y = getLedgerInfoDbl("shoresideproxy", "y", ok_proxy_y);
  if(ok_proxy_x && ok_proxy_y) {
      double dist_to_proxy = hypot(m_osx - proxy_x, m_osy - proxy_y);
      if(dist_to_proxy >= 90) {
          setComplete(); // End the behavior
          return(0);
      }
  }
//----------------------------------------------------------------------------------------------------------------
  
  // Added Aug11,2008 on GLINT to handle sync AUV multistatic - mikerb
  if(m_extrapolate && m_extrapolator.isDecayMaxed())
    return(0);

  calculateTrailPoint();
  postViewableTrailPoint();
    
    
  // double adjusted_angle = angle180(m_cnh + m_trail_angle);
  // projectPoint(adjusted_angle, m_trail_range, m_cnx,
  //              m_cny, m_trail_pt_x, m_trail_pt_y);

  // Calculate the relevance first. If zero-relevance, we won't
  // bother to create the objective function.
  double relevance = getRelevance();

  postRepeatableMessage("TRAIL_RELEVANCE", relevance);
  
  m_cnh =angle360(m_cnh);

  postRepeatableMessage("TRAIL_CTR", 1);

  if(relevance <= 0) {
    postMessage("PURSUIT", 0);
    return(0);
  }
  
  postMessage("PURSUIT", 1);
  
  IvPFunction *ipf = 0;
  double head_x = cos(headingToRadians(m_cnh));
  double head_y = sin(headingToRadians(m_cnh));
  
  double distance = updateTrailDistance();
  bool   outside = (distance > m_radius);
    
    //TEST------------------------------------------------------------
        bool contact_stationary = (m_cnv < 0.2);
        if(contact_stationary) {
            
          postMessage("TRAIL_STATUS", "Contact stationary - orbiting");

          double desired_orbit_radius = 10.0;
          double orbit_speed = 1.5;
          double kp = 1.0;                    // heading correction gain

          // Distance to trail point
          double current_radius = hypot(m_osx - m_trail_pt_x, m_osy - m_trail_pt_y);

          // Compute how far off we are from desired radius
          double error = current_radius - desired_orbit_radius;

          // Compute angle to trail point
          double angle_to_trail = relAng(m_osx, m_osy, m_trail_pt_x, m_trail_pt_y);

          // Add correction to heading: clockwise orbit (+90) ± error correction
          double circle_angle = angle_to_trail + 90 - kp * error;

          // Normalize heading
          circle_angle = angle360(circle_angle);

          // Build heading ZAIC
          ZAIC_PEAK hdg_zaic(m_domain, "course");
          hdg_zaic.setParams(circle_angle, 30, 180, 50, 0, 100);
          hdg_zaic.setValueWrap(true);
          IvPFunction *hdg_ipf = hdg_zaic.extractIvPFunction();

          // Fixed orbit speed
          ZAIC_PEAK spd_zaic(m_domain, "speed");
          spd_zaic.setParams(orbit_speed, 0.1, 2.0, 20, 0, 100);
          IvPFunction *spd_ipf = spd_zaic.extractIvPFunction();

          // Couple heading and speed
          OF_Coupler coupler;
          ipf = coupler.couple(hdg_ipf, spd_ipf);
        }
    //-------------------------------------------------------------------
 
  else if(outside) {
    if(distance > m_nm_radius) {  // Outside nm_radius
      postMessage("REGION", "Outside nm_radius");
      
      AOF_CutRangeCPA aof(m_domain);
      aof.setParam("cnlat", m_trail_pt_y);
      aof.setParam("cnlon", m_trail_pt_x);
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
      
      OF_Reflector reflector(&aof);
      reflector.create(m_build_info);
      if(!reflector.stateOK())
    postWMessage(reflector.getWarnings());
      else
    ipf = reflector.extractIvPFunction();
    }
    else { // inside nm_radius
      postMessage("REGION", "Inside nm_radius");
      
      double ahead_by = head_x*(m_osx-m_trail_pt_x)+head_y*(m_osy-m_trail_pt_y) ;
      //bool ahead = (ahead_by > 0);
      
      // head toward point nm_radius ahead of trail point
      double ppx = head_x*m_nm_radius+m_trail_pt_x;
      double ppy = head_y*m_nm_radius+m_trail_pt_y;
      double distp=hypot((ppx-m_osx), (ppy-m_osy));
      double bear_x = (head_x*m_nm_radius+m_trail_pt_x-m_osx)/distp;
      double bear_y = (head_y*m_nm_radius+m_trail_pt_y-m_osy)/distp;
      double modh = radToHeading(atan2(bear_y,bear_x));
      
      postIntMessage("TRAIL_HEADING", modh);
      
      ZAIC_PEAK hdg_zaic(m_domain, "course");
      
      // summit, pwidth, bwidth, delta, minutil, maxutil
      hdg_zaic.setParams(modh, 30, 150, 50, 0, 100);
      hdg_zaic.setValueWrap(true);
      
      IvPFunction *hdg_ipf = hdg_zaic.extractIvPFunction();
      
      // If ahead, reduce speed proportionally
      // if behind, increaase speed proportionally
      
      double modv = m_cnv * (1 - 0.5*ahead_by/m_nm_radius);
      
      if(modv < 0 || !m_extrapolate)
    modv = 0;
      
      // snap to one decimal precision to reduce excess postings.
      double snapped_modv = snapToStep(modv, 0.1);
      postMessage("TRAIL_SPEED", snapped_modv);
      
      ZAIC_PEAK spd_zaic(m_domain, "speed");
      
      spd_zaic.setSummit(modv);
      spd_zaic.setPeakWidth(0.1);
      spd_zaic.setBaseWidth(2.0);
      spd_zaic.setSummitDelta(50.0);
      
      // the following creates 0 desired speed. HS 032708
      //      spd_zaic.addSummit(modv, 0, 2.0, 10, 0, 25);
      //      spd_zaic.setValueWrap(true);
      
      IvPFunction *spd_ipf = spd_zaic.extractIvPFunction();
      
      OF_Coupler coupler;
      ipf = coupler.couple(hdg_ipf, spd_ipf);
    }
  }
  else {
    postMessage("REGION", "Inside radius");
    ZAIC_PEAK hdg_zaic(m_domain, "course");
    
    // summit, pwidth, bwidth, delta, minutil, maxutil
    hdg_zaic.setParams(m_cnh, 30, 150, 50, 0, 100);
    hdg_zaic.setValueWrap(true);
    
    IvPFunction *hdg_ipf = hdg_zaic.extractIvPFunction();
    
    ZAIC_PEAK spd_zaic(m_domain, "speed");
    
    // If inside radius and ahead, reduce speed a little
    double modv=m_cnv;
    //      if (ahead)
    //    modv = m_cnv - 0.1;
    
    if(modv < 0 || !m_extrapolate)
      modv = 0;
    
    postMessage("TRAIL_SPEED", modv);
    
    // summit, pwidth, bwidth, delta, minutil, maxutil
    spd_zaic.setParams(modv, 0.1, 2.0, 50, 0, 100);
      
      
    
    // the following creates 0 desired speed. HS 032708
    //      spd_zaic.addSummit(modv, 0, 2.0, 10, 0, 25);
    //      spd_zaic.setValueWrap(true);
    
    IvPFunction *spd_ipf = spd_zaic.extractIvPFunction();
    
    OF_Coupler coupler;
    ipf = coupler.couple(hdg_ipf, spd_ipf);
  }
  
  if(ipf) {
    ipf->getPDMap()->normalize(0.0, 100.0);
    ipf->setPWT(relevance * m_priority_wt);
  }
  
  return(ipf);
}

//-----------------------------------------------------------
// Procedure: onRunToIdleState()

void BHV_NewTrail::onRunToIdleState()
{
    postMessage("PURSUIT", 0);
}

//-----------------------------------------------------------
// Procedure: onIdleState()

void BHV_NewTrail::onIdleState()
{
  calculateTrailPoint();
  if(m_post_trail_dist_on_idle)
    updateTrailDistance();
}

//-----------------------------------------------------------
// Procedure: getRelevance()

//double BHV_NewTrail::getRelevance()
//{
//  // For now just return 1.0 if within max_range. But we could
//  // imagine that we would reduce its relevance (linearly perhaps)
//  // if the vehicle were already in a good position.
//
//  if(m_pwt_outer_dist == 0)
//    return(1.0);
//
//  postIntMessage("TRAIL_RANGE", m_contact_range );
//
//  if(m_contact_range < m_pwt_outer_dist)
//    return(1.0);
//  else
//    return(0.0);
//}

double BHV_NewTrail::getRelevance() //TEST
{
    // Immediately return 0 if ANY connection is missing
    if (!m_connections_ok){
        return 0.0;
    }

    // Existing relevance logic (based on range)
    if (m_pwt_outer_dist == 0)
        return 1.0;

    postIntMessage("TRAIL_RANGE", m_contact_range);
    return (m_contact_range < m_pwt_outer_dist) ? 1.0 : 0.0;
}

//-----------------------------------------------------------
// Procedure: postViewableTrailPoint()

void BHV_NewTrail::postViewableTrailPoint()
{
  XYPoint m_trail_point;
  m_trail_point.set_vertex(m_trail_pt_x, m_trail_pt_y);
  m_trail_point.set_label(m_us_name + "_trailpoint");
  m_trail_point.set_active(true);

  m_trail_point.set_vertex_size(8);
  m_trail_point.set_vertex_color("gray50");
  m_trail_point.set_time(getBufferCurrTime());

  string spec = m_trail_point.get_spec();
  postMessage("VIEW_POINT", spec);
}


//-----------------------------------------------------------
// Procedure: updateTrailDistance()

double BHV_NewTrail::updateTrailDistance()
{
  double distance = distPointToPoint(m_osx, m_osy, m_trail_pt_x, m_trail_pt_y);
  postIntMessage("TRAIL_DISTANCE", distance);
  return distance;
}

//-----------------------------------------------------------
// Procedure: calculateTrailPoint()

//void BHV_NewTrail::calculateTrailPoint()
//{
//  // Calculate the trail point based on trail_angle, trail_range.
//  //  double m_trail_pt_x, m_trail_pt_y;
//
//  if(m_angle_relative) {
//    double abs_angle = headingToRadians(angle360(m_cnh+m_trail_angle));
//    m_trail_pt_x = m_cnx + m_trail_range*cos(abs_angle);
//    m_trail_pt_y = m_cny + m_trail_range*sin(abs_angle);
//  }
//  else
//    projectPoint(m_trail_angle, m_trail_range, m_cnx, m_cny, m_trail_pt_x, m_trail_pt_y);
//
//}

//void BHV_NewTrail::calculateTrailPoint()
//{
//  bool ok1, ok2, ok3, ok4;
//  double veh1_x = this->IvPBehavior::getLedgerInfoDbl("shoresideproxy", "x", ok1);
//  double veh1_y = this->IvPBehavior::getLedgerInfoDbl("shoresideproxy", "y", ok2);
//  double veh2_x = this->IvPBehavior::getLedgerInfoDbl("alpha", "x", ok3);
//  double veh2_y = this->IvPBehavior::getLedgerInfoDbl("alpha", "y", ok4);
//
//
//  if (ok1 && ok2 && ok3 && ok4) {
//    m_trail_pt_x = (veh1_x + veh2_x) / 2.0;
//    m_trail_pt_y = (veh1_y + veh2_y) / 2.0;
//  } else if (m_angle_relative) {
//    double abs_angle = headingToRadians(angle360(m_cnh+m_trail_angle));
//    m_trail_pt_x = m_cnx + m_trail_range*cos(abs_angle);
//    m_trail_pt_y = m_cny + m_trail_range*sin(abs_angle);
//  } else {
//    projectPoint(m_trail_angle, m_trail_range, m_cnx, m_cny, m_trail_pt_x, m_trail_pt_y);
//  }
//}

void BHV_NewTrail::calculateTrailPoint() //TEST
{
    bool ok1, ok2, ok3, ok4;
    double veh1_x = getLedgerInfoDbl("shoresideproxy", "x", ok1);
    double veh1_y = getLedgerInfoDbl("shoresideproxy", "y", ok2);
    double veh2_x = getLedgerInfoDbl("alpha", "x", ok3);
    double veh2_y = getLedgerInfoDbl("alpha", "y", ok4);

    bool shoreside_available = ok1 && ok2; // Both x and y for shoresideproxy
    bool alpha_available = ok3 && ok4;      // Both x and y for alpha
    m_connections_ok = shoreside_available && alpha_available;

    if (m_connections_ok) {
        // Only compute the midpoint if BOTH connections are valid
        m_trail_pt_x = (veh1_x + veh2_x) / 2.0;
        m_trail_pt_y = (veh1_y + veh2_y) / 2.0;
    }else {
        // If either connection is missing, do NOT compute a trail point
        // The trail_pt_x/y values will retain their previous state, but relevance is 0
    }
}
