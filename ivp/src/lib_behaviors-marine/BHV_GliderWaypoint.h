/*****************************************************************/
/*    NAME: Michael Benjamin and Henrik Schmidt                  */
/*    ORGN: NAVSEA Newport RI and MIT Cambridge MA               */
/*    FILE: BHV_GliderWaypoint.h                                 */
/*    DATE: Nov 2004                                             */
/*                                                               */
/* This program is free software; you can redistribute it and/or */
/* modify it under the terms of the GNU General Public License   */
/* as published by the Free Software Foundation; either version  */
/* 2 of the License, or (at your option) any later version.      */
/*                                                               */
/* This program is distributed in the hope that it will be       */
/* useful, but WITHOUT ANY WARRANTY; without even the implied    */
/* warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR       */
/* PURPOSE. See the GNU General Public License for more details. */
/*                                                               */
/* You should have received a copy of the GNU General Public     */
/* License along with this program; if not, write to the Free    */
/* Software Foundation, Inc., 59 Temple Place - Suite 330,       */
/* Boston, MA 02111-1307, USA.                                   */
/*****************************************************************/
 
#ifndef BHV_GLIDERWAYPOINT_HEADER
#define BHV_GLIDERWAYPOINT_HEADER

#include <string>
#include "IvPBehavior.h"
#include "WaypointEngine.h"
#include "XYPoint.h"

class BHV_GliderWaypoint : public IvPBehavior {
public:
  BHV_GliderWaypoint(IvPDomain);
  ~BHV_GliderWaypoint() {};
  
  IvPFunction* onRunState();
  bool         setParam(std::string, std::string);
  void         onIdleState() {};
  void         onRunToIdleState();
  void         onSetParamComplete();

protected:
  bool         updateInfoIn();
  bool         setNextWaypoint();
  IvPFunction* buildOF(std::string);

  void         postStatusReport();
  void         postViewableSegList();
  void         postErasableSegList();
  void         postCycleFlags();
  void         handleVisualHint(std::string);

protected: 
  WaypointEngine m_waypoint_engine;

protected: // configuration parameters
  double      m_cruise_speed;
  bool        m_lead_to_start;
  double      m_lead_distance;
  double      m_lead_damper;
  std::string m_ipf_type;

  // Configurable names of MOOS variables for reports
  std::string m_var_report;
  std::string m_var_index;
  std::string m_var_cyindex;
  std::string m_var_suffix;

  // Var-Data flags for posting when behavior finishes cycle
  std::vector<VarDataPair> m_cycle_flags;

  // Visual hints affecting properties of polygons/points
  std::string m_hint_vertex_color;
  std::string m_hint_edge_color;
  double      m_hint_vertex_size;
  double      m_hint_edge_size;

protected: // intermediate or object global variables.
  double    m_osv;  // Ownship velocity
  double    m_osx;  // Ownship x position
  double    m_osy;  // Ownship y position

  XYPoint   m_nextpt;
  XYPoint   m_trackpt;
  XYPoint   m_markpt;

  double m_current;
  double m_current_direction;
  double m_current_x;
  double m_current_y;
  double m_water_speed;
};
#endif

