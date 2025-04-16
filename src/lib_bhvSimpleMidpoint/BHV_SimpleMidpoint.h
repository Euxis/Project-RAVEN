/************************************************************/
/*    NAME: Aiden Mai                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_SimpleMidpoint.h                                      */
/*    DATE:                                                 */
/************************************************************/

#ifndef SimpleMidpoint_HEADER
#define SimpleMidpoint_HEADER

#include <string>

#include "VarDataPair.h"
#include "IvPContactBehavior.h"
#include "IvPBehavior.h"

class IvPDomain;
class BHV_SimpleMidpoint : public IvPContactBehavior {
public:
  BHV_SimpleMidpoint(IvPDomain);
  ~BHV_SimpleMidpoint() {}
  IvPFunction* onRunState();
  bool         setParam(std::string, std::string);
  void         onRunToIdleState();
  void         onIdleState();
  void         onHelmStart();
  //void         onCompleteState();

protected:
  double  getRelevance();
  double  getPriority();

  bool    handleConfigModTrailRange(double);
  bool    handleConfigModTrailRangePct(double);
  void    postViewableTrailPoint();
  double  updateTrailDistance();  
  void    calculateTrailPoint();

private: // Configuration parameters
  double  m_min_trail_range;
  double  m_trail_range;
  double  m_trail_angle;
  double  m_radius;
  double  m_nm_radius;
  double  m_pwt_outer_dist;
  double  m_max_util_cpa_dist;
  double  m_trail_pt_x;
  double  m_trail_pt_y;
  double  m_trail_pt_z;
  bool    m_angle_relative;
  bool    m_post_trail_dist_on_idle;
    double m_station_y;
    double m_station_x;

  bool    m_no_alert_request;
};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_SimpleMidpoint(domain);}
}
#endif
