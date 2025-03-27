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
  ~BHV_SimpleMidpoint() {};
  
  bool         setParam(std::string, std::string);
  void         onIdleState();
  void         onHelmStart();
  IvPFunction* onRunState();

protected: // Local Utility functions
double updateMidpointDistance();
void calculateMidpoint();
void postViewableMidpoint();

protected: // Configuration parameters
  double m_comm_radius;   // The communication radius of ownship
  double m_station_point_x;
  double m_station_point_y;
  double m_midpoint_x;
  double m_midpoint_y;


protected: // State variables
};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_SimpleMidpoint(domain);}
}
#endif
