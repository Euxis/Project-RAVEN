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

class BHV_SimpleMidpoint : public IvPContactBehavior {
public:
  BHV_SimpleMidpoint(IvPDomain);
  ~BHV_SimpleMidpoint() {};
  
  bool         setParam(std::string, std::string);
  void         onSetParamComplete();
  void         onCompleteState();
  void         onIdleState();
  void         onHelmStart();
  void         postConfigStatus();
  void         onRunToIdleState();
  void         onIdleToRunState();
  IvPFunction* onRunState();

protected: // Local Utility functions

protected: // Configuration parameters
  double m_comm_radius;   // The communication radius of ownship

protected: // State variables
};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_SimpleMidpoint(domain);}
}
#endif
