/************************************************************/
/*    NAME: Michael Benjamin, H.Schmidt, J.Leonard          */
/*    ORGN: NAVSEA Newport RI and MIT Cambridge             */
/*    FILE: TS_MOOSApp.h                                    */
/*    DATE: May 21st 2009                                   */
/************************************************************/

#ifndef TIMER_SCRIPT_MOOS_APP_HEADER
#define TIMER_SCRIPT_MOOS_APP_HEADER

#include <vector>
#include <string>
#include "MOOSLib.h"
#include "VarDataPair.h"

class TS_MOOSApp : public CMOOSApp
{
 public:
  TS_MOOSApp();
  virtual ~TS_MOOSApp();

 public: // Standard MOOSApp functions to overload
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

 protected: // Locally defined and locally used functions
  void RegisterVariables();
  bool addNewEvent(std::string);
  void sortEvents();
  void printScript();
  void checkForPostings();

 protected: // Configuration parameters
  std::vector<VarDataPair> m_pairs;
  std::vector<double>      m_ptime;
  std::vector<bool>        m_poked;
  
  std::string m_var_next_event;

 protected: // State variables
  double   m_elapsed_time;
  double   m_start_time;
  double   m_skip_time;
};

#endif 
