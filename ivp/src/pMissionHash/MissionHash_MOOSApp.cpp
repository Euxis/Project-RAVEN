/************************************************************/
/*    NAME: Mike Benjamin                                   */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: MissionHash_MOOSApp.cpp                         */
/*    DATE: July 15th, 2023                                 */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "HashUtils.h"
#include "ACTable.h"
#include "MissionHash_MOOSApp.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

MissionHash_MOOSApp::MissionHash_MOOSApp()
{
  // Init Config Vars
  m_mission_hash_var = "MISSION_HASH";
  
  // Init State Vars
  m_last_mhash_post = 0;
  
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool MissionHash_MOOSApp::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key  = msg.GetKey();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
    string sval = msg.GetString(); 
#endif

    bool handled = true;
    if(key == "RESET_MHASH") 
      handled = setMissionHash();    
    else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      handled = false;

    if(!handled) 
      reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool MissionHash_MOOSApp::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool MissionHash_MOOSApp::Iterate()
{
  AppCastingMOOSApp::Iterate();

  if(m_mission_hash_var != "") {
    double elapsed = m_curr_time - m_last_mhash_post;
    if(elapsed > 30) {
      Notify(m_mission_hash_var, m_mission_hash);
      m_last_mhash_post = m_curr_time;
    }
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool MissionHash_MOOSApp::OnStartUp()
{
  // App efaults are usually sufficient. No config block required
  string directives = "must_have_moosblock=false";
  AppCastingMOOSApp::OnStartUpDirectives(directives);

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  m_MissionReader.GetConfiguration(GetAppName(), sParams);

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "mission_hash_var") {
      handled = setNonWhiteVarOnString(m_mission_hash_var, value);
      if(tolower(m_mission_hash_var) == "off")
	m_mission_hash_var = "";
    }


    if(!handled)
      reportUnhandledConfigWarning(orig);

  }

  setMissionHash();
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void MissionHash_MOOSApp::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("RESET_MHASH");
}

//----------------------------------------------------------------------
// Procedure: setMissionHash()

bool MissionHash_MOOSApp::setMissionHash()
{
  double actual_utc = MOOSTime();
  if(m_time_warp != 0)
    actual_utc = MOOSTime() / m_time_warp;

  string hash = missionHash();
  
  m_mission_hash = "mhash=" + hash;
  m_mission_hash += ",utc=" + doubleToString(actual_utc,2);

  biteString(hash, '-');
  biteString(hash, '-');
  m_mission_hash_short = hash;
  
  return(true);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool MissionHash_MOOSApp::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one" << "two" << "three" << "four";
  m_msgs << actab.getFormattedString();

  return(true);
}




