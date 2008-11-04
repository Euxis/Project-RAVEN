/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: NAVSEA Newport RI and MIT Cambridge MA               */
/*    FILE: PokeDB.h                                             */
/*    DATE: May 9th 2008                                         */
/*          Motivated by Matt Grund's uMOOSPoke App              */
/*****************************************************************/

#include <string>
#include "MOOSLib.h"

class PokeDB : public CMOOSApp  
{
 public:
  PokeDB(std::string g_server, double g_port) 
    {m_db_start_time=0; m_iteration=0; m_sServerHost=g_server; m_lServerPort=g_port;};

  PokeDB() {m_db_start_time=0; m_iteration=0; m_sServerHost="localhost"; m_lServerPort=9000;};

  virtual ~PokeDB() {};
  
  bool Iterate();
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool OnConnectToServer();
  bool OnStartUp();

  void setPokeDouble(const std::string& varname, double val);
  void setPokeString(const std::string& varname, const std::string& val);

 protected:
  void registerVariables();
  void updateVariable(CMOOSMsg& msg);
  void printReport();

  bool ConfigureComms();

protected:
  std::vector<std::string>  m_varname;
  std::vector<std::string>  m_valtype;
  std::vector<std::string>  m_svalue_poke;
  std::vector<double>       m_dvalue_poke;

  std::vector<std::string>  m_svalue_read;
  std::vector<std::string>  m_dvalue_read;
  std::vector<std::string>  m_source_read;
  std::vector<std::string>  m_valtype_read;
  std::vector<std::string>  m_wrtime_read;
  std::vector<bool>         m_varname_recd;

  double m_db_start_time;
  int    m_iteration;

  std::string m_server;
  double m_port;
};
