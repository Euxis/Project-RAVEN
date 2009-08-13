/*****************************************************************/
/*    NAME: Michael Benjamin and John Leonard                    */
/*    ORGN: NAVSEA Newport RI and MIT Cambridge MA               */
/*    FILE: Populator_LogPlots.h                                 */
/*    DATE: June 5th, 2005 (Sun in Kenmorre)                     */
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

#ifndef POPULATOR_LOGPLOTS_HEADER
#define POPULATOR_LOGPLOTS_HEADER

#include <string>
#include <map>
#include <vector>
#include "LogPlot.h"
#include "ALogEntry.h"

class Populator_LogPlots 
{
public:
  Populator_LogPlots() {};
  ~Populator_LogPlots() {};

  bool    setFileALog(std::string);
  bool    populateFromALog();
  bool    populateFromEntries(const std::vector<ALogEntry>&);

  LogPlot getLogPlot(unsigned int);
  LogPlot getLogPlot(std::string);
  int     size()         {return(m_logplots.size());};

  // The vehicle name should be determined from the alog file
  // but if not, a default name may be given.
  void    setDefaultVName(std::string s) {m_vname=s;};
  std::string getVName() {return(m_vname);};

 protected:
  bool     handleNodeReports();

protected:
  std::string               m_vname;
  std::vector<LogPlot>      m_logplots;

  std::vector<std::string>  m_node_reports;

  // Mapping from logplot variable to index in m_loplots vector
  std::map<std::string, unsigned int> m_logplot_var_map;
  
  std::vector<std::string> m_alog_entry_time;
  std::vector<std::string> m_alog_entry_var;
  std::vector<std::string> m_alog_entry_src;
  std::vector<std::string> m_alog_entry_val;
};
#endif 








