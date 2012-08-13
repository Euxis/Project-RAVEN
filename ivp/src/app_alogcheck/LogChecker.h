/*****************************************************************/
/*    NAME: Mike Benjamin, Christopher Gagner                    */
/*    DATE: July 29, 2010                                        */
/*    DATE: Aug  07, 2012                                        */
/*    FILE: LogChecker.h                                         */
/*    DESCRIPTION: Checks an alog for specified conditions and   */
/*              reports if those conditions have been satisfied. */
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

#ifndef _LOGCHECKER_H
#define	_LOGCHECKER_H

#include <vector>
#include <string>
#include "ALogEntry.h"
#include "InfoBuffer.h"
#include "LogicCondition.h"
#include "LogUtils.h"

const std::string m_timestamp = "ALOG_TIMESTAMP";

class LogChecker {
 public:
  LogChecker();
  virtual ~LogChecker();
  
  virtual bool parseInputFile(std::string input_file);
  virtual bool check(std::string alogfile, std::string output_file = "");
  virtual bool addFlag(std::string value, std::vector<LogicCondition> &flags);
  virtual bool updateInfoBuffer(ALogEntry entry);
  virtual bool checkFlags(std::vector<LogicCondition> flags);
  
  // Inlined functions
  virtual void setVerbose(bool verbose){m_verbose = verbose;};
  virtual bool getVerbose(void){return m_verbose;};
  
  virtual bool addStartFlag(std::string value) {
    return addFlag(value, m_start_flags);
  };
  
  virtual bool addEndFlag(std::string value) {
    return addFlag(value, m_end_flags);
  };
  
  virtual bool addFailFlag(std::string value) {
    return addFlag(value, m_fail_flags);
  };
  
  virtual bool addPassFlag(std::string value) {
    return addFlag(value, m_pass_flags);
  };
  
  virtual bool checkStartFlags() {return(checkFlags(m_start_flags));};
  virtual bool checkEndFlags()   {return(checkFlags(m_end_flags));};
  virtual bool checkFailFlags()  {return(checkFlags(m_fail_flags));};
  virtual bool checkPassFlags()  {return(checkFlags(m_pass_flags));};
    
 protected:
  
  bool m_verbose;
  bool m_overwrite_output;

  InfoBuffer* m_info_buffer;
  FILE* m_output_file;
  
  std::vector<LogicCondition> m_start_flags;
  std::vector<LogicCondition> m_end_flags;
  std::vector<LogicCondition> m_pass_flags;
  std::vector<LogicCondition> m_fail_flags;
  
  std::vector<std::string> m_vars;
};

#endif	
