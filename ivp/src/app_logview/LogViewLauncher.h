/*****************************************************************/
/*    NAME: Michael Benjamin and John Leonard                    */
/*    ORGN: NAVSEA Newport RI and MIT Cambridge MA               */
/*    FILE: LogViewLauncher.h                                    */
/*    DATE: May 31st, 2005                                       */
/*          August 2009 - Took over most of main.cpp's role      */
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

#ifndef LOGVIEW_LAUNCHER
#define LOGVIEW_LAUNCHER

#include <vector>
#include "Populator_LogPlots.h"
#include "Populator_VPlugPlots.h"
#include "Populator_HelmPlots.h"
#include "ALogEntry.h"
#include "REPLAY_GUI.h"

class LogViewLauncher
{
 public:
  LogViewLauncher();
  virtual ~LogViewLauncher() {};
  
  REPLAY_GUI *launch(int argc, char **argv);

protected:
  void setBackground(int argc, char **argv);
  void setSizeOfGUI(int argc, char **argv);
  void setALogFiles(int argc, char **argv);
  bool setALogFileSkews();
  void parseALogFiles();
  void parseALogFile(unsigned int);

  bool buildLogPlots();
  bool buildHelmPlots();
  bool buildVPlugPlots();
  
  bool buildGraphical();

private:
  // The .alog files provided at launch time
  std::vector<std::string> m_alog_files;
  std::vector<double>      m_alog_files_skew;

  // geometry specifications for the viewer
  double  m_gui_height;
  double  m_gui_width;

  // Background image specification
  std::string  m_tif_file;

  // Intermediate semi-raw data from the alog files
  std::vector<std::vector<ALogEntry> > m_entries_log_plot;
  std::vector<std::vector<ALogEntry> > m_entries_bhv_ipf;
  std::vector<std::vector<ALogEntry> > m_entries_vplug_plot;
  std::vector<std::vector<ALogEntry> > m_entries_helm_plot;

  // The various plots created from the alog data before passing
  // to the logview gui or viewerws
  std::vector<std::vector<LogPlot> > m_log_plots;
  std::vector<HelmPlot>  m_helm_plots;
  std::vector<VPlugPlot> m_vplug_plots;

  REPLAY_GUI *m_gui;
};

#endif 
