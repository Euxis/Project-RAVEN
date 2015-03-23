/*****************************************************************/
/*    NAME: Michael Benjamin, Henrik Schmidt, and John Leonard   */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: main.cpp                                             */
/*    DATE: May 1st, 2005                                        */
/*                                                               */
/* This file is part of MOOS-IvP                                 */
/*                                                               */
/* MOOS-IvP is free software: you can redistribute it and/or     */
/* modify it under the terms of the GNU General Public License   */
/* as published by the Free Software Foundation, either version  */
/* 3 of the License, or (at your option) any later version.      */
/*                                                               */
/* MOOS-IvP is distributed in the hope that it will be useful,   */
/* but WITHOUT ANY WARRANTY; without even the implied warranty   */
/* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See  */
/* the GNU General Public License for more details.              */
/*                                                               */
/* You should have received a copy of the GNU General Public     */
/* License along with MOOS-IvP.  If not, see                     */
/* <http://www.gnu.org/licenses/>.                               */
/*****************************************************************/

#include <vector>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include "REPLAY_GUI.h"
#include "MBUtils.h"
#include "ReleaseInfo.h"
#include "LogViewLauncher.h"

using namespace std;

void help_message();

REPLAY_GUI* gui = 0;

//--------------------------------------------------------
// Procedure: idleProc

void idleProc(void *)
{
  if(gui) gui->conditional_step();
  Fl::flush();
  millipause(10);
}

//--------------------------------------------------------
// Procedure: main

int main(int argc, char *argv[])
{
  LogViewLauncher launcher;
  ALogDataBroker  dbroker;

  for(int i=1; i<argc; i++) {
    string argi = argv[i];
    if((argi=="-v") || (argi=="--version") || (argi=="-version")) {
      showReleaseInfo("alogview", "gpl");
      return(0);
    }
    else if((argi == "-h") || (argi == "--help") || (argi=="-help")) {
      help_message();
      return(0);
    }
    else if(strEnds(argi, ".alog")) 
      dbroker.addALogFile(argi);
    else if(strBegins(argi, "--plot=")) {
      string plot_request = argi.substr(7);
      bool ok = launcher.addPlotRequest(plot_request);
      if(!ok) {
	cout << "Plot request --plot=" << plot_request << " not handled." << endl;
	cout << "Exiting." << endl;
	exit(0);
      }
    }
  }

  gui = launcher.launch(argc, argv);
  
  if(gui) {
    gui->setCurrTime();
    Fl::add_idle(idleProc);
    return(Fl::run());
  }
  else
    return(0);
}

//--------------------------------------------------------
// Procedure: help_message()

void help_message()
{
  cout << "Usage:                                                    " << endl;
  cout << "  alogview file.alog [another_file.alog] [OPTIONS]        " << endl;
  cout << "                                                          " << endl;
  cout << "Synopsis:                                                 " << endl;
  cout << "  Renders vehicle paths from multiple MOOS .alog files.   " << endl;
  cout << "  Renders time-series plots for any logged numerical data." << endl;
  cout << "  Renders IvP Helm mode information vs. vehicle position. " << endl;
  cout << "  Renders IvP Helm behavior objective functions.          " << endl;
  cout << "                                                          " << endl;
  cout << "Standard Arguments:                                       " << endl;
  cout << "  file.alog - The input logfile.                          " << endl;
  cout << "                                                          " << endl;
  cout << "Options:                                                  " << endl;
  cout << "  -h,--help       Displays this help message              " << endl;
  cout << "  -v,--version    Displays the current release version    " << endl;
  cout << "                                                          " << endl;
  cout << "  --plot=VAR,FLD,...,FLD                                  " << endl;
  cout << "                  Make extra numerical plots based on the " << endl;
  cout << "                  given MOOS variable and one or more flds" << endl;

  //cout << "  --mintime=val   Clip data with timestamps < val         " << endl;
  //cout << "  --maxtime=val   Clip data with timestamps > val         " << endl;
  //cout << "  --nowtime=val   Set the initial startup time            " << endl;
  //cout << "  --geometry=val  Viewer window pixel size in HGTxWID     " << endl;
  //cout << "                  or large, medium, small, xsmall         " << endl;
  //cout << "                  Default size is 1400x1100 (large)       " << endl;
  //cout << "  --image                                                 " << endl;
  //cout << "  --ipfs                                                  " << endl;

  cout << "                                                          " << endl;
  cout << "Further Notes:                                            " << endl;
  cout << "  (1) Multiple .alog files ok - typically one per vehicle " << endl;
  cout << "  (2) Non alog files will be scanned for polygons         " << endl;
  cout << "  (3) See also: alogscan, alogrm, alogclip, aloggrep      " << endl;
  cout << endl;
}






