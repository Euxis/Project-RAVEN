/*****************************************************************/
/*    NAME: Kyle Woerner                                         */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: ScanHandle.cpp                                       */
/*    DATE: June 5rd, 2008                                       */
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

#include <iostream>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "MBUtils.h"
#include "ALogScanner.h"
#include "LogUtils.h"
#include "EffHandler.h"
#include "ColorParse.h"
#include "MBUtils.h"


using namespace std;



//--------------------------------------------------------
// Constructor

EffHandler::EffHandler()
{
  m_pcolor_map["pHelmIvP"]       = "blue";
  m_pcolor_map["pNodeReporter"]  = "green";
  
  m_pcolors.push_back("blue");
  m_pcolors.push_back("green");
  m_pcolors.push_back("red");
  m_pcolors.push_back("magenta");
  m_pcolors.push_back("cyan");
  m_pcolors.push_back("yellow");
  m_pcolors.push_back("nocolor");

  m_next_color_ix = 2;

  m_use_colors = true;
  
  m_avoid_out_string = "";
  m_transit_out_string = "";
};


//--------------------------------------------------------
// Procedure: setParam
//     Notes: 

void EffHandler::setParam(const string& param, const string& value)
{
  if(param == "sort_style")
    m_sort_style = value;
  else if(param == "proc_colors")
    setBooleanOnString(m_use_colors, value);
}

//--------------------------------------------------------
// Procedure: procColor
//     Notes: 

string EffHandler::procColor(string proc_name)
{

  string pcolor = m_pcolor_map[proc_name];
  if(pcolor != "") {
    return(pcolor);
  }

  unsigned int total_colors = m_pcolors.size(); 
  if(total_colors == 0)
    return("nocolor");
  
  if(m_next_color_ix >= total_colors-1) {
    string last_color = m_pcolors[total_colors-1];
    m_pcolor_map[proc_name] = last_color;
    return(last_color);
  }
  else {
    string next_color = m_pcolors[m_next_color_ix];
    m_pcolor_map[proc_name] = next_color;
    m_next_color_ix++;
    return(next_color);
  }

}

//--------------------------------------------------------
// Procedure: handle
//     Notes: 

void EffHandler::handle(int argc, char **argv)
{

  /*
  ALogScanner scanner;
  bool ok = scanner.openALogFile(alogfile);
  if(!ok) {
    cout << "Unable to find or open " << alogfile << " - Exiting." << endl;
    return;
  }
  */
  setALogFiles(argc, argv);  
  m_num_collisions = 0;
  //  m_alog_files = "";

 
  //begin code adopted from ALogScanner.cpp
  ScanReport report;

  //  bool more_files = true;

  bool avoid_bool = false; // tracks status of collision avoidance behavior
  bool true_to_false = false; // boolean to track when AVOIDING moves from true to false to ensure one more waypoint efficiency is grabbed

  string alogfile = "";
  int vsize = m_alog_files.size();

  printf("%s \n","lin_dist,odo_dist,eff");
  for(int i=0; i<vsize; i++) {
    alogfile = m_alog_files[i];

    cout << "processing " << alogfile << endl;
    m_file = fopen(alogfile.c_str(), "r");
    if(!m_file)
      return;
    bool done = false; 

    while(!done) {
      ALogEntry entry = getNextRawALogEntry(m_file, true);
      string status = entry.getStatus();
      // Check for the end of the file
      if(status == "eof")
	done = true;
      // If otherwise a "normal" line, process
      else if(status != "invalid") {
	string src = entry.getSource();
	// If the source is the IvP Helm, then see if the behavior
	// information is present and append to the source
	if(src == "pHelmIvP") {
	  string src_aux = entry.getSrcAux();
	  if(src_aux != "") {
	    if(strContains(src_aux, ':')) {
	      string iter = biteString(src_aux, ':');
	      string bhv  = src_aux;
	      if(bhv != "")
		src = src + ":" + bhv;
	    }
	    else
	      src += ":" + src_aux;
	  }	      
	}
      
	string var_name = entry.getVarName();
	string data = "";

	// THIS IS WHERE YOU CAN DETERMINE THE VARIABLE NAME AND ACT ON IT... //

	if(var_name == "AVOIDING"){
	  //get data value and turn into boolean
	  data = entry.getStringVal();
	  //	cout << "data in AVOIDING is " << data << endl;
	  data = tolower(data);
	  if(data == "true"){
	    avoid_bool = true;
	    true_to_false = true;
	    //	  cout << "TRUE" << endl;
	  }
	  else if(data == "false"){
	    avoid_bool = false;
	    //	  cout << "FALSE" << endl;
	  }
	  else{
	    cout << "AVOID has entry other than T/F" << endl;
	  }
      	}
	else if(var_name == "VEHICLE_COLLISION"){
	  m_num_collisions++;
	}
	else if(var_name == "WPT_EFFICIENCY_LEG"){
	  data = entry.getStringVal();
	  //	cout << "data in WPT_EFFICIENCY_LEG is " << data << endl;
	  // 
	  double lin_dist = getParsedDoubleValue(data, "linear_dist");
	  double odo_dist = getParsedDoubleValue(data, "odo_dist");
	  double eff = getParsedDoubleValue(data, "efficiency");

	  string out_string = "" + doubleToString(lin_dist) + "," + doubleToString(odo_dist) + "," + doubleToString(eff);
	  //	cout << "doubles: " << doubleToString(lin_dist) << " ; " << doubleToString(odo_dist) << " ; " << doubleToString(eff) << endl;

	  if((!avoid_bool) && (!true_to_false)){
	    // previous leg was transit leg without collision avoidance
	    transit_data.add_eff(lin_dist,odo_dist,eff);
	    cout << "adding to transit data: "  << out_string << endl;
	    m_transit_out_string += out_string + "\n";
	

	    //	  printf("%s \n", out_string.c_str());
	  }
	  else if((!avoid_bool) && (true_to_false)){
	    // not currently avoiding, but first waypoint reached after an avoidance maneuver
	    avoid_data.add_eff(lin_dist,odo_dist,eff);
	    true_to_false = false;
	    cout << "adding to avoid data: "  << out_string << endl;
	    string out_string = "" + doubleToString(lin_dist) + "," + doubleToString(odo_dist) + "," + doubleToString(eff);
	    m_avoid_out_string += out_string + "\n";
	  }
	  else if((avoid_bool) && (true_to_false)){
	    // currently conducting collision avoidance while passing through waypoint
	    // will record this leg and next leg in avoidance data
	    avoid_data.add_eff(lin_dist,odo_dist,eff);
	    cout << "adding to avoid data: "  << out_string << endl;
	    m_avoid_out_string += out_string + "\n";
	  }

	}




	//      report.addLine(entry.getTimeStamp(),    entry.getVarName(),    src,    entry.getStringVal());
      } 
    }  // done with current file
  } // done with all files

    cout << "\n\n\n\n-------------------------------------------------- \n\n" << endl;
    cout << "avoiding data follows: \n" << avoid_data.get_report() << endl;
    cout << "transit data follows: \n" << transit_data.get_report() << endl;
    cout << "number of collisions: " << doubleToString(m_num_collisions,0) << endl;
    string new_alogfile = "MOOS_COLLISION_OUTPUT_FOR_MATLAB_AVOIDING.csv";
    string file_header = "lin_dist,odo_dist,eff";
    m_file_out = fopen(new_alogfile.c_str(), "w");
    fprintf(m_file_out, "%s\n",file_header.c_str());
    fprintf(m_file_out, "%s\n",m_avoid_out_string.c_str());
    fclose(m_file_out);

    new_alogfile = "MOOS_COLLISION_OUTPUT_FOR_MATLAB_TRANSITING.csv";
    m_file_out = fopen(new_alogfile.c_str(), "w");
    fprintf(m_file_out, "%s\n",file_header.c_str());
    fprintf(m_file_out, "%s\n",m_transit_out_string.c_str());
    fclose(m_file_out);

    new_alogfile = "MOOS_COLLISION_OUTPUT_FOR_MATLAB_COLLISIONS.csv";
    m_file_out = fopen(new_alogfile.c_str(), "w");
    file_header = "num_collisions";
    fprintf(m_file_out, "%s\n",file_header.c_str());
    fprintf(m_file_out, "%s\n",doubleToString(m_num_collisions,0).c_str());
    fclose(m_file_out);

  return;
  //end code adopted from ALogScanner.cpp
  /*
  ALogScanner scanner;
  bool ok = scanner.openALogFile(alogfile);
  if(!ok) {
    cout << "Unable to find or open " << alogfile << " - Exiting." << endl;
    return;
  }

  m_report = scanner.scan();
  if(m_report.size() == 0) {
    cout << "Empty log file - exiting." << endl;
    return;
  }

  m_report.sort(m_sort_style);

#ifdef _WIN32
  int line_digits  = (int)(log10( (double)m_report.getMaxLines())) + 2;
  int char_digits  = (int)(log10( (double)m_report.getMaxChars())) + 2;
  int start_digits = (int)(log10( m_report.getMaxStartTime())) + 4;
  int stop_digits  = (int)(log10( m_report.getMaxStopTime())) + 4;
#else 
  int line_digits  = static_cast<int>(log10(static_cast<double>(m_report.getMaxLines()))) + 2;
  int char_digits  = static_cast<int>(log10(static_cast<double>(m_report.getMaxChars()))) + 2;
  int start_digits = (int)(log10(m_report.getMaxStartTime())) + 4;
  int stop_digits  = (int)(log10(m_report.getMaxStopTime())) + 4;
#endif 

  //cout << "line_digits:  " << line_digits  << endl;
  //cout << "char_digits:  " << char_digits  << endl;
  //cout << "start_digits: " << start_digits << endl;
  //cout << "stop_digits:  " << stop_digits  << endl;

  if(line_digits  < 5)  line_digits  = 5;
  if(char_digits  < 5)  char_digits  = 5;
  if(start_digits < 6)  start_digits = 6;
  if(stop_digits  < 6)  stop_digits  = 6;

  string sline_digits  = intToString(line_digits);
  string schar_digits  = intToString(char_digits);
  string svname_digits = intToString(m_report.getVarNameMaxLength());
  string sstart_digits = intToString(start_digits);
  string sstop_digits  = intToString(stop_digits);

  string hformat_string = ("%-" + svname_digits + "s ");
  hformat_string += ("%" + sline_digits + "s ");
  hformat_string += ("%" + schar_digits + "s  ");
  hformat_string += ("%" + sstart_digits + "s  ");
  hformat_string += ("%" + sstop_digits  + "s  %s\n");

  string bformat_string = ("%-" + svname_digits + "s ");
  bformat_string += ("%" + sline_digits + "d ");
  bformat_string += ("%" + schar_digits + "d  ");
  bformat_string += ("%" + sstart_digits + "s  ");
  bformat_string += ("%" + sstop_digits  + "s  %s\n");

  printf("\n");
  printf(hformat_string.c_str(),
	 "Variable Name", "Lines", "Chars", " Start", "  Stop", "Sources");
  printf(hformat_string.c_str(),
	 "-------------", "-----", "-----", "------", "------", "-------");

  unsigned int i, vsize = m_report.size();
  for(i=0; i<vsize; i++) {
    string varname     = m_report.getVarName(i);
    string varsources  = m_report.getVarSources(i);
    double first       = m_report.getVarFirstTime(i);
    double last        = m_report.getVarLastTime(i);
    unsigned int lcnt  = m_report.getVarCount(i);
    unsigned int chars = m_report.getVarChars(i);

    string sfirst = doubleToString(first, 2);
    string slast  = doubleToString(last,  2);

    if(m_use_colors) {
      string varsources_copy = varsources;
      string first_source = biteString(varsources_copy, ',');

      string key = first_source;
      if(strContains(first_source, ':')) {
	string app_name = biteString(first_source, ':');
	if(app_name != "")
	  key = app_name;
	else
	  key = first_source;
      }
      string color = termColor(procColor(key));
      printf("%s", color.c_str());
    }

    printf(bformat_string.c_str(),  varname.c_str(), lcnt, 
	   chars, sfirst.c_str(), slast.c_str(), varsources.c_str());

    if(m_use_colors)
      printf("%s", termColor().c_str());
  }

  string start_time = doubleToString(m_report.getMinStartTime(),2);
  string stop_time  = doubleToString(m_report.getMaxStopTime(),2);

  unsigned int digits = m_report.getVarNameMaxLength() + line_digits + 
    char_digits + start_digits + stop_digits;
  string separator = "------------------";
  for(i=0; i<digits; i++)
    separator += "-";
  printf("%s\n", separator.c_str());
  printf("Total variables: %d\n", m_report.size());
  printf("Start/Stop Time: %s / %s\n", start_time.c_str(),
	 stop_time.c_str());
  */
}



//--------------------------------------------------------
// Procedure: appStatReport
//     Notes: 

void EffHandler::appStatReport()
{
  if(m_report.size() == 0)
    return;

  m_report.fillAppStats();
  vector<string> all_sources = m_report.getAllSources();

  unsigned int vsize = all_sources.size();
  if(vsize == 0)
    return;
  
  unsigned int i, max_source_name_length = 0;
  for(i=0; i<vsize; i++) {
    string source = all_sources[i];
    if(source.length() > max_source_name_length)
      max_source_name_length = source.length();
  }

  printf("\n\n\n");

  string app_header = "MOOS Application";
  app_header = padString(app_header, max_source_name_length+6, false);

  string app_dash = "---------------";
  app_dash = padString(app_dash, max_source_name_length+6, false);

  printf("%s  %s  %s  %s  %s\n", app_header.c_str(), "Total Lines", 
	 "Total Chars", "Lines/Total", "Chars/Total");
  printf("%s  -----------  -----------  -----------  -----------\n", 
	 app_dash.c_str());


  for(i=0; i<vsize; i++) {
    string source = all_sources[i];
    double lines  = m_report.getLinesBySource(source);
    double chars  = m_report.getCharsBySource(source);
    double lines_pct = m_report.getLinesPctBySource(source);
    double chars_pct = m_report.getCharsPctBySource(source);

    string s_lines = doubleToString(lines,0);
    string s_chars = doubleToString(chars,0);
    string s_lines_pct = doubleToString((lines_pct*100.0),2);
    string s_chars_pct = doubleToString((chars_pct*100.0),2);

    s_lines = padString(dstringCompact(s_lines), 8, true);
    s_chars = padString(dstringCompact(s_chars), 10, true);
    s_lines_pct = padString(s_lines_pct, 6, true);
    s_chars_pct = padString(s_chars_pct, 6, true);
    source = padString(source, max_source_name_length, false);

    printf("%s           %s   %s       %s       %s\n", 
	   source.c_str(), s_lines.c_str(), 
	   s_chars.c_str(), s_lines_pct.c_str(), s_chars_pct.c_str());
  }  
}

double EffHandler::getParsedDoubleValue(string str, string val){
  // code adapted from M. Benjamin
  vector<string> svector = parseString(str, ',');
  unsigned int i, vsize = svector.size();
  for(i=0; i<vsize; i++) {
    str = svector[i];
    string param = biteString(str, '=');
    if(param == val){
      return  atof(str.c_str());
    }
  }
  return(-9999);
}


void EffHandler::setALogFiles(int argc, char **argv)
{
  // code from alogview (M. Benjamin)
  for(int i=1; i<argc; i++)
    if(strContains(argv[i], ".alog")){
      m_alog_files.push_back(argv[i]);
      cout << "added file: " << argv[i] << endl;
    }
  // why did I have to add a * to line above???
}