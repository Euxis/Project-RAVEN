/*****************************************************************/
/*    NAME: Michael Benjamin, Henrik Schmidt, and John Leonard   */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: NodeRepUtils.cpp                                     */
/*    DATE: Jun 26th 2011                                        */
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

#include "NodeRecordUtils.h"
#include "MBUtils.h"

using namespace std;

//---------------------------------------------------------
// Procedure: handleNodeReport
//   Example: NAME=alpha,TYPE=KAYAK,UTC_TIME=1267294386.51,
//            X=29.66,Y=-23.49,LAT=43.825089, LON=-70.330030, 
//            SPD=2.00, HDG=119.06,YAW=119.05677,DEPTH=0.00,     
//            LENGTH=4.0,MODE=ENGAGED

NodeRecord string2NodeRecord(const string& node_rep_string)
{
  NodeRecord empty_record;
  NodeRecord new_record;

  vector<string> svector = parseString(node_rep_string, ',');
  unsigned int i, vsize = svector.size();
  for(i=0; i<vsize; i++) {
    string param = toupper(stripBlankEnds(biteString(svector[i], '=')));
    string value = stripBlankEnds(svector[i]);
    
    if(param == "NAME")
      new_record.setName(value);
    else if(param == "TYPE")
      new_record.setType(value);
    else if(param == "MODE")
      new_record.setMode(value);
    else if(param == "ALLSTOP")
      new_record.setAllStop(value);
    else if(isNumber(value)) {
      if((param == "UTC_TIME") || (param == "TIME"))
	new_record.setTimeStamp(atof(value.c_str()));
      else if(param == "X")
	new_record.setX(atof(value.c_str()));
      else if(param == "Y")
	new_record.setY(atof(value.c_str()));
      else if(param == "LAT")
	new_record.setLat(atof(value.c_str()));
      else if(param == "LON")
	new_record.setLon(atof(value.c_str()));

      else if(param == "SPD")
	new_record.setSpeed(atof(value.c_str()));
      else if(param == "SPD_OG")
	new_record.setSpeedOG(atof(value.c_str()));

      else if(param == "HDG")
	new_record.setHeading(atof(value.c_str()));
      else if(param == "HDG_OG")
	new_record.setHeadingOG(atof(value.c_str()));

      else if(param == "YAW")
	new_record.setYaw(atof(value.c_str()));
      else if(param == "DEP")
	new_record.setDepth(atof(value.c_str()));
      else if(param == "LENGTH")
	new_record.setLength(atof(value.c_str()));
    }
  }
  
  if(!new_record.valid())
    return(empty_record);

  return(new_record);
}
