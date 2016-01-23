/*****************************************************************/
/*    NAME: Michael Benjamin, Henrik Schmidt, and John Leonard   */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: XYFormatUtilsSeglr.cpp                               */
/*    DATE: April 27th, 2015                                     */
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

#include <cmath>
#include <cstdlib>
#include <vector>
#include "XYFormatUtilsSeglr.h"
#include "MBUtils.h"

using namespace std;

XYSeglr string2Seglr(const string& str)
{
  XYSeglr null_seglr;
  XYSeglr new_xy_seglr;

  vector<double> xpts;
  vector<double> ypts;
  double ray_angle = 0;
  
  string rest = str;

  while(rest != "") {
    string left = biteStringX(rest, '=');

    if(left == "pts") {
      string pstr = biteStringX(rest, '}');
      
      // Empty set of points is an error
      if(pstr == "")
	return(null_seglr);

      // Points should begin with an open brace (but discard now)
      if(pstr[0] != '{') 
	return(null_seglr);
      else
	pstr = pstr.substr(1);

      // If more components after pts={}, then it should begin w/ comma
      if(rest != "") {
	if(rest[0] != ',')
	  return(null_seglr);
	else
	  rest = rest.substr(1);
      }

      vector<string> svector = parseString(pstr, ':');
      for(unsigned int i=0; i<svector.size(); i++) {
	string vertex = stripBlankEnds(svector[i]);
	string xstr = biteStringX(vertex, ',');
	string ystr = biteStringX(vertex, ',');
	  
	if(!isNumber(xstr) || !isNumber(ystr))
	  return(null_seglr);
	double xval = atof(xstr.c_str());
	double yval = atof(ystr.c_str());
	xpts.push_back(xval);
	ypts.push_back(yval);
      }
    }
    else if(left == "ray") {
      string right = biteStringX(rest, ',');
      double angle = atof(right.c_str());
      ray_angle = angle;
    }      
    else {
      string right = biteStringX(rest, ',');
      new_xy_seglr.set_param(left, right);
    }
  }	  				  

  Seglr seglr;
  for(unsigned int i=0; i<xpts.size(); i++) 
    seglr.addVertex(xpts[i], ypts[i]);
  seglr.setRayAngle(ray_angle);
  
  new_xy_seglr.setSeglr(seglr); 

  return(new_xy_seglr);
}
