/*****************************************************************/
/*    NAME: Michael Benjamin and John Leonard                    */
/*    ORGN: NAVSEA Newport RI and MIT Cambridge MA               */
/*    FILE: Populator_BehaviorSet.cpp                            */
/*    DATE:                                                      */
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
#ifdef _WIN32
#pragma warning(disable : 4786)
#endif

#ifdef _WIN32
#define strncasecmp _strnicmp
#endif

#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "MBUtils.h"
#include "FileBuffer.h"
#include "Populator_BehaviorSet.h"

// CORE Behaviors
#include "BHV_Waypoint.h"
#include "BHV_SimpleWaypoint.h"
#include "BHV_Loiter.h"
#include "BHV_OpRegion.h"
#include "BHV_ConstantDepth.h"
#include "BHV_ConstantHeading.h"
#include "BHV_ConstantSpeed.h"
#include "BHV_PeriodicSpeed.h"
#include "BHV_PeriodicSurface.h"
#include "BHV_Trail.h"
#include "BHV_Shadow.h"
#include "BHV_Timer.h"
#include "BHV_StationKeep.h"
#include "BHV_RStationKeep.h"
#include "BHV_CutRange.h"
#include "BHV_AvoidCollision.h"
#include "BHV_AvoidObstacles.h"
#include "BHV_GoToDepth.h"
#include "BHV_MemoryTurnLimit.h"
#include "BHV_Attractor.h"
#include "BHV_RubberBand.h"

using namespace std;

//-------------------------------------------------------------
// Procedure: Constructor

Populator_BehaviorSet::Populator_BehaviorSet(IvPDomain g_domain,
					     InfoBuffer *g_buffer)
{
  // cout << "Populator_BehaviorSet::Constructor()" << endl;
  // g_domain.print();
  // cout << endl << endl;

  domain             = g_domain;
  info_buffer        = g_buffer;
  define_mode        = 0;
  open_behavior_mode = false;
  ok                 = false;
}

//-------------------------------------------------------------
// Procedure: populate

BehaviorSet *Populator_BehaviorSet::populate(set<string> bhv_files)
{
  unsigned int i;
  set<string>::const_iterator p;
  for(p=bhv_files.begin(); p!=bhv_files.end(); p++) {

    string filename = *p;

    FILE *f = fopen(filename.c_str(), "r");
    
    if(!f) 
      cout << "Could not find File: " << filename << endl;
    else {
      cout << "Successfully found file: " << filename << endl;
      fclose(f);
      
      vector<string> file_vector = fileBufferSlash(filename);
      unsigned int lineCount = file_vector.size();
    
      for(i=0; i<lineCount; i++) {
	string line = stripBlankEnds(file_vector[i]);
	
	if((line.length()!=0) && ((line)[0]!='#')) {
	  int res = handleLine(line);
	  if(!res) {
	    cout << " Problem with line " << i+1;
	    cout << " in the BehaviorSet file: " << filename << endl;
	    cout << line << endl;
	    return(0);
	  }
	}
      }
    }
  }

  if(behaviors.size() == 0) 
    return(0);
  else {
    BehaviorSet *bset = new BehaviorSet;
    for(i=0; i<behaviors.size(); i++) {
      behaviors[i]->setInfoBuffer(info_buffer);
      bset->addBehavior(behaviors[i]);
    }
    for(i=0; i<initial_vars.size(); i++)
      bset->addInitialVar(initial_vars[i]);
    for(i=0; i<default_vars.size(); i++)
      bset->addDefaultVar(default_vars[i]);
    return(bset);
  }
}

//-------------------------------------------------------------
// Procedure: populate
//      Note: A convenience function when the behaviors are 
//            given by only one file.

BehaviorSet *Populator_BehaviorSet::populate(string filestr)
{
  set<string> bhv_files;
  bhv_files.insert(filestr);
  return(populate(bhv_files));
}

//----------------------------------------------------------
// Procedure: handleLine
//   Returns: 1 if all OK
//            0 otherwise
//
bool Populator_BehaviorSet::handleLine(string line)
{
  // Comments are anything to the right of a "#" or "//"
  line = stripComment(line, "//");
  line = stripComment(line, "#");
  line = stripBlankEnds(line);
  if(line.size() == 0)  // Either blank or comment line
    return(true);  
  
  if(!strncasecmp("initialize", line.c_str(), 10))
    line = findReplace(line, "initialize", "initial_var = ");

  vector<string> svector = chompString(line, '=');

  string left  = stripBlankEnds(svector[0]);
  string right = stripBlankEnds(svector[1]);

  if(right == "") {
    if((left != "{") && (left != "}")) {
      cout << "PROBLEM #1" << endl;
      return(false);
    }

    if(left == "{") {
      if(define_mode == 0) {
	if(open_behavior_mode)
	  define_mode = 1;
	return(true);
      }
      else { 
	cout << "PROBLEM #2" << endl;
	return(false);
      }
    }
      
    if(left == "}") {
      open_behavior_mode = false;
      define_mode = 0;
      return(true);
    }
  }
  
  // Handle initialization lines
  string str = tolower(left);
  if((str == "initial_var") && (define_mode == 0)) {
    right = findReplace(right, ',', '=');
    vector<string> dvector = parseString(right, '=');
    if(dvector.size() != 2) {
      cout << "PROBLEM #2" << endl;
      return(false);
    }
    VarDataPair msg(dvector[0], dvector[1], "auto");
    initial_vars.push_back(msg);
    return(true);
  }

  // Handle default lines
  if((str == "default") && (define_mode == 0)) {
    vector<string> dvector = parseString(right, '=');
    if(dvector.size() != 2)
      return(false);
    VarDataPair msg(dvector[0], dvector[1], "auto");
    default_vars.push_back(msg);
    return(true);
  }
  
  if(define_mode == 0) {
    if(left != "Behavior") {
      if(open_behavior_mode)
	return(false);
      else
	return(true);
    }
    else
      open_behavior_mode = true;

    IvPBehavior *bhv = initializeBehavior(right); 

    if(bhv)
      behaviors.push_back(bhv);
    return(bhv!=0);
  }
  
  if(define_mode == 1) {
    left = tolower(left);
    IvPBehavior *bhv = behaviors[behaviors.size()-1];
    bool result = bhv->setParam(left.c_str(), right.c_str());
    return(result);
  }

  return(false);
}


//----------------------------------------------------------
// Procedure: initializeBehavior

IvPBehavior* Populator_BehaviorSet::initializeBehavior(string bhv_name)
{
  IvPBehavior *bhv = 0;      

  if(bhv_name == "BHV_OpRegion")
    bhv = new BHV_OpRegion(domain);
  else if(bhv_name == "BHV_SimpleWaypoint")   
    bhv = new BHV_SimpleWaypoint(domain);
  else if(bhv_name == "BHV_Waypoint")   
    bhv = new BHV_Waypoint(domain);
  else if(bhv_name == "BHV_ConstantSpeed")     
    bhv = new BHV_ConstantSpeed(domain);
  else if(bhv_name == "BHV_Trail")      
    bhv = new BHV_Trail(domain);
  else if(bhv_name == "BHV_ConstantDepth")      
    bhv = new BHV_ConstantDepth(domain);
  else if(bhv_name == "BHV_ConstantHeading")      
    bhv = new BHV_ConstantHeading(domain);
  else if(bhv_name == "BHV_Loiter")     
    bhv = new BHV_Loiter(domain);
  else if(bhv_name == "BHV_StationKeep")     
    bhv = new BHV_StationKeep(domain);
  else if(bhv_name == "BHV_RStationKeep")     
    bhv = new BHV_RStationKeep(domain);
  else if(bhv_name == "BHV_Timer")     
    bhv = new BHV_Timer(domain);
  else if(bhv_name == "BHV_Shadow")     
    bhv = new BHV_Shadow(domain);
  else if(bhv_name == "BHV_CutRange")   
    bhv = new BHV_CutRange(domain);
  else if(bhv_name == "BHV_AvoidCollision") 
    bhv = new BHV_AvoidCollision(domain);
  else if(bhv_name == "BHV_AvoidObstacles") 
    bhv = new BHV_AvoidObstacles(domain);
  else if(bhv_name == "BHV_PeriodicSpeed") 
    bhv = new BHV_PeriodicSpeed(domain);
  else if(bhv_name == "BHV_PeriodicSurface") 
    bhv = new BHV_PeriodicSurface(domain);
  else if(bhv_name == "BHV_GoToDepth")      
    bhv = new BHV_GoToDepth(domain);
  else if(bhv_name == "BHV_MemoryTurnLimit")      
    bhv = new BHV_MemoryTurnLimit(domain);
  else if(bhv_name == "BHV_Attractor")      
    bhv = new BHV_Attractor(domain);
  else if(bhv_name == "BHV_RubberBand")      
    bhv = new BHV_RubberBand(domain);

  return(bhv);
}







