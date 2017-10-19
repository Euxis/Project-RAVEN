/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: ObstacleFieldGenerator.cpp                           */
/*    DATE: Oct 18th 2017                                        */
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

#include <iostream>
#include <cstdlib>
#include <cmath>
#include "ObstacleFieldGenerator.h"
#include "MBUtils.h"
#include "AngleUtils.h"
#include "XYFormatUtilsPoly.h"

using namespace std;

//---------------------------------------------------------
// Constructor

ObstacleFieldGenerator::ObstacleFieldGenerator()
{
  m_min_poly_size = 10;
  m_max_poly_size = 10;

  m_min_range = 2;
  m_amount    = 1;
}

//---------------------------------------------------------
// Procedure: setPolygon

bool ObstacleFieldGenerator::setPolygon(string str)
{
  XYPolygon poly = string2Poly(str);

  if(!poly.is_convex())
    return(false);

  m_poly_region = poly;
  return(true);
}

//---------------------------------------------------------
// Procedure: setAmount()

bool ObstacleFieldGenerator::setAmount(string str)
{
  if(!isNumber(str))
    return(false);

  int ival = atoi(str.c_str());
  if(ival <= 0)
    return(false);

  m_amount = (unsigned int)(ival);
  return(true);
}

//---------------------------------------------------------
// Procedure: setMinRange()

bool ObstacleFieldGenerator::setMinRange(string str)
{
  return(setNonNegDoubleOnString(m_min_range, str));
}

//---------------------------------------------------------
// Procedure: setObstacleMinSize()

bool ObstacleFieldGenerator::setObstacleMinSize(string str)
{
  if(!isNumber(str))
    return(false);

  double dval = atof(str.c_str());
  if(dval <= 0)
    return(false);

  m_min_poly_size = dval;
  if(m_max_poly_size < m_min_poly_size)
    m_max_poly_size = m_min_poly_size;

  return(true);
}

//---------------------------------------------------------
// Procedure: setObstacleMaxSize()

bool ObstacleFieldGenerator::setObstacleMaxSize(string str)
{
  if(!isNumber(str))
    return(false);

  double dval = atof(str.c_str());
  if(dval <= 0)
    return(false);

  m_max_poly_size = dval;
  if(m_min_poly_size > m_max_poly_size)
    m_min_poly_size = m_max_poly_size;

  return(true);
}


//---------------------------------------------------------
// Procedure: generate()

bool ObstacleFieldGenerator::generate()
{
  srand(time(NULL));

  bool ok = true;
  for(unsigned int i=0; (ok && (i<m_amount)); i++) {
    ok = ok && generateObstacle(100);
  }

  return(ok);
}

//---------------------------------------------------------
// Procedure: generateObstacle()

bool ObstacleFieldGenerator::generateObstacle(unsigned int tries)
{
  if(!m_poly_region.is_convex())
    return(false);
    
  double minx = m_poly_region.get_min_x();
  double miny = m_poly_region.get_min_y();
  double maxx = m_poly_region.get_max_x();
  double maxy = m_poly_region.get_max_y();

  double xlen = maxx - minx;
  double ylen = maxy - miny;

  for(unsigned int i=0; i<tries; i++) {
  
    int rand_int_x = rand() % 10000;
    int rand_int_y = rand() % 10000;
    
    double rand_pct_x = (double)(rand_int_x) / 10000;
    double rand_pct_y = (double)(rand_int_y) / 10000;

    double rand_x = minx + (rand_pct_x * xlen);
    double rand_y = miny + (rand_pct_y * ylen);

    // Reject if poly center point is not in the overall region
    if(!m_poly_region.contains(rand_x, rand_y))
      continue;

    // Reject if poly center point is in an existing obstacle
    for(unsigned int i=0; i<m_obstacles.size(); i++) {
      if(m_obstacles[i].contains(rand_x, rand_y))
	continue;
    }

    //"radial:: x=val, y=val, radius=val, pts=val, snap=val, label=val"
    string str = "radial:: x=" + doubleToString(rand_x,1); 
    str += ", y=" + doubleToString(rand_y,1);
    str += "radius=10,pts=8,label=ob_" + uintToString(m_obstacles.size());
 
    XYPolygon try_poly = string2Poly(str);
    if(!try_poly.is_convex())
      return(false);
    
    // Reject if poly intersects any existing obstacle
    for(unsigned int i=0; i<m_obstacles.size(); i++) {
      if(m_obstacles[i].intersects(try_poly))
	continue;
    }
    
    // Reject if poly is too close to any existing obstacle
    for(unsigned int i=0; i<m_obstacles.size(); i++) {
      if(m_obstacles[i].dist_to_poly(try_poly) < m_min_range)
	continue;
    }

    // Success!!!
    m_obstacles.push_back(try_poly);
    return(true);
  }
    
  return(false);
}









