/*****************************************************************/
/*    NAME: Michael Benjamin, Henrik Schmidt, and John Leonard   */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: Seglr.cpp                                            */
/*    DATE: Mar 21st, 2015                                       */
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
#include "GeomUtils.h"
#include "Seglr.h" 

using namespace std;

//---------------------------------------------------------------
// Procedure: clear

void Seglr::clear()
{
  m_vx.clear();
  m_vy.clear();
  m_ray_angle = 0;
}

//---------------------------------------------------------------
// Procedure: getVX()

double Seglr::getVX(unsigned int ix) const
{
  if(ix >= m_vx.size())
    return(0);
  return(m_vx[ix]);
}

//---------------------------------------------------------------
// Procedure: getVY()

double Seglr::getVY(unsigned int ix) const
{
  if(ix >= m_vy.size())
    return(0);
  return(m_vy[ix]);
}

//---------------------------------------------------------------
// Procedure: getRayAngle

double Seglr::getRayAngle() const
{
  return(m_ray_angle);
}


//---------------------------------------------------------------
// Procedure: CrossesLine()

bool Seglr::crossesLine(double x1, double y1, double x2, double y2,
			double& ix, double& iy, bool ray_first) const
{
  // Sanity check
  unsigned int vsize = m_vx.size();
  if(vsize == 0)
    return(false);

  // Part 1: If ray-check is first, Check if the ray interects
  if(ray_first) {
    bool crosses = lineRayCross(m_vx[vsize-1], m_vy[vsize-1], m_ray_angle,
				x1, y1, x2, y2, ix, iy);
    if(crosses)
      return(true);
  }

  // Part 2: Check if any of the initial line segments intersect
  for(unsigned int i=0; i<vsize; i++) {
    if((i+1) < vsize) {
      bool crosses = linesCross(m_vx[i], m_vy[i], m_vx[i+1], m_vy[i+1],
				x1, y1, x2, y2, ix, iy);
      if(crosses)
	return(true);
    }
  }

  // Part 3: If ray-check is last, now check if the ray interects
  if(!ray_first) {
    bool crosses = lineRayCross(m_vx[vsize-1], m_vy[vsize-1], m_ray_angle,
				x1, y1, x2, y2, ix, iy);
    if(crosses)
      return(true);
  }

  return(false);
}

//---------------------------------------------------------------
// Procedure: CrossesLine()

bool Seglr::crossesLine(double x1, double y1, double x2, double y2,
			bool ray_first) const
{
  double ix, iy;
  return(crossesLine(x1, y1, x2, y2, ix, iy, ray_first));
}
