/*****************************************************************/
/*    NAME: Michael Benjamin and John Leonard                    */
/*    ORGN: NAVSEA Newport RI and MIT Cambridge MA               */
/*    FILE: XYPolygon.cpp                                        */
/*    DATE: Apr 20th, 2005                                       */
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
#include <math.h>
#include "XYPolygon.h"
#include "MBUtils.h"
#include "GeomUtils.h"
#include "AngleUtils.h"

using namespace std;

#ifdef _WIN32
#define strncasecmp _strnicmp
#endif


//---------------------------------------------------------------
// Procedure: Constructor

XYPolygon::XYPolygon()
{
  convex_state = false;
}

//---------------------------------------------------------------
// Procedure: add_vertex
//      Note: A call to "determine_convexity()" is made since this
//            operation may result in a change in the convexity.

bool XYPolygon::add_vertex(double x, double y)
{
  XYSegList::add_vertex(x,y);
  side_xy.push_back(-1);

  // With new vertex, we don't know if the new polygon is valid
  determine_convexity();
  return(convex_state);
}

//---------------------------------------------------------------
// Procedure: initialize
//      Note: A call to "determine_convexity()" is made since this
//            operation may result in a change in the convexity.

bool XYPolygon::initialize(string str)
{
  str = stripBlankEnds(str);
  if(!strncasecmp("radial:", str.c_str(), 7))
    return(init_radial(str.c_str()+7));

  if(!strncasecmp("arc:", str.c_str(), 4))
    return(init_arc(str.c_str()+4));

  bool ok;
  if(!strncasecmp("polygon:", str.c_str(), 8))
    ok = XYSegList::initialize(str.c_str()+8);
  else if(!strncasecmp("poly:", str.c_str(), 5))
    ok = XYSegList::initialize(str.c_str()+5);
  else
    ok = XYSegList::initialize(str);

  for(int i=0; i<size(); i++) 
    side_xy.push_back(-1);

  if(!ok)
    convex_state = false;
  else
    determine_convexity();

  return(convex_state);
}


//---------------------------------------------------------------
// Procedure: init_radial

/// Initializes a polygon that approximates a circle.
/// The format of the string is 
/// "radial:x_val,y_val,radius,num_points,[snap_value=0],[label]
bool XYPolygon::init_radial(string str)
{
  if(!strncasecmp("radial:", str.c_str(), 7))
    str = str.c_str()+7;

  clear();
  vector<string> svector = parseString(str, ',');
  int vsize = svector.size();

  if((vsize < 4) || (vsize > 6))
    return(false);

  double px   = atof(svector[0].c_str());
  double py   = atof(svector[1].c_str());
  double prad = atof(svector[2].c_str());
  double ppts = atof(svector[3].c_str());

  if(prad <= 0)
    return(false);

  double snap_value = 0;
  if(vsize == 5)
    snap_value = atof(svector[4].c_str());
    
  if(vsize == 6) // Label present
  	set_label(svector[5]);

  double delta = 360.0 / ppts;
  for(double deg=(delta/2); deg<360; deg+=delta) {
    double new_x, new_y;
    projectPoint(deg, prad, px, py, new_x, new_y);
    add_vertex(new_x, new_y);
  }
  if(snap_value >= 0)
    apply_snap(snap_value);

  determine_convexity();
  return(convex_state);
}

//---------------------------------------------------------------
// Procedure: init_arc

bool XYPolygon::init_arc(string str)
{
  if(!strncasecmp("arc:", str.c_str(), 7))
    str = str.c_str()+7;

  clear();
  vector<string> svector = parseString(str, ',');
  int vsize = svector.size();
  
  if((vsize < 6) || (vsize > 7))
    return(false);

  double px   = atof(svector[0].c_str());
  double py   = atof(svector[1].c_str());
  double prad = atof(svector[2].c_str());
  double lang = atof(svector[3].c_str());
  double rang = atof(svector[4].c_str());
  double ppts = atof(svector[5].c_str());

  lang = angle360(lang);
  rang = angle360(rang);

  double snap_value = 0;
  if(vsize == 7)
    snap_value = atof(svector[6].c_str());

  double delta = 360.0 / ppts;
  double new_x, new_y;

  // Case 1: Arc does not cross the ZERO degree mark
  if((rang >=0) && (rang <= lang)) {
    for(double deg=lang; deg>rang; deg-=delta) {
      projectPoint(deg, prad, px, py, new_x, new_y);
      add_vertex(new_x, new_y);
    }
    projectPoint(rang, prad, px, py, new_x, new_y);
    add_vertex(new_x, new_y);
  }
  // Case 2: Arc DOES cross the ZERO degree mark
  else {
    for(double deg=rang; ((deg>=rang)||(deg<lang)); deg+=delta) {
      deg = angle360(deg);
      projectPoint(deg, prad, px, py, new_x, new_y);
      add_vertex(new_x, new_y);
    }
    projectPoint(lang, prad, px, py, new_x, new_y);
    add_vertex(new_x, new_y);
  }

  if(snap_value >= 0)
    apply_snap(snap_value);

  determine_convexity();
  return(convex_state);
}

//---------------------------------------------------------------
// Procedure: alter_vertex
//   Purpose: Given a new vertex, find the existing vertex that is
//            closest, and replace it with the new one.
//      Note: A call to "determine_convexity()" is made since this
//            operation may result in a change in the convexity.

bool XYPolygon::alter_vertex(double x, double y)
{
  XYSegList::alter_vertex(x,y);

  determine_convexity();
  return(convex_state);
}

//---------------------------------------------------------------
// Procedure: delete_vertex
//   Purpose: Given a new vertex, find the existing vertex that is
//            closest, and delete it.
//      Note: A call to "determine_convexity()" is made since this
//            operation may result in a change in the convexity.

bool XYPolygon::delete_vertex(double x, double y)
{
  int vsize = vertex_x.size();

  if(vsize == 0)
    return(false);

  int i, ix = closest_vertex(x, y); 

  vector<int>  new_xy;
  
  for(i=0; i<ix; i++)
    new_xy.push_back(side_xy[i]);
  for(i=ix+1; i<vsize; i++)
    new_xy.push_back(side_xy[i]);
  
  side_xy  = new_xy;

  XYSegList::delete_vertex(x,y);

  determine_convexity();
  return(convex_state);
}

//---------------------------------------------------------------
// Procedure: insert_vertex
//   Purpose: Given a new vertex, find the existing segment that is
//            closest, and add the vertex between points
//      Note: A call to "determine_convexity()" is made since this
//            operation may result in a change in the convexity.

bool XYPolygon::insert_vertex(double x, double y)
{
  int vsize = vertex_x.size();

  if(vsize <= 1)
    return(add_vertex(x,y));

  int i, ix = XYPolygon::closest_segment(x, y); 

  vector<int> new_xy;
  
  for(i=0; i<=ix; i++) 
    new_xy.push_back(side_xy[i]);
  new_xy.push_back(2);
  for(i=ix+1; i<vsize; i++)
    new_xy.push_back(side_xy[i]);
  
  side_xy  = new_xy;

  XYSegList::insert_vertex(x,y);

  determine_convexity();
  return(convex_state);
}

//---------------------------------------------------------------
// Procedure: clear

void XYPolygon::clear()
{
  XYSegList::clear();
  side_xy.clear();
  convex_state = false;
}


//---------------------------------------------------------------
// Procedure: is_clockwise()
//      Note: Determine if the ordering of points in the internal
//            vector of stored points constitutes a clockwise walk
//            around the polygon. Algorithm base on progression of
//            relative angle from the center. Result is somewhat
//            undefined if the polygon is not convex. If it is 
//            "nearly" convex, it should still be accurate.

bool XYPolygon::is_clockwise() const
{
  int vsize = vertex_x.size();

  if(vsize < 3)
    return(false);

  int inc_count = 0;
  int dec_count = 0;

  double cx = get_center_x();
  double cy = get_center_y();

  for(int i=0; i<vsize; i++) {
    int j = i+1; 
    if(j == vsize)
      j = 0;
    double relative_angle_1 = relAng(cx, cy, vertex_x[i], vertex_y[i]);
    double relative_angle_2 = relAng(cx, cy, vertex_x[j], vertex_y[j]);
    if(relative_angle_2 > relative_angle_1)
      inc_count++;
    else
      dec_count++;
  }

  bool clockwise;
  if(inc_count > dec_count)
    clockwise = true;
  else
    clockwise = false;

  return(clockwise);
}

//---------------------------------------------------------------
// Procedure: apply_snap
//      Note: A call to "determine_convexity()" is made since this
//            operation may result in a change in the convexity.
//      Note: Will not allow a snap that changes the convexity state
//            from TRUE to FALSE
//   Returns: true if the snap was successfully 

bool XYPolygon::apply_snap(double snapval)
{
  vector<double> tmp_vertex_x = vertex_x;
  vector<double> tmp_vertex_y = vertex_y;

  // Determine if it is convex prior to applying the snapval
  bool start_convex = is_convex();

  XYSegList::apply_snap(snapval);
  determine_convexity();
  if(is_convex() || !start_convex)
    return(true);
  else {
    vertex_x = tmp_vertex_x;
    vertex_y = tmp_vertex_y;
    determine_convexity();
    return(false);
  }
}

//---------------------------------------------------------------
// Procedure: reverse
//      Note: A call to "determine_convexity()" is made since this
//            operation needs to have side_xy[i] reset for each i.

void XYPolygon::reverse()
{
  XYSegList::reverse();
  determine_convexity();
}

//---------------------------------------------------------------
// Procedure: rotate
//      Note: A call to "determine_convexity()" is made since this
//            operation needs to have side_xy[i] reset for each i.


void XYPolygon::rotate(double val)
{
  XYSegList::rotate(val);
  determine_convexity();
}


//---------------------------------------------------------------
// Procedure: contains

bool XYPolygon::contains(double x, double y) const
{
  if(!convex_state)
    return(false);

  int vsize = vertex_x.size();
  
  if(vsize == 0)
    return(false);

  double x1,y1,x2,y2;
  for(int ix=0; ix<vsize; ix++) {

    x1 = vertex_x[ix];
    y1 = vertex_y[ix];
    
    int ixx = ix+1;
    if(ix == vsize-1)
      ixx = 0;
    
    x2 = vertex_x[ixx];
    y2 = vertex_y[ixx];

    int vside = side(x1, y1, x2, y2, x, y);
    if((vside != 2) && (vside != side_xy[ix]))
      return(false);
  }
  return(true);
}

//---------------------------------------------------------------
// Procedure: intersects

bool XYPolygon::intersects(const XYPolygon &poly) const
{
  int this_size = vertex_x.size();
  int poly_size = poly.size();
  
  if(this_size == 0)
    return(false);
  if(poly_size == 0)
    return(false);

  // First check that no vertices from "this" polygon are
  // contained in the given polygon
  int i;
  for(i=0; i<this_size; i++) {
    double x = vertex_x[i];
    double y = vertex_y[i];
    if(poly.contains(x, y))
      return(true);
  }
  // Then check that no vertices from the given polygon are
  // contained in "this" polygon
  for(i=0; i<poly_size; i++) {
    double x = poly.get_vx(i);
    double y = poly.get_vy(i);
    if(this->contains(x, y))
      return(true);
  }

  // Then check that no segments from "this" polygon intersect
  // the given polygon
  for(i=0; i<this_size; i++) {
    double x1 = this->get_vx(i);
    double y1 = this->get_vy(i);
    double x2 = this->get_vx(0);
    double y2 = this->get_vy(0);
    if((i+1) < this_size) {
      x2 = this->get_vx(i+1);
      y2 = this->get_vy(i+1);
    }
    if(poly.seg_intercepts(x1,y1,x2,y2))
      return(true);
  }

  return(false);
}


//---------------------------------------------------------------
// Procedure: dist_to_poly

double XYPolygon::dist_to_poly(double px, double py) const
{
  int vsize = vertex_x.size();
  
  if(vsize == 0)
    return(-1);

  if(vsize == 1)
    return(distPointToPoint(px, py, vertex_x[0], vertex_y[0]));
  
  if(vsize == 2)
    return(distPointToSeg(vertex_x[0], vertex_y[0], 
			  vertex_x[1], vertex_y[1], px, py)); 
	   
  // Distance to poly is given by the shortest distance to any
  // one of the edges.
  double x1,y1,x2,y2;
  double dist;
  for(int ix=0; ix<vsize; ix++) {

    x1 = vertex_x[ix];
    y1 = vertex_y[ix];
    
    int ixx = ix+1;
    if(ix == vsize-1)
      ixx = 0;
    
    x2 = vertex_x[ixx];
    y2 = vertex_y[ixx];

    double idist = distPointToSeg(vertex_x[ix], vertex_y[ix], 
				  vertex_x[ixx], vertex_y[ixx], 
				  px, py); 
    if((ix==0) || (idist < dist))
      dist = idist;
  }

  return(dist);
}

//---------------------------------------------------------------
// Procedure: dist_to_poly
//      Note: Determine the distance between the line segment given
//            by x3,y3,x4,y4 to the polygon. An edge-by-edge check
//            is performed and the minimum returned.

double XYPolygon::dist_to_poly(double x3, double y3, 
			       double x4, double y4) const
{
  int vsize = vertex_x.size();
  
  if(vsize == 0)
    return(-1);
  
  if(vsize == 1)
    return(distPointToSeg(x3,y3,x4,y4, vertex_x[0], vertex_y[0]));
  
  if(vsize == 2)
    return(distSegToSeg(vertex_x[0], vertex_y[0], 
			vertex_x[1], vertex_y[1], x3,y3,x4,y4)); 
	   
  // Distance to poly is given by the shortest distance to any
  // one of the edges.
  double x1,y1,x2,y2;
  double dist;
  for(int ix=0; ix<vsize; ix++) {

    x1 = vertex_x[ix];
    y1 = vertex_y[ix];
    
    int ixx = ix+1;
    if(ix == vsize-1)
      ixx = 0;
    
    x2 = vertex_x[ixx];
    y2 = vertex_y[ixx];

    double idist = distSegToSeg(vertex_x[ix], vertex_y[ix], 
				vertex_x[ixx], vertex_y[ixx], 
				x3, y3, x4, y4); 
    if((ix==0) || (idist < dist))
      dist = idist;
  }
  
  return(dist);
}

//---------------------------------------------------------------
// Procedure: dist_to_poly
//      Note: Determine the distance between the point given by px,py
//            to the polygon along a given angle. An edge-by-edge check
//            is performed and the minimum returned. 
//   Returns: -1 if given ray doesn't intersect the polygon

double XYPolygon::dist_to_poly(double px, double py, double angle) const 
{
  int vsize = vertex_x.size();
  
  if(vsize == 0)
    return(-1);
  
  if(vsize == 1)
    return(distPointToSeg(vertex_x[0], vertex_y[0], 
			  vertex_x[0], vertex_y[0], px,py,angle)); 
  
  if(vsize == 2)
    return(distPointToSeg(vertex_x[0], vertex_y[0], 
			  vertex_x[1], vertex_y[1], px,py,angle)); 
	   
  // Distance to poly is given by the shortest distance to any
  // one of the edges.
  double dist = -1;
  bool   first_hit = true;

  for(int ix=0; ix<vsize; ix++) {

    double x1 = vertex_x[ix];
    double y1 = vertex_y[ix];
    
    int ixx = ix+1;
    if(ix == vsize-1)
      ixx = 0;
    
    double x2 = vertex_x[ixx];
    double y2 = vertex_y[ixx];

    double idist = distPointToSeg(x1,y1,x2,y2,px,py, angle); 
    if(idist != -1)
      if(first_hit || (idist < dist)) {
	dist = idist;
	first_hit = false;
      }
  }
  
  return(dist);
}

//---------------------------------------------------------------
// Procedure: seg_intercepts
//   Purpose: Return true if the given segment intercepts the 
//              polygon. Checks are made whether the segment crosses
//              any of the polygon edges. Intersection is also true
//              if the segment is entirely within the polygon.

bool XYPolygon::seg_intercepts(double x1, double y1, 
			       double x2, double y2) const
{
  int vsize = vertex_x.size();
  if(vsize == 0)
    return(false);

  double x3,y3,x4,y4;

  if(vsize == 1) {
    x3 = x4 = vertex_x[0];
    y3 = y4 = vertex_y[0];
    return(segmentsCross(x1,y1,x2,y2,x3,y3,x4,y4));
  }

  // Special case 2 vertices, otherwise the single edge will be checked
  // twice if handled by the general case.
  if(vsize == 2) {
    x3 = vertex_x[0];
    y3 = vertex_y[0];
    x4 = vertex_x[1];
    y4 = vertex_y[1];
    return(segmentsCross(x1,y1,x2,y2,x3,y3,x4,y4));
  }

  // Now handle the general case of more than two vertices
  // First check if one of the ends of the segment are contained
  // in the polygon

  if(contains(x1,y1) || contains(x2,y2))
    return(true);

  // Next check if the segment intersects any of the polgyon edges.
  for(int ix=0; ix<vsize; ix++) {
    int ixx = ix+1;
    if(ix == vsize-1)
      ixx = 0;
    x3 = vertex_x[ix];
    y3 = vertex_y[ix];
    x4 = vertex_x[ixx];
    y4 = vertex_y[ixx];

    bool result = segmentsCross(x1,y1,x2,y2,x3,y3,x4,y4);
    if(result == true)
      return(true);
  }
  return(false);
}

//---------------------------------------------------------------
// Procedure: vertex_is_viewable
//   Purpose: Determine if the line segment given by the vertex ix, 
//            and the point x1,y1, intersects the polygon *only*
//            at the vertex. If so, we say that the vertex is 
//            "viewable" from the given point.
//      Note: We return false if the given point is contained in 
//            the polygon


bool XYPolygon::vertex_is_viewable(int ix, double x1, double y1) const
{
  //cout << "Checking if Vertex " << ix << " is viewable -----" << endl;

  int vsize = vertex_x.size();
  if(vsize == 0)
    return(false);

  // Simple Range check
  if((ix < 0) || (ix > vsize-1))
    return(false);

  // Special case, poly has one vertex, viewable from any point.
  if(vsize == 1)
    return(true);

  double x2,y2;
  x2 = vertex_x[ix];
  y2 = vertex_y[ix];

  // Special case, the query point and query vertex are the same
  if((x1==x2) && (y1==y2))
    return(true);

  // Special case, the query point is contained by (or *on*) the
  // polygon, return false. (except of course for the special case
  // handled above where the query vertex and point are the same)
  if(contains(x1,y1))
    return(false);

  // Special case, 2 vertices. Tricky since the general case does not
  // properly handle a point that lays on the line, but not the line
  // segment, given by the one edge in the polygon.
  if(vsize == 2) {
    int ixx = 0;  
    if(ix == 0)   // set index of the "other" vertex.
      ixx = 1;

    double x = vertex_x[ixx];
    double y = vertex_y[ixx];
    // if the other vertex point is on the query line segment, false
    if(segmentsCross(x1,y1,x2,y2,x,y,x,y))
      return(false);
    return(true);
  }

  // Now handle the general case of more than two vertices
  // Next check how many polygon edges intersect the query segment.
  // Answer should be at least two since the query segment will 
  // interesct the two edges that share the query vertex. 
  // If the query segment intersects more, it must have passed thru
  // the polygon, so we declare false.

  int count = 0;
  double x3,y3,x4,y4;
  for(int i=0; ((i<vsize) && (count <= 2)); i++) {
    int j = i+1;
    if(i == vsize-1)
      j = 0;
    x3 = vertex_x[i];
    y3 = vertex_y[i];
    x4 = vertex_x[j];
    y4 = vertex_y[j];

    bool res = segmentsCross(x1,y1,x2,y2,x3,y3,x4,y4);
    //cout << "Cross Result segment " << i << ": " << res << endl;
    if(res)
      count++;
   }

  //cout << "   Cross Count: " << count << endl;

  if(count > 2)
    return(false);
  else
    return(true);
}

//---------------------------------------------------------------
// Procedure: side
//   Purpose: determines which "side" of the line given by x1,y1 
//            x2,y2 the point x3,y3 lies on. Returns either 0, 1, 2. 
//            0 indicates the point is below.
//            1 indicates the point is above.
//            2 indicates the point is on the line.
//            For vertical lines, we declare the right is "below".
//            If the given line is a point, we say the point is on the line
//
//             x            x        
//            /             |        
//       1   /  0         1 | 0        
//          /               |        
//         /                |        
//        x                 x          
//                            

int XYPolygon::side(double x1, double y1, double x2, double y2,
		    double x3, double y3) const
{
  // Handle special cases
  if(x1 == x2) {
    if(y1 == y2)
      return(2);
    else {
      if(x3 >  x1) return(0);
      if(x3 <  x1) return(1);
      if(x3 == x1) return(2);
    }
  }
    
  // Find the line equation y = mx + b
  double rise = y2 - y1;
  double run  = x2 - x1;
  double m    = rise / run;
  double b    = y2 - (m * x2);

  // calculate the value of y on the line for x3
  double y = (m * x3) + b;

  // Determine which side the point lies on
  if(y > y3)  
    return(0);
  else if(y < y3)  
    return(1);
  else  // if(y == y3) 
    return(2);
}

//---------------------------------------------------------------
// Procedure: set_side
//      Note: An edge given by index ix is the edge from ix to ix+1
//            When ix is the last vertex, the edge is from ix to 0
//      Note: If an edge in a polygon is valid, all other vertices
//            in the polygon on are the same side. This function
//            determines which side that is. 
//   Returns: 0 if all points are on the 0 side
//            1 if all points are on the 1 side
//            -1 if not all points are on the same side, OR, if all
//               points are *on* the segment.
//      Note: This function serves as more than a convexity test. 
//            The side of each each is an intermediate value that
//            is used in the dist_to_poly() and contains() function.

void XYPolygon::set_side(int ix)
{
  int vsize = vertex_x.size();
  if((ix < 0) || (ix >= vsize))
    return;

  // Handle special cases
  if(vsize == 1) side_xy[0] = -1;
  if(vsize == 2) side_xy[1] = -1;
  if(vsize <= 2)
    return;

  double x1,y1,x2,y2,x3,y3;

  x1 = vertex_x[ix];
  y1 = vertex_y[ix];

  int ixx = ix+1;
  if(ix == vsize-1)
    ixx = 0;

  x2 = vertex_x[ixx];
  y2 = vertex_y[ixx];

  side_xy[ix] = -1;

  bool fresh = true;
  for(int j=0; j<vsize; j++) {
    if((j!=ix) && (j!=ixx)) {
      x3 = vertex_x[j];
      y3 = vertex_y[j];
      int iside = side(x1,y1,x2,y2,x3,y3);

      if(iside != 2) {
	if(fresh) {
	  side_xy[ix] = iside;
	  fresh = false;
	}
	else
	  if(iside != side_xy[ix]) {
	    side_xy[ix] = -1;
	  }
      }
    }
  }
}

//---------------------------------------------------------------
// Procedure: determine_convexity
//   Purpose: determine whether the object represents a convex 
//            polygon. We declare that a polygon is *not* convex
//            unless it contains at least three points. 
//            Most of the work is done by the calls to set_side().
//

void XYPolygon::determine_convexity()
{
  for(int i=0; i<size(); i++)
    set_side(i);

  convex_state = (size() >= 3);
  for(int i=0; i<size(); i++)
    convex_state = convex_state && (side_xy[i] != -1);
}









