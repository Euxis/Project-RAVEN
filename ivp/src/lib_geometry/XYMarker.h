/*****************************************************************/
/*    NAME: Michael Benjamin, Henrik Schmidt, and John Leonard   */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: XYMarker.h                                           */
/*    DATE: May 12th, 2011                                       */
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
 
#ifndef XY_MARKER_HEADER
#define XY_MARKER_HEADER

#include "XYObject.h"
#include "ColorPack.h"

class XYMarker : public XYObject {
public:
  XYMarker();
  virtual ~XYMarker() {}

  // Setters
  bool    set_type(const std::string&);
  void    set_vx(double v)       {m_x=v;m_x_set=true;}
  void    set_vy(double v)       {m_y=v;m_y_set=true;}
  void    set_width(double v);
  void    set_range(double v);
 
  // Getters
  double      get_vx() const     {return(m_x);}
  double      get_vy() const     {return(m_y);}
  double      get_width() const  {return(m_width);}
  double      get_range() const  {return(m_range);}
  std::string get_type() const   {return(m_type);}


  // Analyzers
  bool        is_set_x() const     {return(m_x_set);};
  bool        is_set_y() const     {return(m_y_set);};
  bool        is_set_range() const {return(m_range_set);};
  bool        is_set_width() const {return(m_width_set);};

  std::string get_spec(std::string s="") const;

protected:
  double       m_x;
  double       m_y;
  double       m_width;
  double       m_range;
  std::string  m_type;

  bool         m_x_set;
  bool         m_y_set;
  bool         m_width_set;
  bool         m_range_set;
};

#endif





