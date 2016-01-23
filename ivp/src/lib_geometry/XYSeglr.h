/*****************************************************************/
/*    NAME: Michael Benjamin, Henrik Schmidt, and John Leonard   */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: XYSeglr.h                                            */
/*    DATE: Apr 27, 2015                                         */
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
 
#ifndef XY_SEGLR_HEADER
#define XY_SEGLR_HEADER

#include <string>
#include "XYObject.h"
#include "Seglr.h"

class XYSeglr : public XYObject {
public:
  XYSeglr() {};
  XYSeglr(Seglr seglr) {m_seglr=seglr;};
  virtual ~XYSeglr() {};

  void   setSeglr(Seglr seglr) {m_seglr=seglr;};
  void   clear();
  
  unsigned int size() const {return(m_seglr.size());};

  double getMinX() const {return(m_seglr.getMinX());};
  double getMaxX() const {return(m_seglr.getMaxX());};
  double getMinY() const {return(m_seglr.getMinY());};
  double getMaxY() const {return(m_seglr.getMaxY());};

  double getVX(unsigned int ix) const {return(m_seglr.getVX(ix));}
  double getVY(unsigned int ix) const {return(m_seglr.getVY(ix));}
  double getRayAngle() const {return(m_seglr.getRayAngle());}

  double getRayBaseX() const;
  double getRayBaseY() const;

  double getAvgX() const {return((getMaxX()-getMinX())/2);};
  double getAvgY() const {return((getMaxY()-getMinY())/2);};
  
  std::string getSpec(int vertex_precision=1) const;

protected:
  Seglr m_seglr;
};

#endif