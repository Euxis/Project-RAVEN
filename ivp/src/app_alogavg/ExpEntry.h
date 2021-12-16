/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Eng                               */
/*    FILE: ExpEntry.h                                           */
/*    DATE: December 15th, 2021                                  */
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

#ifndef EXP_ENTRY_HEADER
#define EXP_ENTRY_HEADER

#include <vector>

class ExpEntry
{
 public:
  ExpEntry();
  ~ExpEntry() {}

  void setXVal(double x) {m_xval=x;}
  void addYVal(double y);

  double getXVal() const {return(m_xval);}

  double getYAvg();
  double getYMin();
  double getYMax();
  double getYStd();

  unsigned int size() const {return(m_yvals.size());}
  
 protected: 

  double m_xval;

  std::vector<double> m_yvals;

  double m_yavg;
  double m_ymin;
  double m_ymax;
  double m_ystd;

  bool m_yavg_set;
  bool m_ymin_set;
  bool m_ymax_set;
  bool m_ystd_set;
};

#endif
