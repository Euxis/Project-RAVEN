/*****************************************************************/
/*    NAME: Michael Benjamin and John Leonard                    */
/*    ORGN: NAVSEA Newport RI and MIT Cambridge MA               */
/*    FILE: ModeEntry.cpp                                        */
/*    DATE: Dec 26th, 2008                                       */
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
#include "ModeEntry.h"
#include "MBUtils.h"

using namespace std;

//------------------------------------------------------------------
// Procedure: print()

void ModeEntry::print()
{
  cout << "++Mode Var:   " << m_mode_var       << endl;
  cout << "  Mode Val:   " << m_mode_val       << endl;
  cout << "  Mode Else:  " << m_mode_val_else  << endl;
  cout << "  Mode Cond:  " << m_mode_condition << endl;
}

//------------------------------------------------------------------
// Procedure: setEntry
// Example:   MODE = ACTIVE {DEPLOY==TRUE} INACTIVE

bool ModeEntry::setEntry(string str)
{
  string mode_var  = stripBlankEnds(biteString(str, '='));
  string remainder = stripBlankEnds(str);

  if((mode_var == "") || (remainder == ""))
    return(false);

  // Ensure that there is only one open brace and one close brace
  // And that the open brace is to the left of the close brace
  int leftcnt  = 0;
  int leftix   = -1;
  int rightcnt = 0;
  int rightix  = -1;
  int len = remainder.length();
  for(int i=0; i<len; i++) {
    if(remainder[i] == '{') {
      leftcnt++;
      leftix = i;
    }
    else if(remainder[i] == '}') {
      rightcnt++;
      rightix = i;
    }
  }
  if((leftcnt != 1) || (rightcnt != 1))
    return(false);
  if(leftix >= rightix)
    return(false);

  string mode_val = stripBlankEnds(biteString(remainder, '{'));
  remainder = stripBlankEnds(remainder);

  string condition = stripBlankEnds(biteString(remainder, '}'));
  string else_val  = stripBlankEnds(remainder);

  return(setEntry(mode_var, mode_val, condition, else_val));
  
}


//------------------------------------------------------------------
// Procedure: setEntry

bool ModeEntry::setEntry(string mode_var,  string mode_val, 
			 string condition, string else_val)
{
  if((strContains(mode_var, ' ')) || strContains(mode_var, '\t'))
    return(false);
  
  m_mode_var       = mode_var;
  m_mode_val       = mode_val;
  m_mode_condition = condition;
  m_mode_val_else  = else_val;

  return(true);
}
