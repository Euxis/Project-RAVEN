/*****************************************************************/
/*    NAME: Michael Benjamin, Henrik Schmidt, and John Leonard   */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: ALogSorter.cpp                                       */
/*    DATE: June 22nd, 2013                                      */
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
#include "MBUtils.h"
#include "ALogSorter.h"
#include "LogUtils.h"

using namespace std;

//--------------------------------------------------------
// Procedure: addEntry()
//   Returns: true if sorting was required

bool ALogSorter::addEntry(const ALogEntry& entry)
{
  // Case 1: list is empty, just add the new entry
  if(m_entries.size() == 0) {
    m_entries.push_back(entry);
    return(false);
  }
  
  // Case 2: new entry is in correct order, just add new entry
  double time_stamp_this = entry.time();
  double time_stamp_prev = m_entries.back().time();
  if(time_stamp_this >= time_stamp_prev) {
    m_entries.push_back(entry);
    return(false);
  }
    
  // Case 3: new entry is out of order!!
#if 1
  if(time_stamp_this < m_entries.front().time())
    m_sort_warnings++;
    
  m_entries.push_back(entry);
  m_entries.sort();
#endif

#if 0
  bool inserted = false;
  list<ALogEntry>::reverse_iterator p;
  for(p=m_entries.rbegin(); ((p!=m_entries.rend()) && !inserted); p++) {
    if(time_stamp_this <= p->time()) {
      p--;
      m_entries.insert(p.base(), entry);
      inserted = true;
    }
  }
  if(!inserted) {
    m_entries.push_front(entry);
    cout << "Sort warning!!!" << endl;
  }
#endif

  return(true);
}

//--------------------------------------------------------
// Procedure: popEntry()

ALogEntry ALogSorter::popEntry()
{
  ALogEntry return_entry;
  if(m_entries.size() > 0) {
    return_entry = m_entries.front();
    m_entries.pop_front();
  }

  // If we're not checking for duplicates, we're done now
  if(!m_check_for_duplicates)
    return(return_entry);

  // Begin checking for and popping duplicates
  bool done = false;
  while(!done) {
    if(m_entries.size() == 0)
      done = true;
    else {
      if(m_entries.front() == return_entry)
	m_entries.pop_front();
      else
	done = true;
    }
  }
  // Done checking for duplicates

  return(return_entry);
}

