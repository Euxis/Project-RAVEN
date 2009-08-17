/*****************************************************************/
/*    NAME: Michael Benjamin and John Leonard                    */
/*    ORGN: NAVSEA Newport RI and MIT Cambridge MA               */
/*    FILE: Populator_IPF_Plot.cpp                               */
/*    DATE: Feb 24th, 2007                                       */
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
#include <stdlib.h>
#include "Populator_IPF_Plot.h"
#include "MBUtils.h"
#include "IvPFunction.h"
#include "FunctionEncoder.h"
#include "FileBuffer.h"

using namespace std;

//---------------------------------------------------------------
// Procedure: populate

bool Populator_IPF_Plot::populateFromEntries(const vector<ALogEntry>& entries)
{
  unsigned int i, vsize = entries.size();  
  for(i=0; i<vsize; i++) {
    if(entries[i].getVarName() == "BHV_IPF") {
      double time_stamp = entries[i].getTimeStamp();
      string var_value  = entries[i].getStringVal();
      m_demuxer.addMuxPacket(var_value, time_stamp);
    }
  }
  
  bool done = false;
  while(!done) {
    double time_stamp = 0;
    string demux_string = m_demuxer.getDemuxString(time_stamp);
    if(demux_string != "")
      handleEntry(time_stamp, demux_string);
    else
      done = true;
  }
  return(true);
}


//---------------------------------------------------------------
// Procedure: handleEntry

void Populator_IPF_Plot::handleEntry(double g_time, 
				     const string& g_ipf_str)
{
  IvPFunction *ipf = StringToIvPFunction(g_ipf_str);

  string context = ipf->getContextStr();


  // The Context string might come in the form of NUM:CONTEXT"
  vector<string> svector = parseString(context, ':');
  if(svector.size() == 2)
    context = svector[1];

  delete(ipf);

  int index = -1;

  int vsize = m_ipf_tags.size();
  for(int i=0; i<vsize; i++) {
    if(m_ipf_tags[i] == context)
      index = i;
  }

  if(index == -1) {
    IPF_Plot new_plot;
    //string ipf_source = m_vname + "_" + context;
    string ipf_source = context;
    new_plot.setSource(ipf_source);
    m_ipf_tags.push_back(context);
    m_ipf_plots.push_back(new_plot);
    index = vsize;
  }
  
  m_ipf_plots[index].addEntry(g_time, g_ipf_str);
}

//---------------------------------------------------------------
// Procedure: getPlotIPF

IPF_Plot Populator_IPF_Plot::getPlotIPF(unsigned int ix)
{
  if(ix >= m_ipf_plots.size()) {
    IPF_Plot null_plot;
    return(null_plot);
  }
  else
    return(m_ipf_plots[ix]);
}

//---------------------------------------------------------------
// Procedure: getTagIPF

string Populator_IPF_Plot::getTagIPF(unsigned int ix)
{
  if(ix >= m_ipf_tags.size())
    return("");
  else
    return(m_ipf_tags[ix]);
  
}

//---------------------------------------------------------------
// Procedure: print

void Populator_IPF_Plot::print()
{
  int tag_size = m_ipf_tags.size();
  int ipf_size = m_ipf_plots.size();

  cout << "Number of IPF_Plots: " << ipf_size << endl;
  cout << "Number of Tags: " << tag_size << endl;

  if(tag_size != ipf_size) {
    cout << "FIGURE OUT WHY THESE ARE DIFFERENT!!!!" << endl;
    return;
  }

  for(int i=0; i<tag_size; i++) {
    cout << "Item[" << i << "]: " << endl;
    cout << "  Tag: " << m_ipf_tags[i] << endl;
    cout << "  Plot[" << i << "]: " << endl;
    cout << "    Entries: " << m_ipf_plots[i].size() << endl;
    cout << "    LowStamp: " << m_ipf_plots[i].getMinTime() << endl;
    cout << "    HighStamp: " << m_ipf_plots[i].getMaxTime() << endl;
    cout << "    IPF_Source: " << m_ipf_plots[i].getSource() << endl;
    cout << "    IPF_VName: "  << m_ipf_plots[i].getVName() << endl;
  }
}






