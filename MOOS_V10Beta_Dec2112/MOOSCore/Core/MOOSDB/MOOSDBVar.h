///////////////////////////////////////////////////////////////////////////
//
//   MOOS - Mission Oriented Operating Suite 
//  
//   A suit of Applications and Libraries for Mobile Robotics Research 
//   Copyright (C) 2001-2005 Massachusetts Institute of Technology and 
//   Oxford University. 
//    
//   This software was written by Paul Newman at MIT 2001-2002 and Oxford 
//   University 2003-2005. email: pnewman@robots.ox.ac.uk. 
//      
//   This file is part of a  MOOS Core Component. 
//        
//   This program is free software; you can redistribute it and/or 
//   modify it under the terms of the GNU General Public License as 
//   published by the Free Software Foundation; either version 2 of the 
//   License, or (at your option) any later version. 
//          
//   This program is distributed in the hope that it will be useful, 
//   but WITHOUT ANY WARRANTY; without even the implied warranty of 
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
//   General Public License for more details. 
//            
//   You should have received a copy of the GNU General Public License 
//   along with this program; if not, write to the Free Software 
//   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 
//   02111-1307, USA. 
//
//////////////////////////    END_GPL    //////////////////////////////////
// MOOSDBVar.h: interface for the CMOOSDBVar class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_MOOSDBVAR_H__EAAB2A16_66EF_49E4_9584_51403C59150D__INCLUDED_)
#define AFX_MOOSDBVAR_H__EAAB2A16_66EF_49E4_9584_51403C59150D__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


#include <string>
#include <map>
#include <set>

using namespace std;

typedef set<string> STRING_SET;

#include "MOOSRegisterInfo.h"


typedef map<string,CMOOSRegisterInfo> REGISTER_INFO_MAP;


class CMOOSDBVar  
{
public:
    bool Reset();
    void RemoveSubscriber(string & sWho);



    bool AddSubscriber(const string & sClient, double dfPeriod);
    CMOOSDBVar(const string & sName);
    CMOOSDBVar();
    virtual ~CMOOSDBVar();

    char   m_cDataType;
    double m_dfTime;
    double m_dfVal;
    double m_dfWriteFreq;
    int m_nOverTicks;
    double m_dfWrittenTime;
    string m_sVal;
    string m_sWhoChangedMe;
	string m_sSrcAux;

    int     m_nWrittenTo;
    string m_sName;

    string m_sOriginatingCommunity;

    REGISTER_INFO_MAP m_Subscribers;
    STRING_SET m_Writers;

};

#endif // !defined(AFX_MOOSDBVAR_H__EAAB2A16_66EF_49E4_9584_51403C59150D__INCLUDED_)
