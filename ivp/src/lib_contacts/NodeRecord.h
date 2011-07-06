/*****************************************************************/
/*    NAME: Michael Benjamin, Henrik Schmidt, and John Leonard   */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: NodeRecord.h                                         */
/*    DATE: Feb 27th 2010                                        */
/*          Jun 26th 2011                                        */
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

#ifndef NODE_RECORD_HEADER
#define NODE_RECORD_HEADER

#include <string>
#include <map>

class NodeRecord
{
 public:
  NodeRecord(std::string vname="", std::string vtype="");
  ~NodeRecord() {};

  void setX(double v)         {m_x=v;   m_x_set=true;};
  void setY(double v)         {m_y=v;   m_y_set=true;};
  void setLat(double v)       {m_lat=v; m_lat_set=true;};
  void setLon(double v)       {m_lon=v; m_lon_set=true;};
  void setSpeed(double v)     {m_speed=v;      m_speed_set=true;};
  void setSpeedOG(double v)   {m_speed_og=v;   m_speed_og_set=true;};
  void setHeading(double v)   {m_heading=v;    m_heading_set=true;};
  void setHeadingOG(double v) {m_heading_og=v; m_heading_og_set=true;};
  void setYaw(double v)       {m_yaw=v;        m_yaw_set=true;};
  void setDepth(double v)     {m_depth=v;      m_depth_set=true;};
  void setLength(double v)    {m_length=v;     m_length_set=true;};
  void setTimeStamp(double v) {m_timestamp=v;  m_timestamp_set=true;};

  void setName(std::string s)    {m_name=s;};
  void setType(std::string s)    {m_type=s;};
  void setMode(std::string s)    {m_mode=s;};
  void setAllStop(std::string s) {m_allstop=s;};

  void setProperty(std::string, double);

  double getX() const         {return(m_x);};
  double getY() const         {return(m_y);};
  double getLat() const       {return(m_lat);};
  double getLon() const       {return(m_lon);};
  double getSpeed() const     {return(m_speed);};
  double getSpeedOG() const   {return(m_speed_og);};
  double getHeading() const   {return(m_heading);};
  double getHeadingOG() const {return(m_heading_og);};
  double getYaw() const       {return(m_heading);};
  double getDepth() const     {return(m_depth);};
  double getLength() const    {return(m_length);};
  double getTimeStamp() const {return(m_timestamp);};

  double getElapsedTime(double) const;
  double getProperty(std::string) const;
  bool   hasProperty(std::string) const;
  bool   valid(std::string check="") const;

  std::string getName(std::string s="") const;
  std::string getType(std::string s="") const;
  std::string getMode(std::string s="") const;
  std::string getAllStop(std::string s="") const;

  std::string getSpec() const;

  std::string getStringValue(std::string) const;

 protected: 
  double m_x;
  double m_y;
  double m_speed;
  double m_speed_og;    // Speed over ground
  double m_heading;
  double m_heading_og;  // Heading over ground
  double m_yaw;
  double m_depth;
  double m_lat;
  double m_lon;
  double m_length;
  double m_timestamp;
  std::string  m_name;
  std::string  m_type;
  std::string  m_mode;
  std::string  m_allstop;
  
  bool m_x_set;
  bool m_y_set;
  bool m_lat_set;
  bool m_lon_set;
  bool m_speed_set;
  bool m_speed_og_set;
  bool m_heading_set;
  bool m_heading_og_set;
  bool m_yaw_set;
  bool m_depth_set;
  bool m_length_set;
  bool m_timestamp_set;

  // General provisions for using this record to store additional
  // information about this contact
  std::map<std::string, double> m_properties;
};

#endif 

