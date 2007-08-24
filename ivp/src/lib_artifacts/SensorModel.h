/*****************************************************************/
/*    NAME: Andrew Shafer and Michael Benjamin                   */
/*    ORGN: MIT Cambridge MA                                     */
/*    FILE: SensorModel.h                                        */
/*    DATE: 3 JUL 2007                                           */
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

#ifndef SENSORMODEL_HEADER
#define SENSORMODEL_HEADER
#include "ArtifactField.h"

#include<string>

class SensorModel {
private:
	double dSensorRadius;
	std::string sSensorType;
	
	std::vector<std::string> queryFRSensor(std::string, ArtifactField*);
	
public:
	bool setSensorModel(std::string);
	void setSensorRadius(double);
	double getSensorRadius() {return dSensorRadius;};
	std::vector<std::string> querySensor(std::string, ArtifactField*);
};

#endif
