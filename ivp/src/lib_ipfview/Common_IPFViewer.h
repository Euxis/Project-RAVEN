/*****************************************************************/
/*    NAME: Michael Benjamin, Henrik Schmidt, and John Leonard   */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: Common_IPFViewer.h                                   */
/*    DATE: Feb 25th, 2007                                       */
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

#ifndef COMMON_IPF_VIEWER_HEADER
#define COMMON_IPF_VIEWER_HEADER

#include <string>
#include "FL/Fl.H"
#include "FL/Fl_Gl_Window.H"
#include "FL/gl.h"
#include "FL/fl_draw.H"
#include "QuadSet.h"
#include "IvPFunction.h"
#include "ColorPack.h"
#include "FColorMap.h"

class Common_IPFViewer : public Fl_Gl_Window
{
 public:
  Common_IPFViewer(int x,int y,int w,int h,const char *l=0);
  virtual ~Common_IPFViewer() {}

  void   resize(int x, int y, int w, int h);
  void   clear();
  void   draw();
  //int    handle(int);

public:
  bool   setQuadSetFromIPF(IvPFunction*);

  bool   setParam(std::string, std::string);
  bool   setParam(std::string, double);
  void   printParams();

  void   setClearColor(std::string s)  {m_clear_color.setColor(s);}
  void   setFrameColor(std::string s)  {m_frame_color.setColor(s);}
  void   setColorMap(std::string s)    {m_color_map.setType(s);}
  
protected:
  void   resetRadVisuals();
  bool   drawQuadSet(const QuadSet&);
  void   drawQuadSet1D();
  bool   drawQuadSet2D(const QuadSet&);

  void   drawFrame(bool full=true);
  void   drawPolarFrame(bool full=true);
  void   drawCenteredShip();
  void   drawOwnPoint();
  void   drawMaxPoint(double, double);

  void   draw1DAxes(const IvPDomain&);
  void   draw1DLabels(const IvPDomain&);
  void   draw1DKeys(std::vector<std::string>, std::vector<ColorPack>);
  void   draw1DLine(double val=0, std::string label="");
  void   draw1DLineX(double, std::string, int, ColorPack);
  void   drawText(int x, int y, std::string s);
  void   drawText2(double px, double py, const std::string& text,
		   const ColorPack& font_c, double font_size);

 protected:
  void   drawQuad(Quad3D quad);


protected:
  ColorPack  m_clear_color;
  ColorPack  m_frame_color;
  ColorPack  m_ship_color;
  FColorMap  m_color_map;
  
  double     m_xRot;
  double     m_zRot;
  double     m_zoom;

  double     m_base;
  double     m_scale;
  double     m_rad_ratio;
  double     m_rad_extent;
  double     m_intensity;
  int        m_polar;
  
  bool       m_draw_ipf;
  bool       m_draw_pin;
  bool       m_draw_base;
  bool       m_draw_pclines;

  bool       m_draw_frame;
  double     m_frame_height;

  bool       m_draw_ship;
  double     m_ship_scale;
  
  QuadSet    m_quadset;
  
  // Information for drawing 1D functions
  int        m_xoffset;
  int        m_yoffset;
  int        m_grid_width;
  int        m_grid_height;
};

#endif 




