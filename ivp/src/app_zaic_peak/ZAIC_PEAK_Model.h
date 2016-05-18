/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: ZAIC_PEAK_Model.h                                    */
/*    DATE: May 16th, 2016                                       */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s).                                      */
/*****************************************************************/

#ifndef ZAIC_PEAK_MODEL_HEADER
#define ZAIC_PEAK_MODEL_HEADER

#include <string>
#include "IvPDomain.h"
#include "ZAIC_Model.h"
#include "ZAIC_PEAK.h"

class ZAIC_PEAK_Model : public ZAIC_Model
{
 public:
  ZAIC_PEAK_Model();
  ~ZAIC_PEAK_Model() {};

 public: // Virtual functions overloaded
  IvPFunction *getIvPFunction();
  void  setDomain(unsigned int);
  void  moveX(double);
  void  currMode(int);
  
 public: // Editing functions
  void  drawMode(int);
  void  setValueWrap(bool);
  void  toggleValueWrap()    {setValueWrap(!m_value_wrap);}

  int   getCurrMode() const  {return(m_curr_mode);}

  // Getters
  double getSummit1();
  double getBaseWidth1();
  double getPeakWidth1();
  double getSummitDelta1();

  double getSummit2();
  double getBaseWidth2();
  double getPeakWidth2();
  double getSummitDelta2();

  double getMinUtil();
  double getMaxUtil();

 protected:
  ZAIC_PEAK* m_zaic_peak1;
  ZAIC_PEAK* m_zaic_peak2;

  int        m_curr_mode;
  int        m_draw_mode;
  bool       m_value_wrap;
};
#endif 
