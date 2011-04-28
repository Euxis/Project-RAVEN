/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: NAVSEA Newport RI and MIT Cambridge MA               */
/*    FILE: BehaviorSetEntry.h                                   */
/*    DATE: Feb 5th 2010 (Broken out from BehaviorSet)           */
/*****************************************************************/

#ifndef BEHAVIORSET_ENTRY_HEADER
#define BEHAVIORSET_ENTRY_HEADER

#include <string>
#include <vector>
#include "IvPBehavior.h"

class BehaviorSetEntry
{
public:
  BehaviorSetEntry(IvPBehavior *b=0) {
    m_behavior = b;
    m_state = "";
    m_state_time_entered = 0;
    m_state_time_elapsed = -1;
  }

  ~BehaviorSetEntry() {};
  
  void   setBehavior(IvPBehavior *b)    {m_behavior=b;};
  void   setState(const std::string& s) {m_state=s;};
  void   setStateTimeEntered(double v)  {m_state_time_entered=v;};
  void   setStateTimeElapsed(double v)  {m_state_time_elapsed=v;};

  void   deleteBehavior() {
    if(m_behavior) 
      delete(m_behavior);
    m_behavior = 0;
  };

  IvPBehavior* getBehavior()         {return(m_behavior);};
  std::string  getState()            {return(m_state);};
  double       getStateTimeEntered() {return(m_state_time_entered);};
  double       getStateTimeElapsed() {return(m_state_time_elapsed);};

  std::string  getBehaviorName()  {
    if(m_behavior)
      return(m_behavior->getDescriptor());
    return("unknown");
  }

  std::string  getBehaviorType()  {
    if(m_behavior)
      return(m_behavior->getBehaviorType());
    return("unknown");
  }

protected:
  IvPBehavior*   m_behavior;
  std::string    m_state;
  double         m_state_time_entered;
  double         m_state_time_elapsed;
};

#endif 
