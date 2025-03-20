#--------------------------------------------------------
# The CMakeLists.txt for:              lib_bhvNewTrail
# Author(s):                            Declan Andrews
#--------------------------------------------------------

# Set System Specific Libraries
if (${WIN32})
  # Windows Libraries
  SET(SYSTEM_LIBS
      )
else (${WIN32})
  # Linux and Apple Libraries
  SET(SYSTEM_LIBS
      m )
endif (${WIN32})


MACRO(ADD_BHV BHV_NAME)
  ADD_LIBRARY(${BHV_NAME} SHARED ${BHV_NAME}.cpp)
  TARGET_LINK_LIBRARIES(${BHV_NAME} 
    helmivp
    behaviors 
    ivpbuild 
    logic 
    ivpcore 
    bhvutil 
    mbutil 
    geometry 
    ${SYSTEM_LIBS} )
ENDMACRO(ADD_BHV)


#--------------------------------------------------------
#                                         BHV_NewTrail
#--------------------------------------------------------
ADD_LIBRARY(BHV_NewTrail SHARED 
  BHV_NewTrail.cpp )
TARGET_LINK_LIBRARIES(BHV_NewTrail
   helmivp
   behaviors 
   ivpbuild 
   logic 
   ivpcore 
   bhvutil 
   mbutil 
   geometry
   contacts
   ${SYSTEM_LIBS} )
