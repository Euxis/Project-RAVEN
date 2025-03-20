##############################################################################
# FILE:        moos-ivp-extend/src/CMakeLists.txt
# DATE:        2010/09/07
#              2020/05/09 minor mods
# DESCRIPTION: CMakeLists.txt file for the moos-ivp-extend source directory
##############################################################################


#============================================================================
# Add the libraries in the current directory to the include path
#============================================================================
FILE(GLOB LOCAL_LIBRARY_DIRS ./lib_*)
INCLUDE_DIRECTORIES(${LOCAL_LIBRARY_DIRS})

#============================================================================
# List the subdirectories to build...
#============================================================================
ADD_SUBDIRECTORY(lib_behaviors-test)
ADD_SUBDIRECTORY(pExampleApp)
ADD_SUBDIRECTORY(pXRelayTest)
ADD_SUBDIRECTORY(pOdometry)
IF(BUILD_DCCL)
  ADD_SUBDIRECTORY(lib_acommsMsgs)
ENDIF()
ADD_SUBDIRECTORY(pMoosKiller)
ADD_SUBDIRECTORY(uSimpleRobot)
ADD_SUBDIRECTORY(lib_bhvSimpleDepth)
ADD_SUBDIRECTORY(lib_bhvNewTrail)
IF(BUILD_WOLFSSL)
  ADD_SUBDIRECTORY(pWolfEncrypt)
  ADD_SUBDIRECTORY(pWolfDecrypt)
ENDIF()

##############################################################################
#                           END of CMakeLists.txt
##############################################################################
