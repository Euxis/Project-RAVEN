#!/bin/bash -e
#-------------------------------------------------------
#  Part 1: Check for and handle command-line arguments
#-------------------------------------------------------
TIME_WARP=1
JUST_MAKE="no"
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
	printf "%s [SWITCHES] [time_warp]   \n" $0
	printf "  --just_make, -j    \n" 
	printf "  --help, -h         \n" 
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ] ; then
	JUST_MAKE="yes"
    else 
        echo "launch.sh Bad arg:" $ARGI " Exiting with code: 1"
        exit 1
    fi
done

#-------------------------------------------------------
#  Part 2: Create the .moos and .bhv files. 
#-------------------------------------------------------
VNAME1="raven"         # The first vehicle Community
VNAME2="shoresideproxy"         # The second vehicle Community
VNAME3="alpha"
START_POS1="0,10"       # Vehicle 1 Behavior configurations
#START_POS2="80,-125"   # Vehicle 2 Behavior configurations
START_POS2="20,30"
START_POS3="70,-125"
SHORE_LISTEN="9300"

VPORT2="9302"
VPORT3="9303"

TRAIL_RANGE1="40"
TRAIL_ANGLE1="135"

# RAVEN
nsplug meta_vehicle.moos targ_$VNAME1.moos -f WARP=$TIME_WARP \
   VNAME=$VNAME1      START_POS=$START_POS1                 \
   VPORT="9001"       SHARE_LISTEN="9301"                   \
   VTYPE="glider"     SHORE_LISTEN=$SHORE_LISTEN            \
   VNAME2=$VNAME2     VPORT2=$VPORT2                         \
   KNOWS_CONTACTS=1   DEPLOY_HGILDA="false"

# SHORESIDE PROXY/GCS
nsplug meta_vehicle.moos targ_$VNAME2.moos -f WARP=$TIME_WARP \
   VNAME=$VNAME2      START_POS=$START_POS2                 \
   VPORT="9002"       SHARE_LISTEN=$VPORT2                   \
   VTYPE="kayak"      SHORE_LISTEN=$SHORE_LISTEN
   
# ALPHA
nsplug meta_vehicle.moos targ_$VNAME3.moos -f WARP=$TIME_WARP \
   VNAME=$VNAME3      START_POS=$START_POS3                 \
   VPORT="9003"       SHARE_LISTEN=$VPORT3                   \
   VTYPE="kayak"      SHORE_LISTEN=$SHORE_LISTEN

# SHORESIDE
nsplug meta_shoreside.moos targ_shoreside.moos -f WARP=$TIME_WARP \
   SNAME="shoreside"  SHARE_LISTEN=$SHORE_LISTEN                  \
   SPORT="9000"       HENRY=$VNAME1 GILDA=$VNAME2 ALPHA=$VNAME3

# RAVEN BEHAVIOR FILE
nsplug meta_$VNAME1.bhv targ_$VNAME1.bhv -f VNAME=$VNAME1  \
    OVNAME=$VNAME2 START_POS=$START_POS2  \
    TRAIL_RANGE=$TRAIL_RANGE1              \
    TRAIL_ANGLE=$TRAIL_ANGLE1

# SHORESIDE PROXY BEHAVIOR FILE
nsplug meta_$VNAME2.bhv targ_$VNAME2.bhv -f VNAME=$VNAME2  \
    OVNAME=$VNAME1 START_POS=$START_POS1

# ALPHA BEHAVIOR FILE
nsplug $VNAME3.bhv targ_$VNAME3.bhv -f VNAME=$VNAME3  \
    OVNAME=$VNAME3 START_POS=$START_POS3  \

if [ ${JUST_MAKE} = "yes" ] ; then
    exit 0
fi

#-------------------------------------------------------
#  Part 3: Launch the processes
#-------------------------------------------------------
printf "Launching $VNAME1 MOOS Community (WARP=%s) \n" $TIME_WARP
pAntler targ_$VNAME1.moos >& /dev/null &
sleep 0.25
printf "Launching $VNAME2 MOOS Community (WARP=%s) \n" $TIME_WARP
pAntler targ_$VNAME2.moos >& /dev/null &
sleep 0.25
printf "Launching $VNAME3 MOOS Community (WARP=%s) \n" $TIME_WARP
pAntler targ_$VNAME3.moos >& /dev/null &
sleep 0.25
printf "Launching $SNAME MOOS Community (WARP=%s) \n"  $TIME_WARP
pAntler targ_shoreside.moos >& /dev/null &
printf "Done \n"

uMAC targ_shoreside.moos

printf "Killing all processes ... \n"
kill %1 %2 %3 %4
printf "Done killing processes.   \n"



