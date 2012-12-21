
#ifndef MOOSAsyncCommClientH
#define MOOSAsyncCommClientH

#include <map>
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include "MOOS/libMOOS/Utils/MOOSThread.h"
#include "MOOS/libMOOS/Utils/SafeList.h"

namespace MOOS
{
class ActiveMailQueue;

	class MOOSAsyncCommClient : public CMOOSCommClient
	{
		//you are unlikely to wa
	public:
		typedef CMOOSCommClient BASE;

		/**
		 * default constructor
		 */
		MOOSAsyncCommClient();
		virtual ~MOOSAsyncCommClient();

		/**
		 * Close the client. This is blocking call.
		 * @param Nice (not used -legacy)
		 * @return true on succes
		 */
		virtual bool Close(bool Nice = true );

		/**
		 * Send a single MOOSMsg
		 * @param Msg
		 * @param bKeepMsgSourceName
		 * @return
		 */
	    virtual bool Post(CMOOSMsg & Msg,bool bKeepMsgSourceName=false);


	    /**
	     * Is client running
	     * @return true if it is
	     */
	    virtual bool IsRunning();

	    /**
	     * Flush all unsent data. Does nothing as data is always sent ASAP
	     * @return true on success
	     */
	    virtual bool Flush();

	    /**
	     * Is this an Asynchronous Client?
	     * @return
	     */
	    virtual bool IsAsynchronous();


		//some thread workers which need to be public so threads can run them
	    //you won't be calling these yourself.
	    bool ReadingLoop();
	    bool WritingLoop();


	protected:

	    /** Called internally when connection needs to be closed*/
	    virtual bool OnCloseConnection();

	    /**
	     * start all the workr threads
	     * @return true on success
	     */
		virtual bool StartThreads();

		/**
		 * Print a banner describing client
		 */
	    virtual void DoBanner();

	    /**
	     * make sure the writing is not happening too fast.
	     * This is an internal function and is used toprevent
	     * rogue users hurting others by swamping the network
	     * @return true on success
	     */
	    bool MonitorAndLimitWriteSpeed();

	    virtual std::string HandShakeKey();

	    /**
	     * perform the management of the incoming data (called internally)
	     * @return
	     */
	    bool DoReading();

	    /**
	     * perform the management of the outgoing data
	     * @return
	     */
	    bool DoWriting();


	    //data members below here
	    CMOOSThread WritingThread_; //handles writing
	    CMOOSThread ReadingThread_; //handles reading

	    double m_dfLastTimingMessage; //time last timing messae was sent
	    double m_dfLastSendTime; 	//time last message was sent
	    unsigned int m_nOverSpeedCount;

	    MOOS::SafeList<CMOOSMsg> OutGoingQueue_; //queue of outgoing mail


	};
};

#endif