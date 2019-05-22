/**
********************************************************************************
*
*	@file       linux_timer.h
*
*	@brief      This file contains a simple and convenient CPU timer for Linux.
*
*	@author     Quentin K. Gautier
*
*	@section	E-mail
*                   qkgautier@gmail.com
********************************************************************************
*/


#ifndef LINUX_TIMER_H
#define LINUX_TIMER_H


//******************************************************************************
//	Include
//******************************************************************************
#include <ctime>

using namespace std;



//******************************************************************************
//	Namespace
//******************************************************************************
//namespace Utils
//{


//==============================================================================
/**
	*	@class	CpuTimer
	*	@brief	CpuTimer is a simple CPU timer for Linux.
	* This is basically just a wrapper for <ctime>.
	* To measure time spent by the actual process, use CpuTimerProcess.
	* To measure the real time elapsed, use CpuTimerMonotonic.
	* @note This class requires linking to the rt library (note available on all systems)
	*/
//==============================================================================

template<clockid_t ClockId> class CpuTimer
{

//******************************************************************************
public:
	//**************************************************************************

	//---------------------------------------------------------------
	/// Record time at construction.
	//---------------------------------------------------------------
	CpuTimer()
	{
		clock_gettime(ClockId, &m_timeStart);
	}


	//---------------------------------------------------------------
	/// Erase previous time record and replace by current time.
	//---------------------------------------------------------------
	void start()
	{
		clock_gettime(ClockId, &m_timeStart);
	}


	//---------------------------------------------------------------
	/// Get the elapsed time in ms since the last record.
	//---------------------------------------------------------------
	double getElapsedTime()
	{
		clock_gettime(ClockId, &m_timeStop);


		if((m_timeStop.tv_nsec - m_timeStart.tv_nsec) < 0)
		{
			return (m_timeStop.tv_sec - m_timeStart.tv_sec - 1) * 1000.0                   // seconds
					+ (1000000000 + m_timeStop.tv_nsec - m_timeStart.tv_nsec) / 1000000.0; // milliseconds
		}
		else
		{
			return (m_timeStop.tv_sec - m_timeStart.tv_sec) * 1000.0          // seconds
					+ (m_timeStop.tv_nsec - m_timeStart.tv_nsec) / 1000000.0; // milliseconds
		}
	}


//******************************************************************************
protected:
	//**************************************************************************



	timespec m_timeStart;
	timespec m_timeStop;
};


typedef CpuTimer<CLOCK_PROCESS_CPUTIME_ID> CpuTimerProcess;
typedef CpuTimer<CLOCK_MONOTONIC> CpuTimerMonotonic;


//} // namespace CartUtils

#endif // LINUX_TIMER_H
