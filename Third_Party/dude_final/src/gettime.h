//------------------------------------------------------------------------------
//  Copyright 2010-2011 by Jyh-Ming Lien and George Mason University
//  See the file "LICENSE" for more information
//------------------------------------------------------------------------------

#ifndef _GETTIME_H_
#define _GETTIME_H_

//-----------------------------------------------------------------------------

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#include <iostream>

//------------------------------------------------------------------------
inline double getTime() //in millisecond
{
#ifdef _WIN32
    return timeGetTime();
#else 
    //assuming unix-type systems
    //timezone tz;
    timeval  tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec*1000000+tv.tv_usec)*1.0/1000;
#endif
}

#endif //_GETTIME_H_

