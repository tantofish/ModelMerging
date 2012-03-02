#ifndef TIMER_H
#define TIMER_H

#include <windows.h>
#include <cstdio>

class MYTimer{
public:
	LARGE_INTEGER nFreq, nBefore, nAfter;
	DWORD dwTime;
	DWORD sum, count;

	MYTimer():sum(0), count(0){}
	void timeInit();
	void timeReport();
	void timeReport2();
	void timeReportFPS();
};



#endif