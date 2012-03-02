#include "myTimer.h"

void MYTimer::timeInit(){
	dwTime = 0;
	memset(&nFreq,   0x00, sizeof nFreq);
	memset(&nBefore, 0x00, sizeof nBefore);
	memset(&nAfter,  0x00, sizeof nAfter);
	QueryPerformanceFrequency(&nFreq);
	QueryPerformanceCounter(&nBefore);
};
void MYTimer::timeReport(){
	QueryPerformanceCounter(&nAfter);
	dwTime =(DWORD)((nAfter.QuadPart-nBefore.QuadPart)*100000/nFreq.QuadPart);
	printf("%.2f ms\n", ((double)dwTime)/100.);
};
void MYTimer::timeReport2(){
	QueryPerformanceCounter(&nAfter);
	dwTime =(DWORD)((nAfter.QuadPart-nBefore.QuadPart)*100000/nFreq.QuadPart);
	sum+=dwTime; count++;
		
	printf("%d ms, avg: %.2f ms\n", (int)(dwTime*0.01), (sum/count)*0.01);
};
void MYTimer::timeReportFPS(){
	QueryPerformanceCounter(&nAfter);
	dwTime =(DWORD)((nAfter.QuadPart-nBefore.QuadPart)*1000/nFreq.QuadPart);
		
	printf("%.2f FPS\n", 1000./dwTime);
};