#ifndef MYGL_H
#define MYGL_H

#include <windows.h>
#include <cstdlib>
#include <cstdio>
#include <iostream>
using namespace std;

#include "modelMaker.h"

#include <GL\glut.h>


#define PI 3.141592653589793
#define toRad(x) ((x)*(PI/180.0))
#define toDeg(x) ((x)*(180.0/PI))

/* projection parameters setting */
#define _FOVY	30.0
#define _ASPECT	1.0
#define _ZNEAR	0.01
#define _ZFAR	10000.0
/* camera parameters setting */
#define _EYEX		0.0
#define _EYEY		0.0
#define _EYEZ		0.0
#define _CENTERX	0.0
#define _CENTERY	0.0
#define _CENTERZ	1.0
#define _UPX		0.0
#define _UPY		1.0
#define _UPZ		0.0
/* glClearColor() Background Color */
#define _CC_R	0.f
#define _CC_G	0.f
#define _CC_B	0.f
#define _CC_A	0.f

/* fps calculation */
#define PRINT_FRAME_INTERVAL 30


class MYGLvariables{
public:
	
	/*
	 * for user interface : viewing parameters
	 */
	float uiPitch;
	float uiYaw;
	float uiRoll;
	float uiTransX;
	float uiTransY;
	float uiTransZ;
	float uiScale;
	float rotateStep;
	float translateStep;
	float scaleStep;
	// mouse series
	int msBtn;
	int msSta;
	int msX, msY;


	GLint winW, winH;

	MYGLvariables() : 
	uiPitch(0.f), uiYaw(0.f), uiRoll(0.f), 
	uiTransX(0.f), uiTransY(0.f), uiTransZ(0.f),
	uiScale(1.f), rotateStep(5.f), translateStep(10.f), scaleStep(0.9f)
	{}
};

/* Program start here, setup opengl pipline*/
void myGLStart();
/* Every Frame's Routine Jobs will be executed in this function */
void myDisplay();
/* trigger when gl window is reshaped*/
void myReshape(GLint w, GLint h);
/* keyboard function parser */
void myKey(GLubyte key, GLint x, GLint y);
/* mouse function */
void myMouse(int button, int state, int x, int y);
/* mouse motion function */
void myMotion(int x, int y);
/* Variables Initialization*/
void myInitial();
/* idle function*/
void myIdle();
/* OpenGL Light Setting*/
void myLightInit();
/* OpenGL Material Setting*/
void myMaterialInit();
/* FPS calculation */
void myFPS(bool isStart);
/* print text onto GL window*/
void myGlPutText(float x, float y, char* text, LPVOID font = GLUT_BITMAP_HELVETICA_18, float r = 1.f, float g = 0.f, float b = 0.f, float a = 0.5f);

#endif