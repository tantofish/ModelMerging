#include "myGL.h"


/* time calculation___ */
LARGE_INTEGER GL_nFreq;				
LARGE_INTEGER GL_nBefore;
LARGE_INTEGER GL_nAfter;
DWORD	GL_dwTime;
int		GL_fpsTimeCounter	= 0;	
double	GL_fpsTimeSum		= 0;
char	GL_fpsString[30]    = {'\0'}; 
int		GL_printCount		= 30;
/* ___time calculation */

MYGLvariables glVars;
ModelMaker mMaker;

void myGLStart(){
	printf("OpenGL Progeam Start!!! \n");
	glutInitDisplayMode(GLUT_RGB | GL_DOUBLE);
	glutInitWindowSize(640, 480);
	glutInitWindowPosition(20, 200);
	int WindowNum = glutCreateWindow("My Head!!");
	myInitial();  
	glutIdleFunc(myIdle);
	glutReshapeFunc(myReshape);
	glutDisplayFunc(myDisplay);
	glutKeyboardFunc(myKey);
	glutMouseFunc(myMouse);
	glutMotionFunc(myMotion);
	glutMainLoop();
}

void myDisplay(){

	myFPS(true);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(_EYEX, _EYEY, _EYEZ, 
			  _CENTERX, _CENTERY, _CENTERZ,
			  _UPX, _UPY, _UPZ);
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	glClearColor(_CC_R, _CC_G, _CC_B, _CC_A);

	if(GL_printCount <= 0)		myGlPutText(-0.95f, -0.95f, GL_fpsString, GLUT_BITMAP_HELVETICA_18, 1.0f, 0.0f, 0.0f, 1.0f);
	else	GL_printCount--;

	
	Point3f c = mMaker.dstCenter;
	
	glTranslatef(glVars.uiTransX, glVars.uiTransY, glVars.uiTransZ);
	glTranslatef(c.x, c.y, c.z);
	glRotatef(glVars.uiPitch, 1, 0, 0);
	glRotatef(glVars.uiYaw  , 0, 1, 0);
	glRotatef(glVars.uiRoll , 0, 0, 1);
	glScalef(glVars.uiScale, glVars.uiScale, glVars.uiScale);
	glTranslatef(-c.x, -c.y, -c.z);
	
	//mMaker.automaticMerge();
	mMaker.matchProcess();

	Vec6f argument(mMaker.mtYaw, mMaker.mtPitch, mMaker.mtRoll, mMaker.mtTransX, mMaker.mtTransY, mMaker.mtTransZ );
	mMaker.transformPointCloud(argument, mMaker.srcPointCloud, mMaker.srcCenter, mMaker.transCloud, mMaker.transCenter);


	/*if(mMaker.isKey){
		mMaker.eFuncNumberCount();
		mMaker.isKey = false;
	}*/
	if(mMaker.showReadModel){
		//printf("inCloud size: %d, inColorSize: %d\n", mMaker.inPointCloud.size(), mMaker.inColorCloud.size());
		mMaker.drawPointCloud(mMaker.inPointCloud, mMaker.inColorCloud);
	}
	else{
		if(mMaker.showSrc)	mMaker.drawPointCloud(mMaker.transCloud, mMaker.srcColorCloud);
		if(mMaker.showDst)	mMaker.drawPointCloud(mMaker.dstPointCloud, mMaker.dstColorCloud);
	}
	
	glutSwapBuffers();
	myFPS(false);
}

void myInitial(){
	/* projection parameters setting */
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(_FOVY, _ASPECT, _ZNEAR, _ZFAR);
	/* modelview parameters setting */
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	/* camera parameters setting */
	gluLookAt(_EYEX, _EYEY, _EYEZ, 
			  _CENTERX, _CENTERY, _CENTERZ,
			  _UPX, _UPY, _UPZ);
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	/* openGL window back ground clor */
	glClearColor(_CC_R, _CC_G, _CC_B, _CC_A);
	
	myLightInit();
	myMaterialInit();
	/* Enable parameters  */
	//glEnable(GL_LIGHTING);
	//glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glEnable(GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	/*
	 * Project Specific Works' own initialization
	 */
	
	mMaker.readData(2, false);
	mMaker.readData(3, true);
	mMaker.fileIndex = 3;
	mMaker.buildTree(mMaker.dstPointCloud);
}

void myReshape(GLint w, GLint h){
	glVars.winW = w;
	glVars.winH = h;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(_FOVY, _ASPECT*w/h, _ZNEAR, _ZFAR);
	glViewport(0,0,w,h);
}

void myKey(GLubyte key, GLint x, GLint y){
	//printf("key = %x, x = %d, y = %d\n",key,x,y);
	mMaker.registerKey(key);
	glutPostRedisplay();
}

void myMouse(int button, int state, int x, int y){
	//printf("mouse function-->  button: %d, state: %d, x: %d, y: %d\n",button, state, x, y);
	glVars.msBtn = button;
	glVars.msSta = state;
	glVars.msX = x;
	glVars.msY = y;
	//glutPostRedisplay();
}

void myMotion(int x, int y){
	//printf("motion function-->  x: %d, y:%d\n", x, y);
	
	int xdis = x - glVars.msX;
	int ydis = y - glVars.msY;

	switch(glVars.msBtn){
	case GLUT_LEFT_BUTTON:		// Rotation
		glVars.uiYaw	+= (float) xdis * 0.5f;
		glVars.uiPitch	-= (float) ydis * 0.5f;

		//printf("yaw :%.2f, pitch: %.2f\n", glVars.uiYaw, glVars.uiPitch);
		break;
	case GLUT_MIDDLE_BUTTON:	// Scale
		if(abs(xdis) > abs(ydis))
			glVars.uiRoll	+= (float) xdis * 0.5f;
		else
			glVars.uiScale  *= (float) 1+(ydis / 480.f);
		break;
	case GLUT_RIGHT_BUTTON:		// Translation
		glVars.uiTransX -= (float) xdis;
		glVars.uiTransY -= (float) ydis;
		//printf("tx :%.2f, ty: %.2f\n", glVars.uiTransX, glVars.uiTransY);
		break;
	}

	glVars.msX = x;
	glVars.msY = y;

	//glutPostRedisplay();
}

void myIdle(){
	glutPostRedisplay();
}

void myLightInit()
{
	float LightPos[4];
	int i = 0;
	float ambient[4],diffuse[4],specular[4];
	float cAtt,lAtt,qAtt;
	float r,alpha,beta;

	for(int j=0;j<3;j++){
		ambient[j]=0.8;
		diffuse[j]=0.8;
		specular[j]=0.3;
	}
	ambient[3]=diffuse[3]=specular[3]=1;

	r=1000;
	cAtt=1;
	lAtt=0.0000000001;
	qAtt=0.0000000000001;
	alpha = 0;
	beta = 0;

	LightPos[0]=r*cos(toRad(alpha))*cos(toRad(beta));
	LightPos[1]=r*cos(toRad(alpha))*sin(toRad(beta));
	LightPos[2]=r*sin(toRad(alpha));
	LightPos[3]=1;

	glLightfv(GL_LIGHT0,GL_AMBIENT,ambient);
	glLightfv(GL_LIGHT0,GL_DIFFUSE,diffuse);
	glLightfv(GL_LIGHT0,GL_SPECULAR,specular);
	glLightfv(GL_LIGHT0,GL_POSITION,LightPos);

	glLightf(GL_LIGHT0,GL_CONSTANT_ATTENUATION,cAtt);
	glLightf(GL_LIGHT0,GL_LINEAR_ATTENUATION,lAtt);
	glLightf(GL_LIGHT0,GL_QUADRATIC_ATTENUATION,qAtt);

	glEnable(GL_LIGHT0);
	LightPos[0] *= -1;
	glLightfv(GL_LIGHT1,GL_AMBIENT,ambient);
	glLightfv(GL_LIGHT1,GL_DIFFUSE,diffuse);
	glLightfv(GL_LIGHT1,GL_SPECULAR,specular);
	glLightfv(GL_LIGHT1,GL_POSITION,LightPos);

	glLightf(GL_LIGHT1,GL_CONSTANT_ATTENUATION,cAtt);
	glLightf(GL_LIGHT1,GL_LINEAR_ATTENUATION,lAtt);
	glLightf(GL_LIGHT1,GL_QUADRATIC_ATTENUATION,qAtt);

	glEnable(GL_LIGHT1);
};

void myMaterialInit()
{
	float matA[]={0.15,0.15,0.15,1};
	float matD[]={0.6,0.6,0.6,1};
	float matS[]={1,1,1,1};
	glEnable(GL_NORMALIZE);

	glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,30);
	glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,matA);
	glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,matD);
	glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,matS);

	glColorMaterial(GL_FRONT_AND_BACK,GL_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);
};

void myGlPutText(float x, float y, char* text, LPVOID font, float r, float g, float b, float a) 
{ 
	if(!text || !strlen(text)) return; 

	/* Projectoin and Modelview Matrix storing then flush */
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	/* blending and lighting status storing then specify for text putting */
    bool blending = false; 
    if(glIsEnabled(GL_BLEND))	blending = true; 
	else	glEnable(GL_BLEND); 
	bool lighting = true; 
    if(glIsEnabled(GL_LIGHTING)) glDisable(GL_LIGHTING);
	else	lighting = false;
	
    /* Start to put text */
    glColor4f(r,g,b,a); 
    glRasterPos2f(x,y); 
    while (*text) { 
        glutBitmapCharacter(font, *text); 
        text++; 
    } 

	/* blending and lighting status resume */
    if(!blending) glDisable(GL_BLEND); 
	if(lighting) glEnable(GL_LIGHTING);
	

	/* Projectoin and Modelview Matrix resume */
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

void myFPS(bool isStart){
	if(isStart){
		/*------------- FPS Calculation before -------------*/
		memset(&GL_nFreq,   0x00, sizeof GL_nFreq);
		memset(&GL_nBefore, 0x00, sizeof GL_nBefore);
		memset(&GL_nAfter,  0x00, sizeof GL_nAfter);
		GL_dwTime = 0;
		QueryPerformanceFrequency(&GL_nFreq);
		QueryPerformanceCounter(&GL_nBefore);
	}else{
		/*------------- FPS Calculation after --------------*/
		QueryPerformanceCounter(&GL_nAfter);
		GL_dwTime =(DWORD)ceil(((GL_nAfter.QuadPart - GL_nBefore.QuadPart)*1000.0/(double)GL_nFreq.QuadPart));
		GL_fpsTimeCounter++;
		GL_fpsTimeSum += GL_dwTime;
		if(GL_fpsTimeCounter > PRINT_FRAME_INTERVAL){
			GL_fpsTimeSum /= (float)(GL_fpsTimeCounter+1);
			sprintf(GL_fpsString,"fps: %.2f \0", 1000/(float)GL_fpsTimeSum);
			GL_fpsTimeCounter = 0;	GL_fpsTimeSum = 0;
		}
	}
}