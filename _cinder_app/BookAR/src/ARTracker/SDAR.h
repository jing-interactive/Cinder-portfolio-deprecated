
// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the SDAR_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// SDAR_API functions as being imported from a DLL, wheras this DLL sees symbols
// defined with this macro as being exported.
#ifdef SDAR_EXPORTS
#define SDAR_API __declspec(dllexport)
#else
#define SDAR_API __declspec(dllimport)
#endif

// This class is exported from the SDAR.dll
class SDAR_API CSDAR {
public:
	CSDAR(void);
	// TODO: add your methods here.
};

extern SDAR_API int nSDAR;

SDAR_API int fnSDAR(void);


// control the system start and stop
// para@w and para @h :
//	   width and height of images or frames to be detected
extern "C" SDAR_API int SDARStart(int frameWidth=320, int frameHeight=240);
extern "C" SDAR_API int SDAREnd();
//extern "C" SDAR_API int SDARLoadCamera(char* CamFile, int isBinFormat);


// perform frame detection
// pBGRImg: the input frame BGR image data
// widthStep:   width step of frame image
extern "C"  SDAR_API int SDARTrack(unsigned char* pBGRImg,  int widthStep);

// get the number of active trackables
extern "C"  SDAR_API unsigned int getNumOfActiveTrackables();

// get the total number of loaded active trackables
extern "C"  SDAR_API unsigned int getNumOfTrackables();

// get the index of the atIdx-th active trackable 
extern "C"  SDAR_API unsigned int getActiveTrackableID(unsigned int atIdx);

// get the name of the atIdx-th active trackable (not available yet)
extern "C"  SDAR_API char* getActiveTrackableName(unsigned int atIdx);

// return 4x4 projection matrix and 4x4 model view matrix in OpenGL style.
// tIdx: tIdx-th target
extern "C"  SDAR_API double* getProjectionMatrix(double dNear, double dFar);
extern "C"  SDAR_API double* getModelViewMatrix(unsigned int tIdx=0);
extern "C"  SDAR_API int     getProjectionMatrixGL(double dNear, double dFar, double projectionMatrix[16]);
extern "C"  SDAR_API int     getModelViewMatrixGL(unsigned int tIdx, double modelViewMatrix[16]);

// the detected i-th vertice on tIdx-th target
extern "C"  SDAR_API float getVertexX(unsigned int tIdx, unsigned int i);
extern "C"  SDAR_API float getVertexY(unsigned int tIdx, unsigned int i);

// for model create
// pBGRImg: the input BGR model image
// width:   width of model image
// height:  height of model image
// wstep:   width step of model image
// ptROI:   indicts the four corner coordinates with the form (x1,y1,x2,y2,x3,y3,x4,y4) in clockwise
// fname:   model file name (including direction) to be saved as
extern "C"  SDAR_API int  SDARLearn(unsigned char* pBGRImg, int width, int height, int wstep, int* ptROI=0, char* fname=0);

extern "C"  SDAR_API void enableMemoryFirstMode();

// for model save
// fname:   model file name (including direction) to saveas
extern "C"  SDAR_API int SDARSaveAs(char* fname);

// for model loading
// fname:   model file name (including direction) to be loaded
extern "C"  SDAR_API int SDARLoad(char* fname);

// for getting the keypoints of mIdx-th model
extern "C"  SDAR_API unsigned int modelKeypointCount(unsigned int mIdx);
extern "C"  SDAR_API int modelKeypoint(unsigned int i, float p[2]);

