#ifndef _OF_UTIL
#define _OF_UTIL

#include <vector>
#include <string>

#if (_MSC_VER)       // microsoft visual studio
	#pragma warning(disable : 4996)     // disable all deprecation warnings
	#pragma warning(disable : 4068)     // unknown pragmas
	#pragma warning(disable : 4101)     // unreferenced local variable
	#pragma	warning(disable : 4312)		// type cast conversion (in qt vp)
	#pragma warning(disable : 4311)		// type cast pointer truncation (qt vp)
	#pragma warning(disable : 4018)		// signed/unsigned mismatch (since vector.size() is a size_t)
	#pragma warning(disable : 4267)		// conversion from size_t to Size warning... possible loss of data
	#pragma warning(disable : 4800)		// 'Boolean' : forcing value to bool 'true' or 'false'
	#pragma warning(disable : 4099)		// for debug, PDB 'vc80.pdb' was not found with...
#endif

using std::vector;
using std::string;

int 	ofNextPow2(int input);

void	ofResetElapsedTimeCounter();		// this happens on the first frame
float 	ofGetElapsedTimef();
int		ofGetElapsedTimeMillis();
int 	ofGetFrameNum();

int 	ofGetSeconds();
int 	ofGetMinutes();
int 	ofGetHours();

unsigned long ofGetSystemTime( );			// system time in milliseconds;

int     ofGetYear();
int     ofGetMonth();
int     ofGetDay();
int     ofGetWeekday();
 

void	ofEnableDataPath();
void	ofDisableDataPath(); 


//set the root path that ofToDataPath will use to search for files relative to the app
//the path must have a trailing slash (/) !!!!
void	ofSetDataPathRoot( string root );

string  ofToString(double value, int precision = 7);
string  ofToString(int  value);

int ofToInt(const string& intString);
float ofToFloat(const string& floatString);

string 	ofGetVersionInfo();

void	ofSaveScreen(string filename);
void	ofSaveFrame();

vector<string>	ofSplitString(const string & text, const string & delimiter);

//--------------------------------------------------
void ofSetLogLevel(int logLevel);
void ofLog(int logLevel, string message);
void ofLog(int logLevel, const char* format, ...);
void ofSetConsoleColor(int color);
void ofRestoreConsoleColor();

//--------------------------------------------------
class ofBuffer;
bool ofReadFile(const string & path, ofBuffer & file);

#endif


