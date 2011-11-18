#include "ofUtils.h"
#include "ofTypes.h"
#include <algorithm>
#include <sstream>  //for ostringsream
#include <iomanip>  //for setprecision

#if defined(TARGET_OF_IPHONE) || defined(TARGET_OSX ) || defined(TARGET_LINUX)
	#include "sys/time.h"
#endif

#ifdef WIN32
	#include <windows.h>
    #include <mmsystem.h>
	#include <time.h>
	#ifdef _MSC_VER
		#include <direct.h>
	#endif
#pragma comment(lib, "winmm.lib")

#endif

#ifdef TARGET_OSX
	#include <mach-o/dyld.h>
#endif 


static bool enableDataPath = true;
static unsigned long startTime = ofGetSystemTime();   //  better at the first frame ?? (currently, there is some delay from static init, to running.

//--------------------------------------
int ofGetElapsedTimeMillis(){
	return (int)(ofGetSystemTime() - startTime);
}

//--------------------------------------
float ofGetElapsedTimef(){
	return ((float) ((int)(ofGetSystemTime() - startTime)) / 1000.0f);
}

//--------------------------------------
void ofResetElapsedTimeCounter(){
	startTime = ofGetSystemTime();
}

//=======================================
// this is from freeglut, and used internally:
/* Platform-dependent time in milliseconds, as an unsigned 32-bit integer.
 * This value wraps every 49.7 days, but integer overflows cancel
 * when subtracting an initial start time, unless the total time exceeds
 * 32-bit, where the GLUT API return value is also overflowed.
 */
unsigned long ofGetSystemTime( ) {
	#ifndef WIN32
		struct timeval now;
		gettimeofday( &now, NULL );
		return now.tv_usec/1000 + now.tv_sec*1000;
	#else
		#if defined(_WIN32_WCE)
			return GetTickCount();
		#else
			return timeGetTime();
		#endif
	#endif
}

//--------------------------------------------------
int ofGetSeconds(){
	time_t 	curr;
	tm 		local;
	time(&curr);
	local	=*(localtime(&curr));
	return local.tm_sec;
}

//--------------------------------------------------
int ofGetMinutes(){
	time_t 	curr;
	tm 		local;
	time(&curr);
	local	=*(localtime(&curr));
	return local.tm_min;
}

//--------------------------------------------------
int ofGetHours(){
	time_t 	curr;
	tm 		local;
	time(&curr);
	local	=*(localtime(&curr));
	return local.tm_hour;
}

//--------------------------------------------------
int ofGetYear(){
  time_t    curr;
  tm       local;
  time(&curr);
  local   =*(localtime(&curr));
  int year = local.tm_year + 1900;
  return year;
}

//--------------------------------------------------
int ofGetMonth(){
  time_t    curr;
  tm       local;
  time(&curr);
  local   =*(localtime(&curr));
  int month = local.tm_mon + 1;
  return month;
}

//--------------------------------------------------
int ofGetDay(){
  time_t    curr;
  tm       local;
  time(&curr);
  local   =*(localtime(&curr));
  return local.tm_mday;
}

//--------------------------------------------------
int ofGetWeekday(){
  time_t    curr;
  tm       local;
  time(&curr);
  local   =*(localtime(&curr));
  return local.tm_wday;
}

//--------------------------------------------------
void ofEnableDataPath(){
	enableDataPath = true;
}

//--------------------------------------------------
void ofDisableDataPath(){
	enableDataPath = false;
}

//--------------------------------------------------
//use ofSetDataPathRoot() to override this
#if defined TARGET_OSX
	static string dataPathRoot = "../../../data/";
#else
	static string dataPathRoot = "data/";
#endif

//--------------------------------------------------
void ofSetDataPathRoot(string newRoot){
	string newPath = "";
	
	#ifdef TARGET_OSX
		#ifndef TARGET_OF_IPHONE 
			char path[1024];
			uint32_t size = sizeof(path);
			if (_NSGetExecutablePath(path, &size) == 0){
				//printf("executable path is %s\n", path);
				string pathStr = string(path);
				//theo: check this with having '/' as a character in a folder name - OSX treats the '/' as a ':'
				//checked with spaces too!
				vector < string> pathBrokenUp = ofSplitString( pathStr, "/");
				newPath = "/";
				for(int i = 0; i < pathBrokenUp.size()-1; i++){
					newPath += pathBrokenUp[i];
					newPath += "/";
				}
				//cout << newPath << endl;   // some sanity checks here
				//system( "pwd" );
				chdir ( newPath.c_str() );
				//system("pwd");				
			}else{
				ofLog(OF_LOG_FATAL_ERROR, "buffer too small; need size %u\n", size);
			}
		#endif 
	#endif
		
	dataPathRoot = newPath+newRoot;
}

//--------------------------------------------------
string ofToDataPath(string path, bool makeAbsolute){
	if( enableDataPath ){

		//check if absolute path has been passed or if data path has already been applied
		//do we want to check for C: D: etc ?? like  substr(1, 2) == ':' ??
		if( path.length()==0 || (path.substr(0,1) != "/" &&  path.substr(1,1) != ":" &&  path.substr(0,dataPathRoot.length()) != dataPathRoot)){
			path = dataPathRoot+path;
		}

		if(makeAbsolute && (path.length()==0 || path.substr(0,1) != "/")){
			#ifndef TARGET_OF_IPHONE

			#ifndef _MSC_VER
				char currDir[1024];
				path = "/"+path;
				path = getcwd(currDir, 1024)+path;
			#else

				char currDir[1024];
				path = "\\"+path;
				path = _getcwd(currDir, 1024)+path;
				std::replace( path.begin(), path.end(), '/', '\\' ); // fix any unixy paths...


			#endif


			#else
				//do we need iphone specific code here?
			#endif
		}

	}
	return path;
}

//--------------------------------------------------
string ofToString(double value, int precision){
	std::stringstream sstr;
	sstr << std::fixed << std::setprecision(precision) << value;
	return sstr.str();
}

//--------------------------------------------------
string ofToString(int value){
	std::stringstream sstr;
	sstr << value;
	return sstr.str();
}

//--------------------------------------------------
int ofToInt(const string& intString) {
   int x;
   sscanf(intString.c_str(), "%d", &x);
   return x;
}

float ofToFloat(const string& floatString) {
   float x;
   sscanf(floatString.c_str(), "%f", &x);
   return x;
}
//--------------------------------------------------
vector<string> ofSplitString(const string& str, const string& delimiter = " "){
    vector<string> elements;
	// Skip delimiters at beginning.
    string::size_type lastPos = str.find_first_not_of(delimiter, 0);
    // Find first "non-delimiter".
    string::size_type pos     = str.find_first_of(delimiter, lastPos);

    while (string::npos != pos || string::npos != lastPos)
    {
        // Found a token, add it to the vector.
    	elements.push_back(str.substr(lastPos, pos - lastPos));
        // Skip delimiters.  Note the "not_of"
        lastPos = str.find_first_not_of(delimiter, pos);
        // Find next "non-delimiter"
        pos = str.find_first_of(delimiter, lastPos);
    }
    return elements;
}
 
 
 


