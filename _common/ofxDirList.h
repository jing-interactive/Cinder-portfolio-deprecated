#ifndef OFX_DIRLIST
#define OFX_DIRLIST

/************************************************************

OpenFrameworks Library

File: 			ofxDirList.h
Description: 	List the contents of a directory
Notes:			Now takes std::string arguments and starts at the data/ folder level
                Now all C++ std::strings! No more arrays.

Last Modified: 2009.03.01
Creator: Theodore Watson

************************************************************/
#ifdef WIN32
#define TARGET_WIN32
#endif

#ifdef TARGET_WIN32
	#include <stdio.h>
	#include <iostream>
	#include <string.h>
#else
	#include <dirent.h>
#endif
#include <string>
#include <vector> 

class ofxDirList{

	public:
		ofxDirList();
		void setVerbose(bool _verbose);
		std::string getName(int pos);
		std::string getPath(int pos);
        void reset();												// resets extension list
		bool allowExt(std::string ext);									// returns true if ext is accepted
		int listDir(std::string directory);								// returns number of files found

	private:
        std::vector <std::string> allowedFileExt;
        std::vector <std::string> nameArray;
        std::vector <std::string> pathArray;

};



// this is WIN32 dirent included in here for ease of usage
// since windows doesn't have dirent by default.  this is just a wrapper....
// because we haven't written win32 version of the same code

//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------

/*
 * dirent.h - dirent API for Microsoft Visual Studio
 *
 * Copyright (C) 2006 Toni Ronkko
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * ``Software''), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED ``AS IS'', WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL TONI RONKKO BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#ifdef TARGET_WIN32

	#include <windows.h>
	#include <tchar.h>
	#include <string.h>
	#include <assert.h>


	typedef struct dirent {
	  /* name of current directory entry (a multi-byte character std::string) */
	  char d_name[MAX_PATH + 1];

	  /* file attributes */
	  WIN32_FIND_DATA data;
	} dirent;


	typedef struct DIR {
	  /* current directory entry */
	  dirent current;

	  /* is there an un-processed entry in current? */
	  int cached;

	  /* file search handle */
	  HANDLE search_handle;

	  /* search pattern (3 = zero terminator + pattern "\\*") */
	  TCHAR patt[MAX_PATH + 3];
	} DIR;


	static DIR *opendir (const char *dirname);
	static struct dirent *readdir (DIR *dirp);
	static int closedir (DIR *dirp);


	/* use the new safe std::string functions introduced in Visual Studio 2005 */
	#if defined(_MSC_VER) && _MSC_VER >= 1400
	# define STRNCPY(dest,src,size) strncpy_s((dest),(size),(src),_TRUNCATE)
	#else
	# define STRNCPY(dest,src,size) strncpy((dest),(src),(size))
	#endif


	/*
	 * Open directory stream DIRNAME for read and return a pointer to the
	 * internal working area that is used to retrieve individual directory
	 * entries.
	 */
	static DIR*
	opendir(
	    const char *dirname)
	{
	  DIR *dirp;
	  assert (dirname != NULL);
	  assert (strlen (dirname) < MAX_PATH);

	  /* construct new DIR structure */
	  dirp = (DIR*) malloc (sizeof (struct DIR));
	  if (dirp != NULL) {
	    TCHAR *p;

	    /* prepare search pattern */
	#ifdef _UNICODE

	    /* convert directory name to wide character std::string */
	    MultiByteToWideChar(
	        CP_ACP,                                /* code page */
	        0,                                     /* conversion flags */
	        dirname,                               /* mb-std::string to convert */
	        -1,                                    /* length of mb-std::string */
	        dirp->patt,                            /* wc-std::string to produce */
	        MAX_PATH);                             /* max length of wc-std::string */
	    dirp->patt[MAX_PATH] = '\0';

	    /* append search pattern to directory name */
	    p = wcschr (dirp->patt, '\0');
	    if (dirp->patt < p  &&  *(p-1) != '\\'  &&  *(p-1) != ':') {
	      *p++ = '\\';
	    }
	    *p++ = '*';
	    *p = '\0';

	#else /* !_UNICODE */

	    /* take directory name... */
	    STRNCPY (dirp->patt, dirname, sizeof(dirp->patt));
	    dirp->patt[MAX_PATH] = '\0';

	    /* ... and append search pattern to it */
	    p = strchr (dirp->patt, '\0');
	    if (dirp->patt < p  &&  *(p-1) != '\\'  &&  *(p-1) != ':') {
	      *p++ = '\\';
	    }
	    *p++ = '*';
	    *p = '\0';

	#endif /* !_UNICODE */

	    /* open stream and retrieve first file */
	    dirp->search_handle = FindFirstFile (dirp->patt, &dirp->current.data);
	    if (dirp->search_handle == INVALID_HANDLE_VALUE) {
	      /* invalid search pattern? */
	      free (dirp);
	      return NULL;
	    }

	    /* there is an un-processed directory entry in memory now */
	    dirp->cached = 1;

	  }
	  return dirp;
	}


	/*
	 * Read a directory entry, and return a pointer to a dirent structure
	 * containing the name of the entry in d_name field.  Individual directory
	 * entries returned by this very function include regular files,
	 * sub-directories, pseudo-directories "." and "..", but also volume labels,
	 * hidden files and system files may be returned.
	 */
	static struct dirent *
	readdir(
	    DIR *dirp)
	{
	  assert (dirp != NULL);

	  if (dirp->search_handle == INVALID_HANDLE_VALUE) {
	    /* directory stream was opened/rewound incorrectly or it ended normally */
	    return NULL;
	  }

	  /* get next directory entry */
	  if (dirp->cached != 0) {
	    /* a valid directory entry already in memory */
	    dirp->cached = 0;
	  } else {
	    /* read next directory entry from disk */
	    if (FindNextFile (dirp->search_handle, &dirp->current.data) == FALSE) {
	      /* the very last file has been processed or an error occured */
	      FindClose (dirp->search_handle);
	      dirp->search_handle = INVALID_HANDLE_VALUE;
	      return NULL;
	    }
	  }

	  /* copy directory entry to d_name */
	#ifdef _UNICODE

	  /* convert entry name to multibyte */
	  WideCharToMultiByte(
	      CP_ACP,                                  /* code page */
	      0,                                       /* conversion flags */
	      dirp->current.data.cFileName,            /* wc-std::string to convert */
	      -1,                                      /* length of wc-std::string */
	      dirp->current.d_name,                    /* mb-std::string to produce */
	      MAX_PATH,                                /* max length of mb-std::string */
	      NULL,                                    /* use sys default character */
	      NULL);                                   /* don't care  */
	  dirp->current.d_name[MAX_PATH] = '\0';

	#else /* !_UNICODE */

	  /* copy as a multibyte character std::string */
	  STRNCPY (dirp->current.d_name, dirp->current.data.cFileName, sizeof(dirp->current.d_name));
	  dirp->current.d_name[MAX_PATH] = '\0';

	#endif /* !_UNICODE */

	  return &dirp->current;
	}


	/*
	 * Close directory stream opened by opendir() function.  Close of the
	 * directory stream invalidates the DIR structure as well as any previously
	 * read directory entry.
	 */
	static int
	closedir(
	    DIR *dirp)
	{
	  assert (dirp != NULL);

	  /* release search handle */
	  if (dirp->search_handle != INVALID_HANDLE_VALUE) {
	    FindClose (dirp->search_handle);
	    dirp->search_handle = INVALID_HANDLE_VALUE;
	  }

	  /* release directory handle */
	  free (dirp);
	  return 0;
	}


#endif

//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------



#include <algorithm>
#include <string>


// Handy std::string functions

static std::string::size_type idx;
static std::string getExt(std::string filename){
    idx = filename.rfind('.');

    if(idx != std::string::npos){
        return filename.substr(idx+1);
    }
    else{
        return "";
    }
}

static std::string strToLower(std::string myStr){
    transform( myStr.begin(), myStr.end(), myStr.begin(), ::tolower );
    return myStr;
}

//----------------------------------------------------------
ofxDirList::ofxDirList(){
    reset();
}

//----------------------------------------------------------
void ofxDirList::reset(){
	allowedFileExt.clear();
	nameArray.clear();
	pathArray.clear();
}

//----------------------------------------------------------
void ofxDirList::setVerbose(bool _verbose){
    //ofLog(OF_LOG_WARNING, "ofxDirList setVerbose is depreciated use ofSetLogLevel instead");
}

//----------------------------------------------------------
bool ofxDirList::allowExt(std::string ext){
	allowedFileExt.push_back( strToLower(ext) );
	return true;
}

//----------------------------------------------------------
std::string ofxDirList::getName(int pos){
	if(pos >= (int)nameArray.size()) return 0;
	return nameArray[pos];
}

//----------------------------------------------------------
std::string ofxDirList::getPath(int pos){
	if(pos >= (int)pathArray.size()) return 0;
	return pathArray[pos];
}

//----------------------------------------------------------
int ofxDirList::listDir(std::string directory){
    //directory = ofToDataPath(directory);

	nameArray.clear();
	pathArray.clear();

    if(directory.length() <= 0)return 0;

    //if the trailing slash was not added - then add it
	if( directory[directory.length()-1] != '/'){
        directory = directory + "/";
	}

	DIR *dir = NULL;
	struct dirent *entry;

    //open the directory
    printf("ofxDirList - attempting to open %s", directory.c_str());
    dir = opendir(directory.c_str());

	if(dir == NULL){
		printf("ofxDirList - error opening directory");
		return 0;
	}else{
		printf("ofxDirList - success opening directory");
	}

    std::string entry_name = "";
    std::string ext = "";
	bool skip = false;

	while ((entry = readdir(dir)) != NULL){

        //turn it into a C++ std::string
        entry_name = entry->d_name;

        //lets get the length of the std::string here as we query it again
        int fileLen = entry_name.length();

		if(fileLen <= 0)continue; //if the name is not existant
		if(entry_name[0] == '.')continue; //ignore invisible files, ./ and ../

		//by default we don't skip files unless we are checking extensions
		skip = false;

		if(allowedFileExt.size() > 0){
			//we will skip this files unless it has an allowed extension
			skip = true;
			for(int i = 0; i < (int)allowedFileExt.size(); i++){

				//if the wildecard * has been entered for an ext type then don't check any extensions
				if( allowedFileExt[i] == "*"){ skip = false; break; }


				int extLen = allowedFileExt[i].length();

				//the extension has to be shorter than the filename - simple check
				if(extLen >= fileLen) continue;

                //lets get the ext as lowercase
                ext = strToLower( getExt(entry_name) );

                //if no ext - then skip this ext check
                if( ext == "" )continue;

                //if we find a match then stop checking and approve this file
                if(ext == allowedFileExt[i]){
                    skip = false;
                    break;
                }
			}
		}
		if(skip) continue;

		//finally we store the result
        pathArray.push_back(directory + entry_name);
        nameArray.push_back(entry_name);

		printf("ofxDirList - listing %s ", nameArray.back().c_str());
	}
	if(dir != NULL) closedir(dir);

	printf("ofxDirList - listed %i files in %s", nameArray.size(), directory.c_str());
	return nameArray.size();
}





#endif
