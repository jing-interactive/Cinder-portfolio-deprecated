#pragma once

#include <windows.h>
#include <commdlg.h>
#include <shlobj.h>

LRESULT CALLBACK aboutDlgProc( HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM  );

struct CComDialog
{
    CComDialog(HWND hwnd);
    ~CComDialog();

    bool PopFileOpenDlg(OUT char* pstrFileName,
                        IN char* szFilter, IN char* extension,
                        IN char* pstrTitleName = "Open");
    bool PopFileSaveDlg(OUT char* pstrFileName, 
						IN char* szFilter, IN char* extension);

	bool PopFolderDlg(OUT char* path, IN char* title = "Open", IN BFFCALLBACK cb = NULL);

	bool PopDlgFromId(int id);

    OPENFILENAME _ofn ;
    HWND _hwnd;
	char _default_filter[MAX_PATH];
};
