#include <Windows.h>
#include "ListFile.h"

vector<string> FileLister::get_filelist(string foldname)
{
	vector<string> flist;
	HANDLE file;
	WIN32_FIND_DATA fileData;
	char line[1024];
	wchar_t fn[1000];
	mbstowcs(fn,foldname.c_str(),999);
	file = FindFirstFile(fn, &fileData);
	if (file == INVALID_HANDLE_VALUE)
		return flist;
	wcstombs(line,(const wchar_t*)fileData.cFileName,259);
	flist.push_back(line);
	while(FindNextFile(file, &fileData)){
		wcstombs(line,(const wchar_t*)fileData.cFileName,259);
		flist.push_back(line);
	}
	return flist;
}



