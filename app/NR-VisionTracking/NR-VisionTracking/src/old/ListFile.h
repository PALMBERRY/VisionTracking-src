#ifndef LISTFILE_H
#define LISTFILE_H
#include <vector>
#include <string>

using namespace std;

static class FileLister{
public:
	vector<string> get_filelist(string foldname);
};
#endif