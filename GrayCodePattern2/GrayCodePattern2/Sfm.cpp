#include <io.h>
#include <direct.h>
#include <stdio.h>
#include <cmath>
#include "Sfm.h"
#include "Tools.h"
#include "Const.h"
#include "Path.h"

using namespace std;

void makeSfmDir(int const numOfGroup) {
	// make a new dir
	String sfmDir = expr_dir + sfm_group_dir;
	if (_access(sfmDir.c_str(), 6) == -1)
	{
		_mkdir(sfmDir.c_str());
	}
	int whiteImgIndex = log(proj_width) / log(2) * 4 + 1;
	ostringstream name;
	name << whiteImgIndex;
	String imagesName = images_file + name.str() + imgType;
	for (int i = 0; i < numOfGroup; i++) {
		// copy images to the new dir
		char* imagesGroupDirTemp1 = new char[images_group_dir_length];
		sprintf(imagesGroupDirTemp1, images_group_dir, i);
		String imagesDir1 = expr_dir + String(imagesGroupDirTemp1) + imagesName;
		ostringstream num;
		num << i;
		Tools::copyFile(imagesDir1, sfmDir + num.str() + imgType);
	}
}

void Sfm::executeSfm(int const numOfGroup) {
	makeSfmDir(numOfGroup);
}