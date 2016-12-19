#include "GrayCodePattern.h"
#include "Sfm.h"
#include "Decode.h"

int main(int argh, char* argv[])
{
	int numOfGroup = GrayCodePattern::getGrayCodeImages();
	Sfm::executeSfm(numOfGroup);
	//Decode::executeDecode();
}