#include "GrayCodePattern.h"
#include "Decode.h"

int main(int argh, char* argv[])
{
	GrayCodePattern::getGrayCodeImages();
	Decode::executeDecode();
}