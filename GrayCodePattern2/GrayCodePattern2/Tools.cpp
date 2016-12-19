#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>;
#include "Tools.h"

using namespace cv;
using namespace std;

int Tools::copyFile(const String fromFile, const String toFile) {
	fstream in(fromFile, ios::in | ios::binary);
	if (!in)
	{
		cerr << "open " << fromFile << " failed" << endl;
		in.close();
		return 0;
	}

	fstream out(toFile, ios::out | ios::binary);
	if (!out)
	{
		cerr << "open " << toFile << " failed" << endl;
		out.close();
		return 0;
	}

	char temp;
	while (in.get(temp))
	{
		out << temp;
	}
	out << endl;
	cout << "success!" << endl;
	in.close();
	out.close();
}