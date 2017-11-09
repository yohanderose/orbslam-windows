#include "StereoSync.h"

#include <iostream>
#include <string>

#include <boost/filesystem.hpp>


using namespace std;
using namespace boost::filesystem;
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>


vector<path> GetFilesInDirectory(string dir)
{
	path p(dir);
	vector<path> v;

	try
	{
		if (exists(p) && is_directory(p))
		{
			copy(directory_iterator(p), directory_iterator(), back_inserter(v));
			sort(v.begin(), v.end());
			return v;
		}
	}
	catch (const filesystem_error& ex)
	{
		cout << ex.what() << endl;
	}
}

StereoSync::StereoSync()
{

}

StereoSync::StereoSync(string leftDir, string rightDir)
{
	idx = 0;

	vector<path> leftFilePaths = GetFilesInDirectory(leftDir);
	vector<path> rightFilePaths = GetFilesInDirectory(rightDir);

	for (int i = 0; i < leftFilePaths.size(); ++i)
	{
		long long tsL = stoll(leftFilePaths[i].stem().generic_string());
		long long bestDiff = -1;
		int bestJ = -1;
		for (int j = 0; j < rightFilePaths.size(); ++j)
		{
			long long tsR = stoll(rightFilePaths[j].stem().generic_string());
			long long diff = abs(tsR - tsL);
			if (diff < bestDiff || bestDiff == -1)
			{
				bestDiff = diff;
				bestJ = j;
			}
		}
		string leftPath = leftFilePaths[i].generic_string();
		string rightPath = rightFilePaths[bestJ].generic_string();
		leftFrames.push_back(leftPath);
		associatedRightFrames.push_back(rightPath);
	}
}


bool StereoSync::GetSyncedFramePaths(std::string &leftPath, std::string &rightPath)
{
	if (idx < leftFrames.size())
	{
		leftPath = leftFrames[idx];
		rightPath = associatedRightFrames[idx];
		idx++;
		return true;
	}
	else
		return false;
}

