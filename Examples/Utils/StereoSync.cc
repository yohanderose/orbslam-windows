#include "StereoSync.h"

#include <fstream>
#include <iostream>
#include <string>

#include <boost/filesystem.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>


using namespace std;
using namespace boost::filesystem;

vector<path> GetFilesInDirectory(string dir);
vector<path> GetSubdirectories(string dir);

StereoSync::StereoSync()
	: idx(0)
{

}
StereoSync::StereoSync(string directory)
	: idx(0)
{
	bool loaded = LoadAssociationsFromFile(directory);
	if (!loaded)
	{
		vector<path> paths = GetSubdirectories(directory);
		vector<string> directories;
		for (path p : paths)
		{
			directories.push_back(p.normalize().generic_string());
		}
		SetAssociations(directories);
	}
}
StereoSync::StereoSync(vector<string> directories)
	:idx(0)
{
	SetAssociations(directories);
}

bool StereoSync::LoadAssociationsFromFile(string directory)
{
	path assocFilePath(directory);
	assocFilePath.append("associations.txt");
	if (exists(assocFilePath))
	{
		cout << "Loading associations from file: " << assocFilePath.generic_string() << " ... ";
		std::ifstream ifs(assocFilePath.generic_string(), std::ifstream::in);
		int associationsSize, associations0Size;
		ifs >> associationsSize;
		ifs >> associations0Size;
		for (int i = 0; i < associationsSize; ++i)
		{
			associations.push_back(vector<string>(associations0Size));
		}
		for (int i = 0; i < associations[0].size(); ++i)
		{
			for (int j = 0; j < associations.size(); ++j)
			{
				ifs >> associations[j][i];
			}
		}
		ifs.close();
		cout << "Done" << endl;
		return true;
	}
	cout << "Saving associations to file: " << assocFilePath.generic_string() << " ... ";
	return false;
}
void StereoSync::SaveAssociationsToFile(string directory)
{
	path assocFilePath(directory);
	assocFilePath.append("associations.txt");

	std::ofstream ofs(assocFilePath.generic_string(), std::ofstream::out);
	ofs << associations.size() << " " << associations[0].size() << endl;
	for (int i = 0; i < associations[0].size(); ++i)
	{
		for (int j = 0; j < associations.size(); ++j)
		{
			ofs << associations[j][i] << " ";
		}
		ofs << endl;
	}
	ofs.close();
	cout << "Done" << endl;
}
void StereoSync::SetAssociations(vector<string> directories)
{
	idx = 0;

	vector<path> firstFilePaths = GetFilesInDirectory(path(directories[0]).append("data").generic_string());

	for (int i = 0; i < directories.size(); ++i)
	{
		associations.push_back(vector<string>());
	}

	for (int i = 0; i < firstFilePaths.size(); ++i)
	{
		long long tsF = stoll(firstFilePaths[i].stem().generic_string());
		string firstPath = firstFilePaths[i].generic_string();
		associations[0].push_back(firstPath);

		for (int j = 1; j < directories.size(); ++j)
		{
			vector<path> otherFilePaths = GetFilesInDirectory(path(directories[j]).append("data").generic_string());
			long long bestDiff = -1;
			int bestK = -1;
			for (int k = 0; k < otherFilePaths.size(); ++k)
			{
				long long ts = stoll(otherFilePaths[k].stem().generic_string());
				long long diff = abs(ts - tsF);
				if (diff < bestDiff || bestDiff == -1)
				{
					bestDiff = diff;
					bestK = k;
				}
			}
			string otherPath = otherFilePaths[bestK].generic_string();
			associations[j].push_back(otherPath);
		}
	}
	SaveAssociationsToFile(path(directories[0]).parent_path().generic_string());
}
bool StereoSync::GetSyncedFramePaths(vector<string>& framePaths)
{
	if (idx < associations[0].size())
	{
		for (int i = 0; i < associations.size(); ++i)
		{
			framePaths.push_back(associations[i][idx]);
		}
		idx++;
		return true;
	}
	else
		return false;
}

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
	return v;
}
vector<path> GetSubdirectories(string dir)
{
	path dirPath(dir);
	vector<path> paths;
	vector<path> subDirs;

	try
	{
		if (exists(dirPath) && is_directory(dirPath))
		{
			copy(directory_iterator(dirPath), directory_iterator(), back_inserter(paths));
			sort(paths.begin(), paths.end());
			for (vector<path>::iterator it = paths.begin(); it != paths.end(); it++)
			{
				path p = *it;
				if (is_directory(p))
				{
					subDirs.push_back(p);
				}
			}
			return subDirs;
		}
	}
	catch (const filesystem_error& ex)
	{
		cout << ex.what() << endl;
	}
	return subDirs;
}

