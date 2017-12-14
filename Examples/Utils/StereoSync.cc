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
vector<long long> GetTimestampsInDirectory(string dir);
long long FindClosestTimestamp(vector<long long> arr, long long val);

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
	: idx(0)
{
	SetAssociations(directories);
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
vector<long long> GetTimestampsInDirectory(string dir)
{
	path p(dir);
	vector<path> v;
	vector<long long> ts;

	try
	{
		if (exists(p) && is_directory(p))
		{
			copy(directory_iterator(p), directory_iterator(), back_inserter(v));
			sort(v.begin(), v.end());
			for (vector<path>::iterator pIt = v.begin(); pIt != v.end(); pIt++)
			{
				ts.push_back(stoll((*pIt).stem().generic_string()));
			}
			return ts;
		}
	}
	catch (const filesystem_error& ex)
	{
		cout << ex.what() << endl;
	}
	return ts;
}
long long FindClosestTimestamp(vector<long long> arr, long long val)
{
	int L = 0, R = arr.size() - 1;
	int m = 0;
	while (true)
	{
		if (L > R)
		{
			if (abs(arr[L] - val) < abs(arr[R] - val))
			{
				return arr[L];
			}
			return arr[R];
		}
		m = floor((L + R) / 2.0);
		if (arr[m] < val)
		{
			L = m + 1;
			continue;
		}
		if (arr[m] > val)
		{
			R = m - 1;
			continue;
		}
		break;
	}
	return arr[m];
}

bool StereoSync::LoadAssociationsFromFile(string directory)
{
	idx = 0;
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
	vector<vector<path>> frames;
	vector<vector<long long>> timestamps;
	vector<path> subDirs;
	for (int i = 0; i < directories.size(); ++i)
	{
		subDirs.push_back(path(directories[i]));
		subDirs[i].append("data");
		timestamps.push_back(GetTimestampsInDirectory(subDirs[i].generic_string()));
	}

	vector<vector<long long>> associatedTimestamps(timestamps.size());
	associations.resize(timestamps.size());
	for (int i = 0; i < timestamps[0].size(); ++i)
	{
		associatedTimestamps[0].push_back(timestamps[0][i]);

		path tmp = path(subDirs[0]);
		tmp.append(to_string(timestamps[0][i]) + ".jpg");
		associations[0].push_back(tmp.normalize().generic_string());
		for (int j = 1; j < timestamps.size(); ++j)
		{
			long long closest = FindClosestTimestamp(timestamps[j], timestamps[0][i]);
			associatedTimestamps[j].push_back(closest);
			tmp = path(subDirs[j]);
			tmp.append(to_string(closest) + ".jpg");
			associations[j].push_back(tmp.normalize().generic_string());
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

