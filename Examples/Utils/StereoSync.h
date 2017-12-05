#pragma once

#include <string>
#include <vector>


class StereoSync
{
public:
	StereoSync(std::vector<std::string> directories);
	StereoSync(std::string directory);
	StereoSync();

private:
	int idx;
	std::vector<std::vector<std::string>> associations;

	void SetAssociations(std::vector<std::string> directories);
	bool LoadAssociationsFromFile(std::string directory);
	void SaveAssociationsToFile(std::string directory);

public:
	bool GetSyncedFramePaths(std::vector<std::string>& framePaths);
};

