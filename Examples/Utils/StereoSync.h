#pragma once

#include <string>
#include <vector>


class StereoSync
{
public:
	StereoSync(std::string leftDir, std::string rightDir);

private:
	int idx;
	std::vector<std::string> leftFrames;
	std::vector<std::string> associatedRightFrames;

public:
	bool GetSyncedFramePaths(std::string &leftPath, std::string &rightPath);
};

