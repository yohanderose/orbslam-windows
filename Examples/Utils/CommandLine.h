#pragma once


#include <string>
#include <vector>
#include <exception>
#include <iostream>
#include <sstream>


class CommandLine
{

public:
	CommandLine(int argc, std::string *argv)
		: mArgc(argc)
	{
		for (int i = 0; i < argc; ++i)
		{
			mArgv.push_back(argv[i]);
		}
	}
	CommandLine(int argc, char **argv)
		: mArgc(argc)
	{
		for (int i = 0; i < argc; ++i)
		{
			mArgv.push_back(std::string(argv[i]));
		}
	}
	CommandLine(int argc, const char **argv)
		: mArgc(argc)
	{
		for (int i = 0; i < argc; ++i)
		{
			mArgv.push_back(std::string(argv[i]));
		}
	}

	bool ContainsKey(std::string key)
	{
		for (std::vector<std::string>::iterator argvIt = mArgv.begin(); argvIt != mArgv.end(); argvIt++)
		{
			std::string arg = *argvIt;
			if ((*argvIt).compare("-" + key) == 0)
			{
				return true;
			}
		}
		return false;
	}

	bool GetIntValue(std::string key, int &result)
	{
		try
		{
			bool seenKey = false;
			bool setVal = false;
			for (std::vector<std::string>::iterator argvIt = mArgv.begin(); argvIt != mArgv.end(); argvIt++)
			{
				std::string arg = *argvIt;
				if ((*argvIt).compare("-" + key) == 0)
				{
					seenKey = true;
				}
				else if (seenKey)
				{
					if (isNumber(arg))
					{
						result = std::stoi(arg);
						setVal = true;
					}
					else
					{
						setVal = false;
					}
					seenKey = false;
				}
			}
			return setVal;
		}
		catch (std::exception& e)
		{
			std::cerr << "[Error] -- Failed to parse program arguments.1" << std::endl;
			return false;
		}
	}
	bool GetDoubleValue(std::string key, double &result)
	{
		try
		{
			bool seenKey = false;
			bool setVal = false;
			for (std::vector<std::string>::iterator argvIt = mArgv.begin(); argvIt != mArgv.end(); argvIt++)
			{
				std::string arg = *argvIt;
				if ((*argvIt).compare("-" + key) == 0)
				{
					seenKey = true;
				}
				else if (seenKey)
				{
					if (isNumber(arg))
					{
						result = std::stod(arg);
						setVal = true;
					}
					else
					{
						setVal = false;
					}
					seenKey = false;
				}
			}
			return setVal;
		}
		catch (std::exception& e)
		{
			std::cerr << "[Error] -- Failed to parse program arguments." << std::endl;
			return false;
		}
	}
	bool GetStringValue(std::string key, std::string &result)
	{
		try
		{
			bool seenKey = false;
			bool setVal = false;
			for (std::vector<std::string>::iterator argvIt = mArgv.begin(); argvIt != mArgv.end(); argvIt++)
			{
				std::string arg = *argvIt;
				if ((*argvIt).compare("-" + key) == 0)
				{
					seenKey = true;
				}
				else if (seenKey)
				{
					result = arg;
					setVal = true;
					seenKey = false;
				}
			}
			return setVal;
		}
		catch (std::exception& e)
		{
			std::cerr << "[Error] -- Failed to parse program arguments." << std::endl;
			return false;
		}
	}

	bool GetMultiIntValue(std::string key, std::vector<int> &result)
	{
		try
		{
			bool seenKey = false;
			bool setVal = false;
			for (std::vector<std::string>::iterator argvIt = mArgv.begin(); argvIt != mArgv.end(); argvIt++)
			{
				std::string arg = *argvIt;
				if ((*argvIt).compare("-" + key) == 0)
				{
					seenKey = true;
				}
				else if (seenKey)
				{
					std::vector<std::string> vals = split(arg, ',');
					for (std::vector<std::string>::iterator valsIt = vals.begin(); valsIt != vals.end(); valsIt++)
					{
						std::string val = *valsIt;
						if (isNumber(val))
						{
							result.push_back(std::stoi(val));
							setVal = true;
						}
						else
						{
							setVal = false;
						}
					}
					seenKey = false;
				}
			}
			return setVal;
		}
		catch (std::exception& e)
		{
			std::cerr << "[Error] -- Failed to parse program arguments." << std::endl;
			return false;
		}
	}
	bool GetMultiDoubleValue(std::string key, std::vector<double> &result)
	{
		try
		{
			bool seenKey = false;
			bool setVal = false;
			for (std::vector<std::string>::iterator argvIt = mArgv.begin(); argvIt != mArgv.end(); argvIt++)
			{
				std::string arg = *argvIt;
				if ((*argvIt).compare("-" + key) == 0)
				{
					seenKey = true;
				}
				else if (seenKey)
				{
					std::vector<std::string> vals = split(arg, ',');
					for (std::vector<std::string>::iterator valsIt = vals.begin(); valsIt != vals.end(); valsIt++)
					{
						std::string val = *valsIt;
						if (isNumber(val))
						{
							result.push_back(std::stod(val));
							setVal = true;
						}
						else
						{
							setVal = false;
						}
					}
					seenKey = false;
				}
			}
			return setVal;
		}
		catch (std::exception& e)
		{
			std::cerr << "[Error] -- Failed to parse program arguments." << std::endl;
			return false;
		}
	}
	bool GetMultiStringValue(std::string key, std::vector<std::string> &result)
	{
		try
		{
			bool seenKey = false;
			bool setVal = false;
			for (std::vector<std::string>::iterator argvIt = mArgv.begin(); argvIt != mArgv.end(); argvIt++)
			{
				std::string arg = *argvIt;
				if ((*argvIt).compare("-" + key) == 0)
				{
					seenKey = true;
				}
				else if (seenKey)
				{
					std::vector<std::string> vals = split(arg, ',');
					for (std::vector<std::string>::iterator valsIt = vals.begin(); valsIt != vals.end(); valsIt++)
					{
						std::string val = *valsIt;
						result.push_back(val);
					}
					setVal = true;
					seenKey = false;
				}
			}
			return setVal;
		}
		catch (std::exception& e)
		{
			std::cerr << "[Error] -- Failed to parse program arguments." << std::endl;
			return false;
		}
	}

private:
	int mArgc;
	std::vector<std::string> mArgv;
	std::vector<std::string> split(std::string str, char delimiter)
	{
		std::stringstream ss(str);
		std::vector<std::string> result;

		while (ss.good())
		{
			std::string substr;
			getline(ss, substr, delimiter);
			result.push_back(substr);
		}

		return result;
	}
	bool isNumber(const std::string& s)
	{
		int nb_point = 0;
		for (int i = 0; i < s.length(); i++)
		{
			if (s[i] == '.')
			{
				nb_point++;
			}
			else if (!isdigit(s[i]))
			{
				return false;
			}
		}
		if (nb_point <= 1)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

};

