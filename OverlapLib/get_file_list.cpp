//#include "stdafx.h"

#include <iostream>
#include <filesystem>

#include "get_file_list.hpp"

using namespace std;
namespace fs = std::filesystem;

std::vector<std::string> get_file_list(const std::string& path)
{
	std::vector<std::string> m_file_list;
	if (!path.empty())
	{
		fs::path apk_path(path);
		fs::directory_iterator end;

		for (fs::directory_iterator i(apk_path); i != end; ++i)
		{
			const fs::path cp = (*i);
			m_file_list.push_back(cp.string());
		}
	}
	return m_file_list;
}