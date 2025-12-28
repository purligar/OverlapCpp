#pragma once
#include <vector>
#include <string>

#include <filesystem>

using namespace std;
namespace fs = std::filesystem;

std::string str_tolower(std::string s);

std::vector<std::string> get_file_list(const std::string& path);

std::vector<std::string> GetFileList(	const std::string& sInputhPath, 
										const std::string& skipPath, 
										std::vector<std::string> valid_extensions);