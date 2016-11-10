#pragma once
#include <iostream>
#include <vector>
#include <string>

std::vector<std::string>& getImagesList(std::string path,std::vector<std::string>& fileNameList);
std::vector<std::string>& getDirectoryList(std::string path,std::vector<std::string>& fileNameList);