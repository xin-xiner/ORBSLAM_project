#include <iostream>
#include <io.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <fstream>
using namespace std;

vector<std::string>& getDirectoryList(string path,vector<std::string>& fileNameList)
{
	long Handle;
	struct _finddata_t FileInfo;
	
	
	if((Handle=_findfirst((path+"\\*").c_str(),&FileInfo))==-1L)
	{
		std::cout<<"没有找到文件夹"<<endl;
	}
	else
	{
		do
		{
			if(strcmp(FileInfo.name,".") != 0  &&  strcmp(FileInfo.name,"..") != 0)
			{
				if(FileInfo.attrib &  _A_SUBDIR)
				{
					string temp = FileInfo.name;
					fileNameList.push_back(temp);
				}
			}
		}
		while(_findnext(Handle,&FileInfo)==0);
		
		_findclose(Handle);
	}
	return fileNameList;
}

vector<std::string>& getImagesList(string path,vector<std::string>& fileNameList)
{
	long Handle;
	struct _finddata_t FileInfo;
	
	
	if((Handle=_findfirst((path+"\\*.*").c_str(),&FileInfo))==-1L)
	{
		std::cout<<path+" 下没有找到图片"<<endl;
	}
	else
	{
		do
		{
			if(strcmp(FileInfo.name,".") != 0  &&  strcmp(FileInfo.name,"..") != 0)
			{
				string temp = FileInfo.name;
				fileNameList.push_back(temp);//(temp.substr(0,temp.size()-4));
			}
		}
		while(_findnext(Handle,&FileInfo)==0);
		
		_findclose(Handle);
	}
	return fileNameList;
}

void createFileList(string imagePath)
{
	string  fileListPath = "fileList\\";
	ofstream directoryList(fileListPath+"directoryList.txt");
	ofstream imageCount(fileListPath+"imageCount.txt");
	ofstream fileList;
	vector<string> classNameList;
	vector<string> fileNameList;
	getDirectoryList(imagePath,classNameList);
	for(int i = 0;i<classNameList.size();i++)
	{
		directoryList<<classNameList[i]<<endl;
		fileNameList.clear();
		getImagesList(imagePath+classNameList[i]+"\\",fileNameList);
		fileList.open(fileListPath+classNameList[i]+".txt");
		imageCount<<fileNameList.size()<<endl;
		for(int j = 0;j<fileNameList.size();j++)
		{
			fileList<<fileNameList[j]<<endl;
		}
		fileList.close();
	}
}