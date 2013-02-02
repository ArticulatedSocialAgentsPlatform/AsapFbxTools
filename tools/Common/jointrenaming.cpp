#include "jointrenaming.h"
#include <fstream>

using namespace std;

void JointRenaming::setupRenaming(const char* filename)	
{
	fstream f;
	f.open(filename);
	char modelName[1024];
	char hanimName[1024];
	while(!f.eof())
	{
		f>>modelName>>hanimName;
		mRenameMap[string(hanimName)]=string(modelName);
		mInverseRenameMap[string(modelName)]=string(hanimName);
	}
	f.close();
}

const char* JointRenaming::getJointName(string hanimName)
{
	map<string,string>::iterator it = mRenameMap.find(hanimName);	
	if(it != mRenameMap.end())
	{
		return it->second.c_str();
	}
	return NULL;
}

const char* JointRenaming::getHanimName(string jointName)
{
	map<string,string>::iterator it = mInverseRenameMap.find(jointName);	
	if(it != mInverseRenameMap.end())
	{
		return it->second.c_str();
	}
	return NULL;
}