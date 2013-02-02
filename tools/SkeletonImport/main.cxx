/**************************************************************************************

 Copyright © 2001 - 2005 Systèmes Alias Québec Inc. and/or its licensors.  
 All rights reserved.

 The coded instructions, statements, computer programs, and/or related material 
 (collectively the "Data") in these files is provided by Alias (as defined in the Alias 
 Software License Agreement that accompanies the Alias software (the "Agreement")) 
 for the exclusive use of the Customer (as defined in the Agreement). Such Customer has 
 the right to use, modify, and incorporate the Data into other products and to 
 distribute such products for use by end-users.

 THE DATA IS PROVIDED “AS IS” AND WITHOUT WARRANTY. ALL WARRANTIES ARE EXPRESSLY
 EXCLUDED AND DISCLAIMED. ALIAS MAKES NO WARRANTY OF ANY KIND WITH RESPECT TO THE DATA,
 EXPRESS, IMPLIED OR ARISING BY CUSTOM OR TRADE USAGE, AND DISCLAIMS ANY IMPLIED
 WARRANTIES OF TITLE, NON-INFRINGEMENT, MERCHANTABILITY OR FITNESS FOR A PARTICULAR 
 PURPOSE. IN NO EVENT SHALL ALIAS, ITS AFFILIATES, PARENT COMPANIES, LICENSORS OR 
 SUPPLIERS (“ALIAS GROUP”) BE LIABLE FOR ANY DAMAGES OR EXPENSES OF ANY KIND INCLUDING
 WITHOUT LIMITATION PUNITIVE OR MULTIPLE DAMAGES OR OTHER SPECIAL, DIRECT, INDIRECT, 
 EXEMPLARY, INCIDENTAL, OR CONSEQUENTIAL DAMAGES OF ANY KIND, WHETHER OR NOT ALIAS HAS
 BEEN ADVISED OF THE POSSIBILITY OF SUCH LOSS, HOWSOEVER CAUSED AND ON ANY THEORY OF
 LIABILITY. 

**************************************************************************************/

/////////////////////////////////////////////////////////////////////////
//
// The scene created in this example is a cylinder linked to a skeleton
// made of 2 segments. Two takes of animation show the influence of the
// skeleton segments over the cylinder.
//
// The example illustrates how to:
//        1) create a patch
//		  2) create a skeleton segment
//		  3) create a link
//		  4) store the bind pose
//        5) store one arbitrary rest pose
//        6) create multiple takes of animation
//        7) create meta-data and add a thumbnail
//        8) export a scene in a .FBX file (ASCII mode)
//
/////////////////////////////////////////////////////////////////////////

#include <fbxsdk.h>
#include <windows.h>
#include "../Common/Common.h"
#include "Thumbnail.h"
#include <stdio.h>
#include "libxml2-2.6.4\include\libxml\parser.h"
#include "libxml2-2.6.4\include\libxml\tree.h"
#include "libxml2-2.6.4\include\libxml\globals.h"

#define SAMPLE_FILENAME "ExportScene01.fbx"

FbxManager* lSdkManager;

// Function prototypes.
bool CreateScene(FbxManager* pSdkManager, FbxScene* pScene);

FbxNode* CreatePatch(FbxManager* pSdkManager, char* pName);
FbxNode* CreateSkeleton(FbxManager* pSdkManager, char* pName);

void LinkPatchToSkeleton(FbxManager* pSdkManager, FbxNode* pPatch, FbxNode* pSkeletonRoot);
void StoreBindPose(FbxManager* pSdkManager, FbxScene* pScene, FbxNode* pPatch, FbxNode* pSkeletonRoot);
void StoreRestPose(FbxManager* pSdkManager, FbxScene* pScene, FbxNode* pSkeletonRoot);
void AnimateSkeleton(FbxManager* pSdkManager, FbxScene* pScene, FbxNode* pSkeletonRoot);
void AddThumbnailToScene(FbxManager* pSdkManager, FbxScene* pScene);
void AddThumbnailToTake(FbxManager* pSdkManager, FbxScene* pScene, FbxString& pTakeName, int pThumbnailIndex);
void AddNodeRecursively(FbxArray<FbxNode*>& pNodeArray, FbxNode* pNode);

void SetXMatrix(FbxAMatrix& pXMatrix, const FbxMatrix& pMatrix);

FbxNode* createJoint(char *name,float x,float y,float z,FbxScene* scene)
{
	FbxString lName(name);
	FbxSkeleton* lSkeletonLimbNodeAttribute = FbxSkeleton::Create(scene, name);
	lSkeletonLimbNodeAttribute->SetSkeletonType(FbxSkeleton::eLimbNode);
	
	//lSkeletonLimbNodeAttribute->SetLimbNodeSize(100.0);

	FbxNode* lSkeletonLimbNode = FbxNode::Create(scene,lName.Buffer());
	lSkeletonLimbNode->SetNodeAttribute(lSkeletonLimbNodeAttribute);    
	lSkeletonLimbNode->LclTranslation.Set(FbxVector4(x, y, z));
	return lSkeletonLimbNode;
}

FbxNode* createRootJoint(char *name,float x,float y,float z,FbxScene* scene)
{
	FbxString lName(name);
	FbxSkeleton* lSkeletonLimbNodeAttribute = FbxSkeleton::Create(scene, name);
	lSkeletonLimbNodeAttribute->SetSkeletonType(FbxSkeleton::eRoot);
	
	//lSkeletonLimbNodeAttribute->SetLimbNodeSize(100.0);

	FbxNode* lSkeletonLimbNode = FbxNode::Create(scene,lName.Buffer());
	lSkeletonLimbNode->SetNodeAttribute(lSkeletonLimbNodeAttribute);    
	lSkeletonLimbNode->LclTranslation.Set(FbxVector4(x, y, z));
	return lSkeletonLimbNode;
}

void parseJoints(xmlNode *node,FbxNode *parent,FbxScene* scene)
{
	xmlNode *curNode = NULL;
	for (curNode = node; curNode; curNode = curNode->next) 
	{
		if (curNode->type == XML_ELEMENT_NODE) 
		{
			if(!xmlStrcmp((const xmlChar*)"Joint",curNode->name))
			{
				xmlChar *name;
				xmlChar *center;
				float x,y,z;
				name = xmlGetProp(curNode, (const xmlChar*)"sid");
				center = xmlGetProp(curNode, (const xmlChar*)"translation");
				sscanf((const char*)center,"%f %f %f",&x,&y,&z);				
				x*=100;
				y*=100;
				z*=100;

				FbxNode *joint = createJoint((char*)name,x,y,z,scene);
				parent->AddChild(joint);
				
				parseJoints(curNode->children,joint,scene);
				xmlFree(name);
				xmlFree(center);								
			}
		}		
	}	
}

FbxNode *parseRoot(xmlNode *node, FbxScene* scene)
{
	xmlNode *curNode = NULL;
	for (curNode = node; curNode; curNode = curNode->next) 
	{
		if (curNode->type == XML_ELEMENT_NODE) 
		{
			if(!xmlStrcmp((const xmlChar*)"Joint",curNode->name))
			{
				xmlChar *name;
				xmlChar *center;
				float x,y,z;
				name = xmlGetProp(curNode, (const xmlChar*)"sid");
				center = xmlGetProp(curNode, (const xmlChar*)"translation");
				sscanf((const char*)center,"%f %f %f",&x,&y,&z);				
				
				x*=100;
				y*=100;
				z*=100;
				FbxNode *root = createRootJoint((char*)name,x,y,z,scene);				

				parseJoints(curNode->children,root,scene);
				xmlFree(name);
				xmlFree(center);
				return root;
			}
		}
	}	
	return NULL;
}

FbxNode *parseHumanoid(xmlNode *node, FbxScene* scene)
{
	if(!xmlStrcmp((const xmlChar*)"MotionbuilderSkeleton",node->name))
	{
		return parseRoot(node->children,scene);
	}
	else
	{
		printf("XML file contains no humanoid\n");
		return NULL;
	}    
}

int main(int argc, char** argv)
{
	if(argc==3)
	{
		lSdkManager = NULL;
		FbxScene* lScene = NULL;

		// Prepare the FBX SDK.
		InitializeSdkObjects(lSdkManager, lScene);



		xmlDoc *doc = xmlReadFile(argv[1], NULL, 0);	
		
		if (doc == NULL) 
		{
			printf("error: could not parse file %s\n",argv[1]);
		}

		xmlNode *root = xmlDocGetRootElement(doc);

		FbxNode* lSkeletonRoot = parseHumanoid(root,lScene);	
		xmlFreeDoc(doc);
		xmlCleanupParser();
		
		FbxNode* lRootNode = lScene->GetRootNode();
		lRootNode->AddChild(lSkeletonRoot);
		
		SaveScene(lSdkManager, lScene, argv[2], false);
		
		// Destroy all objects created by the FBX SDK.
		DestroySdkObjects(lSdkManager,true);	

	}
	else
	{
		printf("Usage: SkeletonImport <humanoid.xml> <skeleton.fbx>\n");
	}
	return 0;
}