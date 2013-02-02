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
// This example illustrates how to detect if a scene is password 
// protected, import and browse the scene to access node and animation 
// information. It displays the content of the FBX file which name is 
// passed as program argument. You can try it with the various FBX files 
// output by the export examples.
//
/////////////////////////////////////////////////////////////////////////
#pragma warning(disable:4786) 

#include <fbxsdk.h>
#include "../Common/Common.h"
#include <fbxsdk/fbxsdk_def.h>
#include "jointrenaming.h"
#include <vector>
#include <map>
#include <math.h>
#define PI 3.1415926535897932384626433832795
//using namespace FBXSDK_NAMESPACE;
using namespace std;


const char *HANIM_JOINTS[]={
	"l_hip","l_knee","l_ankle","l_subtalar","l_midtarsal","l_metatarsal","r_hip","r_knee","r_ankle","r_subtalar",
	"r_midtarsal","r_metatarsal","vl5","vl4","vl3","vl2","vl1","vt12","vt11","vt10","vt9","vt8","vt7","vt6",
	"vt5","vt4","vt3","vt2","vt1","vc7","vc6","vc5","vc4","vc3","vc2","vc1","l_sternoclavicular",
	"l_acromioclavicular","l_shoulder","l_elbow","l_wrist","r_sternoclavicular","r_acromioclavicular","r_shoulder","r_elbow","r_wrist",
	"HumanoidRoot","sacroiliac","skullbase","l_pinky0","l_pinky1","l_pinky2","l_pinky3","l_ring0","l_ring1","l_ring2","l_ring3",
	"l_middle0","l_middle1","l_middle2","l_middle3","l_index0","l_index1","l_index2","l_index3","l_thumb1","l_thumb2","l_thumb3",
	"r_pinky0","r_pinky1","r_pinky2","r_pinky3","r_ring0","r_ring1","r_ring2","r_ring3","r_middle0","r_middle1","r_middle2","r_middle3",
	"r_index0","r_index1","r_index2","r_index3","r_thumb1","r_thumb2","r_thumb3","l_eyeball_joint","r_eyeball_joint","l_eyebrow_joint","r_eyebrow_joint",
	"l_eyelid_joint","r_eyelid_joint","temporomandibular",

	//hack hack: actor nodes
	"nodeHips", "nodeLeftHip", "nodeLeftKnee", "nodeLeftAnkle", "nodeRightHip", "nodeRightKnee", "nodeRightAnkle", "nodeWaist", 
	"nodeLeftCollar", "nodeLeftShoulder", "nodeLeftArmRoll", "nodeLeftElbow", "nodeLeftForeArmRoll", "nodeLeftWrist", "nodeRightCollar", 
	"nodeRightShoulder", "nodeRightArmRoll", "nodeRightElbow", "nodeRightForeArmRoll", "nodeRightWrist", "nodeNeck", "nodeHead"
};

const char *HANIM_TRANSLATORS[]=
{
	"vl5","sacroiliac","HumanoidRoot",

	//hack hack: actor nodes
	"nodeHips"
};

/*
const char *HANIM_TRANSLATORS[]=
{
	"HumanoidRoot"
};
const int HANIM_TRANSLATORNR=1;
*/
/* modified to include actor joints
const int HANIM_TRANSLATORNR=3;
const int HANIM_JOINTNR=92;
*/
const int HANIM_TRANSLATORNR=4;
const int HANIM_JOINTNR=116;

struct ltstr
{
  bool operator()(const char* s1, const char* s2) const
  {
    return strcmp(s1, s2) < 0;
  }
};

struct TranslationRotationTriple
{
	FbxAnimCurve *rotX;
	FbxAnimCurve *rotY;
	FbxAnimCurve *rotZ;
	//float defaultX,defaultY,defaultZ;
	FbxAnimCurve *transX;
	FbxAnimCurve *transY;
	FbxAnimCurve *transZ;
};

struct RotationTriple
{
	FbxAnimCurve *rotX;
	FbxAnimCurve *rotY;
	FbxAnimCurve *rotZ;
	//float defaultX,defaultY,defaultZ;
};

struct TranslationTriple
{
	FbxAnimCurve *transX;
	FbxAnimCurve *transY;
	FbxAnimCurve *transZ;
};

/*
struct DefaultJoint
{
	float rotX;
	float rotY;
	float rotZ;
	float transX;
	float transY;
	float transZ;
};

// Local function prototypes.
void GetSkeleton(KFbxScene* pScene,std::map <const char*, DefaultJoint,ltstr> &skeleton);
void GetSkeleton(KFbxNode* pNode,std::map <const char*, DefaultJoint,ltstr> &skeleton);
*/





const char *isHanim(const char *s)
{
	for(int i=0;i<HANIM_JOINTNR;i++)
	{
		if(strstr (s,HANIM_JOINTS[i])) 
		{
			return HANIM_JOINTS[i];
		}
	}
	//return NULL;
	return s;
}

const char *isTranslator(const char *s)
{
	//return isHanim(s);
	for(int i=0;i<HANIM_TRANSLATORNR;i++)
	{
		if(strstr (s,HANIM_TRANSLATORS[i])) 
		{
			return HANIM_TRANSLATORS[i];
		}
	}
	//return NULL;	
	return s;
}


const char * isValidRotation(FbxAnimLayer* pAnimLayer, FbxNode* pNode)
{
	float rot;
	
	//const char *name = isHanim(pNode->GetName());
	const char *name = isHanim(pNode->GetInitialName());
	
	
	//printf("Checking rot validity for %s\n",name);
	/*
	KFCurve *lCurveX = lCurrentTakeNode->GetEulerRotationX();
	KFCurve *lCurveY = lCurrentTakeNode->GetEulerRotationY();
	KFCurve *lCurveZ = lCurrentTakeNode->GetEulerRotationZ();
	*/
	FbxAnimCurve *lCurveX = pNode->LclRotation.GetCurve(pAnimLayer,FBXSDK_CURVENODE_COMPONENT_X);
	FbxAnimCurve *lCurveY = pNode->LclRotation.GetCurve(pAnimLayer,FBXSDK_CURVENODE_COMPONENT_Y);
	FbxAnimCurve *lCurveZ = pNode->LclRotation.GetCurve(pAnimLayer,FBXSDK_CURVENODE_COMPONENT_Z);

	int i;
	if(name && lCurveX && lCurveX->KeyGetCount()>1)
	{
		for(i=0;i<lCurveX->KeyGetCount();i++)
		{
			rot = lCurveX->KeyGetValue(i);				
			if(rot!=0)return name;
		}

		for(i=0;i<lCurveY->KeyGetCount();i++)
		{
			rot = lCurveY->KeyGetValue(i);
			if(rot!=0)return name;
		}

		for(i=0;i<lCurveZ->KeyGetCount();i++)
		{
			rot = lCurveZ->KeyGetValue(i);
			if(rot!=0)return name;
		}	
	}
	//printf("Invalid rot %s, take %s\n",name,lCurrentTakeName);
	//if(!lCurveX) printf("lCurveX == null\n",name);
	return NULL;
}

/*
const char * isValidTranslation(KFbxNode* pNode,std::map <const char*, DefaultJoint,ltstr> skeleton)
{
	float transX, transY, transZ;
	KFbxTakeNode* lCurrentTakeNode = pNode->GetCurrentTakeNode();
	
	const char *name = isTranslator(pNode->GetName());
	KFCurve *lCurveX = lCurrentTakeNode->GetTranslationX();
	KFCurve *lCurveY = lCurrentTakeNode->GetTranslationY();
	KFCurve *lCurveZ = lCurrentTakeNode->GetTranslationZ();
	
	int i;
	DefaultJoint j;
	if(name && lCurveX->KeyGetCount()>1)
	{
		j = skeleton[name];
		for(i=0;i<lCurveX->KeyGetCount();i++)
		{
			transX = lCurveX->KeyGetValue(i);				
			//printf("transX, j.transX=%f,%f\n",transX, j.transX);
			if(j.transX != transX)
			{
				//printf("%s x offset(%d): %f\n",name,i ,j.transX-lCurveX->KeyGetValue(i));
				return name;			
			}
		}

		for(i=0;i<lCurveY->KeyGetCount();i++)
		{
			transY = lCurveY->KeyGetValue(i);
			//printf("transY, j.transY=%f,%f\n",transY, j.transY);
			if(j.transY != transY) 
			{
				//printf("%s y offset(%d): %f\n",name, i,j.transX-lCurveY->KeyGetValue(i));
				return name;
			}
		}

		for(i=0;i<lCurveZ->KeyGetCount();i++)
		{
			transZ = lCurveZ->KeyGetValue(i);
			//printf("transZ, j.transZ=%f,%f\n",transZ, j.transZ);
			if(j.transZ != transZ) 
			{
				//printf("%s z offset(%d): %f\n",name, i,j.transX-lCurveZ->KeyGetValue(i));
				return name;
			}
		}	
	}
	return NULL;
}
*/

//a translation is valid if the node moves irt its start position
const char * isValidTranslation(FbxAnimLayer* pAnimLayer, FbxNode* pNode)
{
	float transX, transY, transZ;
	//KFbxTakeNode* lCurrentTakeNode = pNode->GetCurrentTakeNode();
	//char* lCurrentTakeName = pNode->GetCurrentTakeNodeName();
	
	const char *name = isTranslator(pNode->GetName());
	
	/*
	KFCurve *lCurveX = lCurrentTakeNode->GetTranslationX();
	KFCurve *lCurveY = lCurrentTakeNode->GetTranslationY();
	KFCurve *lCurveZ = lCurrentTakeNode->GetTranslationZ();
	*/	
	FbxAnimCurve *lCurveX = pNode->LclRotation.GetCurve(pAnimLayer,FBXSDK_CURVENODE_COMPONENT_X);
	FbxAnimCurve *lCurveY = pNode->LclRotation.GetCurve(pAnimLayer,FBXSDK_CURVENODE_COMPONENT_Y);
	FbxAnimCurve *lCurveZ = pNode->LclRotation.GetCurve(pAnimLayer,FBXSDK_CURVENODE_COMPONENT_Z);
	
	int i;
	//DefaultJoint j;
	if(name && lCurveX && lCurveX->KeyGetCount()>1)
	{
		for(i=0;i<lCurveX->KeyGetCount();i++)
		{
			float transStart = lCurveX->KeyGetValue(0);				
			transX = lCurveX->KeyGetValue(i);				
			//printf("transX, j.transX=%f,%f\n",transX, j.transX);
			if(fabs (transX - transStart)>0.01)
			{
				//printf("%s x offset(%d): %f start:%f \n",name, i,transX,transStart);
				return name;			
			}
		}

		for(i=0;i<lCurveY->KeyGetCount();i++)
		{
			float transStart = lCurveY->KeyGetValue(0);		
			transY = lCurveY->KeyGetValue(i);
			//printf("transY, j.transY=%f,%f\n",transY, j.transY);
			if(fabs (transY - transStart) > 0.01) 
			{
				//printf("%s y offset(%d): %f start:%f \n",name, i,transY,transStart);
				return name;
			}
		}

		for(i=0;i<lCurveZ->KeyGetCount();i++)
		{
			float transStart = lCurveZ->KeyGetValue(0);		
			transZ = lCurveZ->KeyGetValue(i);
			//printf("transZ, j.transZ=%f,%f\n",transZ, transStart);
			if(fabs (transZ - transStart) > 0.01) 
			{
				//printf("%s y offset(%d): %f start:%f \n",name, i,transZ,transStart);	
				return name;
			}
		}	
	}
	return NULL;
}
/*
const char *isTranslator(const char *s)
{
	for(int i=0;i<HANIM_TRANSLATORNR;i++)
	{
		if(strstr (s,HANIM_TRANSLATORS[i])) 
		{
			return HANIM_TRANSLATORS[i];
		}
	}
	return NULL;
}*/

void euler2Quaternion(float pitch,float yaw,float roll, float& x,float& y,float& z, float&w)
{
	float cx = (float)cos(pitch/2.0);
	float cy = (float)cos(yaw/2.0);
	float cz = (float)cos(roll/2.0);
	float sx = (float)sin(pitch/2.0);
	float sy = (float)sin(yaw/2.0);
	float sz = (float)sin(roll/2.0);
	w = cx * cy * cz + sx * sy * sz;
	x = cx * sy * sz + sx * cy * cz;
	y = cx * sy * cz - sx * cy * sz;
	z = cx * cy * sz - sx * sy * cz;    
}
/**
 * Converts euler angles to axis angles.
 */
void euler2AxisAngle(float ex,float ey,float ez, float& x,float& y,float& z, float& angle) {
  float s1,s2,s3,c1,c2,c3,norm;
  
  /*
    assumption:  
    ex = attitude
    ey = heading  
    ez = bank
    
    then this is true:
    c1 = cos(heading / 2) 
    c2 = cos(attitude / 2) 
    c3 = cos(bank / 2) 
    s1 = sin(heading / 2) 
    s2 = sin(attitude / 2) 
    s3 = sin(bank / 2) 
  */
  if(ex==0&&ey==0&&ez==0) {
    x=1;y=0;z=0;angle=0;
    return;
  }

  c1=(float)cos(ez/2);
  c2=(float)cos(ey/2);
  c3=(float)cos(ex/2);
  s1=(float)sin(ez/2);
  s2=(float)sin(ey/2);
  s3=(float)sin(ex/2);

  /*
    angle = 2 * acos(c1c2c3 + s1s2s3)
    x = c1c2s3 - s1s2c3
    y = c1s2c3 + s1c2s3
    z = s1c2c3 - c1s2s3
  */

  angle = 2 * (float)acos(c1*c2*c3 + s1*s2*s3);

  x = c1*c2*s3 - s1*s2*c3;
  y = c1*s2*c3 + s1*c2*s3;
  z = s1*c2*c3 - c1*s2*s3;

  norm = (float)sqrt(x*x+y*y+z*z);
  x/=norm;
  y/=norm;
  z/=norm;
}

/*
void GetBothKeys(KFbxNode* pNode,vector<TranslationRotationTriple> &transrot,const std::map <const char*, DefaultJoint,ltstr> skeleton)
{
    KFbxTakeNode* lCurrentTakeNode = pNode->GetCurrentTakeNode();
	KFbxTakeNode* lDefaultTakeNode = pNode->GetDefaultTakeNode();
	int lModelCount;
	//bool lFirst = first;

	//printf("\nlfirst: %d\n",lFirst);
	// Display nothing if the current take node points to default values.
	float startRotX,startRotY,startRotZ;
	
	startRotX = lDefaultTakeNode->GetEulerRotationX()->GetValue();
	startRotY = lDefaultTakeNode->GetEulerRotationY()->GetValue();
	startRotZ = lDefaultTakeNode->GetEulerRotationZ()->GetValue();
	//printf("start rotations: %f %f %f",startRotX,startRotY,startRotZ);

    if(lCurrentTakeNode && lCurrentTakeNode != lDefaultTakeNode)
    {
		const char *name = isValidRotation(pNode);
		TranslationRotationTriple r;
		r.rotX = lCurrentTakeNode->GetEulerRotationX();
		r.rotY = lCurrentTakeNode->GetEulerRotationY();
		r.rotZ = lCurrentTakeNode->GetEulerRotationZ();
		r.defaultX = startRotX;
		r.defaultY = startRotY;
		r.defaultZ = startRotZ;
		if(name && r.rotX->KeyGetCount()>1)
		{
			//printf("rot pushback\n");
			r.transX = lCurrentTakeNode->GetTranslationX();
			r.transY = lCurrentTakeNode->GetTranslationY();
			r.transZ = lCurrentTakeNode->GetTranslationZ();
			if(!isValidTranslation(pNode) || r.transX->KeyGetCount()<=1)
			{
				r.transX = NULL;
				r.transY = NULL;
				r.transZ = NULL;
			}
			transrot.push_back(r);						
		}
	}

	for(lModelCount = 0; lModelCount < pNode->GetChildCount(); lModelCount++)
    {
        GetBothKeys(pNode->GetChild(lModelCount),transrot,skeleton);
    }	
}
*/

void GetBothKeys(FbxNode* pNode, FbxAnimLayer* pAnimLayer,vector<TranslationRotationTriple> &transrot)
{
    /*
	KFbxTakeNode* lCurrentTakeNode = pNode->GetCurrentTakeNode();
	KFbxTakeNode* lDefaultTakeNode = pNode->GetDefaultTakeNode();
	*/
	int lModelCount;
	//bool lFirst = first;

	//printf("\nlfirst: %d\n",lFirst);
	// Display nothing if the current take node points to default values.
	//float startRotX,startRotY,startRotZ;
	
	/*
	startRotX = lDefaultTakeNode->GetEulerRotationX()->GetValue();
	startRotY = lDefaultTakeNode->GetEulerRotationY()->GetValue();
	startRotZ = lDefaultTakeNode->GetEulerRotationZ()->GetValue();
	*/
	
	/*
	startRotX = pNode->LclRotation.GetKFCurve(KFCURVENODE_R_X, lDefaultTakeName)->GetValue();
	startRotY = pNode->LclRotation.GetKFCurve(KFCURVENODE_R_Y, lDefaultTakeName)->GetValue();
	startRotZ = pNode->LclRotation.GetKFCurve(KFCURVENODE_R_Z, lDefaultTakeName)->GetValue();
	*/

	//printf("start rotations: %f %f %f",startRotX,startRotY,startRotZ);

    //if(lCurrentTakeName /*&& lCurrentTakeName != lDefaultTakeName*/)
    {
		const char *name = isValidRotation(pAnimLayer,pNode);
		TranslationRotationTriple r;
		/*
		r.rotX = lCurrentTakeNode->GetEulerRotationX();
		r.rotY = lCurrentTakeNode->GetEulerRotationY();
		r.rotZ = lCurrentTakeNode->GetEulerRotationZ();
		*/

		

		r.rotX = pNode->LclRotation.GetCurve(pAnimLayer,FBXSDK_CURVENODE_COMPONENT_X);
		r.rotY = pNode->LclRotation.GetCurve(pAnimLayer,FBXSDK_CURVENODE_COMPONENT_Y);
		r.rotZ = pNode->LclRotation.GetCurve(pAnimLayer,FBXSDK_CURVENODE_COMPONENT_Z);
		/*		
		r.defaultX = startRotX;
		r.defaultY = startRotY;
		r.defaultZ = startRotZ;
		*/
		if(name && r.rotX->KeyGetCount()>1)
		{
			//printf("rot pushback\n");
			/*
			r.transX = lCurrentTakeNode->GetTranslationX();
			r.transY = lCurrentTakeNode->GetTranslationY();
			r.transZ = lCurrentTakeNode->GetTranslationZ();
			*/			
			r.rotX = pNode->LclRotation.GetCurve(pAnimLayer,FBXSDK_CURVENODE_COMPONENT_X);
			r.rotY = pNode->LclRotation.GetCurve(pAnimLayer,FBXSDK_CURVENODE_COMPONENT_Y);
			r.rotZ = pNode->LclRotation.GetCurve(pAnimLayer,FBXSDK_CURVENODE_COMPONENT_Z);

			//if(!isValidTranslation(pAnimLayer,pNode) || r.transX->KeyGetCount()<=1)
			//2013fix
			if(!isValidTranslation(pAnimLayer,pNode))
			{
				r.transX = NULL;
				r.transY = NULL;
				r.transZ = NULL;
			}
			transrot.push_back(r);						
		}
	}

	for(lModelCount = 0; lModelCount < pNode->GetChildCount(); lModelCount++)
    {
        GetBothKeys(pNode->GetChild(lModelCount),pAnimLayer,transrot);
    }	
}

void GetRotationKeys(FbxNode* pNode, FbxAnimLayer* pAnimLayer,vector<RotationTriple> &rot)
{
    /*
	KFbxTakeNode* lCurrentTakeNode = pNode->GetCurrentTakeNode();
	KFbxTakeNode* lDefaultTakeNode = pNode->GetDefaultTakeNode();
	*/
	//char* lCurrentTakeName = pNode->GetCurrentTakeNodeName(); 
    //char* lDefaultTakeName = pNode->GetTakeNodeName(0);
	int lModelCount;
	//bool lFirst = first;

	//printf("\nlfirst: %d\n",lFirst);
	// Display nothing if the current take node points to default values.
	
	/*
	startRotX = lDefaultTakeNode->GetEulerRotationX()->GetValue();
	startRotY = lDefaultTakeNode->GetEulerRotationY()->GetValue();
	startRotZ = lDefaultTakeNode->GetEulerRotationZ()->GetValue();
	*/
	/*
	startRotX = pNode->LclRotation.GetKFCurve(KFCURVENODE_R_X, lDefaultTakeName)->GetValue();
	startRotY = pNode->LclRotation.GetKFCurve(KFCURVENODE_R_Y, lDefaultTakeName)->GetValue();
	startRotZ = pNode->LclRotation.GetKFCurve(KFCURVENODE_R_Z, lDefaultTakeName)->GetValue();
	*/
	//printf("start rotations: %f %f %f",startRotX,startRotY,startRotZ);

    //if(lCurrentTakeName /*&& lCurrentTakeName != lDefaultTakeName*/)
    {
		const char *name = isValidRotation(pAnimLayer,pNode);
		RotationTriple r;

		r.rotX = pNode->LclRotation.GetCurve(pAnimLayer,FBXSDK_CURVENODE_COMPONENT_X);
		r.rotY = pNode->LclRotation.GetCurve(pAnimLayer,FBXSDK_CURVENODE_COMPONENT_Y);
		r.rotZ = pNode->LclRotation.GetCurve(pAnimLayer,FBXSDK_CURVENODE_COMPONENT_Z);
		//r.defaultX = startRotX;
		//r.defaultY = startRotY;
		//r.defaultZ = startRotZ;
		if(name && r.rotX->KeyGetCount()>1)
		{
			//printf("rot pushback\n");
			rot.push_back(r);			
		}
	}

	for(lModelCount = 0; lModelCount < pNode->GetChildCount(); lModelCount++)
    {
        GetRotationKeys(pNode->GetChild(lModelCount),pAnimLayer,rot);
    }	
}

/*
void GetTranslationKeys(KFbxNode* pNode,vector<TranslationTriple> &trans,const std::map <const char*, DefaultJoint,ltstr> skeleton)
{
    KFbxTakeNode* lCurrentTakeNode = pNode->GetCurrentTakeNode();
	int lModelCount;

    if(lCurrentTakeNode && lCurrentTakeNode != pNode->GetDefaultTakeNode())
    {
		TranslationTriple t;
		const char *name = isValidTranslation(pNode);
		t.transX = lCurrentTakeNode->GetTranslationX();
		t.transY = lCurrentTakeNode->GetTranslationY();
		t.transZ = lCurrentTakeNode->GetTranslationZ();

		if(name && t.transX->KeyGetCount()>1)
		{
			trans.push_back(t);			
		}
	}

	for(lModelCount = 0; lModelCount < pNode->GetChildCount(); lModelCount++)
    {
        GetTranslationKeys(pNode->GetChild(lModelCount),trans);
    }	
}
*/
void GetTranslationKeys(FbxNode* pNode, FbxAnimLayer* pAnimLayer,vector<TranslationTriple> &trans)
{
    //KFbxTakeNode* lCurrentTakeNode = pNode->GetCurrentTakeNode();
	int lModelCount;

    //if(lCurrentTakeName /*&& lCurrentTakeName != lDefaultTakeName*/)
    {
		TranslationTriple t;
		const char *name = isValidTranslation(pAnimLayer, pNode);		
		t.transX = pNode->LclRotation.GetCurve(pAnimLayer,FBXSDK_CURVENODE_COMPONENT_X);
		t.transY = pNode->LclRotation.GetCurve(pAnimLayer,FBXSDK_CURVENODE_COMPONENT_Y);
		t.transZ = pNode->LclRotation.GetCurve(pAnimLayer,FBXSDK_CURVENODE_COMPONENT_Z);

		if(name && t.transX->KeyGetCount()>1)
		{
			trans.push_back(t);			
		}
	}

	for(lModelCount = 0; lModelCount < pNode->GetChildCount(); lModelCount++)
    {
        GetTranslationKeys(pNode->GetChild(lModelCount),pAnimLayer,trans);
    }	
}
//void GetBothKeys(KFbxNode* pNode,const std::map <const char*, DefaultJoint,ltstr> skeleton, int numTrans)
//{
//	vector<TranslationRotationTriple> trans;
//	GetBothKeys(pNode,trans);
//
//	if(trans.size()==0)
//		return;
//
//	bool first;
//	vector<TranslationRotationTriple>::iterator r=trans.begin();
//	int keys = (*r).rotX->KeyGetCount();
//	float rotX,rotY,rotZ;
//	float x=0,y=0,z=0,w=1;
//	float startRotX,startRotY,startRotZ;
//	
//	float *xOld=new float[trans.size()];
//	float *yOld=new float[trans.size()];
//	float *zOld=new float[trans.size()];
//	float *wOld=new float[trans.size()];
//
//	float transOffsetX=0,transOffsetZ=0;
//	if(r!=trans.end() && numTrans>0)
//	{
//		transOffsetX = (*r).transX->KeyGetValue(0)/100;
//		transOffsetZ = (*r).transZ->KeyGetValue(0)/100;
//	}
//
//	for(int i = 0;i<keys ;i++)
//	{
//		first=true;
//		int j = 0;
//		for (r = trans.begin(); r != trans.end(); r++)
//		{
//			if(first)
//			{
//				KTime   lKeyTime=(*r).rotX->KeyGetTime(i);
//				//lKeyTime.GetTimeString(lTimeString);
//				//printf("%d",(int)(lKeyTime.GetSecondDouble()*1000.0));
//				printf("%f ",lKeyTime.GetSecondDouble());
//				first=false;				
//			}
//			else
//			{
//				printf(" ");
//			}
//
//			if((*r).transX==NULL)
//			{
//				if(numTrans>1)
//				{
//					printf("0 0 0");
//				}
//			}			
//			else if(i<(*r).transX->KeyGetCount()&& i<(*r).transY->KeyGetCount()&&i<(*r).transZ->KeyGetCount())
//			{
//				float transX = (*r).transX->KeyGetValue(i)/100.0-transOffsetX;
//				float transY = (*r).transY->KeyGetValue(i)/100.0;
//				float transZ = (*r).transZ->KeyGetValue(i)/100.0-transOffsetZ;
//				
//				/* hack hack, enable for -90 degree turn
//				float transZ = static_cast<float>((*t).transX->KeyGetValue(i))/100-transOffsetX;
//				float transY = static_cast<float>((*t).transY->KeyGetValue(i))/100;
//				float transX = -static_cast<float>((*t).transZ->KeyGetValue(i))/100-transOffsetZ;
//				*/
//
//				printf("%f %f %f",transX,transY,transZ);
//			}
//			else
//			{
//				printf("Not enough translation keys on i:%d\n",i);
//				printf("# transX :%d\n",(*r).transX->KeyGetCount());
//				printf("# transY :%d\n",(*r).transY->KeyGetCount());
//				printf("# transZ :%d\n",(*r).transZ->KeyGetCount());
//			}
//
//			
//			
//			if( i<(*r).rotX->KeyGetCount()  && i<(*r).rotY->KeyGetCount() && i<(*r).rotZ->KeyGetCount())
//			{
//				rotX = (*r).rotX->KeyGetValue(i); //- (*r).defaultX;
//				rotY = (*r).rotY->KeyGetValue(i); //- (*r).defaultY;
//				rotZ = (*r).rotZ->KeyGetValue(i); //- (*r).defaultZ;			
//				/*
//				printf("\n%d Rot x,y,z %f %f %f\n",i,rotX,rotY,rotZ);
//				printf("%d Default x,y,z %f %f %f\n",i,(*r).defaultX,(*r).defaultY,(*r).defaultZ);
//				printf("%d Rot in curve x,y,z %f %f %f\n",i,(*r).rotX->KeyGetValue(i),(*r).rotY->KeyGetValue(i),(*r).rotZ->KeyGetValue(i));
//				printf("%d Default in curve x,y,z %f %f %f\n",i,(*r).rotX->GetValue(),(*r).rotY->GetValue(),(*r).rotZ->GetValue());
//				//euler2Quaternion(rotX*PI/180,rotY*PI/180,rotZ*PI/180, x,y,z,w);
//				*/
//
//				euler2AxisAngle(rotX*PI/180,rotY*PI/180,rotZ*PI/180, x,y,z,w);
//				//printf(" %f %f %f %f",x,y,z,w);
//				
//				//euler2Quaternion(rotX*PI/180,rotY*PI/180,rotZ*PI/180, x,y,z,w);
//				float angle = w;
//				w = cos(angle/2);
//				x = sin(angle/2)*x;
//				y = sin(angle/2)*y;
//				z = sin(angle/2)*z;
//				if(i>0)
//				{
//					//flip coordinates if needed so that this quaternion 
//					//is the minimum distance from the previous rotation
//					if(x*xOld[j]+y*yOld[j]+z*zOld[j]+w*wOld[j]<0)
//					{
//						x = -x;
//						y = -y;
//						z = -z;
//						w = -w;
//					}
//				}
//				printf(" %f %f %f %f",w,x,y,z);
//			}
//			else
//			{
//				printf("Not enough rotation keys on i:%d\n",i);
//				printf("# rotX :%d\n",(*r).rotX->KeyGetCount());
//				printf("# rotY :%d\n",(*r).rotY->KeyGetCount());
//				printf("# rotZ :%d\n",(*r).rotZ->KeyGetCount());				
//			}		
//			xOld[j] = x;
//			yOld[j] = y;
//			zOld[j] = z;
//			wOld[j] = w;
//			j++;
//		}   				
//		if(!first)
//			printf("\n");
//	}	
//	delete[] xOld;
//	delete[] yOld;
//	delete[] zOld;
//	delete[] wOld;
//}

void GetBothKeys(FbxNode* pNode, FbxAnimLayer* pAnimLayer, int numTrans)
{
	vector<TranslationRotationTriple> trans;
	GetBothKeys(pNode,pAnimLayer,trans);

	if(trans.size()==0)
		return;

	bool first;
	vector<TranslationRotationTriple>::iterator r=trans.begin();
	int keys = (*r).rotX->KeyGetCount();
	float rotX,rotY,rotZ;
	float x=0,y=0,z=0,w=1;
	//float startRotX,startRotY,startRotZ;
	
	float *xOld=new float[trans.size()];
	float *yOld=new float[trans.size()];
	float *zOld=new float[trans.size()];
	float *wOld=new float[trans.size()];

	float transOffsetX=0,transOffsetZ=0;
	if(r!=trans.end() && numTrans>0)
	{
		transOffsetX = (*r).transX->KeyGetValue(0)/100;
		transOffsetZ = (*r).transZ->KeyGetValue(0)/100;
	}

	for(int i = 0;i<keys ;i++)
	{
		first=true;
		int j = 0;
		for (r = trans.begin(); r != trans.end(); r++)
		{
			if(first)
			{
				FbxTime   lKeyTime=(*r).rotX->KeyGetTime(i);
				//lKeyTime.GetTimeString(lTimeString);
				//printf("%d",(int)(lKeyTime.GetSecondDouble()*1000.0));
				printf("%f ",lKeyTime.GetSecondDouble());
				first=false;				
			}
			else
			{
				printf(" ");
			}

			if((*r).transX==NULL)
			{
				if(numTrans>1)
				{
					printf("0 0 0");
				}
			}			
			else if(i<(*r).transX->KeyGetCount()&& i<(*r).transY->KeyGetCount()&&i<(*r).transZ->KeyGetCount())
			{
				float transX = (*r).transX->KeyGetValue(i)/100.0-transOffsetX;
				float transY = (*r).transY->KeyGetValue(i)/100.0;
				float transZ = (*r).transZ->KeyGetValue(i)/100.0-transOffsetZ;
				
				/* hack hack, enable for -90 degree turn
				float transZ = static_cast<float>((*t).transX->KeyGetValue(i))/100-transOffsetX;
				float transY = static_cast<float>((*t).transY->KeyGetValue(i))/100;
				float transX = -static_cast<float>((*t).transZ->KeyGetValue(i))/100-transOffsetZ;
				*/

				printf("%f %f %f",transX,transY,transZ);
			}
			else
			{
				printf("Not enough translation keys on i:%d\n",i);
				printf("# transX :%d\n",(*r).transX->KeyGetCount());
				printf("# transY :%d\n",(*r).transY->KeyGetCount());
				printf("# transZ :%d\n",(*r).transZ->KeyGetCount());
			}

			
			
			if( i<(*r).rotX->KeyGetCount()  && i<(*r).rotY->KeyGetCount() && i<(*r).rotZ->KeyGetCount())
			{
				rotX = (*r).rotX->KeyGetValue(i); //- (*r).defaultX;
				rotY = (*r).rotY->KeyGetValue(i); //- (*r).defaultY;
				rotZ = (*r).rotZ->KeyGetValue(i); //- (*r).defaultZ;			
				/*
				printf("\n%d Rot x,y,z %f %f %f\n",i,rotX,rotY,rotZ);
				printf("%d Default x,y,z %f %f %f\n",i,(*r).defaultX,(*r).defaultY,(*r).defaultZ);
				printf("%d Rot in curve x,y,z %f %f %f\n",i,(*r).rotX->KeyGetValue(i),(*r).rotY->KeyGetValue(i),(*r).rotZ->KeyGetValue(i));
				printf("%d Default in curve x,y,z %f %f %f\n",i,(*r).rotX->GetValue(),(*r).rotY->GetValue(),(*r).rotZ->GetValue());
				//euler2Quaternion(rotX*PI/180,rotY*PI/180,rotZ*PI/180, x,y,z,w);
				*/

				euler2AxisAngle(rotX*PI/180,rotY*PI/180,rotZ*PI/180, x,y,z,w);
				//printf(" %f %f %f %f",x,y,z,w);
				
				//euler2Quaternion(rotX*PI/180,rotY*PI/180,rotZ*PI/180, x,y,z,w);
				float angle = w;
				w = cos(angle/2);
				x = sin(angle/2)*x;
				y = sin(angle/2)*y;
				z = sin(angle/2)*z;
				if(i>0)
				{
					//flip coordinates if needed so that this quaternion 
					//is the minimum distance from the previous rotation
					if(x*xOld[j]+y*yOld[j]+z*zOld[j]+w*wOld[j]<0)
					{
						x = -x;
						y = -y;
						z = -z;
						w = -w;
					}
				}
				printf(" %f %f %f %f",w,x,y,z);
			}
			else
			{
				printf("Not enough rotation keys on i:%d\n",i);
				printf("# rotX :%d\n",(*r).rotX->KeyGetCount());
				printf("# rotY :%d\n",(*r).rotY->KeyGetCount());
				printf("# rotZ :%d\n",(*r).rotZ->KeyGetCount());				
			}		
			xOld[j] = x;
			yOld[j] = y;
			zOld[j] = z;
			wOld[j] = w;
			j++;
		}   				
		if(!first)
			printf("\n");
	}	
	delete[] xOld;
	delete[] yOld;
	delete[] zOld;
	delete[] wOld;
}


//void GetKeys(KFbxNode* pNode,const std::map <const char*, DefaultJoint,ltstr> skeleton)
//{
//	vector<RotationTriple> rot;
//	vector<TranslationTriple> trans;
//	GetTranslationKeys(pNode,trans,skeleton);
//	GetRotationKeys(pNode,rot);
//	
//	if(rot.size()==0 && trans.size()==0)
//		return;
//
//	bool first;
//	vector<RotationTriple>::iterator r=rot.begin();
//	int keys = (*r).rotX->KeyGetCount();
//	float rotX,rotY,rotZ;
//	float x,y,z,w;
//	float startRotX,startRotY,startRotZ;
//	
//	
//	for(int i = 0;i<keys ;i++)
//	{
//		first=true;
//		for (r = rot.begin(); r != rot.end(); r++)
//		{
//			if(first)
//			{
//				KTime   lKeyTime=(*r).rotX->KeyGetTime(i);
//				//lKeyTime.GetTimeString(lTimeString);
//				//printf("%d",(int)(lKeyTime.GetSecondDouble()*1000.0));
//				printf("%f",lKeyTime.GetSecondDouble());
//				first=false;				
//			}
//			
//			if( i<(*r).rotX->KeyGetCount()  && i<(*r).rotY->KeyGetCount() && i<(*r).rotZ->KeyGetCount())
//			{
//				rotX = (*r).rotX->KeyGetValue(i) - (*r).defaultX;
//				rotY = (*r).rotY->KeyGetValue(i) - (*r).defaultY;
//				rotZ = (*r).rotZ->KeyGetValue(i) - (*r).defaultZ;			
//				/*
//				printf("\n%d Rot x,y,z %f %f %f\n",i,rotX,rotY,rotZ);
//				printf("%d Default x,y,z %f %f %f\n",i,(*r).defaultX,(*r).defaultY,(*r).defaultZ);
//				printf("%d Rot in curve x,y,z %f %f %f\n",i,(*r).rotX->KeyGetValue(i),(*r).rotY->KeyGetValue(i),(*r).rotZ->KeyGetValue(i));
//				printf("%d Default in curve x,y,z %f %f %f\n",i,(*r).rotX->GetValue(),(*r).rotY->GetValue(),(*r).rotZ->GetValue());
//				//euler2Quaternion(rotX*PI/180,rotY*PI/180,rotZ*PI/180, x,y,z,w);
//				*/
//				euler2AxisAngle(rotX*PI/180,rotY*PI/180,rotZ*PI/180, x,y,z,w);
//				
//				printf(" %f %f %f %f",x,y,z,w);
//			}
//			else
//			{
//				printf("Not enough rotation keys on i:%d\n",i);
//				printf("# rotX :%d\n",(*r).rotX->KeyGetCount());
//				printf("# rotY :%d\n",(*r).rotY->KeyGetCount());
//				printf("# rotZ :%d\n",(*r).rotZ->KeyGetCount());				
//			}			
//		}   		
//		vector<TranslationTriple>::iterator t;
//		t = trans.begin();
//		float transOffsetX=0,transOffsetZ=0;
//		if(t!=trans.end())
//		{
//			transOffsetX = (*t).transX->KeyGetValue(0)/100.0;
//			transOffsetZ = (*t).transZ->KeyGetValue(0)/100.0;
//		}
//		for (t = trans.begin(); t != trans.end(); t++)
//		{
//			//printf("nr of trans keys for i: %d=%d\n",i,trans.size());
//			if(first)
//			{
//				printf("%f",(*t).transX->KeyGetTime(i));				
//				first=false;
//			}
//			else
//			{
//				printf(" ");
//			}
//			if(i<(*t).transX->KeyGetCount()&& i<(*t).transY->KeyGetCount()&&i<(*t).transZ->KeyGetCount())
//			{
//				float transX = (*t).transX->KeyGetValue(i)/100.0-transOffsetX;
//				float transY = (*t).transY->KeyGetValue(i)/100.0;
//				float transZ = (*t).transZ->KeyGetValue(i)/100.0-transOffsetZ;
//				
//				/* hack hack, enable for -90 degree turn
//				float transZ = static_cast<float>((*t).transX->KeyGetValue(i))/100-transOffsetX;
//				float transY = static_cast<float>((*t).transY->KeyGetValue(i))/100;
//				float transX = -static_cast<float>((*t).transZ->KeyGetValue(i))/100-transOffsetZ;
//				*/
//
//				printf("%f %f %f",transX,transY,transZ);
//			}
//			else
//			{
//				printf("Not enough translation keys on i:%d\n",i);
//				printf("# transX :%d\n",(*t).transX->KeyGetCount());
//				printf("# transY :%d\n",(*t).transY->KeyGetCount());
//				printf("# transZ :%d\n",(*t).transZ->KeyGetCount());
//			}
//		}			
//		if(!first)
//			printf("\n");
//	}	
//}

int GetRotatedParts(FbxNode* pNode,FbxAnimLayer* pAnimLayer,std::string parts[], int numParts)
{
    //KFbxTakeNode* lCurrentTakeNode = pNode->GetCurrentTakeNode();
	//char* lCurrentTakeName = pNode->GetCurrentTakeNodeName();
    //char* lDefaultTakeName = pNode->GetTakeNodeName(0);

	int lModelCount;
	const char *name;
	//bool lFirst = first;

	//printf("\nlfirst: %d\n",lFirst);
	// Display nothing if the current take node points to default values.
    //if(lCurrentTakeName /*&& lCurrentTakeName != lDefaultTakeName*/)
    {
		
		name = isValidRotation(pAnimLayer,pNode);
		if(name)
		{
			//TODO: add part
			int found = 0;
			for(int i=0;i<numParts;i++)
			{
				if(!strcmp(parts[i].c_str(),name))
				{
					found = 1;
				}
			}
			if(!found)
			{
				parts[numParts]=name;
				numParts++;
			}
		}
	
		//DisplayChannels(pNode, lCurrentTakeNode, DisplayCurveKeys, DisplayListCurveKeys, isSwitcher);		
	}

	for(lModelCount = 0; lModelCount < pNode->GetChildCount(); lModelCount++)
    {
        numParts = GetRotatedParts(pNode->GetChild(lModelCount),pAnimLayer,parts,numParts);
    }	
	return numParts;
}

/*
int GetTranslatedParts(KFbxNode* pNode,const std::map <const char*, DefaultJoint,ltstr> skeleton, std::string parts[], int numParts)
{
	KFbxTakeNode* lCurrentTakeNode = pNode->GetCurrentTakeNode();
	int lModelCount;
	// Display nothing if the current take node points to default values.
    if(lCurrentTakeNode && lCurrentTakeNode != pNode->GetDefaultTakeNode())
    {
		const char *name = isValidTranslation(pNode);
		KFCurve *lCurve = lCurrentTakeNode->GetTranslationX();

		if(name && lCurve->KeyGetCount()>1)
		{
			//printf(name);						
			//sprintf("%s",name[numParts]);
			parts[numParts]=name;
			numParts++;			
		}
	
		//DisplayChannels(pNode, lCurrentTakeNode, DisplayCurveKeys, DisplayListCurveKeys, isSwitcher);		
	}

	for(lModelCount = 0; lModelCount < pNode->GetChildCount(); lModelCount++)
    {
        numParts = GetTranslatedParts(pNode->GetChild(lModelCount),skeleton,parts,numParts);
    }	
	return numParts;
}
*/
int GetTranslatedParts(FbxNode* pNode, FbxAnimLayer* pAnimLayer, std::string parts[], int numParts)
{
	//char* lCurrentTakeName = pNode->GetCurrentTakeNodeName();
    //char* lDefaultTakeName = pNode->GetTakeNodeName(0);

	int lModelCount;
	// Display nothing if the current take node points to default values.
    //if(lCurrentTakeName /*&& lCurrentTakeName != lDefaultTakeName*/)
    {
		const char *name = isValidTranslation(pAnimLayer,pNode);
		//KFCurve *lCurve = lCurrentTakeNode->GetTranslationX();
		FbxAnimCurve *lCurve = pNode->LclRotation.GetCurve(pAnimLayer,FBXSDK_CURVENODE_COMPONENT_X);
		if(name && lCurve->KeyGetCount()>1)
		{
			//printf(name);						
			//sprintf("%s",name[numParts]);
			parts[numParts]=name;
			numParts++;			
		}
	
		//DisplayChannels(pNode, lCurrentTakeNode, DisplayCurveKeys, DisplayListCurveKeys, isSwitcher);		
	}

	for(lModelCount = 0; lModelCount < pNode->GetChildCount(); lModelCount++)
    {
        numParts = GetTranslatedParts(pNode->GetChild(lModelCount),pAnimLayer,parts,numParts);
    }	
	return numParts;
}


/*

void GetSkeleton(KFbxNode* pNode,std::map <const char*, DefaultJoint,ltstr> &skeleton)
{
	KFbxNodeAttribute::EAttributeType lAttributeType;
	int i;
	DefaultJoint j;
	KFbxTakeNode* lCurrentTakeNode = pNode->GetCurrentTakeNode();

	if(pNode->GetNodeAttribute() == NULL)
    {
		printf("NULL Node Attribute\n\n");
    }
	else
    {
		lAttributeType = (pNode->GetNodeAttribute()->GetAttributeType());
		if(lAttributeType==KFbxNodeAttribute::eSKELETON)
		{
			//printf("Name: %s\n",pNode->GetName());
			//DisplayDefaultAnimation(pNode);		
			const char *name = isHanim(pNode->GetName());

			KFCurve *lCurve = lCurrentTakeNode->GetTranslationX();
			j.transX = lCurve->GetValue();
			lCurve = lCurrentTakeNode->GetTranslationY();
			j.transY = lCurve->GetValue();
			lCurve = lCurrentTakeNode->GetTranslationZ();
			j.transZ = lCurve->GetValue();

			lCurve = lCurrentTakeNode->GetEulerRotationX();
			j.rotX = lCurve->GetValue();
			lCurve = lCurrentTakeNode->GetEulerRotationY();
			j.rotY = lCurve->GetValue();
			lCurve = lCurrentTakeNode->GetEulerRotationZ();
			j.rotZ = lCurve->GetValue();


			skeleton[name]=j;
		}		
	}

	
	
    for(i = 0; i < pNode->GetChildCount(); i++)
	{
		GetSkeleton(pNode->GetChild(i),skeleton);
	}
	
}


void GetSkeleton(KFbxScene* pScene,std::map <const char*, DefaultJoint,ltstr> &skeleton)
{
	int i;
	KFbxNode* lNode = pScene->GetRootNode();

	if(lNode)
	{
		for(i = 0; i < lNode->GetChildCount(); i++)
		{
			GetSkeleton(lNode->GetChild(i),skeleton);
		}
	}
}

*/
JointRenaming jointRenaming;

int main(int argc, char** argv)
{
	FbxManager* lSdkManager = NULL;
	FbxScene* lScene = NULL;
	bool lResult;
	char *renamingFile = NULL;

	// Prepare the FBX SDK.
	InitializeSdkObjects(lSdkManager, lScene);

	// Load the scene.

	// The example can take a FBX file as an argument.
	if(argc > 1)
	{
		//printf("\n\nFile: %s\n\n", argv[1]);
		lResult = LoadScene(lSdkManager, lScene, argv[1]);
	}
	else
	{
		//printf("\n\nFile: %s\n\n", argv[1]);
		//lResult = LoadScene(lSdkManager, lScene, "c:/local/mocap/t_pose.fbx");
		printf("Usage: SkeletonInterpolatorImport <animation.fbx> [<renaming.txt>]\n");
		return 0;
		/*
		// Destroy all objects created by the FBX SDK.
		DestroySdkObjects(lSdkManager);
		lResult = false;

		printf("\n\nUsage: ImportScene <FBX file name>\n\n");
		*/
	}


	if(lResult == false)
	{
		printf("\n\nAn error occured while loading the scene...");
		return 0;
	}
	else 
	{
		
		if(argc > 2)
		{
			jointRenaming.setupRenaming(argv[2]);
		}

		//std::map <const char*, DefaultJoint,ltstr> skeleton;
		//GetSkeleton(lScene,skeleton);
		FbxArray<FbxString*> lTakeNameArray;				
		lScene->FillAnimStackNameArray(lTakeNameArray);

		for(int i = 0; i < lTakeNameArray.GetCount(); i++)
		{
			//printf("Take name: %s\n",lTakeNameArray.GetAt(i)->Buffer());
			
			// It's useless to display the default animation because it is always empty.
			if(lTakeNameArray.GetAt(i)->Compare(FBXSDK_TAKENODE_DEFAULT_NAME) == 0)
			{
				continue;
			}
			
			lScene->ActiveAnimStackName = lTakeNameArray.GetAt(i)->Buffer();
			/*
			KString lOutputString = "Take Name: ";

			lScene->SetCurrentTake(lTakeNameArray.GetAt(i)->Buffer());
			lOutputString += lScene->GetCurrentTakeName();
			lOutputString += "\n\n";
			printf(lOutputString);
			*/
			//printf("<VOPartsPathInterpolator rotationEncoding=\"quaternions\" ");
			
			//printf("<SkeletonInterpolator rotationEncoding=\"axisangles\"");
			printf("<SkeletonInterpolator rotationEncoding=\"quaternions\"");
			//bool b=true;
			printf(" parts=\"");
			std::string parts[255];
			int numParts = 0;

			FbxAnimStack *pAnimStack = FbxCast<FbxAnimStack>(lScene->GetSrcObject<FbxAnimStack>(0));
			FbxAnimLayer *pAnimLayer = pAnimStack->GetMember<FbxAnimLayer>(0);
			
			//numParts = GetTranslatedParts(lScene->GetRootNode(),skeleton,parts,numParts);
			numParts = GetTranslatedParts(lScene->GetRootNode(),pAnimLayer,parts,numParts);
			int numTranslatedParts = numParts;
			numParts = GetRotatedParts(lScene->GetRootNode(),pAnimLayer,parts,numParts);
			for(int i=0;i<numParts;i++)
			{
				const char *jointName = jointRenaming.getHanimName(parts[i]);
				if(jointName==NULL)
				{
					printf(parts[i].c_str());					
				}
				else
				{
					printf(jointName);
				}
				
				if(i<numParts-1)
				{
					printf(" ");
				}
			}
			printf("\"");
			printf(" encoding=\"");
			if(numTranslatedParts==0)
			{
				printf("R");
			}
			else if(numTranslatedParts==1)
			{	
				printf("T1R");
			}
			else
			{
				printf("TR");
			}
			printf("\">\n");

			//FIXME: exports both animations currently
			return 1;
			
			//GetKeys(lScene->GetRootNode(),skeleton);
			//GetBothKeys(lScene->GetRootNode(),skeleton, numTranslatedParts);
			GetBothKeys(lScene->GetRootNode(),pAnimLayer,numTranslatedParts);

			printf("</SkeletonInterpolator>");
			//DisplayAnimation(lScene->GetRootNode());
		}
		//removed for 2013 conversion
		//DeleteAndClear(lTakeNameArray);
	}
	// Destroy all objects created by the FBX SDK.
	DestroySdkObjects(lSdkManager, true);
	
	
	return 0;
}
