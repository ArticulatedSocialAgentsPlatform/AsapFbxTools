#ifndef JOINT_RENAMING_H
#define JOINT_RENAMING_H
#include <string>
#include <map>
/**
 * Handles the mapping between h-anim joint names and joint names as used in a graphical model
 */
class JointRenaming
{
	public:
		/**
		 * Reads the joint mapping from a text file.
		 * This file must be set up as:
		 * jointname1 hanimname1
		 * jointname2 hanimname2
		 * etc
		 */
		void setupRenaming(const char* filename);
		
		/**
		 * Gets the 'real' joint name of a certain H-anim joint, NULL if not found.
		 */
		const char * getJointName(std::string hanimName);

		/**
		 * Gets the H-anim joint name of a joint, NULL if not found
		 */
		const char * getHanimName(std::string jointName);
		
	private:
		std::map<std::string,std::string> mRenameMap;
		std::map<std::string,std::string> mInverseRenameMap;
};
#endif