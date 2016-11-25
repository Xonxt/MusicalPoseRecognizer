#pragma once

#include "opencv2/imgproc/imgproc.hpp"

#include "KinectPose.h"

#include "Utils.h"

#include <OpenNI.h>
#include <NiTE.h>

#include <iostream>
#include <iterator>

/**
	The class that represents a person standing in front of the Kinect devices
	and attempting to do the musical positions.
*/
class KinectUser {

public:
	KinectUser();
	KinectUser(nite::UserId userId);

	/**
		Get the current state of the user's skeleton (tracked, calibrating, new, etc.)
	*/
	nite::SkeletonState getSkeletonState();
	/**
		Update the state of the user's skeleton
	*/
	void setSkeletonState(nite::SkeletonState state);

	/**
		Assign a new user tracker for the user
		@param userTracker Reference to the user tracker object
	*/
	void setUserTracker(nite::UserTracker* userTracker);

	/**
		Get a reference to the user's skeleton
	*/
	const nite::Skeleton* getSkeleton() const;

	/**
		Get the reference to the user data
	*/
	const nite::UserData& getUserData() const;

	/**
		Assign new user data to the user
		@param userData Reference to the user data object
	*/
	void setUserData(const nite::UserData& userData);

	/**
		Assign a new skeleton to the user
		@param rSkeleton Pointer to the skeleton object
	*/
	void setSkeleton(const nite::Skeleton* rSkeleton);

	/**
		Set a reference to the current frame, as viewed by the Kinect using the PrimeSensor SDK

		@param userTrackerFrame Pointer to the frame reference
	*/
	void setTrackerFrameRef(nite::UserTrackerFrameRef* userTrackerFrame);

	/**
		Get the current user's id
	*/
	nite::UserId getUserId();

	/**
		Get the id of the current user's pose.
		@return Returns the id (index) of the current recognized pose or -1 if no pose is detected
	*/
	int getPoseIndex();

	/**
		Set the new pose index
	*/
	void setPoseIndex(const int index);

	/**
		Get the name of the current pose (instrument)
	*/
	std::string getPoseName();

	/**
		Change the name of the current pose, assumed by the user
	*/
	void setPoseName(const std::string& poseName);

	/**
		Set the new pose

		@param pose Pointer to the pose
	*/
	void setPose(KinectPose* pose);

	/**
		Get the pointer to the current pose of the user
	*/
	KinectPose* getPose();

	/**
		Extract the user's features from the current frame.
		Refer to the KinectPose.h for details.
	*/
	FeatureString extractUserFeatures();

	/**
		Draw the user's skeleton On the image (only the upper body)

		@param image Reference to the cv::Mat image, on which we need to draw the skeleton
	*/
	void drawUserSkeleton(cv::Mat& image);

	/**
		Extract the 2D coordinates of the specified joint. The coordinates are given in the simples
		Cartesian (Descartes) coordinated system, where the upper left corner is (0;0).

		@param type The name of the joint. See nite::JOINT_XXXXX
		@return Returns a cv::Point2f object with the point coordinates
	*/
	cv::Point2f extractJoint2D(const nite::JointType type);

	/**
		Get the 2D coordinated of the specified joint. The coordinates are given as the "world coordinates",
		relative to the Kinect's actual camera (which is 0;0;0). The coordinates are given in milimeters.

		@param type The name of the joint. See nite::JOINT_XXXXX
		@return Returns a cv::Point3f object with the point coordinates
	*/
	cv::Point3f extractJoint3D(const nite::JointType type);

	/**
		Get the tumbler array for the current pose recognition attempt.
		See the "poseTumbler" attribute for details.
	*/
	std::vector<int> getPoseTumbler();

	/**
		Add a new number to the tumbler.
		@param tumbler ID of the pose
		@param max Maximum number of tumblers (i.e. for how many consecutive frames does
		the user need to hold the pose for it ti be recognized). Default: 3.
	*/
	void addPoseTumbler(const int tumbler, const int max = 3);

	/**
		Get the avera value from the tumbler
	*/
	float getPoseTumblerAvg();

	~KinectUser();

private:
	// Pointer to the main UserTracker object
	nite::UserTracker* userTracker;

	// The current user data
	nite::UserData userData;

	// The pointer to the current pose of the user
	KinectPose* userPose;

	// The id of the user
	nite::UserId userId;

	// The current skeleton state
	nite::SkeletonState g_skeletonState = nite::SKELETON_NONE;

	// The reference to the current frame extracted by NiTE from the Kinect
	nite::UserTrackerFrameRef* userTrackerFrame;

	// Is the user visible?
	bool g_visibleUser = false;

	// The current pose name
	std::string poseName = "";

	// current pose id
	int poseIndex = -1;

	// Pointer to the current skeleton object
	const nite::Skeleton* rSkeleton;

	// vector of salien points (most important joint coordinates)
	std::vector<cv::Point3f> salientPoints;

	// distance between user's shoulders
	double shoulderDist;

	// the center of the body. (Deprecated).
	cv::Point2f bodyCenter;

	// Full lisr of the extracted joints
	std::vector<nite::SkeletonJoint> aJoints;

	/**
		Extracts all the skeleton joints for the current user in the current frame
	*/
	void extractAllJoints();

	/**
		The pose tumbler. This is basiaclly a queue of pose id's, which is kept below certain length.
		This is used to check, for how many concesutive frames does the user hold a certain pose, before
		concluding that this is indeed a newly recognized pose.

		For example, let's saya user is first standing still (-1). And then at frame 6 he assumes 
		a "Violin playing position" (2). The the tumbler will look like this:

		Frame:		Tumbler:
		0											<---- Nothing
		1					-1							
		2					-1 -1
		3					-1 -1 -1		<---- Still nothing
		4					-1 -1 -1
		5					-1 -1 -1
		6					 2 -1 -1		<---- User assumes position...
		7					 2  2 -1
		8					 2  2  2    <---- "Violin" pose recognized!!
	*/
	std::vector<int> poseTumbler;

	/**
		Returns, how confident is the NiTE sensor about the position of all the skeleton joints.
	*/
	double getJointConfidence();
};

