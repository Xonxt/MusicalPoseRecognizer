#pragma once

#include "opencv2/highgui/highgui.hpp"
#include <fstream>
#include <iostream>

#include "Utils.h"

/**
	The vector of features, extracted from the current user in one frame.
	The featues are the following 

	Name:										Type:							  Description
	Hand_Elbow_Angles				cv::Point2f					The angles between the upper arm and lower arm
																							for each hand (the elbow-angles) in Radians. Both
																							hands are recorded in a single cv::Point2f object,
																							with X being the left hand, and Y being the right hand
																							cv::Point2f::x = left_elbow_angle
																							cv::Point2f::y - right_hand_angle
	
	Hand_Direction_Angles		cv::Point2f					The angle between the position of the user's center of mass
																							and the positions of the left and right hands (wrists).
																							The "center of mass" in this case is the head position,
																							but it could also be the center of the torso.
																							The angles are given in the Descartes (Cartesian) coordinate system:
																																		270
																																		 |
																															180----0----360/0
																																		 |
																																		 90
																							Like in the previous case, the angles are stored in one
																							cv::Point2f object, with X and Y being left and right hands
																							respectively.

	Left_Elbow_Position			cv::Point2f					The position of the user's left elbow relative to the center of mass

	Left_Hand_Position			cv::Point2f					The position of the user's left wrist relative to the center of mass

	Right_Elbow_Position		cv::Point2f					The position of the user's right elbow relative to the center of mass

	Right_Hand_Position			cv::Point2f					The position of the user's right wrist relative to the center of mass

	In the latter four cases the positions of the wrists and elbows are calculated relative to the center of mass 
	(int this particular case - position of the head). Because positions are relative, the left hand will be a negative
	value, and the right hand will be in most cases a positive value.
	Note, that the positions are not only relative, but also scale invariant, as the X coordinates are divided by the 
	distance between uses's shoulders and the Y is divided by the height of the uses's torso.

	Thus, the feature vector in this case looks like this:
	FeatureString[0] = cv::Point2f(left_elbow_angle, right_elbow_angle)
	FeatureString[1] = cv::Point2f(left_hand_angle,  right_hand_angle)
	FeatureString[2] = cv::Point2f(left_elbow.x, left_elbow.y)
	FeatureString[3] = cv::Point2f(left_hand.x,  left_hand.y
	FeatureString[4] = cv::Point2f(right_elbow.x, right_elbow.y)
	FeatureString[5] = cv::Point2f(right_hand.x,  right_hand.y
*/
typedef std::vector<cv::Point2f> FeatureString;


/**
	A temporary struct to store the estimation results for every training sample
	@var distance Stores the Euclidian distance between the current feature vector and the training sample
	@var index The number of the pose, for which the extimation is done
*/
struct EstimationResult {

	int index;

	double distance;

	EstimationResult(int indx, double dist) { 
		index = indx;
		distance = dist;
	}
};


/**
	The class, incapsulating the information about a position, that needs to be recognized
*/
class KinectPose {
public:
	
	/**
		Create a new pose object by parsing the specified data file
	*/
	KinectPose(const int index, const std::string& fileName);

	/**
		Create a new pose object with the specified name, using a vector of feature vectors
	*/
	KinectPose(const int index, const std::string& poseName, const std::vector<FeatureString>& featureVector);

	/**
		These two shouldn't be used
	*/
	KinectPose();
	KinectPose(const int index);

	/**
		Parse the provided file and read all the data about the pose (training samples)

		@param fileName The full or relative path to the file containing pose info.
	*/
	void parsePoseDataFile(const std::string& fileName);

	/**
		Get the entire training data of the pose.
	*/
	std::vector<FeatureString> getFeatureVector();

	/**
		Get the name of the current pose
	*/
	std::string getPoseName();

	/**
		Set the new name for the current pose

		@param poseName The name of the position.
	*/
	void setPoseName(const std::string& poseName);

	/**
		Get the reference value for this pose. This is a value that denotes
		the degree of freedom that the recognition algorithm has when estimating
		the position. Higher value means the user can derive more from the training samples
	*/
	std::vector<double> getReferenceVector();
	double getReferenceEstimate();

	/**
		Get the name of the file that contains the tranining info for this pose
	*/
	std::string getFileName();

	/**
		Set a new file name for this pose

		@param fileName The path to the file containing the training info for this pose
	*/
	void setFileName(const std::string& fileName);

	/**
		Get the id of the pose.
	*/
	int getPoseIndex();

	/**
		Calculate the distance vector between the provided test sample and the training samples
		The distance is calculated as the Euclidian distance: d = sqrt( (x1-x2)^2 + (y1-y2)^2 )

		@param featureString Feature vector for the current test sample
		@return Returns a vector of distances, one for every training sample available
	*/	
	std::vector<double> estimateLikelihood(const FeatureString& featureString);

	/**
		This will add the provided feature vector as a new training sample for the current pose

		@param featureString The feature vector from the current instance for the first user
	*/
	void addNewTrainingSample(const FeatureString& featureString);

	~KinectPose();

private:
	// a vector of all traning samples
	std::vector<FeatureString> featureVector;

	// name of the pose
	std::string poseName;

	// name of the file, that contains the training data
	std::string fileName;

	// hte index (ID) of the pose
	int poseIndex;

	// the reference vector, containing the optimal (minimal) distance vector for each feature
	std::vector<double> referenceVector = { 0.3, 0.3, 15, 15, 0.2, 0.5, 0.2, 0.5 };
	double referenceEstimate = 0;

	/**
		Get the difference between two angles in the Descartes (Cartesian) coordinate system (0..360)
		@param alpha First angle
		@param beta Second angle
	*/
	double angleDifference(const double alpha, const double beta);

	/**
		Determines if the provided value lies in the specified range

		@param value The value in question
		@param A The lower limit
		@param B The upper limit
	*/
	bool isInRange(const double value, const double A, const double B);
};

