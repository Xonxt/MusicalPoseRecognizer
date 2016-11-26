#include "KinectUser.h"

void KinectUser::extractAllJoints() {

	aJoints.clear();
	aJoints.push_back(rSkeleton->getJoint(nite::JOINT_HEAD));
	aJoints.push_back(rSkeleton->getJoint(nite::JOINT_NECK));
	aJoints.push_back(rSkeleton->getJoint(nite::JOINT_LEFT_SHOULDER));
	aJoints.push_back(rSkeleton->getJoint(nite::JOINT_RIGHT_SHOULDER));
	aJoints.push_back(rSkeleton->getJoint(nite::JOINT_LEFT_ELBOW));
	aJoints.push_back(rSkeleton->getJoint(nite::JOINT_RIGHT_ELBOW));
	aJoints.push_back(rSkeleton->getJoint(nite::JOINT_LEFT_HAND));
	aJoints.push_back(rSkeleton->getJoint(nite::JOINT_RIGHT_HAND));
	aJoints.push_back(rSkeleton->getJoint(nite::JOINT_TORSO));
	aJoints.push_back(rSkeleton->getJoint(nite::JOINT_LEFT_HIP));
	aJoints.push_back(rSkeleton->getJoint(nite::JOINT_RIGHT_HIP));
	aJoints.push_back(rSkeleton->getJoint(nite::JOINT_LEFT_KNEE));
	aJoints.push_back(rSkeleton->getJoint(nite::JOINT_RIGHT_KNEE));
	aJoints.push_back(rSkeleton->getJoint(nite::JOINT_LEFT_FOOT));
	aJoints.push_back(rSkeleton->getJoint(nite::JOINT_RIGHT_FOOT));
}

double KinectUser::getJointConfidence() {
	
	double result = 0;

	std::vector<double> confidenceVector;

	if (aJoints.size() > 0) { 
	
		for (int i = 2; i < 8; i++) { 
			confidenceVector.push_back(aJoints[i].getPositionConfidence());
		}

		result = *(std::min_element(confidenceVector.begin(), confidenceVector.end()));

		if (result < 0.5)
			result = 0;

	}

	return result;

}

KinectUser::KinectUser() {
		
}

KinectUser::KinectUser(nite::UserId userId) {
	this->userId = userId;
}

nite::SkeletonState KinectUser::getSkeletonState() {
	return this->g_skeletonState;
}

void KinectUser::setSkeletonState(nite::SkeletonState state) {
	this->g_skeletonState = state;
}

void KinectUser::setUserTracker(nite::UserTracker* userTracker) {
	this->userTracker = userTracker;
}

const nite::Skeleton* KinectUser::getSkeleton() const {
	return this->rSkeleton;
}

const nite::UserData& KinectUser::getUserData() const {
	return this->userData;
}

void KinectUser::setUserData(const nite::UserData& userData) {
	this->userData;
}

void KinectUser::setSkeleton(const nite::Skeleton* rSkeleton) {
	this->rSkeleton = rSkeleton;

	extractAllJoints();
}

void KinectUser::setTrackerFrameRef(nite::UserTrackerFrameRef* userTrackerFrame) {
	this->userTrackerFrame = userTrackerFrame;
}

nite::UserId KinectUser::getUserId() {
	return this->userId;
}

int KinectUser::getPoseIndex() {
	return this->poseIndex;
}

void KinectUser::setPoseIndex(const int index) {
	this->poseIndex = index;
}

std::string KinectUser::getPoseName() {
	return this->poseName;
}

void KinectUser::setPoseName(const std::string & poseName) {
	this->poseName = poseName;
}

void KinectUser::setPose(KinectPose * pose) {
	this->userPose = pose;
	this->poseName = pose->getPoseName();
	this->poseIndex = pose->getPoseIndex();
}

KinectPose * KinectUser::getPose() {
	return this->userPose;
}

FeatureString KinectUser::extractUserFeatures() {

	if (getJointConfidence() == 0)
		return FeatureString();

	FeatureString featureString;

	// extract all joints
	extractAllJoints();

	// distance between shoulders
	double shoulderDist = calcDistance(extractJoint3D(nite::JOINT_LEFT_SHOULDER), extractJoint3D(nite::JOINT_RIGHT_SHOULDER));

	// height from hip to shoulder
	double heightDist = calcDistance(extractJoint3D(nite::JOINT_LEFT_HIP), extractJoint3D(nite::JOINT_LEFT_SHOULDER));

	// extract point at the torso center
	cv::Point2f bodyCenter; 
	{
		cv::Point3f pointTorso	= extractJoint3D(nite::JOINT_TORSO);
		cv::Point2f pointNeck		= extractJoint2D(nite::JOINT_NECK);

		bodyCenter = cv::Point2f((pointNeck.x + pointTorso.x) / 2, (pointNeck.y + pointTorso.y) / 2);
	}

	// extract elbow angles for each hand:
	cv::Point2f handAngles(
		getThreePointAngle(extractJoint3D(nite::JOINT_LEFT_SHOULDER), extractJoint3D(nite::JOINT_LEFT_ELBOW), extractJoint3D(nite::JOINT_LEFT_HAND)),
		getThreePointAngle(extractJoint3D(nite::JOINT_RIGHT_SHOULDER), extractJoint3D(nite::JOINT_RIGHT_ELBOW), extractJoint3D(nite::JOINT_RIGHT_HAND))
	);
	featureString.push_back(handAngles);

	// extract direction angles to the hands
	
	// Previous version: using the center of the body (between neck and solar plexus) as the reference point
	//double angleToLeftHand	= getAngle(cv::Point2f(bodyCenter.x, bodyCenter.y), cv::Point2f(extractJoint3D(nite::JOINT_LEFT_HAND).x, extractJoint3D(nite::JOINT_LEFT_HAND).y));
	//double angleToRightHand = getAngle(cv::Point2f(bodyCenter.x, bodyCenter.y), cv::Point2f(extractJoint3D(nite::JOINT_RIGHT_HAND).x, extractJoint3D(nite::JOINT_RIGHT_HAND).y));

	// new version: using the head coordinates as the reference point
	double angleToLeftHand = getAngle(cv::Point2f(extractJoint3D(nite::JOINT_HEAD).x, extractJoint3D(nite::JOINT_HEAD).y),
		cv::Point2f(extractJoint3D(nite::JOINT_LEFT_HAND).x, extractJoint3D(nite::JOINT_LEFT_HAND).y));
	
	double angleToRightHand = getAngle(cv::Point2f(extractJoint3D(nite::JOINT_TORSO).x, extractJoint3D(nite::JOINT_TORSO).y),
		cv::Point2f(extractJoint3D(nite::JOINT_RIGHT_HAND).x, extractJoint3D(nite::JOINT_RIGHT_HAND).y));
	
	featureString.push_back(cv::Point2f(angleToLeftHand, angleToRightHand));

	// extract other points:
	std::vector<cv::Point3f> salientPoints;
	salientPoints.push_back(extractJoint3D(nite::JOINT_LEFT_ELBOW));
	salientPoints.push_back(extractJoint3D(nite::JOINT_LEFT_HAND));
	salientPoints.push_back(extractJoint3D(nite::JOINT_RIGHT_ELBOW));
	salientPoints.push_back(extractJoint3D(nite::JOINT_RIGHT_HAND));

	// make points position and scale invariant
	for (int i = 0; i < salientPoints.size(); i++) {		
		salientPoints[i] -= extractJoint3D(nite::JOINT_HEAD);
		//salientPoints[i] /= shoulderDist;

		// divide by the shoulder length and the torso height, to make it invariant to user's height and size
		salientPoints[i].x /= shoulderDist;
		salientPoints[i].y /= heightDist;

		featureString.push_back(cv::Point2f(salientPoints[i].x, salientPoints[i].y));
	}

	return featureString;
}

void KinectUser::drawUserSkeleton(cv::Mat& image) {

	if (rSkeleton->getState() == nite::SKELETON_TRACKED) {

		if (aJoints.size() == 0)
			extractAllJoints();

		if (getJointConfidence() == 0)
			return;

		std::vector<cv::Point2f> aPoint;
		for (int s = 0; s < 8; ++s) {		// we only need 8 points (upper body)
			aPoint.push_back(extractJoint2D((nite::JointType) s));
		}

		// draw skeleton
		if (!image.empty()) {
			cv::line(image, aPoint[0], aPoint[1], cv::Scalar(255, 0, 0), 2);
			cv::line(image, aPoint[1], aPoint[2], cv::Scalar(255, 0, 0), 2);
			cv::line(image, aPoint[1], aPoint[3], cv::Scalar(255, 0, 0), 2);
			cv::line(image, aPoint[2], aPoint[4], cv::Scalar(255, 0, 0), 2);
			cv::line(image, aPoint[3], aPoint[5], cv::Scalar(255, 0, 0), 2);
			cv::line(image, aPoint[4], aPoint[6], cv::Scalar(255, 0, 0), 2);
			cv::line(image, aPoint[5], aPoint[7], cv::Scalar(255, 0, 0), 2);
		}

		// draw joints
		for (int s = 0; s < aPoint.size(); ++s) {
			if (aJoints[s].getPositionConfidence() > 0.5)
				cv::circle(image, aPoint[s], 3, cv::Scalar(0, 0, 255), 2);
			else
				cv::circle(image, aPoint[s], 3, cv::Scalar(0, 255, 0), 2);
		}
	}
}

cv::Point2f KinectUser::extractJoint2D(const nite::JointType type) {

	const nite::Point3f jointRealWorldNite = this->aJoints[(int)type].getPosition();

	cv::Point2f jointCameraCV;

	this->userTracker->convertJointCoordinatesToDepth(jointRealWorldNite.x, jointRealWorldNite.y, jointRealWorldNite.z, 
		&(jointCameraCV.x), &(jointCameraCV.y));

	return jointCameraCV;
}

cv::Point3f KinectUser::extractJoint3D(const nite::JointType type) {
	
	const nite::Point3f jointRealWorldNite = this->aJoints[(int)type].getPosition();

	return cv::Point3f(jointRealWorldNite.x, jointRealWorldNite.y, jointRealWorldNite.z);

}

std::vector<int> KinectUser::getPoseTumbler() {
	return poseTumbler;
}

void KinectUser::addPoseTumbler(const int tumbler, const int max) {

	poseTumbler.push_back(tumbler);

	if (poseTumbler.size() > max)
		poseTumbler.erase(std::begin(poseTumbler));

}

float KinectUser::getPoseTumblerAvg() {
	float avg = 0;
	std::for_each(std::begin(poseTumbler), std::end(poseTumbler), [&avg](int& x) { avg += x; });
	avg /= poseTumbler.size();
	return avg;
}

KinectUser::~KinectUser() {
}
