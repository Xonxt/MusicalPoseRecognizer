#include "KinectPose.h"

KinectPose::KinectPose() {
	this->poseIndex = 0;
	featureVector = std::vector<FeatureString>(6);
	poseName = "None";
}

KinectPose::KinectPose(const int index) {
	this->poseIndex = index;
	featureVector = std::vector<FeatureString>(6);
	poseName = "None";
}

KinectPose::KinectPose(const int index, const std::string & fileName) {
	this->poseIndex = index;
	parsePoseDataFile(fileName);
}

KinectPose::KinectPose(const int index, const std::string & poseName, const std::vector<FeatureString> & featureVector) {
	this->poseIndex = index;
	this->poseName = poseName;
	this->featureVector = featureVector;
}

void KinectPose::parsePoseDataFile(const std::string & fileName) {

	this->fileName = fileName;

	this->featureVector.clear();

	std::ifstream ifs(fileName, std::ios::in);

	std::string line;

	// Get the first line: the pose name

	std::getline(ifs, line);

	if (line.find(';') == std::string::npos && line.find(',') == std::string::npos) { 
		setPoseName(line);
		this->poseName;
	}

	// Get the second line: the pose degree of freedom (see KinectPose::getReferenceEstimate() for details)

	std::getline(ifs, line);

	if (line.find(';') == std::string::npos && line.find(',') == std::string::npos) {
		this->referenceEstimate = std::stod(line);
	}

	int i = 0;

	// now get the actual features

	while (std::getline(ifs, line)) {
		FeatureString featureString;
		featureString.clear();

		std::stringstream sstream(line);
		std::string substr;

		// read the next cv::Point2f value

		while (std::getline(sstream, substr, ';')) {
			std::stringstream ss2stream(substr);
			std::string val;

			float a, b;

			// parse the X and Y values (because ... ; cv::Point2f( X, Y ) ; ... ; ... ;)

			std::getline(ss2stream, val, ',');
			a = std::stof(val);

			std::getline(ss2stream, val);
			b = std::stof(val);

			featureString.push_back(cv::Point2f(a, b));
		}
		this->featureVector.push_back(featureString);
	}

	ifs.close();
}

std::vector<FeatureString> KinectPose::getFeatureVector() {
	return this->featureVector;
}

std::string KinectPose::getPoseName() {
	return this->poseName;
}

void KinectPose::setPoseName(const std::string & poseName) {
	this->poseName = poseName;
}

std::vector<double> KinectPose::getReferenceVector() {
	return this->referenceVector;
}

double KinectPose::getReferenceEstimate() {
	if (referenceEstimate <= 0) { 
		double result = 0;

		std::for_each(std::begin(referenceVector), std::end(referenceVector), [&result](double& x) { result += std::pow(x, 2); });

		referenceEstimate = std::sqrt(result);
	}

	return referenceEstimate;
}

std::string KinectPose::getFileName() {
	return this->fileName;
}

void KinectPose::setFileName(const std::string & fileName) {
	this->fileName = fileName;
}

int KinectPose::getPoseIndex() {
	return this->poseIndex;
}

std::vector<double> KinectPose::estimateLikelihood(const FeatureString & featureString) {
	if (this->featureVector.size() == 0)
		return std::vector<double>();

	std::vector<double> results;	

	for (int j = 0; j < featureVector[0].size(); j++) {
		std::vector<double> estimateVector;

		// Calculate difference between the angles
		for (int i = 0; i < 2; i++) {

			double estimateX = 0;
			double estimateY = 0;

			cv::Point2f angles = featureVector[i][j];

			// if it's an elbow angle
			if (i == 0) {
				estimateX = std::abs(angles.x - featureString[i].x);
				estimateY = std::abs(angles.y - featureString[i].y);
			}
			else {	// if it's a direction angle
				estimateX = angleDifference(angles.x, featureString[i].x);
				estimateY = angleDifference(angles.y, featureString[i].y);
			}

			estimateVector.push_back(estimateX);
			estimateVector.push_back(estimateY);
		}

		// for the points (elbows and hands)
		for (int i = 2; i < featureVector.size(); i++) {
			double estimate = 0;
			
			estimate = calcDistance(featureString[i], featureVector[i][j]);			
			
			estimateVector.push_back(estimate);
		}


		// calculate the square root of the sum of every value to the power of 2:
		// Result = sqrt( a^2 + b^2 + c^2 + ... )
		double result = 0;

		for (double x : estimateVector) { 
			result += std::pow(x, 2);
		}
		result = std::sqrt(result);

		results.push_back(result);

	}

	return results;
	
}

void KinectPose::addNewTrainingSample(const FeatureString & featureString) {

	if (this->featureVector.size() == 0) { 
		this->featureVector = std::vector<FeatureString>(featureString.size());
	}

	for (int i = 0; i < featureString.size(); i++) {
		this->featureVector[i].push_back(featureString[i]);
	}

	std::ofstream ofs(this->fileName, std::ios::out);
	{ 
		ofs << poseName << std::endl;
		ofs << getReferenceEstimate() << std::endl;

		for (int i = 0; i < featureVector.size(); i++) {
			for (int j = 0; j < featureVector[i].size(); j++) {
				ofs << featureVector[i][j].x << "," << featureVector[i][j].y << ";";
			}
			ofs << std::endl;
		}
	}
	ofs.close();
}


KinectPose::~KinectPose() {
	poseName.clear();
	featureVector.clear();
}

double KinectPose::angleDifference(const double alpha, const double beta) {
	if (isInRange(alpha, 270, 360) && isInRange(beta, 0, 90)) {
		return (360 - alpha) + beta;
	}
	else if (isInRange(beta, 270, 360) && isInRange(alpha, 0, 90)) {
		return (360 - beta) + alpha;
	}
	else {
		return std::abs(alpha - beta);
	}
}

// check if the a value lies between two constrains
bool KinectPose::isInRange(const double value, const double A, const double B) {
	if (value >= fmin(A, B) && value <= fmax(A, B))
		return true;
	else
		return false;
}