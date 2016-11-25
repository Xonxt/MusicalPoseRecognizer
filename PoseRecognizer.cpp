#include "PoseRecognizer.h"

std::vector<KinectPose> PoseRecognizer::initPoseData(const std::string path) {
	std::vector<std::string> fileList = get_all_files_names_within_folder(path);
	std::vector<KinectPose> poseVector;
	int i = 0;
	for (std::string fileName : fileList) {
		poseVector.push_back(KinectPose(i++, fileName));
	}
	return poseVector;
}

std::vector<std::string> PoseRecognizer::get_all_files_names_within_folder(std::string folder) {
	std::vector<std::string> names;
	std::string s = folder + "/*.txt";
	std::wstring search_path = std::wstring(s.begin(), s.end());

	WIN32_FIND_DATA fd;
	HANDLE hFind = ::FindFirstFile(search_path.c_str(), &fd);
	if (hFind != INVALID_HANDLE_VALUE) {
		do {
			if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {

				std::string fullPath = folder + "/";

				std::wstring fname(std::wstring(fullPath.begin(), fullPath.end()) + fd.cFileName);

				fullPath = std::string(fname.begin(), fname.end());

				names.push_back(fullPath);
			}
		} while (::FindNextFile(hFind, &fd));
		::FindClose(hFind);
	}
	return names;
}

void PoseRecognizer::updateUserState(const nite::UserData & user, unsigned long long ts) {
	if (user.isNew())
		USER_MESSAGE("New")
	else if (user.isVisible() && !g_visibleUsers[user.getId()])
		USER_MESSAGE("Visible")
	else if (!user.isVisible() && g_visibleUsers[user.getId()])
		USER_MESSAGE("Out of Scene")
	else if (user.isLost())
		USER_MESSAGE("Lost")

		g_visibleUsers[user.getId()] = user.isVisible();

	if (g_skeletonStates[user.getId()] != user.getSkeleton().getState()) {
		switch (g_skeletonStates[user.getId()] = user.getSkeleton().getState()) {
		case nite::SKELETON_NONE:
			USER_MESSAGE("Stopped tracking.")
				break;
		case nite::SKELETON_CALIBRATING:
			USER_MESSAGE("Calibrating...")
				break;
		case nite::SKELETON_TRACKED:
			USER_MESSAGE("Tracking!")
				break;
		case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
		case nite::SKELETON_CALIBRATION_ERROR_HANDS:
		case nite::SKELETON_CALIBRATION_ERROR_LEGS:
		case nite::SKELETON_CALIBRATION_ERROR_HEAD:
		case nite::SKELETON_CALIBRATION_ERROR_TORSO:
			USER_MESSAGE("Calibration Failed... :-|")
				break;
		}
	}
}

void PoseRecognizer::fillUserList(const nite::Array<nite::UserData>& users) {

	for (int u = 0; u < users.getSize(); ++u) {
		const nite::UserData& user = users[u];
		const nite::Skeleton& rSkeleton = user.getSkeleton();

		if (displayDebug)
			updateUserState(user, userTrackerFrame.getTimestamp());

		if (user.isNew()) {			// if this is a new user, add him to the list
			userTracker.startSkeletonTracking(user.getId());

			KinectUser kinectUser(user.getId());
			kinectUser.setSkeleton(&rSkeleton);
			kinectUser.setUserTracker(&userTracker);
			kinectUser.setUserData(user);
			kinectUser.setTrackerFrameRef(&userTrackerFrame);

			userList.push_back(kinectUser);
		}
		else {
			// if it's an already existing user, find him in the list
			auto existingUser = std::find_if(userList.begin(), userList.end(), [&user](KinectUser& u) { return u.getUserId() == user.getId(); });

			// it is really in the list
			if (existingUser != userList.end()) {
				if (user.isLost()) {		// if he's lost (no longer visible), remove him
					userList.erase(existingUser);
				}
				else {		// update the user's skeleton data
					(*existingUser).setUserData(user);
					(*existingUser).setSkeleton(&rSkeleton);
				}
			}
		}
	}
}

int PoseRecognizer::estimatePose(KinectUser & user, const FeatureString & featureString, const int nearestNeighbours) {

	int poseIndex = -1;

	double distanceToUser = user.extractJoint3D(nite::JOINT_TORSO).z / 1000;	// distance to user in meters

	double distanceMultiplier = 1.0;

	if (distanceToUser > 2.0) { 
		distanceMultiplier += (distanceToUser - 2.0) / 2;
	}

	std::vector<EstimationResult> estimationResults;
	
	// calculate the difference between extracted features and training samples
	for (KinectPose pose : poseVector) {
		std::vector<double> likelihoods = pose.estimateLikelihood(featureString);

		for (double x : likelihoods) {
			if (x <= (pose.getReferenceEstimate() * distanceMultiplier) )
				estimationResults.push_back(EstimationResult(pose.getPoseIndex(), x));
		}
	}

	// sort the distance vector
	std::sort(std::begin(estimationResults), std::end(estimationResults), [](EstimationResult a, EstimationResult b) { 
		return a.distance < b.distance; 
	});

	std::vector<int> neigbours(poseVector.size());

	// get the first N estimations and fill a "histogram" with them
	if (estimationResults.size() >= nearestNeighbours) {
		for (int i = 0; i < nearestNeighbours; i++) {
			neigbours[estimationResults[i].index] += 1;
		}		
	}

	// find the "peak" in the "histogram" - this will be the most likely pose
	int minResultIndex = std::distance(std::begin(neigbours), std::max_element(std::begin(neigbours), std::end(neigbours)));

	// the index of the most likely pose
	int result = neigbours[minResultIndex];

	// check for how long the pose is held before recognizing it
	if (result > (nearestNeighbours-1) && estimationResults.size() >= nearestNeighbours)
		user.addPoseTumbler(minResultIndex);
	else
		user.addPoseTumbler(-1);

	// now check if this pose is held for at least 3 consecuitive instances
	if (user.getPoseTumbler().size() == (HOLD_POSE % 2) ? HOLD_POSE : HOLD_POSE + 1) {

		int mid = (0 + user.getPoseTumbler().size()) / 2;

		if (std::abs(user.getPoseTumblerAvg() - user.getPoseTumbler()[mid]) < 0.001) {

			poseIndex = user.getPoseTumbler()[mid];

			user.setPoseName( (poseIndex >= 0) ? poseVector[poseIndex].getPoseName() : "" );
			user.setPoseIndex( poseIndex );
		}
	}

	return poseIndex;
}

void PoseRecognizer::drawInstrument(KinectUser & user, cv::Mat& image) {

	if (user.getPoseIndex() < 0 || user.getPoseName() == "")
		return;

	std::string fileName = "./instruments/" + user.getPoseName() + ".png";

	cv::Mat instrument = cv::imread(fileName, cv::IMREAD_UNCHANGED);

	cv::Point2f pt1 = user.extractJoint2D(nite::JOINT_LEFT_HAND);
	cv::Point2f pt2 = user.extractJoint2D(nite::JOINT_RIGHT_HAND);

	double handDist = calcDistance(pt1, pt2);
	
	cv::Point location = pt1;

	// make some adjustments so the pictures look nice-ish.
	// flute
	if (user.getPoseIndex() == 0) { 
		location.x -= instrument.cols / 4;
		location.y -= instrument.rows / 2;
	}

	// clarinet
	if (user.getPoseIndex() == 1) { 
		location.y -= instrument.rows / 4;
	}

	// violin
	if (user.getPoseIndex() == 2) {
		//location.y -= instrument.rows / 3;
	}

	// cello
	if (user.getPoseIndex() == 3) {
		location.x -= instrument.cols / 2;
		location.y -= instrument.rows / 5;
	}

	// trombone
	if (user.getPoseIndex() == 4) {
		location.y -= instrument.rows / 2;
	}

	// conductor
	if (user.getPoseIndex() == 6) {
		location.y -= instrument.rows;
		location.x -= instrument.cols;
	}

	// drums
	if (user.getPoseIndex() == 5) {
		location.y -= instrument.rows / 3;
		location.x -= (instrument.cols - handDist) / 2;
	}

	overlayImage(image, instrument, location);
}

PoseRecognizer::PoseRecognizer() {
}

bool PoseRecognizer::initialize() {

	// close the previous open cv capture, if necessary
	if (cap.isOpened())
		cap.release();

	// open new video capture from Kinect
	cap = cv::VideoCapture(CV_CAP_OPENNI2);

	if (!cap.isOpened()) {
		std::cerr << "Error while opening the capture device!" << std::endl;
		return false;
	}

	// Because there's about 6-7 cm between the Depth-camera and the RGB-camera,
	// we should shift them so they show roughly the same viewpoint
	cap.set(CV_CAP_PROP_OPENNI_REGISTRATION, 1);

	// init NiTE
	niteRc = nite::NiTE::initialize();

	if (niteRc != nite::STATUS_OK) {
		std::cerr << "Couldn't initialize" << std::endl;
		return false;
	}
	
	// create a nite::UserTracker
	niteRc = userTracker.create();

	if (niteRc != nite::STATUS_OK) {
		std::cerr << "Couldn't create user tracker" << std::endl;
		return false;
	}

	// read the pose data from the files
	poseVector = initPoseData("./poses");
	if (poseVector.size() == 0) { 
		std::cerr << "Failed to load the pose data!" << std::endl;
		return false;
	}
}

bool PoseRecognizer::reloadPoseData(const std::string folder) {

	poseVector = this->initPoseData(folder);

	return (poseVector.size() > 0);
}

void PoseRecognizer::displayDebugInformation(bool flag) {
	this->displayDebug = flag;
}

void PoseRecognizer::start() {

	skipFrames = true;

	std::thread mainThread([&] {

		while (true) {
			
			int ch = cv::waitKey(5);
			if (ch == 27)
				break;
			else if (ch == KEY_PGDN) {
				declim(currentPoseNumber, 0);
				std::cout << "Learning pose: " << poseVector[currentPoseNumber].getPoseName() << std::endl;
			}
			else if (ch == KEY_PGUP) {
				inclim(currentPoseNumber, poseVector.size() - 1);
				std::cout << "Learning pose: " << poseVector[currentPoseNumber].getPoseName() << std::endl;
			}
			else if (ch == KEY_TRAIN) {
				rememberPose = true;
				frameSkip = -1;
			}

			processNextFrame();

			cv::imshow("video", getModifiedFrame());
		}
	});

	mainThread.join();

	skipFrames = false;

}

void PoseRecognizer::setNearestNeighbours(const int nNeighbours) {
	this->nearestNeighbours = nNeighbours;
}

// public
std::vector<KinectUser*> PoseRecognizer::processNextFrame() {

	// how many frames to skip every time? default: 4
	if (skipFrames)
		frameSkip = ++frameSkip % FRAME_SKIP;


	// frab the frames from Kinect
	cap.grab();
	cap.retrieve(depthMap, CV_16UC1);
	cap.retrieve(bgrImage, CV_32FC1);
	
	// grab the frame to the NiTE
	niteRc = userTracker.readFrame(&userTrackerFrame);	

	if (niteRc != nite::STATUS_OK) {
		std::cerr << "Get next frame failed" << std::endl;
		return std::vector<KinectUser*>();
	}

	// get te list of users from the current frame
	const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();

	// fill the list of users
	fillUserList(users);

	std::vector<KinectUser*> recognitionResult;

	// for every user: extract features and estimate pose:
	if (frameSkip == 0) {		

		for (KinectUser& user : userList) {

			FeatureString featureString = user.extractUserFeatures();
			
			if (featureString.size() > 0 && !rememberPose) {
				
				if (estimatePose(user, featureString, nearestNeighbours) >= 0) {
					recognitionResult.push_back(&user);
				}

			}

			// add the feature string as a new training sample, if the key 'b' was pressed
			if (rememberPose) { 		
			
				if (featureString.size() > 0) {
					poseVector[currentPoseNumber].addNewTrainingSample(featureString);
					std::cout << "New training sample added for " << poseVector[currentPoseNumber].getPoseName() << "!" << std::endl;
				}

				rememberPose = false;
			
			}
		}
	}
	return recognitionResult;
}

cv::Mat PoseRecognizer::getOriginalFrame() {
	return this->bgrImage;
}

cv::Mat PoseRecognizer::getModifiedFrame() {

	if (this->bgrImage.empty())
		return cv::Mat(480, 640, CV_8U);
	
	cv::Mat image = this->bgrImage.clone();
	
	// draw skeletons;	
	for (KinectUser usr : userList) {
		usr.drawUserSkeleton(image);
	}		

	// add some effects for interactivity
	for (KinectUser usr : userList) { 
		// either just write the name of the pose near the user's head:
		// cv::putText(image, usr.getPoseName(), usr.extractJoint2D(nite::JOINT_HEAD) + cv::Point2f(20, 10), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 3);

		// or just overlay the pictures of instruments near the user's hands
		drawInstrument(usr, image);
	}
	
	return image;
}


PoseRecognizer::~PoseRecognizer() {

	nite::NiTE::shutdown();

	cap.release();

}
