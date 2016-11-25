#pragma once
#include "KinectUser.h"
#include <iterator>

#include <thread>

#define KEY_PGUP 2228224	// the key code for the PageUp button
#define KEY_PGDN 2162688	// the key code for the PageDown button
#define KEY_TRAIN 'b'			// the key that is pressed for adding a training sample

#define FRAME_SKIP 4			// how many frames to skip

#define MAX_USERS 10			// maximum number of users in the frame at the same time

#define HOLD_POSE	3				// how long the pose needs to be held before it's recognized. Should be odd number. Default: 3

#define USER_MESSAGE(msg) \
	{printf("[%08llu] User #%d:\t%s\n",ts, user.getId(),msg);}

/**
	The main class, that initializes the OpenCV, OpenNI and NiTE, and performs the
	pose recognition process.
*/
class PoseRecognizer {

public:

	/**
		The constructor. Doesn't really do anything productive.
	*/
	PoseRecognizer();

	/**
		Initialize the musical pose recognition system.
		Does things like: read pose data from file, initialize OpenCV,
		initialize NiTE and OpenNI.

		@return Returns "true" if initialization was successful.
	*/
	bool initialize();

	/**
		Reload the pose information from the files in a different folder.
		See the description of the initPoseData() method for details on how 
		the files should be formatted.

		@param folder The name of the folder that contains the pose data.

		@return Returns "true" if the reinitialization of the pose data was successful.
	*/
	bool reloadPoseData(const std::string folder);

	/**
		Toggles displaying debug information on/off. The Debug information includes
		showing the updated status of newly detected and/or tracked users.

		@param flag The state of the flag. Set "true" to show the info.
	*/
	void displayDebugInformation(bool flag);

	/**
		Starts the entire pose recognition process. The process will run in a separate thread.
		Every 4 frames the pose estimation will be performed. At the end of each cycle
		the image with overlayed skeletons will be shown along with the name of every detected
		pose near the head of each user.
	*/
	void start();

	/**
		Set the number of nearest neighbours for the algorithm. 

		@param nNeighbours Number of nearest neighbours. Default: 3.
	*/
	void setNearestNeighbours(const int nNeighbours = 3);

	/**
		Grabs the next frame from the Kinect device, locates every visible user in the image,
		performs the pose estimation and returns the list of each user, for whom a pose was 
		recognized.
		This method is automatically continuously run in the start() method, with the exception
		that the start() method will skip every 4 frames and display the modified image at the end
		of each cycle.

		@return Returns a vector of pointers to users, for whom in the current frame a pose was 
		recognized.
	*/
	std::vector<KinectUser*> processNextFrame();

	/**
		Gets the original unmodified frame from the Kinect's RGB camera.

		@return An OpenCV Mat image, in the RGB format.
	*/
	cv::Mat getOriginalFrame();

	/**
		Gets the modified frame from the Kinect's camera. The image has the skeletons
		of every user displayed (only the upper body), as well as the name of every 
		recognized pose near the user's head.

		@return An OpenCV Mat image, in the RGB format
	*/
	cv::Mat getModifiedFrame();

	~PoseRecognizer();


private:
	// The video capture object for the OpenCV
	cv::VideoCapture cap;

	// RGB and Depth images from Kinect
	cv::Mat depthMap;
	cv::Mat bgrImage;

	// NiTE user tracker, that tracks users on the frame
	nite::UserTracker userTracker;

	// status for the NiTE functions
	nite::Status niteRc;

	// the reference to a frame from the UserTracker
	nite::UserTrackerFrameRef userTrackerFrame;

	// the visibility status for the users in the frame
	bool g_visibleUsers[MAX_USERS] = { false };

	// the skeleton states for the users
	nite::SkeletonState g_skeletonStates[MAX_USERS] = { nite::SKELETON_NONE };

	// user list
	std::vector<KinectUser> userList;

	// pose data from the files
	std::vector<KinectPose> poseVector;

	// the number of the current frame. Used for the frame skipping
	int frameSkip = 0;

	// the flag that says whether we need to skip every few frames or not
	bool skipFrames = false;

	// current pose number, for which we can add new taining samples.
	int currentPoseNumber = 0;

	// diplsy debug info about the kikect users or not?
	bool displayDebug = false;

	// number of nearest neighbours
	int nearestNeighbours = 7;

	/**
		Read the position data from the files in the provided folder. Every files has to have
		the .txt file extension and needs to have the following contents:

		Pose_name
		Degree_of_freedom
		Left_elbow_angle,Right_elbow_angle;
		Left_hand_direction,Right_hand_direction;
		Left_elbow_position;
		Left_wrist_position;
		Right_elbow_position;
		Right_wrist_position;

		here, "Pose_name" is just the name of the pose, like "Flute", "Violin" or "Conducting"
		"Degree_of_freedom" is the value that determines how much freedom is given to the recognition
		algorithm when trying to recognize this particular pose. The bigger the value, the more precise
		the user needs to be when emulating the pose. Values from 15 to 20 are optimal.

		You can easily add new positions to the system, by adding a new file with only 
		the first two lines present. To record completely new data for existing pose, remove
		everything except the first two lines.

		@param path The name of the folder that contains the pose data.

		@return Returns the vector of new poses, that were parsed from the files in the folder.
	*/
	std::vector<KinectPose> initPoseData(const std::string path);

	/**
		Gets the list of files in the folder. Uses Windows API, so it needs to be reworked
		for other operating systems

		@param folder The folder that contains files

		@return A string vector with the names of files in the folder.
	*/
	std::vector<std::string> get_all_files_names_within_folder(std::string folder);

	/**
		Function used for debugging purposes. Displays the information about newly detected
		and/or tracked users in the console

		@param user The reference to the current user
		@param ts The current timestamp
	*/
	void updateUserState(const nite::UserData& user, unsigned long long ts);
	
	/**
		Detect all the new and old users in the NiTE-frame and add/update/remove them from the
		user list

		@param users A NiTE Array of NiTE users
	*/
	void fillUserList(const nite::Array<nite::UserData>& users);

	/**
		Estimate the pose of the current user. The method calculates the 'distance' from the
		current user's feature vector to every training sample of evey pose, then uses the 
		K Nearest Neighbours algorithm to recognize the position. 

		If a position was recognized it will automatically update the info about the current pose
		in the user list.

		@param user The reference to the current user, for whom the recognition is performed
		@param featureString A vector of features, extracted for the curren user
		@param nearestNeighbours The parameter for the Nearest Neighbours algorithm. Default: 5.

		@return The number of the recognized pose in the pose vector or -1 if no pose was recognized.
	*/
	int estimatePose(KinectUser& user, const FeatureString& featureString, const int nearestNeighbours = 5);

	/**
		Draws the image of the instrument on the user, instead of simple text
	*/
	void drawInstrument(KinectUser& user, cv::Mat& image);
	
	// Toggle if the feature vector from the current iteration should be added as a training sample
	bool rememberPose = false;
};

