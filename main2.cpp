#include "stdafx.h"

#include <Windows.h>

#include "PoseRecognizer.h"

int main() {

	// Create a new recognizer object
	PoseRecognizer pr = PoseRecognizer();

	if (!pr.initialize()) { 
		std::cerr << "init error!" << std::endl;
	}
	
	// INSTRUCTIONS:

	// Option one
	// Either just start the app by calling .start()
	// It will automatically run in a new thread
	 pr.start();

	// Option two
	// Or you can do this manually and process each frame individually
	/*
	while (cv::waitKey(5) != 27) {	// while ESC key is not pressed
	
		// this returns a vector if every user, for which
		// a musical pose was detected
		std::vector<KinectUser*> users =  pr.processNextFrame();

		// you can display the modified image with the name of the
		// musical instrument overlayed, along with the skeleton
		// cv::imshow("video", pr.getModifiedFrame());

		// or you can get the original image and and do whatever you like
		// with it. Maybe overlay the text and "graphix" yourself.
		cv::Mat frame = pr.getOriginalFrame();
		for (KinectUser* usr : users) { 
			cv::putText(frame, usr->getPoseName(), usr->extractJoint2D(nite::JOINT_HEAD), 
								  CV_FONT_HERSHEY_PLAIN, 2, CV_RGB(255, 0, 0), 3);
		}
		cv::imshow("video", frame);
	}
	*/

	return 0;
}