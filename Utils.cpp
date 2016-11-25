#include "Utils.h"

// check if the a value lies between two constrains
bool isInRange(const double value, const double A, const double B) {
	if (value >= std::fmin(A, B) && value <= std::fmax(A, B))
		return true;
	else
		return false;
}

// correct the angle to a proper Descartes (0..360) orientation
double correctAngle(double alpha) {
	double angle = alpha;

	if (isInRange(alpha, 90, 180)) {				// 1st quadrant
		angle -= 90;
	}
	else if (isInRange(alpha, -90, -180)) {	// 2nd quadrant
		angle = (180 - abs(alpha)) + 90;
	}
	else if (isInRange(alpha, -0, -90)) {		// 3rd quadrant
		angle = 180 + (90 - abs(alpha));
	}
	else if (isInRange(alpha, 0, 90)) {			// 4th quadrant
		angle = 270 + abs(alpha);
		if (angle >= 360)
			angle = 0;
	}

	return (angle);
}

// get the angle of a vector, from pt1 to pt2
double getAngle(const cv::Point2f& pt1, const cv::Point2f& pt2) {
	double alpha; // result

	// calculate vector coordinates
	double
		y = pt2.y - pt1.y,
		x = pt2.x - pt1.x;

	// hypotenuse
	double sqrtResult = 0;
	sqrtResult = std::sqrt(x * x + y * y);
	double angle;
	angle = std::asin(abs(x) / sqrtResult);

	alpha = angle * 180 / PI; // angle from North in degrees

	// correction
	if (x > 0) {
		// I or IV quadrant
		if (y < 0) {
			// IV quadrant
			alpha = 180 - alpha;
		}
	}
	else {
		// II or III quadrant
		if (y > 0) {
			// II quadrant
			alpha = -alpha;
		}
		else
			alpha = alpha - 180;
	}

	return correctAngle(alpha);
}

double angleDifference(const double alpha, const double beta) {
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

double calcDistance(const cv::Point3f& a, const cv::Point3f& b) {
	return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
}

double calcDistance(const cv::Point2f& a, const cv::Point2f& b) {
	return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

double getThreePointAngle(const cv::Point3f& a, const cv::Point3f& b, const cv::Point3f& c) {

	cv::Point3f upperArm = a - b;
	cv::Point3f lowerArm = b - c;

	double angle = (upperArm.ddot(lowerArm)) /
		(std::sqrt(std::pow(upperArm.x, 2) + std::pow(upperArm.y, 2) + std::pow(upperArm.z, 2)) *
		 std::sqrt(std::pow(lowerArm.x, 2) + std::pow(lowerArm.y, 2) + std::pow(lowerArm.z, 2)));

	return std::acos(angle);
}


/**
Decrements the value, but doesn't allow it to go lower than the provided limit

@param value The variable that needs to be decremented
@param lowerLimit The minimal value beyond which the variable will not decrease
*/
void declim(int& value, const int lowerLimit) {
	value = (--value < lowerLimit) ? lowerLimit : value;
}

/**
Increments the value, but doesn't allow it to go higher than the provided limit

@param value The variable that needs to be incremented
@param upperLimit The maximum value beyond which the variable will not increase
*/
void inclim(int& value, const int upperLimit) {
	value = (++value >= upperLimit) ? upperLimit : value;
}


void overlayImage(cv::Mat& src, cv::Mat& overlay, const cv::Point& location) {
	for (int y = std::max(location.y, 0); y < src.rows; ++y) {
		int fY = y - location.y;

		if (fY >= overlay.rows)
			break;

		for (int x = std::max(location.x, 0); x < src.cols; ++x) {
			int fX = x - location.x;

			if (fX >= overlay.cols)
				break;

			double opacity = ((double)overlay.data[fY * overlay.step + fX * overlay.channels() + 3]) / 255;

			for (int c = 0; opacity > 0 && c < src.channels(); ++c) {
				unsigned char overlayPx = overlay.data[fY * overlay.step + fX * overlay.channels() + c];
				unsigned char srcPx = src.data[y * src.step + x * src.channels() + c];
				src.data[y * src.step + src.channels() * x + c] = srcPx * (1. - opacity) + overlayPx * opacity;
			}
		}
	}
}

