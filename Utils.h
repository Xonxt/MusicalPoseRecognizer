#pragma once

#include <iostream>

#include "opencv2/highgui/highgui.hpp"

#define PI 3.1415926538

/** 
	Check if the a value lies in the specified range

	@param value The value in question
	@param A The lower limit
	@param B The upper limit
	@return Returns "true" if the value lies in the range A <= V <= B
*/
bool isInRange(const double value, const double A, const double B);

/** 
	Correct the angle to a proper Cartesian (0..360) orientation

	@param alpha The input angle
	@return The angle in the 0..360 range
*/
double correctAngle(double alpha);

/** 
	Get the angle of a vector between two specified points
	
	@param pt1 The starting point of the vector
	@param pt2 The ending poing of the vector

	@return The value of the angle in degrees (not radians!)
*/
double getAngle(const cv::Point2f& pt1, const cv::Point2f& pt2);

/**
	Calculate difference between two angles in the Cardesian coordinate system (0..360)
	@param alpha The first angle
	@param beta The second angle
	@return The correct difference between two angles
*/
double angleDifference(const double alpha, const double beta);

/**
	Calculate the Euclidian distance between two points in the 3D space
	using formula: D = sqrt( (x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2 )

	@param a The first point
	@param b The second point
	@return The distance between points
*/
double calcDistance(const cv::Point3f& a, const cv::Point3f& b);

/**
	Calculate the Euclidian distance between two points in the 2D space
	using formula: D = sqrt( (x1-x2)^2 + (y1-y2)^2 )

	@param a The first point
	@param b The second point
	@return The distance between points
*/
double calcDistance(const cv::Point2f& a, const cv::Point2f& b);

/**
  Calculate the angle, formed by three points in 3D space.

    a\                             a /
      \                             /
       \              or           /
        \___________              /____________
        b           c           b             c

  Used to calculate the elbow angle between the upper arm and lower arm,
  where the three points are: shoulder - elbow - wrist.

  @param a The first point
  @param b The second point
  @param c The third point
  @return The angle in radians
*/
double getThreePointAngle(const cv::Point3f& a, const cv::Point3f& b, const cv::Point3f& c);


/**
	Decrements the value, but doesn't allow it to go lower than the provided limit

	@param value The variable that needs to be decremented
	@param lowerLimit The minimal value beyond which the variable will not decrease
*/
void declim(int& value, const int lowerLimit);

/**
	Increments the value, but doesn't allow it to go higher than the provided limit

	@param value The variable that needs to be incremented
	@param upperLimit The maximum value beyond which the variable will not increase
*/
void inclim(int& value, const int upperLimit);

/**
	Overlays a transparent image over another image
	@param src The original soure image, upon which another image will be overlaid
	@param overlat The image that will be overlaid on the source image
	@param The upper left corner position of the overlay image on the destination image
*/
void overlayImage(cv::Mat& src, cv::Mat& overlay, const cv::Point & location);
