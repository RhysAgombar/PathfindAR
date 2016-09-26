#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

//using namespace cv;
using namespace std;

class gridSquare {
public:
  cv::Point corner[4];
  // add 'contains' method for finding location
  // add  shading functions
};

/// Global variables
cv::Mat src, erosion_dst, dilation_dst;

int erosion_elem = 0;
int erosion_size = 0;
int dilation_elem = 0;
int dilation_size = 0;
int const max_elem = 2;
int const max_kernel_size = 21;

cv::Point2f test;

gridSquare** grid;


void genMarkers() {
  cv::Mat markerImage1, markerImage2, markerImage3, markerImage4;
  cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
  //cv::aruco::drawMarker(dictionary, 23, 200, markerImage, 1);

  dictionary.drawMarker(0, 200, markerImage1, 1);
  dictionary.drawMarker(1, 200, markerImage2, 1);
  dictionary.drawMarker(2, 200, markerImage3, 1);
  dictionary.drawMarker(3, 200, markerImage4, 1);

  imshow("Top Left Marker", markerImage1);
  imshow("Top Right Marker", markerImage2);
  imshow("Bottom Right Marker", markerImage3);
  imshow("Bottom Left Marker", markerImage4);

  cv::waitKey(0);
}

static bool readDetectorParameters(string filename, cv::Ptr<cv::aruco::DetectorParameters> &params) {
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if (!fs.isOpened())
    return false;
  fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
  fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
  fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
  fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
  fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
  fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
  fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
  fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
  fs["minDistanceToBorder"] >> params->minDistanceToBorder;
  fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
  fs["doCornerRefinement"] >> params->doCornerRefinement;
  fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
  fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
  fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
  fs["markerBorderBits"] >> params->markerBorderBits;
  fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
  fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
  fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
  fs["minOtsuStdDev"] >> params->minOtsuStdDev;
  fs["errorCorrectionRate"] >> params->errorCorrectionRate;
  return true;
}

vector<cv::Point2f> findCorners(vector<vector<cv::Point2f>> corners, vector<int> ids) {

  cv::Point2f midPoint;
  midPoint.x = 0;
  midPoint.y = 0;

  for (int i = 0; i < corners.size(); i++) {
    for (int j = 0; j < 4; j++) {
      midPoint.x += corners.at(i).at(j).x;
      midPoint.y += corners.at(i).at(j).y;
    }
  }

  midPoint.x = midPoint.x / (corners.size() * 4);
  midPoint.y = midPoint.y / (corners.size() * 4);

  vector<cv::Point2f> gridCorners(4);

  for (int i = 0; i < corners.size(); i++) {
    float pDist = INFINITY;
    for (int j = 0; j < 4; j++) {
      float dist = sqrt(pow(abs(midPoint.x - corners.at(i).at(j).x), 2.0) + pow(abs(midPoint.y - corners.at(i).at(j).y), 2.0));

      if (dist < pDist) {
        pDist = dist;
        gridCorners.at(ids.at(i)) = corners.at(i).at(j);
      }

    }
  }

  return gridCorners;
}

cv::Point findIntersection(cv::Point h1, cv::Point h2, cv::Point v1, cv::Point v2) {

  float hSlope, vSlope, hInt, vInt;
  float eps = 1e-6;

  cv::Point intersection;

  hSlope = (h2.y - h1.y) / (float)(h2.x - h1.x);
  hInt = h1.y - hSlope * h1.x;

  vSlope = (v2.y - v1.y) / (float)(v2.x - v1.x);
  vInt = v1.y - vSlope * v1.x;


  if (hSlope < eps) {
    intersection.y = h1.y;
  }
  else if (isinf(hSlope)) {
    intersection.x = h1.x;
  }
  else {
    intersection.y = hSlope * intersection.x + hInt;
  }


  if (vSlope < eps) {
    intersection.y = v1.y;
  }
  else if (isinf(vSlope)) {
    intersection.x = v1.x;
  }
  else {
    intersection.x = (vInt - hInt) / (float)(hSlope - vSlope);
  }


  return intersection;

}

void drawGrid(cv::Mat imageCopy, vector<cv::Point2f> gridCorners, int horiz, int vert) {
  cv::Mat dirVec[4]; // top left -> top right, bottom left -> bottom right, top left -> bottom left, top right -> bottom right
  float hDist, vDist, dist;

  vert = vert + 1;
  horiz = horiz + 1;

  grid = new gridSquare*[vert];
  for (int i = 0; i < vert; i++) {
    grid[i] = new gridSquare[horiz];
  }

  cv::Point **hLines;
  hLines = new cv::Point*[vert];
  for (int i = 0; i < vert; i++) {
    hLines[i] = new cv::Point[2];
  }

  cv::Point **vLines;
  vLines = new cv::Point*[horiz];
  for (int i = 0; i < horiz; i++) {
    vLines[i] = new cv::Point[2];
  }

  horiz = horiz - 1;
  vert = vert - 1;

  for (int i = 0; i <= vert; i++) {

    float x = 0;
    float y = 0;

    float distx;
    float disty;

    distx = gridCorners.at(3).x - gridCorners.at(0).x;
    disty = gridCorners.at(3).y - gridCorners.at(0).y;


    hLines[i][0] = cv::Point(gridCorners.at(0).x + (distx * (i / (float)vert)), gridCorners.at(0).y + (disty) * (float)(i / (float)vert));

    //cv::circle(imageCopy, cv::Point(gridCorners.at(0).x + (distx * (i/(float) vert)), gridCorners.at(0).y + (disty) * (float)(i/(float) vert)), 2, cv::Scalar(0, 0, 255));

    distx = gridCorners.at(2).x - gridCorners.at(1).x;
    disty = gridCorners.at(2).y - gridCorners.at(1).y;


    hLines[i][1] = cv::Point(gridCorners.at(1).x + (distx * (i / (float)vert)), gridCorners.at(1).y + (disty) * (float)(i / (float)vert));
  }

  for (int i = 0; i <= horiz; i++) {

    float x = 0;
    float y = 0;

    float distx;
    float disty;

    distx = gridCorners.at(1).x - gridCorners.at(0).x;
    disty = gridCorners.at(1).y - gridCorners.at(0).y;

    vLines[i][0] = cv::Point(gridCorners.at(0).x + (distx * (i / (float)horiz)), gridCorners.at(0).y + (disty) * (float)(i / (float)horiz));

    distx = gridCorners.at(2).x - gridCorners.at(3).x;
    disty = gridCorners.at(2).y - gridCorners.at(3).y;

    vLines[i][1] = cv::Point(gridCorners.at(3).x + (distx * (i / (float)horiz)), gridCorners.at(3).y + (disty) * (float)(i / (float)horiz));
  }

  for (int i = 0; i <= vert; i++) {
    cv::line(imageCopy, hLines[i][0], hLines[i][1], cv::Scalar(255, 255, 255));
  }

  for (int i = 0; i <= horiz; i++) {
    cv::line(imageCopy, vLines[i][0], vLines[i][1], cv::Scalar(255, 255, 255));
  }


  for (int i = 0; i < vert; i++) {
    for (int j = 0; j < horiz; j++) {
      grid[j][i].corner[0] = findIntersection(hLines[i][0], hLines[i][1], vLines[j][0], vLines[j][1]);
      grid[j][i].corner[1] = findIntersection(hLines[i][0], hLines[i][1], vLines[j + 1][0], vLines[j + 1][1]);
      grid[j][i].corner[2] = findIntersection(hLines[i + 1][0], hLines[i + 1][1], vLines[j + 1][0], vLines[j + 1][1]);
      grid[j][i].corner[3] = findIntersection(hLines[i + 1][0], hLines[i + 1][1], vLines[j][0], vLines[j][1]);
    }
  }


  //cv::circle(imageCopy, grid[2][2].corner[0], 2, cv::Scalar(0, 0, 255));
  //cv::circle(imageCopy, grid[2][2].corner[1], 2, cv::Scalar(0, 0, 255));
  //cv::circle(imageCopy, grid[2][2].corner[2], 2, cv::Scalar(0, 0, 255));
  //cv::circle(imageCopy, grid[2][2].corner[3], 2, cv::Scalar(0, 0, 255));


  //for (int i = 0; i <= horiz; i++) {
  //  for (int j = 0; j <= vert; j++) {

  //    cv::circle(imageCopy, findIntersection(hLines[i][0], hLines[i][1], vLines[j][0], vLines[j][1]), 2, cv::Scalar(0, 0, 255));
  //  }
  //}

}

int main(int, char** argv)
{
  cv::Mat inputImage;

  cv::Mat image, imageCopy;

  string imageName("../Images/ArUcoGridLarge3x3.png"); // by default
  image = cv::imread(imageName.c_str(), cv::IMREAD_COLOR); // Read the file

  int dictionaryId = 10; // alias for the DICT_6X6_250 dictionary

  cv::Ptr<cv::aruco::Dictionary> dictionary =
    cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

  cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();

  vector< int > ids;
  vector< vector< cv::Point2f > > corners, rejected;

  cv::aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
  readDetectorParameters("../detector_params.yml", detectorParams);

  image.copyTo(imageCopy);
  if (ids.size() > 0) {
    cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
  }

  vector<cv::Point2f> gridCorners = findCorners(corners, ids);

  // All rectangular object corners are organized in arrays, with the numbers going clockwise. eg: at(0) is the top left corner, at(1) is the top right, at(2) is the bottom right, at(3) is bottom left
  // This follows the convention that ArUco uses.
  cv::circle(imageCopy, gridCorners.at(0), 10, cv::Scalar(255, 255, 255), CV_FILLED, 8, 0);
  cv::circle(imageCopy, gridCorners.at(1), 10, cv::Scalar(255, 255, 255), CV_FILLED, 8, 0);
  cv::circle(imageCopy, gridCorners.at(2), 10, cv::Scalar(255, 255, 255), CV_FILLED, 8, 0);
  cv::circle(imageCopy, gridCorners.at(3), 10, cv::Scalar(255, 255, 255), CV_FILLED, 8, 0);

  int horiz = 3;//21;
  int vert = 3;//30;

  drawGrid(imageCopy, gridCorners, horiz, vert);


  cv::imshow("test", imageCopy);



  cv::waitKey(0);

  return 0;
}