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

class arMarker {
public:
  cv::Point corner[4];
  cv::Point center;
  int id;
};

/// Global variables
cv::Mat src, erosion_dst, dilation_dst;

int erosion_elem = 0;
int erosion_size = 0;
int dilation_elem = 0;
int dilation_size = 0;
int const max_elem = 2;
int const max_kernel_size = 21;
cv::RNG rng(12345);

cv::Point2f test;

gridSquare** grid;
arMarker arUco[4];


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

        // updates arUco markers
        for (int k = 0; k < 4; k++) {
          arUco[ids.at(i)].corner[k] = corners.at(i).at(k);
        }

        arUco[ids.at(i)].id = ids.at(i);

        cv::Point arUcoCenter = cv::Point(0,0);
        for (int l = 0; l < 4; l++) {
          arUcoCenter.x += arUco[ids.at(i)].corner[l].x;
          arUcoCenter.y += arUco[ids.at(i)].corner[l].y;
        }

        arUcoCenter.x = arUcoCenter.x / 4;
        arUcoCenter.y = arUcoCenter.y / 4;

        arUco[ids.at(i)].center = arUcoCenter;        
        
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

cv::Point getCenter(vector<cv::Point> contour) {
  // We assume all contours are somewhat balanced, therefore the center is determined using a bounding box.

  int minX = 1e9;
  int maxX = -1;
  int minY = 1e9;
  int maxY = -1;

  for (int i = 0; i < contour.size() - 1; i++) {

    if (contour.at(i).x < minX) {
      minX = contour.at(i).x;
    }
    if (contour.at(i).x > maxX) {
      maxX = contour.at(i).x;
    }

    if (contour.at(i).y < minY) {
      minY = contour.at(i).y;
    }
    if (contour.at(i).y > maxY) {
      maxY = contour.at(i).y;
    }

  }

  return cv::Point((minX + maxX) / 2, (minY + maxY) / 2);

}

float distance(cv::Point p1, cv::Point p2) {
  return sqrt(pow(abs(p2.x - p1.x), 2.0) + pow(abs(p2.y - p1.y), 2.0));
}

bool isWithin(cv::Point p1, cv::Point p2, int rad) {

  if (distance(p1, p2) <= rad) {
    return true;
  }
  else {
    return false;
  }

}

void findTokens(cv::Mat imageCopy) {
  cv::Mat imageGray, image2;
  vector<vector<cv::Point> > contours, cont2;
  vector<cv::Vec4i> hierarchy;


  cv::cvtColor(imageCopy,imageGray,CV_RGB2GRAY);

  cv::GaussianBlur(imageGray, imageGray, cv::Size(5, 5), 1.5, 1.5);

  cv::Canny(imageGray, imageGray, 100, 200, 3);

  cv::findContours(imageGray, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE,cv::Point(0, 0));
  
  cv::Mat drawing = cv::Mat::zeros(imageGray.size(), CV_8UC3);

  for (int i = 0; i < contours.size(); i++) {
   // if (hierarchy[i][3] == -1) {
      if (contours.at(i).capacity() > 3) {
        cv::Point cent = getCenter(contours.at(i));

        bool flag = true;

        // eliminate aruco markers
        for (int j = 0; j < 4; j++) {
          if (isWithin(cent, arUco[j].center, 10)) {
            flag = false;
          }
        }

        if (flag == true) {
          cont2.push_back(contours.at(i));
        }
        
        
      }      
   // }
  }


  for (int i = 0; i < cont2.size(); i++)
  {
    cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    drawContours(drawing, cont2, i, color, 2, 8, hierarchy, 0, cv::Point());

    cv::Point cent = getCenter(cont2.at(i));

    cv::circle(drawing, cent, 5, cv::Scalar(255, 0, 255), -1);

  }


  cv::imshow("test2", drawing);

}

//int main(int, char** argv)
//{
//
//  cv::Mat inputImage;
//
//  cv::Mat image, imageCopy;
//
//  string imageName("../Images/Capture2.PNG"); // by default
//  image = cv::imread(imageName.c_str(), cv::IMREAD_COLOR); // Read the file
//
//  cv::GaussianBlur(image, image, cv::Size(3, 3), 1.5, 1.5);
//
//  cv::imshow("Original", image);
//
//  int dictionaryId = 10; // alias for the DICT_6X6_250 dictionary
//
//  cv::Ptr<cv::aruco::Dictionary> dictionary =
//    cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
//
//  cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
//
//  vector< int > ids;
//  vector< vector< cv::Point2f > > corners, rejected;
//
//  cv::aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
//  readDetectorParameters("../detector_params.yml", detectorParams);
//
//  image.copyTo(imageCopy);
//  if (ids.size() > 0) {
//    cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
//  }
//
//  vector<cv::Point2f> gridCorners = findCorners(corners, ids);
//  // All rectangular object corners are organized in arrays, with the numbers going clockwise. eg: at(0) is the top left corner, at(1) is the top right, at(2) is the bottom right, at(3) is bottom left
//  // This follows the convention that ArUco uses.
//
//
//  int horiz = 11;//21;
//  int vert = 10;//30;
//
//  drawGrid(imageCopy, gridCorners, horiz, vert);
//
//  cv::circle(imageCopy, arUco[0].center, 5, cv::Scalar(255, 0, 255), -1);
//  cv::circle(imageCopy, arUco[1].center, 5, cv::Scalar(255, 0, 255), -1);
//  cv::circle(imageCopy, arUco[2].center, 5, cv::Scalar(255, 0, 255), -1);
//  cv::circle(imageCopy, arUco[3].center, 5, cv::Scalar(255, 0, 255), -1);
//
//  findTokens(image);
//
//  cv::imshow("test", imageCopy);
//
//  cv::waitKey(0);
//
//  return 0;
//}


int main(int, char** argv)
{

  cv::VideoCapture inputVideo; 
  inputVideo.open("../Videos/GridClose.mp4");

  int dictionaryId = 10; // alias for the DICT_6X6_250 dictionary

  cv::Ptr<cv::aruco::Dictionary> dictionary =
    cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

  cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();

  bool start = false;

  while (inputVideo.grab()) {
    cv::Mat image, imageCopy; inputVideo.retrieve(image); image.copyTo(imageCopy);

    std::vector<int> ids; 
    std::vector<std::vector<cv::Point2f> > corners, rejected; 
    
    cv::resize(image, image, cv::Size(640, 480), 0, 0, cv::INTER_CUBIC);

    GaussianBlur(image, image, cv::Size(3, 3), 1.5, 1.5);

    //readDetectorParameters("../detector_params.yml", detectorParams);
   // cv::aruco::detectMarkers(image, dictionary, corners, ids,detectorParams, rejected);
    cv::aruco::detectMarkers(image, dictionary, corners, ids);
   
    //for (std::vector< vector<cv::Point2f> >::iterator it = rejected.begin(); it != rejected.end(); ++it) {
    //  vector<cv::Point2f> sqPoints = *it;
    //  //cout << sqPoints.size() << endl;
    //  //Point pt2(it[1].x, it[1].y);
    //  cv::line(image, sqPoints[0], sqPoints[1], CV_RGB(255, 0, 0));
    //  cv::line(image, sqPoints[2], sqPoints[1], CV_RGB(255, 0, 0));
    //  cv::line(image, sqPoints[2], sqPoints[3], CV_RGB(255, 0, 0));
    //  cv::line(image, sqPoints[0], sqPoints[3], CV_RGB(255, 0, 0));
    //}
    //GaussianBlur(image, image, cv::Size(3, 3), 1.5, 1.5);
    

    // if at least one marker detected 
    if (ids.size() > 0) cv::aruco::drawDetectedMarkers(image, corners, ids);

   

    if (ids.size() == 4) {
      vector<cv::Point2f> gridCorners = findCorners(corners, ids);

      start = true;

    }

    if (start == true){

    }

    cv::imshow("out", image); 
    char key = (char)cv::waitKey(30); 
    if (key == 27) break;
  }

  //cv::VideoCapture cap("../Videos/GridClose.mp4"); // open the default camera
  //if (!cap.isOpened())  // check if we succeeded
  //  return -1;

  //cv::Mat edges;
  //cv::namedWindow("edges", 1);

  //
  //int dictionaryId = 10; // alias for the DICT_6X6_250 dictionary

  //cv::Ptr<cv::aruco::Dictionary> dictionary =
  //  cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

  //cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();

  //vector< int > ids;
  //vector< vector< cv::Point2f > > corners, rejected;

  ////cv::aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
  //readDetectorParameters("../detector_params.yml", detectorParams);


  //for (;;)
  //{
  //  cv::Mat image;
  //  cap >> image; // get a new frame from camera
  //  //cvtColor(image, image, cv::COLOR_BGR2GRAY);

  //  //cv::resize(image, image, cv::Size(640, 480), 0, 0, cv::INTER_CUBIC);

  //  //GaussianBlur(image, image, cv::Size(3, 3), 1.5, 1.5);

  //  //findTokens(image);
  //  cv::aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

  //    if (ids.size() > 0) {
  //      cv::aruco::drawDetectedMarkers(image, corners, ids);
  //    }

  //      //int horiz = 11;//21;
  //      //int vert = 10;//30;
  //    
  //      //vector<cv::Point2f> gridCorners = findCorners(corners, ids);
  //      //drawGrid(image, gridCorners, horiz, vert);


  //  cv::imshow("edges", image);
  //  if (cv::waitKey(30) >= 0) break;
  //}


  return 0;
}