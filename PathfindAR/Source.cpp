﻿// REFACTOR THE HELL OUT OF THIS.
// REFACTOR THE HELL OUT OF THIS.
// REFACTOR THE HELL OUT OF THIS.
// REFACTOR THE HELL OUT OF THIS.
// REFACTOR THE HELL OUT OF THIS.


#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <string.h>


#include <time.h>

//using namespace cv;
using namespace std;

//class grid {
//public: 
//  cv::Point corner[4]; // The 4 corners of the grid, going in a clockwise fashion, starting with the top left
//
//};


class gridSquare {
public:
  cv::Point corner[4];

  vector<cv::Point> getPointsArray() {
    vector<cv::Point> cornerVec(4);

    for (int i = 0; i < 4; i++) {
      cornerVec.at(i) = corner[i];
    }

    return cornerVec;
  }

  bool contains(cv::Point center) {

    vector<cv::Point> cVec = getPointsArray();
    
    int result = cv::pointPolygonTest(cVec, center, false); // doesn't work

    if (result == 1) {
      return true;
    } else {
      return false;
    }


  }

  // add  shading functions
};

class arMarker {
public:
  cv::Point corner[4];
  cv::Point center;
  int id;
};

class Token {
public:
  int id;
  string name;
  cv::Point location;
  int mRange;
  int mRemain;
  int aRange;
  int lifespan;
  bool found = false;
  cv::Scalar colour;
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
bool start;

// Something is going very wrong with the grid count
int gridCount[12][10];

int hSize = 11;
int vSize = 9;

int idGen = 0;
vector<Token> tokenVec;
vector<cv::Point> uTokens;

int frameCount = 1;
long squareSum = 0;
int timer = 5;

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

float distance(cv::Point p1, cv::Point p2) {
  return sqrt(pow(abs(p2.x - p1.x), 2.0) + pow(abs(p2.y - p1.y), 2.0));
}

vector<cv::Point2f> updateCorners(vector<vector<cv::Point2f>> corners, vector<int> ids) {

  vector<cv::Point2f> gridCorners(4);

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



  for (int i = 0; i < corners.size(); i++) {

    float pDist = INFINITY;

    for (int j = 0; j < 4; j++) {
      float dist = distance(corners.at(i).at(j), midPoint);

      if (dist < pDist) {
        pDist = dist;
        gridCorners.at(ids.at(i)) = corners.at(i).at(j);

        // updates arUco markers
        for (int k = 0; k < 4; k++) {
          arUco[ids.at(i)].corner[k] = corners.at(i).at(k);
        }

        arUco[ids.at(i)].id = ids.at(i);

        cv::Point arUcoCenter = cv::Point(0, 0);
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

vector<cv::Point2f> findCorners(vector<vector<cv::Point2f>> corners, vector<int> ids) {

  if (corners.size() == 4) {

    // if all markers are found, we're good to go. Update the markers.
    return updateCorners(corners, ids);
 
  }  else {
    
    // If only some of the markers are found, we need to compensate.
    for (int i = 0; i < corners.size(); i++) { // for each detected marker...

      cv::Point center; // Get the center of it
      center.x = (corners[i][0].x + corners[i][1].x + corners[i][2].x + corners[i][3].x) / 4;
      center.y = (corners[i][0].y + corners[i][1].y + corners[i][2].y + corners[i][3].y) / 4;

      float dist = 1e9;
      float ndist;
      int swap = 0;

      for (int j = 0; j < 4; j++) { // out of all the previously detected markers 

        ndist = distance(center, arUco[j].center); // find the closest marker to the detected one

        if (ndist < dist)
        {
          dist = ndist;
          swap = j; // Save the location of the marker to update
        }
      }

      arUco[swap].corner[0] = corners[i][0]; // update marker
      arUco[swap].corner[1] = corners[i][1];
      arUco[swap].corner[2] = corners[i][2];
      arUco[swap].corner[3] = corners[i][3];

      arUco[swap].center = center;      
      // In this way, we only update the markers we have info for. The missing markers retain their last known value.
    }
    
    // Format the data in such a way that the Update function can read it.
    vector<vector<cv::Point2f>> cCorners(4, vector<cv::Point2f> (4));
    vector<int> cIds(4);

    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        cCorners.at(i).at(j) = arUco[i].corner[j];
      }
      cIds.at(i) = arUco[i].id;
    }
    
    return updateCorners(cCorners, cIds);
  
  }

}

cv::Point findIntersection(cv::Point h1, cv::Point h2, cv::Point v1, cv::Point v2) {

  cv::Point x = v1 - h1;
  cv::Point hDist = h2 - h1;
  cv::Point vDist = v2 - v1;

  float cross = hDist.x*vDist.y - hDist.y*vDist.x;

  // I'm not sure how this part works, but it does.
  double t = (x.x * vDist.y - x.y * vDist.x) / cross;
  cv::Point intersection = h1 + hDist * t;

  return intersection;

}

void drawGrid(cv::Mat imageCopy, vector<cv::Point2f> gridCorners, int horiz, int vert) {
  cv::Mat dirVec[4]; // top left -> top right, bottom left -> bottom right, top left -> bottom left, top right -> bottom right
  float hDist, vDist, dist;


  if (gridCorners.size() >= 3) {

    vert = vert + 1;
    horiz = horiz + 1;

    grid = new gridSquare*[vert];
    //gridCount = new int*[vert];
    for (int i = 0; i < vert; i++) {
      grid[i] = new gridSquare[horiz];
      //gridCount[i] = new int[horiz];
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

        grid[i][j].corner[0] = findIntersection(hLines[i][0], hLines[i][1], vLines[j][0], vLines[j][1]);
        grid[i][j].corner[1] = findIntersection(hLines[i][0], hLines[i][1], vLines[j + 1][0], vLines[j + 1][1]);
        grid[i][j].corner[2] = findIntersection(hLines[i + 1][0], hLines[i + 1][1], vLines[j + 1][0], vLines[j + 1][1]);
        grid[i][j].corner[3] = findIntersection(hLines[i + 1][0], hLines[i + 1][1], vLines[j][0], vLines[j][1]);

        //cv::circle(imageCopy, grid[j][i].corner[0], 2, cv::Scalar(0, 0, 255));
        //cv::circle(imageCopy, grid[j][i].corner[1], 2, cv::Scalar(0, 0, 255));
        //cv::circle(imageCopy, grid[j][i].corner[2], 2, cv::Scalar(0, 0, 255));
        //cv::circle(imageCopy, grid[j][i].corner[3], 2, cv::Scalar(0, 0, 255));

      }
    }
  }

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

bool isWithin(cv::Point p1, cv::Point p2, int rad) {

  if (distance(p1, p2) <= rad) {
    return true;
  }
  else {
    return false;
  }

}

void idTokens() {

  int closestDist = 1e9;
  int pos = 0;

  for (int i = 0; i < tokenVec.size(); i++) {
    tokenVec.at(i).found = false;
  }

  if (tokenVec.size() > 0) {
    vector<vector<float>> uTokenDists;

    for (int i = 0; i < tokenVec.size(); i++) {
      vector<float> newVec;
      uTokenDists.push_back(newVec);
      for (int k = 0; k < uTokens.size(); k++) {
        uTokenDists.at(i).push_back(distance(tokenVec.at(i).location, uTokens.at(k)));
      }
    }

    if (uTokens.size() > 5) {

      int k = 0;
      k++;

    }

    for (int k = 0; k < uTokens.size(); k++) {
      float dist = 1e9;
      int pos = -1;

      for (int i = 0; i < tokenVec.size(); i++) {
        if (uTokenDists.at(i).at(k) < dist) {
          float tempDist = uTokenDists.at(i).at(k);
          bool flag = true;

          for (int j = 0; j < uTokens.size(); j++) {
            if (tempDist > uTokenDists.at(i).at(j)) {
              flag = false;
            }
          }

          if (flag == true) {
            dist = uTokenDists.at(i).at(k);

            if (dist > 2) {
              if (dist < 3) {
                int stop = 0;
                stop++;
              }
            }

            pos = i;
          }   


        }
      }

      if (pos >= 0) {
        if (tokenVec.at(pos).found == false) {
          tokenVec.at(pos).location = uTokens.at(k);
          tokenVec.at(pos).found = true;
          uTokens.at(k).x = -1e9;
        }
      }
      

    }


  }

  for (int k = 0; k < uTokens.size(); k++) {
    if (uTokens.at(k).x > -1e4) {
      Token newToken;
      newToken.location = uTokens.at(k);
      newToken.lifespan = 1;
      newToken.id = idGen++;
      newToken.name = idGen;
      newToken.found = true;

      newToken.colour = cv::Scalar(rand() % 255, rand() % 255, rand() % 255);

      tokenVec.push_back(newToken);
    }
  }


  vector<Token> newTokenVec;

  for (int i = 0; i < tokenVec.size(); i++) {
    if (tokenVec.at(i).found == true) {
      tokenVec.at(i).lifespan++;

      if (tokenVec.at(i).lifespan > 100) {
        tokenVec.at(i).lifespan = 100;
      }

    }
    else {
      tokenVec.at(i).lifespan--;
    }

    if (tokenVec.at(i).lifespan > 0) {
      newTokenVec.push_back(tokenVec.at(i));
    }

  }

  tokenVec = newTokenVec;

  uTokens.clear();

}

void findTokens(cv::Mat imageCopy, cv::Mat imageOut) {
  cv::Mat imageGray, image2;
  vector<vector<cv::Point> > contours, cont2;
  vector<cv::Vec4i> hierarchy;


  cv::cvtColor(imageCopy,imageGray,CV_RGB2GRAY);

  cv::GaussianBlur(imageGray, imageGray, cv::Size(5, 5), 1.5, 1.5);

  cv::imshow("Gray", imageGray);

  cv::Canny(imageGray, imageGray, 100, 200, 3);

  cv::imshow("Edges", imageGray);

  cv::findContours(imageGray, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE,cv::Point(0, 0));
  
  cv::Mat drawing = cv::Mat::zeros(imageGray.size(), CV_8UC3);
  cv::Mat con = cv::Mat::zeros(imageGray.size(), CV_8UC3);

  for (int i = 0; i < contours.size(); i++) {

    if (contours.at(i).capacity() > 3) { // Eliminates the remaining noise
        cv::Point cent = getCenter(contours.at(i));

        bool flag = true;

        // eliminate aruco markers
        for (int j = 0; j < 4; j++) {
          if (isWithin(cent, arUco[j].center, (0.75 * distance(arUco[j].corner[0], arUco[j].corner[2])))) {
            flag = false;
          }
        }

        if (flag == true) {
          //if (hierarchy[i][3] < 0) {
            cont2.push_back(contours.at(i));
          //}                
        }
        
        
      }      

  }

  for (int i = 0; i < cont2.size(); i++)
  {
    cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    
    drawContours(con, cont2, i, color, 2, 8, hierarchy, 0, cv::Point());


    // IDEA! Let's take advantage of the fact that all the contours for each token will be situated around a central point.
    // Let's find the squares where more than 1 or 2 countour centers are found (to eliminate noise/displacement)
    cv::Point cent = getCenter(cont2.at(i));
        
    for (int j = 0; j < vSize - 1; j++) {
      for (int k = 0; k < hSize - 1; k++) {
        //bool contains = grid[j][k].contains(cent);

        if (grid[j][k].contains(cent) == true) {
          gridCount[j][k]++;
        }

        // something is wrong with my grid assignment
/*
        cv::circle(drawing, grid[j][k].corner[0], 5, cv::Scalar(255, 0, 0), -1);
        cv::circle(drawing, grid[j][k].corner[1], 5, cv::Scalar(255, 0, 0), -1);
        cv::circle(drawing, grid[j][k].corner[2], 5, cv::Scalar(255, 0, 0), -1);
        cv::circle(drawing, grid[j][k].corner[3], 5, cv::Scalar(255, 0, 0), -1);*/

        //cv::imshow("Contours", con);

        //cv::waitKey(0);

      }
    }

    for (int j = 0; j < vSize - 1; j++) {
      for (int k = 0; k < hSize - 1; k++) {

        if (gridCount[j][k] > 1) { // Requiring more than one countour in a square cuts down on noise/drift a bit

          vector<cv::Point> testVec = grid[j][k].getPointsArray();

          cv::Point polyPoints[1][4];
          polyPoints[0][0] = grid[j][k].corner[0];
          polyPoints[0][1] = grid[j][k].corner[1];
          polyPoints[0][2] = grid[j][k].corner[2];
          polyPoints[0][3] = grid[j][k].corner[3];

          const cv::Point* ppt[1] = { polyPoints[0] };
          int npt[] = { 4 };

          cv::fillPoly(imageOut, ppt, npt, 1, cv::Scalar(255, 255, 255));

          gridCount[j][k] = 0;

          if (!(find(uTokens.begin(), uTokens.end(), cv::Point(j, k)) != uTokens.end())) {
            uTokens.push_back(cv::Point(j, k));
          }   

        }

      }
    }

    cv::circle(drawing, cent, 5, cv::Scalar(255, 0, 255), -1);

  }

  bool pauseTracking = false;
  float squareAverage = 0;


  // this algorithm is fucky
  //if (squareSum > 10) {
  //  float squareAverage = (squareSum /(float) frameCount);
  //  if (squareAverage < (uTokens.size() - 1)) {  //(squareAverage/2)) ) {

  //    // Hand detected

  //    cv::circle(imageOut, cv::Point(0,0), 20, cv::Scalar(0, 0, 255), -1);
  //    pauseTracking = true;

  //    uTokens.clear();

  //  } else {
  //    frameCount++;
  //    squareSum += uTokens.size();
  //  }
  //} else {
  //  frameCount++;
  //  squareSum += uTokens.size();
  //}

  //if (pauseTracking == false && timer == 5) {
    idTokens();
  //}
  //else {
  //  timer--;
  //  cv::circle(imageOut, cv::Point(0, 0), 20, cv::Scalar(0, 0, 255), -1);

  //  if (timer < 0) {
  //    frameCount++;
  //    squareSum += uTokens.size();
  //    timer = 5;
  //  }
  //}

    uTokens.clear();


  pauseTracking = false;
 

  for (int i = 0; i < tokenVec.size(); i++) {

    int j = tokenVec.at(i).location.x;
    int k = tokenVec.at(i).location.y;

    cv::Point polyPoints[1][4];
    polyPoints[0][0] = grid[j][k].corner[0];
    polyPoints[0][1] = grid[j][k].corner[1];
    polyPoints[0][2] = grid[j][k].corner[2];
    polyPoints[0][3] = grid[j][k].corner[3];

    const cv::Point* ppt[1] = { polyPoints[0] };
    int npt[] = { 4 };

    cv::fillPoly(imageOut, ppt, npt, 1, tokenVec.at(i).colour);
  }

  cv::imshow("Contour Centers", drawing);

}

int main(int, char** argv)
{

  cv::VideoCapture inputVideo; 
  inputVideo.open("../Videos/TEST_ADD.mp4");

  int dictionaryId = 10; // alias for the DICT_6X6_250 dictionary

  cv::Ptr<cv::aruco::Dictionary> dictionary =
    cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
  
  start = false;

  while (inputVideo.grab()) {
    cv::Mat image, imageCopy; 
    inputVideo.retrieve(image);  

    std::vector<int> ids; 
    std::vector<std::vector<cv::Point2f> > corners, rejected; 
    
    cv::resize(image, image, cv::Size(640, 480), 0, 0, cv::INTER_CUBIC);
    image.copyTo(imageCopy);

    GaussianBlur(image, image, cv::Size(3, 3), 1.5, 1.5);
    cv::aruco::detectMarkers(image, dictionary, corners, ids);  

    vector<cv::Point2f> gridCorners;

    if (ids.size() == 4) {  
      start = true;
    }

    if (start == true){
      // if at least one marker detected 
      if (ids.size() > 0) {
        //cv::aruco::drawDetectedMarkers(image, corners, ids);
        gridCorners = findCorners(corners, ids);

        cv::circle(image, gridCorners.at(0), 5, cv::Scalar(255, 0, 0), -1);
        cv::circle(image, gridCorners.at(1), 5, cv::Scalar(255, 0, 0), -1);
        cv::circle(image, gridCorners.at(2), 5, cv::Scalar(255, 0, 0), -1);
        cv::circle(image, gridCorners.at(3), 5, cv::Scalar(255, 0, 0), -1);
      }

      cv::circle(image, arUco[0].center, 5, cv::Scalar(255, 0, 255), -1);
      cv::circle(image, arUco[1].center, 5, cv::Scalar(255, 0, 255), -1);
      cv::circle(image, arUco[2].center, 5, cv::Scalar(255, 0, 255), -1);
      cv::circle(image, arUco[3].center, 5, cv::Scalar(255, 0, 255), -1);

      drawGrid(image, gridCorners, hSize, vSize);

      // refine findTokens
      findTokens(imageCopy, image);
      
    }

    cv::imshow("out", image); 
    char key = (char)cv::waitKey(30); 
    if (key == 27) break;
  }

  return 0;
}