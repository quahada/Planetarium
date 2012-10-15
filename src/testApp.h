#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"

// uncomment this to read from two kinects simultaneously
//#define USE_TWO_KINECTS

class testApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
	
	void drawPointCloud();
  
  void drawBlurryLine(const ofPoint & p1, const ofPoint & p2);
  float distanceBetweenPoints(const ofPoint & p1, const ofPoint & p2);
	
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
  
  vector< vector<double> > distanceMatrix;
	
	ofxKinect kinect;
	
#ifdef USE_TWO_KINECTS
	ofxKinect kinect2;
#endif
	
	ofxCvColorImage colorImg;
	
	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
	
  ofxCvGrayscaleImage grayShadowEffect; // the shadow Effect
  ofxCvGrayscaleImage grayOld; // for Diff
  ofxCvGrayscaleImage grayDiff; // for Diff
  
	
	ofxCvContourFinder contourFinder;
	
	bool bThreshWithOpenCV;
	bool bDrawPointCloud;
	
	int nearThreshold;
	int farThreshold;
	
	int angle;
  int width;
  int height;
  
  ofPoint blobCenter;
  ofPoint blobCenter1;
  ofPoint blobCenter2;
	
	// used for viewing the point cloud
	ofEasyCam easyCam;
};
