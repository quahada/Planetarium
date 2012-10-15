#include "testApp.h"
#include <math.h>

//--------------------------------------------------------------
void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	//kinect.init();
	kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
	
#ifdef USE_TWO_KINECTS
	kinect2.init();
	kinect2.open();
#endif
	
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
  grayShadowEffect.allocate(kinect.width, kinect.height);
  grayOld.allocate(kinect.width, kinect.height);
	
	nearThreshold = 230;
	farThreshold = 7;
	bThreshWithOpenCV = true;
	
	ofSetFrameRate(60);
  ofSetLineWidth(3);
  ofEnableSmoothing();
  //ofEnableBlendMode(ofBlendMode blendMode);
  ofEnableAlphaBlending();
	
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	// start from the front
	bDrawPointCloud = false;
  
  width = 240;
  height = 360;
}

//--------------------------------------------------------------
void testApp::update() {
	
	ofBackground(100, 100, 100);
	
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		
		// load grayscale depth image from the kinect source
		//grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
		grayImage.setFromPixels(kinect.getPixels(), kinect.width, kinect.height);
		
		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		if(bThreshWithOpenCV) {
      
      grayOld = grayImage;
      grayImage.blurGaussian(20);
      /*
      grayDiff = grayImage;
      grayDiff -= grayOld;
      grayOld = grayImage;
      grayImage = grayDiff;
      */
      colorImg = grayImage;
			grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			grayThreshNear.threshold(nearThreshold, true);
			//grayThreshNear.threshold(nearThreshold);
			grayThreshFar.threshold(farThreshold);
      grayShadowEffect.convertToRange(0, 235);
      grayThreshFar.mirror(false, true);
      grayShadowEffect += grayThreshFar;
      
      contourFinder.findContours(grayThreshFar, 150, (kinect.width*kinect.height)/10, 10, false, true);      
      //cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);

      ///*
      for (int blog_index = 0; blog_index<contourFinder.nBlobs; blog_index++){
        colorImg.drawBlobIntoMe(contourFinder.blobs.at(blog_index), 255);
        //blobCenter = contourFinder.blobs.at(blog_index).centroid;
      }
      //*/
      //colorImg.blurGaussian(50);
      grayShadowEffect.blurGaussian(50);
      
		} else {
			
			// or we do it ourselves - show people how they can work with the pixels
			unsigned char * pix = grayImage.getPixels();
			
			int numPixels = grayImage.getWidth() * grayImage.getHeight();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
		}
		
		// update the cv images
		grayImage.flagImageChanged();
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		//contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
	}
	
#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif
}

//--------------------------------------------------------------
void testApp::draw() {
	
	ofSetColor(255, 255, 255);
	
	if(bDrawPointCloud) {
		easyCam.begin();
		drawPointCloud();
		easyCam.end();
	} else {
		// draw from the live kinect
		//kinect.drawDepth(10, 10, 400, 300);
		//kinect.draw(420, 10, 400, 300);
		
    //grayImage.draw(0, 0);
    //grayThreshFar.draw(width+10, 0, 2*width+10, height);
    //grayShadowEffect.draw(width+10, 0, 2*width+10, height);
    //colorImg.draw(width+10, 0, 2*width+10, height);
    grayShadowEffect.draw(0,0);
    grayOld.draw(kinect.width+10, 0);
    ///*
    //ofRect(0, 0, width, height);
    //contourFinder.draw(width+10, height+10, 2*width+10, 2*height+10);
    ofColor c(255, 255, 255);
    ///*
    //blobCenter1 = contourFinder.blobs.at(0).centroid;
    //blobCenter1.x += 1;
    //blobCenter1.y += 1;
    blobCenter1 = ofPoint(0,0);
    distanceMatrix.resize( contourFinder.nBlobs , vector<double>( contourFinder.nBlobs , 0 ) );
    for(int i = 0; i < contourFinder.nBlobs; i++) {
      /*
      ofRectangle r = contourFinder.blobs.at(i).boundingRect;
      r.x += width+10; r.y += height+10;
      c.setHsb(i * 64, 255, 255);
      ofSetColor(c);
      ofRect(r);
      */
      //contourFinder.blobs.at(i).draw();
      blobCenter2 = contourFinder.blobs.at(i).centroid;
      ///*
      int min1 = 0;
      int max = 0;
      int secondmax = 0;
      int thirdmax = 0;
      for(int j = 0; j<i; j++){
        if (i != j){
          distanceMatrix[i][j] = distanceBetweenPoints(contourFinder.blobs.at(i).centroid,contourFinder.blobs.at(j).centroid);
          if ((distanceMatrix[i][j] >distanceMatrix[i][max]) & (distanceMatrix[i][j] < 700)){
            thirdmax = secondmax;
            secondmax = max;
            max = j;
          }
          if ((distanceMatrix[i][j] < distanceMatrix[i][min1]) | (distanceMatrix[i][min1] == 0)){
            min1 = j;
          }
        }
        else{
          distanceMatrix[i][j] = 0;
        }
      }
      //*/
      if (i>0){
        //ofLine(blobCenter1.x+width+10,blobCenter1.y+height+10,blobCenter2.x+width+10,blobCenter2.y+height+10);
        //ofLine(blobCenter1,blobCenter2);
        //drawBlurryLine(blobCenter1,blobCenter2);
        drawBlurryLine(contourFinder.blobs.at(i).centroid,contourFinder.blobs.at(max).centroid);
        drawBlurryLine(contourFinder.blobs.at(i).centroid,contourFinder.blobs.at(min1).centroid);
      }
      blobCenter1 = blobCenter2;
    }
    //*/
    
    

    
#ifdef USE_TWO_KINECTS
		kinect2.draw(420, 320, 400, 300);
#endif
	}
	
	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;
	reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
	<< ofToString(kinect.getMksAccel().y, 2) << " / "
	<< ofToString(kinect.getMksAccel().z, 2) << endl
	<< "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
	<< "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
	<< "set near threshold " << nearThreshold << " (press: + -)" << endl
	<< "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
	<< ", fps: " << ofGetFrameRate() << endl
	<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl
	<< "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl;
	ofDrawBitmapString(reportStream.str(),20,652);
}

void testApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				mesh.addColor(kinect.getColorAt(x,y));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	glEnable(GL_DEPTH_TEST);
	mesh.drawVertices();
	glDisable(GL_DEPTH_TEST);
	ofPopMatrix();
}

//--------------------------------------------------------------
float testApp::distanceBetweenPoints(const ofPoint & p1, const ofPoint & p2){
	float distance = sqrt(pow(p1.x - p1.y,2) + pow(p1.z - p2.x,2) + pow(p2.y - p2.z,2));
  //cout << "\ndistance: ";
  //cout << distance;
}

//--------------------------------------------------------------
void testApp::drawBlurryLine(const ofPoint & p1, const ofPoint & p2){
  //ofSetColor(255,255,255,255);
	//ofLine(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);
  //ofSetColor(255,255,153,85); // Pale Canary Yellow
  ofSetColor(255,255,255,85);
	ofLine(p1.x+2, p1.y, p1.z, p2.x+2, p2.y, p2.z);
	ofLine(p1.x+1, p1.y, p1.z, p2.x+1, p2.y, p2.z);
	ofLine(p1.x-1, p1.y, p1.z, p2.x-1, p2.y, p2.z);
	ofLine(p1.x-2, p1.y, p1.z, p2.x-2, p2.y, p2.z);
}

//--------------------------------------------------------------
void testApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	
#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;
			
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
			
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
			
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
	}
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}
