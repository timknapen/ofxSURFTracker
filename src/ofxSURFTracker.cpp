//
//  ofxSURFTracker.cpp
//
//  Created by Tim Knapen on 25/07/14.
//
//

#include "ofxSURFTracker.h"

//-----------------------------------------------------
ofxSURFTracker::ofxSURFTracker(){
    width = 320;
    height = 240;
    trackImg.allocate(width, height);
    inputImg.allocate(width, height);
    hessianThreshold = 500;
    octaves = 3;
    octaveLayers = 4;
    bUpright = bContrast = false;
    distanceThreshold = 0.2;
    bDrawFeatures = bDrawMatches = bDrawHomography = bDrawCrossHairs = true;
    bDrawImage = bDrawResponses = false;
    minMatches = 5;
}

//-----------------------------------------------------
ofxSURFTracker::~ofxSURFTracker(){
    
}

#pragma mark - DRAW

//-----------------------------------------------------
void ofxSURFTracker::draw(){
	
	ofPushStyle();
	ofDisableSmoothing();
    // Crosshairs
	if(bDrawCrossHairs){
		ofNoFill();
		ofPushStyle();
		ofSetColor(255, 255, 0);
		ofSetLineWidth(1);
		int d = 10; // length of croshairs
		ofDrawLine(0, 0, 0 + d, 0);
		ofDrawLine(0, 0, 0, 0 + d);
		ofDrawLine(width, 0, width - d, 0);
		ofDrawLine(width, 0, width, 0 + d);
		ofDrawLine(width, height, width - d, height);
		ofDrawLine(width, height, width, height - d);
		ofDrawLine(0, height, 0 + d, height);
		ofDrawLine(0, height, 0, height - d);
		ofPopStyle();
	}
    if(bDrawImage){
		ofPushStyle();
        ofSetColor(255);
        trackImg.draw(0,0);
		ofPopStyle();
    }
    if(bDrawResponses) {
        drawResponses();
    }
    if(bDrawMatches){
        drawMatches();
    }
    if(bDrawFeatures){
        drawFeatures();
    }
    if(bDrawHomography){
        drawHomoGraphy();
    }
	ofPopStyle();
}

//-----------------------------------------------------
int ofxSURFTracker::getNumGoodMatches(){
    return good_Matches.size();
}

//-----------------------------------------------------
void ofxSURFTracker::drawFeatures(){
    
    ofNoFill();
    ofSetColor(0, 255, 0);
    for(int i = 0; i < keyPoints_Scene.size(); i++){
        // ofDrawCircle(keyPoints_Scene[i].pt.x, keyPoints_Scene[i].pt.y, 2);
		ofPushMatrix();
		ofTranslate(keyPoints_Scene[i].pt.x, keyPoints_Scene[i].pt.y);
		ofDrawLine(-2, -2, 2, 2);
		ofDrawLine(2, -2, -2, 2);
		ofPopMatrix();
		
    }
    
}

//-----------------------------------------------------
void ofxSURFTracker::drawResponses(){
    ofNoFill();
    ofSetColor(0, 255, 0);
    for(int i = 0; i < keyPoints_Scene.size(); i++){
        KeyPoint kp = keyPoints_Scene[i];
        float l = kp.response/1000;
        ofDrawCircle(kp.pt.x, kp.pt.y,  l);
        ofDrawLine(kp.pt.x, kp.pt.y, kp.pt.x + l*cos(kp.angle), kp.pt.y + l*sin(kp.angle));
    }
}

//-----------------------------------------------------
void ofxSURFTracker::drawMatches(){
    for(int i = 0; i < good_Matches.size();i++){
        DMatch match = good_Matches[i];
        int d = match.distance/distanceThreshold * 255;
        ofSetColor(d, 0, 255 - d);
        Point2f p1 = keyPoints_Object[ good_Matches[i].queryIdx ].pt;
        Point2f p2 = keyPoints_Scene[ good_Matches[i].trainIdx ].pt;
        ofDrawLine( p1.x, p1.y, p2.x, p2.y);
        ofFill();
        ofDrawCircle(p1.x, p1.y, 2);
        ofDrawCircle(p2.x, p2.y, 2);
    }
}

//-----------------------------------------------------
void ofxSURFTracker::drawHomoGraphy(){
    // Draw the transformed bounding box
    ofNoFill();
    ofBeginShape();
    for(int i = 0; i < objectBounds_Transformed.size(); i++){
        ofVertex(objectBounds_Transformed[i].x, objectBounds_Transformed[i].y);
    }
    if( objectBounds_Transformed.size() > 0){
        ofVertex(objectBounds_Transformed[0].x, objectBounds_Transformed[0].y);
    }
    ofEndShape();
}

#pragma mark - track

//-----------------------------------------------------
void ofxSURFTracker::detect(ofImage &img){
    
    int inputWidth = img.getWidth();
    int inputHeight = img.getHeight();
    
    if(inputWidth < width || inputHeight < height){
        return; // detection impossible, because I can't crop out of this image
    }
    detect(img.getPixels(), inputWidth, inputHeight);
}

//-----------------------------------------------------
void ofxSURFTracker::detect(unsigned char *pix, int inputWidth, int inputHeight){
    
    /***
    code adapted from http://docs.opencv.org/doc/tutorials/features2d/feature_homography/feature_homography.html
     ***/
	
	// clear existing keypoints from previous frame
	keyPoints_Scene.clear();
	objectBounds_Transformed.clear();
	
    if( inputWidth != inputImg.getWidth() || inputHeight != inputImg.getHeight()){
        // this should only happen once
        inputImg.clear();
        inputImg.allocate(inputWidth, inputHeight);
    }
    
    // create the cvImage from the ofImage
    inputImg.setFromPixels(pix, inputWidth, inputHeight);
    inputImg.setROI( ofRectangle((inputWidth-width)/2,
                                 (inputHeight-height)/2,
                                 width,
                                 height
                                 )
                    );
    
    // take out the piece that we want to use.
    croppedImg.setFromPixels(inputImg.getRoiPixels(), width, height);
    
    // make it into a trackable grayscale image
    trackImg = croppedImg;
    
    // do some fancy contrast stuff
    if(bContrast){
        trackImg.contrastStretch();
    }
    
    // set up the feature detector
    detector =  SurfFeatureDetector(hessianThreshold,
                                    octaves,
                                    octaveLayers,
									bExtended,
                                    bUpright);
    
	
    
    // get the Mat to do the feature detection on
    Mat trackMat = cvarrToMat(trackImg.getCvImage());
    detector.detect( trackMat, keyPoints_Scene);
    
    // Calculate descriptors (feature vectors)
    extractor.compute( trackMat, keyPoints_Scene, descriptors_Scene );
	
}


//-----------------------------------------------------
void ofxSURFTracker::learnFeatures(){
    // copy features from the scene into the object
    keyPoints_Object = keyPoints_Scene;
    descriptors_Object = descriptors_Scene;
    
    // create bounding box to demonstrate the perspective transformation
    ofRectangle boundingBox;
    objectBounds.clear();
    for(int i = 0; i < keyPoints_Object.size(); i++){
        Point2f pt = keyPoints_Object[i].pt;
        if(i == 0){
            boundingBox.set(pt.x, pt.y, 0, 0);
        }
        boundingBox.growToInclude(pt.x, pt.y);
    }
    Point2f p1(boundingBox.position.x, boundingBox.position.y);
    Point2f p2(boundingBox.position.x + boundingBox.width, boundingBox.position.y + boundingBox.height);
    objectBounds.push_back(p1);
    objectBounds.push_back(Point2f(p2.x, p1.y));
    objectBounds.push_back(Point2f(p2.x, p2.y));
    objectBounds.push_back(Point2f(p1.x, p2.y));
}

//-----------------------------------------------------
int ofxSURFTracker::match(vector<KeyPoint> keyPoints, Mat descriptors, vector <Point2f> bounds){
	// this function tries to match keypoints and descriptors with the current scene
	// if there is no match, it returns 0,
	// if there is a match, it returns the number of matches and saves the perspective transform and bounding box
	

	
	// Matching descriptor vectors using FLANN matcher
	vector< DMatch > matches;
	if(!descriptors.empty() && !descriptors_Scene.empty() ){
		flannMatcher.match( descriptors, descriptors_Scene, matches);
	}
	
	// Quick calculation of max and min distances between keypoints
	double max_dist = 0;
	double min_dist = 100;
	for( int i = 0; i < matches.size(); i++ ) {
		double dist = matches[i].distance;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
	}
	
	// Filter matches upon quality : distance is between 0 and 1, lower is better
	good_Matches.clear();
	for( int i = 0; i < matches.size(); i++ ){
		if(matches[i].distance < 3 * min_dist && matches[i].distance < distanceThreshold){
			good_Matches.push_back( matches[i]);
		}
	}
	if(good_Matches.size() > minMatches){
		// being here means we have found a decent match
		
		return good_Matches.size();
	}else{
		return 0;
	}
	
}

//-----------------------------------------------------
void ofxSURFTracker::createHomography(vector<KeyPoint> keyPoints, vector <Point2f> bounds){
	// find the homography
	// transform the bounding box for this scene
	vector <Point2f> scene_pts;
	vector <Point2f> object_pts;
	objectBounds_Transformed.clear();
	if(good_Matches.size() > minMatches){
		for( int i = 0; i < good_Matches.size(); i++ )
		{
			//-- Get the keypoints from the good matches
			object_pts.push_back( keyPoints[ good_Matches[i].queryIdx ].pt );
			scene_pts.push_back( keyPoints_Scene[ good_Matches[i].trainIdx ].pt );
		}
		if( scene_pts.size() >5 && object_pts.size() > 5){
			homography = findHomography( object_pts, scene_pts, CV_RANSAC);
			perspectiveTransform( bounds, objectBounds_Transformed, homography);
		}
		
	}
}


#pragma mark - Transform

//-----------------------------------------------------
void ofxSURFTracker::transFormPoints(vector<ofPoint> & points){
    if(points.size() > 0){
        if(homography.empty()) return;
        vector<Point2f > inputs;
        vector<Point2f > results;
        for(int i = 0; i < points.size(); i++){
            inputs.push_back(Point2f( points[i].x,  points[i].y));
        }
        perspectiveTransform(inputs, results, homography);
        // back to the points array
        points.clear();
        for(int i = 0; i < results.size(); i++){
            points.push_back(ofPoint( results[i].x,  results[i].y));
        }
    }
}


#pragma mark - get and set



//-----------------------------------------------------
void ofxSURFTracker::setSize(int _width, int _height){
    // set the width and height of the image used for detection
    // this will be cropped out of the center of the image being fed to the tracker
	if(_width != width || _height != height){
    width = _width;
    height = _height;
    trackImg.clear();
    trackImg.allocate(width, height);
	}
}

//-----------------------------------------------------
vector <KeyPoint> ofxSURFTracker::getObjectKeyPoints(){
	return keyPoints_Object;
}

//-----------------------------------------------------
vector <Point2f> ofxSURFTracker::getObjectBounds(){
	return objectBounds;
}

//-----------------------------------------------------
Mat ofxSURFTracker::getObjectDescriptors(){
	return descriptors_Object;
}

//-----------------------------------------------------
ofImage ofxSURFTracker::getCroppedImage(){
	ofImage newImg;
	newImg.setFromPixels(croppedImg.getPixels(), croppedImg.getWidth(), croppedImg.getHeight(), OF_IMAGE_COLOR);
	return newImg;
}

//-----------------------------------------------------
float ofxSURFTracker::getWidth(){
	return width;
}

//-----------------------------------------------------
float ofxSURFTracker::getHeight(){
	return height;
}

//-----------------------------------------------------
vector <ofPoint> ofxSURFTracker::getTransformedBounds(){
	vector <ofPoint> pts;
	for(int i = 0; i < objectBounds_Transformed.size(); i++){
		pts.push_back(ofPoint(objectBounds_Transformed[i].x, objectBounds_Transformed[i].y));
	}
	return pts;
}


//-----------------------------------------------------
vector <ofPoint> ofxSURFTracker::getBounds(){
	vector <ofPoint> pts;
	for(int i = 0; i < objectBounds.size(); i++){
		pts.push_back(ofPoint(objectBounds[i].x, objectBounds[i].y));
	}
	return pts;
}





