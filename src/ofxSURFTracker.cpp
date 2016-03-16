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
    bDrawFeatures = bDrawMatches  = bDrawHomography = true;
    bDrawImage = bDrawResponses = false;
    minMatches = 5;
    objectLifeTime = 0;
}

//-----------------------------------------------------
ofxSURFTracker::~ofxSURFTracker(){
    
}

#pragma mark - DRAW

//-----------------------------------------------------
void ofxSURFTracker::draw(){
    
    // Crosshairs
    ofNoFill();
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
    
    if(bDrawImage){
        ofSetColor(255);
        trackImg.draw(0,0);
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
}

//-----------------------------------------------------
int ofxSURFTracker::getNumGoodMatches(){
    return good_matches.size();
}

//-----------------------------------------------------
void ofxSURFTracker::drawFeatures(){
    
    ofNoFill();
    ofSetColor(0, 255, 0);
    for(int i = 0; i < keypoints_scene.size(); i++){
        ofDrawCircle(keypoints_scene[i].pt.x, keypoints_scene[i].pt.y, 2);
    }
    
}

//-----------------------------------------------------
void ofxSURFTracker::drawResponses(){
    ofNoFill();
    ofSetColor(0, 255, 0);
    for(int i = 0; i < keypoints_scene.size(); i++){
        KeyPoint kp = keypoints_scene[i];
        float l = kp.response/1000;
        ofDrawCircle(kp.pt.x, kp.pt.y,  l);
        ofDrawLine(kp.pt.x, kp.pt.y, kp.pt.x + l*cos(kp.angle), kp.pt.y + l*sin(kp.angle));
    }
}

//-----------------------------------------------------
void ofxSURFTracker::drawMatches(){
    for(int i = 0; i < good_matches.size();i++){
        DMatch match = good_matches[i];
        int d = match.distance*500;
        ofSetColor(d, 0, 500 - d);
        Point2f p1 = keypoints_object[ good_matches[i].queryIdx ].pt;
        Point2f p2 = keypoints_scene[ good_matches[i].trainIdx ].pt;
        ofDrawLine( p1.x, p1.y, p2.x, p2.y);
        ofFill();
        ofDrawCircle(p1.x, p1.y, 2);
        ofDrawCircle(p2.x, p2.y, 2);
    }
}


//-----------------------------------------------------
void ofxSURFTracker::drawHomoGraphy(){
    // Draw the transformed bounding box
    ofSetColor(255, 255 - 255* objectLifeTime, 0);
    ofNoFill();
    ofBeginShape();
    for(int i = 0; i < object_transformed.size(); i++){
        ofVertex(object_transformed[i].x, object_transformed[i].y);
    }
    if( object_transformed.size() > 0){
        ofVertex(object_transformed[0].x, object_transformed[0].y);
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
    
    if( inputWidth != inputImg.getWidth() || inputHeight != inputImg.getHeight()){
        // this should only happen once
        inputImg.clear();
        inputImg.allocate(inputWidth, inputHeight);
        cout << "ofxSURFTracker : re-allocated the input image."<<endl;
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
                                    bUpright);
    
    // clear existing keypoints from previous frame
    keypoints_scene.clear();
    
    // get the Mat to do the feature detection on
    Mat trackMat = cvarrToMat(trackImg.getCvImage());
    detector.detect( trackMat, keypoints_scene);
    
    // Calculate descriptors (feature vectors)
    extractor.compute( trackMat, keypoints_scene, descriptors_scene );
    
    // Matching descriptor vectors using FLANN matcher
    vector< DMatch > matches;
    if(!descriptors_object.empty() && !descriptors_scene.empty() ){
        flannMatcher.match( descriptors_object, descriptors_scene, matches);
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
    good_matches.clear();
    for( int i = 0; i < matches.size(); i++ ){
        if(matches[i].distance < 3 * min_dist && matches[i].distance < distanceThreshold){
            good_matches.push_back( matches[i]);
        }
    }
    
    // find the homography
    // transform the bounding box for this scene
    vector <Point2f> scene_pts;
    vector <Point2f> object_pts;
    object_transformed.clear();
    if(good_matches.size() > minMatches){
        for( int i = 0; i < good_matches.size(); i++ )
        {
            //-- Get the keypoints from the good matches
            object_pts.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
            scene_pts.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
        }
        if( scene_pts.size() >5 && object_pts.size() > 5){
            homography = findHomography( object_pts, scene_pts, CV_RANSAC);
            perspectiveTransform( object, object_transformed, homography);
        }
        // being here means we have found a decent match
        objectLifeTime += 0.05;
        
    }else{
        // we haven't found a decent match
        objectLifeTime -= 0.05;
    }
    if(objectLifeTime > 1){
        objectLifeTime = 1;
    }else if( objectLifeTime < 0){
        objectLifeTime = 0;
    }
}


//-----------------------------------------------------
void ofxSURFTracker::learnFeatures(){
    // copy features from the scene into the object
    keypoints_object = keypoints_scene;
    descriptors_object = descriptors_scene;
    
    // create bounding box to demonstrate the perspective transformation
    ofRectangle boundingBox;
    object.clear();
    for(int i = 0; i < keypoints_object.size(); i++){
        Point2f pt = keypoints_object[i].pt;
        if(i == 0){
            boundingBox.set(pt.x, pt.y, 0, 0);
        }
        boundingBox.growToInclude(pt.x, pt.y);
    }
    Point2f p1(boundingBox.position.x, boundingBox.position.y);
    Point2f p2(boundingBox.position.x + boundingBox.width, boundingBox.position.y + boundingBox.height);
    object.push_back(p1);
    object.push_back(Point2f(p2.x, p1.y));
    object.push_back(Point2f(p2.x, p2.y));
    object.push_back(Point2f(p1.x, p2.y));
    
    // a new object starts its life at age 0
    objectLifeTime = 0;
}

#pragma mark - Transform

void ofxSURFTracker::transFormPoints(vector<ofPoint> & points){
    
    if(objectLifeTime >=1 && points.size() > 0){
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
    width = _width;
    height = _height;
    trackImg.clear();
    trackImg.allocate(width, height);
}




