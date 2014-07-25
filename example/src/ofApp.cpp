
#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {

    ofSetVerticalSync(true);
    vidGrabber.initGrabber(CAM_W, CAM_H);
    bRecording = true;
    surfTracker.setSize( SURF_W, SURF_H );
    surfTracker.bContrast = true;
    fps = 0;
    
}



//--------------------------------------------------------------
void ofApp::update() {
    
    if (bRecording){
        vidGrabber.update();
        if(vidGrabber.isFrameNew()) {
            if(bLearnFeatures){
                surfTracker.learnFeatures(); // has to happen before the detect step
            }
            surfTracker.detect(vidGrabber.getPixels(), CAM_W, CAM_H);
        }
    }
    
    fps += (ofGetFrameRate() - fps)/10;
    
}


//--------------------------------------------------------------
void ofApp::draw() {
	
	ofBackground(255);
    ofSetColor(255);
    
    vidGrabber.draw(0, 0);
    
    ofPushMatrix();
    { // draw the tracking image on top of the camera image
        ofTranslate((CAM_W - SURF_W)/2, (CAM_H - SURF_H)/2);
        surfTracker.draw();
    }
    ofPopMatrix();
    
    ofSetColor(0);
    
    ofDrawBitmapString("fps: " + ofToString(fps, 1) +
                       " - Life: " + ofToString(surfTracker.objectLifeTime, 2) +
                       " - Matches: " + ofToString(surfTracker.getNumGoodMatches())
                       , 10, CAM_H + 20);
    
    ofDrawBitmapString((string)"Keys: " +
                       "\n[SPACE]: learn features " + (bLearnFeatures ? "on" : "off") +
                       "\n[p]: pause/play camera " +
                       "\n[f]: toggle features " +
                       "\n[m]: toggle matches " +
                       "\n[r]: toggle responses " +
                       "\n[i]: toggle image "
                       , 10, CAM_H + 50);
    ofDrawBitmapString((string)"Test: "
                       + "\nHold something in front of the camera,"
                       + "\npress SPACE to start learning its features"
                       + "\nPress SPACE again to stop learning"
                       + "\nMove the object around to see the tracking"
                       , 250, CAM_H + 50);
    
    
}

#pragma mark - EVENTS
//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
	switch (key) {
        case ' ':
            bLearnFeatures = !bLearnFeatures;
            break;
        case 'p':
            bRecording = !bRecording;
            break;
        case 'f':
            surfTracker.bDrawFeatures = !surfTracker.bDrawFeatures;
            break;
        case 'm':
            surfTracker.bDrawMatches = !surfTracker.bDrawMatches;
            break;
        case 'r':
            surfTracker.bDrawResponses = !surfTracker.bDrawResponses;
            break;
        case 'i':
            surfTracker.bDrawImage = !surfTracker.bDrawImage;
            break;
            
	}
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
}

//--------------------------------------------------------------
void ofApp::exit() {
}



