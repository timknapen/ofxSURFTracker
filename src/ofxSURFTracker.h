//
//  ofxSURFTracker.h
//  Created by Tim Knapen on 25/07/14.
//
//

#ifndef __ofxSURFTracker__
#define __ofxSURFTracker__

#include "ofMain.h"
#include "ofxOpenCv.h"

// This depends on ofxOpenCV and opencv2
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;

class ofxSURFTracker {
public:
    
    ofxSURFTracker();
    ~ofxSURFTracker();
    
    void draw();
    void drawFeatures();
    void drawResponses();
    void drawMatches();
    void drawHomoGraphy();
    void setSize(int _width, int _height);
    void detect(ofImage& img);
    void detect(unsigned char * pix, int inputWidth, int inputHeight);
    void learnFeatures();
    
    float objectLifeTime; // is the object detected a long enough time to be trusted?
    
    
    // options
    float distanceThreshold;
    float hessianThreshold;
    float octaves;
    float octaveLayers;
    float minMatches;
    bool bUpright;
    bool bContrast;
    
    // Drawing options
    bool bDrawImage;
    bool bDrawFeatures;
    bool bDrawHomography;
    bool bDrawMatches;
    bool bDrawResponses;
    
    // transform a set of points with the found perspective transformation
    void transFormPoints( vector<ofPoint>&points);
    
    // get the number of approved matches, a measure for the quality
    int getNumGoodMatches();

private:
    int width, height;
    ofxCvColorImage inputImg;
    ofxCvColorImage croppedImg;
    ofxCvGrayscaleImage trackImg;
    
    // keypoints and descriptors
    vector<KeyPoint> keypoints_object, keypoints_scene;
    vector< DMatch > good_matches;
    Mat descriptors_object, descriptors_scene;
    Mat homography;
    vector <Point2f> object;
    vector <Point2f> object_transformed;
    
    SurfFeatureDetector detector;
    SurfDescriptorExtractor extractor;
    FlannBasedMatcher flannMatcher;
    
};



#endif /* defined(__ofxSURFTracker__) */
