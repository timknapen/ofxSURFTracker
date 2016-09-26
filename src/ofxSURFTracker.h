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
//#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
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
	/** 
	 PARAMETERS
	 INT extended
		0 means that the basic descriptors (64 elements each) shall be computed
		1 means that the extended descriptors (128 elements each) shall be computed
	 
	 INT upright
	 0 means that detector computes orientation of each feature.
	 1 means that the orientation is not computed (which is much, much faster). 
	 For example, if you match images from a stereo pair, or do image stitching, the matched features likely have very similar angles, and you can speed up feature extraction by setting upright=1.
	 
	 DOUBLE hessianThreshold
	 Threshold for the keypoint detector. 
	 Only features, whose hessian is larger than hessianThreshold are retained by the detector. 
	 Therefore, the larger the value, the less keypoints you will get. 
	 A good default value could be from 300 to 500, depending from the image contrast.
	 
	 INT nOctaves
	 The number of a gaussian pyramid octaves that the detector uses. 
	 It is set to 4 by default. If you want to get very large features, use the larger value. 
	 If you want just small features, decrease it.
	 
	 INT nOctaveLayers
	 The number of images within each octave of a gaussian pyramid. It is set to 2 by default.
	 **/
    float distanceThreshold;
    float hessianThreshold;
    float octaves;
    float octaveLayers;
    float minMatches;
	bool bExtended;
    bool bUpright;
    bool bContrast;
    
    // Drawing options
    bool bDrawImage;
    bool bDrawFeatures;
    bool bDrawHomography;
    bool bDrawMatches;
    bool bDrawResponses;
	bool bDrawCrossHairs;
    
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
