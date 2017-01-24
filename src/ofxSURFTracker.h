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
	
	// detect features in the supplied image
	void detect(ofImage& img);
    void detect(unsigned char * pix, int inputWidth, int inputHeight);
	
	// save the detected keypoints and descriptors in the "object"
    void learnFeatures();
	
	// detect match with the current scene and return the number of matches
	int match(vector<KeyPoint> keyPoints, Mat descriptors, vector <Point2f> bounds);
	
	// calculate the perspective transform and apply to bounds polygon
	void createHomography(vector<KeyPoint> keyPoints, vector <Point2f> bounds);
	
	// options
	/**  PARAMETERS *****
	 
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
	
	// access the tracked object:
	vector <KeyPoint> getObjectKeyPoints();
	vector <Point2f> getObjectBounds();
	Mat getObjectDescriptors();
	ofImage getCroppedImage();

	float getWidth();
	float getHeight();
	
	vector <ofPoint> getBounds();
	vector <ofPoint> getTransformedBounds();
	
private:
    int width, height;
    ofxCvColorImage inputImg;
    ofxCvColorImage croppedImg;
    ofxCvGrayscaleImage trackImg;
    
    // keypoints and descriptors
	
	vector<KeyPoint> keyPoints_Object;			// keypoints of the object we're tracking
	Mat descriptors_Object;						// descriptors the object we're tracking
	vector <Point2f> objectBounds;				// bounds of the original object

	
	vector<KeyPoint> keyPoints_Scene;			// keypoints in the current scene
	Mat descriptors_Scene;						// descriptors in the current scene
	
	vector< DMatch > good_Matches;				// matches between original and new descriptors
    Mat homography;								// prespective transform between original and new features
    vector <Point2f> objectBounds_Transformed;	// perspective transformed bounds
    
    SurfFeatureDetector detector;
    SurfDescriptorExtractor extractor;
    FlannBasedMatcher flannMatcher;
    
};



#endif /* defined(__ofxSURFTracker__) */
