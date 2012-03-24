#ifndef _TEST_APP
#define _TEST_APP

//#define USE_IR // Uncomment this to use infra red instead of RGB cam...

#include "ofxOpenNI.h"
#include "ofMain.h"
#include "RGBPoint.h"

class testApp : public ofBaseApp{

public:
	void setup();
	void update();
	void draw();

	void keyPressed  (int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y );
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);

	void	setupRecording(string _filename = "");

	bool				isTrackingHands, isCloud;
    

	ofxOpenNIContext	recordContext;
	ofxDepthGenerator	recordDepth;

#ifdef USE_IR
	ofxIRGenerator		recordImage;
#else
	ofxImageGenerator	recordImage;
#endif

	ofxHandGenerator	recordHandTracker;

	ofxUserGenerator	recordUser;
    ofxTrackedHand *    trackedHands[2];
    
    ofVec3f             handPositions[2], previousHandPositions[2], handVelocities[2];
    float               handRadius;
    
#if defined (TARGET_OSX) //|| defined(TARGET_LINUX) // only working on Mac/Linux at the moment (but on Linux you need to run as sudo...)
	ofxHardwareDriver	hardware;
#endif

    void				freezePointCloud();
    void                scatterPointCloud();
    void                drawPointCloud();
    
    vector<RGBPoint>    pointCloud;

	int					nearThreshold, farThreshold;
    int                 cloudWidth, cloudHeight, cloudX, cloudY, cloudZ;
	int					pointCloudRotationY;
    
    float               damping;

	float				filterFactor;
    float               reassemblyRate;

};

#endif
