#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup() {

	isTrackingHands	= true;
	isCloud			= false;

	nearThreshold = 500;
	farThreshold  = 1000;

	filterFactor = 0.1f;
    
    reassemblyRate = 0.05f;     // how quickly should particles return to starting position?
    damping = 0.1f;             // damping factor for velocities
    cloudX = 0;                 // translate cloud X
    cloudY = -500;               // translate cloud Y
    cloudZ = 0;                 // translate cloud Z
    pointCloudRotationY = 193;  // starting rotation; limit between 160-226

	setupRecording();

	ofBackground(0, 0, 0);
    
    for (int i = 0; i < 2; i++) {
        previousHandPositions[i] = ofVec3f(0,0,0);
        handPositions[i] = ofVec3f(0,0,0);
        handVelocities[i] = ofVec3f(0,0,0);
        trackedHands[i] = NULL;
    }

    handRadius = 50.0;
    
}

void testApp::setupRecording(string _filename) {

#if defined (TARGET_OSX) //|| defined(TARGET_LINUX) // only working on Mac/Linux at the moment (but on Linux you need to run as sudo...)
	hardware.setup();				// libusb direct control of motor, LED and accelerometers
	hardware.setLedOption(LED_OFF); // turn on the led just for yacks (or for live installation/performances ;-)
#endif

	recordContext.setup();	// all nodes created by code -> NOT using the xml config file at all
	//recordContext.setupUsingXMLFile();
	recordDepth.setup(&recordContext);
	recordImage.setup(&recordContext);

	recordUser.setup(&recordContext);
	recordUser.setSmoothing(filterFactor);				// built in openni skeleton smoothing...
	recordUser.setUseCloudPoints(isCloud);
	recordUser.setMaxNumberOfUsers(2);					// use this to set dynamic max number of users (NB: that a hard upper limit is defined by MAX_NUMBER_USERS in ofxUserGenerator)

	recordHandTracker.setup(&recordContext, 4);
	recordHandTracker.setSmoothing(filterFactor);		// built in openni hand track smoothing...
	recordHandTracker.setFilterFactors(filterFactor);	// custom smoothing/filtering (can also set per hand with setFilterFactor)...set them all to 0.1f to begin with

	recordContext.toggleRegisterViewport();
	recordContext.toggleMirror();
    
}

//--------------------------------------------------------------
void testApp::update(){

#ifdef TARGET_OSX // only working on Mac at the moment
	hardware.update();
#endif

    // update all nodes
    recordContext.update();
    recordDepth.update();
    recordImage.update();
    
    // update tracking/recording nodes
    if (isTrackingHands) {
        int i = 0;
        for (i = 0; i < recordHandTracker.getNumTrackedHands(); i++) {
            trackedHands[i] = recordHandTracker.getHand(i);
        }
        for (; i < 2; i++) {
            trackedHands[i] = NULL;
        }
    }
    for (int i = 0; i < 2; i++) {
        if (trackedHands[i] != NULL) {
            previousHandPositions[i] = handPositions[i];
            handPositions[i].set(trackedHands[i]->projectPos.x, trackedHands[i]->projectPos.y, trackedHands[i]->projectPos.z);
            handVelocities[i] = (handPositions[i] - previousHandPositions[i]) / ofGetLastFrameTime();
        }
        else {
            previousHandPositions[i] = NULL;
            handPositions[i] = NULL;
            handVelocities[i] = NULL;
        }
    }
    
    if (isCloud) {
        vector<RGBPoint>::iterator it;
        for(it = pointCloud.begin(); it < pointCloud.end(); it++) {
            for (int i = 0; i < 2; i++) {
                if (trackedHands[i] != NULL) {
                    if (it->distance(handPositions[i]) < handRadius) {
                        it->velocity += handVelocities[i];
                    }
                }
            }
            it->velocity *= damping;
            (*it) += it->velocity;
            if (it->x != it->originalX || it->y != it->originalY || it->z != it->originalZ) {
                it->x -= (it->x - it->originalX) * reassemblyRate;
                it->y -= (it->y - it->originalY) * reassemblyRate;
                it->z -= (it->z - it->originalZ) * reassemblyRate;
            }
            
        }
    }
}

//--------------------------------------------------------------
void testApp::draw(){

	ofSetColor(255, 255, 255);

	glPushMatrix();
	glScalef(4, 4, 4);

    
    if (isCloud) drawPointCloud();	// 0 gives you all point clouds; use userID to see point clouds for specific users
    
    if (isTrackingHands) {
        for (int i = 0; i < 2; i++) {
            if (trackedHands[i] != NULL) {
                ofEnableAlphaBlending();
                
                ofSetColor(255,255,255, 32);
                ofPushMatrix();
                ofTranslate(cloudWidth + cloudX, cloudHeight/2 + cloudY, cloudZ);
                ofRotateY(pointCloudRotationY);
                ofSphere(handPositions[i], handRadius);
                ofPopMatrix();
                ofDisableAlphaBlending();
            }
        }
        
    }

	glPopMatrix();

	ofSetColor(255, 255, 0);

	string statusHardware;

#ifdef TARGET_OSX // only working on Mac at the moment
	ofPoint statusAccelerometers = hardware.getAccelerometers();
	stringstream	statusHardwareStream;

	statusHardwareStream
	<< "ACCELEROMETERS:"
	<< " TILT: " << hardware.getTiltAngle() << "/" << hardware.tilt_angle
	<< " x - " << statusAccelerometers.x
	<< " y - " << statusAccelerometers.y
	<< " z - " << statusAccelerometers.z;

	statusHardware = statusHardwareStream.str();
#endif

	stringstream msg;

	msg
	<< "space : capture point cloud" << endl
	<< "click : scatter point cloud" << endl
	<< "- / + : nearThreshold         : " << ofToString(nearThreshold) << endl
	<< "< / > : farThreshold          : " << ofToString(farThreshold) << endl
	<< "[ / ] : damping               : " << ofToString(damping) << endl
	<< "; / ' : reassemblyRate        : " << ofToString(reassemblyRate) << endl
    << "k / l : handRadius            : " << ofToString(handRadius) << endl;

    //	<< "FPS   : " << ofToString(ofGetFrameRate()) << "  " << statusHardware << endl;

	ofDrawBitmapString(msg.str(), 20, 560);

}


void testApp::freezePointCloud() {
    const unsigned char * pDepth;
	const unsigned char * pColor;
	
    int depth = 0;
    pColor = recordImage.getPixels();

    cloudWidth = recordDepth.getWidth();
	cloudHeight = recordDepth.getHeight();
    
    pointCloud.clear();
    int step = 1, nIndex = 0;
    for(int y = 0; y < cloudHeight; y += step) {
		for(int x = 0; x < cloudWidth; x += step, nIndex += step) {
            depth = recordDepth.getPixelDepth(x,y);
            if (depth < nearThreshold) continue;
            if (depth > farThreshold) continue;
            pointCloud.push_back(RGBPoint(x, y, depth, pColor[nIndex*3], pColor[nIndex*3+1], pColor[nIndex*3+2]));
        }
    }
    isCloud = true;
	
}

void testApp::scatterPointCloud() {
    vector<RGBPoint>::iterator it;
	for(it = pointCloud.begin(); it < pointCloud.end(); it++) {
        ofVec3f randomForce = ofVec3f(ofRandom(-100,100),ofRandom(-100,100),ofRandom(-100,100));        
        (*it) += randomForce;
    }
}

void testApp::drawPointCloud() {

	glPushMatrix();
	glTranslatef(cloudWidth + cloudX, cloudHeight/2 + cloudY, cloudZ);
	ofRotateY(pointCloudRotationY);
	glBegin(GL_POINTS);
    
    vector<RGBPoint>::iterator it;
	for(it = pointCloud.begin(); it < pointCloud.end(); it++) {
        glColor4ub((unsigned char)it->color.r, (unsigned char)it->color.g, (unsigned char)it->color.b, (unsigned char)it->color.a);
        glVertex3f(it->x, it->y, it->z);
    }

	glEnd();

	glColor3f(1.0f, 1.0f, 1.0f);

	glPopMatrix();
}


//--------------------------------------------------------------
void testApp::keyPressed(int key){

	float smooth;

	switch (key) {
#ifdef TARGET_OSX // only working on Mac at the moment
		case 357: // up key
			hardware.setTiltAngle(hardware.tilt_angle++);
			break;
		case 359: // down key
			hardware.setTiltAngle(hardware.tilt_angle--);
			break;
#endif
		case ' ':
            freezePointCloud();
			break;
        case '[':
        case '{':
            damping -= 0.05;
            break;
        case ']':
        case '}':
            damping += 0.05;
            break;
        case ';':
        case ':':
            reassemblyRate -= 0.05;
            break;
        case '\'':
        case '"':
            reassemblyRate += 0.05;
            break;
        case 'k':
        case 'K':
            handRadius -= 5;
            if (handRadius <= 0) handRadius = 5;
            break;
        case 'l':
        case 'L':
            handRadius += 5;
            break;
		case '>':
		case '.':
			farThreshold += 50;
			if (farThreshold > recordDepth.getMaxDepth()) farThreshold = recordDepth.getMaxDepth();
			break;
		case '<':
		case ',':
			farThreshold -= 50;
			if (farThreshold < 0) farThreshold = 0;
			break;

		case '+':
		case '=':
			nearThreshold += 50;
			if (nearThreshold > recordDepth.getMaxDepth()) nearThreshold = recordDepth.getMaxDepth();
			break;

		case '-':
		case '_':
			nearThreshold -= 50;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
		case 'r':
			recordContext.toggleRegisterViewport();
			break;
		default:
			break;
	}
}

//--------------------------------------------------------------
void testApp::keyReleased(int key){

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){

	if (isCloud) pointCloudRotationY = ofMap(x, 0, ofGetWidth(), 160, 226);
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){
    cout << pointCloudRotationY << std::endl;
    scatterPointCloud();

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}

