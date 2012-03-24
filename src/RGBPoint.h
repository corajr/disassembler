//
//  RGBPoint.h
//  openNiSample007
//
//  Created by Christopher Johnson-Roberson on 3/20/12.
//  Copyright 2012 __MyCompanyName__. All rights reserved.
//

#ifndef openNiSample007_RGBPoint_h
#define openNiSample007_RGBPoint_h

#include "ofMain.h"

class RGBPoint : public ofPoint {
public:
    RGBPoint(float _x, float _y, float _z, float _r, float _g, float _b);
    RGBPoint (ofPoint pos);
    
    float originalX, originalY, originalZ;
    ofVec3f velocity;
    ofColor color;
};

#endif
