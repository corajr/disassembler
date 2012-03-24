//
//  RGBPoint.cpp
//  openNiSample007
//
//  Created by Christopher Johnson-Roberson on 3/20/12.
//  Copyright 2012 __MyCompanyName__. All rights reserved.
//

#include "RGBPoint.h"

RGBPoint::RGBPoint(float _x, float _y, float _z, float _r, float _g, float _b) {
    x = originalX = _x;
    y = originalY = _y;
    z = originalZ = _z;
    color = ofColor(_r,_g,_b);
    velocity = ofVec3f(0,0,0);
}

RGBPoint::RGBPoint (ofPoint pos) {
    x = originalX = pos.x;
    y = originalY = pos.y;
    z = originalZ = pos.z;
    color =  ofColor(255.0, 255.0, 255.0);
    velocity = ofVec3f(0,0,0);
}