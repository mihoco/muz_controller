//
//  Particle.h
//  ofxKinectExample
//
//  Created by Mihoko Abe on 1/02/13.
//
//

#ifndef ofxKinectExample_Particle_h
#define ofxKinectExample_Particle_h

#pragma once
#include "ofMain.h"

class Particle{
public:
    
    ofVec2f pos; //position
    ofVec2f vel; //velocity
    ofVec2f frc;//acceleration
    float damping;//friction
    
    Particle();
    virtual ~Particle(){};
    void resetForce();
    void addForce(float x, float y);
    void addDampingForce();
    void setInitialCondition(float px, float py, float vx, float vy);
    void update();
    void draw();
    
protected:
private:
    
};

#endif
