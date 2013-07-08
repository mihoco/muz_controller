//
//  Particle.cpp
//  ofxKinectExample
//
//  Created by Mihoko Abe on 1/02/13.
//
//

#include "Particle.h"

//initialize constractor
Particle::Particle(){
    setInitialCondition(0, 0, 0, 0);
    damping = 0.01f;
}

//reset acceleration
void Particle::resetForce(){
    frc.set(0,0);
}
// add force
void Particle::addForce(float x, float y){
    frc.x = frc.x+x;
    frc.y =frc.y +y;
}
//calculate resistance
void Particle::addDampingForce(){
    frc.x=frc.x - vel.x * damping;
    frc.y = frc.y - vel.y * damping;
}
//set initial state
void Particle::setInitialCondition(float px, float py, float vx, float vy)
{
    pos.set(px,py);
    vel.set(vx,vy);
}
//renew
void Particle::update(){
    vel = vel + frc;
    pos = pos + vel;
}
//draw
void Particle::draw(){
    ofCircle(pos.x, pos.y, 1);
}
