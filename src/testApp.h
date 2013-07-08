#pragma once
#include <deque>
#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "MyCircle.h"
#include "Particle.h"
#include "makeFloatingWindow.h"
#include "ofxMaxim.h"


// uncomment this to read from two kinects simultaneously
//#define USE_TWO_KINECTS

class testApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
    //added
    void expose();
	
	void drawPointCloud();
	
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
    void mouseMoved(int x, int y);
	void windowResized(int w, int h);
	
	ofxKinect kinect;
    
  
    Particle p[100];
    ofImage img;
	
#ifdef USE_TWO_KINECTS
	ofxKinect kinect2;
#endif
	
	ofxCvColorImage colorImg;
	
	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
    
    //added
    ofxCvGrayscaleImage grayThresh;
	
	ofxCvContourFinder contourFinder;
	
	bool bThreshWithOpenCV;
	bool bDrawPointCloud;
    //added
  
    bool drag;
    bool drumsDrag;
    bool rhoseDrag;
    bool isFullScreen;
	
	int nearThreshold;
	int farThreshold;
	
	int angle;
	
	// used for viewing the point cloud
	ofEasyCam easyCam;
    
    //added
    int screenWidth;
    int screenHeight;
    int flexWidth;
    int flexHeight;
    int leftPadding;
    int topPadding;
    ofxCvBlob blob;
   
    int exposed;
    deque<CGPoint> trail;
    deque<int> depths;
    int depth;
    deque<MyCircle*> circles;
    CGPoint point;
    int videoMode;
    
    //for sound player
    ofSoundPlayer mySynth;;
    ofSoundPlayer mySynthStab;
    ofSoundPlayer myDrums;
    ofSoundPlayer myFill1;
    ofSoundPlayer myRhodes;
    ofSoundPlayer mySax;
    ofSoundPlayer mySax2;
    ofSoundPlayer mySax3;
    float* fft;
    int nBandsToGet;
    
    
    ofImage synthImage;
    //volume controler
    bool volumeFlag;
    //for synth
    
    int synthVolume;
    int synthPosition;
    
    //drums
    int drumsVolume;
    int drumsPosition;
    //Rhodes
    int rhodesVolume;
    int rhodesPosition;
    
    
    void audioOut(float * input, int bufferSize, int nChannels);
  
       
    int initialBufferSize;
    int mode;
    double wave, sample, output[2];
    ofxMaxiMix mymix;
      
    
    ofSoundStream soundStream;
    float pan;
    int sampleRate;
    bool bNoise;
    float volume;
    vector<float> lAudio;
    vector<float> rAudio;
    
    float 	targetFrequency;
    float 	phase;
    float 	phaseAdder;
    float 	phaseAdderTarget;
   
};
