/* Author Mihoko Abe
 *
 *
 */
#include "testApp.h"


//--------------------------------------------------------------
void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
	
#ifdef USE_TWO_KINECTS
	kinect2.init();
	kinect2.open();
#endif
	
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
    grayThresh.allocate(kinect.width, kinect.height);
	
	nearThreshold = 225;
	farThreshold = 205;
	bThreshWithOpenCV = true;
    videoMode = 0;
	
	ofSetFrameRate(60);
    
    isFullScreen = false;
    drag = false;
    drumsDrag =false;
    rhoseDrag = false;
    volumeFlag = false;
    
    	
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	// start from the front
	bDrawPointCloud = false;

    ofSetCircleResolution(32);
    ofSetVerticalSync(true);
    ofEnableBlendMode(OF_BLENDMODE_ADD);
    synthImage.loadImage("particle13.png");
    
    //setup sounds
    
    nBandsToGet = 1024;
    mySynth.loadSound("sounds/drone-01.wav");
    mySynth.setLoop(true);
    mySynth.setVolume(0.75f);
    mySynth.setMultiPlay(true);
    
    
    myDrums.loadSound("sounds/Fastarp.wav");
    myDrums.setLoop(true);
    myDrums.setVolume(0.75f);
    myDrums.setMultiPlay(true);
    
    myRhodes.loadSound("sounds/drone.wav");
    myRhodes.setLoop(true);
    myRhodes.setVolume(1.0f);
    myRhodes.setMultiPlay(true);
    
    
    myFill1.loadSound("sounds/crazy.mp3");
    myFill1.setLoop(false);
    myFill1.setVolume(0.50f);
    myFill1.setMultiPlay(true);
    
    mySynthStab.loadSound("sounds/AFS.mp3");
    mySynthStab.setLoop(false);
    mySynthStab.setVolume(0.75f);
    mySynthStab.setMultiPlay(true);
    
    
    mySax.loadSound("sounds/saxMel.mp3");
    mySax.setLoop(false);
    mySax.setVolume(0.80f);
    mySax.setMultiPlay(true);
    
    mySax2.loadSound("sounds/saxMel.mp3");
    mySax2.setLoop(false);
    mySax2.setVolume(0.80f);
    mySax2.setMultiPlay(true);

    mySax3.loadSound("sounds/saxMel.mp3");
    mySax3.setLoop(false);
    mySax3.setVolume(0.80f);
    mySax3.setMultiPlay(true);

    
    
  // mySynthStab.play();
    screenWidth = ofGetWidth();
    screenHeight = ofGetHeight();

   // makeFloatingWindow();
    
    int bufferSize		= 512;
	sampleRate 			= 44100;
	phase 				= 0;
	phaseAdder 			= 0.0f;
	phaseAdderTarget 	= 0.0f;
	volume				= 0.1f;

  //volumes
    
    synthVolume = screenWidth/4;
    synthPosition = screenWidth/4;
    topPadding = 75;
    leftPadding = 75;
    
  //drums Volume
    drumsVolume = screenWidth/4;
    drumsPosition = screenWidth/4;
    
  //rhose volume
    rhodesVolume = screenWidth/4;
    rhodesPosition = screenWidth/4;
   

 
}

//--------------------------------------------------------------
void testApp::update() {
	
	//ofBackground(100, 100, 100);
    ofBackground(0,0,0);
	
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		
		// load grayscale depth image from the kinect source
		
        grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
		grayImage.mirror(false, true);
		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		if(bThreshWithOpenCV) {
			grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		} else {
			
			// or we do it ourselves - show people how they can work with the pixels
			unsigned char * pix = grayImage.getPixels();
			
			int numPixels = grayImage.getWidth() * grayImage.getHeight();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
		}
        for (depth = 255; depth > 220; depth -= 5) {
            grayThreshFar = grayImage;
            grayThreshNear = grayImage;
            grayThreshNear.threshold(depth, true);
            grayThreshFar.threshold(depth-15);
            cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayThresh.getCvImage(), NULL);
            grayThresh.flagImageChanged();
            contourFinder.findContours(grayThresh, 1000, (kinect.width*kinect.height)/20, 10, false);
            
            if (contourFinder.nBlobs) {
                break;
            }
        }
        		
		// update the cv images
		grayImage.flagImageChanged();
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
	}
    for(deque <MyCircle *>::iterator it = circles.begin(); it != circles.end();){
        (*it)->update();
        if((*it)->dead){
            if((*it)->dead){
                delete *it;
                it = circles.erase(it);
                
            }else{
                ++it;
            }
        }
    }
    
    //particle2
    for(int i= 0; i <100 ; i++){
        p[i].resetForce();
        p[i].addForce(0,0.1);
        p[i].addDampingForce();
        p[i].update();
    }
    synthImage.update();
    
	
#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif
   
    
      
    fft = ofSoundGetSpectrum(nBandsToGet);
     ofSoundUpdate();
    
    
}

//--------------------------------------------------------------
void testApp::draw() {
    
    if(isFullScreen){
        flexWidth = ofGetScreenWidth();
        flexHeight = ofGetScreenHeight();
    }else{
        flexWidth = screenWidth;
        flexHeight = screenHeight;
    }

	
	ofSetColor(255, 255, 255);
	
	if(bDrawPointCloud) {
		easyCam.begin();
		drawPointCloud();
		easyCam.end();
	} else {
        
		grayImage.draw(-100,-70,flexWidth,flexHeight);
		contourFinder.draw(-100,-70,flexWidth,flexHeight);

               
#ifdef USE_TWO_KINECTS
		kinect2.draw(420, 320, 400, 300);
#endif
	}
	
	    ofSetColor(255,255,255);
    
        
    //set width and height up to screen mode
   
    for(int i=0;i <100;i++){
             p[i].draw();
        
    }

    //mySynth
    ofCircle(synthPosition,flexHeight/4, 50);
    synthImage.draw(synthPosition-60,flexHeight/4-2);
    ofLine(100,flexHeight/4,synthPosition,flexHeight/4);
    ofDrawBitmapString("Drone01",20,flexHeight/4);
    int preSynthPosition = synthPosition;
    mySynth.setVolume(((float)synthVolume-100)/((float)flexWidth/4.0f-100));
    
    
    //myDrum
    ofCircle(drumsPosition, flexHeight/16*7, 50);
    synthImage.draw(drumsPosition-60,flexHeight/16*7-2);
     ofLine(100,flexHeight/16*7,drumsPosition,flexHeight/16*7);
    ofDrawBitmapString("Drone02",20,flexHeight/16*7);
    int preDrumsPosition = drumsPosition;
    myDrums.setVolume(((float)drumsVolume-100)/((float)flexWidth/4.0f-100));

    
    //myRhose
    ofCircle(rhodesPosition, flexHeight/8*5, 50);// 10/16
    synthImage.draw(rhodesPosition-60, flexHeight/8*5-2);
    ofLine(100, flexHeight/8*5, rhodesPosition,flexHeight/8*5);
    ofDrawBitmapString("Drone03",20, flexHeight/8*5);
    int preRhosePosition = rhodesPosition;
    myRhodes.setVolume(((float)rhodesVolume-100)/((float)flexWidth/4.0f-100));
    
    //my fill1
    ofCircle(flexWidth/4*3, flexHeight/4, 50);
    //my fill2
    ofCircle(flexWidth/8*7, flexHeight/8*3, 50);
   
    //myfill3

    ofRect(flexWidth-300, flexHeight/2, 200, flexHeight/2-75);
    ofDrawBitmapString("Sound Pad", flexWidth-280,flexHeight/2+10);
    
    
  
    //add particle && press the circle
    for (int i = 0; i < contourFinder.blobs.size(); i++){
        circles.push_back(new MyCircle(ofPoint(contourFinder.blobs[i].centroid.x, contourFinder.blobs[i].centroid.y), ofRandom(1, 8), 0.4, 0.1, 12.0));
        
        ofxCvBlob myBlob = contourFinder.blobs[i];
        float ratio=myBlob.boundingRect.height/myBlob.boundingRect.width;
        float density = myBlob.area/myBlob.boundingRect.width/myBlob.boundingRect.height;
        
            
        //mySynth - volume
        
              
    
        
        if(myBlob.centroid.x >preSynthPosition-120 && myBlob.centroid.x <preSynthPosition-75
           && myBlob.centroid.y>flexHeight/4-75 && myBlob.centroid.y <flexHeight/4-50)
        {
            if(density>0.65){
            synthPosition = myBlob.centroid.x+100;
            synthVolume = myBlob.centroid.x*screenWidth/4/(screenWidth/4-100);
            
                if(synthPosition<125){
                synthPosition =125;
                synthVolume=100;//100 means position 0 for blob
                
                }else if(synthPosition > flexWidth/4){
                synthPosition=flexWidth/4;
                synthVolume = flexWidth/4;
                }
            }

            if(!drag){
                if(!mySynth.getIsPlaying() && density<0.45&&
                   myBlob.centroid.x >preSynthPosition-100 && myBlob.centroid.x <preSynthPosition-80
            
                   )
                    mySynth.play();
                    
                else if(mySynth.getIsPlaying() && density>0.65)
                
                    drag =true;
                
            }else{
                //when dragging
                if(mySynth.getIsPlaying() && density<0.45)
                
                    drag = false;
                    
                else if(mySynth.getIsPlaying() && density >0.65){
                    
                    
                }
            }//the end of dragging
            
        }//the end of blobo
        
        if(myBlob.centroid.x >preDrumsPosition-120 && myBlob.centroid.x <preDrumsPosition-75
           && myBlob.centroid.y>flexHeight/16*7-150 && myBlob.centroid.y <flexHeight/16*7-120)
        {
            if(density>0.65){
                drumsPosition = myBlob.centroid.x+100;
                drumsVolume = myBlob.centroid.x*flexWidth/4/(flexWidth/4-100);
                
                if(drumsPosition<125){
                    drumsPosition =125;
                    drumsVolume=100;//100 means position 0 for blob
                    
                }
                else if(drumsPosition > flexWidth/4){
                    drumsPosition=flexWidth/4;
                    drumsVolume = flexWidth/4;
                }
            }
            
            if(!drumsDrag){
                if(!myDrums.getIsPlaying() && density<0.45 &&
                   myBlob.centroid.x >preDrumsPosition-100 && myBlob.centroid.x <preDrumsPosition-80
                   )
                    myDrums.play();
               
               
                else if(myDrums.getIsPlaying() && density>0.65)
                                       
                    drumsDrag =true;
               
                
            }else{
                //when dragging
                if(
                   //myDrums.getIsPlaying() &&
                   density<0.45)
                   
                    drumsDrag = false;
                    
               
                
            }
            
            
        }


        
   //Rhose
      // if(!isFullScreen){
            
        
        if(myBlob.centroid.x >preRhosePosition-120 && myBlob.centroid.x <preRhosePosition-75
           && myBlob.centroid.y>flexHeight/16*10-250
           && myBlob.centroid.y <flexHeight/16*10-225
           )
        {
            if(density>0.65){
                rhodesPosition = myBlob.centroid.x+100;
                rhodesVolume = myBlob.centroid.x*flexWidth/4/(flexWidth/4-100);
                
                if(rhodesPosition<125){
                    rhodesPosition =125;
                    rhodesVolume=100;//100 means position 0 for blob
                    
                }
                else if(rhodesPosition > flexWidth/4){
                    rhodesPosition=flexWidth/4;
                    rhodesVolume = flexWidth/4;
                }
            }//density
            
            if(!rhoseDrag){
                if(!myRhodes.getIsPlaying() && density<0.45 &&
                   myBlob.centroid.x >preRhosePosition-100 && myBlob.centroid.x <preRhosePosition-80)
                    
                    myRhodes.play();
                
                else if(myRhodes.getIsPlaying() && density>0.65)
                  
                    
                    rhoseDrag =true;
                    
                
                
            }else{
                //when dragging
                if(myRhodes.getIsPlaying() && density<0.45)                
                    rhoseDrag = false;
                    
            
            }//!drag
       
        }//blob
    
    

        if(!isFullScreen){
            
        
        if(myBlob.centroid.x >flexWidth/4*3-400 && myBlob.centroid.x < flexWidth/4*3-375
           && myBlob.centroid.y>flexHeight/4-75 && myBlob.centroid.y <flexHeight/4-50)
        {
            
            
            if(!myFill1.getIsPlaying() && density<0.45){
                
            
              myFill1.play();
                for(int i=0;i <50;i++){
                    p[i].setInitialCondition(flexWidth/4*3, flexHeight/4-100, ofRandom(-10,10), ofRandom((-10,10)));
                 //   p[i].draw();
                    
                }// end of for


            }   //end of density
            
        }//end of blob
        }else{//full screen
            
            if(myBlob.centroid.x >flexWidth/4+135
               && myBlob.centroid.x < flexWidth/4+165
              && myBlob.centroid.y> flexHeight/4-105 && myBlob.centroid.y < flexHeight/4-75
               )
            {
                
                
                if(!myFill1.getIsPlaying() && density<0.45){
                    
                    
                    myFill1.play();
                    for(int i=0;i <50;i++){
                        p[i].setInitialCondition(flexWidth/4*3, flexHeight/4-100, ofRandom(-10,10), ofRandom((-10,10)));
                        //   p[i].draw();
                        
                    }// end of for
                    
                    
                }//density
            }//blob

            
    }//screen
        if(!isFullScreen){
        if(myBlob.centroid.x>flexWidth/8*7-475 && myBlob.centroid.x < flexWidth/8*7-450
           && myBlob.centroid.y> flexHeight/8*3-125 && myBlob.centroid.y < flexHeight/8*3-75){
            
            
            if(!mySynthStab.getIsPlaying() && density<0.45)
                mySynthStab.play();

        
                   
        }//blob
        }else{//not full screen
            if(
               myBlob.centroid.x>flexWidth/8*7-690 && myBlob.centroid.x < flexWidth/8*7-660
               && myBlob.centroid.y> flexHeight/8*2-25 && myBlob.centroid.y < flexHeight/8*2+25
               ){
                
    
                if(!mySynthStab.getIsPlaying() && density<0.45)
                    mySynthStab.play();

        }
        }//not fullscreen
        
        //sax pad
        if(myBlob.centroid.x >flexWidth/4+165
           && myBlob.centroid.y> flexHeight/4+100
           ){
            
           if(!mySax.getIsPlaying())
                mySax.play();
            if(!mySax2.getIsPlaying())
                mySax2.play();
            
            if(!mySax3.getIsPlaying())
                mySax3.play();


            
            
            mySax.setSpeed( (1.0f-((myBlob.centroid.y-((float)flexHeight/4+100)) /((float)flexHeight/2) *10)));
            mySax2.setSpeed( (((myBlob.centroid.y-((float)flexHeight/4+100)) /((float)flexHeight/2) *10)));
            mySax3.setSpeed( (1.0f-((myBlob.centroid.y-((float)flexHeight/4+100)) /((float)flexHeight/2) *10)));


            
        }
    
    }
    
    
    //draw particle
    for(deque <MyCircle *>::iterator it = circles.begin(); it != circles.end(); ++it){
        (*it)->draw();
    }
    glPopMatrix();
    
    //audio
  // if(isFullScreen){
    
    float width = float(flexWidth)/float(nBandsToGet)/2.0f;
    for(int i = 0;i<nBandsToGet;i++){
        int b = float(255)/float(nBandsToGet)*i;
        int g = 31;
        int r = 255-b;
        ofSetColor(r,g,b);
    
        ofCircle(flexWidth/2 + width*i,flexHeight/2, fft[i]*800);
        
        ofCircle(flexWidth/2-width *i,
                 flexHeight/2, fft[i]*800);
        //synth
        if(mySynth.getIsPlaying()){
        int r1 = float(255)/float(nBandsToGet)*i;
        int g1 = 200;
        int b1 = 255-r;
        ofSetColor(r1,g1,b1);
        
        
        ofCircle(preSynthPosition,
                 flexHeight/4, fft[i]*100);
        }//the end of for
        //drums
        if(myDrums.getIsPlaying()){
            int g2 = 147;
            int b2 = 215;
            int r2 = 249;
            ofSetColor(r2,g2,b2);
            
            ofCircle(preDrumsPosition ,flexHeight/16*7, fft[i]*300);
            
        }
        //Rhodes
        if(myRhodes.getIsPlaying()){
            int g2 = 243;
            int b2 = 250;
            int r2 = 181;
            ofSetColor(r2,g2,b2);
            
            ofCircle(preRhosePosition ,flexHeight/16*10, fft[i]*300);
            
            }
        
        if(myFill1.getIsPlaying()){
            int r3 = 253;
            int g3 = 240;
            int b3 = 62;
            ofSetColor(r3,g3,b3);
            
            ofCircle(flexWidth/4*3, flexHeight/4, fft[i]*600);
           
        }
        if(mySynthStab.getIsPlaying()){
            int r3 = 238;
            int g3 = 117;
            int b3 = 42;
            ofSetColor(r3,g3,b3);
            
            ofCircle(flexWidth/8*7, flexHeight/8*3, fft[i]*500);
            
        }



    
    }
    ofNoFill();
	ofPushStyle();
    ofPushMatrix();

    ofPopMatrix();
	ofPopStyle();
    
    
}


void testApp::drawPointCloud() {
	//int w = 640;
	//int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 2;
	for(int y = 0; y < screenHeight; y += step) {
		for(int x = 0; x < screenWidth; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
                mesh.addColor(kinect.getColorAt(x,y));
               
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	glPointSize(3);
    
	//ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	glEnable(GL_DEPTH_TEST);
	mesh.drawVertices();
    //contourFinder.draw();
	glDisable(GL_DEPTH_TEST);
	ofPopMatrix();
    

   

}
void testApp::audioOut(float * output, int bufferSize, int nChannels){
	//pan = 0.5f;
	float leftScale = 1 - pan;
	float rightScale = pan;
    
	// sin (n) seems to have trouble when n is very large, so we
	// keep phase in the range of 0-TWO_PI like this:
	while (phase > TWO_PI){
		phase -= TWO_PI;
	}
    
	if ( bNoise == true){
		// ---------------------- noise --------------
		for (int i = 0; i < bufferSize; i++){
			lAudio[i] = output[i*nChannels    ] = ofRandom(0, 1) * volume * leftScale;
			rAudio[i] = output[i*nChannels + 1] = ofRandom(0, 1) * volume * rightScale;
		}
	} else {
		phaseAdder = 0.95f * phaseAdder + 0.05f * phaseAdderTarget;
		for (int i = 0; i < bufferSize; i++){
			phase += phaseAdder;
			float sample = sin(phase);
			lAudio[i] = output[i*nChannels    ] = sample * volume * leftScale;
			rAudio[i] = output[i*nChannels + 1] = sample * volume * rightScale;
		}
	}
    
}//--------------------------------------------------------------
void testApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	
#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;
			
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
			
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
			
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
            
        case 'f':
            //make screen full size
            
            ofToggleFullscreen();
            if(isFullScreen)
                isFullScreen = false;
            else
                isFullScreen = true;
            
            break;
        case 's':
            //make screen full size
            mySynth.stop();
           
            break;
        case 'd':
            //make screen full size
            myDrums.stop();
            
            break;
        case 'r':
            //make screen full size
            myRhodes.stop();
            
            break;



			
		case '1':
			kinect.setLed(ofxKinect::LED_GREEN);
			break;
			
		case '2':
			kinect.setLed(ofxKinect::LED_YELLOW);
			break;
			
		case '3':
			kinect.setLed(ofxKinect::LED_RED);
			break;
			
		case '4':
			kinect.setLed(ofxKinect::LED_BLINK_GREEN);
			break;
			
		case '5':
			kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
			break;
			
		case '0':
			kinect.setLed(ofxKinect::LED_OFF);
			break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
	}
}

void testApp::mouseMoved(int x, int y)
{
    int width = ofGetWidth();
	pan = (float)x / (float)width;
	float height = (float)ofGetHeight();
	float heightPct = ((height-y) / height);
	targetFrequency = 2000.0f * heightPct;
	phaseAdderTarget = (targetFrequency / (float) sampleRate) * TWO_PI;

}
//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{
    
   }

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}
