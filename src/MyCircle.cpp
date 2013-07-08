#include "MyCircle.h"

//コンストラクタ
MyCircle::MyCircle(ofPoint _pos, float _radius, float _maxSpeed, float _phaseSpeed, float _lifeLength) 
{
    pos = _pos;
    radius = _radius;
    phaseSpeed = _phaseSpeed;
    maxSpeed = _maxSpeed;
    lifeLength = _lifeLength;
    
    //スピードを設定
    speed.x = ofRandom(-maxSpeed, maxSpeed);
    speed.y = ofRandom(-maxSpeed, maxSpeed);
    //初期位相
    phase = ofRandom(0, PI*2);
    //生死
    dead = false;
}

//デストラクタ
MyCircle::~MyCircle()
{

}

void MyCircle::update()
{
    //円の半径の伸び縮みの位相を更新
    phase += phaseSpeed;
    if (phase > lifeLength) {
        dead = true;
    }
    //位置を更新
    pos += speed;
}

void MyCircle::draw()
{
    float r = sin(phase)*radius;

    ofSetColor(127, 255, 255, 11); 
    ofCircle(pos.x, pos.y, radius*2.0);
    
    ofSetColor(31, 127, 255, 127); 
    ofCircle(pos.x, pos.y, r);

    ofSetColor(255, 255, 255); 
    ofCircle(pos.x, pos.y, r*0.25);
}

void MyCircle::setPos(ofPoint _pos)
{
    pos = _pos;
}

void MyCircle::setRadius(float _radius)
{
    radius = _radius;
}