
//--------------------------------------------------------------
// Matthew Scully
// 
// Final Project Lunar Landing Simulation - CS134
// 
// Lander Class for binding to particle system
//
//  Student Name:   < Matthew Scully >
//  Date: <12/8/22>

#pragma once

#ifndef test		

//#include header guard for including libraries 

#include "Octree.h"
#include "ofxAssimpModelLoader.h"
#include "box.h"
#include "ray.h"
#include "particle.h"
#include "ParticleSystem.h"
#endif // !

//#include "ofApp.h"


class Lander {
public:

	//constructor
	Lander() {

		position.set(0, 25, 0);
		velocity.set(0, 0, 0);
		acceleration.set(0, 0, 0);
		thrust.set(0, 0, 0);
		forces.set(0, 0, 0);
		rotation1 = 0;
		mass = 1;
		angularVelocity = 1;
		angularAcceleration = 1;
		angularForce = 1;
		bLanderLoaded = false;
		damping = 1;

		rayPoint = lander1.getPosition();
		rayDir = ofVec3f(0, -1, 0);
	}

	//physics which ended up being used elsewhere
	Particle p;
	ofVec3f position;
	ofVec3f velocity;
	ofVec3f acceleration;
	ofVec3f forces;
	ofVec3f thrust;
	float gravity;

	//create vector of force vectors

	//in integrate function go through vector of force vectors and set them one by one

	ofVec3f rayPoint;
	ofVec3f rayDir;
	Ray altitudeRay;

	float speed = 1.64;
	float angularVelocity;
	float angularAcceleration;
	float angularForce;
	float framerate = ofGetFrameRate();
	float dt = 1.0 / framerate;
	float	damping;
	float   mass;
	float damping2 = 0;
	float rotation1;
	float rotation2;



	//misc
	bool rotateTrue = false;
	bool headshow = true;
	bool bLanderLoaded;						//boolean variable used to check if the lander properly loaded in ofApp.cpp
	vector<Box> bboxList;
	ofxAssimpModelLoader lander1;
	Box landingBox;

	//functions
	ofVec3f headingv();
	
	void initializeLander();
	void integrate();
	void drawLander();
	void variableTest();
	bool rayIntersect(ofVec3f *pointRet,Octree *octree);

};
