
//--------------------------------------------------------------
// Matthew Scully
// 
// Final Project Lunar Landing Simulation - CS134
// 
// Lander Class for binding to particle system
//
//  Student Name:   < Matthew Scully >
//  Date: <12/8/22>

#include "lander.h"
// initialize lander on screen

void Lander::initializeLander() {
	//set particle for lander's initial position
	p.position = ofVec3f(0, 50, 0);

	//load lander and assign to particle
	lander1.loadModel("geo/lander.obj");
	cout << "loaded" << endl;
	lander1.setScaleNormalization(false);
	
	lander1.setPosition(p.position.x, p.position.y, p.position.z);

	bLanderLoaded = true;
	for (int i = 0; i < lander1.getMeshCount(); i++) {
		bboxList.push_back(Octree::meshBounds(lander1.getMesh(i)));
	}
	glm::vec3 min = lander1.getSceneMin();
	glm::vec3 max = lander1.getSceneMax();

	//create bounding box
	landingBox = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));


	cout << "Mesh Count: " << lander1.getMeshCount() << endl;
	return;
};

void Lander::drawLander() {
	if (bLanderLoaded) {
		lander1.drawFaces();
		//p.draw();
		ofSetColor(ofColor::green);
		if (headshow) {
			//function for drawing heading vector;
			//ofDrawLine(lander1.getPosition(), lander1.getPosition() + headingv() * 25);
		}

		//for drawing altitude ray approximation
		//ofDrawLine(rayPoint, rayPoint + rayDir * 100);

	}
}


void Lander::integrate() {

	//update bouncing box every update step
	ofVec3f min = lander1.getSceneMin() + lander1.getPosition();
	ofVec3f max = lander1.getSceneMax() + lander1.getPosition();

	landingBox = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
	//forces.set(0, 0, 0);
	
}


// integrate lander across screen
ofVec3f Lander::headingv() {

	float num = ofDegToRad(rotation1);

	//for yaw of directionvec

	glm::vec3 directionvec = glm::vec3(cos(num)+sin(num), 0, -sin(num)+cos(num));

	glm::vec3 normalizedvec = glm::normalize(directionvec);

	return normalizedvec;
}

//tester function
void Lander::variableTest() {
	
	cout << forces.x << forces.y << forces.z << endl;
	cout << position.x << position.y << position.z << endl;
}


bool Lander::rayIntersect(ofVec3f *pointRet,Octree *octree) {
	
		rayDir.normalize();

		TreeNode selectedNode;
		bool pointSelected = (* octree).intersect(altitudeRay, (*octree).root, selectedNode);
		
		if (pointSelected) {
			*pointRet = (*octree).mesh.getVertex(selectedNode.points[0]);
		}
		return pointSelected;

}