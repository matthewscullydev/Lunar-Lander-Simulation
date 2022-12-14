
//--------------------------------------------------------------
// Matthew Scully
// 
// Final Project Lunar Landing Simulation - CS134
// 
//
//  Student Name:   < Matthew Scully >
//  Date: <12/8/22>

#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include  "ofxAssimpModelLoader.h"
#include "Octree.h"
#include <glm/gtx/intersect.hpp>
#include "lander.h"	
#include "ParticleEmitter.h"
#include "TransformObject.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
		
		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent2(ofDragInfo dragInfo);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		void drawAxis(ofVec3f);
		void initLightingAndMaterials();
		void savePicture();
		void toggleWireframeMode();
		void togglePointsDisplay();
		void toggleSelectTerrain();
		void setCameraTarget();
		bool mouseIntersectPlane(ofVec3f planePoint, ofVec3f planeNorm, ofVec3f &point);
		bool raySelectWithOctree(ofVec3f &pointRet);
		bool rayIntersect(ofVec3f* pointRet);
		void loadVbo();

		//bool explode = false;

		glm::vec3 ofApp::getMousePointOnPlane(glm::vec3 p , glm::vec3 n);

		//easy cam that can target lander with t
		ofEasyCam cam;

		//easy cam that is spawned at position of lander and faces down.
		ofEasyCam landercam;

		//easy cam that targets the node in the lander and looks at it when selected.
		ofEasyCam targetcm;
		

		//load sounds
		ofSoundPlayer shipsound;

		ofSoundPlayer takeoff;

		ofSoundPlayer explosion;

		ofSoundPlayer thrustsound;

		ofSoundPlayer houston;

		ofSoundPlayer eagle;

		//thruster sound booleans
		bool keyHeld = false;
		bool playSound = false;
		bool playSound2 = true;

		//initialize ray vector

		ofVec3f returnval = ofVec3f(0,0,0);

		Vector3 rayp;
		Vector3 rayd = Vector3(0,-1,0);
		Ray testray;
		
		//initialize lander

		ofxAssimpModelLoader mars, lander;
	
		//initialize boxes and octree for collision based detection
		Box boundingBox, landerBounds;
		Box testBox;
		vector<Box> colBoxList;
		bool bLanderSelected = false;
		Octree octree;
		Octree stupid;
		TreeNode selectedNode;
		glm::vec3 mouseDownPos, mouseLastPos;
		bool bInDrag = false;

		float startTime;
		float endTime;

		//timer floats and strings
		float startTimeKey;
		float timeUsedstart;
		float timeUsedfinish;
		float totalTime;

		string FuelT;

		float fuelLeft = 120;
		bool noFuel = false;
		bool explodedraw;

		//for octree
		ofxIntSlider numLevels;
		ofxPanel gui;

		bool bAltKeyDown;
		bool bCtrlKeyDown;
		bool bWireframe;
		bool bDisplayPoints;
		bool bPointSelected;
		bool bHide;
		bool pointSelected = false;
		bool bDisplayLeafNodes = false;
		bool bDisplayOctree = false;
		bool showBoxes = false;
		bool bDisplayBBoxes = false;
		bool collisionD = false;
		bool bLanderLoaded;
		bool bTerrainSelected;

		//altitude value
		double altitude;

		//for mouse selection
		ofVec3f selectedPoint;
		ofVec3f intersectPoint;

		//keymap for input detection
		map<int, bool> keymap;
		
		//initialize return bbox list for collision and range
		vector<Box> bboxList;
		const float selectionRange = 4.0;

		//background
		ofImage bgImage;
	
		bool explode = false;
		Lander testLander;

		//booleans and timer for bounce detection
		bool startBounce = false;
		bool changePos = false;
		bool reset = false;
		bool scoreSet = false;
		bool isInside;
		bool landed = false;

		float bounceTimer;

		//landing score
		int landingscore = 0;

		//particle systems
		
		//player system
		ParticleSystem partsys;

		//particle emitters
		ParticleEmitter emitter;
		ParticleEmitter emitter2;

		//ship particle forces
		GravityForce* grav;
		TurbulenceForce* turb;
		ThrustForce* thrust;
		ImpulseForce* impulse;

		//particle emitter 1 forces
		GravityForce* grav2;
		TurbulenceForce* turb2;

		//particle emitter 2 forces
		TurbulenceForce* turbForce;
		GravityForce* gravityForce;
		ImpulseRadialForce* radialForce;

		//textures for shaders
		ofTexture  particleTex;
		ofTexture  particleTex2;

		// shaders
		//
		ofVbo vbo;
		ofVbo vbo2;
		ofShader shader;
		ofShader shader2;

		//cameras
		bool scenarioA = false;
		bool camLand = false;
		bool camTracking = false;

		ofFbo fbo;
		
		//fuel timer
		bool takeTime;

		//lighting

		ofLight keyLight,rimLight2, fillLight, rimLight;

		ofxGuiGroup rimLightgroup;

		ofxFloatSlider rimLightAttenuation;
		ofxFloatSlider rimLightAmbientColor;
		ofxFloatSlider rimLightDiffuseColor;
		ofxFloatSlider rimLightSpecularColor;

		ofxFloatSlider rimLightRotateY;
		ofxFloatSlider rimLightRotateX;
		ofxVec3Slider rimLightPos;

		//key light sliders for testing

		ofxGuiGroup keyLightgroup;

		ofxFloatSlider keyLightAttenuation;
		ofxVec3Slider keyLightAmbientColor;
		ofxFloatSlider keyLightDiffuseColor;
		ofxFloatSlider keyLightSpecularColor;
		ofxVec2Slider keyLightArea;
		ofxVec3Slider keyLightPos;
		//fill light sliders for testing

		ofxGuiGroup fillLightgroup;

		ofxFloatSlider fillLightAttenuation;
		ofxFloatSlider fillLightAmbientColor;
		ofxFloatSlider fillLightDiffuseColor;
		ofxFloatSlider fillLightSpecularColor;

		ofxIntSlider fillLightSpot;

};
