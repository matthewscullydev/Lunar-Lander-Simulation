
//--------------------------------------------------------------
// Matthew Scully
// 
// Final Project Lunar Landing Simulation - CS134
// 
//
//  Student Name:   < Matthew Scully >
//  Date: <12/8/22>


#include "ofApp.h"
#include "Util.h"


//--------------------------------------------------------------
// setup scene, lighting, state and load geometry
//
void ofApp::setup(){


	//bg image load
	bgImage.load("images/stars.png");

	//load model
	mars.loadModel("geo/moon-houdini.obj");
	mars.setScaleNormalization(false);

	bWireframe = false;
	bDisplayPoints = false;
	bAltKeyDown = false;
	bCtrlKeyDown = false;
	bLanderLoaded = false;
	bTerrainSelected = true;

	//camera initialization
	targetcm.setDistance(10);
	targetcm.setNearClip(.1);
	targetcm.setFov(65.5);

	cam.setDistance(10);
	cam.setNearClip(.1);
	cam.setFov(65.5);   // approx equivalent to 28mm in 35mm format

	// texture loading
	//
	ofDisableArbTex();     // disable rectangular textures

	// load textures
	//
	if (!ofLoadImage(particleTex, "images/dot.png")) {
		cout << "Particle Texture File: images/dot.png not found" << endl;
		ofExit();
	}

	//load the shader
#ifdef TARGET_OPENGLES
	shader.load("shaders_gles/shader");
#else
	shader.load("shaders/shader");
#endif

	ofSetVerticalSync(true);
	cam.disableMouseInput();
	ofEnableSmoothing();
	ofEnableDepthTest();
	cam.enableMouseInput();

	initLightingAndMaterials();



	// create sliders for testing
	
	gui.setup();
	
	//keylight setup
	keyLight.setup();
	keyLight.enable();
	keyLight.setDiffuseColor(ofFloatColor(1, 1, 1));
	keyLight.setSpecularColor(ofFloatColor(1, 1, 1));
	keyLight.rotate(-45, ofVec3f(0, 1, 0));
	keyLight.rotate(45, ofVec3f(1, 0, 0));
	keyLight.setPosition(46, 83, 5);						//setting key light position to center of screen raised for good lighting

	// rimlight 1
	rimLight.setup();
	rimLight.enable();
	rimLight.setSpotlight();
	rimLight.setScale(.05);
	rimLight.setSpotlightCutOff(100);
	rimLight.setAttenuation(.2, .001, .001);
	rimLight.setAmbientColor(ofFloatColor(0.1, 0.1, 0.1));
	rimLight.setDiffuseColor(ofFloatColor(1, 1, 1));
	rimLight.setSpecularColor(ofFloatColor(1, 1, 1));
	rimLight.rotate(180, ofVec3f(0, 1, 0));
	rimLight.setPosition(36, 36, -234);						//setting rim light behind scene

	//rimlight 2
	rimLight2.setup();
	rimLight2.enable();
	rimLight2.setSpotlight();
	rimLight2.setScale(.05);
	rimLight2.setSpotlightCutOff(100);
	rimLight2.setAttenuation(.2, .001, .001);
	rimLight2.setAmbientColor(ofFloatColor(0.1, 0.1, 0.1));
	rimLight2.setDiffuseColor(ofFloatColor(1, 1, 1));
	rimLight2.setSpecularColor(ofFloatColor(1, 1, 1));
	rimLight2.rotate(180, ofVec3f(0, 1, 0));
	rimLight2.setPosition(-200, 36, 0);
	rimLight2.setOrientation(ofVec3f(0, -90, 0));

	//fill light setup
	fillLight.setup();
	fillLight.enable();
	fillLight.setSpotlight();
	fillLight.setScale(.05);
	fillLight.setPosition(-218, 72, 255);
	fillLight.setAttenuation(2, .001, .001);
	fillLight.setAmbientColor(ofFloatColor(0.1, 0.1, 0.1));
	fillLight.setDiffuseColor(ofFloatColor(1, 1, 1));
	fillLight.setSpecularColor(ofFloatColor(1, 1, 1));
	fillLight.setPosition(300, 36, 0);					
	fillLight.setSpotlightCutOff(400);
	fillLight.setOrientation(ofVec3f(0, 90, 0));
	
	//sound loading

	shipsound.load("shipsound.wav");
	shipsound.setVolume(0.8);

	thrustsound.load("thrustsound.wav");
	thrustsound.setVolume(0.8);

	takeoff.load("takeoff.wav");
	takeoff.setVolume(0.4);

	explosion.load("explosion.wav");
	explosion.setVolume(0.2);
	explosion.setSpeed(1.2);

	houston.load("houston.wav");
	houston.setVolume(0.8);

	eagle.load("eagle.wav");
	eagle.setVolume(1);

	//display octree levels
	gui.add(numLevels.setup("Number of Octree Levels", 1, 1, 10));

	//hide boxes or show boxs boolean
	bHide = false;

	//timer for octree setup
	startTime = ofGetElapsedTimeMillis();

	octree.create(mars.getMesh(0), 10);

	stupid.create(mars.getMesh(0), 10);

	endTime = ofGetElapsedTimeMillis();
	

	cout << "Time to create Octree: " << endTime - startTime << " millisec" << endl;

	cout << "Number of Verts: " << mars.getMesh(0).getNumVertices() << endl;

	//box for lander surface
	testBox = Box(Vector3(-25, 0, -25), Vector3(25, 0, 25));


	//load lander object 

	testLander.initializeLander();

	//initialize particle system for lander object to attach to
	partsys.add(testLander.p);
	grav = new GravityForce(ofVec3f(0, 0, 0));
	turb = new TurbulenceForce(ofVec3f(0), ofVec3f(0));
	thrust = new ThrustForce(ofVec3f(0), 0);
	impulse = new ImpulseForce(ofVec3f(0), ofVec3f(0),0.99);

	partsys.addForce(thrust);
	partsys.addForce(turb);
	partsys.addForce(grav);
	partsys.addForce(impulse);

	//initialize particle system for lander thrust emissions

	grav2 = new GravityForce(ofVec3f(0, 0, 0));
	turb2 = new TurbulenceForce(ofVec3f(0), ofVec3f(0));

	emitter.sys->addForce(turb2);
	emitter.sys->addForce(grav2);
	emitter.setRate(27.87);
	emitter.setLifespan(0.3);
	emitter.setParticleRadius(0.4);
	emitter.setVelocity(ofVec3f(0, -5, 0));

	turb2->set(ofVec3f(-100, 43, 0), ofVec3f(100, -59, 5));
	grav2->set(ofVec3f(0, -1.64, 0));

	//initialize particle system for lander explosion upon collision

	turbForce = new TurbulenceForce(ofVec3f(-400, 210, 200), ofVec3f(400, -210, -200));
	gravityForce = new GravityForce(ofVec3f(0, 8, 0));
	

	emitter2.sys->addForce(turbForce);
	emitter2.sys->addForce(gravityForce);

	emitter2.setRate(27.87);
	emitter2.setLifespan(3);
	emitter2.setParticleRadius(0.4);
	emitter2.setVelocity(ofVec3f(0, 10, 0));



}

void ofApp::loadVbo() {								//UNIMPLEMENTED shader function
	/*
	if (emitter.sys->particles.size() < 1) return;

	vector<ofVec3f> sizes;
	vector<ofVec3f> points;

	vector<ofVec3f> sizes2;
	vector<ofVec3f> points2;


	for (int i = 0; i < emitter.sys->particles.size(); i++) {
		points.push_back(emitter.sys->particles[i].position);
		sizes.push_back(ofVec3f(7));
	}

	for (int i = 0; i < emitter2.sys->particles.size(); i++) {
		points2.push_back(emitter2.sys->particles[i].position);
		sizes2.push_back(ofVec3f(9));
	}

	// upload the data to the vbo
	//
	int total = (int)points.size();
	int total2 = (int)points2.size();

	vbo.clear();
	vbo2.clear();

	vbo.setVertexData(&points[0], total, GL_STATIC_DRAW);
	vbo2.setVertexData(&points2[0], total, GL_STATIC_DRAW);

	vbo.setNormalData(&sizes[0], total, GL_STATIC_DRAW);
	vbo2.setNormalData(&sizes[0], total, GL_STATIC_DRAW);
*/
}




//--------------------------------------------------------------
// incrementally update scene (animation)
//
void ofApp::update() {

	emitter.update();
	emitter2.update();

	emitter.setPosition(testLander.lander1.getPosition());

	testLander.integrate();

	//code for determining if the landers bounce has occured for long enough
	//if it has been long enough the program is now unable to change the landers position 
	// 
	//**until movement if desired, unimplemented for project as this was not an aspect of the design)

	if (startBounce) {
		bounceTimer+=10;
		if(bounceTimer == 1000){
			changePos = true;
			shipsound.stop();
			eagle.play();
			startBounce = false;
		}
	
	}
	
	//explosion boolean detection which plays sound effects and draws explosion emitter

	if (explode) {
		explosion.play();
		houston.play();
		emitter2.start();
		explode = false;
		explodedraw = true;
	}
	
	// play ambient radio wave sounds if fuel is 25% depleted

	if (fuelLeft <= 90 && playSound2) {
		shipsound.play();
		playSound2 = false;
	}

	//set forces for particle system, gravity and turbulence are always relevant no matter what movement is input.
	if (!changePos) {
		turb->set(ofVec3f(-3, -3, 0), ofVec3f(3, 3, 0));
	}

	grav->set(ofVec3f(0, -1.64, 0));


	//update forces on particle system and integrate particle system if the lander is not selected by the mouse cursor
	if (!bLanderSelected) {
		partsys.update();
	}


	//set testLanders particle to be equal to this particle for easy reference
	testLander.p = partsys.particles.at(0);


	// THRUST SOUND

	if (playSound) {
		thrustsound.play();
		playSound = false;
	}


	//set position of lander equal to position of particle as a part of player particle system which is in movement
	if (!bLanderSelected) {
		testLander.lander1.setPosition(partsys.particles.at(0).position.x, partsys.particles.at(0).position.y, partsys.particles.at(0).position.z);
	}

	// MOVEMENT KEYMAPS [[IN THIS STEP SO WE CAN UPDATE THEM EVERY FRAME]]


	if (keymap['q']) {							//rotate heading vector left with respect to y-axis and also rotate associated model
		testLander.rotation1 += 0.64;
		testLander.lander1.setRotation(0, testLander.rotation1, 0, 1, 0);

	}
												//rotate heading vector right with respect to y-axis and also rotate associated model
	if (keymap['e']) {
		testLander.rotation1 -= 0.64;
		testLander.lander1.setRotation(0, testLander.rotation1, 0, 1, 0);
	}

	if (keymap['w']) {							//up movement : apply thrust force

		//for applying impulse force
		if (changePos) {
			changePos = !changePos;
			//scoreSet = !scoreSet;
		}
		//checking if fuel is zero, if it is we set it to zero so it will not be negative

		if (fuelLeft == 0) {
			fuelLeft == 0;
			keyHeld = true;
			noFuel = true;
			return;
		}
		//check if key is held and start particle emitter and fuel usage timer if so
		else if (!keyHeld) {
			emitter.start();
			playSound = true;
			keyHeld = true;
			timeUsedstart = ofGetElapsedTimef();
		}
		//if not out of fuel, allow the lander to move.
		if (!noFuel) {
			thrust->set(ofVec3f(0, 6, 0), .95);
		}
		

	}

	if (keymap['s']) {							//down movement : apply thrust force
		if (changePos) {
			changePos = !changePos;
		}
		if (fuelLeft == 0) {
			fuelLeft = 0;
			keyHeld = true;
			noFuel = true;
			return;
		}
		else if (!keyHeld) {
			emitter.start();
			playSound = true;
			keyHeld = true;
			timeUsedstart = ofGetElapsedTimef();
		}

		thrust->set(ofVec3f(0, -3, 0), .95);

	}

	if (keymap['d']) {							//thrust left : apply thrust force
		if (changePos) {
			changePos = !changePos;
		}
		if (fuelLeft == 0) {
			fuelLeft = 0;
			keyHeld = true;
			noFuel = true;
			return;
		}
		else if (!keyHeld) {
			emitter.start();
			playSound = true;
			keyHeld = true;
			timeUsedstart = ofGetElapsedTimef();
		}

		thrust->set(ofVec3f(4 * testLander.headingv().x, 0, 4 * testLander.headingv().z), .95);

	}

	if (keymap['a']) {							//thrust right : apply thrust force

		if (changePos) {
			changePos = !changePos;
		}
		if (fuelLeft == 0) {
			fuelLeft = 0;
			keyHeld = true;
			noFuel = true;
			return;
		}
		else if (!keyHeld) {
			emitter.start();
			playSound = true;
			keyHeld = true;
			timeUsedstart = ofGetElapsedTimef();
		}

		thrust->set(ofVec3f(-4 * testLander.headingv().x, 0, -4 * testLander.headingv().z), .95);
	}

	//for use in finding impulse force

	testLander.velocity = partsys.particles.at(0).velocity;


	//if the box collides with a surface trigger a boolean which is used to show box collisions


	if (octree.intersect(testLander.landingBox, octree.root, colBoxList)) {
		collisionD = true;
	}

	//if there is a collision...

	if (colBoxList.size() > 0) {

		//if it overlaps the landing pad...
		if (testBox.overlap(testLander.landingBox)) {

			isInside = true;

			if (testLander.velocity.y > -1 && !scoreSet) {
				landingscore = 2;
				scoreSet = true;
			}

			else if (testLander.velocity.y < -1.5) {

				//hardlanding score
				if (!scoreSet) {
					landingscore = 1;
					if (testLander.velocity.y < -2.8) {
						landingscore = 0;
					}
					scoreSet = true;
				}

			}
		}

		//if the y velocity is too fast launch the lander high up.

		if (testLander.velocity.y < -2.8) {

			testLander.velocity.y = 0;
			emitter2.setPosition(testLander.lander1.getPosition());
			explode = true;

			thrust->set(ofVec3f(0, 8000, 0), 0.95);
			
			//if the score isnt set, set the landerscore

			if (!scoreSet) {

				//zero score for failure
				landingscore = 0;
				scoreSet = true;
			}
			
		}

		else if(isInside) {

			thrust->set(ofVec3f(0, 6, 0), 0.95);

			if (testLander.velocity.y < 0.4 && testLander.velocity.y > 0) {
				if (!changePos) {
					bounceTimer = 0;
				}


				//otherwise softly bounce on surface
				thrust->set(ofVec3f(0, 2, 0), 0.95);
				impulse->set(ofVec3f(0, 0, 0), ofVec3f(0, 1, 0), 0.95);
				grav->set(ofVec3f(0, 0, 0));
				
				fuelLeft = 0;
				startBounce = true;
				ofResetElapsedTimeCounter();

				if (changePos) {
					thrust->set(ofVec3f(0, 0, 0), 0.95);
					turb->set(ofVec3f(0, 0, 0), ofVec3f(0, 0, 0));
					impulse->set(ofVec3f(0, 0, 0), ofVec3f(0, 0, 0), 0.95);
					emitter.stop();
				}
				

				if (!scoreSet) {
					landingscore = 2;
					scoreSet = true;
				}
			}
		}

		else {
			//if outside the landing pad... (will always crash according to rubric)
			testLander.velocity.y = 0;
			emitter2.setPosition(testLander.lander1.getPosition());
			explode = true;
			//apply catapulting force
			thrust->set(ofVec3f(0, 8000, 0), 0.95);
			//zero score for failure
			if (!scoreSet) {
				emitter2.setPosition(testLander.lander1.getPosition());
				explode = true;
				landingscore = 0;
				scoreSet = true;
			}
		}

	}

	//update ray emitted from lander for altitude detection

	testLander.rayPoint = testLander.lander1.getPosition();

	rayp = Vector3(testLander.lander1.getPosition().x, testLander.lander1.getPosition().y, testLander.lander1.getPosition().z);

	testray = Ray(rayp, rayd);

	rayIntersect(&returnval);
	ofVec3f distancev = testLander.rayPoint - returnval;

	altitude = sqrt(distancev.x * distancev.x + distancev.y * distancev.y + distancev.z * distancev.z);

	
}
//--------------------------------------------------------------
void ofApp::draw() {
	loadVbo();

	//draw background
	ofBackground(ofColor::black);
	glDepthMask(false);
	bgImage.draw(0, 0, ofGetWindowWidth(), ofGetWindowHeight());

	//if (!bHide) gui.draw();
	glDepthMask(true);
	
	//start camera 
	
	cam.begin();

	ofPushMatrix();


	//draw lighting for debug purposes

	/*
	rimLight2.draw();
	keyLight.draw();
	fillLight.draw();
	rimLight.draw();
	*/


	//three cameras including the free cam. 4 views.

	//test if camera is in certain veiws for different perspectives

	if (camLand == true) {
		cam.end();

		landercam.setPosition(testLander.lander1.getPosition());
		landercam.lookAt(testLander.lander1.getPosition()-ofVec3f(0,200,0));
		landercam.draw();
		landercam.begin();

	}

	if (scenarioA) {
		cam.end();
		landercam.setPosition(0, 50, 75);
		landercam.lookAt(testLander.lander1.getPosition());
		landercam.draw();
		landercam.begin();

		//set original camera to look back at ship when key is released
		cam.setPosition(ofVec3f(0, 50, 75));
		cam.lookAt(testLander.lander1.getPosition());
	}

	if (camTracking == true) {

		cam.end();
			
		targetcm.setPosition(testLander.lander1.getPosition());
		targetcm.lookAt(testLander.lander1.getPosition());
		targetcm.draw();
		targetcm.begin();

		//set original camera to look back at ship when key is released

		cam.setPosition(ofVec3f(0, 10, 50));

		
		cam.lookAt(testLander.lander1.getPosition());
		
	}


	if (bWireframe) {                    // optional unimplemented wireframe mode  (include axis)
		ofDisableLighting();
		ofSetColor(ofColor::slateGray);
		mars.drawWireframe();

		if (bLanderLoaded) {
			lander.drawWireframe();
			if (!bTerrainSelected) drawAxis(lander.getPosition());
		}
		if (bTerrainSelected) drawAxis(ofVec3f(0, 0, 0));
	}


	else {

		ofEnableLighting();              // shaded mode
		mars.drawFaces();
		ofMesh mesh;

		if (keyHeld) {
			emitter.draw();
		}
		if (explodedraw) {
			emitter2.draw();
		}
		
		//draw the lander object and associated heading and ray vectors if desired
		testLander.drawLander();

		if (testLander.bLanderLoaded) {

			//display box detection
			if (!bTerrainSelected) drawAxis(testLander.lander1.getPosition());
			if (bDisplayBBoxes) {
				ofNoFill();
				ofSetColor(ofColor::white);

				Octree::drawBox(testBox);

				for (int i = 0; i < testLander.lander1.getNumMeshes(); i++) {
					ofPushMatrix();
					ofMultMatrix(testLander.lander1.getModelMatrix());
					ofRotate(-90, 1, 0, 0);
					Octree::drawBox(testLander.bboxList[i]);
					ofPopMatrix();
				}
			}

			//mouse selection detection
			if (bLanderSelected) {

				ofVec3f min = testLander.lander1.getSceneMin() + testLander.lander1.getPosition();
				ofVec3f max = testLander.lander1.getSceneMax() + testLander.lander1.getPosition();

				Box bounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
				ofSetColor(ofColor::white);
				Octree::drawBox(bounds);

			}

			//optional way to show boxes collided with
			
			if (collisionD) {

				// draw colliding boxes
				//
				if (showBoxes) {
					ofSetColor(ofColor::lightBlue);
					for (int i = 0; i < colBoxList.size(); i++) {
						Octree::drawBox(colBoxList[i]);
					}
				}
				colBoxList.clear();
				//collisionD = false;
			}
		}
	}

	//rgb axis in center (disabled)
	//if (bTerrainSelected) drawAxis(ofVec3f(0, 0, 0));

	// display points as an option  

	if (bDisplayPoints) {                  
		glPointSize(3);
		ofSetColor(ofColor::green);
		mars.drawVertices();
	}

	// recursively draw octree

	//ofDisableLighting();
	int level = 0;


	if (bDisplayLeafNodes) {
		octree.drawLeafNodes(octree.root);
		cout << "num leaf: " << octree.numLeaf << endl;
    }
	else if (bDisplayOctree) {
		ofNoFill();
		ofSetColor(ofColor::white);
		octree.draw(numLevels, 0);
	}

	// UNUSED
	/*
	// if point selected, draw a sphere
	//
	if (pointSelected) {
		ofVec3f p = octree.mesh.getVertex(selectedNode.points[0]);
		ofVec3f d = p - cam.getPosition();
		ofSetColor(ofColor::lightGreen);
		ofDrawSphere(p, .02 * d.length());
	}
	*/


	ofPopMatrix();

	cam.end();

	//shader 1          //working shader    UNIMPLEMENTED because it breaks explosion effect
	/*
	cam.begin();
	shader.begin();

	glDepthMask(GL_FALSE);

	ofSetColor(255, 10, 90);

	ofEnableBlendMode(OF_BLENDMODE_ADD);
	ofEnablePointSprites();

	emitter.draw();

	particleTex.bind();
	vbo.draw(GL_POINTS, 0, (int)emitter.sys->particles.size());
	particleTex.unbind();

	//  end drawing in the camera
	// 
	cam.end();
	shader.end();


	ofDisablePointSprites();
	ofDisableBlendMode();
	ofEnableAlphaBlending();

	// set back the depth mask
	//
	glDepthMask(GL_TRUE);

	*/
	//second shader did not work


	//cam.begin();
	ofDisableLighting();
	//cam.end();

	//bitmap string drawings of game information

	string str3;
	str3 = "landingscore: " + std::to_string(landingscore) + "\n";
	ofSetColor(ofColor::white);
	ofDrawBitmapString(str3, ofGetWindowWidth() - 210, 35);

	FuelT = "Fuel Time Left : " + std::to_string(fuelLeft) + " Seconds\n";
	ofSetColor(ofColor::white);
	ofDrawBitmapString(FuelT, ofGetWindowWidth() - 200, 55);

	string str2;
	str2 += "Altitude: " + std::to_string(altitude);
	ofSetColor(ofColor::white);
	ofDrawBitmapString(str2, ofGetWindowWidth() - 170, 15);

}


//  UNUSED
// Draw an XYZ axis in RGB at world (0,0,0) for reference.
//
void ofApp::drawAxis(ofVec3f location) {

	ofPushMatrix();
	ofTranslate(location);

	ofSetLineWidth(1.0);

	// X Axis
	ofSetColor(ofColor(255, 0, 0));
	ofDrawLine(ofPoint(0, 0, 0), ofPoint(1, 0, 0));
	

	// Y Axis
	ofSetColor(ofColor(0, 255, 0));
	ofDrawLine(ofPoint(0, 0, 0), ofPoint(0, 1, 0));

	// Z Axis
	ofSetColor(ofColor(0, 0, 255));
	ofDrawLine(ofPoint(0, 0, 0), ofPoint(0, 0, 1));

	ofPopMatrix();
}


void ofApp::keyPressed(int key) {
	keymap[key] = true;
	switch (key) {
	case ' ':
		//camTracking = true;
		scenarioA = true;
		break;
		//takeoff.play();
	case 'B':
	case 'b':
		bDisplayBBoxes = !bDisplayBBoxes;
		break;
	case 'C':
		if (showBoxes) {
			showBoxes = false;
		}
		else {
			showBoxes = true;
		}
	case 'c':
		if (cam.getMouseInputEnabled()) cam.disableMouseInput();
		else {
			cam.enableMouseInput();
		}
		break;
	case 'F':
	case 'f':
		ofToggleFullscreen();
		break;
	case 'H':
	case 'h':
		break;
	case 'L':
	case 'l':
		bDisplayLeafNodes = !bDisplayLeafNodes;
		break;
	case 'O':
	case 'o':
		bDisplayOctree = !bDisplayOctree;
		break;
	case 'r':
		cam.reset();
		break;
	case 's':
		//savePicture();
		break;
	case 't':
		if (camLand) {
			camLand = !camLand;
			cam.setPosition(ofVec3f(0, 50, 75));
			cam.lookAt(testLander.lander1.getPosition());
			landercam.end();
		}
		else {
			camLand = true;
		}
		break;
	case 'x':
		if (scenarioA) {
			scenarioA = !scenarioA;
			landercam.end();
		}
		else {
			scenarioA = true;
		}
		break;
	case 'y':
		if (camTracking) {
			camTracking = !camTracking;
			targetcm.end();
		}
		else {
			camTracking = true;
		}
		break;
	case 'u':
		break;
	case 'v':
		togglePointsDisplay();
		break;
	case 'V':
		break;
	case 'w':
		//toggleWireframeMode();
		break;
	case OF_KEY_ALT:
		cam.enableMouseInput();
		bAltKeyDown = true;
		break;
	case OF_KEY_CONTROL:
		bCtrlKeyDown = true;
		break;
	case OF_KEY_SHIFT:
		break;
	case OF_KEY_DEL:
		break;

	default:
		break;
	}
}

void ofApp::toggleWireframeMode() {
	bWireframe = !bWireframe;
}

void ofApp::toggleSelectTerrain() {
	bTerrainSelected = !bTerrainSelected;
}

void ofApp::togglePointsDisplay() {
	bDisplayPoints = !bDisplayPoints;
}

void ofApp::keyReleased(int key) {
	keymap[key] = false;
	switch (key) {
	
	case OF_KEY_ALT:
		cam.disableMouseInput();
		bAltKeyDown = false;
		break;
	case OF_KEY_CONTROL:
		bCtrlKeyDown = false;
		break;
	case OF_KEY_SHIFT:
		break;
	case OF_KEY_LEFT:
		testLander.rotateTrue = false;
		break;
	case OF_KEY_RIGHT:
		testLander.rotateTrue = false;
		break;
	case 'w':
		if(!noFuel){
			//calculation for fuel depleted during holding of key
			//subtract from total fuel
			timeUsedfinish = ofGetElapsedTimef();
			totalTime = timeUsedfinish - timeUsedstart;
			fuelLeft = fuelLeft - totalTime;
			if (fuelLeft < 0) {
				fuelLeft = 0;
			}
		}
		thrustsound.stop();
		emitter.stop();
		playSound = false;
		keyHeld = false;
		break;
	case 'a':
		if (!noFuel) {
			timeUsedfinish = ofGetElapsedTimef();
			totalTime = timeUsedfinish - timeUsedstart;
			fuelLeft = fuelLeft - totalTime;
			if (fuelLeft < 0) {
				fuelLeft = 0;
			}
		}
		thrustsound.stop();
		emitter.stop();
		playSound = false;
		keyHeld = false;
		break;
	case 's':
		if (!noFuel) {
			timeUsedfinish = ofGetElapsedTimef();
			totalTime = timeUsedfinish - timeUsedstart;
			fuelLeft = fuelLeft - totalTime;
			if (fuelLeft < 0) {
				fuelLeft = 0;
			}
		}
		thrustsound.stop();
		emitter.stop();
		playSound = false;
		keyHeld = false;
		break;
	case 'd':
		if (!noFuel) {
			timeUsedfinish = ofGetElapsedTimef();
			totalTime = timeUsedfinish - timeUsedstart;
			fuelLeft = fuelLeft - totalTime;
			if (fuelLeft < 0) {
				fuelLeft = 0;
			}
		}
		emitter.stop();
		thrustsound.stop();
		playSound = false;
		keyHeld = false;
		break;

	case 't':
		//camLand = false;
		break;
	case 'x':
		//scenarioA = false;
		break;
	case 'y':
		//camTracking = false;
		break;
	default:
		break;
	}
}



//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

	
}


//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

	// if moving camera, don't allow mouse interaction
	//
	if (cam.getMouseInputEnabled()) return;

	// if moving camera, don't allow mouse interaction
//
	if (cam.getMouseInputEnabled()) return;

	// if rover is loaded, test for selection
	//
	if (testLander.bLanderLoaded) {
		glm::vec3 origin = cam.getPosition();
		glm::vec3 mouseWorld = cam.screenToWorld(glm::vec3(mouseX, mouseY, 0));
		glm::vec3 mouseDir = glm::normalize(mouseWorld - origin);

		ofVec3f min = testLander.lander1.getSceneMin() + testLander.lander1.getPosition();
		ofVec3f max = testLander.lander1.getSceneMax() + testLander.lander1.getPosition();

		Box bounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
		bool hit = bounds.intersect(Ray(Vector3(origin.x, origin.y, origin.z), Vector3(mouseDir.x, mouseDir.y, mouseDir.z)), 0, 10000);
		if (hit) {
			
			bLanderSelected = true;
			mouseDownPos = getMousePointOnPlane(testLander.lander1.getPosition(), cam.getZAxis());
			mouseLastPos = mouseDownPos;
			bInDrag = true;
		}
		else {
			bLanderSelected = false;
		}


	}
	else {
		ofVec3f p;

		startTime = ofGetElapsedTimeMicros();


		raySelectWithOctree(p);

		endTime = ofGetElapsedTimeMicros();

		cout << "Time to intersect: " << endTime - startTime << " microseconds" << endl;

	}
}

bool ofApp::raySelectWithOctree(ofVec3f &pointRet) {
	ofVec3f mouse(mouseX, mouseY);
	ofVec3f rayPoint = cam.screenToWorld(mouse);
	ofVec3f rayDir = rayPoint - cam.getPosition();
	rayDir.normalize();
	Ray ray = Ray(Vector3(rayPoint.x, rayPoint.y, rayPoint.z),
		Vector3(rayDir.x, rayDir.y, rayDir.z));

	pointSelected = octree.intersect(ray, octree.root, selectedNode);

	if (pointSelected) {
		pointRet = octree.mesh.getVertex(selectedNode.points[0]);
	}
	return pointSelected;
}

//ray intersect function used to return first node intersected by ray which points down from lander

bool ofApp::rayIntersect(ofVec3f *pointRet) {

	bool pointSelected = octree.intersect(testray, octree.root, selectedNode);

	if (pointSelected) {
		*pointRet = octree.mesh.getVertex(selectedNode.points[0]);
	}
	return pointSelected;

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

	// if moving camera, don't allow mouse interaction
	//
	if (cam.getMouseInputEnabled()) return;

	if (bInDrag) {

		glm::vec3 landerPos = testLander.lander1.getPosition();

		glm::vec3 mousePos = getMousePointOnPlane(landerPos, cam.getZAxis());
		glm::vec3 delta = mousePos - mouseLastPos;
	
		landerPos += delta;
		testLander.lander1.setPosition(landerPos.x, landerPos.y, landerPos.z);
		partsys.particles.at(0).position.x = landerPos.x;
		partsys.particles.at(0).position.y = landerPos.y;
		partsys.particles.at(0).position.z = landerPos.z;

		mouseLastPos = mousePos;

		ofVec3f min = testLander.lander1.getSceneMin() + testLander.lander1.getPosition();
		ofVec3f max = testLander.lander1.getSceneMax() + testLander.lander1.getPosition();

		Box bounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));

		colBoxList.clear();

		ofNoFill();
		octree.intersect(bounds, octree.root, colBoxList);
		
		/*
		if (bounds.overlap(testBox)) {
			cout << "overlap" << endl;
		}
		else {
			cout << "OK" << endl;
		}
		*/

	}
	else {



		ofVec3f p;
		raySelectWithOctree(p);
	}
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {
	bInDrag = false;
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}



//--------------------------------------------------------------
// setup basic ambient lighting in GL  (for now, enable just 1 light)
//
void ofApp::initLightingAndMaterials() {

	static float ambient[] =
	{ .5f, .5f, .5, 1.0f };
	static float diffuse[] =
	{ 1.0f, 1.0f, 1.0f, 1.0f };

	static float position[] =
	{5.0, 5.0, 5.0, 0.0 };

	static float lmodel_ambient[] =
	{ 1.0f, 1.0f, 1.0f, 1.0f };

	static float lmodel_twoside[] =
	{ GL_TRUE };


	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, position);

	glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT1, GL_POSITION, position);


	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
	glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
//	glEnable(GL_LIGHT1);
	glShadeModel(GL_SMOOTH);
} 


void ofApp::savePicture() {
	ofImage picture;
	picture.grabScreen(0, 0, ofGetWidth(), ofGetHeight());
	picture.save("screenshot.png");
	cout << "picture saved" << endl;
}

//--------------------------------------------------------------
//
// support drag-and-drop of model (.obj) file loading.  when
// model is dropped in viewport, place origin under cursor
//
void ofApp::dragEvent2(ofDragInfo dragInfo) {

	ofVec3f point;
	mouseIntersectPlane(ofVec3f(0, 0, 0), cam.getZAxis(), point);
	if (lander.loadModel(dragInfo.files[0])) {
		
		lander.setScaleNormalization(false);
//		lander.setScale(.1, .1, .1);
	//	lander.setPosition(point.x, point.y, point.z);
		lander.setPosition(1, 1, 0);

		bLanderLoaded = true;
		for (int i = 0; i < lander.getMeshCount(); i++) {
			bboxList.push_back(Octree::meshBounds(lander.getMesh(i)));
		}

		cout << "Mesh Count: " << lander.getMeshCount() << endl;
	}
	else cout << "Error: Can't load model" << dragInfo.files[0] << endl;
}

bool ofApp::mouseIntersectPlane(ofVec3f planePoint, ofVec3f planeNorm, ofVec3f &point) {
	ofVec2f mouse(mouseX, mouseY);
	ofVec3f rayPoint = cam.screenToWorld(glm::vec3(mouseX, mouseY, 0));
	ofVec3f rayDir = rayPoint - cam.getPosition();
	rayDir.normalize();
	return (rayIntersectPlane(rayPoint, rayDir, planePoint, planeNorm, point));
}

//--------------------------------------------------------------
//
// support drag-and-drop of model (.obj) file loading.  when
// model is dropped in viewport, place origin under cursor
//
void ofApp::dragEvent(ofDragInfo dragInfo) {
	if (lander.loadModel(dragInfo.files[0])) {
		cout << dragInfo.files[0] << endl;
		bLanderLoaded = true;
		lander.setScaleNormalization(false);
		lander.setPosition(0, 0, 0);
		cout << "number of meshes: " << lander.getNumMeshes() << endl;
		bboxList.clear();
		for (int i = 0; i < lander.getMeshCount(); i++) {
			bboxList.push_back(Octree::meshBounds(lander.getMesh(i)));
		}

		//		lander.setRotation(1, 180, 1, 0, 0);

				// We want to drag and drop a 3D object in space so that the model appears 
				// under the mouse pointer where you drop it !
				//
				// Our strategy: intersect a plane parallel to the camera plane where the mouse drops the model
				// once we find the point of intersection, we can position the lander/lander
				// at that location.
				//

				// Setup our rays
				//
		glm::vec3 origin = cam.getPosition();
		glm::vec3 camAxis = cam.getZAxis();
		glm::vec3 mouseWorld = cam.screenToWorld(glm::vec3(mouseX, mouseY, 0));
		glm::vec3 mouseDir = glm::normalize(mouseWorld - origin);
		float distance;

		bool hit = glm::intersectRayPlane(origin, mouseDir, glm::vec3(0, 0, 0), camAxis, distance);
		if (hit) {
			// find the point of intersection on the plane using the distance 
			// We use the parameteric line or vector representation of a line to compute
			//
			// p' = p + s * dir;
			//
			glm::vec3 intersectPoint = origin + distance * mouseDir;

			// Now position the lander's origin at that intersection point
			//
			glm::vec3 min = lander.getSceneMin();
			glm::vec3 max = lander.getSceneMax();
			float offset = (max.y - min.y) / 2.0;
			lander.setPosition(intersectPoint.x, intersectPoint.y - offset, intersectPoint.z);

			// set up bounding box for lander while we are at it
			//
			landerBounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
		}
	}


}


//  intersect the mouse ray with the plane normal to the camera 
//  return intersection point.   (package code above into function)
//
glm::vec3 ofApp::getMousePointOnPlane(glm::vec3 planePt, glm::vec3 planeNorm) {
	// Setup our rays
	//
	glm::vec3 origin = cam.getPosition();
	glm::vec3 camAxis = cam.getZAxis();
	glm::vec3 mouseWorld = cam.screenToWorld(glm::vec3(mouseX, mouseY, 0));
	glm::vec3 mouseDir = glm::normalize(mouseWorld - origin);
	float distance;

	bool hit = glm::intersectRayPlane(origin, mouseDir, planePt, planeNorm, distance);

	if (hit) {
		// find the point of intersection on the plane using the distance 
		// We use the parameteric line or vector representation of a line to compute
		//
		// p' = p + s * dir;
		//
		glm::vec3 intersectPoint = origin + distance * mouseDir;

		return intersectPoint;
	}
	else return glm::vec3(0, 0, 0);
}
