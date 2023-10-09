/* Copyright (C) 2022 Hoang Ha Le
*
* This software may be distributed under the terms
* of the MIT license. See the LICENSE file for details.
*/
#include "ISpeedOp.h"
#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"
#include "GenericPlatform.h"
#include "GenericPlatformProcess.h"
#include <chai3d.h>

using namespace chai3d;

/**
* this class provides the basic functionality to access the haptic device
*/

//void close(void) {
//	// stop the simulation
//	simulationRunning = false;
//
//	// wait for graphics and haptics loops to terminate
//	while (!simulationFinished) { cSleepMs(100); }
//	for (int i = 0; i < numHapticDevices; i++)
//	{
//		// close haptic device
//		tool[i]->stop();
//	}
//
//	// delete resources
//	delete hapticsThread;
//	delete world;
//	delete handler;
//}


class FSpeedOp : public ISpeedOp
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

	FVector getPosition(int);
	FMatrix getRotation(int);
	FVector getForce(int);
	FVector getTorque(int);
	FVector getLinearVelocity(int);
	FVector getAngularVelocity(int);
	bool isFirstButtonActivated(int);
	bool isSecondButtonActivated(int);
	bool connect();
	void disconnect();
	int  getNumOfDevices();
	void createVibrationEffect(int deviceId, float maxForce);
	void createDampingEffect(int deviceId, float maxForce);
	void createVirtualPort(int deviceId, FVector desiredPos, FVector port, float maxForce);
	void createViscosityEffect(int deviceId, float maxForce);
	void createMagneticEffect(int deviceId, FVector currentPosition, float intensityScale);
	void createPrimitiveObject(FVector objectPosition, FVector orientation, float stiffness, FString type, float size);
	void setPrimitiveObjectPositionAsDevicePos(int objectId);
	void setForce(FVector, int);
	void setTorque(FVector, int);
	void setForceAndTorque(FVector force, FVector torque, int deviceId);
	void updateHaptics(int deviceId);

private:
	int tickCount = 0;
	int numHapticDevices = 0;
	float intensityScale = 0;
	bool forceField[MAX_DEVICES];
	bool playOnce[MAX_DEVICES];
	FVector currentPosition;
	cToolCursor* tool[MAX_DEVICES];
	cHapticDeviceHandler* handler;
	cWorld* world;
	// haptic thread
	cThread* hapticsThread;
	//cPrecisionClock clock;
	cGenericHapticDevicePtr hapticDevice[MAX_DEVICES];

	cShapeSphere* vibration;
	cShapeSphere* viscosity;
	cShapeBox* surface;
	cShapeSphere* magnet;
	double workspaceScaleFactor = 0.0;
	cVector3d position;
	cMatrix3d rotation;
	bool alreadyConnectedWithPhantom = false;
	float toolRadius = 0.01;
	cMesh* base;
	cMesh* left;
	cMesh* right;
	cMesh* up;
	cShapeSphere* sphere;
	cShapeSphere* sphereSoft;
	cShapeCylinder* cylinder;
	cShapeCylinder* cylinderY1;
	cShapeCylinder* cylinderY2;
	cShapeCylinder* cylinderY3;
	cShapeCylinder* cylinderY4;
	cShapeBox* box;
	cShapeBox* hapticObject;
	double maxStiffness;
	bool doOnce[MAX_DEVICES];
	bool collision[MAX_DEVICES];

	// a frequency counter to measure the simulation haptic rate
	cFrequencyCounter freqCounterHaptics;

};

IMPLEMENT_MODULE(FSpeedOp, SpeedOp)

void FSpeedOp::StartupModule(){
}

void FSpeedOp::ShutdownModule(){
}

/*
 * open a connection to the haptic device to make it ready for communication
*/
bool FSpeedOp::connect() {	
	// create a new world.
	world = new cWorld();

	


	// create a haptic device handler
	handler = new cHapticDeviceHandler();
	// get number of haptic devices
	numHapticDevices = handler->getNumDevices();

	for (int i = 0; i < numHapticDevices; i++)
	{
		// get a handle to the first haptic device
		handler->getDevice(hapticDevice[i], i);

		playOnce[i] = true;
		forceField[i] = false;
		doOnce[i] = true;
		collision[i] = false;

		// open a connection to haptic device
		//hapticDevice[i]->open();

		//// calibrate device (if necessary)
		//hapticDevice[i]->calibrate();

		//// if the device has a gripper, enable the gripper to simulate a user switch
		//hapticDevice[i]->setEnableGripperUserSwitch(true);

		tool[i] = new cToolCursor(world);
		world->addChild(tool[i]);

		// connect the haptic device to the virtual tool
		tool[i]->setHapticDevice(hapticDevice[i]);



		// define a radius for the virtual tool (sphere)
		
		tool[i]->setRadius(toolRadius);

		// map the physical workspace of the haptic device to a larger virtual workspace.
		tool[i]->setWorkspaceRadius(1.0);

		// haptic forces are enabled only if small forces are first sent to the device;
		// this mode avoids the force spike that occurs when the application starts when 
		// the tool is located inside an object for instance. 
		tool[i]->setWaitForSmallForce(true);

		// start the haptic tool
		tool[i]->start();

		

		
	}
	if (numHapticDevices > 0)
	{
		// retrieve information about the current haptic device
		cHapticDeviceInfo info = hapticDevice[0]->getSpecifications();


		///////////////////////////////////////////////////////////////////////
	   // "VIBRATIONS"
	  ////////////////////////////////////////////////////////////////////////

	   // create a sphere and define its radius
		vibration = new cShapeSphere(0.01);
		viscosity = new cShapeSphere(0.01);
		//magnet = new cShapeSphere(0.3);
		surface = new cShapeBox(0.03, 0.03, 0.03);
		// add object to world
		world->addChild(vibration);
		world->addChild(surface);
		world->addChild(viscosity);
		//world->addChild(magnet);

		// set the position of the object at the center of the world
		vibration->setLocalPos(100, 100, 100);
		surface->setLocalPos(200, 200, 200);
		viscosity->setLocalPos(300, 300, 300);
		//magnet->setLocalPos(400, 400, 400);


		// read the scale factor between the physical workspace of the haptic
		// device and the virtual workspace defined for the tool
		workspaceScaleFactor = tool[0]->getWorkspaceScaleFactor();

		// get properties of haptic device
		double maxLinearForce = cMin(info.m_maxLinearForce, 7.0);
		maxStiffness = info.m_maxLinearStiffness / workspaceScaleFactor;
		double maxDamping = info.m_maxLinearDamping / workspaceScaleFactor;

		// set haptic properties
		vibration->m_material->setVibrationFrequency(100);
		vibration->m_material->setVibrationAmplitude(0.1 * 30);   // % of maximum linear force
		vibration->m_material->setStiffness(0.1 * 10);              // % of maximum linear stiffness
		//surface->m_material->setStiffness(maxLinearForce);// % of maximum linear force

		/*magnet->m_material->setStiffness(0.4 * maxStiffness);
		magnet->m_material->setMagnetMaxDistance(3);
		magnet->m_material->setViscosity(0.1 * maxDamping);*/

		// create a haptic vibration effect
		vibration->createEffectVibration();

		// create a haptic surface effect
		surface->createEffectSurface();

		// create a haptic viscosity effect
		viscosity->createEffectViscosity();

		sphere = new cShapeSphere(0);
		// add object to world

		world->addChild(sphere);

		sphere->createEffectSurface();

		sphereSoft = new cShapeSphere(0);
		// add object to world

		world->addChild(sphereSoft);

		sphereSoft->createEffectSurface();
		
		cylinder = new cShapeCylinder(0, 0, 0);

		world->addChild(cylinder);

		cylinder->createEffectSurface();

		cylinderY1 = new cShapeCylinder(0, 0, 0);

		world->addChild(cylinderY1);

		cylinderY1->createEffectSurface();

		cylinderY2 = new cShapeCylinder(0, 0, 0);

		world->addChild(cylinderY2);

		cylinderY2->createEffectSurface();

		cylinderY3 = new cShapeCylinder(0, 0, 0);

		world->addChild(cylinderY3);

		cylinderY3->createEffectSurface();

		cylinderY4 = new cShapeCylinder(0, 0, 0);

		world->addChild(cylinderY4);

		cylinderY4->createEffectSurface();

		box = new cShapeBox(0, 0, 0);

		world->addChild(box);

		box->createEffectSurface();

		hapticObject = new cShapeBox(0.2, 0.2, 0.2);
		hapticObject->m_material->setStiffness(75);

		world->addChild(hapticObject);

		hapticObject->createEffectSurface();

		base = new cMesh();

		// add object to world
		world->addChild(base);

		// build mesh using a cylinder primitive
		cCreateBox(base,
			0.05,
			1,
			0.01,
			cVector3d(0.0, 0.0, 0.0),
			cMatrix3d(cDegToRad(0), cDegToRad(0), cDegToRad(0), C_EULER_ORDER_XYZ)
		);


		// set material properties
		base->m_material->setGrayGainsboro();
		base->m_material->setStiffness(0.5 * maxStiffness);

		// build collision detection tree
		base->createAABBCollisionDetector(toolRadius);

		left = new cMesh();
		right = new cMesh();
		up = new cMesh();

		cCreateBox(left,
			0.05,
			1,
			0.01,
			cVector3d(-0.02, 0.0, 0.02),
			cMatrix3d(cDegToRad(0), cDegToRad(90), cDegToRad(0), C_EULER_ORDER_XYZ)
		);

		cCreateBox(right,
			0.05,
			1,
			0.01,
			cVector3d(0.02, 0.0, 0.02),
			cMatrix3d(cDegToRad(0), cDegToRad(90), cDegToRad(0), C_EULER_ORDER_XYZ)
		);

		cCreateBox(up,
			0.05,
			1,
			0.01,
			cVector3d(0.0, 0.0, 0.05),
			cMatrix3d(cDegToRad(0), cDegToRad(0), cDegToRad(0), C_EULER_ORDER_XYZ)
		);
		base->addChild(left);
		base->addChild(right);
		base->addChild(up);
		base->setLocalPos(100, 100, 100);
		// set material properties
		left->m_material->setGrayGainsboro();
		left->m_material->setStiffness(0.5 * maxStiffness);

		// build collision detection tree
		left->createAABBCollisionDetector(toolRadius);

		// set material properties
		right->m_material->setGrayGainsboro();
		right->m_material->setStiffness(0.5 * maxStiffness);

		// build collision detection tree
		right->createAABBCollisionDetector(toolRadius);

		// set material properties
		up->m_material->setGrayGainsboro();
		up->m_material->setStiffness(0.5 * maxStiffness);

		// build collision detection tree
		up->createAABBCollisionDetector(toolRadius);
		// create a haptic magnetic effect
		//magnet->createEffectMagnetic();

		//--------------------------------------------------------------------------
	   // START SIMULATION
	   //--------------------------------------------------------------------------

	   // create a thread which starts the main haptics rendering loop
		//hapticsThread = new cThread();
		//hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

		//// setup callback when application exits
		//atexit(close);


		return true;
	}
	return false;
	

}



void FSpeedOp::updateHaptics(int deviceId)
{

	//if (forceField[deviceId])
	//{
	//	cVector3d desiredPosition(currentPosition.X, currentPosition.Y, currentPosition.Z);
	//	hapticDevice[deviceId]->getPosition(position);

	//	// desired orientation
	//	cMatrix3d desiredRotation;
	//	desiredRotation.identity();

	//	// variables for forces
	//	cVector3d force(0, 0, 0);
	//	cVector3d torque(0, 0, 0);
	//	double gripperForce = 0.0;
	//	// compute linear force
	//	double Kp = 100 * intensityScale; // [N/m]

	//	// read linear velocity 
	//	cVector3d linearVelocity;
	//	hapticDevice[deviceId]->getLinearVelocity(linearVelocity);
	//	cVector3d forceField = Kp * (desiredPosition - position);
	//	force.add(forceField);


	//	// compute angular torque
	//	double Kr = 0.05 * intensityScale; // [N/m.rad]
	//	cVector3d axis;
	//	double angle;

	//	cMatrix3d deltaRotation = cTranspose(rotation) * desiredRotation;
	//	deltaRotation.toAxisAngle(axis, angle);
	//	torque = rotation * ((Kr * angle) * axis);
	//	hapticDevice[deviceId]->setForceAndTorqueAndGripperForce(force, torque, gripperForce);
	//}
	cVector3d linearVel = tool[deviceId]->getDeviceLocalLinVel();
	cVector3d toolPos = tool[deviceId]->getDeviceGlobalPos();
	cMatrix3d toolRot = tool[deviceId]->getDeviceGlobalRot();
	if (collision[deviceId]) {
		
		if (playOnce[deviceId]) {
			hapticObject->setLocalPos(cAdd(toolPos, linearVel / 7));
			playOnce[deviceId] = false;


		}

		cVector3d currentPos = hapticObject->getLocalPos();
		if ((linearVel.x() > 0.2) || (linearVel.x() < -0.2)) {
			linearVel = cVector3d(linearVel.x(), 0, linearVel.z());
		}
		else if ((linearVel.y() > 0.2) || (linearVel.y() < -0.2)) {
			linearVel = cVector3d(0, linearVel.y(), linearVel.z());
		}
		else if ((linearVel.z() > 0.2) || (linearVel.z() < -0.2)) {
			linearVel = cVector3d(linearVel.x(), 0, linearVel.z());
		}

		hapticObject->setLocalPos(currentPos + linearVel / 200);
	}


	else {
		
		playOnce[deviceId] = true;
		// compute global reference frames for each object
		

	}
	
		
	world->computeGlobalPositions(true);
	// update position and orientation of tool
	tool[deviceId]->updateFromDevice();
	// compute interaction forces
	tool[deviceId]->computeInteractionForces();
	// send forces to haptic device
	tool[deviceId]->applyToDevice();
		

		freqCounterHaptics.signal(1);

}


/*
* disconnect from the haptic device
*/
void FSpeedOp::disconnect(void) {
	for (int i = 0; i < numHapticDevices; i++)
	{
		// close haptic device
		tool[i]->stop();
		hapticDevice[i]->setForce(cVector3d(0, 0, 0));
		hapticDevice[i]->close();
	}
}


/**
* get number of haptic devices
*/
int FSpeedOp::getNumOfDevices() {
	return handler->getNumDevices();
}

/**
* get the current force of the haptic device
*/
FVector FSpeedOp::getForce(int deviceId) {
	cVector3d force(0, 0, 0);
	hapticDevice[deviceId]->getForce(force);
	return FVector(force.x(), force.y(), force.z());
}

/**
* check if the first button is activated
*/
bool FSpeedOp::isFirstButtonActivated(int deviceId) {
	bool result = false;
	hapticDevice[deviceId]->getUserSwitch(0, result);
	return result;
}

/**
* check if the second button is activated
*/
bool FSpeedOp::isSecondButtonActivated(int deviceId) {
	bool result = false;
	hapticDevice[deviceId]->getUserSwitch(1, result);
	return result;
}

/*
* get the current linear velocity
*/
FVector FSpeedOp::getLinearVelocity(int deviceId) {
	cVector3d chai_res(0, 0, 0);
	hapticDevice[deviceId]->getLinearVelocity(chai_res);
	return FVector(chai_res.x(), chai_res.y(), chai_res.z());
}

/*
* get the current angular velocity
*/
FVector FSpeedOp::getAngularVelocity(int deviceId) {
	cVector3d chai_res(0, 0, 0);
	hapticDevice[deviceId]->getAngularVelocity(chai_res);
	return FVector(chai_res.x(), chai_res.y(), chai_res.z());
}

/**
* get the current position of the haptic device
*/
FVector FSpeedOp::getPosition(int deviceId) {
	cVector3d position(0, 0, 0);
	hapticDevice[deviceId]->getPosition(position);
	return FVector(position.x(), position.y(), position.z());
}

/**
* get the current rotation of the haptic device as a 3x3 Matrix
*/
FMatrix FSpeedOp::getRotation(int deviceId) {
	cMatrix3d rotation(0, 0, 0);
	hapticDevice[deviceId]->getRotation(rotation);
	return FMatrix(FVector(rotation.getCol0().x(), rotation.getCol0().y(), rotation.getCol0().z()),
				   FVector(rotation.getCol1().x(), rotation.getCol1().y(), rotation.getCol1().z()),
				   FVector(rotation.getCol2().x(), rotation.getCol2().y(), rotation.getCol2().z()),
				   FVector(0, 0, 0));
}


/**
* create vibration effect
*/

void FSpeedOp::createVibrationEffect(int deviceId, float maxForce) {


	hapticDevice[deviceId]->getPosition(position);
	hapticDevice[deviceId]->getRotation(rotation);

	vibration->m_material->setVibrationAmplitude(maxForce);

	vibration->setLocalPos(cMul(workspaceScaleFactor, position));
	vibration->setLocalRot(rotation);

	vibration->m_material->setVibrationAmplitude(maxForce);


	

}

void FSpeedOp::createVirtualPort(int deviceId, FVector pos, FVector port, float maxForce) {


	/*hapticDevice[deviceId]->getPosition(position);
	hapticDevice[deviceId]->getRotation(rotation);*/
	/*cVector3d linearVelocity;
	hapticDevice[deviceId]->getLinearVelocity(linearVelocity);
	cVector3d force = -linearVelocity * 50 * maxForce;
	if (force.length() < 8)
	{
		hapticDevice[deviceId]->setForce(force);
	}*/

	//bool result;
	//if (maxForce == 0) {
	//	result = false;
	//}
	//else {
	//	result = true;
	//}

	if (pos.X == 0 && pos.Y == 0 && pos.Z == 0)
	{
	
		doOnce[deviceId] = true;
		
		base->m_material->setStiffness(0);
		left->m_material->setStiffness(0);
		right->m_material->setStiffness(0);
		up->m_material->setStiffness(0);
	}
	else
	{
		base->m_material->setStiffness(0.5 * maxStiffness);
		left->m_material->setStiffness(0.5 * maxStiffness);
		right->m_material->setStiffness(0.5 * maxStiffness);
		up->m_material->setStiffness(0.5 * maxStiffness);
		cVector3d linearVelocity;
		hapticDevice[deviceId]->getLinearVelocity(linearVelocity);
		hapticDevice[deviceId]->getPosition(position);
		cVector3d vector(0.0, -0.07, 0.0);
		hapticDevice[deviceId]->getRotation(rotation);

		if (linearVelocity.length() != 0 && doOnce[deviceId]) {

			

			base->setLocalPos(cMul(workspaceScaleFactor, (position + vector)));
			

		/*	FMatrix rot(FVector(rotation.getCol0().x(), rotation.getCol0().y(), rotation.getCol0().z()),
				FVector(rotation.getCol1().x(), rotation.getCol1().y(), rotation.getCol1().z()),
				FVector(rotation.getCol2().x(), rotation.getCol2().y(), rotation.getCol2().z()),
				FVector(0, 0, 0));
			FVector euler = rot.Rotator().Euler();*/
			base->setLocalRot(cMatrix3d(cDegToRad(-port.X), cDegToRad(-port.Y), cDegToRad(-port.Z), C_EULER_ORDER_XYZ));
			
			doOnce[deviceId] = false;

		}
	}
	



	////else if (result && !playOnce && distance > 0.01)
	////{
	////	playOnce = true;
	////}

	////else if (!result) {
	////	playOnce = true;
	////}


	//  // desired position
	//cVector3d desiredPosition;
	//desiredPosition.set(0.0, 0.0, 0.0);

	//hapticDevice[deviceId]->getPosition(position);

	//// desired orientation
	//cMatrix3d desiredRotation;
	//desiredRotation.identity();

	//// variables for forces    
	//cVector3d force(0, 0, 0);
	//cVector3d torque(0, 0, 0);
	//double gripperForce = 0.0;

	//// compute linear force
	//double Kp = 25 * maxForce; // [N/m]
	////cVector3d forceField = Kp * (desiredPosition - position);
	//cVector3d linearVelocity;
	//hapticDevice[deviceId]->getLinearVelocity(linearVelocity);
	//cVector3d forceField = Kp * linearVelocity;
	//force.add(forceField);


	//// read angular velocity
	//cVector3d angularVelocity;
	//hapticDevice[deviceId]->getAngularVelocity(angularVelocity);

	//// compute angular damping force
	//double Kvr = 1.0 * maxForce;
	//cVector3d torqueDamping = Kvr * angularVelocity;
	//torque.add(torqueDamping);
	//hapticDevice[deviceId]->setForceAndTorqueAndGripperForce(-force, -torque, -gripperForce);


}

void FSpeedOp::createViscosityEffect(int deviceId, float maxForce) {


	hapticDevice[deviceId]->getPosition(position);
	hapticDevice[deviceId]->getRotation(rotation);

	viscosity->m_material->setViscosity(maxForce);

	viscosity->setLocalPos(cMul(workspaceScaleFactor, position));
	viscosity->setLocalRot(rotation);


}

void FSpeedOp::createMagneticEffect(int deviceId, FVector pos, float force) {
	/*if (pos.X == 0 && pos.Y == 0 && pos.Z == 0)
	{
		forceField[deviceId] = false;
		playOnce[deviceId] = true;
	}
	else if(playOnce[deviceId]) {
		forceField[deviceId] = true;
		currentPosition = pos;
		playOnce[deviceId] = false;
		intensityScale = scale;
	}*/
	if (force > 0) {
		collision[deviceId] = true;
	}
	else {
		//collision[deviceId] = false;
	}
}

void FSpeedOp::createDampingEffect(int deviceId, float maxForce) {
	
	cVector3d force(0, 0, 0);
	cVector3d linearVelocity;
	hapticDevice[deviceId]->getLinearVelocity(linearVelocity);

	cHapticDeviceInfo info = hapticDevice[deviceId]->getSpecifications();

	// compute linear force
	double Kv = maxForce * info.m_maxLinearDamping;
	cVector3d forceDamping = -Kv * linearVelocity;
	force.add(forceDamping);
	hapticDevice[deviceId]->setForce(force);
	


}

void FSpeedOp::createPrimitiveObject(FVector objectPosition, FVector orientation, float stiffness, FString type, float size) {
	
	if (type == "Sphere") {
		//sphere = new cShapeSphere(size/10);
		//// add object to world

		//world->addChild(sphere);
		sphere->m_material->setStiffness(75 * stiffness);
		
		
		sphere->setRadius(workspaceScaleFactor * size / 15);
		//sphere->setLocalPos(-objectPosition.X / 150, objectPosition.Y / 150, objectPosition.Z / 75);
		cVector3d pos(-objectPosition.X / 1000, objectPosition.Y / 1000, objectPosition.Z / 500);

		sphere->setLocalPos(cMul(workspaceScaleFactor, pos));

	}
	else if (type == "SphereSoft") {
		//sphere = new cShapeSphere(size/10);
		//// add object to world

		//world->addChild(sphere);
		sphereSoft->m_material->setStiffness(75 * stiffness);


		sphereSoft->setRadius(workspaceScaleFactor * size / 15);
		//sphere->setLocalPos(-objectPosition.X / 150, objectPosition.Y / 150, objectPosition.Z / 75);
		cVector3d pos(-objectPosition.X / 1000, objectPosition.Y / 1000, objectPosition.Z / 500);

		sphereSoft->setLocalPos(cMul(workspaceScaleFactor, pos));

	}
	else if (type == "Cylinder") {

		cylinder->m_material->setStiffness(75 * stiffness);
	

		cylinder->setBaseRadius(workspaceScaleFactor * size / 15);
		cylinder->setTopRadius(workspaceScaleFactor * size / 15);
		cylinder->setHeight(workspaceScaleFactor * size / 2);
		cVector3d pos(- objectPosition.X / 1000, size / 4 + objectPosition.Y / 1000, objectPosition.Z / 500);
		cylinder->setLocalPos(cMul(workspaceScaleFactor, pos));
		cylinder->setLocalRot(cMatrix3d(cDegToRad(orientation.X), cDegToRad(orientation.Z), cDegToRad(orientation.Y), C_EULER_ORDER_XYZ));
	}
	else if (type == "CylinderY1") {

		cylinderY1->m_material->setStiffness(75 * stiffness);


		cylinderY1->setBaseRadius(workspaceScaleFactor * size / 15);
		cylinderY1->setTopRadius(workspaceScaleFactor * size / 15);
		cylinderY1->setHeight(workspaceScaleFactor * size / 2);
		cVector3d pos(-size / 4 -objectPosition.X / 1000, objectPosition.Y / 1000, objectPosition.Z / 500);
		cylinderY1->setLocalPos(cMul(workspaceScaleFactor, pos));
		cylinderY1->setLocalRot(cMatrix3d(cDegToRad(orientation.X), cDegToRad(orientation.Z), cDegToRad(orientation.Y), C_EULER_ORDER_XYZ));
	}
	else if (type == "CylinderY2") {

		cylinderY2->m_material->setStiffness(75 * stiffness);


		cylinderY2->setBaseRadius(workspaceScaleFactor * size / 15);
		cylinderY2->setTopRadius(workspaceScaleFactor * size / 15);
		cylinderY2->setHeight(workspaceScaleFactor * size / 2);
		cVector3d pos(-size / 4 - objectPosition.X / 1000, objectPosition.Y / 1000, objectPosition.Z / 500);
		cylinderY2->setLocalPos(cMul(workspaceScaleFactor, pos));
		cylinderY2->setLocalRot(cMatrix3d(cDegToRad(orientation.X), cDegToRad(orientation.Z), cDegToRad(orientation.Y), C_EULER_ORDER_XYZ));
	}

	else if (type == "CylinderY3") {

		cylinderY3->m_material->setStiffness(75 * stiffness);


		cylinderY3->setBaseRadius(workspaceScaleFactor * size / 15);
		cylinderY3->setTopRadius(workspaceScaleFactor * size / 15);
		cylinderY3->setHeight(workspaceScaleFactor * size / 2);
		cVector3d pos(-size / 4 - objectPosition.X / 1000, objectPosition.Y / 1000, objectPosition.Z / 500);
		cylinderY3->setLocalPos(cMul(workspaceScaleFactor, pos));
		cylinderY3->setLocalRot(cMatrix3d(cDegToRad(orientation.X), cDegToRad(orientation.Z), cDegToRad(orientation.Y), C_EULER_ORDER_XYZ));
	}
	else if (type == "CylinderY4") {

		cylinderY4->m_material->setStiffness(75 * stiffness);


		cylinderY4->setBaseRadius(workspaceScaleFactor * size / 15);
		cylinderY4->setTopRadius(workspaceScaleFactor * size / 15);
		cylinderY4->setHeight(workspaceScaleFactor * size / 2);
		cVector3d pos(-size / 4 - objectPosition.X / 1000, objectPosition.Y / 1000, objectPosition.Z / 500);
		cylinderY4->setLocalPos(cMul(workspaceScaleFactor, pos));
		cylinderY4->setLocalRot(cMatrix3d(cDegToRad(orientation.X), cDegToRad(orientation.Z), cDegToRad(orientation.Y), C_EULER_ORDER_XYZ));
	}

	else if (type == "Box") {

		box->m_material->setStiffness(75 * stiffness);


		box->setSize(size, size, size/100);
		
		cVector3d pos(objectPosition.X, objectPosition.Y, objectPosition.Z);
		box->setLocalPos(pos);
		
	}
}

void FSpeedOp::setPrimitiveObjectPositionAsDevicePos(int objectId) {
	hapticDevice[0]->getPosition(position);
	sphere->setLocalPos(cMul(workspaceScaleFactor, position));
}

/**
* send a force to the haptic device
*/
void FSpeedOp::setForce(FVector force, int deviceId) {
	cVector3d cForce(force.X, force.Y, force.Z);
	hapticDevice[deviceId]->setForce(cForce);
}

/**
* send a torque to the haptic device
*/
void FSpeedOp::setTorque(FVector torque, int deviceId) {
	cVector3d cTorque(torque.X, torque.Y, torque.Z);
	cVector3d cForce(0,0,0);
	hapticDevice[deviceId]->getForce(cForce);
	hapticDevice[deviceId]->setForceAndTorque(cForce, cTorque);
}

/**
* send force and torque to the haptic device
*/
void FSpeedOp::setForceAndTorque(FVector force, FVector torque, int deviceId) {
	cVector3d cForce(force.X, force.Y, force.Z);
	cVector3d cTorque(torque.X, torque.Y, torque.Z);
	hapticDevice[deviceId]->setForceAndTorque(cForce, cTorque);
}


/**
* get the current torque of the haptic device
*/
FVector FSpeedOp::getTorque(int deviceId) {
	cVector3d torque(0, 0, 0);
	hapticDevice[deviceId]->getTorque(torque);
	return FVector(torque.x(), torque.y(), torque.z());
}


