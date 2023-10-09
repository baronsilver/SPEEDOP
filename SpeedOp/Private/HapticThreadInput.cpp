/* Copyright (C) 2022 Hoang Ha Le
*
* This software may be distributed under the terms
* of the MIT license. See the LICENSE file for details.
*/
#include "HapticThreadInput.h"
#include "ISpeedOp.h"

/**
* return true if the haptic thread is supposed to be running
*/
bool UHapticThreadInput::shouldThreadRun() {
	return runThread; 
}

/**
* determine if the haptic thread should run. If this is to false while the haptic thread runs, it will stop the thread. 
*/
void UHapticThreadInput::setRunThread(bool r) { 
	runThread = r;
}

/**
* set the force that will be given to the haptic thread so that the force is applied in the next tick
*/
void UHapticThreadInput::setForceToApply(FVector force, int deviceId) {
	forceToApply[deviceId] = force;
}

/**
* get the force that should be applied in the next haptic tick
*/
FVector UHapticThreadInput::getForceToApply(int deviceId) {
	return forceToApply[deviceId];
}

/**
* set the torque that will be given to the haptic thread so that the torque is applied in the next tick
*/
void UHapticThreadInput::setTorqueToApply(FVector torque, int deviceId) {
	torqueToApply[deviceId] = torque;
}

/**
* get the torque that should be applied in the next haptic tick
*/
FVector UHapticThreadInput::getTorqueToApply(int deviceId) {
	return torqueToApply[deviceId];
}

/**
* set the maximum vibration force
*/
void UHapticThreadInput::setMaxVibrationForce(int deviceId, float maxForce) {
	
	maxVibrationForce[deviceId] = maxForce;
}

/**
* get the maximum vibration force
*/
float UHapticThreadInput::getMaxVibrationForce(int deviceId) {
	return maxVibrationForce[deviceId];
}


/**
* set the maximum surface force
*/
void UHapticThreadInput::setMaxSurfaceForce(int deviceId, float maxForce) {

	maxSurfaceForce[deviceId] = maxForce;
}

/**
* get the maximum surface force
*/
float UHapticThreadInput::getMaxSurfaceForce(int deviceId) {
	return maxSurfaceForce[deviceId];
}

/**
* set the maximum damping force
*/
void UHapticThreadInput::setMaxDampingForce(int deviceId, float maxForce) {
	maxDampingForce[deviceId] = maxForce;
}

/**
* get the maximum damping force
*/
float UHapticThreadInput::getMaxDampingForce(int deviceId) {
	return maxDampingForce[deviceId];
}

/**
* set the maximum viscosity force
*/
void UHapticThreadInput::setMaxViscosityForce(int deviceId, float maxForce) {
	maxViscosityForce[deviceId] = maxForce;
}

/**
* get the maximum viscosity force
*/
float UHapticThreadInput::getMaxViscosityForce(int deviceId) {
	return maxViscosityForce[deviceId];
}


/**
* set the maximum magnetic force
*/
void UHapticThreadInput::setMaxMagneticForce(int deviceId, float maxForce) {
	maxMagneticForce[deviceId] = maxForce;
}

/**
* get the maximum magnetic force
*/
float UHapticThreadInput::getMaxMagneticForce(int deviceId) {
	return maxMagneticForce[deviceId];
}


/**
* set the object orientation
*/
void UHapticThreadInput::setPortOrientation(int deviceId, FVector port) {
	portOrientation[deviceId] = port;
}
/**
* get the object orientation
*/
FVector UHapticThreadInput::getPortOrientation(int deviceId) {
	return portOrientation[deviceId];
}


/**
* set primitive object properties
*/
void UHapticThreadInput::setPrimitiveObjectProperties(int objectId, FVector position, FVector orientation, float stiffness, FString type, float size) {
	primitiveObjectPosition[objectId] = position;
	primitiveObjectOrientation[objectId] = orientation;
	primitiveObjectType[objectId] = type;
	primitiveObjectStiffness[objectId] = stiffness;
	primitiveObjectSize[objectId] = size;
}


/**
* get primitive object position
*/
FVector UHapticThreadInput::getPrimitiveObjectPosition(int objectId) {
	return primitiveObjectPosition[objectId];
}

/**
* get primitive object orientation
*/
FVector UHapticThreadInput::getPrimitiveObjectOrientation(int objectId) {
	return primitiveObjectOrientation[objectId];
}

/**
* get primitive object stiffness
*/
float UHapticThreadInput::getPrimitiveObjectStiffness(int objectId) {
	return primitiveObjectStiffness[objectId];
}

/**
* get primitive object type
*/
FString UHapticThreadInput::getPrimitiveObjectType(int objectId) {
	return primitiveObjectType[objectId];
}

/**
* get primitive object size
*/
float UHapticThreadInput::getPrimitiveObjectSize(int objectId) {
	return primitiveObjectSize[objectId];
}