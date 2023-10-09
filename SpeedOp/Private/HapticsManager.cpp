/* Copyright (C) 2022 Hoang Ha Le
*
* This software may be distributed under the terms
* of the MIT license. See the LICENSE file for details.
*/
#include "HapticThread.h"
#include "ISpeedOp.h"
#include "HapticsManager.h"
#include "AsyncWork.h"
#include "HapticThreadInput.h"
#include "HapticThreadOutput.h"

/**
 * constructs an instance of the haptic manager
*/
AHapticsManager::AHapticsManager()
{
	PrimaryActorTick.bCanEverTick = true;
}

/**
 * Called when the actor is spawned and starts the haptic thread
 */
void AHapticsManager::BeginPlay()
{
	Super::BeginPlay();
	UHapticThreadInput::getInst().setRunThread(true);
	(new FAutoDeleteAsyncTask<FHapticThread>(ISpeedOp::Get(), this))->StartBackgroundTask();
}

/**
* Called when the actor is destroyed and ends the haptic thread
*/
void  AHapticsManager::EndPlay(EEndPlayReason::Type type)
{
	Super::EndPlay(type);
	UHapticThreadInput::getInst().setRunThread(false);
}

/**
 * Called every frame
*/
void AHapticsManager::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

/**
 * set the force that should be applied to the haptic device in the next tick of the haptic thread
*/
void AHapticsManager::setForceToApply(FVector force, int deviceId) {
	UHapticThreadInput::getInst().setForceToApply(force, deviceId);
}

/**
* set the torque that should be applied to the haptic device in the next tick of the haptic thread
*/
void AHapticsManager::setTorqueToApply(FVector torque, int deviceId) {
	UHapticThreadInput::getInst().setTorqueToApply(torque, deviceId);
}

/**
* start vibration effect
*/
void AHapticsManager::startVibrationEffect(int deviceId, float maxForce) {
	UHapticThreadInput::getInst().setMaxVibrationForce(deviceId, maxForce);
}

/**
* end vibration effect
*/
void AHapticsManager::endVibrationEffect(int deviceId) {
	UHapticThreadInput::getInst().setMaxVibrationForce(deviceId, 0);
}


/**
* start damping effect
*/
void AHapticsManager::startDampingEffect(int deviceId, float maxForce) {
	UHapticThreadInput::getInst().setMaxDampingForce(deviceId, maxForce);
}

/**
* end damping effect
*/
void AHapticsManager::endDampingEffect(int deviceId) {
	UHapticThreadInput::getInst().setMaxDampingForce(deviceId, 0);
}


/**
* start surface effect
*/
void AHapticsManager::startVirtualPort(int deviceId, float maxForce, FVector port) {
	UHapticThreadInput::getInst().setMaxSurfaceForce(deviceId, maxForce);
	UHapticThreadInput::getInst().setPortOrientation(deviceId, port);
}

/**
* end surface effect
*/
void AHapticsManager::endVirtualPort(int deviceId) {
	UHapticThreadInput::getInst().setMaxSurfaceForce(deviceId, 0);
}

/**
* start viscosity effect
*/
void AHapticsManager::startViscosityEffect(int deviceId, float maxForce) {
	UHapticThreadInput::getInst().setMaxViscosityForce(deviceId, maxForce);
}

/**
* end viscosity effect
*/
void AHapticsManager::endViscosityEffect(int deviceId) {
	UHapticThreadInput::getInst().setMaxViscosityForce(deviceId, 0);
}


/**
* start magnetic effect
*/
void AHapticsManager::startMagneticEffect(int deviceId, float maxForce) {
	UHapticThreadInput::getInst().setMaxMagneticForce(deviceId, maxForce);
}

/**
* end magnetic effect
*/
void AHapticsManager::endMagneticEffect(int deviceId) {
	UHapticThreadInput::getInst().setMaxMagneticForce(deviceId, 0);
}

/**
* gets the current position of the haptic device end affector
*/
FVector AHapticsManager::getHapticDevicePosition(int deviceId) {
	return UHapticThreadOutput::getInst().getHapticCursorPosition(deviceId);
}

/**
* get the current linear velocity of the haptic device end affector
*/
FVector AHapticsManager::getHapticDeviceLinearVelocity(int deviceId) {
	return UHapticThreadOutput::getInst().getHapticCursorLinearVelocity(deviceId);
}

/**
* get the current angular velocity of the haptic device end affector
*/
FVector AHapticsManager::getHapticDeviceAngularVelocity(int deviceId) {
	return UHapticThreadOutput::getInst().getHapticCursorAngularVelocity(deviceId);
}

/**
* get the current rotation of the haptic device end affector
*/
FMatrix AHapticsManager::getHapticDeviceRotation(int deviceId) {
	return UHapticThreadOutput::getInst().getHapticCursorRotation(deviceId);
}

/**
* get the current rotation of the haptic device end affector as an unreal rotator
*/
FRotator AHapticsManager::getHapticDeviceRotationAsUnrealRotator(int deviceId) {
	FMatrix rotation = UHapticThreadOutput::getInst().getHapticCursorRotation(deviceId);
	FVector euler = rotation.Rotator().Euler();
	return FRotator(calibratedDirectionRotX*(calibratedRotatorX - euler.X), calibratedDirectionRotY * (calibratedRotatorY - euler.Z), calibratedDirectionRotZ * (calibratedRotatorZ - euler.Y));
}

/**
* get the current position of the haptic device end affector in unreal coordinates
*/
FVector AHapticsManager::getHapticDevicePositionInUnrealCoordinates(int deviceId) {
	FVector position = UHapticThreadOutput::getInst().getHapticCursorPosition(deviceId);
	return FVector(calibratedDirectionPosX * (calibratedX - position.X * 1000), calibratedDirectionPosY * (calibratedY + position.Y * 1000), calibratedDirectionPosZ * (calibratedZ + position.Z * 500));
}

/**
* broad casts the new haptic data as a multicast delegate
*/
void AHapticsManager::broadCastNewHapticData(FVector position, FMatrix rotation, FVector linearVelocity, FVector angularVelocity, int deviceId) {
	OnHapticTick.Broadcast(position, rotation, linearVelocity, angularVelocity, deviceId);
}

void AHapticsManager::calibrate(int deviceId, float x, float y, float z, float rotatorX, float rotatorY, float rotatorZ, float dX, float dY, float dZ, float dRx, float dRy, float dRz) {
	calibratedX = x;
	calibratedY = y;
	calibratedZ = z;

	calibratedRotatorX = rotatorX;
	calibratedRotatorY = rotatorY;
	calibratedRotatorZ = rotatorZ;

	calibratedDirectionPosX = dX;
	calibratedDirectionPosY = dY;
	calibratedDirectionPosZ = dZ;

	calibratedDirectionRotX = dRx;
	calibratedDirectionRotY = dRy;
	calibratedDirectionRotZ = dRz;

}

void AHapticsManager::createPrimitiveObject(FVector objectPosition, FVector orientation, float stiffness, FString type, float size) {
	UHapticThreadInput::getInst().setPrimitiveObjectProperties(primitiveObjectCount, objectPosition, orientation, stiffness, type, size);
	primitiveObjectCount++;

}
void AHapticsManager::setPrimitiveObjectPositionAsDevicePos() {
	
	primitiveObjectPosSetAsDevicePos = true;
}

bool AHapticsManager::getPrimitiveObjectPositionAsDevicePos() {

	return	primitiveObjectPosSetAsDevicePos;
}

int AHapticsManager::getPrimitiveObjectCount() {
	return primitiveObjectCount;
}


