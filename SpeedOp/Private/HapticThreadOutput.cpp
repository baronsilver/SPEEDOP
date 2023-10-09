/* Copyright (C) 2022 Hoang Ha Le
*
* This software may be distributed under the terms
* of the MIT license. See the LICENSE file for details.
*/
#include "HapticThreadOutput.h"
#include "ISpeedOp.h"

/**
* initializes the class members when the class is created
*/
UHapticThreadOutput::UHapticThreadOutput()
{

	for (int i = 0; i < MAX_DEVICES; i++)
	{
		hapticCursorPosition[i] = FVector(0, 0, 0);
		hapticCursorLinearVelocity[i] = FVector(0, 0, 0);
		hapticCursorAngularVelocity[i] = FVector(0, 0, 0);
		hapticCursorRotation[i] = FMatrix(FVector(0, 0, 0), FVector(0, 0, 0), FVector(0, 0, 0), FVector(0, 0, 0));
	}

}

/**
* sets the current position of the haptic device end affector
*/
void UHapticThreadOutput::setHapticCursorPosition(FVector position, int deviceId) {
	hapticCursorPosition[deviceId] = position;
}

/**
* gets the current position of the haptic device end affector
*/
FVector UHapticThreadOutput::getHapticCursorPosition(int deviceId) {
	return hapticCursorPosition[deviceId];
}

/**
* sets the current rotation of the haptic device end affector
*/
void UHapticThreadOutput::setHapticCursorRotation(FMatrix rotation, int deviceId) {
	hapticCursorRotation[deviceId] = rotation;
}

/**
* gets the current rotation of the haptic device end affector
*/
FMatrix UHapticThreadOutput::getHapticCursorRotation(int deviceId) {
	return hapticCursorRotation[deviceId];
}

/**
* gets the current linear velocity of the haptic device end affector
*/
FVector UHapticThreadOutput::getHapticCursorLinearVelocity(int deviceId) {
	return hapticCursorLinearVelocity[deviceId];
}

/**
* gets the current angular velocity of the haptic device end affector
*/
FVector UHapticThreadOutput::getHapticCursorAngularVelocity(int deviceId) {
	return hapticCursorAngularVelocity[deviceId];
}

/**
* sets the current angular velocity of the haptic device end affector
*/
void UHapticThreadOutput::setHapticCursorAngularVelocity(FVector angularVelocity, int deviceId){
	hapticCursorAngularVelocity[deviceId] = angularVelocity;
}

/**
* sets the current linear velocity of the haptic device end affector
*/
void UHapticThreadOutput::setHapticCursorLinearVelocity(FVector linearVelocity, int deviceId) {
	hapticCursorLinearVelocity[deviceId] = linearVelocity;
}
