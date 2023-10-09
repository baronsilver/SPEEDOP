/* Copyright (C) 2022 Hoang Ha Le
*
* This software may be distributed under the terms
* of the MIT license. See the LICENSE file for details.
*/
#include "HapticThread.h"
#include "ISpeedOp.h"
#include "HapticDeviceButtonHandler.h"
#include "HapticsManager.h"
#include "Async.h"
#include "HapticThreadOutput.h"
#include "HapticThreadInput.h"
/**
* this is the haptic thread that retrieves the data from the device and sets forces and torques
*/

void FHapticThread::DoWork()
{
	HapticDeviceButtonHandler buttonHandler(&speedOp);
	speedOp.connect();
	int numOfDevices = speedOp.getNumOfDevices();
	int objectCount = hapticsManager->getPrimitiveObjectCount();
	for (int i = 0; i < objectCount; i++) {
		float stiffness = UHapticThreadInput::getInst().getPrimitiveObjectStiffness(i);
		float size = UHapticThreadInput::getInst().getPrimitiveObjectSize(i);
		FVector rot = UHapticThreadInput::getInst().getPrimitiveObjectOrientation(i);
		FVector pos = UHapticThreadInput::getInst().getPrimitiveObjectPosition(i);
		FString type = UHapticThreadInput::getInst().getPrimitiveObjectType(i);

		speedOp.createPrimitiveObject(pos, rot, stiffness, type, size);
	}
	while (UHapticThreadInput::getInst().shouldThreadRun()) {
		for (int i = 0; i < numOfDevices; i++)
		{
			FVector position = speedOp.getPosition(i);
			FMatrix rotation = speedOp.getRotation(i);
			FVector linearVelocity = speedOp.getLinearVelocity(i);
			FVector angularVelocity = speedOp.getAngularVelocity(i);

			UHapticThreadOutput::getInst().setHapticCursorPosition(position, i);
			UHapticThreadOutput::getInst().setHapticCursorRotation(rotation, i);
			UHapticThreadOutput::getInst().setHapticCursorAngularVelocity(angularVelocity, i);
			UHapticThreadOutput::getInst().setHapticCursorLinearVelocity(linearVelocity, i);

			AHapticsManager* hapticManagerPointer = hapticsManager;
			bool button1clicked = buttonHandler.button1Clicked(i);
			bool button2clicked = buttonHandler.button2Clicked(i);
			AsyncTask(ENamedThreads::GameThread, [hapticManagerPointer, button1clicked, button2clicked, position, rotation, i]() {
				if (button1clicked) {
					hapticManagerPointer->button1Clicked(i);
				}
				else if (button2clicked) {
					hapticManagerPointer->button2Clicked(i);
				}

				});

			hapticsManager->broadCastNewHapticData(position, rotation, linearVelocity, angularVelocity, i);

			if (hapticsManager->getPrimitiveObjectPositionAsDevicePos()) {
				speedOp.setPrimitiveObjectPositionAsDevicePos(0);
				hapticsManager->primitiveObjectPosSetAsDevicePos = false;
			}
			
			float maxVibrationForce = UHapticThreadInput::getInst().getMaxVibrationForce(i);
	
			float maxDampingForce = UHapticThreadInput::getInst().getMaxDampingForce(i);

			float maxSurfaceForce = UHapticThreadInput::getInst().getMaxSurfaceForce(i);

			float maxViscosityForce = UHapticThreadInput::getInst().getMaxViscosityForce(i);

			float maxMagneticForce = UHapticThreadInput::getInst().getMaxMagneticForce(i);
			FVector port = UHapticThreadInput::getInst().getPortOrientation(i);

			if (maxVibrationForce > 0)
			{
				speedOp.createVibrationEffect(i, maxVibrationForce);
			}
			if (maxDampingForce > 0)
			{
				speedOp.createDampingEffect(i, maxDampingForce);
			}
			if (maxSurfaceForce > 0)
			{
				speedOp.createVirtualPort(i, position, port, maxSurfaceForce);
			}
			if (maxViscosityForce > 0) 
			{
				speedOp.createViscosityEffect(i, maxViscosityForce);
			}

			if (maxMagneticForce > 0)
			{
				speedOp.createMagneticEffect(i, position, maxMagneticForce);
			}

			if (maxSurfaceForce == 0) {
				speedOp.createVirtualPort(i, FVector(0, 0, 0), FVector(0, 0, 0), 0);
			}

			if (maxMagneticForce == 0) {
				speedOp.createMagneticEffect(i, FVector(0,0,0), 0);
			}

			//else if (maxVibrationForce == 0 && maxDampingForce == 0 && maxSurfaceForce == 0) {
			//	speedOp.setForce(FVector(0,0,0), i);
			//}

			speedOp.updateHaptics(i);
			/*FVector force = UHapticThreadInput::getInst().getForceToApply(i);
			FVector torque = UHapticThreadInput::getInst().getTorqueToApply(i);*/
			
		}
		

	}
	
	speedOp.disconnect();
	
}