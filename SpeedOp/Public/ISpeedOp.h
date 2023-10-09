/* Copyright (C) 2022 Hoang Ha Le
*
* This software may be distributed under the terms
* of the MIT license. See the LICENSE file for details.
*/
#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleInterface.h"
#include "Modules/ModuleManager.h"
const int MAX_DEVICES = 16;
const int MAX_PRIMITIVE_OBJECT = 100;

/**
 * The public interface to the SpeedOp module.
 */
class ISpeedOp : public IModuleInterface
{
	
public:

	/**
	 * @return Returns singleton instance, loading the module on demand if needed
	 */
	static inline ISpeedOp& Get()
	{
		return FModuleManager::LoadModuleChecked< ISpeedOp >("SpeedOp");
	}

	/**
	 * Checks to see if this module is loaded and ready.  It is only valid to call Get() if IsAvailable() returns true.
	 *
	 * @return True if the module is loaded and ready to use
	 */
	static inline bool IsAvailable()
	{
		return FModuleManager::Get().IsModuleLoaded( "SpeedOp" );
	}

	virtual FVector getForce(int deviceId) = 0;
	virtual FVector getTorque(int deviceId) = 0;
	virtual bool isFirstButtonActivated(int deviceId) = 0;
	virtual bool isSecondButtonActivated(int deviceId) = 0;
	virtual FVector getLinearVelocity(int deviceId) = 0;
	virtual FVector getAngularVelocity(int deviceId) = 0;
	virtual FVector getPosition(int deviceId) = 0;
	virtual FMatrix getRotation(int deviceId) = 0;
	virtual void createVibrationEffect(int deviceId, float maxForce) = 0;
	virtual void createDampingEffect(int deviceId, float maxForce) = 0;
	virtual void createVirtualPort(int deviceId, FVector desiredPos, FVector port, float maxForce) = 0;
	virtual void createViscosityEffect(int deviceId, float maxForce) = 0;
	virtual void createMagneticEffect(int deviceId, FVector currentPosition, float intensityScale) = 0;
	virtual void createPrimitiveObject(FVector objectPosition, FVector orientation, float stiffness, FString type, float size) = 0;
	virtual void setPrimitiveObjectPositionAsDevicePos(int objectId) = 0;
	virtual void setForce(FVector force, int deviceId) = 0;
	virtual void setTorque(FVector torque, int deviceId) = 0;
	virtual void setForceAndTorque(FVector force, FVector torque, int deviceId) = 0;
	virtual bool connect() = 0;
	virtual void disconnect() = 0;
	virtual int  getNumOfDevices() = 0;
	virtual void updateHaptics(int deviceId) = 0;
};

