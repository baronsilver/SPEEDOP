/* Copyright (C) 2022 Hoang Ha Le
*
* This software may be distributed under the terms
* of the MIT license. See the LICENSE file for details.
*/
#pragma once
#include "CoreMinimal.h"

class UHapticThreadInput
{

private:
	bool runThread = true;
	FVector forceToApply[MAX_DEVICES];
	FVector torqueToApply[MAX_DEVICES];
	FVector objectOrientation[MAX_DEVICES];
	bool isVibrationEffectCreated[MAX_DEVICES];
	bool isDampingEffectCreated[MAX_DEVICES];
	float maxVibrationForce[MAX_DEVICES];
	float maxDampingForce[MAX_DEVICES];
	float maxSurfaceForce[MAX_DEVICES];
	float maxViscosityForce[MAX_DEVICES];
	float maxMagneticForce[MAX_DEVICES];
	float primitiveObjectStiffness[MAX_PRIMITIVE_OBJECT];
	
	FVector portOrientation[MAX_DEVICES];
	int primitiveObjectId[MAX_PRIMITIVE_OBJECT];
	float primitiveObjectSize[MAX_PRIMITIVE_OBJECT];
	FString primitiveObjectType[MAX_PRIMITIVE_OBJECT];
	FVector primitiveObjectPosition[MAX_PRIMITIVE_OBJECT];
	FVector primitiveObjectOrientation[MAX_PRIMITIVE_OBJECT];

public:
	static UHapticThreadInput& getInst() {
		static UHapticThreadInput instance;
		return instance;
	}
	bool shouldThreadRun();
	void setRunThread(bool);
	void setForceToApply(FVector force, int deviceId);
	void setTorqueToApply(FVector torque, int deviceId);
	void setMaxVibrationForce(int deviceId, float maxForce);
	void setMaxSurfaceForce(int deviceId, float maxForce);
	void setMaxDampingForce(int deviceId, float maxForce);
	void setMaxViscosityForce(int deviceId, float maxForce);
	void setMaxMagneticForce(int deviceId, float maxForce);
	void setPortOrientation(int deviceId, FVector port);
	void setPrimitiveObjectProperties(int objectId, FVector position, FVector orientation, float stiffness, FString type, float size);



	float getMaxVibrationForce(int deviceId);
	float getMaxDampingForce(int deviceId);
	float getMaxSurfaceForce(int deviceId);
	float getMaxViscosityForce(int deviceId);
	float getMaxMagneticForce(int deviceId);

	FVector getPortOrientation(int deviceId);
	FVector getPrimitiveObjectOrientation(int objectId);
	FVector getPrimitiveObjectPosition(int objectId);
	float getPrimitiveObjectStiffness(int objectId);
	float getPrimitiveObjectSize(int objectId);
	FString getPrimitiveObjectType(int objectId);
	FVector getForceToApply(int deviceId);
	FVector getTorqueToApply(int deviceId);
};