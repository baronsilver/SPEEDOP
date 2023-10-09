/* Copyright (C) 2022 Hoang Ha Le
*
* This software may be distributed under the terms
* of the MIT license. See the LICENSE file for details.
*/
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "HapticsManager.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE_FiveParams(FNewHapticData, FVector, Position, FMatrix, Rotation, FVector, LinearVelocity, FVector, AngularVelocity, int, DeviceId);

UCLASS(BlueprintType)
class SPEEDOP_API AHapticsManager : public AActor
{
	GENERATED_BODY()
	
public:	

	AHapticsManager();

	/**
	* the multicast delegate that is fired every tick and provides the up to date haptic data
	*/
	UPROPERTY(BlueprintAssignable)
		FNewHapticData OnHapticTick;
private:
	float calibratedX = 0;
	float calibratedY = 0;
	float calibratedZ = 0;

	float calibratedRotatorX = 0;
	float calibratedRotatorY = 0;
	float calibratedRotatorZ = 0;

	float calibratedDirectionPosX = 1;
	float calibratedDirectionPosY = 1;
	float calibratedDirectionPosZ = 1;

	float calibratedDirectionRotX = 1;
	float calibratedDirectionRotY = 1;
	float calibratedDirectionRotZ = 1;
	int primitiveObjectCount = 0;
	



protected:

	virtual void BeginPlay() override;
	virtual void EndPlay(EEndPlayReason::Type type) override;
	

public:	

	virtual void Tick(float DeltaTime) override;
	
	bool primitiveObjectPosSetAsDevicePos = false;

	/**
	* broad casts the new haptic data as a multicast delegate
	*/
	void broadCastNewHapticData(FVector position, FMatrix rotation, FVector linearVelocity, FVector angularVelocity, int deviceId);

	bool getPrimitiveObjectPositionAsDevicePos();

	/**
	* set the force that should be applied to the haptic device in the next tick of the haptic thread
	*/
	UFUNCTION(BlueprintCallable)
	static void setForceToApply(FVector force, int deviceId);

	/**
	* set the torque that should be applied to the haptic device in the next tick of the haptic thread
	*/
	UFUNCTION(BlueprintCallable)
	static void setTorqueToApply(FVector torque, int deviceId);

	/**
	* get the current position of the haptic device end affector
	*/
	UFUNCTION(BlueprintCallable)
	FVector getHapticDevicePosition(int deviceId);

	/**
	* get the current position of the haptic device end affector in unreal coordinates
	*/
	UFUNCTION(BlueprintCallable)
	FVector getHapticDevicePositionInUnrealCoordinates(int deviceId);

	/**
	* get the current rotation of the haptic device end affector
	*/
	UFUNCTION(BlueprintCallable)
	FMatrix getHapticDeviceRotation(int deviceId);

	/**
	* get the current rotation of the haptic device end affector as an unreal rotator
	*/
	UFUNCTION(BlueprintCallable)
	FRotator getHapticDeviceRotationAsUnrealRotator(int deviceId);

	/**
	* get the current angular velocity of the haptic device end affector
	*/
	UFUNCTION(BlueprintCallable)
	FVector getHapticDeviceAngularVelocity(int deviceId);

	/**
	* get the current linear velocity of the haptic device end affector
	*/
	UFUNCTION(BlueprintCallable)
	FVector getHapticDeviceLinearVelocity(int deviceId);
		
	/**
	* start vibration effect
	*/
	UFUNCTION(BlueprintCallable)
	void startVibrationEffect(int deviceId, float maxForce);

	/**
	* end vibration effect
	*/
	UFUNCTION(BlueprintCallable)
	void endVibrationEffect(int deviceId);

	/**
	* start damping effect
	*/
	UFUNCTION(BlueprintCallable)
	void startDampingEffect(int deviceId, float maxForce);

	/**
	* end damping effect
	*/
	UFUNCTION(BlueprintCallable)
	void endDampingEffect(int deviceId);

	/**
	* start virtual port
	*/
	UFUNCTION(BlueprintCallable)
	void startVirtualPort(int deviceId, float maxForce, FVector port);

	/**
	* end virtual port
	*/
	UFUNCTION(BlueprintCallable)
	void endVirtualPort(int deviceId);

	/**
	* start viscosity effect
	*/
	UFUNCTION(BlueprintCallable)
		void startViscosityEffect(int deviceId, float maxForce);

	/**
	* end viscosity effect
	*/
	UFUNCTION(BlueprintCallable)
		void endViscosityEffect(int deviceId);

	/**
	* start magnetic effect
	*/
	UFUNCTION(BlueprintCallable)
		void startMagneticEffect(int deviceId, float maxForce);

	/**
	* end magnetic effect
	*/
	UFUNCTION(BlueprintCallable)
		void endMagneticEffect(int deviceId);
	
	
	/**
	* fire an event when the button 1 is clicked
	*/
	UFUNCTION(BlueprintImplementableEvent, meta = (DisplayName = "Button1Clicked"))
	void button1Clicked(int deviceId);

	/**
	* fire an event when the button 2 is clicked
	*/
	UFUNCTION(BlueprintImplementableEvent, meta = (DisplayName = "Button2Clicked"))
	void button2Clicked(int deviceId);

	/**
	* calibrate device
	*/
	UFUNCTION(BlueprintCallable)
	void calibrate(int deviceId, float x, float y, float z, float rotatorX, float rotatorY, float rotatorZ, float dX, float dY, float dZ, float dRx, float dRy, float dRz);

	/**
	* create primitive object
	*/
	UFUNCTION(BlueprintCallable)
	void createPrimitiveObject(FVector objectPosition, FVector orientation, float stiffness, FString type, float size);

	/**
* set primitive object pos
*/
	UFUNCTION(BlueprintCallable)
	void setPrimitiveObjectPositionAsDevicePos();
		
	/**
	* return primitive object count
	*/
	UFUNCTION(BlueprintCallable)
	int getPrimitiveObjectCount();
};
