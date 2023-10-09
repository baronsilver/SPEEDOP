/* Copyright (C) 2022 Hoang Ha Le
*
* This software may be distributed under the terms
* of the MIT license. See the LICENSE file for details.
*/
#pragma once
#include "CoreMinimal.h"

class UHapticThreadOutput
{
private:
	FVector hapticCursorPosition[MAX_DEVICES];
	FMatrix hapticCursorRotation[MAX_DEVICES];
	FVector hapticCursorLinearVelocity[MAX_DEVICES];
	FVector hapticCursorAngularVelocity[MAX_DEVICES];

public:
	static UHapticThreadOutput& getInst() {
		static UHapticThreadOutput instance;
		return instance;
	}
	UHapticThreadOutput();

	FVector getHapticCursorLinearVelocity(int deviceId);
	FVector getHapticCursorAngularVelocity(int deviceId);
	void setHapticCursorPosition(FVector position, int deviceId);
	FVector getHapticCursorPosition(int deviceId);
	void setHapticCursorRotation(FMatrix rotation, int deviceId);
	FMatrix getHapticCursorRotation(int deviceId);
	void setHapticCursorAngularVelocity(FVector angularVelocity, int deviceId);
	void setHapticCursorLinearVelocity(FVector linearVelocity, int deviceId);
};



