/* Copyright (C) 2022 Hoang Ha Le
*
* This software may be distributed under the terms
* of the MIT license. See the LICENSE file for details.
*/
#pragma once

#include "CoreMinimal.h"
#include "ISpeedOp.h"

class HapticDeviceButtonHandler
{
public:
	HapticDeviceButtonHandler(ISpeedOp* speedOp);
	bool button1Clicked(int deviceId);
	bool button2Clicked(int deviceId);

private:
	bool button1AlreadyPressed = false;
	bool button2AlreadyPressed = false;
	ISpeedOp* speedOp;

};
