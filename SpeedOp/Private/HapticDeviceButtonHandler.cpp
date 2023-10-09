/* Copyright (C) 2022 Hoang Ha Le
*
* This software may be distributed under the terms
* of the MIT license. See the LICENSE file for details.
*/
#include "HapticDeviceButtonHandler.h"
#include "ISpeedOp.h"

/**
 * creates an instance of the button handler
*/
HapticDeviceButtonHandler::HapticDeviceButtonHandler(ISpeedOp* hap)
{
	speedOp = hap;
}

/** checks if the first button of the haptic device is clicked. This Method only returns true ones when the button is pressed until it is released again.
*/
bool HapticDeviceButtonHandler::button1Clicked(int deviceId) {
	bool button1Pressed = speedOp->isFirstButtonActivated(deviceId);
	if (button1Pressed && !button1AlreadyPressed) {
		button1AlreadyPressed = true;
		return true;
	}
	else if (button1Pressed && button1AlreadyPressed) {
		return false;
	}
	else if (!button1Pressed && button1AlreadyPressed) {
		button1AlreadyPressed = false;
	}
	return false;
}

/** checks if the second button of the haptic device is clicked. This Method only returns true ones when the button is pressed until it is released again.
*/
bool HapticDeviceButtonHandler::button2Clicked(int deviceId) {
	bool button2Pressed = speedOp->isSecondButtonActivated(deviceId);
	if (button2Pressed && !button2AlreadyPressed) {
		button2AlreadyPressed = true;
		return true;
	}
	else if (button2Pressed && button2AlreadyPressed) {
		return false;
	}
	else if (!button2Pressed && button2AlreadyPressed) {
		button2AlreadyPressed = false;
	}
	return false;
}