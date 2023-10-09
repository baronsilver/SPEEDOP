/* Copyright (C) 2018 Sinan Demirtas
*
* This software may be distributed under the terms
* of the MIT license. See the LICENSE file for details.
*/
#pragma once
#include "AsyncWork.h"
#include "ISpeedOp.h"

class AHapticsManager;

class FHapticThread : public FNonAbandonableTask
{
	friend class FAutoDeleteAsyncTask<FHapticThread>;

public:
	FHapticThread(ISpeedOp& speedOpModule, AHapticsManager* hManager) :
		speedOp(speedOpModule),
		hapticsManager(hManager)
	{}

protected:
	ISpeedOp& speedOp;
	AHapticsManager* hapticsManager;

	/**
	* this is the haptic thread that retrieves the data from the device and sets forces and torques
	*/
	void DoWork();

	FORCEINLINE TStatId GetStatId() const
	{
		RETURN_QUICK_DECLARE_CYCLE_STAT(FHapticThread, STATGROUP_ThreadPoolAsyncTasks);
	}
};