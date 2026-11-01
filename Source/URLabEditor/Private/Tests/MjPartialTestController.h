// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// The fixture the ApplyControls ordering tests are run against
// (URLab.Controllers.ApplyControlsPartial).
//
// A controller that drives one actuator and leaves the rest, as an arm's IK
// does on an articulation composed of a mobile base and an arm. What
// ApplyControls writes for the actuators it leaves is the question.

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Controllers/MjArticulationController.h"
#include "MjPartialTestController.generated.h"

UCLASS(NotBlueprintable)
class UMjPartialTestController : public UMjArticulationController
{
	GENERATED_BODY()

public:
	/** The one actuator it writes (a compiled id), and what it writes there. */
	int32  DrivenId = INDEX_NONE;
	double Output = 0.0;

	virtual void ComputeAndApply(mjModel* m, mjData* d, uint8 Source) override;
};
