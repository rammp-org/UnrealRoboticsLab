// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// AMjArticulation::ApplyControls under a controller that drives only some of
// the articulation's actuators.
//
// The staged controls are written first and the controller after, so the
// controller's output wins where it writes and every other owned actuator keeps
// its staged target. Either regression shows here: returning after the
// controller (the others never get their targets) or writing the staged
// controls last (they overwrite the controller).

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#if WITH_EDITOR

#include "MjPartialTestController.h"
#include "MjTestHelpers.h"

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Gen/Elements/Actuators/MjActuator.gen.h"
#include "MuJoCo/Gen/Elements/Actuators/MjMotor.gen.h"
#include "MuJoCo/Gen/Elements/Joints/MjJoint.gen.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>
THIRD_PARTY_INCLUDES_END

void UMjPartialTestController::ComputeAndApply(mjModel* m, mjData* d, uint8 /*Source*/)
{
	if (m != nullptr && d != nullptr && DrivenId >= 0 && DrivenId < m->nu)
	{
		d->ctrl[DrivenId] = Output;
	}
}

namespace
{

/**
 * Two motors on the session's hinge, and the partial controller on the
 * articulation before it compiles, so the compile binds it as the
 * articulation's controller.
 */
void ConfigureTwoMotorsAndPartialController(FMjUESession& Sess)
{
	Sess.Joint->SetType(EMjJointType::hinge);
	UMjActuator* Section = Sess.Add<UMjActuator>(Sess.Robot->Spec);
	if (Section == nullptr)
	{
		return;
	}
	for (const TCHAR* Name : { TEXT("DrivenMotor"), TEXT("StagedMotor") })
	{
		if (UMjMotor* A = Sess.Add<UMjMotor>(Section, Name))
		{
			A->SetJoint(Sess.Joint->MjName.GetValue());
		}
	}
	UMjPartialTestController* Controller = NewObject<UMjPartialTestController>(Sess.Robot);
	Controller->RegisterComponent();
}

} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjApplyControlsPartialController,
	"URLab.Controllers.ApplyControlsPartial",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjApplyControlsPartialController::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init(&ConfigureTwoMotorsAndPartialController))
	{
		AddInfo(FString::Printf(TEXT("session did not stand up: %s"), *S.LastError));
		return true;
	}

	mjModel* m = S.Manager->PhysicsEngine ? S.Manager->PhysicsEngine->GetModel() : nullptr;
	mjData* d = S.Manager->PhysicsEngine ? S.Manager->PhysicsEngine->GetData() : nullptr;
	UMjPartialTestController* Controller = S.Robot->FindComponentByClass<UMjPartialTestController>();
	const TArray<int32> Owned = S.Robot->GetOwnedActuatorIds();
	if (m == nullptr || d == nullptr || Controller == nullptr || Owned.Num() < 2)
	{
		AddError(TEXT("the session compiled without two owned actuators and the controller"));
		S.Cleanup();
		return false;
	}
	TestTrue(TEXT("the compile bound the controller"), Controller->IsBound());

	const int32 Driven = Owned[0];
	const int32 Staged = Owned[1];
	S.Robot->ControlSource = 0;
	S.Robot->StageNetworkControl(Driven, 0.25);
	S.Robot->StageNetworkControl(Staged, 0.75);
	Controller->DrivenId = Driven;
	Controller->Output = -0.5;

	// The controller's output where it writes; the staged target everywhere else.
	d->ctrl[Driven] = d->ctrl[Staged] = 99.0;
	S.Robot->ApplyControls(m, d, /*bSkipController=*/false);
	TestEqual(TEXT("the controller's actuator takes its output"), (double)d->ctrl[Driven], -0.5, 1e-9);
	TestEqual(TEXT("the actuator it leaves takes its staged target"), (double)d->ctrl[Staged], 0.75, 1e-9);

	// Disabled, it writes nothing: every actuator takes its staged target.
	Controller->bEnabled = false;
	d->ctrl[Driven] = d->ctrl[Staged] = 99.0;
	S.Robot->ApplyControls(m, d, /*bSkipController=*/false);
	TestEqual(TEXT("disabled: the driven actuator takes its staged target"), (double)d->ctrl[Driven], 0.25, 1e-9);
	TestEqual(TEXT("disabled: the other takes its staged target"), (double)d->ctrl[Staged], 0.75, 1e-9);

	// Skipped, as the bridge's raw mode asks, nothing is written at all.
	Controller->bEnabled = true;
	d->ctrl[Driven] = d->ctrl[Staged] = 99.0;
	S.Robot->ApplyControls(m, d, /*bSkipController=*/true);
	TestEqual(TEXT("skipped: the driven actuator is left alone"), (double)d->ctrl[Driven], 99.0, 1e-9);
	TestEqual(TEXT("skipped: the other is left alone"), (double)d->ctrl[Staged], 99.0, 1e-9);

	S.Cleanup();
	return true;
}

#endif // WITH_EDITOR
