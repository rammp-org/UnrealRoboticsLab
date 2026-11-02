// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0.

// UMjBody::ApplyForce against mjData. It used to write the torque into the
// force slots and the force into the torque slots, scaled by a cm-to-m factor
// that does not apply to newtons: a 10 N push arrived as a 0.1 N.m twist about
// the wrong axis, and nothing read the wrench back. These pin the layout
// MuJoCo reads (force first, torque second), the axis mirror for a vector and
// for a pseudovector, the readback, and the lifetime the header promises:
// held until replaced or cleared.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Tests/MjTestHelpers.h"

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Elements/MjBody.h"

// ============================================================================
// URLab.Runtime.ApplyForceLandsInXfrcApplied
//   The six numbers MuJoCo sees for a known force and torque.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBodyApplyForceLayoutTest,
	"URLab.Runtime.ApplyForceLandsInXfrcApplied",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjBodyApplyForceLayoutTest::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		return false;
	}

	UMjPhysicsEngine* Engine = S.Manager->PhysicsEngine;
	mjModel* M = Engine->GetModel();
	mjData* D = Engine->GetData();
	UMjBody* Body = S.Body;
	if (!M || !D || !Body || !Body->GetBoundId().IsSet())
	{
		AddError(TEXT("the session's body did not bind"));
		S.Cleanup();
		return false;
	}
	const int32 BodyId = Body->GetBoundId().GetValue();
	TestTrue(TEXT("the body is a dynamic body, not the world"), BodyId > 0 && BodyId < M->nbody);

	// Unreal axes in; MuJoCo's mirror out. A force is a vector (Y flips), a
	// torque a pseudovector (X and Z flip), and the force comes first.
	Body->ApplyForce(FVector(10.0f, 20.0f, 30.0f), FVector(1.0f, 2.0f, 3.0f));
	Engine->StepSync(1);

	const double* Xfrc = D->xfrc_applied + 6 * BodyId;
	TestEqual(TEXT("force x is the first slot, unscaled"), Xfrc[0], 10.0, 1e-9);
	TestEqual(TEXT("force y is mirrored"), Xfrc[1], -20.0, 1e-9);
	TestEqual(TEXT("force z"), Xfrc[2], 30.0, 1e-9);
	TestEqual(TEXT("torque x is the fourth slot and flips under the mirror"), Xfrc[3], -1.0, 1e-9);
	TestEqual(TEXT("torque y keeps its sign"), Xfrc[4], 2.0, 1e-9);
	TestEqual(TEXT("torque z flips"), Xfrc[5], -3.0, 1e-9);

	// The getters read the published snapshot and undo the mirror.
	const FVector Force = Body->GetAppliedForce();
	const FVector Torque = Body->GetAppliedTorque();
	TestTrue(TEXT("the force reads back in Unreal axes"), Force.Equals(FVector(10.0f, 20.0f, 30.0f), 1e-4f));
	TestTrue(TEXT("the torque reads back in Unreal axes"), Torque.Equals(FVector(1.0f, 2.0f, 3.0f), 1e-4f));

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Runtime.ApplyForceHoldsUntilCleared
//   A wrench stays on for every step until the next ApplyForce or a
//   ClearForce, including steps whose xfrc_applied a pre-step callback
//   zeroed (UMjPerturbation does on every step).
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBodyApplyForceLifetimeTest,
	"URLab.Runtime.ApplyForceHoldsUntilCleared",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjBodyApplyForceLifetimeTest::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		return false;
	}

	UMjPhysicsEngine* Engine = S.Manager->PhysicsEngine;
	mjData* D = Engine->GetData();
	UMjBody* Body = S.Body;
	if (!D || !Body || !Body->GetBoundId().IsSet())
	{
		AddError(TEXT("the session's body did not bind"));
		S.Cleanup();
		return false;
	}
	const int32 BodyId = Body->GetBoundId().GetValue();
	double* Xfrc = D->xfrc_applied + 6 * BodyId;

	auto Sum = [Xfrc]() {
		double Total = 0.0;
		for (int32 i = 0; i < 6; ++i)
			Total += FMath::Abs(Xfrc[i]);
		return Total;
	};

	Body->ApplyForce(FVector(5.0f, 0.0f, 0.0f), FVector::ZeroVector);
	Engine->StepSync(1);
	TestEqual(TEXT("the wrench is on for the step after the call"), Xfrc[0], 5.0, 1e-9);

	Engine->StepSync(3);
	TestEqual(TEXT("and stays on for the steps after that"), Xfrc[0], 5.0, 1e-9);

	// What UMjPerturbation's pre-step callback does before every step.
	FMemory::Memzero(D->xfrc_applied, sizeof(mjtNum) * 6 * Engine->GetModel()->nbody);
	Engine->StepSync(1);
	TestEqual(TEXT("a step whose xfrc_applied was zeroed gets it back"), Xfrc[0], 5.0, 1e-9);

	// The latest of two submissions in one step is the one that lands, and
	// replaces the held one.
	Body->ApplyForce(FVector(6.0f, 0.0f, 0.0f), FVector::ZeroVector);
	Body->ApplyForce(FVector(7.0f, 0.0f, 0.0f), FVector::ZeroVector);
	Engine->StepSync(2);
	TestEqual(TEXT("the last submission before the step wins, and holds"), Xfrc[0], 7.0, 1e-9);

	Body->ClearForce();
	Engine->StepSync(1);
	TestEqual(TEXT("ClearForce takes it off"), Sum(), 0.0, 1e-9);
	Engine->StepSync(1);
	TestEqual(TEXT("and it stays off"), Sum(), 0.0, 1e-9);

	// ClearForce after a call in the same step: the clear wins.
	Body->ApplyForce(FVector(9.0f, 0.0f, 0.0f), FVector::ZeroVector);
	Body->ClearForce();
	Engine->StepSync(1);
	TestEqual(TEXT("ClearForce zeroes the pending wrench"), Sum(), 0.0, 1e-9);

	S.Cleanup();
	return true;
}
