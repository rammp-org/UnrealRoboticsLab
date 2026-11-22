// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Tests/MjTestHelpers.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "HAL/IConsoleManager.h"
#include "Engine/World.h"
#include "mujoco/mujoco.h"

// ============================================================================
// URLab.Thread.FrameLockedStepping
//   urlab.LockedFrameSeconds: the live worker steps exactly the batch the
//   manager grants each frame and nothing else. The headless session has no
//   world tick, so the test plays the frame itself: the manager's pre-actor-tick
//   handler, then (no actors) its post-actor-tick handler.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjFrameLockedStepping,
	"URLab.Thread.FrameLockedStepping",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

namespace
{
double PublishedSimTime(UMjPhysicsEngine* Engine)
{
	double T = -1.0;
	Engine->WithRenderState([&T](const FMjRenderSnapshot& Snap) { T = Snap.SimTime; });
	return T;
}

double LiveSimTime(UMjPhysicsEngine* Engine)
{
	FScopeLock Lock(&Engine->CallbackMutex);
	return Engine->m_data ? Engine->m_data->time : -1.0;
}

/** Waits (bounded) for the worker to have finished Count batches. */
bool WaitForBatches(UMjPhysicsEngine* Engine, uint64 Count)
{
	const double Start = FPlatformTime::Seconds();
	while (Engine->LockedBatchesDone.load(std::memory_order_acquire) < Count)
	{
		if (FPlatformTime::Seconds() - Start > 10.0)
			return false;
		FPlatformProcess::Sleep(0.001f);
	}
	return true;
}
} // namespace

bool FMjFrameLockedStepping::RunTest(const FString& Parameters)
{
	IConsoleVariable* Var = IConsoleManager::Get().FindConsoleVariable(TEXT("urlab.LockedFrameSeconds"));
	if (!Var)
	{
		AddError(TEXT("urlab.LockedFrameSeconds is not registered"));
		return false;
	}
	const float SavedSeconds = Var->GetFloat();

	FMjUESession S;
	if (!S.Init())
	{
		AddError(FString::Printf(TEXT("Init() failed: %s"), *S.LastError));
		return false;
	}
	UMjPhysicsEngine* Engine = S.Manager->PhysicsEngine;
	UWorld* World = S.World;
	const double Dt = S.Model()->opt.timestep;
	constexpr int32 N = 7;
	Var->Set((float)(N * Dt), ECVF_SetByConsole);

	auto Frame = [&]() {
		S.Manager->OnWorldPreActorTick(World, LEVELTICK_All, 0.016f);
		S.Manager->OnWorldPostActorTick(World, LEVELTICK_All, 0.016f);
	};

	// Armed before the worker starts (as BeginPlay does), it never takes a step nobody granted, unpaused or not.
	Engine->SetStepMode(EStepMode::Live);
	Engine->SetSimSpeed(100.0f);
	S.Manager->OnWorldPostActorTick(World, LEVELTICK_All, 0.016f);
	TestTrue(TEXT("the lock is armed before the worker runs"), Engine->bLockedStepping.load());
	Engine->SetPaused(false);
	Engine->RunMujocoAsync();
	const double WorkerWait = FPlatformTime::Seconds();
	while (!Engine->bWorkerRunning.load() && FPlatformTime::Seconds() - WorkerWait < 2.0)
	{
		FPlatformProcess::Sleep(0.001f);
	}
	if (!TestTrue(TEXT("the worker started"), Engine->bWorkerRunning.load()))
	{
		Var->Set(SavedSeconds, ECVF_SetByConsole);
		return false;
	}
	FPlatformProcess::Sleep(0.05f);
	TestEqual(TEXT("an armed worker with nothing granted takes no step"), LiveSimTime(Engine), 0.0);

	// Each frame: exactly one batch of N steps; the published state is the end of a whole batch, and only the
	// game thread publishes it (no worker step does). The batches are counted from the worker's own count.
	const uint64 Done0 = Engine->LockedBatchesDone.load();
	uint64 Granted = Done0;
	S.Manager->OnWorldPostActorTick(World, LEVELTICK_All, 0.016f); // the first grant
	++Granted;
	TestTrue(TEXT("the first batch was granted"),
		Engine->LockedStepBudget.load() > 0 || Engine->LockedBatchesDone.load() >= Granted);
	uint64 IdAtGrant = Engine->GetRenderFrameId();
	bool bExact = true;
	for (int32 K = 1; K <= 6; ++K)
	{
		if (!WaitForBatches(Engine, Granted))
		{
			AddError(FString::Printf(TEXT("batch %d did not finish"), K));
			bExact = false;
			break;
		}
		const double LiveAtBatchEnd = LiveSimTime(Engine);
		FPlatformProcess::Sleep(0.01f); // an idle worker between batches: nothing moves
		bExact &= TestEqual(FString::Printf(TEXT("batch %d: the worker idles after it"), K),
			LiveSimTime(Engine), LiveAtBatchEnd);
		bExact &= TestTrue(FString::Printf(TEXT("batch %d: exactly N steps"), K),
			FMath::IsNearlyEqual(LiveAtBatchEnd, K * N * Dt, 1e-9));
		bExact &= TestEqual(FString::Printf(TEXT("batch %d: no worker step published"), K),
			(int64)Engine->GetRenderFrameId(), (int64)IdAtGrant);
		Frame(); // publishes batch K, grants K + 1
		++Granted;
		IdAtGrant = Engine->GetRenderFrameId();
		bExact &= TestTrue(FString::Printf(TEXT("batch %d: published at the frame boundary"), K),
			FMath::IsNearlyEqual(PublishedSimTime(Engine), K * N * Dt, 1e-9));
	}
	TestTrue(TEXT("every frame advanced exactly one batch"), bExact);

	// Paused: no grants and no steps; unpaused, it carries on a batch per frame, none lost and none free-running.
	WaitForBatches(Engine, Granted); // batch 7
	Engine->SetPaused(true);
	for (int32 K = 0; K < 3; ++K)
	{
		Frame(); // the first publishes batch 7; none grants
	}
	const double PausedAt = LiveSimTime(Engine);
	TestTrue(TEXT("paused after 7 batches"), FMath::IsNearlyEqual(PausedAt, 7 * N * Dt, 1e-9));
	FPlatformProcess::Sleep(0.05f);
	TestEqual(TEXT("paused: the worker takes no step"), LiveSimTime(Engine), PausedAt);
	Engine->SetPaused(false);
	FPlatformProcess::Sleep(0.05f);
	TestEqual(TEXT("unpaused with nothing granted: still no step (no free run)"), LiveSimTime(Engine), PausedAt);
	for (int32 K = 0; K < 3; ++K)
	{
		Frame(); // grants 8, 9, 10
		++Granted;
		TestTrue(TEXT("a batch after the pause finishes"), WaitForBatches(Engine, Granted));
	}
	TestTrue(TEXT("after the pause: exactly one batch per frame"),
		FMath::IsNearlyEqual(LiveSimTime(Engine), 10 * N * Dt, 1e-9));

	// A pause that interrupts a batch holds the rest of it until a frame boundary releases it. Unpaused during the
	// actor ticks, it does not resume there: the post-tick releases it. Unpaused between frames, the next pre-tick
	// releases it and waits it out before any actor ticks. Long batches, so that the pause lands inside one.
	constexpr int32 BigN = 20000;
	Var->Set((float)(BigN * Dt), ECVF_SetByConsole);
	auto HeldBatch = [&](bool bUnpauseBetweenFrames) {
		const TCHAR* Case = bUnpauseBetweenFrames ? TEXT("unpaused between frames") : TEXT("unpaused in the actor ticks");
		const double Before = LiveSimTime(Engine);
		Frame(); // publishes the last batch, grants a long one
		++Granted;
		Engine->SetPaused(true);
		S.Manager->OnWorldPreActorTick(World, LEVELTICK_All, 0.016f); // paused under the batch: holds the rest
		if (Engine->LockedBatchesDone.load() >= Granted)
		{
			// It finished before the pause (the pre-tick published it): nothing to hold.
			TestFalse(FString::Printf(TEXT("%s: a finished batch is not held"), Case), Engine->bLockedBatchHeld.load());
			AddInfo(FString::Printf(TEXT("%s: the long batch finished before the pause; not exercised"), Case));
			Engine->SetPaused(false);
		}
		else if (!TestTrue(FString::Printf(TEXT("%s: an unfinished batch is held"), Case), Engine->bLockedBatchHeld.load()))
		{
			Engine->SetPaused(false);
		}
		else if (bUnpauseBetweenFrames)
		{
			S.Manager->OnWorldPostActorTick(World, LEVELTICK_All, 0.016f); // paused: keeps holding it
			TestTrue(FString::Printf(TEXT("%s: still held after a paused post-tick"), Case),
				Engine->bLockedBatchHeld.load());
			Engine->SetPaused(false);                                     // as a Slate click or an RPC would, between frames
			S.Manager->OnWorldPreActorTick(World, LEVELTICK_All, 0.016f); // releases it and waits it out
			TestTrue(FString::Printf(TEXT("%s: the pre-tick finished the batch"), Case),
				Engine->LockedBatchesDone.load() >= Granted);
			// A stall would have turned the lock off and the variable to 0.
			TestTrue(FString::Printf(TEXT("%s: without a stall, the lock still on"), Case),
				Engine->bLockedStepping.load() && Var->GetFloat() > 0.0f);
		}
		else
		{
			TestTrue(FString::Printf(TEXT("%s: held while unfinished"), Case), Engine->LockedBatchesDone.load() < Granted);
			Engine->SetPaused(false); // as an actor tick would
			const double HeldAt = LiveSimTime(Engine);
			FPlatformProcess::Sleep(0.05f);
			TestEqual(FString::Printf(TEXT("%s: no step until the post-tick"), Case), LiveSimTime(Engine), HeldAt);
			S.Manager->OnWorldPostActorTick(World, LEVELTICK_All, 0.016f); // releases the rest; no new grant
			TestFalse(FString::Printf(TEXT("%s: released by the post-tick"), Case), Engine->bLockedBatchHeld.load());
		}
		TestTrue(FString::Printf(TEXT("%s: the batch finishes"), Case), WaitForBatches(Engine, Granted));
		TestTrue(FString::Printf(TEXT("%s: exactly its steps, none lost or added"), Case),
			FMath::IsNearlyEqual(LiveSimTime(Engine), Before + BigN * Dt, 1e-6));
	};
	HeldBatch(false);
	HeldBatch(true);

	// The pre-tick itself waits out a batch still under way (here called straight after each grant, with a long
	// batch), then publishes exactly that whole batch.
	double Expect = LiveSimTime(Engine) + BigN * Dt;
	S.Manager->OnWorldPostActorTick(World, LEVELTICK_All, 0.016f); // grants a long batch
	++Granted;
	for (int32 K = 1; K <= 3; ++K)
	{
		S.Manager->OnWorldPreActorTick(World, LEVELTICK_All, 0.016f);
		TestTrue(FString::Printf(TEXT("pre-tick %d: waited the batch out"), K), Engine->LockedBatchesDone.load() >= Granted);
		TestTrue(FString::Printf(TEXT("pre-tick %d: published exactly the whole batch"), K),
			FMath::IsNearlyEqual(PublishedSimTime(Engine), Expect, 1e-6));
		S.Manager->OnWorldPostActorTick(World, LEVELTICK_All, 0.016f); // grants the next
		++Granted;
		Expect += BigN * Dt;
	}
	TestTrue(TEXT("the lock held through the waits"), Engine->bLockedStepping.load() && Var->GetFloat() > 0.0f);
	TestTrue(TEXT("the last long batch finishes"), WaitForBatches(Engine, Granted));
	Var->Set((float)(N * Dt), ECVF_SetByConsole);

	// Turned off: the worker runs on its own again.
	Var->Set(0.0f, ECVF_SetByConsole);
	Frame();
	TestFalse(TEXT("the lock is off with the variable at 0"), Engine->bLockedStepping.load());
	const double OffAt = LiveSimTime(Engine);
	FPlatformProcess::Sleep(0.1f);
	TestTrue(TEXT("off: the worker steps on its own"), LiveSimTime(Engine) > OffAt);

	Engine->bShouldStopTask = true;
	if (Engine->StepRequestEvent)
		Engine->StepRequestEvent->Trigger();
	if (Engine->AsyncPhysicsFuture.IsValid())
		Engine->AsyncPhysicsFuture.Wait();
	Var->Set(SavedSeconds, ECVF_SetByConsole);
	S.Cleanup();
	return true;
}
