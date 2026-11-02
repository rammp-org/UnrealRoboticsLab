// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Elements/MjBody.h"

#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "PhysicsEngine/BodySetup.h"

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Core/MjRenderSnapshot.h"
#include "MuJoCo/Utils/URLabAxisConv.h"
#include "State/MjCanonicalName.h"
#include "State/MjStateTypes.h"
#include "Utils/URLabLogging.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace
{

/**
 * The engine's model and data, plus the body id this element bound to.
 *
 * Same shape as the joint library's resolver, with the range check the caller
 * there has to do folded in: there is only one count a body can be checked
 * against, so there is no reason to make every accessor repeat it.
 */
bool ResolveBoundBody(const UMjBody* Body, const UMjPhysicsEngine*& OutEngine,
	const mjModel*& OutModel, int32& OutId)
{
	if (Body == nullptr)
	{
		return false;
	}
	const TOptional<int32>& Id = Body->GetBoundId();
	if (!Id.IsSet() || Id.GetValue() < 0)
	{
		return false;
	}
	const UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(Body);
	if (Engine == nullptr)
	{
		return false;
	}
	const mjModel* Model = Engine->GetModel();
	if (Model == nullptr || Id.GetValue() >= static_cast<int32>(Model->nbody))
	{
		return false;
	}
	OutEngine = Engine;
	OutModel = Model;
	OutId = Id.GetValue();
	return true;
}

/** The bound id alone, for the submissions the engine range-checks itself. */
bool ResolveBodyId(const UMjBody* Body, int32& OutId)
{
	if (Body == nullptr)
	{
		return false;
	}
	const TOptional<int32>& Id = Body->GetBoundId();
	if (!Id.IsSet() || Id.GetValue() < 0)
	{
		return false;
	}
	OutId = Id.GetValue();
	return true;
}

} // namespace

UMjBody::UMjBody()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
}

void UMjBody::BeginPlay()
{
	Super::BeginPlay();

	RefreshMeshPivotOffset();
}

void UMjBody::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// Only a mocap body has anything to push. The flag is an authored attribute
	// the generated setter can change at any point, so the tick asks each frame
	// rather than arming itself once at BeginPlay.
	if (!GetMocap())
	{
		return;
	}

	int32 Id = 0;
	if (!ResolveBodyId(this, Id))
	{
		return;
	}
	UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(this);
	if (Engine == nullptr)
	{
		return;
	}

	double MjPos[3];
	double MjQuat[4];
	URLabAxisConv::UePositionToMj(GetComponentLocation(), MjPos);
	URLabAxisConv::UeQuatToMj(GetComponentQuat(), MjQuat);
	Engine->SubmitMocapPose(Id, MjPos, MjQuat);
}

void UMjBody::ApplyRenderState(const FMjRenderSnapshot& Snap)
{
	// A mocap body is written by Unreal and read by MuJoCo; driving it from the
	// snapshot would fight the pose the tick just submitted.
	if (GetMocap())
	{
		return;
	}

	int32 Id = 0;
	if (!ResolveBodyId(this, Id))
	{
		return;
	}

	// The world body is not a pose to copy: an articulation's worldbody
	// component is the actor-relative container of authored content (mocap
	// targets, static geoms), and body id 0's snapshot pose is always the
	// MuJoCo origin. Driving it pins that whole subtree to the world origin
	// the moment the actor sits anywhere else -- a mocap child then pushes
	// the origin back into the model and the scene welds itself there.
	// Every bound child body sets its own world transform from the snapshot
	// regardless, so skipping the world body loses nothing.
	if (Id == 0)
	{
		return;
	}

	const int32 PosIdx = Id * 3;
	const int32 QuatIdx = Id * 4;
	if (Snap.XPos.Num() <= PosIdx + 2 || Snap.XQuat.Num() <= QuatIdx + 3)
	{
		// An empty snapshot means the physics worker has not published one yet
		// (the first ticks after BeginPlay, before the consumer-rate publish
		// fills it). Skip this frame and retry next tick -- do NOT disable, or
		// the body freezes permanently once the snapshot does fill. Warn only
		// when the snapshot is populated yet still too small for this id, which
		// is a real model/index mismatch.
		if (Snap.XPos.Num() > 0 && !m_bWarnedSnapshotRange)
		{
			UE_LOG(LogURLabBind, Warning,
				TEXT("UMjBody::ApplyRenderState - Body '%s' (id=%d) out of range "
					 "of a populated snapshot (XPos=%d, XQuat=%d)."),
				*GetName(), Id, Snap.XPos.Num(), Snap.XQuat.Num());
			m_bWarnedSnapshotRange = true;
		}
		return;
	}

	const FVector MuJoCoWorldPos = URLabAxisConv::MjPositionToUe(&Snap.XPos[PosIdx]);
	const FQuat MuJoCoWorldQuat = URLabAxisConv::MjQuatToUe(&Snap.XQuat[QuatIdx]);

	// A zero or non-finite snapshot row would write a degenerate transform that
	// NaN-floods the renderer (NIL LocalToWorld in the distance-field pass) for
	// every mesh under this body. Skip the frame and name the offender once.
	if (MuJoCoWorldPos.ContainsNaN() || MuJoCoWorldQuat.ContainsNaN()
		|| MuJoCoWorldQuat.SizeSquared() < KINDA_SMALL_NUMBER)
	{
		if (!m_bWarnedDegenerateXform)
		{
			UE_LOG(LogURLabBind, Warning,
				TEXT("UMjBody::ApplyRenderState - Body '%s' (id=%d) got a "
					 "degenerate snapshot transform (pos=%s quat=[%f %f %f %f]); "
					 "skipping."),
				*GetName(), Id, *MuJoCoWorldPos.ToString(),
				Snap.XQuat[QuatIdx], Snap.XQuat[QuatIdx + 1],
				Snap.XQuat[QuatIdx + 2], Snap.XQuat[QuatIdx + 3]);
			m_bWarnedDegenerateXform = true;
		}
		return;
	}

	FVector CorrectedPos = MuJoCoWorldPos;

	// MuJoCo reports the pose of the frame the converted collision shape was
	// built around, which is the mesh bounds centre, not the pivot the artist
	// placed. Rotate the offset into world and back it out so the component
	// lands where the mesh expects it.
	if (bIsQuickConverted)
	{
		CorrectedPos = MuJoCoWorldPos - MuJoCoWorldQuat.RotateVector(m_MeshPivotOffset);
	}

	SetWorldLocationAndRotation(CorrectedPos, MuJoCoWorldQuat);
}

void UMjBody::RefreshMeshPivotOffset()
{
	m_MeshPivotOffset = FVector::ZeroVector;

	TArray<USceneComponent*> AllChildren;
	GetChildrenComponents(true, AllChildren);
	for (USceneComponent* Child : AllChildren)
	{
		const UStaticMeshComponent* Mesh = Cast<UStaticMeshComponent>(Child);
		if (Mesh == nullptr || Mesh->GetStaticMesh() == nullptr)
		{
			continue;
		}
		const UBodySetup* Setup = Mesh->GetStaticMesh()->GetBodySetup();
		if (Setup == nullptr)
		{
			continue;
		}
		// The first mesh wins. A converted body is one mesh plus its collision;
		// where there are several, none of them is more the pivot than another.
		m_MeshPivotOffset = Setup->AggGeom.CalcAABB(FTransform::Identity).GetCenter();
		return;
	}
}

FVector UMjBody::GetWorldPosition() const
{
	const UMjPhysicsEngine* Engine = nullptr;
	const mjModel* Model = nullptr;
	int32 Id = 0;
	if (!ResolveBoundBody(this, Engine, Model, Id))
	{
		return FVector::ZeroVector;
	}
	double WorldPos[3] = {0.0, 0.0, 0.0};
	if (!MjSnapshotRange(*Engine, Id * 3, 3, WorldPos,
			[](const FMjRenderSnapshot& S) -> const TArray<mjtNum>& { return S.XPos; }))
	{
		return FVector::ZeroVector;
	}
	return URLabAxisConv::MjPositionToUe(WorldPos);
}

FQuat UMjBody::GetWorldRotation() const
{
	const UMjPhysicsEngine* Engine = nullptr;
	const mjModel* Model = nullptr;
	int32 Id = 0;
	if (!ResolveBoundBody(this, Engine, Model, Id))
	{
		return FQuat::Identity;
	}
	double WorldQuat[4] = {1.0, 0.0, 0.0, 0.0};
	if (!MjSnapshotRange(*Engine, Id * 4, 4, WorldQuat,
			[](const FMjRenderSnapshot& S) -> const TArray<mjtNum>& { return S.XQuat; }))
	{
		return FQuat::Identity;
	}
	return URLabAxisConv::MjQuatToUe(WorldQuat);
}

FMuJoCoSpatialVelocity UMjBody::GetSpatialVelocity() const
{
	FMuJoCoSpatialVelocity Result;

	int32 Id = 0;
	if (!ResolveBodyId(this, Id))
	{
		return Result;
	}
	UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(this);
	if (Engine == nullptr)
	{
		return Result;
	}

	Engine->WithRenderState([Id, &Result](const FMjRenderSnapshot& Snap) {
		const int32 Idx = Id * 6;
		if (Snap.CVel.Num() <= Idx + 5)
		{
			return;
		}
		// cvel is [angular xyz, linear xyz] in MuJoCo axes, m/s and rad/s.
		// Unreal flips Y and measures in cm/s and deg/s.
		const mjtNum* CVel = &Snap.CVel[Idx];

		Result.Linear.X = static_cast<float>(CVel[3]) * 100.0f;
		Result.Linear.Y = -static_cast<float>(CVel[4]) * 100.0f;
		Result.Linear.Z = static_cast<float>(CVel[5]) * 100.0f;

		Result.Angular.X = FMath::RadiansToDegrees(static_cast<float>(CVel[0]));
		Result.Angular.Y = -FMath::RadiansToDegrees(static_cast<float>(CVel[1]));
		Result.Angular.Z = FMath::RadiansToDegrees(static_cast<float>(CVel[2]));
	});

	return Result;
}

void UMjBody::ApplyForce(FVector Force, FVector Torque)
{
	int32 Id = 0;
	if (!ResolveBodyId(this, Id))
	{
		return;
	}
	UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(this);
	if (Engine == nullptr)
	{
		return;
	}

	// xfrc_applied is [force xyz, torque xyz] in MuJoCo's world axes: the
	// engine reads +0 as the force and +3 as the torque when it accumulates the
	// wrench into cfrc (engine_core_smooth.c). Newtons and newton-metres are SI
	// on both sides, so no scale applies, only the axis mirror. Under the Y
	// mirror a force flips Y; a torque is a pseudovector and flips X and Z
	// instead, the rule MjQuatToUe applies to a rotation.
	double Xfrc[6];
	Xfrc[0] = static_cast<double>(Force.X);
	Xfrc[1] = -static_cast<double>(Force.Y);
	Xfrc[2] = static_cast<double>(Force.Z);
	Xfrc[3] = -static_cast<double>(Torque.X);
	Xfrc[4] = static_cast<double>(Torque.Y);
	Xfrc[5] = -static_cast<double>(Torque.Z);
	Engine->SubmitWrench(Id, Xfrc);
}

FVector UMjBody::GetAppliedForce() const
{
	int32 Id = 0;
	if (!ResolveBodyId(this, Id))
	{
		return FVector::ZeroVector;
	}
	UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(this);
	if (Engine == nullptr)
	{
		return FVector::ZeroVector;
	}

	FVector Result = FVector::ZeroVector;
	Engine->WithRenderState([Id, &Result](const FMjRenderSnapshot& Snap) {
		const int32 Idx = Id * 6;
		if (Snap.XfrcApplied.Num() <= Idx + 5)
		{
			return;
		}
		// The force half of the wrench, back through the Y mirror.
		Result.X = static_cast<float>(Snap.XfrcApplied[Idx]);
		Result.Y = -static_cast<float>(Snap.XfrcApplied[Idx + 1]);
		Result.Z = static_cast<float>(Snap.XfrcApplied[Idx + 2]);
	});
	return Result;
}

FVector UMjBody::GetAppliedTorque() const
{
	int32 Id = 0;
	if (!ResolveBodyId(this, Id))
	{
		return FVector::ZeroVector;
	}
	UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(this);
	if (Engine == nullptr)
	{
		return FVector::ZeroVector;
	}

	FVector Result = FVector::ZeroVector;
	Engine->WithRenderState([Id, &Result](const FMjRenderSnapshot& Snap) {
		const int32 Idx = Id * 6;
		if (Snap.XfrcApplied.Num() <= Idx + 5)
		{
			return;
		}
		// The torque half, a pseudovector: X and Z flip back under the mirror.
		Result.X = -static_cast<float>(Snap.XfrcApplied[Idx + 3]);
		Result.Y = static_cast<float>(Snap.XfrcApplied[Idx + 4]);
		Result.Z = -static_cast<float>(Snap.XfrcApplied[Idx + 5]);
	});
	return Result;
}

void UMjBody::ClearForce()
{
	int32 Id = 0;
	if (!ResolveBodyId(this, Id))
	{
		return;
	}
	if (UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(this))
	{
		Engine->SubmitClearForce(Id);
	}
}

bool UMjBody::IsAwake() const
{
	int32 Id = 0;
	if (!ResolveBodyId(this, Id))
	{
		return true;
	}
	const UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(this);
	return Engine != nullptr ? Engine->IsBodyAwake(Id) : true;
}

void UMjBody::Wake()
{
	int32 Id = 0;
	if (!ResolveBodyId(this, Id))
	{
		return;
	}
	if (UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(this))
	{
		Engine->ApplyWakeBody(Id);
	}
}

void UMjBody::PutToSleep()
{
	int32 Id = 0;
	if (!ResolveBodyId(this, Id))
	{
		return;
	}
	if (UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(this))
	{
		Engine->ApplySleepBody(Id);
	}
}

void UMjBody::DescribeState(const mjModel* m, mjData* d, FMjArticulationState& Out) const
{
	// Runs on the physics worker with the model and data handed in, so this one
	// reads its arguments instead of resolving the engine like the accessors do.
	const TOptional<int32>& Id = GetBoundId();
	if (m == nullptr || d == nullptr || !Id.IsSet() || Id.GetValue() < 0 || Id.GetValue() >= m->nbody)
	{
		return;
	}

	FMjBodyState& B = Out.Bodies.AddDefaulted_GetRef();
	B.Name = FMjCanonicalName::PartSegment(Cast<AMjArticulation>(GetOwner()), MjName.Get(FString()));
	for (int32 i = 0; i < 3; ++i)
	{
		B.Xpos[i] = d->xpos[Id.GetValue() * 3 + i];
	}
	for (int32 i = 0; i < 4; ++i)
	{
		B.Xquat[i] = d->xquat[Id.GetValue() * 4 + i];
	}
}
