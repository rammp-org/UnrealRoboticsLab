// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Core/MjRenderSnapshot.h"
#include "MuJoCo/Core/MjDebugVisualizer.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Gen/Elements/Options/MjCompiler.gen.h"
#include "MuJoCo/Gen/Elements/Options/MjFlag.gen.h"
#include "MuJoCo/Gen/Elements/MjModel.gen.h"
#include "MuJoCo/Gen/Elements/Options/MjOption.gen.h"
#include "EngineUtils.h"
#include "Transport/NetworkManager.h"
#include "MuJoCo/Input/MjInputHandler.h"
#include "MuJoCo/Input/MjPerturbation.h"
#include "Replay/MjReplayManager.h"
#include "mujoco/mujoco.h"

#include "Kismet/GameplayStatics.h"
#include "Engine/World.h"
#include "HAL/IConsoleManager.h"
#include "Blueprint/UserWidget.h"
#include "Transport/ZmqPublishTransport.h"
#include "Transport/ZmqSubscribeTransport.h"
#include "Bridge/RpcDispatcher.h"
#include "Bridge/BridgeServerConfig.h"
#include "Bridge/BridgeServerConfigUtils.h"
#include "State/MjMsgpackEncoder.h"
#include "State/MjStateTypes.h"
#include "State/MjCanonicalName.h"
#include "State/MjObservationLevel.h"
#include "UserChannels/MjUserChannelComponent.h"
#include "Urdf/UrdfExporter.h"
#include "Transport/ShmPublishTransport.h"
#include "Transport/ShmRpcTransport.h"
#include "MuJoCo/Core/MjSimulationState.h"
#include "Utils/URLabLogging.h"
#include "Bridge/BridgeServerProvider.h"
#if WITH_EDITOR
#include "Misc/MessageDialog.h"
#endif

// Frame-locked stepping: see UMjPhysicsEngine::bLockedStepping and AAMjManager::OnWorldPostActorTick.
static TAutoConsoleVariable<float> CVarUrlabLockedFrameSeconds(
	TEXT("urlab.LockedFrameSeconds"), 0.0f,
	TEXT("Frame-locked stepping (live mode). Above 0: MuJoCo steps exactly round(this / timestep) steps per game frame, ")
		TEXT("between the frame's actor ticks, so every actor tick reads the state at the end of one whole batch and every ")
			TEXT("command lands on the next batch's first step, whatever the frame rate; the simulation then runs at the ")
				TEXT("batch length times the frame rate (slow motion on slow frames). 0, the default: MuJoCo runs on its own, ")
					TEXT("paced to the wall clock. Set back to 0 by the manager if a granted batch stops making progress."),
	ECVF_Default);

// How long the game thread waits on a granted batch that takes no step before it turns locked stepping off.
static constexpr double kLockedStallTimeoutSec = 2.0;
// The most one poll of that wait counts towards it (the polls are 1 ms apart; a longer gap is the process suspended).
static constexpr double kLockedPollShareMaxSec = 0.01;

AAMjManager* AAMjManager::Instance = nullptr;

AAMjManager::AAMjManager()
{
	PrimaryActorTick.bCanEverTick = true;

	PhysicsEngine = CreateDefaultSubobject<UMjPhysicsEngine>(TEXT("PhysicsEngine"));
	DebugVisualizer = CreateDefaultSubobject<UMjDebugVisualizer>(TEXT("DebugVisualizer"));
	NetworkManager = CreateDefaultSubobject<UMjNetworkManager>(TEXT("NetworkManager"));
	InputHandler = CreateDefaultSubobject<UMjInputHandler>(TEXT("InputHandler"));
	Perturbation = CreateDefaultSubobject<UMjPerturbation>(TEXT("Perturbation"));

	// The scene is one MuJoCo spec and this is its root. Sections hang off
	// it rather than off the actor directly, because a spec with two roots
	// is not a spec -- and the writer walks exactly this tree.
	SceneSpec = CreateDefaultSubobject<UMjModel>(TEXT("SceneSpec"));
	SceneOption = CreateDefaultSubobject<UMjOption>(TEXT("SceneOption"));
	SceneFlags = CreateDefaultSubobject<UMjFlag>(TEXT("SceneFlags"));
	SceneCompiler = CreateDefaultSubobject<UMjCompiler>(TEXT("SceneCompiler"));
	SceneOption->SetupAttachment(SceneSpec);
	SceneFlags->SetupAttachment(SceneOption);
	SceneCompiler->SetupAttachment(SceneSpec);
	if (RootComponent == nullptr)
	{
		RootComponent = SceneSpec;
	}

	// URLab's two departures from MuJoCo's own defaults, authored rather than
	// hard-coded so they show up in the details panel and in written MJCF.
	// Everything else stays unset, which is what leaves MuJoCo's default in
	// place: a value written here is written into every scene, and MuJoCo is
	// free to change its own mind between versions.
	SceneOption->Integrator = EMjIntegrator::implicitfast;
	SceneCompiler->Conflict = EMjConflict::merge;
}

FSpecRef AAMjManager::GetSceneSpec() const
{
	return FSpecRef::OverActor(const_cast<AAMjManager&>(*this));
}

void AAMjManager::RefreshStateCaches()
{
	BuildEntityCache();
	// Rebuild the state-IR producer cache off the same trigger as the entity
	// cache (initial compile + every recompile). Both run on the game thread.
	StateCollector.Init(this);
	StateCollector.RebuildProducerCacheGameThread();
	// Re-export the URDF(s) so the robot_description matches the fresh model.
	ExportRobotDescriptions();
}

void AAMjManager::ExportRobotDescriptions()
{
	RobotDescriptions.Reset();
	if (!PhysicsEngine || !PhysicsEngine->m_model)
		return;

	const mjModel* Model = PhysicsEngine->m_model;
	const FString RootDir = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("URLab"), TEXT("UrdfExport"));
	const FUrdfExportConfig Cfg;

	for (AMjArticulation* Art : GetAllArticulations())
	{
		if (!Art)
			continue;
		const FString RawName = Art->GetName();
		const FName Segment = FMjCanonicalName::ArtSegment(Art);
		const FString SegmentStr = Segment.ToString();
		const FString OutDir = FPaths::Combine(RootDir, SegmentStr);

		const FUrdfModel Urdf = FUrdfExporter::ExportToDir(
			Model, SegmentStr, RawName, OutDir, Cfg);
		if (Urdf.Links.Num() == 0)
			continue;

		RobotDescriptions.Add(Segment, Urdf.Xml);
		for (const FString& W : Urdf.Warnings)
			UE_LOG(LogURLab, Warning, TEXT("[URDF] %s: %s"), *SegmentStr, *W);
		UE_LOG(LogURLab, Log,
			TEXT("[URDF] %s: %d links, %d joints, %d meshes -> %s"),
			*SegmentStr, Urdf.Links.Num(), Urdf.Joints.Num(), Urdf.MeshIds.Num(),
			*FPaths::Combine(OutDir, TEXT("model.urdf")));
	}
}

void AAMjManager::BuildEntityCache()
{
	EntityCache.Reset();
	if (!PhysicsEngine || !PhysicsEngine->m_model)
		return;
	UWorld* World = GetWorld();
	if (!World)
		return;

	mjModel* m = PhysicsEngine->m_model;

	TSet<AMjArticulation*> ArticSet;
	for (AMjArticulation* A : PhysicsEngine->m_articulations)
		if (A)
			ArticSet.Add(A);

	for (TActorIterator<AActor> It(World); It; ++It)
	{
		AActor* Actor = *It;
		if (!Actor)
			continue;
		if (AMjArticulation* AsArt = Cast<AMjArticulation>(Actor))
		{
			if (ArticSet.Contains(AsArt))
				continue;
		}
		TArray<UMjBody*> Bodies;
		Actor->GetComponents<UMjBody>(Bodies);
		for (UMjBody* B : Bodies)
		{
			if (B == nullptr)
				continue;
			const int32 Id = B->GetBoundId().Get(-1);
			if (Id < 0 || Id >= m->nbody)
				continue;

			FMjEntityRecord Rec;
			Rec.MjId = Id;
			Rec.Name = B->MjName.Get(B->GetName());
			Rec.BodyComp = B;
			if (m->body_jntnum && m->body_jntadr)
			{
				int FirstJnt = m->body_jntadr[Id];
				int NumJnt = m->body_jntnum[Id];
				Rec.bHasFreeBase = (FirstJnt >= 0 && NumJnt > 0 && FirstJnt < m->njnt && m->jnt_type[FirstJnt] == mjJNT_FREE);
			}
			EntityCache.Add(Rec);
		}
	}
}

void AAMjManager::Compile()
{
	if (!PhysicsEngine)
		return;

	PhysicsEngine->Compile();

	// Sync discovery lists from PhysicsEngine
	m_MujocoComponents = PhysicsEngine->m_MujocoComponents;
	m_articulations = PhysicsEngine->m_articulations;
	m_heightfieldActors = PhysicsEngine->m_heightfieldActors;
	m_ArticulationMap = PhysicsEngine->m_ArticulationMap;

	RefreshStateCaches();
}

void AAMjManager::BeginPlay()
{
	Super::BeginPlay();

	if (Instance != nullptr && Instance != this)
	{
		UE_LOG(LogURLab, Error, TEXT("[AAMjManager] Multiple AAMjManager actors detected in level. Only one is supported — this instance (%s) will be ignored."), *GetName());
		return;
	}
	Instance = this;

	// Frame-locked stepping (urlab.LockedFrameSeconds): off unless the console variable is set; the handlers decide
	// each frame. Armed now if it is set, before the worker starts below, so the worker never takes an ungranted step
	// (whatever unpauses it, and whenever).
	PreActorTickHandle = FWorldDelegates::OnWorldPreActorTick.AddUObject(this, &AAMjManager::OnWorldPreActorTick);
	PostActorTickHandle = FWorldDelegates::OnWorldPostActorTick.AddUObject(this, &AAMjManager::OnWorldPostActorTick);
	if (PhysicsEngine && CVarUrlabLockedFrameSeconds.GetValueOnGameThread() > 0.0f)
	{
		ArmLockedStepping();
	}

	// Auto-create ReplayManager BEFORE the dispatcher / ZMQ components so the
	// dispatcher's Init can cache its pointer. (Transport worker threads later
	// read this cache to avoid TActorIterator, which asserts IsInGameThread.)
	{
		AMjReplayManager* ExistingReplay = Cast<AMjReplayManager>(
			UGameplayStatics::GetActorOfClass(GetWorld(), AMjReplayManager::StaticClass()));
		if (!ExistingReplay)
		{
			FActorSpawnParameters SpawnParams;
			AMjReplayManager* ReplayMgr = GetWorld()->SpawnActor<AMjReplayManager>(SpawnParams);
			if (ReplayMgr)
			{
				UE_LOG(LogURLab, Log, TEXT("[AAMjManager] Auto-created AMjReplayManager"));
			}
		}
	}

	// Resolve a bridge server. In editor builds the URLabEditor module
	// installs a resolver via URLabBridgeProvider that hands back the
	// subsystem's server (lifetime spans PIE sessions). Cooked builds
	// have no resolver, fall through to creating our own and marking it
	// as owned-by-manager so EndPlay tears it down.
	UURLabBridgeServer* ResolvedServer = URLabBridgeProvider::ResolveEditorServer();
	if (ResolvedServer)
	{
		BridgeServer = ResolvedServer;
		// Don't claim ownership — subsystem controls lifetime.
	}
	else
	{
		// Cooked path: no editor subsystem. Manager owns its bridge and
		// brings up both RPC transports inline. Bridge owns all RPC
		// transports; manager only owns publish/subscribe streams.
		FURLabBridgeServerConfig CookedConfig;
		URLabBridgeServerConfigUtils::LoadFromIni(CookedConfig);
		URLabBridgeServerConfigUtils::ApplyEnvAndCommandLineOverrides(CookedConfig);

		BridgeServer = NewObject<UURLabBridgeServer>(this, TEXT("BridgeServer"));
		BridgeServer->SetOwnedByManager(true);
		BridgeServer->SetInstanceConfig(CookedConfig);
		const FString StepEndpoint = FString::Printf(TEXT("tcp://%s:%d"),
			*CookedConfig.BindAddress, CookedConfig.StepPort);
		BridgeServer->Start(StepEndpoint);
		BridgeServer->EnsureShmBound(CookedConfig.InstanceId);
	}
	BridgeServer->RegisterManager(this);

	// Bind external (ROS) transports if their module is present. No-op when
	// URLabRos is not loaded (its factory hooks are unbound), so non-ROS builds
	// and non-ROS users are unaffected. This is what starts in-process ROS
	// publishing/services for a live session.
	BridgeServer->EnsureExternalTransportsBound();

	// State PUB binds the configured address + port so farm instances don't
	// collide; single-editor defaults reproduce tcp://0.0.0.0:5555.
	const FURLabBridgeServerConfig& NetConfig = BridgeServer->GetInstanceConfig();
	const FString StateEndpoint = FString::Printf(TEXT("tcp://%s:%d"),
		*NetConfig.BindAddress, NetConfig.StatePort);

	// Auto-create the streaming transports (PIE-only producers — they
	// tap PhysicsEngine pre/post-step callbacks). Bridge-style UObject
	// lifecycle: NewObject + SetOwningManager + TransportInit. Manager
	// owns streaming; bridge owns RPC.
	{
		UE_LOG(LogURLab, Log, TEXT("[AAMjManager] Creating manager-owned streaming transports"));

		UURLabZmqPublishTransport* Broadcaster = NewObject<UURLabZmqPublishTransport>(
			this, TEXT("AutoZmqBroadcaster"));
		if (Broadcaster)
		{
			Broadcaster->ZmqEndpoint = StateEndpoint;
			Broadcaster->SetOwningManager(this);
			if (Broadcaster->TransportInit())
			{
				ManagerOwnedPublishTransports.Add(Broadcaster);
				UE_LOG(LogURLab, Log,
					TEXT("[AAMjManager] Created UURLabZmqPublishTransport (%s)"), *StateEndpoint);
			}
		}

		UURLabShmPublishTransport* SmPub = NewObject<UURLabShmPublishTransport>(
			this, TEXT("AutoSmSnapshotPublisher"));
		if (SmPub)
		{
			SmPub->SetOwningManager(this);
			if (SmPub->TransportInit())
			{
				ManagerOwnedPublishTransports.Add(SmPub);
				UE_LOG(LogURLab, Log,
					TEXT("[AAMjManager] Created UURLabShmPublishTransport (state.shm)"));
			}
		}

		UURLabZmqSubscribeTransport* Subscriber = NewObject<UURLabZmqSubscribeTransport>(
			this, TEXT("AutoZmqSubscriber"));
		if (Subscriber)
		{
			Subscriber->SetOwningManager(this);
			if (Subscriber->TransportInit())
			{
				ManagerOwnedSubscribeTransports.Add(Subscriber);
				UE_LOG(LogURLab, Log,
					TEXT("[AAMjManager] Created UURLabZmqSubscribeTransport (tcp://127.0.0.1:5556)"));
			}
		}
	}

	Compile();
	if (NetworkManager)
		NetworkManager->UpdateCameraStreamingState();

	// Register ONE PreStep + ONE PostStep callback that walks both
	// manager-owned transport arrays.
	if (PhysicsEngine)
	{
		TWeakObjectPtr<AAMjManager> WeakSelf(this);
		PhysicsEngine->RegisterPreStepCallback(
			[WeakSelf](mjModel* m, mjData* d) {
				AAMjManager* Self = WeakSelf.Get();
				if (!Self)
					return;
				for (const TObjectPtr<UURLabSubscribeTransport>& T : Self->ManagerOwnedSubscribeTransports)
				{
					if (T)
						T->PreStep(m, d);
				}
				for (const TObjectPtr<UURLabPublishTransport>& T : Self->ManagerOwnedPublishTransports)
				{
					if (T)
						T->PreStep(m, d);
				}
			});
		PhysicsEngine->RegisterPostStepCallback(
			[WeakSelf](mjModel* m, mjData* d) {
				AAMjManager* Self = WeakSelf.Get();
				if (!Self)
					return;
				for (const TObjectPtr<UURLabPublishTransport>& T : Self->ManagerOwnedPublishTransports)
				{
					if (T)
						T->PostStep(m, d);
				}
				for (const TObjectPtr<UURLabSubscribeTransport>& T : Self->ManagerOwnedSubscribeTransports)
				{
					if (T)
						T->PostStep(m, d);
				}
			});
	}

	if (PhysicsEngine)
	{
		// Register debug data capture as a post-step callback. Fires whenever any
		// debug overlay needs fresh mjData — contact forces (key 1), body shader
		// overlays (Island / Segmentation modes), or tendon/muscle rendering.
		PhysicsEngine->RegisterPostStepCallback([this](mjModel* m, mjData* d) {
			if (!DebugVisualizer)
				return;
			const bool bNeedsCapture =
				DebugVisualizer->bShowDebug || DebugVisualizer->DebugShaderMode != EMjDebugShaderMode::Off || DebugVisualizer->bGlobalDrawTendons;
			if (bNeedsCapture)
			{
				DebugVisualizer->CaptureDebugData();
			}
		});

		// Build the state IR once per physics step, encode it to the canonical
		// msgpack `state_full` snapshot, and fan the bytes out to every
		// IMjSnapshotPublisher (ZMQ PUB, SHM ring, ...). This runs inside the
		// engine's CallbackMutex, so the collector's persistent snapshot buffer
		// never races an on-demand Collect from an RPC step reply.
		TWeakObjectPtr<AAMjManager> WeakSelf(this);
		PhysicsEngine->RegisterPostStepCallback(
			[WeakSelf](mjModel* m, mjData* d) {
				if (AAMjManager* Self = WeakSelf.Get())
					Self->FanOutStateSnapshot(m, d);
			});

		PhysicsEngine->RunMujocoAsync();
	}

	// Auto-create simulate widget AFTER Compile so articulations are registered
	if (bAutoCreateSimulateWidget && !SimulateWidget)
	{
		static const TCHAR* WidgetBPPath = TEXT("/UnrealRoboticsLab/UI/WBP_MjSimulate.WBP_MjSimulate_C");
		UClass* WidgetClass = LoadClass<UUserWidget>(nullptr, WidgetBPPath);
		if (WidgetClass)
		{
			APlayerController* PC = GetWorld()->GetFirstPlayerController();
			if (PC)
			{
				UUserWidget* Widget = CreateWidget<UUserWidget>(PC, WidgetClass);
				if (Widget)
				{
					Widget->AddToViewport(0);
					SimulateWidget = Widget;
					UE_LOG(LogURLab, Log, TEXT("[AAMjManager] Auto-created MjSimulate widget (press Tab to show)"));
				}
			}
		}
		else
		{
			UE_LOG(LogURLab, Warning, TEXT("[AAMjManager] Could not load WBP_MjSimulate Blueprint class. Widget not created."));
		}
	}
}

void AAMjManager::ToggleSimulateWidget()
{
	if (SimulateWidget)
	{
		bool bIsVisible = SimulateWidget->IsVisible();
		SimulateWidget->SetVisibility(bIsVisible ? ESlateVisibility::Collapsed : ESlateVisibility::Visible);
	}
}

void AAMjManager::RegisterSnapshotPublisher(IMjSnapshotPublisher* Publisher,
	UObject* OwnerObj)
{
	if (!Publisher || !OwnerObj)
		return;
	FScopeLock Lock(&SnapshotPublishersMutex);
	for (const FRegisteredSnapshotPublisher& R : SnapshotPublishers)
	{
		if (R.Publisher == Publisher)
			return; // already registered
	}
	SnapshotPublishers.Add({OwnerObj, Publisher});
}

void AAMjManager::UnregisterSnapshotPublisher(IMjSnapshotPublisher* Publisher)
{
	if (!Publisher)
		return;
	FScopeLock Lock(&SnapshotPublishersMutex);
	SnapshotPublishers.RemoveAll([Publisher](const FRegisteredSnapshotPublisher& R) {
		return R.Publisher == Publisher;
	});
}

void AAMjManager::RegisterStateProducer(TScriptInterface<IMjStateProducer> Producer)
{
	UObject* Obj = Producer.GetObject();
	if (!Obj)
		return;
	{
		FScopeLock Lock(&StateProducersMutex);
		for (const TWeakObjectPtr<UObject>& P : StateProducers)
		{
			if (P.Get() == Obj)
				return; // already registered
		}
		StateProducers.Add(Obj);
	}
	StateCollector.MarkProducerCacheDirty();
}

void AAMjManager::UnregisterStateProducer(TScriptInterface<IMjStateProducer> Producer)
{
	UObject* Obj = Producer.GetObject();
	if (!Obj)
		return;
	{
		FScopeLock Lock(&StateProducersMutex);
		StateProducers.RemoveAll([Obj](const TWeakObjectPtr<UObject>& P) {
			return P.Get() == Obj;
		});
	}
	StateCollector.MarkProducerCacheDirty();
}

void AAMjManager::GetStateProducers(TArray<TWeakObjectPtr<UObject>>& Out) const
{
	FScopeLock Lock(&StateProducersMutex);
	Out = StateProducers;
}

namespace
{
// The canonical art segment a user-channel component contributes under, or empty
// for scene scope. Mirrors the collector's producer scope resolution.
FString UserChannelScopeSegment(const UMjUserChannelComponent* Comp)
{
	if (!Comp)
		return FString();
	AActor* Owner = Comp->GetOwner();
	if (const AMjArticulation* Art = Cast<AMjArticulation>(Owner))
		return FMjCanonicalName::ArtSegment(Art).ToString();
	return FString();
}
} // namespace

bool AAMjManager::ApplyUserChannelInput(FName ArtOrNone, FName Channel,
	const FMjUserChannel& Value)
{
	const FString TargetScope = ArtOrNone.IsNone() ? FString() : ArtOrNone.ToString();

	TArray<TWeakObjectPtr<UObject>> Producers;
	GetStateProducers(Producers);

	bool bApplied = false;
	for (const TWeakObjectPtr<UObject>& Weak : Producers)
	{
		UMjUserChannelComponent* Comp = Cast<UMjUserChannelComponent>(Weak.Get());
		if (!Comp)
			continue;
		if (UserChannelScopeSegment(Comp) != TargetScope)
			continue;
		EMjUserChannelKind Declared;
		if (!Comp->GetDeclaredInputKind(Channel, Declared))
			continue;
		if (Comp->ApplyInput(Channel, Value))
			bApplied = true;
	}
	return bApplied;
}

void AAMjManager::GetUserInputChannels(TArray<FMjUserInputChannelInfo>& Out) const
{
	TArray<TWeakObjectPtr<UObject>> Producers;
	GetStateProducers(Producers);

	for (const TWeakObjectPtr<UObject>& Weak : Producers)
	{
		UMjUserChannelComponent* Comp = Cast<UMjUserChannelComponent>(Weak.Get());
		if (!Comp)
			continue;
		const FString Scope = UserChannelScopeSegment(Comp);
		TArray<TPair<FName, EMjUserChannelKind>> Declared;
		Comp->GetDeclaredInputChannels(Declared);
		for (const TPair<FName, EMjUserChannelKind>& Pair : Declared)
		{
			FMjUserInputChannelInfo Info;
			Info.ArtSegment = Scope;
			Info.Channel = Pair.Key;
			Info.Kind = Pair.Value;
			Out.Add(MoveTemp(Info));
		}
	}
}

void AAMjManager::RegisterStateConsumer(IMjStateConsumer* Consumer, UObject* OwnerObj)
{
	if (!Consumer || !OwnerObj)
		return;
	FScopeLock Lock(&StateConsumersMutex);
	for (const FRegisteredStateConsumer& R : StateConsumers)
	{
		if (R.Consumer == Consumer)
			return; // already registered
	}
	StateConsumers.Add({OwnerObj, Consumer});
}

void AAMjManager::UnregisterStateConsumer(IMjStateConsumer* Consumer)
{
	if (!Consumer)
		return;
	FScopeLock Lock(&StateConsumersMutex);
	StateConsumers.RemoveAll([Consumer](const FRegisteredStateConsumer& R) {
		return R.Consumer == Consumer;
	});
}

void AAMjManager::FanOutStateSnapshot(mjModel* m, mjData* d)
{
	// Build the state IR once per physics step, encode it to the canonical
	// msgpack `state_full` snapshot, and fan the bytes out to every
	// IMjSnapshotPublisher (ZMQ PUB, SHM ring, ...). Registered IMjStateConsumers
	// receive the same typed IR and run their own encoders. Runs inside the
	// engine's CallbackMutex, so the collector's persistent snapshot buffer never
	// races an on-demand Collect from an RPC step reply.
	TArray<IMjSnapshotPublisher*> Pubs;
	{
		FScopeLock Lock(&SnapshotPublishersMutex);
		Pubs.Reserve(SnapshotPublishers.Num());
		for (const FRegisteredSnapshotPublisher& R : SnapshotPublishers)
		{
			if (R.Publisher && R.Owner.IsValid())
				Pubs.Add(R.Publisher);
		}
	}

	TArray<IMjStateConsumer*> Consumers;
	{
		FScopeLock Lock(&StateConsumersMutex);
		Consumers.Reserve(StateConsumers.Num());
		for (const FRegisteredStateConsumer& R : StateConsumers)
		{
			if (R.Consumer && R.Owner.IsValid())
				Consumers.Add(R.Consumer);
		}
	}

	// bPublishersPaused gates the msgpack byte fan-out only: it is set on
	// Direct / Puppet mode entry so the step reply is the sole delivery to the
	// stepping client (no double-write). A typed consumer is a distinct sink, so
	// it receives the IR every step in all modes regardless of the pause.
	const bool bByteFanOut = Pubs.Num() > 0
						  && !bPublishersPaused.load(std::memory_order_acquire);

	if (!bByteFanOut && Consumers.Num() == 0)
		return;

	FURLabRpcDispatcher* Disp = GetStepDispatcher();
	const int64 StepIdx = Disp ? Disp->GetStepCounter() : 0;
	const FMjStateSnapshot& Snap = StateCollector.Collect(m, d, StepIdx);

	for (IMjStateConsumer* Consumer : Consumers)
		Consumer->ConsumeState(Snap);

	if (!bByteFanOut)
		return;

	TArray<uint8> Buf = FMjMsgpackEncoder::EncodeSnapshotBytes(
		Snap, EObservationLevel::Standard);
	if (Buf.Num() == 0)
		return;
	for (IMjSnapshotPublisher* Pub : Pubs)
		Pub->PublishSnapshot(Buf);
}

void AAMjManager::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	FWorldDelegates::OnWorldPreActorTick.Remove(PreActorTickHandle);
	FWorldDelegates::OnWorldPostActorTick.Remove(PostActorTickHandle);
	PreActorTickHandle.Reset();
	PostActorTickHandle.Reset();
	// The worker free again before it is told to stop (a locked worker idles on the step event, which wakes it).
	StopLockedStepping();

	// Stop the physics async thread BEFORE Super::EndPlay so PostStep
	// callbacks don't race into resources child components tear down.
	// Bounded wait: a pathological mj_step can take many seconds; an
	// unbounded Wait() would freeze PIE-stop. On timeout we detach and
	// leak m_model/m_data to avoid use-after-free in the still-running
	// step (one-time per session).
	bool bAsyncExited = true;
	if (PhysicsEngine)
	{
		PhysicsEngine->bShouldStopTask = true;
		// Wake the worker if parked on the step-request event so it
		// observes bShouldStopTask without burning the Wait timeout.
		if (PhysicsEngine->StepRequestEvent)
			PhysicsEngine->StepRequestEvent->Trigger();
		if (PhysicsEngine->AsyncPhysicsFuture.IsValid())
		{
			constexpr double kShutdownTimeoutSec = 3.0;
			bAsyncExited = PhysicsEngine->AsyncPhysicsFuture.WaitFor(
				FTimespan::FromSeconds(kShutdownTimeoutSec));
			if (!bAsyncExited)
			{
				UE_LOG(LogURLab, Warning,
					TEXT("Physics async thread did not exit within %.1fs — detaching. ")
						TEXT("mj_step is likely stuck; MuJoCo resources will leak for this session ")
							TEXT("to avoid a use-after-free in the still-running step."),
					kShutdownTimeoutSec);
			}
		}
		// Clearing callbacks takes CallbackMutex, which a wedged worker holds for
		// its whole iteration. Only safe once the worker has provably exited;
		// on the detach path the callbacks leak with the rest of the accepted
		// leak rather than deadlock PIE-stop.
		if (bAsyncExited)
			PhysicsEngine->ClearCallbacks();
	}

	// Manager-owned transports aren't UActorComponents, so EndPlay
	// doesn't propagate to them; explicit TransportShutdown required.
	for (TObjectPtr<UURLabSubscribeTransport>& T : ManagerOwnedSubscribeTransports)
	{
		if (T)
			T->TransportShutdown();
	}
	ManagerOwnedSubscribeTransports.Reset();
	for (TObjectPtr<UURLabPublishTransport>& T : ManagerOwnedPublishTransports)
	{
		if (T)
			T->TransportShutdown();
	}
	ManagerOwnedPublishTransports.Reset();

	Super::EndPlay(EndPlayReason);
	if (Instance == this)
		Instance = nullptr;

	// Manager-owned servers also tear down the dispatcher itself;
	// subsystem-owned servers stay alive across PIE cycles.
	if (BridgeServer)
	{
		BridgeServer->UnregisterManager(this);
		if (BridgeServer->IsOwnedByManager())
		{
			BridgeServer->Stop();
		}
		BridgeServer = nullptr;
	}

	// Clear tracked actors to prevent dangling pointers on level restart
	m_heightfieldActors.Empty();
	m_articulations.Empty();
	m_MujocoComponents.Empty();

	// Only touch MuJoCo resources if the async thread actually exited — a
	// detached thread may still be executing mj_step and reading these.
	if (PhysicsEngine && bAsyncExited)
	{
		// The model belongs to the compiled scene, which also owns the specs it
		// was compiled from and knows the order they have to go in, so the two
		// pointers cannot be freed on their own.
		PhysicsEngine->ReleaseCompiledScene();
		PhysicsEngine->m_heightfieldActors.Empty();
		PhysicsEngine->m_articulations.Empty();
		PhysicsEngine->m_MujocoComponents.Empty();
	}
}

void AAMjManager::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (!PhysicsEngine || !PhysicsEngine->IsInitialized())
	{
		return;
	}

	// Locked stepping applied this frame's batch before the actors ticked: the state is the same.
	if (bLockedStateApplied)
	{
		bLockedStateApplied = false;
		return;
	}
	ApplyLatestRenderState();
}

void AAMjManager::OnWorldPreActorTick(UWorld* World, ELevelTick TickType, float DeltaSeconds)
{
	bLockedStateApplied = false;
	if (World != GetWorld() || !PhysicsEngine || !PhysicsEngine->IsInitialized()
		|| !PhysicsEngine->bLockedStepping.load(std::memory_order_acquire))
	{
		return;
	}
	if (PhysicsEngine->GetStepMode() != EStepMode::Live)
	{
		// A client took over the stepping (direct / puppet) since the last grant: it publishes its own frames. Drop
		// the batch but stay armed, so a return to live waits for a grant rather than running on the wall clock.
		ArmLockedStepping();
		return;
	}
	// Wait for the batch granted after the last frame's actor ticks. Nothing finishes a batch while paused, out of
	// live mode (the worker steps only live batches), stopping or with the worker gone: this frame reads as unlocked
	// then (Tick applies whatever is published), and an unfinished batch finishes once the worker steps it again.
	// Stalled is the time spent waiting with no step taken, a poll's share capped, so a debugger break or a suspended
	// process (both threads frozen together) is not taken for a stuck worker.
	double Stalled = 0.0;
	double LastPoll = FPlatformTime::Seconds();
	int32 LastBudget = PhysicsEngine->LockedStepBudget.load(std::memory_order_acquire);
	while (PhysicsEngine->LockedBatchesDone.load(std::memory_order_acquire) < LockedBatchesGranted)
	{
		if (PhysicsEngine->bPausedAtomic.load(std::memory_order_acquire))
		{
			// A pause interrupted the batch: hold the rest until a frame boundary releases it (the next unpaused
			// pre-tick or post-tick), so an unpause during this frame's actor ticks does not run it under them.
			PhysicsEngine->bLockedBatchHeld.store(true, std::memory_order_release);
			return;
		}
		if (PhysicsEngine->GetStepMode() != EStepMode::Live)
		{
			ArmLockedStepping(); // a client took over mid-batch: drop the rest, as above
			return;
		}
		if (PhysicsEngine->bShouldStopTask || !PhysicsEngine->bWorkerRunning.load(std::memory_order_acquire))
		{
			return;
		}
		// Unpaused at a frame boundary: a batch a pause held runs on now, and this wait finishes it before any actor
		// ticks. (Released on the same read of the pause that let the wait go on, so an unpause racing this cannot
		// leave the wait on a held batch. An unpause during the actor ticks leaves it to the post-tick.)
		if (PhysicsEngine->bLockedBatchHeld.exchange(false, std::memory_order_acq_rel) && PhysicsEngine->StepRequestEvent)
		{
			PhysicsEngine->StepRequestEvent->Trigger();
		}
		// A long batch is fine while it is stepping; one that takes no step for the timeout is stuck.
		const double Now = FPlatformTime::Seconds();
		const int32 Budget = PhysicsEngine->LockedStepBudget.load(std::memory_order_acquire);
		Stalled += FMath::Min(Now - LastPoll, kLockedPollShareMaxSec);
		LastPoll = Now;
		if (Budget != LastBudget)
		{
			LastBudget = Budget;
			Stalled = 0.0;
		}
		else if (Stalled > kLockedStallTimeoutSec)
		{
			UE_LOG(LogURLab, Error,
				TEXT("[AAMjManager] urlab.LockedFrameSeconds: a granted batch took no step for %.1f s (%d of its steps ")
					TEXT("left); frame-locked stepping is off and the variable is set back to 0 (set it again to retry)."),
				kLockedStallTimeoutSec, Budget);
			StopLockedStepping();
			if (IConsoleVariable* Var = CVarUrlabLockedFrameSeconds.AsVariable())
			{
				Var->Set(0.0f, ECVF_SetByConsole);
			}
			return;
		}
		if (PhysicsEngine->LockedBatchEvent)
		{
			PhysicsEngine->LockedBatchEvent->Wait(1);
		}
	}
	// No batch is outstanding now: a hold set as a pause met the batch's last step in flight holds nothing.
	PhysicsEngine->bLockedBatchHeld.store(false, std::memory_order_release);
	// Publish the finished batch here, not from the worker at its last step: the snapshot then changes only at this
	// frame boundary, so every game-thread reader in the frame (actor ticks, render, Slate) sees the batch the actors
	// saw. The worker is idle outside CallbackMutex between batches, so the lock is uncontended.
	const uint64 Done = PhysicsEngine->LockedBatchesDone.load(std::memory_order_acquire);
	if (Done != LockedBatchesPublished)
	{
		FScopeLock Lock(&PhysicsEngine->CallbackMutex);
		PhysicsEngine->PushRenderState();
		LockedBatchesPublished = Done;
	}
	ApplyLatestRenderState();
	bLockedStateApplied = true;
}

void AAMjManager::ArmLockedStepping()
{
	// Count from the worker's own count, nothing granted yet; the worker idles until the first grant. (A locked
	// iteration still under way finishes uncounted: it spends no step from the zeroed budget. One that ended a batch
	// just before can count it after this read; OnWorldPostActorTick resyncs the grant count then.)
	PhysicsEngine->LockedStepBudget.store(0, std::memory_order_release);
	PhysicsEngine->bLockedBatchHeld.store(false, std::memory_order_release);
	PhysicsEngine->bLockedBatchFresh.store(false, std::memory_order_release);
	LockedBatchesGranted = PhysicsEngine->LockedBatchesDone.load(std::memory_order_acquire);
	LockedBatchesPublished = LockedBatchesGranted;
	PhysicsEngine->bLockedStepping.store(true, std::memory_order_release);
}

void AAMjManager::StopLockedStepping()
{
	if (PhysicsEngine && PhysicsEngine->bLockedStepping.exchange(false, std::memory_order_acq_rel))
	{
		PhysicsEngine->LockedStepBudget.store(0, std::memory_order_release);
		PhysicsEngine->bLockedBatchHeld.store(false, std::memory_order_release);
		PhysicsEngine->bLockedBatchFresh.store(false, std::memory_order_release);
	}
}

void AAMjManager::OnWorldPostActorTick(UWorld* World, ELevelTick TickType, float DeltaSeconds)
{
	if (World != GetWorld() || !PhysicsEngine)
	{
		return;
	}
	const float Seconds = CVarUrlabLockedFrameSeconds.GetValueOnGameThread();
	if (Seconds <= 0.0f)
	{
		StopLockedStepping();
		return;
	}
	// Armed whatever the worker is doing (paused, not started, between models, a client stepping it), so it never
	// steps on the wall clock: an unpause, a (re)start or a return to live waits for the next grant.
	if (PhysicsEngine->GetStepMode() != EStepMode::Live)
	{
		ArmLockedStepping(); // and nothing granted while a client steps it
		return;
	}
	if (!PhysicsEngine->bLockedStepping.load(std::memory_order_acquire))
	{
		ArmLockedStepping();
	}
	if (!PhysicsEngine->IsInitialized() || !PhysicsEngine->bWorkerRunning.load(std::memory_order_acquire)
		|| PhysicsEngine->bPausedAtomic.load(std::memory_order_acquire))
	{
		return;
	}
	// The rest of a batch a pause interrupted runs now, between frames, like any batch; no new grant over it. (A
	// hold over a batch that has since finished holds nothing: the grant goes ahead.)
	if (PhysicsEngine->bLockedBatchHeld.exchange(false, std::memory_order_acq_rel)
		&& PhysicsEngine->LockedBatchesDone.load(std::memory_order_acquire) < LockedBatchesGranted)
	{
		if (PhysicsEngine->StepRequestEvent)
		{
			PhysicsEngine->StepRequestEvent->Trigger();
		}
		return;
	}
	// The model's timestep now, not as the worker last saw it: a set_sim_options or a recompile since the last batch
	// sizes this one. The worker is between iterations while this holds CallbackMutex, so the batch count read after
	// it is exact (the worker counts a batch under the lock).
	double Dt = 0.0;
	{
		FScopeLock Lock(&PhysicsEngine->CallbackMutex);
		if (PhysicsEngine->m_model)
		{
			Dt = PhysicsEngine->m_model->opt.timestep;
		}
	}
	const uint64 Done = PhysicsEngine->LockedBatchesDone.load(std::memory_order_acquire);
	if (Done > LockedBatchesGranted)
	{
		// A batch the worker counted after the lock was (re)armed: count it as granted; the next pre-tick publishes it.
		UE_LOG(LogURLab, Log, TEXT("[AAMjManager] urlab.LockedFrameSeconds: %llu batch(es) finished after the lock was armed; resynced."),
			Done - LockedBatchesGranted);
		LockedBatchesGranted = Done;
	}
	// No grant over the last batch while it is unfinished or finished but not yet published (it finished after this
	// frame's pre-tick wait gave up): the next pre-tick publishes it, so a frame never advances by two batches.
	if (Done < LockedBatchesGranted || Done != LockedBatchesPublished || Dt <= 0.0)
	{
		return;
	}
	const int32 Steps = FMath::Max(1, FMath::RoundToInt(Seconds / Dt));
	++LockedBatchesGranted;
	PhysicsEngine->bLockedBatchFresh.store(true, std::memory_order_release); // before the budget the worker waits on
	PhysicsEngine->LockedStepBudget.store(Steps, std::memory_order_release);
	if (PhysicsEngine->StepRequestEvent)
	{
		PhysicsEngine->StepRequestEvent->Trigger();
	}
}

void AAMjManager::ApplyLatestRenderState()
{
	if (!PhysicsEngine || !PhysicsEngine->IsInitialized())
	{
		return;
	}

	// Ask the live-mode worker to publish a fresh snapshot; it copies the
	// full state only when a consumer (this tick) has requested one.
	PhysicsEngine->bSnapshotWanted.store(true, std::memory_order_release);

	const TArray<AMjArticulation*>& Arts = PhysicsEngine->GetAllArticulations();
	const TArray<UMjQuickConvertComponent*> Quicks = PhysicsEngine->GetAllQuickComponents();

	// Copy the frame out and apply it after releasing the snapshot lock. The
	// stepping thread takes that lock to publish, and applying a frame moves
	// every body and geom component in the scene -- milliseconds on a large
	// model, tens of them on a slow frame -- while copying it is microseconds.
	// Applying under the lock held the stepper behind the game thread for
	// most of every frame: a 140-body, 485-geom robot stepped at about a
	// quarter of real time with MuJoCo itself idle 95% of the time.
	//
	// Game thread only, and used start to finish within this call, so one
	// buffer serves every call and keeps its allocations between frames.
	static FMjRenderSnapshot Applied;
	PhysicsEngine->WithRenderState([](const FMjRenderSnapshot& Snap) { Applied = Snap; });

	for (AMjArticulation* Art : Arts)
	{
		if (Art)
		{
			Art->ApplyRenderState(Applied);
		}
	}
	for (UMjQuickConvertComponent* Quick : Quicks)
	{
		if (Quick)
		{
			Quick->ApplyRenderState(Applied);
		}
	}
	// Record which post-step state the actors now reflect so cameras can
	// tag their readbacks with it (frame_id association for the bridge).
	LastAppliedRenderFrameId.store(Applied.FrameId, std::memory_order_release);
	LastAppliedRenderSimTime.store(Applied.SimTime, std::memory_order_release);
}

AAMjManager* AAMjManager::GetManager()
{
	return Instance;
}

UMjPhysicsEngine* AAMjManager::ResolveEngine(const UObject* WorldCtx)
{
	if (AAMjManager* Manager = GetManager())
	{
		if (Manager->PhysicsEngine)
			return Manager->PhysicsEngine;
	}
	if (WorldCtx)
	{
		if (UWorld* World = WorldCtx->GetWorld())
		{
			for (TActorIterator<AAMjManager> It(World); It; ++It)
			{
				if (It->PhysicsEngine)
					return It->PhysicsEngine;
			}
		}
	}
	return nullptr;
}

void AAMjManager::SetPaused(bool bPaused)
{
	if (PhysicsEngine)
		PhysicsEngine->SetPaused(bPaused);
}

bool AAMjManager::IsRunning() const
{
	return PhysicsEngine ? PhysicsEngine->IsRunning() : false;
}

bool AAMjManager::IsInitialized() const
{
	return PhysicsEngine ? PhysicsEngine->IsInitialized() : false;
}

FString AAMjManager::GetLastCompileError() const
{
	return PhysicsEngine ? PhysicsEngine->GetLastCompileError() : FString();
}

void AAMjManager::StepSync(int32 NumSteps)
{
	if (PhysicsEngine)
		PhysicsEngine->StepSync(NumSteps);
}

bool AAMjManager::CompileModel()
{
	if (!PhysicsEngine)
		return false;

	bool Result = PhysicsEngine->CompileModel();

	// Re-sync discovery lists after recompile
	m_MujocoComponents = PhysicsEngine->m_MujocoComponents;
	m_articulations = PhysicsEngine->m_articulations;
	m_heightfieldActors = PhysicsEngine->m_heightfieldActors;
	m_ArticulationMap = PhysicsEngine->m_ArticulationMap;

	// Component ids/views were re-bound; rebuild the state-IR caches so the
	// collector's weak ptrs and entity table match the fresh model.
	RefreshStateCaches();

	return Result;
}

AMjArticulation* AAMjManager::GetArticulation(const FString& ActorName) const
{
	return PhysicsEngine ? PhysicsEngine->GetArticulation(ActorName) : nullptr;
}

const TArray<AMjArticulation*>& AAMjManager::GetAllArticulations() const
{
	return PhysicsEngine ? PhysicsEngine->GetAllArticulations() : m_articulations;
}

TArray<UMjQuickConvertComponent*> AAMjManager::GetAllQuickComponents() const
{
	return PhysicsEngine ? PhysicsEngine->GetAllQuickComponents() : m_MujocoComponents;
}

TArray<AMjHeightfieldActor*> AAMjManager::GetAllHeightfields() const
{
	return PhysicsEngine ? PhysicsEngine->GetAllHeightfields() : m_heightfieldActors;
}

float AAMjManager::GetSimTime() const
{
	return PhysicsEngine ? PhysicsEngine->GetSimTime() : 0.0f;
}

float AAMjManager::GetTimestep() const
{
	return PhysicsEngine ? PhysicsEngine->GetTimestep() : 0.002f;
}

// --- Recording / replay ---

void AAMjManager::StartRecording()
{
	AMjReplayManager* ReplayMgr = Cast<AMjReplayManager>(UGameplayStatics::GetActorOfClass(GetWorld(), AMjReplayManager::StaticClass()));
	if (ReplayMgr)
	{
		ReplayMgr->StartRecording();
		UE_LOG(LogURLab, Log, TEXT("StartRecording called on ReplayManager."));
	}
	else
	{
		UE_LOG(LogURLab, Warning, TEXT("ReplayManager not found in scene!"));
	}
}

void AAMjManager::StopRecording()
{
	AMjReplayManager* ReplayMgr = Cast<AMjReplayManager>(UGameplayStatics::GetActorOfClass(GetWorld(), AMjReplayManager::StaticClass()));
	if (ReplayMgr)
	{
		ReplayMgr->StopRecording();
		UE_LOG(LogURLab, Log, TEXT("StopRecording called on ReplayManager."));
	}
	// OnPostStep callback stays registered; the replay manager gates
	// recording with bIsRecording so it can be restarted without re-registering.
}

void AAMjManager::StartReplay()
{
	AMjReplayManager* ReplayMgr = Cast<AMjReplayManager>(UGameplayStatics::GetActorOfClass(GetWorld(), AMjReplayManager::StaticClass()));
	if (ReplayMgr)
	{
		ReplayMgr->StartReplay();
		UE_LOG(LogURLab, Log, TEXT("StartReplay called on ReplayManager."));
	}
}

void AAMjManager::StopReplay()
{
	AMjReplayManager* ReplayMgr = Cast<AMjReplayManager>(UGameplayStatics::GetActorOfClass(GetWorld(), AMjReplayManager::StaticClass()));
	if (ReplayMgr)
	{
		ReplayMgr->StopReplay();
		UE_LOG(LogURLab, Log, TEXT("StopReplay called on ReplayManager."));
	}
}

void AAMjManager::ResetSimulation()
{
	if (PhysicsEngine)
	{
		// Locked: the rest of a batch under way, or one a pause holds, belongs to the old episode; the next grant
		// starts the new one with a whole batch (the reset is applied on its first step). Game thread only (the
		// counters are the game thread's): a caller on another thread (an RPC replay_start) only gets here out of
		// live mode, where the handlers re-arm every frame anyway.
		if (IsInGameThread() && PhysicsEngine->bLockedStepping.load(std::memory_order_acquire))
		{
			ArmLockedStepping();
		}
		PhysicsEngine->ResetSimulation();
	}
	UE_LOG(LogURLab, Log, TEXT("MuJoCo Manager: Reset requested."));
}

UMjSimulationState* AAMjManager::CaptureSnapshot()
{
	return PhysicsEngine ? PhysicsEngine->CaptureSnapshot() : nullptr;
}

void AAMjManager::RestoreSnapshot(UMjSimulationState* Snapshot)
{
	if (PhysicsEngine && Snapshot)
	{
		// Locked: as ResetSimulation, the restored state starts a whole batch.
		if (IsInGameThread() && PhysicsEngine->bLockedStepping.load(std::memory_order_acquire))
		{
			ArmLockedStepping();
		}
		PhysicsEngine->RestoreSnapshot(Snapshot);
	}
}