// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Where an asset resolves once the project is not where it was imported.
//
// A spec records the absolute path it was read from, and every asset `file`
// resolves against that directory. That makes the resolution a fact about the
// importing machine: clone the same content somewhere else and each mesh
// resolves into a directory that does not exist. Nothing reaches the VFS, and
// the compile fails on the mount name it was left holding -- a mangled
// `<owner>_<file>` -- which names nothing anybody can go and look at.
//
// The content is portable, so the resolution has to be: what the recorded
// directory still says correctly is where under a project the spec sat, and
// that tail is tried here. These tests fabricate the recorded path so the
// "other machine" is the case under test rather than the one nobody can run.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"

#include "MuJoCo/Elements/MjMesh.h"
#include "MuJoCo/Spec/MjAssetSink.h"
#include "MuJoCo/Spec/MjSpecBuild.h"
#include "MuJoCo/Spec/MjSpecRef.h"

namespace MjAssetProvenanceTests
{

/** Every request, so a test can assert on what was and was not found. */
class FRecordingSink final : public IMjAssetSink
{
public:
	TArray<FMjAssetRequest> Found;
	TArray<FMjAssetRequest> Missing;
	TArray<int32> ByteCounts;

	void OnMesh(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override
	{
		Found.Add(Request);
		ByteCounts.Add(Bytes.Num());
	}
	void OnTexture(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override {}
	void OnHeightField(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override {}
	void OnMissing(const FMjAssetRequest& Request) override { Missing.Add(Request); }
};

/**
 * A spec parsed as though read from `SourcePath`, over a live actor.
 *
 * The path is never opened -- the XML comes from the string -- so it can name
 * a checkout that does not exist, which is exactly what this file needs.
 */
struct FScratchDoc
{
	UWorld* World = nullptr;
	AActor* Actor = nullptr;

	~FScratchDoc()
	{
		if (World != nullptr)
		{
			World->DestroyWorld(false);
		}
	}

	FSpecRef Ref() const { return FSpecRef::OverActor(*Actor); }
};

bool Parse(FAutomationTestBase& Test, FScratchDoc& Doc, const FString& Xml, const FString& SourcePath)
{
	const FString Stem = FString::Printf(TEXT("MjProv_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));
	Doc.World = UWorld::CreateWorld(EWorldType::Editor, /*bInformEngineOfWorld=*/false, FName(*Stem));
	if (Doc.World == nullptr)
	{
		Test.AddError(TEXT("could not create a scratch world"));
		return false;
	}
	Doc.Actor = Doc.World->SpawnActor<AActor>();
	if (Doc.Actor == nullptr)
	{
		Test.AddError(TEXT("could not spawn a scratch actor"));
		return false;
	}
	if (!MjParseIntoActor(*Doc.Actor, Xml, SourcePath).IsOk())
	{
		Test.AddError(TEXT("parse failed"));
		return false;
	}
	return true;
}

/** A model naming one mesh at `File`. */
FString MeshModel(const FString& File)
{
	return FString::Printf(
		TEXT("<mujoco model=\"prov\"><asset><mesh name=\"part\" file=\"%s\"/></asset>")
			TEXT("<worldbody><geom type=\"box\" size=\".1 .1 .1\"/></worldbody></mujoco>"),
		*File);
}

/** A real file on disk, with enough in it to tell an empty read from a full one. */
bool WriteMesh(const FString& Path)
{
	const FString Obj = TEXT("v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 2 3\nf 1 2 4\nf 1 3 4\nf 2 3 4\n");
	return FFileHelper::SaveStringToFile(Obj, *Path);
}

/** An absolute path under this project's Saved directory. */
FString ScratchPath(const FString& Relative)
{
	return FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / Relative);
}

/** A short unique name, so concurrent or repeated runs cannot collide. */
FString UniqueName()
{
	return FGuid::NewGuid().ToString(EGuidFormats::Digits);
}

/**
 * A scratch directory, removed when the test leaves.
 *
 * Takes the absolute path rather than composing one: these tests care exactly
 * where a tree sits, because where it sits under the project is what decides
 * what the re-root produces.
 */
struct FScratchTree
{
	FString Root;

	explicit FScratchTree(const FString& AbsoluteRoot)
		: Root(AbsoluteRoot)
	{
		IFileManager::Get().MakeDirectory(*Root, /*Tree=*/true);
	}

	~FScratchTree() { IFileManager::Get().DeleteDirectory(*Root, /*RequireExists=*/false, /*Tree=*/true); }
};

} // namespace MjAssetProvenanceTests

// ============================================================================
// URLab.Assets.AMovedProjectStillResolvesItsMeshes
//
// The shape a real import leaves behind: the spec was read from
// `<Project>/Saved/URLab/ImportPrep/<model>/`, and `file` climbs back out of
// there to reach content committed elsewhere in the project. Recorded on one
// machine, resolved on another, the recorded root is wrong and every `..` in
// the reference has already collapsed the project folder away -- so the repair
// has to happen on the directory, before the combine.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjMovedProjectResolvesTest, "URLab.Assets.AMovedProjectStillResolvesItsMeshes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjMovedProjectResolvesTest::RunTest(const FString& Parameters)
{
	using namespace MjAssetProvenanceTests;

	// The content, where THIS project has it.
	FScratchTree Tree(ScratchPath(FString(TEXT("URLab/TestProvenance/")) + UniqueName()));
	const FString MeshPath = Tree.Root / TEXT("part.obj");
	if (!TestTrue(TEXT("the scratch mesh was written"), WriteMesh(MeshPath)))
	{
		return false;
	}

	// The same relative walk the importer records, rooted at a checkout that is
	// not there: out of ImportPrep/<model>, back to the project, and down into
	// the folder the mesh really sits in.
	FString Tail = Tree.Root;
	FPaths::MakePathRelativeTo(Tail, *FPaths::ConvertRelativePathToFull(FPaths::ProjectDir()));
	const FString File = FString(TEXT("../../../../")) / Tail / TEXT("part.obj");

	const FString AbsentRoot = FString(TEXT("/MjNoSuchCheckout_")) + FGuid::NewGuid().ToString(EGuidFormats::Digits);
	const FString SourcePath = AbsentRoot / TEXT("Saved/URLab/ImportPrep/prov/prov_ue.xml");
	TestFalse(TEXT("the fabricated checkout really is absent"), FPaths::DirectoryExists(AbsentRoot));

	FScratchDoc Doc;
	if (!Parse(*this, Doc, MeshModel(File), SourcePath))
	{
		return false;
	}

	UMjMesh* Mesh = Doc.Actor->FindComponentByClass<UMjMesh>();
	if (!TestNotNull(TEXT("the mesh element"), Mesh))
	{
		return false;
	}

	const FString Resolved = MjResolveAssetPath(*Mesh, FString(), Mesh->File.Get(FString()));
	TestEqual(TEXT("the mesh resolves to the copy this project has"), Resolved,
		FPaths::ConvertRelativePathToFull(MeshPath));

	// And end to end: the pass that feeds the VFS has to find it, because a
	// request that comes back missing is the one that mounts nothing and leaves
	// the compiler opening a mount name.
	FRecordingSink Sink;
	FMjAssetSink Pass(Sink);
	Pass.Collect(Doc.Ref());
	TestEqual(TEXT("nothing was reported missing"), Sink.Missing.Num(), 0);
	if (TestEqual(TEXT("one mesh was collected"), Sink.Found.Num(), 1))
	{
		TestFalse(TEXT("the collected mesh is not missing"), Sink.Found[0].bMissing);
		TestTrue(TEXT("its bytes were read"), Sink.ByteCounts[0] > 0);
	}
	return true;
}

// ============================================================================
// URLab.Assets.AResolvablePathIsNeverRedirected
//
// The repair may only ever rescue a path that has already failed. A spec read
// from a directory that is really there must resolve exactly where it says,
// even when a same-named file sits under this project as well -- otherwise the
// machine that did the import quietly starts loading something else.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjResolvableNotRedirectedTest, "URLab.Assets.AResolvablePathIsNeverRedirected",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjResolvableNotRedirectedTest::RunTest(const FString& Parameters)
{
	using namespace MjAssetProvenanceTests;

	// The decoy goes exactly where a re-root of the recorded directory lands,
	// which is the only placement that can catch the repair firing when it
	// should not. So the model sits in a directory that both EXISTS and has a
	// project folder inside it: `<Saved>/URLab/TestProvenance/<n>/Saved/<n>`,
	// whose last `Saved/` tail re-roots to `<Saved>/<n>` -- the decoy.
	const FString Name = UniqueName();
	FScratchTree Outer(ScratchPath(FString(TEXT("URLab/TestProvenance/")) + Name));
	FScratchTree Decoyed(ScratchPath(Name));

	const FString BesideDir = Outer.Root / TEXT("Saved") / Name;
	IFileManager::Get().MakeDirectory(*BesideDir, /*Tree=*/true);

	const FString Real = BesideDir / TEXT("part.obj");
	const FString Decoy = Decoyed.Root / TEXT("part.obj");
	if (!TestTrue(TEXT("the real mesh was written"), WriteMesh(Real))
		|| !TestTrue(TEXT("the decoy mesh was written"), WriteMesh(Decoy)))
	{
		return false;
	}

	// The fixture is only meaningful if the decoy really is what a re-root
	// would find, so assert the geometry of the test itself.
	if (!TestNotEqual(TEXT("the decoy is somewhere else than the real file"), Real, Decoy))
	{
		return false;
	}

	FScratchDoc Doc;
	if (!Parse(*this, Doc, MeshModel(TEXT("part.obj")), BesideDir / TEXT("prov_ue.xml")))
	{
		return false;
	}

	UMjMesh* Mesh = Doc.Actor->FindComponentByClass<UMjMesh>();
	if (!TestNotNull(TEXT("the mesh element"), Mesh))
	{
		return false;
	}

	const FString Resolved = MjResolveAssetPath(*Mesh, FString(), Mesh->File.Get(FString()));
	TestEqual(TEXT("a path that resolves is left exactly where it pointed"), Resolved,
		FPaths::ConvertRelativePathToFull(Real));
	TestNotEqual(TEXT("and is not the copy a re-root would have found"), Resolved,
		FPaths::ConvertRelativePathToFull(Decoy));
	return true;
}

// ============================================================================
// URLab.Assets.ASpecDirectlyInAProjectFolderResolves
//
// The tail can be the whole directory: a spec read from `<Project>/Saved`
// itself, rather than from somewhere below it. The recorded directory then ends
// at the project folder with nothing after it, which is exactly the spelling a
// normalised path loses its trailing separator on -- and a marker that insists
// on one stops matching.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjDirectlyInProjectFolderTest, "URLab.Assets.ASpecDirectlyInAProjectFolderResolves",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjDirectlyInProjectFolderTest::RunTest(const FString& Parameters)
{
	using namespace MjAssetProvenanceTests;

	const FString Name = UniqueName();
	FScratchTree Tree(ScratchPath(Name));
	const FString MeshPath = Tree.Root / TEXT("part.obj");
	if (!TestTrue(TEXT("the scratch mesh was written"), WriteMesh(MeshPath)))
	{
		return false;
	}

	// Recorded as read from `<AbsentCheckout>/Saved` itself: the directory ends
	// at the project folder, so the tail it contributes is just `Saved/`.
	const FString AbsentRoot = FString(TEXT("/MjNoSuchCheckout_")) + UniqueName();
	const FString SourcePath = AbsentRoot / TEXT("Saved/prov_ue.xml");

	FScratchDoc Doc;
	if (!Parse(*this, Doc, MeshModel(Name / TEXT("part.obj")), SourcePath))
	{
		return false;
	}

	UMjMesh* Mesh = Doc.Actor->FindComponentByClass<UMjMesh>();
	if (!TestNotNull(TEXT("the mesh element"), Mesh))
	{
		return false;
	}

	TestEqual(TEXT("a spec read from the project folder itself still resolves"),
		MjResolveAssetPath(*Mesh, FString(), Mesh->File.Get(FString())),
		FPaths::ConvertRelativePathToFull(MeshPath));
	return true;
}

// ============================================================================
// URLab.Assets.AMissingFileStillNamesWhatItLookedFor
//
// A file that is nowhere must still be reported missing, and the path it
// reports has to be the one provenance asked for. Reporting a speculative
// re-root instead sends whoever reads the error to a directory nothing ever
// referenced.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjMissingStillNamedTest, "URLab.Assets.AMissingFileStillNamesWhatItLookedFor",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjMissingStillNamedTest::RunTest(const FString& Parameters)
{
	using namespace MjAssetProvenanceTests;

	const FString AbsentRoot = FString(TEXT("/MjNoSuchCheckout_")) + FGuid::NewGuid().ToString(EGuidFormats::Digits);
	const FString SourceDir = AbsentRoot / TEXT("Saved/URLab/ImportPrep/prov");

	FScratchDoc Doc;
	if (!Parse(*this, Doc, MeshModel(TEXT("nowhere.obj")), SourceDir / TEXT("prov_ue.xml")))
	{
		return false;
	}

	UMjMesh* Mesh = Doc.Actor->FindComponentByClass<UMjMesh>();
	if (!TestNotNull(TEXT("the mesh element"), Mesh))
	{
		return false;
	}

	TestEqual(TEXT("the reported path is the one provenance named"),
		MjResolveAssetPath(*Mesh, FString(), Mesh->File.Get(FString())),
		FPaths::ConvertRelativePathToFull(SourceDir / TEXT("nowhere.obj")));

	FRecordingSink Sink;
	FMjAssetSink Pass(Sink);
	Pass.Collect(Doc.Ref());
	TestEqual(TEXT("the mesh is reported missing"), Sink.Missing.Num(), 1);
	return true;
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
