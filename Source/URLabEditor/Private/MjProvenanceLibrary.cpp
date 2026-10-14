// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MjProvenanceLibrary.h"

#include "Engine/Blueprint.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "UObject/UObjectHash.h"

#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Gen/Elements/MjModel.gen.h"
#include "MuJoCo/Spec/MjAssetSink.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjProjectPaths.h"
#include "MuJoCo/Spec/MjSpecRef.h"

namespace
{
/** One record moving from `Old` to `New`. */
struct FMove
{
	FString Old;
	FString New;
};

/**
 * Carry each template's move onto every loaded object built from it that still
 * holds the template's old value -- placed actors, child Blueprints' templates
 * and defaults, nested archetypes. One left behind would differ from its new
 * archetype, so reinstancing would keep the old path as though someone had set
 * it, and the next save would write it out (the rule CarryEditToInstances
 * follows for any template edit). One that differs already was set on purpose
 * and is left alone.
 *
 * One pass over the loaded objects of `Class` for every move at once, each
 * object finding its nearest moved archetype up its chain: asking each template
 * for its archetype instances instead is a scan of every loaded object per
 * template, and a model of a thousand elements made that a thousand scans.
 * `Exclude` keeps class defaults out of the pass unless the records live on
 * them, as an articulation's model path does.
 */
template <typename TOwner>
void CarryToFollowers(const TMap<const UObject*, FMove>& Moves, FString& (*RecordOf)(TOwner&), EObjectFlags Exclude)
{
	if (Moves.Num() == 0)
	{
		return;
	}
	ForEachObjectOfClass(
		TOwner::StaticClass(),
		[&Moves, RecordOf](UObject* Object) {
			if (Moves.Contains(Object))
			{
				return; // a template itself; written below
			}
			for (const UObject* Archetype = Object->GetArchetype(); Archetype != nullptr; Archetype = Archetype->GetArchetype())
			{
				if (const FMove* Move = Moves.Find(Archetype))
				{
					FString& Record = RecordOf(*CastChecked<TOwner>(Object));
					if (Record == Move->Old)
					{
						Object->Modify();
						Record = Move->New;
					}
					return;
				}
			}
		},
		/*bIncludeDerivedClasses=*/true, Exclude, EInternalObjectFlags::Garbage);
}

template <typename TOwner>
void ApplyMoves(const TMap<const UObject*, FMove>& Moves, FString& (*RecordOf)(TOwner&))
{
	for (const TPair<const UObject*, FMove>& Move : Moves)
	{
		TOwner* Template = const_cast<TOwner*>(CastChecked<TOwner>(Move.Key));
		Template->Modify();
		RecordOf(*Template) = Move.Value.New;
	}
}

FString& SourceFileOf(UMjNodeComponent& Element)
{
	return Element.SourceFile;
}

FString& ModelFileOf(AMjArticulation& Articulation)
{
	return Articulation.MuJoCoXMLFile.FilePath;
}

/** Hears nothing: the paths-only asset pass is read through its requests. */
class FPathsOnlySink final : public IMjAssetSink
{
public:
	void OnMesh(const FMjAssetRequest&, const TArray<uint8>&) override {}
	void OnTexture(const FMjAssetRequest&, const TArray<uint8>&) override {}
	void OnHeightField(const FMjAssetRequest&, const TArray<uint8>&) override {}
};

/**
 * The elements whose asset lives outside the project.
 *
 * An element's record is the base its `file` resolves from, and a model inside
 * the project can still reach out of it -- a prepared copy under Saved rewrites
 * even an absolute texture path as a climb from itself. Read against another
 * checkout's project such a climb lands somewhere else, so those elements keep
 * the absolute record that finds their file today. Resolved with the records as
 * they stand, before any is changed.
 */
TSet<const UMjNodeComponent*> ElementsWithExternalAssets(UBlueprint& Blueprint)
{
	TSet<const UMjNodeComponent*> External;
	FPathsOnlySink Sink;
	FMjAssetSink Pass(Sink);
	Pass.bLoadBytes = false;
	Pass.Collect(FSpecRef::OverBlueprint(Blueprint));
	for (const FMjAssetRequest& Request : Pass.GetRequests())
	{
		if (Request.Element != nullptr && !Request.ResolvedPath.IsEmpty()
			&& MjProjectRelativePath(Request.ResolvedPath) == Request.ResolvedPath)
		{
			External.Add(Request.Element);
		}
	}
	return External;
}
} // namespace

bool MjSetModelFile(AMjArticulation& Defaults, const FString& Path)
{
	const FString Old = Defaults.MuJoCoXMLFile.FilePath;
	const FString New = MjProjectRelativePath(Path);
	if (New == Old)
	{
		return false;
	}
	TMap<const UObject*, FMove> Moves;
	Moves.Add(&Defaults, FMove{Old, New});
	// Child Blueprints' defaults are class defaults too, and they are where an
	// inherited model path lives.
	CarryToFollowers<AMjArticulation>(Moves, &ModelFileOf, RF_NoFlags);
	ApplyMoves<AMjArticulation>(Moves, &ModelFileOf);
	return true;
}

int32 MjMakeProvenanceProjectRelative(UBlueprint& Blueprint)
{
	AMjArticulation* Cdo = Blueprint.GeneratedClass != nullptr
							 ? Cast<AMjArticulation>(Blueprint.GeneratedClass->GetDefaultObject())
							 : nullptr;

	// The elements are parsed from a prepared copy under Saved when mesh
	// preparation runs, and that copy's asset paths climb back out to the model.
	// For a model outside the project those climbs are only right from where
	// the project sits now, so read against another checkout's project they land
	// somewhere else: its elements keep the absolute record, as the model does.
	const FString Model = Cdo != nullptr ? MjProjectAbsolutePath(Cdo->MuJoCoXMLFile.FilePath) : FString();
	const bool bModelInProject = Model.IsEmpty() || MjProjectRelativePath(Model) != Model;

	TMap<const UObject*, FMove> Moves;
	if (bModelInProject)
	{
		const TSet<const UMjNodeComponent*> External = ElementsWithExternalAssets(Blueprint);
		auto Consider = [&Moves, &External](UMjNodeComponent& Element) {
			const FString New = MjProjectRelativePath(Element.SourceFile);
			if (New != Element.SourceFile && !External.Contains(&Element))
			{
				Moves.Add(&Element, FMove{Element.SourceFile, New});
			}
		};
		if (Blueprint.SimpleConstructionScript != nullptr)
		{
			for (USCS_Node* Node : Blueprint.SimpleConstructionScript->GetAllNodes())
			{
				if (UMjNodeComponent* Element = Node != nullptr ? Cast<UMjNodeComponent>(Node->ComponentTemplate) : nullptr)
				{
					Consider(*Element);
				}
			}
		}
		// The model root is a default subobject of the articulation, not a node
		// of its construction script, and the parse stamps it like any element.
		if (UMjModel* Root = Cdo != nullptr ? Cdo->Spec.Get() : nullptr)
		{
			Consider(*Root);
		}
	}
	CarryToFollowers<UMjNodeComponent>(Moves, &SourceFileOf, RF_ClassDefaultObject);
	ApplyMoves<UMjNodeComponent>(Moves, &SourceFileOf);

	int32 Changed = Moves.Num();
	if (Cdo != nullptr && MjSetModelFile(*Cdo, Cdo->MuJoCoXMLFile.FilePath))
	{
		++Changed;
	}
	if (Changed > 0)
	{
		Blueprint.MarkPackageDirty();
	}
	return Changed;
}

int32 UMjProvenanceLibrary::MakeProvenanceProjectRelative(UBlueprint* Blueprint)
{
	if (Blueprint == nullptr)
	{
		return 0;
	}
	const int32 Changed = MjMakeProvenanceProjectRelative(*Blueprint);
	if (Changed > 0)
	{
		FKismetEditorUtilities::CompileBlueprint(Blueprint);
	}
	return Changed;
}
