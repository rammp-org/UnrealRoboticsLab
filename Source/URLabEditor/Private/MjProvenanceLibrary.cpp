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

#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Gen/Elements/MjModel.gen.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjProjectPaths.h"

namespace
{
/**
 * Write the relative form of `Record` (a member of `Template`, reached through
 * `RecordOf`) onto the template and onto every loaded instance still holding
 * the old value. An instance left behind would differ from its new archetype,
 * so reinstancing would keep the absolute path as though someone had set it,
 * and the next save would write it out -- the rule CarryEditToInstances
 * follows for any template edit. One that differs already was set on purpose
 * and is left alone.
 */
template <typename TOwner>
bool CarryRelative(TOwner& Template, FString& (*RecordOf)(TOwner&))
{
	const FString Old = RecordOf(Template);
	const FString Relative = MjProjectRelativePath(Old);
	if (Relative == Old)
	{
		return false;
	}
	TArray<UObject*> Instances;
	Template.GetArchetypeInstances(Instances);
	for (UObject* Instance : Instances)
	{
		TOwner* Follower = Cast<TOwner>(Instance);
		if (Follower != nullptr && RecordOf(*Follower) == Old)
		{
			Follower->Modify();
			RecordOf(*Follower) = Relative;
		}
	}
	Template.Modify();
	RecordOf(Template) = Relative;
	return true;
}

FString& SourceFileOf(UMjNodeComponent& Element)
{
	return Element.SourceFile;
}

FString& ModelFileOf(AMjArticulation& Articulation)
{
	return Articulation.MuJoCoXMLFile.FilePath;
}
} // namespace

int32 MjMakeProvenanceProjectRelative(UBlueprint& Blueprint)
{
	int32 Changed = 0;
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

	if (bModelInProject && Blueprint.SimpleConstructionScript != nullptr)
	{
		for (USCS_Node* Node : Blueprint.SimpleConstructionScript->GetAllNodes())
		{
			if (UMjNodeComponent* Element = Node != nullptr ? Cast<UMjNodeComponent>(Node->ComponentTemplate) : nullptr)
			{
				Changed += CarryRelative<UMjNodeComponent>(*Element, &SourceFileOf) ? 1 : 0;
			}
		}
	}
	if (Cdo != nullptr)
	{
		// The model root is a default subobject of the articulation, not a node
		// of its construction script, and the parse stamps it like any element.
		if (UMjModel* Root = Cdo->Spec.Get(); Root != nullptr && bModelInProject)
		{
			Changed += CarryRelative<UMjNodeComponent>(*Root, &SourceFileOf) ? 1 : 0;
		}
		Changed += CarryRelative<AMjArticulation>(*Cdo, &ModelFileOf) ? 1 : 0;
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
