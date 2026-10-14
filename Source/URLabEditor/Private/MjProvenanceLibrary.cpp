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
bool MakeRelative(UObject& Owner, FString& Record)
{
	const FString Relative = MjProjectRelativePath(Record);
	if (Relative == Record)
	{
		return false;
	}
	Owner.Modify();
	Record = Relative;
	return true;
}
} // namespace

int32 MjMakeProvenanceProjectRelative(UBlueprint& Blueprint)
{
	int32 Changed = 0;
	if (Blueprint.SimpleConstructionScript != nullptr)
	{
		for (USCS_Node* Node : Blueprint.SimpleConstructionScript->GetAllNodes())
		{
			if (UMjNodeComponent* Element = Node != nullptr ? Cast<UMjNodeComponent>(Node->ComponentTemplate) : nullptr)
			{
				Changed += MakeRelative(*Element, Element->SourceFile) ? 1 : 0;
			}
		}
	}
	// The model root is a default subobject of the articulation, not a node of
	// its construction script, and the parse stamps it like any element.
	AMjArticulation* Cdo = Blueprint.GeneratedClass != nullptr
							 ? Cast<AMjArticulation>(Blueprint.GeneratedClass->GetDefaultObject())
							 : nullptr;
	if (Cdo != nullptr)
	{
		if (UMjModel* Root = Cdo->Spec.Get())
		{
			Changed += MakeRelative(*Root, Root->SourceFile) ? 1 : 0;
		}
		Changed += MakeRelative(*Cdo, Cdo->MuJoCoXMLFile.FilePath) ? 1 : 0;
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
