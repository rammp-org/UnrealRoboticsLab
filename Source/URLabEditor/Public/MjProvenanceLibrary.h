// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"

#include "MjProvenanceLibrary.generated.h"

class AMjArticulation;
class UBlueprint;

/**
 * Set an articulation's model path (MuJoCoXMLFile) -- recorded relative to the
 * project where it lies inside it -- and carry the change to every loaded object
 * still holding the old value: placed actors, and child Blueprints' defaults,
 * which is where an inherited model path lives. Left behind, those would differ
 * from their new archetype and reinstancing would keep the old path as an
 * override. Returns whether the record changed.
 */
URLABEDITOR_API bool MjSetModelFile(AMjArticulation& Defaults, const FString& Path);

/**
 * Record an imported articulation's provenance relative to the project wherever
 * it points inside it (MjProjectPaths.h): the SourceFile of every element in its
 * construction script and of its model root, and its MuJoCoXMLFile. Left as
 * they are: records outside the project, empty, or already relative; the
 * elements of a model that is itself outside the project; and any element whose
 * asset file resolves outside the project (see the .cpp). Every loaded object
 * still holding a record's old value -- placed actors, child Blueprints'
 * templates and defaults -- takes the new one too, so that reinstancing does
 * not keep the old path as an override.
 *
 * Returns how many records changed on the Blueprint itself, and marks its
 * package dirty when any did; saving is the caller's.
 */
URLABEDITOR_API int32 MjMakeProvenanceProjectRelative(UBlueprint& Blueprint);

UCLASS()
class URLABEDITOR_API UMjProvenanceLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/**
	 * Bring an articulation imported before provenance was recorded relative to
	 * the project up to date: MjMakeProvenanceProjectRelative, then a compile
	 * when anything changed. Returns how many records changed. Save the
	 * Blueprint afterwards.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Import")
	static int32 MakeProvenanceProjectRelative(UBlueprint* Blueprint);
};
