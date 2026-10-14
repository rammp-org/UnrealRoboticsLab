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

class UBlueprint;

/**
 * Record an imported articulation's provenance relative to the project wherever
 * it points inside it (MjProjectPaths.h): the SourceFile of every element in its
 * construction script and of its model root, and its MuJoCoXMLFile. Records
 * outside the project, empty, or already relative are left as they are.
 *
 * Returns how many records changed, and marks the Blueprint's package dirty
 * when any did; saving is the caller's.
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
