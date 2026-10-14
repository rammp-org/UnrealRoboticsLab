// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Where a model came from, recorded so that the record survives the project
// moving.
//
// An imported element remembers the file it was read from
// (UMjNodeComponent::SourceFile), and an articulation the model it reimports
// from (AMjArticulation::MuJoCoXMLFile). Recorded as absolute paths those are
// facts about one machine: they carry its user's folder names into every
// asset, they resolve only on its checkout -- anywhere else every mesh goes
// through the recovery in MjAssetSink and warns -- and on a second checkout of
// the same machine they resolve, silently, to the first one's files.
//
// So a model inside the project is recorded relative to the project, and a
// relative record is read back against whichever project is open. A model
// outside the project keeps its absolute path: there is nothing for it to be
// relative to.

#include "CoreMinimal.h"

/**
 * `Path` relative to this project's directory when it lies inside it. Any
 * other path -- empty, already relative, or outside the project -- comes back
 * unchanged.
 */
URLAB_API FString MjProjectRelativePath(const FString& Path);

/**
 * The absolute path a recorded path names: a relative record is read against
 * this project's directory, an absolute or empty one is returned as it is.
 */
URLAB_API FString MjProjectAbsolutePath(const FString& Path);
