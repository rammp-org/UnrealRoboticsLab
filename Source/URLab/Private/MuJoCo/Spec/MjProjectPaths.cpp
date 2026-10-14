// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjProjectPaths.h"

#include "Misc/Paths.h"

namespace
{
FString ProjectRoot()
{
	FString Root = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir());
	if (!Root.EndsWith(TEXT("/")))
	{
		Root.AppendChar(TEXT('/'));
	}
	return Root;
}

/** Unreal's own relative form, relative to the engine's binaries directory.
 *  A project record never climbs, so the two cannot be mistaken. */
bool IsEngineRelative(const FString& Path)
{
	return Path == TEXT("..") || Path.StartsWith(TEXT("../")) || Path.StartsWith(TEXT("..\\"));
}
} // namespace

FString MjProjectRelativePath(const FString& Path)
{
	if (Path.IsEmpty() || (FPaths::IsRelative(Path) && !IsEngineRelative(Path)))
	{
		return Path;
	}
	const FString Full = FPaths::ConvertRelativePathToFull(Path);
	const FString Root = ProjectRoot();
	if (!FPaths::IsUnderDirectory(Full, Root))
	{
		return Path;
	}
	FString Relative = Full;
	if (!FPaths::MakePathRelativeTo(Relative, *Root) || Relative.StartsWith(TEXT("..")))
	{
		return Path;
	}
	return Relative;
}

FString MjProjectAbsolutePath(const FString& Path)
{
	if (Path.IsEmpty() || !FPaths::IsRelative(Path))
	{
		return Path;
	}
	if (IsEngineRelative(Path))
	{
		return FPaths::ConvertRelativePathToFull(Path);
	}
	return FPaths::ConvertRelativePathToFull(ProjectRoot(), Path);
}
