// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjAssetSink.h"

#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjNodeComponent.h"

namespace
{
/**
 * `File` resolved against `Directory` re-rooted onto THIS project, when that is
 * where the file turns out to be.
 *
 * Provenance is recorded as an absolute path when the spec is read, so the
 * directory every asset `file` resolves against is a fact about the machine
 * that imported it. Open the same content from a checkout at another path --
 * another machine, or just a moved folder -- and every mesh resolves into a
 * directory that is not there. Nothing is mounted in the VFS, and the compile
 * fails reporting the mount name it was left holding, a mangled
 * `<owner>_<file>`, rather than the path it went looking for.
 *
 * The recorded directory still says where under a project it sat, though, and
 * that part is portable: the tail from `Saved/`, `Content/`, `Plugins/` or
 * `Intermediate/` onward is re-rooted here and tried. It has to happen on the
 * directory rather than on the resolved path, because a `file` that climbs back
 * out with `..` -- which is what an import beside the project writes --
 * collapses those segments away and leaves nothing to key on.
 *
 * Only a path that has already failed to resolve reaches this, and only a
 * candidate that exists is returned, so a resolution that works is never
 * redirected and a genuinely missing file is still reported missing.
 */
bool ResolveUnderThisProject(const FString& Directory, const FString& File, FString& OutPath)
{
	static const TCHAR* const ProjectFolders[] = {TEXT("/Saved/"), TEXT("/Content/"), TEXT("/Plugins/"), TEXT("/Intermediate/")};

	// Normalized, then given the trailing separator back. Each marker carries
	// one so that `Content` cannot match inside `ContentAddressable`, and
	// NormalizeDirectoryName strips exactly that character -- which left a spec
	// sitting DIRECTLY in `<Project>/Saved` or `<Project>/Content` matching
	// nothing at all, the one case where the tail is the whole directory.
	FString Normalized = Directory;
	FPaths::NormalizeDirectoryName(Normalized);
	Normalized.AppendChar(TEXT('/'));

	FString ProjectRoot = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir());
	if (!ProjectRoot.EndsWith(TEXT("/")))
	{
		ProjectRoot.AppendChar(TEXT('/'));
	}

	// Every point where the recorded directory crosses into a project folder,
	// rather than whichever marker comes first in the list above: a directory
	// like `<old>/Plugins/MyPlugin/Content/Models` sits under two of them, and
	// the tail keeping the most structure is the one that names the same place
	// here. Tried longest first, so the plugin-relative reading wins over the
	// bare `Content/` one; a marker that was really part of the old machine's
	// path ABOVE its project just names a candidate that is not there.
	TArray<int32> Tails;
	for (const TCHAR* const Folder : ProjectFolders)
	{
		int32 At = INDEX_NONE;
		while ((At = Normalized.Find(Folder, ESearchCase::IgnoreCase, ESearchDir::FromStart, At + 1)) != INDEX_NONE)
		{
			Tails.AddUnique(At + 1);
		}
	}
	Tails.Sort();

	for (const int32 Tail : Tails)
	{
		const FString Rebased = FPaths::Combine(ProjectRoot, Normalized.RightChop(Tail));
		const FString Candidate = FPaths::ConvertRelativePathToFull(FPaths::Combine(Rebased, File));

		// It has to land under THIS project, or it is not what this is for. A
		// `file` carrying enough `..` climbs straight back out of whatever it
		// is rebased onto -- and a model that referenced something outside any
		// project to begin with would then be "rescued" to the very
		// machine-specific location that makes it unportable, which is the
		// opposite of the point.
		if (!Candidate.StartsWith(ProjectRoot) || !FPaths::FileExists(Candidate))
		{
			continue;
		}
		OutPath = Candidate;
		return true;
	}
	return false;
}

/**
 * The directory an asset path resolves against.
 *
 * MJCF resolves an asset path relative to the model file, then through the
 * compiler's meshdir / texturedir. The element records the file it came from, so
 * an included sub-spec's assets resolve beside the include rather than
 * beside the root -- which is the whole reason provenance is per element.
 */
FString AssetBaseDirectory(const UMjNodeComponent& Element, const FString& AssetDir)
{
	const FString SourceDir = Element.SourceFile.IsEmpty() ? FString() : FPaths::GetPath(Element.SourceFile);
	if (AssetDir.IsEmpty())
	{
		return SourceDir;
	}
	if (FPaths::IsRelative(AssetDir))
	{
		return FPaths::Combine(SourceDir, AssetDir);
	}
	return AssetDir;
}
} // namespace

#if URLAB_MJ_GEN

#include "MuJoCo/Spec/MjSpecProfile.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"

THIRD_PARTY_INCLUDES_START
#include "protospec/model_core.h"
#include "reflect.h"
THIRD_PARTY_INCLUDES_END

namespace
{
using namespace urlab::spec;

/** The MJCF string attribute `Attr` of `Node` when it was authored. */
bool AuthoredString(UMjNodeComponent& Node, const char* Attr, FString& Out)
{
	bool bAuthored = false;
	gen::DispatchByType(Node, [&](auto& Element) {
		using P = FMjInstanceProfile;
		using E = std::decay_t<decltype(Element)>;
		const int FieldId = pssdk::internal::FieldIdByName(gen::TMjElementType<E>::Value, Attr);
		if (FieldId < 0)
		{
			return;
		}
		std::string Text;
		if (pssdk::internal::GetStrField<P>(Element, FieldId, Text))
		{
			Out = gen::FMjStrPolicy::FromUtf8(Text);
			bAuthored = true;
		}
	});
	return bAuthored;
}

/** The same, for a caller that cannot act on the difference. */
FString StringAttribute(UMjNodeComponent& Node, const char* Attr)
{
	FString Out;
	AuthoredString(Node, Attr, Out);
	return Out;
}

/**
 * A node's children in whichever graph the spec lives in.
 *
 * A Blueprint's templates are linked only by USCS_Node::ChildNodes -- they are
 * never attached to one another -- so reading the attachment tree over an SCS
 * spec walks an empty list and the whole pass silently finds nothing.
 */
TArray<FMjOrderedChild> ChildrenOf(const FSpecRef& Spec, UMjNodeComponent& Parent)
{
#if WITH_EDITOR
	if (Spec.GetGraph() == EMjSpecGraph::Scs && Spec.GetBlueprint() != nullptr)
	{
		FMjScsScope Scope(*Spec.GetBlueprint());
		return FMjScsAdapter::OrderedChildren(Parent);
	}
#endif
	return FMjInstanceAdapter::OrderedChildren(Parent);
}

/** `Name` with an ordinal before its extension: `base.obj`, `base_2.obj`, ... */
FString NumberedName(const FString& Name, int32 Ordinal)
{
	const FString Extension = FPaths::GetExtension(Name, /*bIncludeDot=*/true);
	return FString::Printf(TEXT("%s_%d%s"), *Name.LeftChop(Extension.Len()), Ordinal, *Extension);
}

/**
 * `Desired`, made unique across the names already mounted.
 *
 * `Mounted` maps each name taken to the file behind it. An element naming a
 * file that is already mounted keeps that mount -- one file referenced twice is
 * one file -- and only a DIFFERENT file takes an ordinal. The walk is in spec
 * order, so the first claimant keeps the clean name and the answer does not
 * depend on the order a directory happened to be read in.
 *
 * Compared case-insensitively, because the basename fallback this exists to
 * keep out of the decision is.
 */
FString UniqueMountName(TMap<FString, FString>& Mounted, const FString& Desired, const FString& ResolvedPath)
{
	FString Name = Desired;
	for (int32 Ordinal = 2;; ++Ordinal)
	{
		const FString* const Taken = Mounted.Find(Name.ToLower());
		if (Taken == nullptr)
		{
			Mounted.Add(Name.ToLower(), ResolvedPath);
			return Name;
		}
		if (*Taken == ResolvedPath)
		{
			return Name;
		}
		Name = NumberedName(Desired, Ordinal);
	}
}

/**
 * The spec-level meshdir / texturedir, read off <compiler>.
 *
 * Authored fields only, which is the reader's own rule: `assetdir` stands in
 * for both and each of the two overrides it only where the document actually
 * says so. Reading them as plain values instead makes an unauthored field an
 * empty string that overrides `assetdir`, and a `<compiler>` carrying nothing
 * relevant erase what an earlier one set.
 */
void ReadAssetDirectories(const FSpecRef& Spec, UMjNodeComponent& Root, FString& OutMeshDir, FString& OutTextureDir)
{
	for (const FMjOrderedChild& Child : ChildrenOf(Spec, Root))
	{
		psm::ElementType Type{};
		if (!gen::ElementTypeOfNode(*Child.Node, Type) || Type != psm::ElementType::Compiler)
		{
			continue;
		}
		FString AssetDir;
		if (AuthoredString(*Child.Node, "assetdir", AssetDir))
		{
			OutMeshDir = AssetDir;
			OutTextureDir = AssetDir;
		}
		FString Directory;
		if (AuthoredString(*Child.Node, "meshdir", Directory))
		{
			OutMeshDir = Directory;
		}
		if (AuthoredString(*Child.Node, "texturedir", Directory))
		{
			OutTextureDir = Directory;
		}
	}
}
} // namespace

#endif // URLAB_MJ_GEN

FString MjAssetElementName(const UMjNodeComponent& Element)
{
	if (Element.MjName.IsSet() && !Element.MjName.GetValue().IsEmpty())
	{
		return Element.MjName.GetValue();
	}
#if URLAB_MJ_GEN
	const FString File = StringAttribute(const_cast<UMjNodeComponent&>(Element), "file");
	if (!File.IsEmpty())
	{
		return FPaths::GetBaseFilename(File);
	}
#endif
	return FString();
}

FString MjResolveAssetPath(const UMjNodeComponent& Element, const FString& AssetDir, const FString& File)
{
	if (File.IsEmpty())
	{
		return FString();
	}
	// MuJoCo takes an absolute `file` as it stands; only a relative one goes
	// through the directories. A spec whose asset was exported back out of
	// Unreal with nowhere relative to be is the case that makes the difference.
	if (!FPaths::IsRelative(File))
	{
		return File;
	}

	const FString Base = AssetBaseDirectory(Element, AssetDir);
	const FString Resolved = FPaths::ConvertRelativePathToFull(FPaths::Combine(Base, File));
	if (FPaths::FileExists(Resolved))
	{
		return Resolved;
	}

	// Not where provenance said. It may be the same place under a project that
	// has since moved, which is what opening the content on a second machine
	// looks like from here.
	FString Rebased;
	if (ResolveUnderThisProject(Base, File, Rebased))
	{
		return Rebased;
	}

	// Still the path provenance asked for: this is the one a caller reports as
	// missing, and it should name what was actually looked for.
	return Resolved;
}

void FMjAssetSink::Collect(const FSpecRef& Spec)
{
	Requests.Reset();
#if URLAB_MJ_GEN
	UMjNodeComponent* Root = Spec.GetRoot();
	if (Root == nullptr || Sink == nullptr)
	{
		return;
	}

	FString MeshDir;
	FString TextureDir;
	ReadAssetDirectories(Spec, *Root, MeshDir, TextureDir);

	// One entry per mount this pass hands out: the name, and the file it
	// carries. Two assets that would take one name are what this is for.
	TMap<FString, FString> Mounted;

	for (const FMjOrderedChild& Section : ChildrenOf(Spec, *Root))
	{
		psm::ElementType SectionType{};
		if (!gen::ElementTypeOfNode(*Section.Node, SectionType) || SectionType != psm::ElementType::Asset)
		{
			continue;
		}
		for (const FMjOrderedChild& Asset : ChildrenOf(Spec, *Section.Node))
		{
			psm::ElementType Type{};
			if (!gen::ElementTypeOfNode(*Asset.Node, Type))
			{
				continue;
			}
			const bool bMesh = Type == psm::ElementType::Mesh;
			const bool bTexture = Type == psm::ElementType::Texture;
			const bool bHeightField = Type == psm::ElementType::Hfield;
			// A skin's `.skn` resolves through meshdir and the model file's own
			// directory, exactly as a mesh does (`user_mesh.cc:3141`), so it
			// belongs on this pass rather than in a second one of its own.
			const bool bSkin = Type == psm::ElementType::Skin;
			if (!bMesh && !bTexture && !bHeightField && !bSkin)
			{
				continue;
			}

			FMjAssetRequest Request;
			Request.Element = Asset.Node;
			Request.Name = MjAssetElementName(*Asset.Node);
			Request.BaseDirectory = AssetBaseDirectory(*Asset.Node, bTexture ? TextureDir : MeshDir);

			const FString File = StringAttribute(*Asset.Node, "file");
			if (!File.IsEmpty())
			{
				Request.File = File;
				Request.ResolvedPath = MjResolveAssetPath(*Asset.Node, bTexture ? TextureDir : MeshDir, File);
				// Under a prefix the caller points the reference at whatever this
				// emits, so a basename carries it. Without one nothing rewrites
				// anything and the reference still says what the document said,
				// which is then the only name that matches it exactly -- and an
				// exact match is what keeps MuJoCo's basename fallback from
				// choosing between two `base.obj` on our behalf.
				const FString Desired = VfsPrefix.IsEmpty() ? File : VfsPrefix + FPaths::GetCleanFilename(File);
				Request.VfsName = UniqueMountName(Mounted, Desired, Request.ResolvedPath);
			}

			// Whether the file is there is a property of the resolution, not of
			// this pass: a caller that only wants to know what a model needs
			// would otherwise have to read every byte of it to find out, and
			// one that does not read the bytes would be told nothing is
			// missing.
			TArray<uint8> Bytes;
			if (!Request.ResolvedPath.IsEmpty())
			{
				Request.bMissing = !FPaths::FileExists(Request.ResolvedPath);
				if (!Request.bMissing && bLoadBytes)
				{
					Request.bMissing = !FFileHelper::LoadFileToArray(Bytes, *Request.ResolvedPath);
				}
			}

			Requests.Add(Request);
			if (Request.bMissing)
			{
				Sink->OnMissing(Request);
			}
			else if (bMesh)
			{
				Sink->OnMesh(Request, Bytes);
			}
			else if (bTexture)
			{
				Sink->OnTexture(Request, Bytes);
			}
			else if (bSkin)
			{
				Sink->OnSkin(Request, Bytes);
			}
			else
			{
				Sink->OnHeightField(Request, Bytes);
			}
		}
	}
#else
	(void)Spec;
#endif
}
