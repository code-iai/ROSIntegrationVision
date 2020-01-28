// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

using System.IO;

public class ROSIntegrationVision : ModuleRules
{
  private string ModulePath
  {
    get { return ModuleDirectory; }
  }

  private string ThirdPartyPath
  {
    get { return Path.GetFullPath(Path.Combine(ModulePath, "../../ThirdParty/")); }
  }

  public ROSIntegrationVision (ReadOnlyTargetRules Target) : base(Target)
  {
    PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

    PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "Public"));

    PrivateIncludePaths.Add(Path.Combine(ModuleDirectory, "Private"));

    PublicDependencyModuleNames.AddRange(
      new string[]
      {
        "Core",
        "Sockets",
        "Networking",
        "ROSIntegration"
        // ... add other public dependencies that you statically link with here ...
      }
    );

    PrivateDependencyModuleNames.AddRange(
      new string[]
      {
        "Core",
        "CoreUObject",
        "Engine",
        "RenderCore",
        "Sockets",
        "Networking",
        "ROSIntegration"
        // ... add private dependencies that you statically link with here ...	
      }
    );

    DynamicallyLoadedModuleNames.AddRange(
      new string[]
      {
        // ... add any modules that your module loads dynamically here ...
      }
    );
  }
}
