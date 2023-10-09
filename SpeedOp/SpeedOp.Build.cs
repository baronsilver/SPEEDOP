using System.IO;
using System;

namespace UnrealBuildTool.Rules
{
    public class SpeedOp : ModuleRules
    {
        private string ModulePath
        {
            get { return ModuleDirectory; }
        }

        private string ThirdPartyPath
        {
            get { return Path.GetFullPath(Path.Combine(ModulePath, "../ThirdParty/")); }
        }

        private string BinariesPath
        {
            get { return Path.GetFullPath(Path.Combine(ModulePath, "../../Binaries/")); }
        }


        public SpeedOp(ReadOnlyTargetRules Target) : base(Target)
        {
            PrivateIncludePaths.Add("SpeedOp/Private");
            PrivatePCHHeaderFile = "Public/SpeedOp.h";

            PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "HAP" });
            PrivateDependencyModuleNames.AddRange(new string[] { "Core" });
            bUseRTTI = true;
            bEnableExceptions = true;  
        }
    }
}
