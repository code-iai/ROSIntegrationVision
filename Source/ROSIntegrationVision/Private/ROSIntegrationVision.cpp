// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.

#include "ROSIntegrationVision.h"

#define LOCTEXT_NAMESPACE "FROSIntegrationVisionModule"

void FROSIntegrationVisionModule::StartupModule()
{
	// This code will execute after your module is loaded into memory; the exact timing is specified in the .uplugin file per-module
	UE_LOG(LogTemp, Warning, TEXT("This is Startup Vision"));
}

void FROSIntegrationVisionModule::ShutdownModule()
{
	// This function may be called during shutdown to clean up your module.  For modules that support dynamic reloading,
	// we call this function before unloading the module.
	UE_LOG(LogTemp, Warning, TEXT("This is Shutdown Vision"));
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FROSIntegrationVisionModule, ROSIntegrationVision)