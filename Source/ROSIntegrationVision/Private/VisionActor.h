// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "Camera/CameraActor.h"
#include "StaticMeshResources.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Materials/MaterialInstanceDynamic.h"

#include "RI/Topic.h"

// rosbridging headers
#include "VisionActor.generated.h"

UCLASS()
class ROSINTEGRATIONVISION_API AVisionActor : public ACameraActor
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	AVisionActor();

	// Sets default values for this actor's properties
	virtual ~AVisionActor();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	// Called every frame
	virtual void Tick(float DeltaSeconds) override;

	// Change the framerate on the fly
	void SetFramerate(const float _Framerate);
	// Pause/resume camera
	void Pause(const bool _Pause = true);
	// Check if paused
	bool IsPaused() const;

	UPROPERTY(EditAnywhere, Category = "RGB-D Settings")
		uint32 Width;
	UPROPERTY(EditAnywhere, Category = "RGB-D Settings")
		uint32 Height;
	UPROPERTY(EditAnywhere, Category = "RGB-D Settings")
		float Framerate;
	UPROPERTY(EditAnywhere, Category = "RGB-D Settings")
		float FieldOfView;
	UPROPERTY(EditAnywhere, Category = "RGB-D Settings")
		int32 ServerPort;

private:
	UPROPERTY()
		UTopic *_CameraInfoPublisher;

	UPROPERTY()
		UTopic *_TFPublisher;

	UPROPERTY()
		UTopic *_ImagePublisher;

	UPROPERTY()
		UTopic *_DepthPublisher;


	//ROSPluginCore* _rpc;
	//rosbridge2cpp::ROSBridge* _ros;
	// TODO destruct topics
	//rosbridge2cpp::ROSTopic* _camera_info_publisher;
	//rosbridge2cpp::ROSTopic* _image_publisher;
	//rosbridge2cpp::ROSTopic* _depth_publisher;
	//rosbridge2cpp::ROSTFBroadcaster* _tf;

	// Private data container
	class PrivateData;
	PrivateData *Priv;

	// The cameras for color, depth and objects;
	USceneCaptureComponent2D *Color;
	USceneCaptureComponent2D *Depth;
	USceneCaptureComponent2D *Object;

	UMaterialInstanceDynamic *MaterialDepthInstance;

	float FrameTime, TimePassed;
	TArray<FFloat16Color> ImageColor, ImageDepth, ImageObject;
	TArray<uint8> DataColor, DataDepth, DataObject;
	TArray<FColor> ObjectColors;
	TMap<FString, uint32> ObjectToColor;
	uint32 ColorsUsed;
	bool Running, Paused;

	void ShowFlagsBasicSetting(FEngineShowFlags &ShowFlags) const;
	void ShowFlagsLit(FEngineShowFlags &ShowFlags) const;
	void ShowFlagsPostProcess(FEngineShowFlags &ShowFlags) const;
	void ShowFlagsVertexColor(FEngineShowFlags &ShowFlags) const;
	void ReadImage(UTextureRenderTarget2D *RenderTarget, TArray<FFloat16Color> &ImageData) const;
	void ReadImageCompressed(UTextureRenderTarget2D *RenderTarget, TArray<FFloat16Color> &ImageData) const;
	void ToColorImage(const TArray<FFloat16Color> &ImageData, uint8 *Bytes) const;
	void ToDepthImage(const TArray<FFloat16Color> &ImageData, uint8 *Bytes) const;
	void StoreImage(const uint8 *ImageData, const uint32 Size, const char *Name) const;
	void GenerateColors(const uint32_t NumberOfColors);
	bool ColorObject(AActor *Actor, const FString &name);
	bool ColorAllObjects();
	void ProcessColor();
	void ProcessDepth();
	void ProcessObject();

	// in must hold Width*Height*2(float) Bytes
	void convertDepth(const uint16_t *in, __m128 *out) const;
};
