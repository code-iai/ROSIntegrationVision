// Author: Tim Fronsee <tfronsee21@gmail.com> 
#pragma once 

#include "Camera/CameraComponent.h"
#include "Components/SceneCaptureComponent2D.h"

#include "RI/Topic.h"

#include "VisionComponent.generated.h"

UCLASS()
class ROSINTEGRATIONVISION_API UVisionComponent : public UCameraComponent
{
  GENERATED_BODY()
  
public:
  UVisionComponent();
  ~UVisionComponent();
  
  void SetFramerate(const float _FrameRate);
  void Pause(const bool _Pause = true);
  bool IsPaused() const;
  
  UPROPERTY(EditAnywhere, Category = "Vision Component")
    FString ParentLink; // Defines the link that binds to the image frame.
  UPROPERTY(EditAnywhere, Category = "Vision Component")
    bool FixedCam;  // When False, TF will not be published for the camera links.
  UPROPERTY(EditAnywhere, Category = "Vision Component")
    uint32 Width;
  UPROPERTY(EditAnywhere, Category = "Vision Component")
    uint32 Height;
  UPROPERTY(EditAnywhere, Category = "Vision Component")
    float Framerate;
  UPROPERTY(EditAnywhere, Category = "Vision Component")
    int32 ServerPort;
  
protected:
  
  virtual void InitializeComponent() override;
  virtual void BeginPlay() override;
  virtual void TickComponent(float DeltaTime, 
                             enum ELevelTick TickType,
                             FActorComponentTickFunction *TickFunction) override;
  virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
   
  float FrameTime, TimePassed;

private:
	UPROPERTY()
		UTopic *_CameraInfoPublisher;

	UPROPERTY()
		UTopic *_TFPublisher;

	UPROPERTY()
		UTopic *_ImagePublisher;

	UPROPERTY()
		UTopic *_DepthPublisher;
    
	// Private data container
	class PrivateData;
	PrivateData *Priv;

	// The cameras for color, depth and objects;
	USceneCaptureComponent2D *Color;
	USceneCaptureComponent2D *Depth;
	USceneCaptureComponent2D *Object;

	UMaterialInstanceDynamic *MaterialDepthInstance;
  
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
