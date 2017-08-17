// Fill out your copyright notice in the Description page of Project Settings.

#include "VisionActor.h"

#include "Engine/TextureRenderTarget2D.h"
#include "Camera/CameraComponent.h"
#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "UObject/Object.h"
#include "UObject/ConstructorHelpers.h"
#include "EngineUtils.h"

#include "ROSTime.h"
#include "tf2_msgs/TFMessage.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"


#include "StopTime.h"
#include "PacketBuffer.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <thread>
#include <mutex>
#include <cmath>
#include <condition_variable>

#if PLATFORM_WINDOWS
#include "immintrin.h"
#define _USE_MATH_DEFINES
#include <cmath>
#elif PLATFORM_LINUX
#include "x86intrin.h"
#endif


// Private data container so that internal structures are not visible to the outside
class ROSINTEGRATIONVISION_API AVisionActor::PrivateData
{
public:
	TSharedPtr<PacketBuffer> Buffer;
	// TCPServer Server;
	std::mutex WaitColor, WaitDepth, WaitObject, WaitDone;
	std::condition_variable CVColor, CVDepth, CVObject, CVDone;
	std::thread ThreadColor, ThreadDepth, ThreadObject;
	bool DoColor, DoDepth, DoObject;
	bool DoneColor, DoneObject;
};

// Sets default values
AVisionActor::AVisionActor() : ACameraActor(), Width(960), Height(540), Framerate(1), FieldOfView(90.0), ServerPort(10000), FrameTime(1.0f / Framerate), TimePassed(0), ColorsUsed(0)
{
	Priv = new PrivateData();

	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	// Initializing buffers for reading images from the GPU
	ImageColor.AddUninitialized(Width * Height);
	ImageDepth.AddUninitialized(Width * Height);
	ImageObject.AddUninitialized(Width * Height);

	UE_LOG(LogTemp, Warning, TEXT("Creating color camera."));
	Color = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("ColorCapture"));
	Color->SetupAttachment(RootComponent);
	Color->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
	Color->TextureTarget = CreateDefaultSubobject<UTextureRenderTarget2D>(TEXT("ColorTarget"));
	Color->TextureTarget->InitAutoFormat(Width, Height);
	Color->FOVAngle = FieldOfView;
	//Color->TextureTarget->TargetGamma = GEngine->GetDisplayGamma();

	UE_LOG(LogTemp, Warning, TEXT("Creating depth camera."))
		Depth = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("DepthCapture"));
	Depth->SetupAttachment(RootComponent);
	Depth->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
	Depth->TextureTarget = CreateDefaultSubobject<UTextureRenderTarget2D>(TEXT("DepthTarget"));
	Depth->TextureTarget->InitAutoFormat(Width, Height);
	Depth->FOVAngle = FieldOfView;

	UE_LOG(LogTemp, Warning, TEXT("Creating object camera."))
		Object = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("ObjectCapture"));
	Object->SetupAttachment(RootComponent);
	Object->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
	Object->TextureTarget = CreateDefaultSubobject<UTextureRenderTarget2D>(TEXT("ObjectTarget"));
	Object->TextureTarget->InitAutoFormat(Width, Height);
	Object->FOVAngle = FieldOfView;

	GetCameraComponent()->FieldOfView = FieldOfView;
	GetCameraComponent()->AspectRatio = Width / (float)Height;

	// Setting flags for each camera
	ShowFlagsLit(Color->ShowFlags);
	ShowFlagsPostProcess(Depth->ShowFlags);
	ShowFlagsVertexColor(Object->ShowFlags);

	UE_LOG(LogTemp, Warning, TEXT("Loading materials"))
		ConstructorHelpers::FObjectFinder<UMaterial> MaterialDepthFinder(TEXT("Material'/ROSIntegrationVision/SceneDepth.SceneDepth'"));
	if (MaterialDepthFinder.Object != nullptr)
	{
		MaterialDepthInstance = UMaterialInstanceDynamic::Create(MaterialDepthFinder.Object, Depth);
		if (MaterialDepthInstance != nullptr)
		{
			Depth->PostProcessSettings.AddBlendable(MaterialDepthInstance, 1);
		}
	}
	else
		UE_LOG(LogTemp, Error, TEXT("Could not load material for depth."));

	// Creating double buffer and setting the pointer of the server object
	Priv->Buffer = TSharedPtr<PacketBuffer>(new PacketBuffer(Width, Height, FieldOfView));
	// Priv->Server.Buffer = Priv->Buffer;
}

AVisionActor::~AVisionActor()
{
	delete Priv;
	//delete _camera_info_publisher;
	//delete _image_publisher;
	//delete _depth_publisher;
	//delete _tf;
	UE_LOG(LogTemp, Display, TEXT("VisionActor got destroyed"));
}

// Called when the game starts or when spawned
void AVisionActor::BeginPlay()
{
	Super::BeginPlay();
	UE_LOG(LogTemp, Display, TEXT("Begin play!"));

	// Starting server
	// Priv->Server.Start(ServerPort); 

	// Coloring all objects
  //  ColorAllObjects();

	Running = true;
	Paused = false;

	Priv->DoColor = false;
	Priv->DoObject = false;
	Priv->DoDepth = false;

	Priv->DoneColor = false;
	Priv->DoneObject = false;

	// Starting threads to process image data
	Priv->ThreadColor = std::thread(&AVisionActor::ProcessColor, this);
	Priv->ThreadDepth = std::thread(&AVisionActor::ProcessDepth, this);
	Priv->ThreadObject = std::thread(&AVisionActor::ProcessObject, this);

	// Establish ROS communication
	//UUnrealROSInstance* rosinst = Cast<UUnrealROSInstance>(GetGameInstance());
	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
	if (rosinst)
	{
		UE_LOG(LogTemp, Warning, TEXT("ROSIntegrationGameInstance available. Setting up ROS Topics ..."));
		_TFPublisher = NewObject<UTopic>(UTopic::StaticClass());
		_TFPublisher->Init(rosinst->_Ric, TEXT("/tf"), TEXT("tf2_msgs/TFMessage"));
		_TFPublisher->Advertise();

		_CameraInfoPublisher = NewObject<UTopic>(UTopic::StaticClass());
		_CameraInfoPublisher->Init(rosinst->_Ric, TEXT("/unreal_ros/camera_info"), TEXT("sensor_msgs/CameraInfo"));
		_CameraInfoPublisher->Advertise();

		_ImagePublisher = NewObject<UTopic>(UTopic::StaticClass());
		_ImagePublisher->Init(rosinst->_Ric, TEXT("/unreal_ros/image_color"), TEXT("sensor_msgs/Image"));
		_ImagePublisher->Advertise();

		_DepthPublisher = NewObject<UTopic>(UTopic::StaticClass());
		_DepthPublisher->Init(rosinst->_Ric, TEXT("/unreal_ros/image_depth"), TEXT("sensor_msgs/Image"));
		_DepthPublisher->Advertise();
	}
	else {
		UE_LOG(LogTemp, Warning, TEXT("UnrealROSInstance not existing."));
	}
	SetFramerate(Framerate); // Update framerate
}

// Called when the game starts or when spawned
void AVisionActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);
	UE_LOG(LogTemp, Display, TEXT("End play"));

	Running = false;

	// Stopping processing threads
	Priv->DoColor = true;
	Priv->DoDepth = true;
	Priv->DoObject = true;
	Priv->CVColor.notify_one();
	Priv->CVDepth.notify_one();
	Priv->CVObject.notify_one();

	Priv->ThreadColor.join();
	Priv->ThreadDepth.join();
	Priv->ThreadObject.join();

	// Priv->Server.Stop();
}

// Called every frame
void AVisionActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// Check if paused
	if (Paused)
	{
		return;
	}

	// Check for framerate
	TimePassed += DeltaTime;
	if (TimePassed < FrameTime)
	{
		return;
	}
	TimePassed -= FrameTime;
	MEASURE_TIME("Tick");

	UpdateComponentTransforms();

	//  std::cout << "FrameRate is: " << Framerate << std::endl;
	//  std::cout << "FrameTime is: " << FrameTime << std::endl;
	//
	  // return; // TODO auskommentieren
	  // Check if client is connected
	  // if(!Priv->Server.HasClient())
	  // {
	  //   return;
	  // }

	FDateTime Now = FDateTime::UtcNow();
	Priv->Buffer->HeaderWrite->TimestampCapture = Now.ToUnixTimestamp() * 1000000000 + Now.GetMillisecond() * 1000000;

	FVector Translation = GetActorLocation();
	FQuat Rotation = GetActorQuat();
	// Convert to meters and ROS coordinate system
	Priv->Buffer->HeaderWrite->Translation.X = Translation.X / 100.0f;
	Priv->Buffer->HeaderWrite->Translation.Y = -Translation.Y / 100.0f;
	Priv->Buffer->HeaderWrite->Translation.Z = Translation.Z / 100.0f;
	Priv->Buffer->HeaderWrite->Rotation.X = -Rotation.X;
	Priv->Buffer->HeaderWrite->Rotation.Y = Rotation.Y;
	Priv->Buffer->HeaderWrite->Rotation.Z = -Rotation.Z;
	Priv->Buffer->HeaderWrite->Rotation.W = Rotation.W;

	// Start writing to buffer
	Priv->Buffer->StartWriting(ObjectToColor, ObjectColors);

	// Read color image and notify processing thread
	Priv->WaitColor.lock();
	ReadImage(Color->TextureTarget, ImageColor);
	Priv->WaitColor.unlock();
	Priv->DoColor = true;
	Priv->CVColor.notify_one();

	// Read object image and notify processing thread
	Priv->WaitObject.lock();
	ReadImage(Object->TextureTarget, ImageObject);
	Priv->WaitObject.unlock();
	Priv->DoObject = true;
	Priv->CVObject.notify_one();

	/* Read depth image and notify processing thread. Depth processing is called last,
	 * because the color image processing thread take more time so they can already begin.
	 * The depth processing thread will wait for the others to be finished and then releases
	 * the buffer.
	 */
	Priv->WaitDepth.lock();
	ReadImage(Depth->TextureTarget, ImageDepth);
	Priv->WaitDepth.unlock();
	Priv->DoDepth = true;
	Priv->CVDepth.notify_one();

	Priv->Buffer->StartReading();
	uint32_t xSize = Priv->Buffer->HeaderRead->Size;
	uint32_t xSizeHeader = Priv->Buffer->HeaderRead->SizeHeader; // Size of the header
	uint32_t xMapEntries = Priv->Buffer->HeaderRead->MapEntries; // Number of map entries at the end of the packet
	uint32_t xWidth = Priv->Buffer->HeaderRead->Width; // Width of the images
	uint32_t xHeight = Priv->Buffer->HeaderRead->Height; // Height of the images

  //  OUT_INFO(TEXT("Buffer Stats: %d %d %d %d %d"), xSize, xSizeHeader, xMapEntries, xWidth, xHeight);

	// Get the data offsets for the different types of images that are in the buffer
	const uint32_t& OffsetColor = Priv->Buffer->OffsetColor;
	const uint32_t& OffsetDepth = Priv->Buffer->OffsetDepth;
	const uint32_t& OffsetObject = Priv->Buffer->OffsetObject;
	// * - Depth image data (width * height * 2 Bytes (Float16))
	uint8_t* DepthPtr = &Priv->Buffer->Read[OffsetDepth];
	uint32_t TargetDepthBufSize = Width*Height * 4;
	uint8_t* TargetDepthBuf = new uint8_t[TargetDepthBufSize]; // Allocate a byte for every pixel * 4 Bytes for a single 32Bit Float

	const uint32_t ColorImageSize = Width * Height * 3;
	convertDepth((uint16_t *)DepthPtr, (__m128*)TargetDepthBuf);
	// convertDepth((uint16_t *)packet.pDepth, (__m128*)&msgDepth->data[0]);

	UE_LOG(LogTemp, Display, TEXT("Buffer Offsets: %d %d %d"), OffsetColor, OffsetDepth, OffsetObject);

	FROSTime time = FROSTime::Now();

	TSharedPtr<ROSMessages::sensor_msgs::Image> ImageMessage(new ROSMessages::sensor_msgs::Image());

	ImageMessage->header.seq = 0;
	ImageMessage->header.time = time;
	ImageMessage->header.frame_id = TEXT("/unreal_ros/image_optical_frame");
	ImageMessage->height = Height;
	ImageMessage->width = Width;
	ImageMessage->encoding = TEXT("bgr8");
	ImageMessage->step = Width * 3;
	ImageMessage->data_ptr = &Priv->Buffer->Read[OffsetColor];
	_ImagePublisher->Publish(ImageMessage);

	TSharedPtr<ROSMessages::sensor_msgs::Image> DepthMessage(new ROSMessages::sensor_msgs::Image());

	DepthMessage->header.seq = 0;
	DepthMessage->header.time = time;
	DepthMessage->header.frame_id = TEXT("/unreal_ros/image_optical_frame");
	DepthMessage->height = Height;
	DepthMessage->width = Width;
	DepthMessage->encoding = TEXT("32FC1");
	DepthMessage->step = Width * 4;
	DepthMessage->data_ptr = TargetDepthBuf;
	_DepthPublisher->Publish(DepthMessage);




	/*bson_t *depth_msg;
	depth_msg = BCON_NEW(
		"header", "{",
		"seq", BCON_INT32(0),
		"stamp", "{",
		"secs", BCON_INT32(time.sec_),
		"nsecs", BCON_INT32(time.nsec_),
		"}",
		"frame_id", BCON_UTF8("/unreal_ros/image_optical_frame"),
		"}",
		"height", BCON_INT32(Height),
		"width", BCON_INT32(Width),
		"encoding", BCON_UTF8("32FC1"),
		"step", BCON_INT32((Width * 4))
	);
	bson_append_binary(depth_msg, "data", -1, BSON_SUBTYPE_BINARY, TargetDepthBuf, TargetDepthBufSize);
	_depth_publisher->Publish(depth_msg);*/




	/*rosbridge2cpp::ROSTime time = rosbridge2cpp::ROSTime::now();

	bson_t *image_msg;
	image_msg = BCON_NEW(
		"header", "{",
		"seq", BCON_INT32(0),
		"stamp", "{",
		"secs", BCON_INT32(time.sec_),
		"nsecs", BCON_INT32(time.nsec_),
		"}",
		"frame_id", BCON_UTF8("/unreal_ros/image_optical_frame"),
		"}",
		"height",BCON_INT32(Height),
		"width",BCON_INT32(Width),
		"encoding",BCON_UTF8("bgr8"),
		"step",BCON_INT32((Width * 3))
		);
	bson_append_binary(image_msg,"data",-1,BSON_SUBTYPE_BINARY, &Priv->Buffer->Read[OffsetColor], ColorImageSize);
	_image_publisher->Publish(image_msg);

	bson_t *depth_msg;
	depth_msg = BCON_NEW(
		"header", "{",
		"seq", BCON_INT32(0),
		"stamp", "{",
		"secs", BCON_INT32(time.sec_),
		"nsecs", BCON_INT32(time.nsec_),
		"}",
		"frame_id", BCON_UTF8("/unreal_ros/image_optical_frame"),
		"}",
		"height",BCON_INT32(Height),
		"width",BCON_INT32(Width),
		"encoding",BCON_UTF8("32FC1"),
		"step",BCON_INT32((Width * 4))
		);
	bson_append_binary(depth_msg,"data",-1,BSON_SUBTYPE_BINARY, TargetDepthBuf, TargetDepthBufSize);
	_depth_publisher->Publish(depth_msg);
*/
// delete image;
// free(image);
	Priv->Buffer->DoneReading();




	FVector pos = GetRootComponent()->GetComponentLocation();
	//  UE_LOG(LogTemp, Warning, TEXT("Actor is now at %f, %f, %f"), pos.X, pos.Y, pos.Z);
	FString posstr = GetRootComponent()->GetComponentLocation().ToString();
	//  UE_LOG(LogTemp, Warning, TEXT("Actor is now at (str) %s"), *posstr);

	double x = Priv->Buffer->HeaderRead->Translation.X;
	double y = Priv->Buffer->HeaderRead->Translation.Y;
	double z = Priv->Buffer->HeaderRead->Translation.Z;
	double rx = Priv->Buffer->HeaderRead->Rotation.X;
	double ry = Priv->Buffer->HeaderRead->Rotation.Y;
	double rz = Priv->Buffer->HeaderRead->Rotation.Z;
	double rw = Priv->Buffer->HeaderRead->Rotation.W;



	{
		TSharedPtr<ROSMessages::tf2_msgs::TFMessage> TFMessage(new ROSMessages::tf2_msgs::TFMessage());
		ROSMessages::geometry_msgs::TransformStamped TransformStamped;
		TransformStamped.header.seq = 0;
		TransformStamped.header.time = time;
		TransformStamped.header.frame_id = TEXT("/world");
		TransformStamped.child_frame_id = TEXT("/unreal_ros/image_frame");
		TransformStamped.transform.translation.x = x;
		TransformStamped.transform.translation.y = y;
		TransformStamped.transform.translation.z = z;
		TransformStamped.transform.rotation.x = rx;
		TransformStamped.transform.rotation.y = ry;
		TransformStamped.transform.rotation.z = rz;
		TransformStamped.transform.rotation.w = rw;

		TFMessage->transforms.Add(TransformStamped);

		_TFPublisher->Publish(TFMessage);
	}
	// Publish optical frame
	FRotator CameraLinkRotator(0.0, -90.0, 90.0);
	FQuat CameraLinkQuaternion(CameraLinkRotator);

	{
		TSharedPtr<ROSMessages::tf2_msgs::TFMessage> TFMessage(new ROSMessages::tf2_msgs::TFMessage());
		ROSMessages::geometry_msgs::TransformStamped TransformStamped;
		TransformStamped.header.seq = 0;
		TransformStamped.header.time = time;
		TransformStamped.header.frame_id = TEXT("/unreal_ros/image_frame");
		TransformStamped.child_frame_id = TEXT("/unreal_ros/image_optical_frame");
		TransformStamped.transform.translation.x = 0;
		TransformStamped.transform.translation.y = 0;
		TransformStamped.transform.translation.z = 0;
		TransformStamped.transform.rotation.x = CameraLinkQuaternion.X;
		TransformStamped.transform.rotation.y = CameraLinkQuaternion.Y;
		TransformStamped.transform.rotation.z = CameraLinkQuaternion.Z;
		TransformStamped.transform.rotation.w = CameraLinkQuaternion.W;

		TFMessage->transforms.Add(TransformStamped);

		_TFPublisher->Publish(TFMessage);
	}

	// Construct and publish CameraInfo

	const float FOVX = Height > Width ? FieldOfView * Width / Height : FieldOfView;
	const float FOVY = Width > Height ? FieldOfView * Height / Width : FieldOfView;
	double halfFOVX = FOVX * PI / 360.0; // was M_PI on gcc
	double halfFOVY = FOVY * PI / 360.0; // was M_PI on gcc
	const double cX = Width / 2.0;
	const double cY = Height / 2.0;

	//  const double K0 = cX / std::tan(halfFOVX);
	//  const double K2 = cX;
	//  const double K4 = cY / std::tan(halfFOVY);
	//  const double K5 = cY;
	//  const double K8 = 1;
	//
	const double K0 = cX / std::tan(halfFOVX);
	const double K2 = cX;
	const double K4 = K0;
	const double K5 = cY;
	const double K8 = 1;

	const double P0 = K0;
	const double P2 = K2;
	const double P5 = K4;
	const double P6 = K5;
	const double P10 = 1;

	{
		TSharedPtr<ROSMessages::sensor_msgs::CameraInfo> CamInfo(new ROSMessages::sensor_msgs::CameraInfo());
		CamInfo->header.seq = 0;
		CamInfo->header.time = time;
		//CamInfo->header.frame_id = 
		CamInfo->height = Height;
		CamInfo->width = Width;
		CamInfo->distortion_model = TEXT("plumb_bob");
		CamInfo->D[0] = 0;
		CamInfo->D[1] = 0;
		CamInfo->D[2] = 0;
		CamInfo->D[3] = 0;
		CamInfo->D[4] = 0;

		CamInfo->K[0] = K0;
		CamInfo->K[1] = 0;
		CamInfo->K[2] = K2;
		CamInfo->K[3] = 0;
		CamInfo->K[4] = K4;
		CamInfo->K[5] = K5;
		CamInfo->K[6] = 0;
		CamInfo->K[7] = 0;
		CamInfo->K[8] = K8;

		CamInfo->R[0] = 1;
		CamInfo->R[1] = 0;
		CamInfo->R[2] = 0;
		CamInfo->R[3] = 0;
		CamInfo->R[4] = 1;
		CamInfo->R[5] = 0;
		CamInfo->R[6] = 0;
		CamInfo->R[7] = 0;
		CamInfo->R[8] = 1;

		CamInfo->P[0] = P0;
		CamInfo->P[1] = 0;
		CamInfo->P[2] = P2;
		CamInfo->P[3] = 0;
		CamInfo->P[4] = 0;
		CamInfo->P[5] = P5;
		CamInfo->P[6] = P6;
		CamInfo->P[7] = 0;
		CamInfo->P[8] = 0;
		CamInfo->P[9] = 0;
		CamInfo->P[10] = P10;
		CamInfo->P[11] = 0;

		CamInfo->binning_x = 0;
		CamInfo->binning_y = 0;

		CamInfo->roi.x_offset = 0;
		CamInfo->roi.y_offset = 0;
		CamInfo->roi.height = 0;
		CamInfo->roi.width = 0;
		CamInfo->roi.do_rectify = false;

		_CameraInfoPublisher->Publish(CamInfo);

	}
	
	// Clean up
	delete(TargetDepthBuf);
}

void AVisionActor::SetFramerate(const float _Framerate)
{
	Framerate = _Framerate;
	FrameTime = 1.0f / _Framerate;
	TimePassed = 0;
}

void AVisionActor::Pause(const bool _Pause)
{
	Paused = _Pause;
}

bool AVisionActor::IsPaused() const
{
	return Paused;
}

void AVisionActor::ShowFlagsBasicSetting(FEngineShowFlags &ShowFlags) const
{
	ShowFlags = FEngineShowFlags(EShowFlagInitMode::ESFIM_All0);
	ShowFlags.SetRendering(true);
	ShowFlags.SetStaticMeshes(true);
	ShowFlags.SetLandscape(true);
	ShowFlags.SetInstancedFoliage(true);
	ShowFlags.SetInstancedGrass(true);
	ShowFlags.SetInstancedStaticMeshes(true);
}

void AVisionActor::ShowFlagsLit(FEngineShowFlags &ShowFlags) const
{
	ShowFlagsBasicSetting(ShowFlags);
	ShowFlags = FEngineShowFlags(EShowFlagInitMode::ESFIM_Game);
	ApplyViewMode(VMI_Lit, true, ShowFlags);
	ShowFlags.SetMaterials(true);
	ShowFlags.SetLighting(true);
	ShowFlags.SetPostProcessing(true);
	// ToneMapper needs to be enabled, otherwise the screen will be very dark
	ShowFlags.SetTonemapper(true);
	// TemporalAA needs to be disabled, otherwise the previous frame might contaminate current frame.
	// Check: https://answers.unrealengine.com/questions/436060/low-quality-screenshot-after-setting-the-actor-pos.html for detail
	ShowFlags.SetTemporalAA(false);
	ShowFlags.SetAntiAliasing(true);
	ShowFlags.SetEyeAdaptation(false); // Eye adaption is a slow temporal procedure, not useful for image capture
}

void AVisionActor::ShowFlagsPostProcess(FEngineShowFlags &ShowFlags) const
{
	ShowFlagsBasicSetting(ShowFlags);
	ShowFlags.SetPostProcessing(true);
	ShowFlags.SetPostProcessMaterial(true);

	GVertexColorViewMode = EVertexColorViewMode::Color;
}

void AVisionActor::ShowFlagsVertexColor(FEngineShowFlags &ShowFlags) const
{
	ShowFlagsLit(ShowFlags);
	ApplyViewMode(VMI_Lit, true, ShowFlags);

	// From MeshPaintEdMode.cpp:2942
	ShowFlags.SetMaterials(false);
	ShowFlags.SetLighting(false);
	ShowFlags.SetBSPTriangles(true);
	ShowFlags.SetVertexColors(true);
	ShowFlags.SetPostProcessing(false);
	ShowFlags.SetHMDDistortion(false);
	ShowFlags.SetTonemapper(false); // This won't take effect here

	GVertexColorViewMode = EVertexColorViewMode::Color;
}

void AVisionActor::ReadImage(UTextureRenderTarget2D *RenderTarget, TArray<FFloat16Color> &ImageData) const
{
	FTextureRenderTargetResource *RenderTargetResource = RenderTarget->GameThread_GetRenderTargetResource();
	RenderTargetResource->ReadFloat16Pixels(ImageData);
}

void AVisionActor::ToColorImage(const TArray<FFloat16Color> &ImageData, uint8 *Bytes) const
{
	const FFloat16Color *itI = ImageData.GetData();
	uint8_t *itO = Bytes;

	// Converts Float colors to bytes
	for (size_t i = 0; i < ImageData.Num(); ++i, ++itI, ++itO)
	{
		*itO = (uint8_t)std::round((float)itI->B * 255.f);
		*++itO = (uint8_t)std::round((float)itI->G * 255.f);
		*++itO = (uint8_t)std::round((float)itI->R * 255.f);
	}
	return;
}

void AVisionActor::ToDepthImage(const TArray<FFloat16Color> &ImageData, uint8 *Bytes) const
{
	const FFloat16Color *itI = ImageData.GetData();
	uint16_t *itO = reinterpret_cast<uint16_t *>(Bytes);

	// Just copies the encoded Float16 values
	for (size_t i = 0; i < ImageData.Num(); ++i, ++itI, ++itO)
	{
		*itO = itI->R.Encoded;
	}
	return;
}

void AVisionActor::StoreImage(const uint8 *ImageData, const uint32 Size, const char *Name) const
{
	std::ofstream File(Name, std::ofstream::out | std::ofstream::binary | std::ofstream::trunc);
	File.write(reinterpret_cast<const char *>(ImageData), Size);
	File.close();
	return;
}

/* Generates at least NumberOfColors different colors.
 * It takes MaxHue different Hue values and additional steps ind Value and Saturation to get
 * the number of needed colors.
 */
void AVisionActor::GenerateColors(const uint32_t NumberOfColors)
{
	const int32_t MaxHue = 50;
	// It shifts the next Hue value used, so that colors next to each other are not very similar. This is just important for humans
	const int32_t ShiftHue = 21;
	const float MinSat = 0.65;
	const float MinVal = 0.65;

	uint32_t HueCount = MaxHue;
	uint32_t SatCount = 1;
	uint32_t ValCount = 1;

	// Compute how many different Saturations and Values are needed
	int32_t left = std::max<int32_t>(0, NumberOfColors - HueCount);
	while (left > 0)
	{
		if (left > 0)
		{
			++ValCount;
			left = NumberOfColors - SatCount * ValCount * HueCount;
		}
		if (left > 0)
		{
			++SatCount;
			left = NumberOfColors - SatCount * ValCount * HueCount;
		}
	}

	const float StepHue = 360.0f / HueCount;
	const float StepSat = (1.0f - MinSat) / std::max(1.0f, SatCount - 1.0f);
	const float StepVal = (1.0f - MinVal) / std::max(1.0f, ValCount - 1.0f);

	ObjectColors.Reserve(SatCount * ValCount * HueCount);
	UE_LOG(LogTemp, Display, TEXT("Generating %d colors."), SatCount * ValCount * HueCount);

	FLinearColor HSVColor;
	for (uint32_t s = 0; s < SatCount; ++s)
	{
		HSVColor.G = 1.0f - s * StepSat;
		for (uint32_t v = 0; v < ValCount; ++v)
		{
			HSVColor.B = 1.0f - v * StepVal;
			for (uint32_t h = 0; h < HueCount; ++h)
			{
				HSVColor.R = ((h * ShiftHue) % MaxHue) * StepHue;
				ObjectColors.Add(HSVColor.HSVToLinearRGB().ToFColor(false));
				UE_LOG(LogTemp, Display, TEXT("Added color %d: %d %d %d"), ObjectColors.Num(), ObjectColors.Last().R, ObjectColors.Last().G, ObjectColors.Last().B);
			}
		}
	}
}

bool AVisionActor::ColorObject(AActor *Actor, const FString &name)
{
	const FColor &ObjectColor = ObjectColors[ObjectToColor[name]];
	TArray<UMeshComponent *> PaintableComponents;
	Actor->GetComponents<UMeshComponent>(PaintableComponents);

	for (auto MeshComponent : PaintableComponents)
	{
		if (MeshComponent == nullptr)
			continue;

		if (UStaticMeshComponent *StaticMeshComponent = Cast<UStaticMeshComponent>(MeshComponent))
		{
			if (UStaticMesh *StaticMesh = StaticMeshComponent->GetStaticMesh())
			{
				uint32 PaintingMeshLODIndex = 0;
				uint32 NumLODLevel = StaticMesh->RenderData->LODResources.Num();
				//check(NumLODLevel == 1);
				FStaticMeshLODResources &LODModel = StaticMesh->RenderData->LODResources[PaintingMeshLODIndex];
				FStaticMeshComponentLODInfo *InstanceMeshLODInfo = NULL;

				// PaintingMeshLODIndex + 1 is the minimum requirement, enlarge if not satisfied
				StaticMeshComponent->SetLODDataCount(PaintingMeshLODIndex + 1, StaticMeshComponent->LODData.Num());
				InstanceMeshLODInfo = &StaticMeshComponent->LODData[PaintingMeshLODIndex];

				{
					InstanceMeshLODInfo->OverrideVertexColors = new FColorVertexBuffer;

					FColor FillColor = FColor(255, 255, 255, 255);
					InstanceMeshLODInfo->OverrideVertexColors->InitFromSingleColor(FColor::White, LODModel.GetNumVertices());
				}

				uint32 NumVertices = LODModel.GetNumVertices();
				//check(InstanceMeshLODInfo->OverrideVertexColors);
				//check(NumVertices <= InstanceMeshLODInfo->OverrideVertexColors->GetNumVertices());

				for (uint32 ColorIndex = 0; ColorIndex < NumVertices; ++ColorIndex)
				{
					uint32 NumOverrideVertexColors = InstanceMeshLODInfo->OverrideVertexColors->GetNumVertices();
					uint32 NumPaintedVertices = InstanceMeshLODInfo->PaintedVertices.Num();
					InstanceMeshLODInfo->OverrideVertexColors->VertexColor(ColorIndex) = ObjectColor;
				}
				BeginInitResource(InstanceMeshLODInfo->OverrideVertexColors);
				StaticMeshComponent->MarkRenderStateDirty();
			}
		}
	}
	return true;
}

bool AVisionActor::ColorAllObjects()
{
	uint32_t NumberOfActors = 0;

	for (TActorIterator<AActor> ActItr(GetWorld()); ActItr; ++ActItr)
	{
		++NumberOfActors;
		FString ActorName = ActItr->GetHumanReadableName();
		UE_LOG(LogTemp, Display, TEXT("Actor with name: %s."), *ActorName);
	}

	//   for(TActorIterator<ACharacter> CharItr(GetWorld()); CharItr; ++CharItr)
	//   {
	//     ++NumberOfActors;
	//     FString CharName = CharItr->GetHumanReadableName();
	//     OUT_INFO(TEXT("Character with name: %s."), *CharName);
	//   }
	//   
	UE_LOG(LogTemp, Display, TEXT("Found %d Actors."), NumberOfActors);
	GenerateColors(NumberOfActors * 2);

	for (TActorIterator<AActor> ActItr(GetWorld()); ActItr; ++ActItr)
	{
		FString ActorName = ActItr->GetHumanReadableName();
		if (!ObjectToColor.Contains(ActorName))
		{
			check(ColorsUsed < (uint32)ObjectColors.Num());
			ObjectToColor.Add(ActorName, ColorsUsed);
			UE_LOG(LogTemp, Display, TEXT("Adding color %d for object %s."), ColorsUsed, *ActorName);

			++ColorsUsed;
		}

		UE_LOG(LogTemp, Display, TEXT("Coloring object %s."), *ActorName);
		ColorObject(*ActItr, ActorName);
	}

	//   for(TActorIterator<ACharacter> CharItr(GetWorld()); CharItr; ++CharItr)
	//   {
	//     FString ActorName = CharItr->GetHumanReadableName();
	//     if(!ObjectToColor.Contains(ActorName))
	//     {
	//       check(ColorsUsed < (uint32)ObjectColors.Num());
	//       ObjectToColor.Add(ActorName, ColorsUsed);
	//       OUT_INFO(TEXT("Adding color %d for object %s."), ColorsUsed, *ActorName);
	//       ++ColorsUsed;
	//     }
	// 
	//     OUT_INFO(TEXT("Coloring object %s."), *ActorName);
	//     ColorObject(*CharItr, ActorName);
	//   }

	return true;
}

void AVisionActor::ProcessColor()
{
	while (true)
	{
		std::unique_lock<std::mutex> WaitLock(Priv->WaitColor);
		Priv->CVColor.wait(WaitLock, [this] {return Priv->DoColor; });
		Priv->DoColor = false;
		if (!this->Running) break;
		ToColorImage(ImageColor, Priv->Buffer->Color);

		Priv->DoneColor = true;
		Priv->CVDone.notify_one();
	}
}

void AVisionActor::ProcessDepth()
{
	while (true)
	{
		std::unique_lock<std::mutex> WaitLock(Priv->WaitDepth);
		Priv->CVDepth.wait(WaitLock, [this] {return Priv->DoDepth; });
		Priv->DoDepth = false;
		if (!this->Running) break;
		ToDepthImage(ImageDepth, Priv->Buffer->Depth);

		// Wait for both other processing threads to be done.
		std::unique_lock<std::mutex> WaitDoneLock(Priv->WaitDone);
		Priv->CVDone.wait(WaitDoneLock, [this] {return Priv->DoneColor && Priv->DoneObject; });

		Priv->DoneColor = false;
		Priv->DoneObject = false;

		// Complete Buffer
		Priv->Buffer->DoneWriting();
	}
}

void AVisionActor::ProcessObject()
{
	while (true)
	{
		std::unique_lock<std::mutex> WaitLock(Priv->WaitObject);
		Priv->CVObject.wait(WaitLock, [this] {return Priv->DoObject; });
		Priv->DoObject = false;
		if (!this->Running) break;
		ToColorImage(ImageObject, Priv->Buffer->Object);

		Priv->DoneObject = true;
		Priv->CVDone.notify_one();
	}
}

// TODO maybe shift towards "server" who publishs async 
void AVisionActor::convertDepth(const uint16_t *in, __m128 *out) const
{
	const size_t size = (Width * Height) / 4;
	for (size_t i = 0; i < size; ++i, in += 4, ++out)
	{
		*out = _mm_cvtph_ps(_mm_set_epi16(0, 0, 0, 0, *(in + 3), *(in + 2), *(in + 1), *(in + 0)));
	}
	//#else
		//UE_LOG(LogTemp, Display, TEXT("F16C not supported on your platform. Skipping depth image generation"));
	//#endif 
}
