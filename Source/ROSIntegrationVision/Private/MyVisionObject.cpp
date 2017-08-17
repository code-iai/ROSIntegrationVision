#include "MyVisionObject.h"

#include "RI/Topic.h"
#include "ROSIntegrationGameInstance.h"


void AMyVisionActor::BeginPlay() {
	Super::BeginPlay();
	UE_LOG(LogTemp, Warning, TEXT("THIS IS MYVISIONACTOR"));

	_TFTopic = NewObject<UTopic>(UTopic::StaticClass());
	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
	_TFTopic->Init(rosinst->_Ric, TEXT("/visiontest"), TEXT("std_msgs/String"));
	TSharedPtr<ROSMessages::std_msgs::String> StringMessage(new ROSMessages::std_msgs::String("VISION for the .... VIN"));
	_TFTopic->Publish(StringMessage);
}