// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "UObject/Object.h"

#include "RI/Topic.h"
#include "std_msgs/StdMsgsString.h"
#include "MyVisionObject.generated.h"


UCLASS()
class ROSINTEGRATIONVISION_API AMyVisionActor : public AActor
{
	GENERATED_BODY()

public:

private:

	UPROPERTY()
	bool test;

	UPROPERTY()
	UTopic *_TFTopic;

	virtual void BeginPlay() override;

};

