// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "TurbulenceModel.generated.h"

UCLASS(EditInlineNew, Abstract)
class SKYPHYS_API UTurbulenceModel : public UObject
{
	GENERATED_BODY()

public:
	UTurbulenceModel();

	virtual FVector GetTurbulenceBodyFrame(float Dt, float Va, float Altitude, float WindSpeed) const PURE_VIRTUAL(UTurbulenceModel::GetTurbulenceBodyFrame, return FVector{};);

protected:

	// Methods

	// Called when the game starts or when spawned
	//virtual void BeginPlay() override;
	
	// Utility Functions
	float MToFt(float M) const;
	float FtToM(float Ft) const;

};
