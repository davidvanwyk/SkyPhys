// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

#include "Dryden.h"
#include "Turbulence/TurbulenceModel.h"

#include "TurbulenceModelDryden.generated.h"

UCLASS()
class SKYPHYS_API UTurbulenceModelDryden : public UTurbulenceModel
{
	GENERATED_BODY()

public: 
	UTurbulenceModelDryden();

	virtual FVector GetTurbulenceBodyFrame(float Dt, float Va, float Altitude, float WindSpeed) const override;

private:

	UPROPERTY(EditAnywhere, Instanced, BlueprintReadWrite, meta = (AllowPrivateAccess = "true", DisplayName = "Dryden Turbulence Hu Model", Tooltip = "Body i Axis (u Velocity) Dryden Model"))
	UDrydenModelTFHu* DrydenHu;
	UPROPERTY(EditAnywhere, Instanced, BlueprintReadWrite, meta = (AllowPrivateAccess = "true", DisplayName = "Dryden Turbulence Hv Model", Tooltip = "Body j Axis (v Velocity) Dryden Model"))
	UDrydenModelTFHv* DrydenHv;
	UPROPERTY(EditAnywhere, Instanced, BlueprintReadWrite, meta = (AllowPrivateAccess = "true", DisplayName = "Dryden Turbulence Hw Model", Tooltip = "Body k Axis (w Velocity) Dryden Model"))
	UDrydenModelTFHw* DrydenHw;

	// Takes in altitude and returns (L_ug, L_vg, L_wg)
	// Note: Assumes only low altitude flight (ie. <300m/1000ft)
	// Altitude in ft
	FVector GetTurbulenceScaleLengths(float Altitude) const;

	// Takes in altitude and airspeed at 20ft (ie. steady airspeed) and returns turbulence intensities (sigma_ug, sigma_vg, sigma_wg)
	// Note: Assumes only low altitude flight (ie. <300m/1000ft)
	// Altitude in ft
	// WindSpeed20Ft in ft/s
	FVector GetTurbulenceRMSIntensities(float Altitude, float WindSpeed20Ft) const;

};