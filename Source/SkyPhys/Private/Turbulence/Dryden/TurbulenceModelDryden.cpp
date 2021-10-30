// Fill out your copyright notice in the Description page of Project Settings.

#include "Turbulence/Dryden/TurbulenceModelDryden.h"

#include <random>

#include "Common/Utils/Integrator.h"
#include "Turbulence/Dryden/Dryden.h"

UTurbulenceModelDryden::UTurbulenceModelDryden()
{
}

FVector UTurbulenceModelDryden::GetTurbulenceBodyFrame(float Dt, float Va, float Altitude, float WindSpeed) const
{

	// Convert to Imperial for Future Calcs
	float AltitudeFt = MToFt(Altitude);
	float AirspeedFtS = MToFt(Va);
	float WindSpeedFtS = MToFt(WindSpeed);

	// Get Setup Params
	FVector ScaleLengths = GetTurbulenceScaleLengths(AltitudeFt);
	FVector RMSIntensities = GetTurbulenceRMSIntensities(AltitudeFt, WindSpeed);

	// Calculate Turbulence in body frame

	FVector Vwg = FVector(0.0f);

	if (DrydenHu)
	{
		Vwg.X = FtToM(DrydenHu->GetTurbulence(AirspeedFtS, Dt, ScaleLengths.X, RMSIntensities.X));
	}

	if (DrydenHv)
	{
		Vwg.Y = FtToM(DrydenHv->GetTurbulence(AirspeedFtS, Dt, ScaleLengths.Y, RMSIntensities.Y));
	}

	if (DrydenHw)
	{
		Vwg.Z = FtToM(DrydenHw->GetTurbulence(AirspeedFtS, Dt, ScaleLengths.Z, RMSIntensities.Z));
	}

	return Vwg;
}

FVector UTurbulenceModelDryden::GetTurbulenceScaleLengths(float Altitude) const
{
	float Lwg = 0.0f;
	float LugLvg = 0.0f;

	if (Altitude >= 10.0f)
	{
		float ClampedAltitude = FMath::Clamp(Altitude, 10.0f, 1000.0f);  // Constrain h to 10ft < h < 1000ft

		Lwg = ClampedAltitude;
		LugLvg = ClampedAltitude / pow(0.177f + 0.000823f * ClampedAltitude, 1.2f); // MIL-F-8785C pg. 55, Figure 10
	}

	return FVector(LugLvg, LugLvg, Lwg);
}

FVector UTurbulenceModelDryden::GetTurbulenceRMSIntensities(float Altitude, float WindSpeed20Ft) const
{
	float AltitudeClamped = FMath::Clamp(Altitude, 0.0f, 1000.0f);  // Constrain h to 0ft < h < 1000ft
	float SigmaW = 0.1f * WindSpeed20Ft;

	float SigmaUSigmaV = SigmaW / pow(0.177f + 0.000823f * AltitudeClamped, 0.4f); // MIL-F-8785C pg. 56, Figure 11

	return FVector(SigmaUSigmaV, SigmaUSigmaV, SigmaW);
}

