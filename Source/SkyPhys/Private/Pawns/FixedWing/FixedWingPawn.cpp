// Fill out your copyright notice in the Description page of Project Settings.

#include "Pawns/FixedWing/FixedWingPawn.h"

#include "GameFramework/RotatingMovementComponent.h"
#include "Misc/App.h"

#include "Actuation/Actuators/Filters/FirstOrderActuator.h"
#include "Actuation/Propulsion/Propeller/PropellerPropulsion.h"
#include "Turbulence/TurbulenceModel.h"

// Sets default values
AFixedWingPawn::AFixedWingPawn()
{
	// Set up components
	PropellerMesh = CreateDefaultSubobject<UPropellerPropulsionStaticMeshComponent>(TEXT("Propeller Mesh"));
	PropellerMotor = CreateDefaultSubobject<UFirstOrderActuator>(TEXT("Propeller Motor"));

	PropellerMesh->AssociateActuatorComponent(PropellerMotor);
}

void AFixedWingPawn::UpdateActuatorState(float DeltaTime)
{
	Super::UpdateActuatorState(DeltaTime);
	PropellerMesh->ApplyActuatorCommand(ActuatorCommandState.dt, DeltaTime);
}

// Calculate Motor Speed (expected Value of 0 -> 1)
void AFixedWingPawn::ApplyThrustCommand(float Value)
{
	// First clamp to make sure we're in the range we expect and then apply
	ActuatorCommandState.dt = 0.0f;// FMath::Clamp(Value, 0.0f, 1.0f);
}

// Calculate Forces and Moments for this Flight System Type
FForcesAndMoments AFixedWingPawn::CalculateAirframeForcesAndMoments()
{
	// Update stall parameters prior to calling the update.
	UpdateAerodynamicStallCalculationParameters();

	return Super::CalculateAirframeForcesAndMoments();
}

// Augment the Aerodynamic Forces on the Fixed Wing Flight System
float AFixedWingPawn::CalculateAdditionalCDParameters() const
{
	const float CDde = AerodynamicControlDerivatives.CD.CDde;

	float de = ActuatorState.de;

	float CD = CDde * de;

	return CD;
}

float AFixedWingPawn::CalculateAdditionalCLParameters() const
{
	const float CLde = AerodynamicControlDerivatives.CL.CLde;

	float de = ActuatorState.de;

	float CL = CLde * de;

	return CL;
}

float AFixedWingPawn::CalculateAdditionalCYParameters() const
{
	const float CYda = AerodynamicControlDerivatives.CY.CYda;
	const float CYdr = AerodynamicControlDerivatives.CY.CYdr;

	float da = ActuatorState.da;
	float dr = ActuatorState.dr;

	float CY = CYda * da + CYdr * dr;

	return CY;
}

// Augment the CLAlpha and CDAlpha values based on the stall model in place
FVector2D AFixedWingPawn::CalculateAdjustedCDAlphaCLAlphaForStall(float CDAlpha, float CLAlpha) const
{

	// ************************ Calculate Stall Model ************************ //

	if (AerodynamicStallParameters.bEnableStallModel)
	{
		// We make use of a flat plate stall model which blends between 0 stall and full stall (which occurs at the stall angle, Alpha0) using a transition rate, M.

		float alpha = AirspeedState.alpha;

		float SigmaAlpha = AerodynamicStallCalculationParameters.SigmaAlpha;

		CLAlpha = (1 - SigmaAlpha) * CLAlpha + SigmaAlpha * 2 * FMath::Sign(alpha) * (pow(sin(alpha), 2.0f) * cos(alpha));
		CDAlpha = (1 - SigmaAlpha) * CDAlpha + SigmaAlpha * 2 * FMath::Sign(alpha) * (pow(sin(alpha), 3.0f));
	}

	// *********************************************************************** //

	return FVector2D(CDAlpha, CLAlpha);
}

// Augment the Aerodynamic Moments on the Fixed Wing Flight System
float AFixedWingPawn::CalculateAdditionalClParameters() const
{
	const float CIda = AerodynamicControlDerivatives.CI.CIda;
	const float CIdr = AerodynamicControlDerivatives.CI.CIdr;

	float da = ActuatorState.da;
	float dr = ActuatorState.dr;

	float CI = CIda * da + CIdr * dr;

	return CI;
}

float AFixedWingPawn::CalculateAdditionalCmParameters() const
{
	const float Cmde = AerodynamicControlDerivatives.Cm.Cmde;

	float de = ActuatorState.de;

	float Cm = Cmde * de;

	return Cm;
}

float AFixedWingPawn::CalculateAdditionalCnParameters() const
{
	const float Cnda = AerodynamicControlDerivatives.Cn.Cnda;
	const float Cndr = AerodynamicControlDerivatives.Cn.Cndr;

	float da = ActuatorState.da;
	float dr = ActuatorState.dr;

	float Cn = Cnda * da + Cndr * dr;

	return Cn;
}

// Adjust the CmAlpha value based on the stall model in place
float AFixedWingPawn::CalculateAdjustedCmAlphaForStall(float CmAlpha) const
{

	// ********************* Set Up Constant Parameters ********************** //

	const float Cmfp = AerodynamicStallParameters.Cmfp;

	// *********************************************************************** //

	// ************************ Calculate Stall Model ************************ //

	if (AerodynamicStallParameters.bEnableStallModel)
	{
		// We make use of a flat plate stall model which blends between 0 stall and full stall (which occurs at the stall angle, Alpha0) using a transition rate, M.

		float alpha = AirspeedState.alpha;
		float SigmaAlpha = AerodynamicStallCalculationParameters.SigmaAlpha;

		CmAlpha = (1 - SigmaAlpha) * CmAlpha + SigmaAlpha * (Cmfp * FMath::Sign(alpha) * pow(sin(alpha), 2.0f));
	}

	// *********************************************************************** //

	return CmAlpha;
}

// Update commonly used parameters for stall calcs
void AFixedWingPawn::UpdateAerodynamicStallCalculationParameters()
{
	// ********************* Set Up Constant Parameters ********************** //

	const float M = AerodynamicStallParameters.M;
	const float Alpha0 = AerodynamicStallParameters.Alpha0;

	// *********************************************************************** //

	// ************************ Calculate Stall Model ************************ //

	float SigmaAlpha = 0.0f;
	float alpha = AirspeedState.alpha;

	if (AerodynamicStallParameters.bEnableStallModel)
	{
		// We make use of a flat plate stall model which blends between 0 stall and full stall (which occurs at the stall angle, Alpha0) using a transition rate, M and a sigmoid mixing function.
		// The scaling parameter used is SigmaAlpha.

		float Numerator = (1 + exp(-M * (alpha - Alpha0)) + exp(M * (alpha + Alpha0)));
		float Denominator = ((1 + exp(-M * (alpha - Alpha0))) * (1 + exp(M * (alpha + Alpha0))));

		// Clamp our numerator and denominator to be between 1 and FLT_MAX, to ensure that we don't get any strange numerical artifacts.
		Numerator = FMath::Clamp(Numerator, 1.0f, FLT_MAX);
		Denominator = FMath::Clamp(Denominator, 1.0f, FLT_MAX);

		SigmaAlpha = Numerator / Denominator;

		// If we still get a NaN SigmaAlpha, then just conservatively assume we are fully stalling.
		if (FMath::IsNaN(SigmaAlpha))
		{
			SigmaAlpha = 1.0f;
		}

	}

	AerodynamicStallCalculationParameters.SigmaAlpha = SigmaAlpha;

}
