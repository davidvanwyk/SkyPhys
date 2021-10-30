// Fill out your copyright notice in the Description page of Project Settings.


#include "Pawns/FlyingPawn.h"
#include "DrawDebugHelpers.h"
#include "Kismet/GameplayStatics.h"
#include "Kismet/KismetMathLibrary.h"
#include "Turbulence/TurbulenceModel.h"
#include "Actuation/Propulsion/Propulsion.h"
#include "UObject/Field.h"

#include "Common/Utils/Helpers.h"

// Sets default values
AFlyingPawn::AFlyingPawn()
{
 	// Set this pawn to call Tick() every frame. All pawns of this type have custom physics, so will need to tick.
	PrimaryActorTick.bCanEverTick = true;
	// Give us the ability to pause without ticking 
	PrimaryActorTick.bTickEvenWhenPaused = false;
	// Ensure that our main tick happens pre-physics, as we will then apply physics in the physics substeps (if they exist).
	PrimaryActorTick.TickGroup = TG_PrePhysics;

	// Set up components
	
	// We expect a base mesh for all flying pawns, and the origin of this component should be the CG of the pawn (where all forces and moments calculated are expected to act)
	AirframeMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Airframe Mesh"));
	RootComponent = AirframeMesh;

	// Grab the body instance, which should be used instead of the root component for applying physics.
	PhysicsBody = Cast<UPrimitiveComponent>(AirframeMesh)->GetBodyInstance();
}

// Called when the game starts or when spawned
void AFlyingPawn::BeginPlay()
{
	Super::BeginPlay();

	// Ensure our root component has expected physics parameters 
	AirframeMesh->SetSimulatePhysics(true);
	AirframeMesh->SetEnableGravity(true);
	AirframeMesh->SetMassOverrideInKg(NAME_None, SystemCharacteristics.Mass, true); // Override the mass with the mass specified in system characteristics (this just forces things to be consistent, really).

	// Find the UDS Weather Actor (if one has been added to the scene).
	SetupWeather();

	// Pre-calculate any characteristics that will be needed during play, but might be computationally intensive and shouldn't be re-done if not needed.
	PreCalculateSystemCharacteristics();

	// Get all of our propulsors so that we can use them to generate forces and moments a bit later.
	GetComponents(Propulsors);

	// Bind to the substep tick method.
	CalculateCustomPhysics.BindUObject(this, &AFlyingPawn::SubstepTick);
}

void AFlyingPawn::SetupWeather()
{

	// If the UDS Weather Class Type has been defined
	if (WeatherSetup.UDSWeatherClassType)
	{
		// Then grab the UDS weather actor if it exists
		TArray<AActor*> WeatherActors;
		UGameplayStatics::GetAllActorsOfClass(GetWorld(), WeatherSetup.UDSWeatherClassType, WeatherActors);
		if (WeatherActors.Num() > 0)
		{
			// Assume it will be the first returned class (as there should only be one...)
			UDSWeatherActor = WeatherActors[0];
		}
	}
}

// Precalculation of System Characteristics (ie. any parameters that should only be calculated once on game start)
void AFlyingPawn::PreCalculateSystemCharacteristics()
{
	// Our Tensors make use of the Eigen library.
	using namespace Eigen;
	// Pre-Calculate our Inertia Tensor and Inverse.
	SystemCharacteristics.J << SystemCharacteristics.Ixx	, 0									, -SystemCharacteristics.Ixz,
							   0							, SystemCharacteristics.Iyy			, 0,
							   -SystemCharacteristics.Ixz	, 0									, SystemCharacteristics.Izz;

	SystemCharacteristics.JInverse = SystemCharacteristics.J.inverse();
}

// Called every frame
void AFlyingPawn::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// Delegate our custom physics substep ticks to occur for every tick.
	// We expect that our root component here is a mesh.
	if (PhysicsBody) {
		PhysicsBody->AddCustomPhysics(CalculateCustomPhysics);
	}

}

// Physics Substep Tick Implementation
void AFlyingPawn::SubstepTick(float DeltaTime, FBodyInstance* BodyInstance)
{
	// First update state (atmospheric and airspeed)
	SubstepStateUpdate(DeltaTime);

	// Get Forces and Moments in NEU
	FForcesAndMoments AirframeForcesAndMoments = CalculateAirframeForcesAndMoments();
	FForcesAndMoments PropulsionForcesAndMoments = CalculatePropulsionForcesAndMoments();

	FForcesAndMoments TotalForcesAndMoments = AirframeForcesAndMoments + PropulsionForcesAndMoments;

	// Apply Forces and Moments into Kinematics
	ApplyKinematics(TotalForcesAndMoments.Forces, TotalForcesAndMoments.Moments, DeltaTime);
}

// Update System State during Substep
void AFlyingPawn::SubstepStateUpdate(float DeltaTime)
{
	// First update the current system state (ie. velocities etc.)
	UpdateCurrentSystemState();
	// Then update our external atmospheric conditions (wind, turbulence etc.)
	UpdateAtmosphericConditionsState(DeltaTime);
	// Now update our airspeed params based on the above
	UpdateAirspeedState();
	// FInally, update the current actuator states
	UpdateActuatorState(DeltaTime);
}

// Update the Current System State to be Used in Other Updates
void AFlyingPawn::UpdateCurrentSystemState()
{
	// Get Rbw

	// Note that UE4 uses a body coordinate system, with the forward vector out the nose, side vector out the right wing and up vector up. 
	// Standard definition of the body frame has the up vector pointing out of the belly (not up through it), so we need to rotate the default, and then switch the side of the side vector. 

	FTransform WorldT = PhysicsBody->GetUnrealWorldTransform();

	// Unreal to world frame
	FRotator Ruw = WorldT.Rotator();

	// Invert to get Rwu
	FRotator Rwu = Ruw.GetInverse();

	// Save these
	SystemState.Ruw = Ruw;
	SystemState.Rwu = Rwu;

	// Get Vb
	FVector Vb = TransformFromWorldToBody(PhysicsBody->GetUnrealWorldVelocity() / 100);  // Scale to m/s (UE4 uses cm as default unit)

	// Get Omegab
	// Note that physics body uses a different definition of positive (Left-Hand +'ve) to what we're looking for (Right-Hand +'ve), so we invert this.
	FVector Omegab = -TransformFromWorldToBody(PhysicsBody->GetUnrealWorldAngularVelocityInRadians());

	// Get Position
	// Note that we need to convert to m (default is cm).
	float CmToM = 1 / 100.0f;
	FVector Position = WorldT.GetTranslation();
	Position *= CmToM;

	// Assign Outputs
	SystemState.Vb = Vb;
	SystemState.Omegab = Omegab;
	SystemState.Position = Position;
}

// Update the Atmospheric Conditions to be used in this substep
void AFlyingPawn::UpdateAtmosphericConditionsState(float DeltaTime)
{

	FVector Vw = FVector(0.0f);

	// If we've defined a weather class, then grab the wind value from the weather to sync this.
	if (UDSWeatherActor)
	{
		FFloatProperty* WindIntensityProperty = FindFProperty<FFloatProperty>(UDSWeatherActor->GetClass(), FName(*WeatherSetup.UDSWeatherWindIntensityPropertyName));

		// If we can pull the wind intensity, then get that from the class.
		if (WindIntensityProperty)
		{
			float WindIntensity = WindIntensityProperty->GetPropertyValue_InContainer(UDSWeatherActor) * WeatherSetup.UDSWindIntensityScalar;

			// Initially assume that the direction is North (ie. 0 rad).
			float WindDirectionRads = FMath::DegreesToRadians(0);

			FFloatProperty* WindDirectionProperty = FindFProperty<FFloatProperty>(UDSWeatherActor->GetClass(), FName(*WeatherSetup.UDSWeatherWindDirectionPropertyName));
			// But if our direction property exists, then use this instead.
			if (WindDirectionProperty)
			{
				float WindDirectionDegs = WindDirectionProperty->GetPropertyValue_InContainer(UDSWeatherActor);
				WindDirectionRads = FMath::DegreesToRadians(WindDirectionDegs);
			}

			// Use intensity and direction to get our wind vector
			Vw.X = WindIntensity * cos(WindDirectionRads);
			Vw.Y = WindIntensity * sin(WindDirectionRads);

			// Ensure we remove any numerical errors we might have with this vector
			Vw = SkyPhysHelpers::RemoveNumericalErrors(Vw);

		}

	}

	// Check if there is an assigned turbulence model
	FVector Vtw(0.0f);

	if (bEnableTurbulenceModel && TurbulenceModel)
	{
		// If there is, and we have enabled turbulence, then calculate and add our turbulence.

		// Turbulence is calculated in the body frame
		FVector Vtb = TurbulenceModel->GetTurbulenceBodyFrame(DeltaTime, AirspeedState.Va, SystemState.Position.Z, AtmosphericConditionsState.VwLowAltitude.Size());
		// Convert to world frame before we add it to the global wind vector (which is also in the world frame)
		Vtw = TransformFromBodyToWorld(Vtb);
		// Ensure we remove any numerical errors we might have with this vector
		Vtw = SkyPhysHelpers::RemoveNumericalErrors(Vtw);
	}

	AtmosphericConditionsState.rho = 1.225f; // Could use temperature from weather class, as well as altitude and atmospheric model to get density.
	AtmosphericConditionsState.VwLowAltitude = Vw; // Our low altitude wind speed is our atmospheric wind value
	AtmosphericConditionsState.Vw = Vw + Vtw; // Also set our world wind velocity to this, which we *might* augment with turbulence (if enabled etc.)
}

// Update the airspeed params to be used in this substep
void AFlyingPawn::UpdateAirspeedState()
{
	// Get wind speed in the body frame
	FVector Vwb = TransformFromWorldToBody(AtmosphericConditionsState.Vw);
	// Then get the component velocity (in the body frame)
	FVector Vb = SystemState.Vb;
	// Now calculate the airspeed in the body frame
	FVector Vab = Vb - Vwb;

	// Just make sure our airspeed vector makes sense.
	Vab = SkyPhysHelpers::RemoveNumericalErrors(Vab);

	// Now calculate our airspeed params
	float Va = Vab.Size();
	// Assume alpha and beta are 0 if Va is close to 0 (as they are then technically undefined).
	float alpha = 0.0f;
	float beta = 0.0f;
	if (!FMath::IsNearlyZero(Va) && !FMath::IsNaN(Va))
	{
		// If Va is not nearly zero, then we can get an alpha and beta.
		alpha = atan2(Vab.Z, Vab.X);
		beta = asin(Vab.Y / Va);
	}

	// Now assign to outputs
	AirspeedState.Vwb = Vwb;
	AirspeedState.Vab = Vab;
	AirspeedState.Va = Vab.Size();
	AirspeedState.alpha = alpha;
	AirspeedState.beta = beta;
}

// Apply Kinematics in the World Frame, with Forces and Moments in the Body Frame (NED)
void AFlyingPawn::ApplyKinematics(FVector Forces, FVector Moments, float DeltaTime) {

	// ************************* Linear Kinematics ************************* //

	// Scaling Factors
	float MToCM = 100.0f;

	// Calculate Linear Differential Velocity

	float mass = SystemCharacteristics.Mass;

	// Rearrange F = m dV/dt to dV = dt*(F/m); and scale to cm/s.
	FVector dVb = DeltaTime * MToCM * Forces / mass;  // Velocity differential, scaled to cm/s

	// Remove numerical errors like small accumulation errors, NaNs etc.
	dVb = SkyPhysHelpers::RemoveNumericalErrors(dVb);

	// Apply Linear Differential Velocity in the world frame
	PhysicsBody->SetLinearVelocity(TransformFromBodyToWorld(dVb), true);

	// ********************************************************************* //

	// ************************* Angular Kinematics ************************ //

	FVector dOmegab(0.0f);

	// The following block will require usage of the Eigen package to do some matrix stuff, so just context block this out of the main scope.
	{
		using namespace Eigen;
		// Calculate Angular Differential Velocity

		// Moments in the body frame
		Vector3d momentsb(Moments.X, Moments.Y, Moments.Z);
		// Angular velocity in the body frame
		Vector3d omegab(SystemState.Omegab.X, SystemState.Omegab.Y, SystemState.Omegab.Z);

		// dOmegab/dt = JInverse * (Moments - Omegab x ( J * Omegab ) -> with everything in the body frame
		// dOmegab = JInverse * (Moments - Omegab x ( J * Omegab ) * dt
		Vector3d domegab = (SystemCharacteristics.JInverse * (momentsb - omegab.cross(SystemCharacteristics.J * omegab))) * DeltaTime;

		// Convert to UE4 type
		dOmegab = FVector(domegab(0), domegab(1), domegab(2));
	}

	// Remove numerical errors like small accumulation errors, NaNs etc.
	dOmegab = SkyPhysHelpers::RemoveNumericalErrors(dOmegab);

	// Apply Angular Differential Velocity in the world frame
	PhysicsBody->SetAngularVelocityInRadians(TransformFromBodyToWorld(-dOmegab), true); // Note: We have to take negative dOmegab due to how PhysicsBody has defined what positive rotation means (which is inconsistent with UE4 def... oh well)

	// ********************************************************************* //

	// ******************************* Debug ******************************* //
	
	//FVector Location = RootComponent->GetComponentLocation();
	//float scaling = 10.0f;

	//// Forces

	//FVector BodyForces = (dVb * mass) / (DeltaTime * MToCM);
	//
	//// Draw Debug Arrows

	//DrawDebugDirectionalArrow(GetWorld(), Location, Location + TransformFromBodyToWorld(FVector(BodyForces.X, 0, 0)) * scaling, 50.0f, FColor(255, 0, 0), false, 0, 0, 5.0f);
	//DrawDebugDirectionalArrow(GetWorld(), Location, Location + TransformFromBodyToWorld(FVector(0, BodyForces.Y, 0)) * scaling, 50.0f, FColor(0, 255, 0), false, 0, 0, 5.0f);
	//DrawDebugDirectionalArrow(GetWorld(), Location, Location + TransformFromBodyToWorld(FVector(0, 0, BodyForces.Z)) * scaling, 50.0f, FColor(0, 0, 255), false, 0, 0, 5.0f);

	//// Moments

	//FVector BodyAngularAcceleration = dOmegab / DeltaTime;

	//// Draw Debug Arrows

	//DrawDebugDirectionalArrow(GetWorld(), Location, Location + TransformFromBodyToWorld(FVector(BodyAngularAcceleration.X, 0, 0)) * scaling*10, 200.0f, FColor(255, 0, 255), false, 0, 0, 5.0f);
	//DrawDebugDirectionalArrow(GetWorld(), Location, Location + TransformFromBodyToWorld(FVector(0, BodyAngularAcceleration.Y, 0)) * scaling*10, 200.0f, FColor(255, 255, 0), false, 0, 0, 5.0f);
	//DrawDebugDirectionalArrow(GetWorld(), Location, Location + TransformFromBodyToWorld(FVector(0, 0, BodyAngularAcceleration.Z)) * scaling*10, 200.0f, FColor(0, 255, 255), false, 0, 0, 5.0f);

	// ********************************************************************* //
}

// Calculate Forces and Moments for this Airframe
FForcesAndMoments AFlyingPawn::CalculateAirframeForcesAndMoments()
{
	// First Update our Aerodynamic Calculation Parameters
	UpdateAerodynamicCalculationParameters();

	// Then calculate forces and moments
	FVector Forces = CalculateAirframeAerodynamicForces();
	FVector Moments = CalculateAirframeAerodynamicMoments();

	return FForcesAndMoments(Forces, Moments);
}

void AFlyingPawn::UpdateAerodynamicCalculationParameters() 
{
	// Geometric Params
	const float b = GeometricCharacteristics.b;
	const float c = GeometricCharacteristics.c;

	// Airspeed Params
	float Va = AirspeedState.Va;

	// Atmospheric Conditions
	float rho = AtmosphericConditionsState.rho;

	// Calculated Params
	float bOver2Va = 0.0f;
	float cOver2Va = 0.0f;

	// If Va isn't 0, we update the above params to their actual values
	if (!FMath::IsNearlyZero(Va))
	{
		bOver2Va = b / (2 * Va);
		cOver2Va = c / (2 * Va);
	}

	// Assign outputs
	AerodynamicCalculationParameters.DynamicPressure = 0.5 * rho * pow(Va, 2.0f);
	AerodynamicCalculationParameters.bOver2Va = bOver2Va;
	AerodynamicCalculationParameters.cOver2Va = cOver2Va;
}

// Calculate the Aerodynamic Forces on the Airframe
FVector AFlyingPawn::CalculateAirframeAerodynamicForces() const
{
	// ********************* Set Up Constant Parameters ********************** //

	// Aerodynamic Coefficients

	// CD
	const float CD0 = AerodynamicCoefficients.CD.CD0;
	const float CDAlpha = AerodynamicCoefficients.CD.CDAlpha;
	const float CDAlpha2 = AerodynamicCoefficients.CD.CDAlpha2;
	const float CDq = AerodynamicCoefficients.CD.CDq;
	const float CDBeta = AerodynamicCoefficients.CD.CDBeta;
	const float CDBeta2 = AerodynamicCoefficients.CD.CDBeta2;

	// CY
	const float CY0 = AerodynamicCoefficients.CY.CY0;
	const float CYBeta = AerodynamicCoefficients.CY.CYBeta;
	const float CYp = AerodynamicCoefficients.CY.CYp;
	const float CYr = AerodynamicCoefficients.CY.CYr;

	// CL
	const float CL0 = AerodynamicCoefficients.CL.CL0;
	const float CLAlpha = AerodynamicCoefficients.CL.CLAlpha;
	const float CLq = AerodynamicCoefficients.CL.CLq;

	// *********************************************************************** //

	// ****************** Calculate Aerodynamic Coefficients ***************** //

	// Common Values

	// Velocity
	float p = SystemState.Omegab.X;
	float q = SystemState.Omegab.Y;
	float r = SystemState.Omegab.Z;

	// Airspeed Params
	float alpha = AirspeedState.alpha;
	float beta = AirspeedState.beta;

	// Calculated Params
	FVector AerodynamicMultiple = AerodynamicCalculationParameters.DynamicPressure * GeometricCharacteristics.A;
	float bOver2Va = AerodynamicCalculationParameters.bOver2Va;
	float cOver2Va = AerodynamicCalculationParameters.cOver2Va;

	// We isolate our "alpha" parts of our coefficients as these will be impacted by our stall model (if enabled).
	float CDALPHA = CD0 + CDAlpha * alpha + CDAlpha2 * pow(alpha, 2.0f);
	float CLALPHA = CL0 + CLAlpha * alpha;

	// Stall Model (if enabled) here.

	FVector2D CDALPHACLALPHA = CalculateAdjustedCDAlphaCLAlphaForStall(CDALPHA, CLALPHA);

	CDALPHA = CDALPHACLALPHA.X;
	CLALPHA = CDALPHACLALPHA.Y;

	// Now add the rest of the coefficient impacts
	float CDCalc = CDALPHA + CDq * cOver2Va * q + CDBeta * beta + CDBeta2 * pow(beta, 2.0f) + CalculateAdditionalCDParameters();
	float CLCalc = CLALPHA + CLq * cOver2Va * q + CalculateAdditionalCLParameters();

	// Side force
	float CY = CY0 + CYBeta * beta + CYp * bOver2Va * p + CYr * bOver2Va * r + CalculateAdditionalCYParameters();

	// We need to rotate our aerodynamic forces, which are in the wind frame, to the body frame. We do this with a rotation by alpha and beta.
	FRotator Rwb(FMath::RadiansToDegrees(alpha), FMath::RadiansToDegrees(beta), 0.0f);

	// *********************************************************************** //

	// We know that CD and CL will be in the negative direction in the wind frame.
	// Then rotate this into the body frame.
	FVector Fxyz = Rwb.RotateVector(FVector(-CDCalc, CY, -CLCalc)) * AerodynamicMultiple;

	return Fxyz;
}

// Calculate the Aerodynamic Moments on the Airframe
FVector AFlyingPawn::CalculateAirframeAerodynamicMoments() const
{
	// ********************* Set Up Constant Parameters ********************** //

	// Geometric Params
	const float b = GeometricCharacteristics.b;
	const float c = GeometricCharacteristics.c;

	// Aerodynamic Coefficients

	// Cl
	const float CI0 = AerodynamicCoefficients.CI.CI0;
	const float CIBeta = AerodynamicCoefficients.CI.CIBeta;
	const float CIp = AerodynamicCoefficients.CI.CIp;
	const float CIr = AerodynamicCoefficients.CI.CIr;

	// Cm
	const float Cm0 = AerodynamicCoefficients.Cm.Cm0;
	const float CmAlpha = AerodynamicCoefficients.Cm.CmAlpha;
	const float Cmq = AerodynamicCoefficients.Cm.Cmq;

	// Cn
	const float Cn0 = AerodynamicCoefficients.Cn.Cn0;
	const float CnBeta = AerodynamicCoefficients.Cn.CnBeta;
	const float Cnp = AerodynamicCoefficients.Cn.Cnp;
	const float Cnr = AerodynamicCoefficients.Cn.Cnr;

	// *********************************************************************** //

	// ****************** Calculate Aerodynamic Coefficients ***************** //

	// Common Values

	// Velocity
	float p = SystemState.Omegab.X;
	float q = SystemState.Omegab.Y;
	float r = SystemState.Omegab.Z;

	// Airspeed Params
	float alpha = AirspeedState.alpha;
	float beta = AirspeedState.beta;

	// Calculated Params
	FVector AerodynamicMultiple = AerodynamicCalculationParameters.DynamicPressure * GeometricCharacteristics.A;
	float bOver2Va = AerodynamicCalculationParameters.bOver2Va;
	float cOver2Va = AerodynamicCalculationParameters.cOver2Va;

	// Calculate

	float CmALPHA = Cm0 + CmAlpha * alpha;

	// Flat plate stall for pitching moment
	CmALPHA = CalculateAdjustedCmAlphaForStall(CmALPHA);

	float CI = CI0 + CIBeta * beta + CIp * bOver2Va * p + CIr * bOver2Va * r + CalculateAdditionalClParameters();
	float Cm = CmALPHA + Cmq * cOver2Va * q + CalculateAdditionalCmParameters();
	float Cn = Cn0 + CnBeta * beta + Cnp * bOver2Va * p + Cnr * bOver2Va * r + CalculateAdditionalCnParameters();

	// *********************************************************************** //

	FVector Mxyz(CI * b, Cm * c, Cn * b);

	Mxyz *= AerodynamicMultiple;

	return Mxyz;
}

FForcesAndMoments AFlyingPawn::CalculatePropulsionForcesAndMoments()
{
	// Note that our propulsors are only updated every primary tick, so this will only be an approximation. It should be quite close, but it's quite difficult in UE4 to simulate physics on 
	// subcomponents whilst still getting constraints etc. to work as intended.

	FForcesAndMoments CumulativePropulsorForcesAndMomentsAtCG;

	float Rho = AtmosphericConditionsState.rho;
	FVector Vw = AtmosphericConditionsState.Vw;
	FVector SystemOmega = TransformFromBodyToWorld(SystemState.Omegab);

	for (UPropulsionStaticMeshComponent* Propulsor : Propulsors) 
	{
		// First, get all the forces and moments at the origin of the propulsor (in the world frame).
		FForcesAndMoments PropulsorForcesAndMoments = Propulsor->GetForcesAndMoments(Rho, Vw, SystemOmega);

		// Then we need to calculate the moments due to the propulsor forces acting at a distance to our CG.

		// So first get the relative location
		// Note that we are only using the locations updated in the primary tick for this, so don't use the physics body.
		FVector CGLocation = RootComponent->GetComponentLocation();
		FVector PropulsorLocation = Propulsor->GetComponentLocation(); //Propulsor->GetBodyInstance()->GetUnrealWorldTransform().GetLocation();

		// Divide by 100 to get to m from cm (because when we do moment calc next, we expect SI units).
		// We take the negative of this (which differs from the standard moment cross product direction), because the unreal world frame uses a LHR convention (instead of the standard RHR).
		FVector r = -(PropulsorLocation - CGLocation)/100.0f;

		// Then calculate the moments due to these forces using the cross product (r x F)
		FVector AdditionalMoments = FVector::CrossProduct(r, PropulsorForcesAndMoments.Forces);

		// Finally add these in to the PropulsorForcesAndMoments.
		PropulsorForcesAndMoments.Moments += AdditionalMoments;

		// Now we need to convert from the world to the body frame
		PropulsorForcesAndMoments.Forces = TransformFromWorldToBody(PropulsorForcesAndMoments.Forces);
		PropulsorForcesAndMoments.Moments = TransformFromWorldToBody(PropulsorForcesAndMoments.Moments);

		// And now we can add all of these forces and moments to our cumulative sum.
		CumulativePropulsorForcesAndMomentsAtCG += PropulsorForcesAndMoments;
	}

	return CumulativePropulsorForcesAndMomentsAtCG;
}

FVector AFlyingPawn::TransformFromWorldToBody(FVector WorldVector)
{
	// To get from the world to the body, we first go from world to unreal
	FVector UnrealFrame = SystemState.Rwu.RotateVector(WorldVector);
	// And then from unreal to body (by just flipping around the Z axis)
	return FVector(UnrealFrame.X, UnrealFrame.Y, -UnrealFrame.Z);
}

FVector AFlyingPawn::TransformFromBodyToWorld(FVector BodyVector)
{
	// To get from the body frame to the world frame, we first flip around the body vector so that it's in the unreal frame
	FVector UnrealFrame(BodyVector.X, BodyVector.Y, -BodyVector.Z);
	// Then we rotate from unreal to world
	return SystemState.Ruw.RotateVector(UnrealFrame);
}