// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Eigen/Eigen"
#include "GameFramework/Pawn.h"
#include "Common/Types.h"

#include "FlyingPawn.generated.h"

// Forward Declares
class UTurbulenceModel;
class UPropulsionStaticMeshComponent;
class UActuatorModel;

// ################# Aerodynamics ################# //

// Force Coefficients

// Lift Force Aerodynamic Coefficients
USTRUCT()
struct FCL
{
	GENERATED_BODY()

	// Stability Derivatives
	UPROPERTY(EditAnywhere, Meta = (DisplayName = "CL 0", Tooltip = "CL 0"))
	float CL0 = 0;
	UPROPERTY(EditAnywhere, Meta = (DisplayName = "CL alpha", Tooltip = "CL alpha"))
	float CLAlpha = 0;
	UPROPERTY(EditAnywhere, Meta = (DisplayName = "CL q", Tooltip = "CL q"))
	float CLq = 0;
};

// Drag Force Aerodynamic Coefficients
USTRUCT()
struct FCD
{
	GENERATED_BODY()

	// Stability Derivatives
	UPROPERTY(EditAnywhere, Meta = (DisplayName = "CD 0", Tooltip = "CD 0"))
	float CD0 = 0;
	UPROPERTY(EditAnywhere, Meta = (DisplayName = "CD alpha", Tooltip = "CD alpha"))
	float CDAlpha = 0;
	UPROPERTY(EditAnywhere, Meta = (DisplayName = "CD alpha^2", Tooltip = "CD alpha^2"))
	float CDAlpha2 = 0;
	UPROPERTY(EditAnywhere, Meta = (DisplayName = "CD q", Tooltip = "CD q"))
	float CDq = 0;
	UPROPERTY(EditAnywhere, Meta = (DisplayName = "CD beta", Tooltip = "CD beta"))
	float CDBeta = 0;
	UPROPERTY(EditAnywhere, Meta = (DisplayName = "CD beta^2", Tooltip = "CD beta^2"))
	float CDBeta2 = 0;
};

// Side Force Aerodynamic Coefficients
USTRUCT()
struct FCY
{
	GENERATED_BODY()

	// Stability Derivatives
	UPROPERTY(EditAnywhere, Meta = (DisplayName = "CY 0", Tooltip = "CY 0"))
	float CY0 = 0;
	UPROPERTY(EditAnywhere, Meta = (DisplayName = "CY beta", Tooltip = "CY beta"))
	float CYBeta = 0;
	UPROPERTY(EditAnywhere, Meta = (DisplayName = "CY p", Tooltip = "CY p"))
	float CYp = 0;
	UPROPERTY(EditAnywhere, Meta = (DisplayName = "CY r", Tooltip = "CY r"))
	float CYr = 0;
};

// Moment Coefficients

// Roll Moment Coefficients (Actually Cl, but use "CI" for unique name purposes)
USTRUCT()
struct FCI
{
	GENERATED_BODY()

	// Stability Derivatives
	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Cl 0", Tooltip = "Cl 0"))
	float CI0 = 0;
	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Cl beta", Tooltip = "Cn beta"))
	float CIBeta = 0;
	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Cl p", Tooltip = "Cl p"))
	float CIp = 0;
	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Cl r", Tooltip = "Cl r"))
	float CIr = 0;
};

// Pitch Moment Coefficients
USTRUCT()
struct FCm
{
	GENERATED_BODY()

	// Stability Derivatives
	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Cm 0", Tooltip = "Cm 0"))
	float Cm0 = 0;
	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Cm alpha", Tooltip = "Cm p"))
	float CmAlpha = 0;
	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Cm q", Tooltip = "Cm q"))
	float Cmq = 0;
};

// Yaw Moment Coefficients
USTRUCT()
struct FCn
{
	GENERATED_BODY()

	// Stability Derivatives

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Cn 0", Tooltip = "Cn 0"))
	float Cn0 = 0;
	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Cn beta", Tooltip = "Cn beta"))
	float CnBeta = 0;
	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Cn p", Tooltip = "Cn p"))
	float Cnp = 0;
	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Cn r", Tooltip = "Cn r"))
	float Cnr = 0;
};

// Aerodynamic Coefficients
USTRUCT()
struct FAerodynamicCoefficients
{
	GENERATED_BODY()

	// Force Coefficients

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "CL", Tooltip = "Lift Force Coefficients"))
	FCL CL;

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "CD", Tooltip = "Drag Force Coefficients"))
	FCD CD;

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "CY", Tooltip = "Side Force Coefficients"))
	FCY CY;

	// Moment Coefficients

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Cl", Tooltip = "Roll Moment Coefficients"))
	FCI CI;

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Cm", Tooltip = "Pitch Moment Coefficients"))
	FCm Cm;

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Cn", Tooltip = "Yaw Moment Coefficients"))
	FCn Cn;
};

// ################################################ //

// ############ System Characteristics ############ //

USTRUCT()
struct FSystemCharacteristics
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Mass (kg)"))
	float Mass;
	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Ixx (kg.m^2)"))
	float Ixx;
	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Iyy (kg.m^2)"))
	float Iyy;
	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Izz (kg.m^2)"))
	float Izz;

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Ixz/Izx (kg.m^2)"))
	float Ixz; // Assumed same as Izx

	// We will pre-calculate J and JInverse as these aren't going to change during runtime.
	Eigen::Matrix3d J;
	Eigen::Matrix3d JInverse;
};

// ################################################ //

// ########### Geometric Characteristics ########## //

USTRUCT()
struct FGeometricCharacteristics
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "b (m)", Tooltip = "Wing Span, b (m)"))
	float b = 0;

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "c (m)", Tooltip = "Mean Aerodynamic Chord, c (m)"))
	float c = 0;

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "A (m^2)", Tooltip = "Reference Areas (m^2). Wing Area, S, for a fixed wing."))
	FVector A = FVector(0);
};

// ################################################ //

// ################ Weather Setup ################# //

USTRUCT()
struct FWeatherSetup
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere)
	TSubclassOf<class AActor> UDSWeatherClassType;

	UPROPERTY(EditAnywhere)
	FString UDSWeatherWindIntensityPropertyName;

	UPROPERTY(EditAnywhere)
	FString UDSWeatherWindDirectionPropertyName;

	// UDS Wind Intensity -> Wind Speed (m/s) scalar
	UPROPERTY(EditAnywhere)
	float UDSWindIntensityScalar = 1.0f;
};

// ################################################ //

// State Structs

struct FAirspeedState
{
	FVector Vwb = FVector(0.0f); // Wind speed (m/s) in the body frame
	FVector Vab = FVector(0.0f); // Airspeed (m/s) in the body frame (ie. Vb - Vwb)

	float Va = 0.0f; // Airspeed (m/s)
	float alpha = 0.0f; // Angle of attack (rad)
	float beta = 0.0f; // Sideslip angle (rad)
};

struct FAtmosphericConditionsState
{
	FVector VwLowAltitude = FVector(0.0f); // Low altitude wind speed (m/s) in world frame
	FVector Vw = FVector(0.0f); // Wind speed (m/s) in world frame
	float rho = 1.225; // Air density (kg/m^3) at current altitude
};

struct FSystemState
{
	FVector Vb = FVector(0.0f); // (u, v, w)
	FVector Omegab = FVector(0.0f); // (p, q, r)
	FVector Position = FVector(0.0f); // (N, E, U)
	
	// Rotations
	FRotator Ruw = FRotator(); // Rotator from unreal to world frame
	FRotator Rwu = FRotator(); // Rotator from world to unreal frame
};

// Calculation Structs

struct FAerodynamicCalculationParameters
{
	float DynamicPressure = 0.0f; // Current dynamic pressure of the aircraft (0.5 * rho * Va^2)
	float cOver2Va = 0.0f;
	float bOver2Va = 0.0f;
};

// This class is abstract and is intended to serve as a base, but should not be used directly.
UCLASS(Abstract, NotBlueprintable)
class SKYPHYS_API AFlyingPawn : public APawn
{
	GENERATED_BODY()

public:
	// Sets default values for this pawn's properties
	AFlyingPawn();

	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Physics substep tick
	virtual void SubstepTick(float DeltaTime, FBodyInstance* BodyInstance);

private:
	// Components
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components", meta = (AllowPrivateAccess = "true"))
	UStaticMeshComponent* AirframeMesh;

	// Methods

	// Find and assign the UDS Weather Actor (if it exists).
	void SetupWeather();

	// Pre-calculate system characteristics
	void PreCalculateSystemCharacteristics();

	// Update our system state
	// This gets called in SubstepStateUpdate()
	void UpdateCurrentSystemState();

	// Update our wind speed (in the world frame), and our density. As well as any other atmospheric parameters.
	// This gets called in SubstepStateUpdate()
	void UpdateAtmosphericConditionsState(float DeltaTime);

	// Update airspeed parameters
	// This gets called in SubstepStateUpdate()
	void UpdateAirspeedState();

	// Update our Aerodynamic Calculation Parameters
	void UpdateAerodynamicCalculationParameters();

	// Apply Kinematics
	void ApplyKinematics(FVector Forces, FVector Moments, float DeltaTime);

	// Parameters
	FBodyInstance* PhysicsBody;
	AActor* UDSWeatherActor;

	// Custom Physics Delegator
	FCalculateCustomPhysics CalculateCustomPhysics;

protected:

	// Editor Properties

	// General System-Level Characteristics
	UPROPERTY(EditAnywhere, Category = "General Setup")
	FSystemCharacteristics SystemCharacteristics;

	UPROPERTY(EditAnywhere, Category = "Geometric Parameters")
	FGeometricCharacteristics GeometricCharacteristics;

	UPROPERTY(EditAnywhere, Category = "Aerodynamic Parameters")
	FAerodynamicCoefficients AerodynamicCoefficients; // All flying systems should have a similar set of aerodynamic coefficients/derivatives. Zero out the ones you don't need.

	// TODO: Maybe create an instanced object so one can select between using this or a directional wind source.
	// UDS Weather Actor
	UPROPERTY(EditDefaultsOnly, Category = "General Setup|Weather")
	FWeatherSetup WeatherSetup;

	// Turbulence Model Selection and Setup
	UPROPERTY(EditAnywhere, Category = "General Setup|Weather|Turbulence", Meta = (DisplayName = "Enable Turbulence Model", Tooltip = "Whether to Enable the Turbulence Model"))
	bool bEnableTurbulenceModel = false;
	
	UPROPERTY(EditAnywhere, Instanced, Category = "General Setup|Weather|Turbulence", Meta = (Tooltip = "The Turbulence Model to be Used", EditCondition = "bEnableTurbulenceModel"))
	UTurbulenceModel* TurbulenceModel;

	// Methods

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// Apply any state updates necessary prior to forces and moments calculation.
	virtual void SubstepStateUpdate(float DeltaTime);

	// Update the current actuator state (based on the current actuator command, which will be calculated/applied seperately)
	// This gets called in SubstepStateUpdate()
	virtual void UpdateActuatorState(float DeltaTime) {};

	// Update the current actuator animation state (based on the current actuator state)
	virtual void UpdateActuatorAnimationState() PURE_VIRTUAL(AFlyingPawn::UpdateActuatorAnimationState, );

	// Calculate the Airframe Forces and Moments (excludes all propulsion elements)
	// 
	// @return The forces and moments generated by the airframe, to be applied at the CoG, expressed in the world frame.
	virtual FForcesAndMoments CalculateAirframeForcesAndMoments();

	// ######### Airframe Aerodynamic Forces ########## //

	// Calculate our airframe aerodynamic forces in the body frame
	virtual FVector CalculateAirframeAerodynamicForces() const;

	// Adjust CDAlpha and CLAlpha with a stall model (if you'd like to implement one - otherwise nothing happens).
	virtual FVector2D CalculateAdjustedCDAlphaCLAlphaForStall(float CDAlpha, float CLAlpha) const { return FVector2D(CDAlpha, CLAlpha); };

	// Calculate any additional CD, CL, CY parameters to be added to the total
	// This will usually be due to things like control surfaces etc. as all body parameters are accounted for.
	virtual float CalculateAdditionalCDParameters() const { return 0.0f; };
	virtual float CalculateAdditionalCLParameters() const { return 0.0f; };
	virtual float CalculateAdditionalCYParameters() const { return 0.0f; };

	// ################################################ //

	// ######## Airframe Aerodynamic Moments ########## //

	// Calculate our airframe aerodynamic moments in the body frame
	virtual FVector CalculateAirframeAerodynamicMoments() const;

	// Adjust CmAlpha with a stall model (if you'd like to implement one - otherwise nothing happens).
	virtual float CalculateAdjustedCmAlphaForStall(float CmAlpha) const { return CmAlpha; };

	// Calculate any additional Cl, Cm, Cn parameters to be added to the total
	// This will usually be due to things like control surfaces etc. as all body parameters are accounted for.
	virtual float CalculateAdditionalClParameters() const { return 0.0f; };
	virtual float CalculateAdditionalCmParameters() const { return 0.0f; };
	virtual float CalculateAdditionalCnParameters() const { return 0.0f; };

	// ################################################ //

	// Calculate the Propulsion Forces and Moments
	// 
	// @return The forces and moments generated by all propulsion elements, to be applied at the CoG, expressed in the world frame.
	virtual FForcesAndMoments CalculatePropulsionForcesAndMoments();

	// Input Calculations
	// Apply the pitch command (expected Value of -1 -> 1)
	virtual void ApplyPitchCommand(float Value) {};
	// Apply the roll command (expected Value of -1 -> 1)
	virtual void ApplyRollCommand(float Value) {};
	// Apply the yaw command (expected Value of -1 -> 1)
	virtual void ApplyYawCommand(float Value) {};
	// Apply the thrust command (expected Value of 0 -> 1)
	virtual void ApplyThrustCommand(float Value) {};

	// Utilities

	// Transformations
	FVector TransformFromWorldToBody(FVector VectorToTransform);
	FVector TransformFromBodyToWorld(FVector VectorToTransform);

	// Parameters

	// Components
	TInlineComponentArray<UPropulsionStaticMeshComponent*> Propulsors; // All the propulsive elements attached to the system.

	// State
	FSystemState SystemState;
	FAirspeedState AirspeedState;
	FAtmosphericConditionsState AtmosphericConditionsState;

	// Calculation State
	FAerodynamicCalculationParameters AerodynamicCalculationParameters;
};
