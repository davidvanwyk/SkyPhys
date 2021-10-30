// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Pawns/FlyingPawn.h"
#include "MultiRotorPawn.generated.h"

// ***************** Declarations ***************** //

// Forward declaration

class UPropulsionPhysicsConstraint;
class UPropellerPropulsionStaticMeshComponent;
class UFirstOrderActuator;

// ************************************************ //

// Editor Declarations

USTRUCT()
struct FMultiRotorActuatorAnimationParameters
{
	GENERATED_BODY()

	// Magnitude Scalars from the physical parameters to the animation parameters (not including unit conversions).
	UPROPERTY(EditAnywhere, Meta = (Tooltip = "Propeller Speed scalar. Scales the animation angular speed by this amount relative to the physical anglular speed being simulated. Be careful about aliasing."))
	float PropellerSpeedScalar = 5;
};

UENUM()
enum class EMultiRotorConfiguration : int8
{
	Plus	= 1			UMETA(DisplayName = "+"),
	Cross	= 2			UMETA(DisplayName = "x")
};

//  **************** State Structs **************** //

// The current actuator commands
struct FMultiRotorCommandState
{
	float PitchCommand = 0.0f; // Pitch command (-1 -> 1)
	float RollCommand = 0.0f; // Roll command (-1 -> 1)
	float YawCommand = 0.0f; // Yaw command (-1 -> 1)
	float ThrustCommand = 0.0f; // Thrust command (0 -> 1)
};

// ************************************************ //

UCLASS(Abstract, NotBlueprintable)
class SKYPHYS_API AMultiRotorPawn : public AFlyingPawn
{
	GENERATED_BODY()
	
public:

	// Sets default values for this pawn's properties
	AMultiRotorPawn();

protected:

	// This is the offset that will be applied to the thrust command to cancel out the weight of the aircraft nominally under steady state conditions.
	// It is recommended to set this value to a point where a descent at the required maximum acceleration is feasible at 0 thrust command. 
	// Essentially with 0 thrust commanded (ie. a throttle/PLA level of 0), this is the baseline value that will still be applied.
	// This should be between 0 -> 1.
	UPROPERTY(EditAnywhere, Category = "General Setup")
	float ThrustCommandOffset = 0.0f;

	// This is the multirotor configuration type, which will either be a "x" type (ie. no propulsion element aligned with body axes) or a "+" type (where there is alignment).
	UPROPERTY(EditAnywhere, Category = "General Setup")
	EMultiRotorConfiguration MultiRotorConfiguration = EMultiRotorConfiguration::Cross;

	UPROPERTY(EditAnywhere, Category = "Animation")
	FMultiRotorActuatorAnimationParameters ActuatorAnimationParameters;

	// Called to update the current actuator state
	virtual void UpdateActuatorState(float DeltaTime) override;

	// Input Calculations

	// Calculate Elevator Angle (expected Value of -1 -> 1)
	virtual void ApplyPitchCommand(float Value) override;
	// Calculate Aileron Angle (expected Value of -1 -> 1)
	virtual void ApplyRollCommand(float Value) override;
	// Calculate Rudder Angle (expected Value of -1 -> 1)
	virtual void ApplyYawCommand(float Value) override;
	// Apply the thrust command (expected Value of 0 -> 1)
	virtual void ApplyThrustCommand(float Value) override;

	// Parameters
	FMultiRotorCommandState MultiRotorCommandState;

};
