// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Pawns/FixedWing/FixedWingPawn.h"
#include "StandardFixedWingPawn.generated.h"

// Forward declarations
class UCtrlSurfaceStaticMeshComponent;
class USecondOrderActuator;

// States of our actuators for animation purposes (hence degrees, and not radians).
// These are NON-FUNCTIONAL and only used for graphical depictions.
USTRUCT(BlueprintType)
struct FStandardFixedWingActuatorAnimationState
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	float PropellerSpeed = 0.0f; // Scaled Motor speed (deg/s)

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	float LeftAileronAngle = 0.0f; // Scaled Left Airleron angle (deg)
	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	float RightAileronAngle = 0.0f; // Scaled Right Aileron angle (deg)

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	float ElevatorAngle = 0.0f; // Scaled Elevator angle (deg)

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	float RudderAngle = 0.0f; // Scaled Rudder angle (deg)
};

USTRUCT()
struct FStandardFixedWingActuatorAnimationParameters
{
	GENERATED_BODY()

	// Magnitude Scalars from the physical parameters to the animation parameters (not including unit conversions).
	UPROPERTY(EditAnywhere, Meta = (Tooltip = "Propeller Speed scalar. Scales the animation angular speed by this amount relative to the physical anglular speed being simulated. Be careful about aliasing."))
	float PropellerSpeedScalar = 590.0f;

	UPROPERTY(EditAnywhere, Meta = (Tooltip = "Aileron scalar parameter. Scales the animation angle by this amount relative to the physical angle being simulated."))
	float AileronAngleScalar = 1.0f;

	UPROPERTY(EditAnywhere, Meta = (Tooltip = "Elevator scalar parameter. Scales the animation angle by this amount relative to the physical angle being simulated."))
	float ElevatorAngleScalar = 1.0f;

	UPROPERTY(EditAnywhere, Meta = (Tooltip = "Rudder scalar parameter. Scales the animation angle by this amount relative to the physical angle being simulated."))
	float RudderAngleScalar = 1.0f;
};

UCLASS(Abstract, Blueprintable)
class SKYPHYS_API AStandardFixedWingPawn : public AFixedWingPawn
{
	GENERATED_BODY()
	
public:
	AStandardFixedWingPawn();

private:
	void UpdateAileronAngles(float DeltaTime);

protected:
	// Components
	// Expect Standard Config Fixed Wings to have 2 Ailerons (one on each wing)
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	UCtrlSurfaceStaticMeshComponent* LeftAileronMesh;
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	USecondOrderActuator* LeftAileronServo;

	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	UCtrlSurfaceStaticMeshComponent* RightAileronMesh;
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	USecondOrderActuator* RightAileronServo;

	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	UCtrlSurfaceStaticMeshComponent* ElevatorMesh; // Expect Standard Fixed Wings to have Elevators (on the tail)
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	USecondOrderActuator* ElevatorServo;

	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	UCtrlSurfaceStaticMeshComponent* RudderMesh; // Expect Standard Fixed Wings to have Rudders (on the tail)
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	USecondOrderActuator* RudderServo;

	UPROPERTY(BlueprintReadOnly, Category = "Animation")
	FStandardFixedWingActuatorAnimationState ActuatorAnimationState;

	UPROPERTY(EditAnywhere, Category = "Animation")
	FStandardFixedWingActuatorAnimationParameters ActuatorAnimationParameters;

	// Called to update the current actuator state
	virtual void UpdateActuatorState(float DeltaTime) override;

	// Called to update the actuator animation state
	virtual void UpdateActuatorAnimationState() override;

	// Calculate Elevator Angle (expected Value of -1 -> 1)
	virtual void ApplyPitchCommand(float Value) override;
	// Calculate Aileron Angle (expected Value of -1 -> 1)
	virtual void ApplyRollCommand(float Value) override;
	// Calculate Rudder Angle (expected Value of -1 -> 1)
	virtual void ApplyYawCommand(float Value) override;
};
