// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Pawns/FixedWing/FixedWingPawn.h"
#include "FlyingWingPawn.generated.h"

// Forward declaration
class UCtrlSurfaceStaticMeshComponent;
class USecondOrderActuator;

// States of our actuators for animation purposes (hence degrees, and not radians).
// These are NON-FUNCTIONAL and only used for graphical depictions.
USTRUCT(BlueprintType)
struct FFlyingWingActuatorAnimationState
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	float PropellerSpeed = 0.0f; // Scaled Propeller speed (deg/s)

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	float LeftElevonAngle = 0.0f; // Scaled Left Elevon angle (deg)
	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	float RightElevonAngle = 0.0f; // Scaled Right Elevon angle (deg)
};

USTRUCT()
struct FFlyingWingActuatorAnimationParameters
{
	GENERATED_BODY()
	
	// Magnitude Scalars from the physical parameters to the animation parameters (not including unit conversions).
	UPROPERTY(EditAnywhere, Meta = (Tooltip = "Propeller Speed scalar. Scales the animation angular speed by this amount relative to the physical anglular speed being simulated. Be careful about aliasing."))
	float PropellerSpeedScalar = 5;

	UPROPERTY(EditAnywhere, Meta = (Tooltip = "Elevon scalar parameter. Scales the animation angle by this amount relative to the physical angle being simulated."))
	float ElevonAngleScalar = 1.0f;
};

UCLASS(Abstract, Blueprintable)
class SKYPHYS_API AFlyingWingPawn : public AFixedWingPawn
{
	GENERATED_BODY()
	
public:
	AFlyingWingPawn();

private:
	// Update the Elevon Angles based on de and da updates
	void UpdateElevonAngles(float DeltaTime);

protected:
	// Components
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	UCtrlSurfaceStaticMeshComponent* LeftElevonMesh; // Expect Flying Wings to have Elevons
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	USecondOrderActuator* LeftElevonServo;

	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	UCtrlSurfaceStaticMeshComponent* RightElevonMesh; // Expect Flying Wings to have Elevons
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	USecondOrderActuator* RightElevonServo;

	UPROPERTY(BlueprintReadOnly, Category = "Animation")
	FFlyingWingActuatorAnimationState ActuatorAnimationState;

	UPROPERTY(EditAnywhere, Category = "Animation")
	FFlyingWingActuatorAnimationParameters ActuatorAnimationParameters;

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
