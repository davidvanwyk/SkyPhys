// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Pawns/FixedWing/FixedWingPawn.h"

#include "VTailPawn.generated.h"

// Forward declarations
class UCtrlSurfaceStaticMeshComponent;
class USecondOrderActuator;

// States of our actuators for animation purposes (hence degrees, and not radians).
// These are NON-FUNCTIONAL and only used for graphical depictions.
USTRUCT(BlueprintType)
struct FVTailActuatorAnimationState
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	float PropellerSpeed = 0.0f; // Scaled Motor speed (deg/s)

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	float LeftAileronAngle = 0.0f; // Scaled Left Aileron angle (deg)
	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	float RightAileronAngle = 0.0f; // Scaled Right Aileron angle (deg)

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	float LeftRuddervatorAngle = 0.0f; // Scaled Left Ruddervator angle (deg)
	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	float RightRuddervatorAngle = 0.0f; // Scaled Right Ruddervator angle (deg)
};

USTRUCT()
struct FVTailActuatorAnimationParameters
{
	GENERATED_BODY()

	// Magnitude Scalars from the physical parameters to the animation parameters (not including unit conversions).
	UPROPERTY(EditAnywhere, Meta = (Tooltip = "Propeller Speed scalar. Scales the animation angular speed by this amount relative to the physical anglular speed being simulated. Be careful about aliasing."))
	float PropellerSpeedScalar = 590.0f;

	UPROPERTY(EditAnywhere, Meta = (Tooltip = "Aileron scalar parameter. Scales the animation angle by this amount relative to the physical angle being simulated."))
	float AileronAngleScalar = 1.0f;

	UPROPERTY(EditAnywhere, Meta = (Tooltip = "Ruddervator scalar parameter. Scales the animation angle by this amount relative to the physical angle being simulated."))
	float RuddervatorAngleScalar = 1.0f;
};

UCLASS()
class SKYPHYS_API AVTailPawn : public AFixedWingPawn
{
	GENERATED_BODY()
	
public:

	AVTailPawn();

private:
	// Update the Ruddervator Angles based on dr and da updates
	void UpdateRuddervatorAngles(float DeltaTime);
	void UpdateAileronAngles(float DeltaTime);

protected:
	// Components
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	UCtrlSurfaceStaticMeshComponent* LeftRuddervatorMesh; // Expect VTails to have Ruddervators
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	USecondOrderActuator* LeftRuddervatorServo;

	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	UCtrlSurfaceStaticMeshComponent* RightRuddervatorMesh; // Expect VTails to have Ruddervators
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	USecondOrderActuator* RightRuddervatorServo;

	// Expect Standard Config Fixed Wings to have 2 Ailerons (one on each wing)
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	UCtrlSurfaceStaticMeshComponent* LeftAileronMesh;
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	USecondOrderActuator* LeftAileronServo;

	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	UCtrlSurfaceStaticMeshComponent* RightAileronMesh;
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	USecondOrderActuator* RightAileronServo;

	UPROPERTY(BlueprintReadOnly, Category = "Animation")
	FVTailActuatorAnimationState ActuatorAnimationState;

	UPROPERTY(EditAnywhere, Category = "Animation")
	FVTailActuatorAnimationParameters ActuatorAnimationParameters;

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
