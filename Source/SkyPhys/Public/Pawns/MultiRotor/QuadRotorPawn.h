// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Pawns/MultiRotor/MultiRotorPawn.h"
#include "QuadRotorPawn.generated.h"

// ***************** Declarations ***************** //

// Forward declaration

class UPropulsionPhysicsConstraint;
class UPropellerPropulsionStaticMeshComponent;
class UFirstOrderActuator;

// ************************************************ //

// States of our actuators for animation purposes (hence degrees, and not radians).
// These are NON-FUNCTIONAL and only used for graphical depictions.
USTRUCT(BlueprintType)
struct FQuadRotorActuatorAnimationState
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	float Propeller1Speed = 0.0f; // Scaled Propeller speed (deg/s)

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	float Propeller2Speed = 0.0f; // Scaled Propeller speed (deg/s)

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	float Propeller3Speed = 0.0f; // Scaled Propeller speed (deg/s)

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	float Propeller4Speed = 0.0f; // Scaled Propeller speed (deg/s)
};

UCLASS(Abstract, Blueprintable)
class SKYPHYS_API AQuadRotorPawn : public AMultiRotorPawn
{
	GENERATED_BODY()
	
public:
	// Sets default values for this pawn's properties
	AQuadRotorPawn();

protected:

	// Components

	// Propeller numbering is as per PX4 standard:
	// https://dev.px4.io/master/en/airframes/airframe_reference.html
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	UPropellerPropulsionStaticMeshComponent* Propeller1Mesh;
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	UPropellerPropulsionStaticMeshComponent* Propeller2Mesh;
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	UPropellerPropulsionStaticMeshComponent* Propeller3Mesh;
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	UPropellerPropulsionStaticMeshComponent* Propeller4Mesh;

	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	UFirstOrderActuator* Propeller1Motor;
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	UFirstOrderActuator* Propeller2Motor;
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	UFirstOrderActuator* Propeller3Motor;
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	UFirstOrderActuator* Propeller4Motor;

	UPROPERTY(BlueprintReadOnly, Category = "Animation")
	FQuadRotorActuatorAnimationState ActuatorAnimationState;

	// Called to update the current actuator state
	virtual void UpdateActuatorState(float DeltaTime) override;

	// Called to update the actuator animation state
	virtual void UpdateActuatorAnimationState() override;
};
