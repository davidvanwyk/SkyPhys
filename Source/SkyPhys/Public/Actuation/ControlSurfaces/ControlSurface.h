// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Common/Types.h"

#include "ControlSurface.generated.h"

// Forward declaration
class UActuatorModel;

// State Structs
USTRUCT(BlueprintType)
struct FControlSurfaceState
{
	GENERATED_BODY()

	float Deflection = 0.0f; // Current control surface deflection (rad), using RHR around its body X-Axis.
};

UCLASS(ClassGroup = "Control Surface", meta = (BlueprintSpawnableComponent))
class SKYPHYS_API UCtrlSurfaceStaticMeshComponent : public UStaticMeshComponent
{
	GENERATED_BODY()

public:
	// Apply the actuator command
	// This will then invoke the actuator driving the control surface and handle the dynamics associated with this
	//
	// @param Cmd: The unitless command signal (expect this to be between -1 and 1)
	// @param DeltaTime: The amount of time since the last command signal (s)
	void ApplyActuatorCommand(const float Cmd, const float DeltaTime);

	// Associate an actuator component to this propulsion model.
	// This actuator model will be responsible for managing the dynamics of the propulsion model.
	//
	// @param ActuatorModel: The actuator model to associate to this propulsion model.
	void AssociateActuatorComponent(UActuatorModel* ActuatorModel);

	// Get the current control surface motion state (eg. deflection angle)
	//
	// @return Deflection angle for the control surface (rad)
	float GetMotionState();

private:
	FControlSurfaceState ControlSurfaceState;

protected:
	// The actuator model to be used (if specified, otherwise just straight feedthrough with no dynamics)
	UActuatorModel* ActuatorModel;

	UPROPERTY(EditAnywhere, Category = "Control Surface", meta = (Tooltip = "The maximum deflection of the control surface (rad). Assumes symmetric about 0."))
	float MaxDeflection;
};