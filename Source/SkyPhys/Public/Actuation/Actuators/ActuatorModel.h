// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "ActuatorModel.generated.h"

UCLASS(Abstract)
class SKYPHYS_API UActuatorModel : public UActorComponent
{
	GENERATED_BODY()

public:
	UActuatorModel();

	// Apply a new command to the actuator.
	//
	// @param Command - The new command to apply to the actuator (unit depends on actuator)
	// @param DeltaTime - The amount of time since the last actuator command was provided (s) 
	//
	// @return The latest state of the actuator (unit depends on the actuator)
	virtual float ApplyActuatorCommand(float Command, float DeltaTime) PURE_VIRTUAL(UBaseActuator::ApplyActuatorCommand, return 0;);

	// Get the current state of the actuator.
	//
	// @return The latest state of the actuator (unit depends on the actuator)
	virtual float GetActuatorState() const PURE_VIRTUAL(UBaseActuator::GetActuatorState, return 0;);

protected:

	// Initialise the actuator
	//
	// @return The latest state of the actuator (unit depends on the actuator)
	virtual void InitialiseActuator() PURE_VIRTUAL(UBaseActuator::InitialiseActuator,);

	UPROPERTY(EditAnywhere, Category = "Actuator Parameters")
	float RateLimit = 0.0f;

	UPROPERTY(EditAnywhere, Category = "Actuator Parameters")
	float UpperSaturation = 0.0f;

	UPROPERTY(EditAnywhere, Category = "Actuator Parameters")
	float LowerSaturation = 0.0f;

	UPROPERTY(EditAnywhere, Category = "Actuator Parameters")
	float InitialActuatorState = 0.0f;

	float ActuatorState = 0.0f;
	bool bActuatorInitialised = false;
};
