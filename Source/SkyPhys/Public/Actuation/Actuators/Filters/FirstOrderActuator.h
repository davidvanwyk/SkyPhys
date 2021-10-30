// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

#include "Actuation/Actuators/ActuatorModel.h"
#include "Common/Utils/Integrator.h"

#include "FirstOrderActuator.generated.h"

UCLASS()
class SKYPHYS_API UFirstOrderActuator : public UActuatorModel
{
	GENERATED_BODY()

public:
	UFirstOrderActuator();

	// Apply a new command to the actuator.
	//
	// @param Command - The new command to apply to the actuator (unit depends on actuator)
	// @param DeltaTime - The amount of time since the last actuator command was provided (s)
	//
	// @return The latest state of the actuator (unit depends on the actuator)
	virtual float ApplyActuatorCommand(float Command, float DeltaTime) override;

	// Get the current state of the actuator.
	//
	// @return The latest state of the actuator (unit depends on the actuator)
	virtual float GetActuatorState() const override;

protected:

	virtual void InitialiseActuator() override;

	UPROPERTY(EditAnywhere, Category = "Actuator Parameters", Meta = (Tooltip = "Natural frequency of the filter (rad/s)"))
	float wn = 0.0f;

	UPROPERTY(EditAnywhere, Category = "Actuator Parameters", Meta = (Tooltip = "DC Gain of the filter (unitless)"))
	float DCGain = 1.0f;

private:

	Integrator integrator;
};