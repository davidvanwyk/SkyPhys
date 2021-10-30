// Fill out your copyright notice in the Description page of Project Settings.


#include "Actuation/Actuators/Filters/FirstOrderActuator.h"

UFirstOrderActuator::UFirstOrderActuator()
{
}

void UFirstOrderActuator::InitialiseActuator()
{
	integrator = Integrator(InitialActuatorState);
	ActuatorState = InitialActuatorState;
	bActuatorInitialised = true;
}

float UFirstOrderActuator::ApplyActuatorCommand(float Command, float DeltaTime)
{
	// First order filter is modelled quite simply as:
	// -DC->(+)--->(wn/s)-------->
	//		(-)              |
	//		 ^               |
	//		 |				 |
	//		 -----------------

	if (!bActuatorInitialised)
	{
		InitialiseActuator();
	}

	float Input = Command * DCGain;
	float Feedback = integrator.X;
	float IntegratorInput = wn * (Input - Feedback);
	float IntegratorOutputExpected = integrator.Integrate(DeltaTime, IntegratorInput);
	float IntegratorOutput = IntegratorOutputExpected;

	if (RateLimit)
	{
		IntegratorOutput = FMath::FInterpConstantTo(Feedback, IntegratorOutputExpected, DeltaTime, RateLimit);
	}
	if (LowerSaturation)
	{
		IntegratorOutput = FMath::Clamp(IntegratorOutput, LowerSaturation, FLT_MAX);
	}
	if (UpperSaturation)
	{
		IntegratorOutput = FMath::Clamp(IntegratorOutput, -FLT_MAX, UpperSaturation);
	}
	ActuatorState = IntegratorOutput;

	return ActuatorState;
}

float UFirstOrderActuator::GetActuatorState() const
{
	return ActuatorState;
}
