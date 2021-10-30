// Fill out your copyright notice in the Description page of Project Settings.


#include "Actuation/Actuators/Filters/SecondOrderActuator.h"

USecondOrderActuator::USecondOrderActuator()
{
}

void USecondOrderActuator::InitialiseActuator()
{
	integrator1 = Integrator(0.0f);
	integrator2 = Integrator(InitialActuatorState);
	ActuatorState = InitialActuatorState;
	bActuatorInitialised = true;
}

float USecondOrderActuator::ApplyActuatorCommand(float Command, float DeltaTime)
{
	// Second order filter is modelled quite simply as:
	// 
	// ---(DC*wn^2)----(+)--->(+)--->(1/s)--------(1/s)--------->
	//				   (-)	  (-)				|		|
	//					|	   ^				|		|
	//					|	   |				|		|
	//					|	   ---(2*zeta*wn)----		|
	//					|								|
	//					-------------(wn^2)--------------			
	//
	// Integrator 1 is first in the feedforward path
	// Integrator 2 is second in the feedforward path
	// Feedback 2 is first in the feedforward path
	// Feedback 1 is second in the feedforward path

	if (!bActuatorInitialised)
	{
		InitialiseActuator();
	}

	float Input = Command * DCGain * pow(wn, 2.0f);
	float Feedback2 = integrator2.X * pow(wn, 2.0f);
	float Feedback1 = integrator1.X * 2 * zeta * wn;

	float Integrator1Input = Input - Feedback2 - Feedback1;
	float Integrator1Current = integrator1.X;
	float Integrator1Output = integrator1.Integrate(DeltaTime, Integrator1Input);

	float Integrator2Input = Integrator1Output;
	float Integrator2Current = integrator2.X;
	float Integrator2OutputExpected = integrator2.Integrate(DeltaTime, Integrator2Input);

	float Output = Integrator2OutputExpected;

	if (RateLimit)
	{
		Output = FMath::FInterpConstantTo(Integrator2Current, Integrator2OutputExpected, DeltaTime, RateLimit);
	}
	if (LowerSaturation)
	{
		Output = FMath::Clamp(Output, LowerSaturation, FLT_MAX);
	}
	if (UpperSaturation)
	{
		Output = FMath::Clamp(Output, -FLT_MAX, UpperSaturation);
	}
	ActuatorState = Output;

	return ActuatorState;
}

float USecondOrderActuator::GetActuatorState() const
{
	return ActuatorState;
}
