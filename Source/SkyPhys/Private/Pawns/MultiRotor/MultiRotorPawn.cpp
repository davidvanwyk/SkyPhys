// Fill out your copyright notice in the Description page of Project Settings.

#include "Pawns/MultiRotor/MultiRotorPawn.h"

// Sets default values
AMultiRotorPawn::AMultiRotorPawn()
{
}

void AMultiRotorPawn::UpdateActuatorState(float DeltaTime)
{
	Super::UpdateActuatorState(DeltaTime);
}

void AMultiRotorPawn::ApplyPitchCommand(float Value)
{
	Super::ApplyPitchCommand(Value);
	// Clamp the input to the expected range
	MultiRotorCommandState.PitchCommand = FMath::Clamp(Value, -1.0f, 1.0f);
}

void AMultiRotorPawn::ApplyRollCommand(float Value)
{
	Super::ApplyRollCommand(Value);
	// Clamp the input to the expected range
	MultiRotorCommandState.RollCommand = FMath::Clamp(Value, -1.0f, 1.0f);
}

void AMultiRotorPawn::ApplyYawCommand(float Value)
{
	Super::ApplyYawCommand(Value);
	// Clamp the input to the expected range
	MultiRotorCommandState.YawCommand = FMath::Clamp(Value, -1.0f, 1.0f);
}

void AMultiRotorPawn::ApplyThrustCommand(float Value)
{
	Super::ApplyThrustCommand(Value);
	// Clamp the input to the expected range
	MultiRotorCommandState.ThrustCommand = FMath::Clamp(Value, 0.0f, 1.0f);
}
