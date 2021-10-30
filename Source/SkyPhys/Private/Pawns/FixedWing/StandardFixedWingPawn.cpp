// Fill out your copyright notice in the Description page of Project Settings.

#include "Pawns/FixedWIng/StandardFixedWingPawn.h"

#include "Actuation/Actuators/Filters/SecondOrderActuator.h"
#include "Actuation/ControlSurfaces/ControlSurface.h"
#include "Actuation/Propulsion/Propeller/PropellerPropulsion.h"

AStandardFixedWingPawn::AStandardFixedWingPawn()
{
	// Set up components
	LeftAileronMesh = CreateDefaultSubobject<UCtrlSurfaceStaticMeshComponent>(TEXT("Left Aileron Mesh"));
	LeftAileronServo = CreateDefaultSubobject<USecondOrderActuator>(TEXT("Left Aileron Servo Motor"));

	RightAileronMesh = CreateDefaultSubobject<UCtrlSurfaceStaticMeshComponent>(TEXT("Right Aileron Mesh"));
	RightAileronServo = CreateDefaultSubobject<USecondOrderActuator>(TEXT("Right Aileron Servo Motor"));

	ElevatorMesh = CreateDefaultSubobject<UCtrlSurfaceStaticMeshComponent>(TEXT("Elevator Mesh"));
	ElevatorServo = CreateDefaultSubobject<USecondOrderActuator>(TEXT("Elevator Servo Motor"));

	RudderMesh = CreateDefaultSubobject<UCtrlSurfaceStaticMeshComponent>(TEXT("Rudder Mesh"));
	RudderServo = CreateDefaultSubobject<USecondOrderActuator>(TEXT("Rudder Servo Motor"));

	// Associate servos to meshes
	LeftAileronMesh->AssociateActuatorComponent(LeftAileronServo);
	RightAileronMesh->AssociateActuatorComponent(RightAileronServo);
	ElevatorMesh->AssociateActuatorComponent(ElevatorServo);
	RudderMesh->AssociateActuatorComponent(RudderServo);

	// Disable collision for the elevons so that they don't interfere with things.
	LeftAileronMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	RightAileronMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);

	ElevatorMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);

	RudderMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
}

void AStandardFixedWingPawn::UpdateActuatorState(float DeltaTime)
{
	Super::UpdateActuatorState(DeltaTime);
	
	// First apply the current actuator commands for the elevator and rudder
	ElevatorMesh->ApplyActuatorCommand(ActuatorCommandState.de, DeltaTime);
	RudderMesh->ApplyActuatorCommand(ActuatorCommandState.dr, DeltaTime);

	// Then grab the current states
	ActuatorState.de = ElevatorMesh->GetMotionState();
	ActuatorState.dr = RudderMesh->GetMotionState();

	// Now update the ailerons
	UpdateAileronAngles(DeltaTime);

	// Now update the animation states
	UpdateActuatorAnimationState();
}

void AStandardFixedWingPawn::UpdateAileronAngles(float DeltaTime)
{
	// We update the aileron angles here

	// First apply the latest command
	// We divide the seconds by 2 as we expect this to get invoked twice per frame (once for each input axis)
	LeftAileronMesh->ApplyActuatorCommand(0.5f * (ActuatorCommandState.de - ActuatorCommandState.dr), DeltaTime);
	RightAileronMesh->ApplyActuatorCommand(0.5f * (ActuatorCommandState.de + ActuatorCommandState.dr), DeltaTime);

	// Now update our current actuator state using the original transform listed above, with our current actuator state value.
	ActuatorState.da = 0.5 * (LeftAileronMesh->GetMotionState() - RightAileronMesh->GetMotionState());
}

void AStandardFixedWingPawn::UpdateActuatorAnimationState()
{
	ActuatorAnimationState.ElevatorAngle = FMath::RadiansToDegrees(ElevatorMesh->GetMotionState()) * ActuatorAnimationParameters.ElevatorAngleScalar;

	ActuatorAnimationState.LeftAileronAngle = FMath::RadiansToDegrees(LeftAileronMesh->GetMotionState()) * ActuatorAnimationParameters.AileronAngleScalar;
	ActuatorAnimationState.RightAileronAngle = -FMath::RadiansToDegrees(RightAileronMesh->GetMotionState()) * ActuatorAnimationParameters.AileronAngleScalar;

	ActuatorAnimationState.RudderAngle = FMath::RadiansToDegrees(RudderMesh->GetMotionState()) * ActuatorAnimationParameters.RudderAngleScalar;

	ActuatorAnimationState.PropellerSpeed = FMath::RadiansToDegrees(PropellerMesh->GetMotionState()) * ActuatorAnimationParameters.PropellerSpeedScalar;
}

void AStandardFixedWingPawn::ApplyPitchCommand(float Value)
{
	Super::ApplyPitchCommand(Value);
	// Clamp the input to the expected range
	ActuatorCommandState.de = FMath::Clamp(Value, -1.0f, 1.0f);
}

void AStandardFixedWingPawn::ApplyRollCommand(float Value)
{
	Super::ApplyRollCommand(Value);
	// Clamp the input to the expected range
	ActuatorCommandState.da = FMath::Clamp(Value, -1.0f, 1.0f);
}

void AStandardFixedWingPawn::ApplyYawCommand(float Value)
{
	Super::ApplyYawCommand(Value);
	// Clamp the input to the expected range
	ActuatorCommandState.dr = FMath::Clamp(Value, -1.0f, 1.0f);
}
