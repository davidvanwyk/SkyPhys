// Fill out your copyright notice in the Description page of Project Settings.


#include "Pawns/FixedWing/VTailPawn.h"

#include "Actuation/Actuators/Filters/SecondOrderActuator.h"
#include "Actuation/ControlSurfaces/ControlSurface.h"
#include "Actuation/Propulsion/Propeller/PropellerPropulsion.h"

AVTailPawn::AVTailPawn()
{
	// Set up components

	LeftRuddervatorMesh = CreateDefaultSubobject<UCtrlSurfaceStaticMeshComponent>(TEXT("Left Ruddervator Mesh"));
	LeftRuddervatorServo = CreateDefaultSubobject<USecondOrderActuator>(TEXT("Left Ruddervator Servo Motor"));

	RightRuddervatorMesh = CreateDefaultSubobject<UCtrlSurfaceStaticMeshComponent>(TEXT("Right Ruddervator Mesh"));
	RightRuddervatorServo = CreateDefaultSubobject<USecondOrderActuator>(TEXT("Right Ruddervator Servo Motor"));

	LeftAileronMesh = CreateDefaultSubobject<UCtrlSurfaceStaticMeshComponent>(TEXT("Left Aileron Mesh"));
	LeftAileronServo = CreateDefaultSubobject<USecondOrderActuator>(TEXT("Left Aileron Servo Motor"));

	RightAileronMesh = CreateDefaultSubobject<UCtrlSurfaceStaticMeshComponent>(TEXT("Right Aileron Mesh"));
	RightAileronServo = CreateDefaultSubobject<USecondOrderActuator>(TEXT("Right Aileron Servo Motor"));

	// Associate servos to meshes
	LeftRuddervatorMesh->AssociateActuatorComponent(LeftRuddervatorServo);
	RightRuddervatorMesh->AssociateActuatorComponent(RightRuddervatorServo);

	LeftAileronMesh->AssociateActuatorComponent(LeftAileronServo);
	RightAileronMesh->AssociateActuatorComponent(RightAileronServo);
}

void AVTailPawn::UpdateActuatorState(float DeltaTime)
{
	Super::UpdateActuatorState(DeltaTime);

	// First update the ruddervator values (this will include de and dr)
	UpdateRuddervatorAngles(DeltaTime);

	// Then update the aileron angles
	UpdateAileronAngles(DeltaTime);
}

void AVTailPawn::UpdateActuatorAnimationState()
{
	// Grab the latest actuator states and associate this to the animation state (which will need to be in UE4 units) for the Ruddervator
	ActuatorAnimationState.RightRuddervatorAngle = FMath::RadiansToDegrees(RightRuddervatorMesh->GetMotionState()) * ActuatorAnimationParameters.RuddervatorAngleScalar;
	ActuatorAnimationState.LeftRuddervatorAngle = FMath::RadiansToDegrees(LeftRuddervatorMesh->GetMotionState()) * ActuatorAnimationParameters.RuddervatorAngleScalar;

	// Then for the ailerons
	ActuatorAnimationState.LeftAileronAngle = FMath::RadiansToDegrees(LeftAileronMesh->GetMotionState()) * ActuatorAnimationParameters.AileronAngleScalar;
	ActuatorAnimationState.RightAileronAngle = -FMath::RadiansToDegrees(RightAileronMesh->GetMotionState()) * ActuatorAnimationParameters.AileronAngleScalar;

	// Now update the propeller speed
	ActuatorAnimationState.PropellerSpeed = FMath::RadiansToDegrees(PropellerMesh->GetMotionState()) * ActuatorAnimationParameters.PropellerSpeedScalar;
}

void AVTailPawn::UpdateRuddervatorAngles(float DeltaTime)
{
	// We update the elevon angles here - these are only used for animation purposes and aren't functional in terms of aerodynamics.
	// This transformation is from:
	// (de) = (1,  1)(drr) 
	// (dr)   (-1, 1)(drl)

	// First apply the latest command
	// We divide the seconds by 2 as we expect this to get invoked twice per frame (once for each input axis)
	RightRuddervatorMesh->ApplyActuatorCommand(0.5f * (ActuatorCommandState.de - ActuatorCommandState.dr), DeltaTime);
	LeftRuddervatorMesh->ApplyActuatorCommand(0.5f * (ActuatorCommandState.de + ActuatorCommandState.dr), DeltaTime);

	// Now update our current actuator state using the original transform listed above, with our current actuator state value.
	ActuatorState.de = RightRuddervatorMesh->GetMotionState() + LeftRuddervatorMesh->GetMotionState();
	ActuatorState.dr = LeftRuddervatorMesh->GetMotionState() - RightRuddervatorMesh->GetMotionState();
}

void AVTailPawn::UpdateAileronAngles(float DeltaTime)
{
	// We update the aileron angles here

	// First apply the latest command
	// We divide the seconds by 2 as we expect this to get invoked twice per frame (once for each input axis)
	LeftAileronMesh->ApplyActuatorCommand(0.5f * (ActuatorCommandState.de - ActuatorCommandState.dr), DeltaTime);
	RightAileronMesh->ApplyActuatorCommand(0.5f * (ActuatorCommandState.de + ActuatorCommandState.dr), DeltaTime);

	// Now update our current actuator state using the original transform listed above, with our current actuator state value.
	ActuatorState.da = 0.5 * (LeftAileronMesh->GetMotionState() - RightAileronMesh->GetMotionState());
}

void AVTailPawn::ApplyPitchCommand(float Value)
{
	Super::ApplyPitchCommand(Value);
	// Clamp the input to the expected range
	ActuatorCommandState.de = FMath::Clamp(Value, -1.0f, 1.0f);
}

void AVTailPawn::ApplyRollCommand(float Value)
{
	Super::ApplyRollCommand(Value);
	// Clamp the input to the expected range
	ActuatorCommandState.da = FMath::Clamp(Value, -1.0f, 1.0f);
}

void AVTailPawn::ApplyYawCommand(float Value)
{
	Super::ApplyYawCommand(Value);
	// Clamp the input to the expected range
	ActuatorCommandState.dr = FMath::Clamp(Value, -1.0f, 1.0f);
}
