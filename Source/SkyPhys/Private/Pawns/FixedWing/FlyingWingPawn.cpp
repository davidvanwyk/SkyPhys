// Fill out your copyright notice in the Description page of Project Settings.


#include "Pawns/FixedWing/FlyingWingPawn.h"

#include "Actuation/Actuators/Filters/SecondOrderActuator.h"
#include "Actuation/ControlSurfaces/ControlSurface.h"
#include "Actuation/Propulsion/Propeller/PropellerPropulsion.h"

AFlyingWingPawn::AFlyingWingPawn()
{
	// Set up components

	LeftElevonMesh = CreateDefaultSubobject<UCtrlSurfaceStaticMeshComponent>(TEXT("Left Elevon Mesh"));
	LeftElevonServo = CreateDefaultSubobject<USecondOrderActuator>(TEXT("Left Elevon Servo Motor"));

	RightElevonMesh = CreateDefaultSubobject<UCtrlSurfaceStaticMeshComponent>(TEXT("Right Elevon Mesh"));
	RightElevonServo = CreateDefaultSubobject<USecondOrderActuator>(TEXT("Right Elevon Servo Motor"));

	// Associate servos to the meshes
	LeftElevonMesh->AssociateActuatorComponent(LeftElevonServo);
	RightElevonMesh->AssociateActuatorComponent(RightElevonServo);
}

void AFlyingWingPawn::UpdateActuatorState(float DeltaTime)
{
	Super::UpdateActuatorState(DeltaTime);

	// We only have elevons for a flying wing (aside from the propulsor), so this method handles everything for us.
	UpdateElevonAngles(DeltaTime);

	// Then we update our animation state.
	UpdateActuatorAnimationState();
}

void AFlyingWingPawn::UpdateActuatorAnimationState()
{
	// Then grab the latest actuator states and associate this to the animation state (which will need to be in UE4 units).
	ActuatorAnimationState.RightElevonAngle = FMath::RadiansToDegrees(RightElevonMesh->GetMotionState()) * ActuatorAnimationParameters.ElevonAngleScalar;
	ActuatorAnimationState.LeftElevonAngle = FMath::RadiansToDegrees(LeftElevonMesh->GetMotionState()) * ActuatorAnimationParameters.ElevonAngleScalar;

	// Then we just need to update the animation state.
	ActuatorAnimationState.PropellerSpeed = FMath::RadiansToDegrees(PropellerMesh->GetMotionState()) * ActuatorAnimationParameters.PropellerSpeedScalar;
}

void AFlyingWingPawn::UpdateElevonAngles(float DeltaTime)
{
	// We update the elevon angles here by applying the commands to the elevon actuators.
	// This transformation is from:
	// (de) = (1,  1)(der) 
	// (da)   (-1, 1)(del)

	// First apply the latest command
	// We divide the seconds by 2 as we expect this to get invoked twice per frame (once for each input axis)
	RightElevonMesh->ApplyActuatorCommand(0.5f * (ActuatorCommandState.de - ActuatorCommandState.da), DeltaTime);
	LeftElevonMesh->ApplyActuatorCommand(0.5f * (ActuatorCommandState.de + ActuatorCommandState.da), DeltaTime);

	// Now update our current actuator state using the original transform listed above, with our current actuator state value.
	ActuatorState.de = RightElevonMesh->GetMotionState() + LeftElevonMesh->GetMotionState();
	ActuatorState.da = LeftElevonMesh->GetMotionState() - RightElevonMesh->GetMotionState();
}

void AFlyingWingPawn::ApplyPitchCommand(float Value)
{
	Super::ApplyPitchCommand(Value);
	// Clamp the input to the expected range
	ActuatorCommandState.de = 0.0f;// FMath::Clamp(Value, -1.0f, 1.0f);
}

void AFlyingWingPawn::ApplyRollCommand(float Value)
{
	Super::ApplyRollCommand(Value);
	// Clamp the input to the expected range
	ActuatorCommandState.da = 0.0f;// FMath::Clamp(Value, -1.0f, 1.0f);
}

void AFlyingWingPawn::ApplyYawCommand(float Value)
{
	Super::ApplyYawCommand(Value);
	// Force rudder to 0 for a Flying Wing (which has no rudder).
	ActuatorCommandState.dr = 0.0f;
	ActuatorState.dr = 0.0f;
}
