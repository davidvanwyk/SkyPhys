// Fill out your copyright notice in the Description page of Project Settings.

#include "Pawns/MultiRotor/QuadRotorPawn.h"

#include "Actuation/Actuators/Filters/FirstOrderActuator.h"
#include "Actuation/Propulsion/Propeller/PropellerPropulsion.h"

#include <Eigen/Eigen>

// Sets default values
AQuadRotorPawn::AQuadRotorPawn()
{
	// Set up components
	Propeller1Mesh = CreateDefaultSubobject<UPropellerPropulsionStaticMeshComponent>(TEXT("Propeller 1 Mesh"));
	Propeller1Motor = CreateDefaultSubobject<UFirstOrderActuator>(TEXT("Propeller 1 Motor"));

	Propeller2Mesh = CreateDefaultSubobject<UPropellerPropulsionStaticMeshComponent>(TEXT("Propeller 2 Mesh"));
	Propeller2Motor = CreateDefaultSubobject<UFirstOrderActuator>(TEXT("Propeller 2 Motor"));

	Propeller3Mesh = CreateDefaultSubobject<UPropellerPropulsionStaticMeshComponent>(TEXT("Propeller 3 Mesh"));
	Propeller3Motor = CreateDefaultSubobject<UFirstOrderActuator>(TEXT("Propeller 3 Motor"));

	Propeller4Mesh = CreateDefaultSubobject<UPropellerPropulsionStaticMeshComponent>(TEXT("Propeller 4 Mesh"));
	Propeller4Motor = CreateDefaultSubobject<UFirstOrderActuator>(TEXT("Propeller 4 Motor"));

	// Associate actuators
	Propeller1Mesh->AssociateActuatorComponent(Propeller1Motor);
	Propeller2Mesh->AssociateActuatorComponent(Propeller2Motor);
	Propeller3Mesh->AssociateActuatorComponent(Propeller3Motor);
	Propeller4Mesh->AssociateActuatorComponent(Propeller4Motor);
}

void AQuadRotorPawn::UpdateActuatorState(float DeltaTime)
{
	Super::UpdateActuatorState(DeltaTime);

	// We're going to do a bunch of matrix maths here, so include Eigen.
	using namespace Eigen;

	// Define our output propeller speeds vector
	Vector4f PropellerSpeeds = Vector4f::Zero();

	// Read in the commands vector
	Vector4f Commands(MultiRotorCommandState.PitchCommand, MultiRotorCommandState.RollCommand, MultiRotorCommandState.YawCommand, MultiRotorCommandState.ThrustCommand);
	
	// We know that our mixing matrices and command offset won't change during runtime, so define these as consts.
	const Vector4f N = Vector4f::Constant(ThrustCommandOffset);

	// We define our initial mixing matrix, M, as a zero matrix, which we will fill in depending on our config.
	Matrix4f M = Matrix4f::Zero();

	// Now we handle things slightly differently depending on what configuration we have
	switch (MultiRotorConfiguration)
	{
		case EMultiRotorConfiguration::Cross:
			{
				// For a cross configuration we have the following:
				// Which can be found in more detail here: https://dev.px4.io/master/en/airframes/airframe_reference.html
				// 
				//        x
				// (3)+   ^     (1)-
				//   \    |     /
				//    \   |    /
				//     --------
				//     |      |  -----> y
				//     --------
				//    /        \
				//   /          \
				// (2)-         (4)+
				//
				// Where the rotation directions (with rotation axis "into the screen" and using RHR) are defined next to the propeller numbers.
				// 
				// We expect for a positive pitch (rotation about positive y), we will increase the speed of Propellers 1 and 3 and decrease for 2 and 4.
				// For a positive roll (rotation about positive x), we will increase the speed of Propellers 2 and 3 and decrease for 1 and 4.
				// For a positive yaw (rotation about positive z; defined as "into the screen"), we will increase the speed of Propellers 1 and 2 and 
				// decrease for 3 and 4 (because the moment exerted on the airframe is opposite to the direction of rotation of the propellers).
				//
				// From this we get the matrix:
				// 
				// M = [ 1 -1   1  1;
				//      -1  1   1  1;
				//       1  1  -1  1;
				//      -1 -1  -1  1]
				//

				const Matrix4f MCross{
				{ 1.0f, -1.0f,  1.0f, 1.0f},
				{-1.0f,  1.0f,  1.0f, 1.0f},
				{ 1.0f,  1.0f, -1.0f, 1.0f},
				{-1.0f, -1.0f, -1.0f, 1.0f}
				};

				M = MCross;
			}

			break;

		case EMultiRotorConfiguration::Plus:
			{
				// The plus configuration we have the following:
				//
				//            x
				//		      ^
				//		      |
				//		      |
				// 
				//		     (3)+     
				//		      |      
				//            |    
				//         --------
				// -(2)----|      |----(1)-     ----> y
				//         --------
				//		      |
				//		      |
				//		     (4)+
				//
				// Similar to the above discussion on the cross (or "x") config, we can determine that:
				//
				// For a positive pitch (positive rotation about y) we have that propeller 3 must increase in speed, and 4 decrease. Propellers 1 and 2 have no impact.
				// For a positive roll (positive rotation about x) we have that propeller 2 must increase in speed, and 1 decrease. Propelelrs 3 and 4 have no impact.
				// For a positive yaw (positive rotation about z, which is "into the screen") we have that propellers 1 and 2 must increase in speed, and 3 and 4 decrease.
				// For a positive thrust, all propellers must increase in speed.
				//
				// Similarly to the above, that results in a mixing matrix of:
				//
				// M = [ 0 -1  1  1;
				//       0  1  1  1;
				//		-1  0 -1  1;
				//		 1	0 -1  1]
				//
				const Matrix4f MPlus{
				{ 0.0f, -1.0f,  1.0f, 1.0f},
				{ 0.0f,  1.0f,  1.0f, 1.0f},
				{-1.0f,  0.0f, -1.0f, 1.0f},
				{ 1.0f,  0.0f, -1.0f, 1.0f}
				};

				M = MPlus;
			}

			break;
	}

	// In the above we have defined our mixing matrices, M, 
	// such that: M * [PitchCommand, RollCommand, YawCommand, ThrustCommand]^T = [P1, P2, P3, P4]^T
	//
	// We also augment this by applying our minimum propeller speed, such that:
	//
	// [P1, P2, P3, P4]^T = N* + ( 1/4 * M * [PitchCommand, RollCommand, YawCommand, ThrustCommand]^T ) * (1 - ThrustCommandOffset)
	// 
	// where N* is a 4x1 vector of the ThrustCommandOffset.
	// and where it should be noted that we have normalised the mixing matrix.
	//
	// This should align with the approach used by flight controllers like PX4.

	PropellerSpeeds = N + 0.25f * M * Commands * (1.0f - ThrustCommandOffset);

	// Finally, all these propeller commands are properly clamped to be between 0 and 1, as expected, which are then scaled to the correct value based on the motor used.
	Propeller1Mesh->ApplyActuatorCommand(FMath::Clamp(PropellerSpeeds(0), 0.0f, 1.0f), DeltaTime);
	Propeller2Mesh->ApplyActuatorCommand(FMath::Clamp(PropellerSpeeds(1), 0.0f, 1.0f), DeltaTime);
	Propeller3Mesh->ApplyActuatorCommand(FMath::Clamp(PropellerSpeeds(2), 0.0f, 1.0f), DeltaTime);
	Propeller4Mesh->ApplyActuatorCommand(FMath::Clamp(PropellerSpeeds(3), 0.0f, 1.0f), DeltaTime);

	// Then update the animation state of the propellers
	UpdateActuatorAnimationState();
}

void AQuadRotorPawn::UpdateActuatorAnimationState()
{
	// Update the animation state of each of the propellers
	ActuatorAnimationState.Propeller1Speed = FMath::RadiansToDegrees(Propeller1Mesh->GetMotionState()) * ActuatorAnimationParameters.PropellerSpeedScalar;
	ActuatorAnimationState.Propeller2Speed = FMath::RadiansToDegrees(Propeller2Mesh->GetMotionState()) * ActuatorAnimationParameters.PropellerSpeedScalar;
	ActuatorAnimationState.Propeller3Speed = FMath::RadiansToDegrees(Propeller3Mesh->GetMotionState()) * ActuatorAnimationParameters.PropellerSpeedScalar;
	ActuatorAnimationState.Propeller4Speed = FMath::RadiansToDegrees(Propeller4Mesh->GetMotionState()) * ActuatorAnimationParameters.PropellerSpeedScalar;
}
