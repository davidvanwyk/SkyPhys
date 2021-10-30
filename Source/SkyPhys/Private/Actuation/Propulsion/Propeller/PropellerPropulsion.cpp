// Fill out your copyright notice in the Description page of Project Settings.

#include "Actuation/Propulsion/Propeller/PropellerPropulsion.h"

#include "Common/Utils/Helpers.h"
#include "DrawDebugHelpers.h"
#include "Pawns/FlyingPawn.h"
#include "Actuation/Actuators/ActuatorModel.h"

#include <Eigen/Eigen>

UPropellerPropulsionStaticMeshComponent::UPropellerPropulsionStaticMeshComponent()
{
}

void UPropellerPropulsionStaticMeshComponent::ApplyActuatorCommand(float dtCmd, float DeltaTime)
{
	if (ActuatorModel) 
	{
		PropellerState.omega = RPMToRPS(ActuatorModel->ApplyActuatorCommand(dtCmd, DeltaTime)) * 2 * PI;
	}
	else
	{
		PropellerState.omega = RPMToRPS(dtCmd * MaxN) * 2 * PI;
	}
}

FForcesAndMoments UPropellerPropulsionStaticMeshComponent::GetForcesAndMoments(float Rho, FVector Vw, FVector SystemOmega)
{
	// First check if we have initialized our parameters and if not, do so.
	if (!PropellerPhysicsCalculationParameters.bPhysicsParametersInitialized)
	{
		InitializePropellerPhysics();
	}

	// Then update the propeller state
	UpdatePropellerState(Rho, Vw);

	// Get the aerodynamic constants
	FAerodynamicConstantResults AerodynamicConstants = GetAerodynamicConstants(RadPerSToRPM(PropellerState.omega), PropellerState.J);

	// Then get forces and moments in the propeller body frame, AT THE PROPELLER (ie. moments do not include the effect of the forces at a distance).
	FVector ForcesBF = CalculateForces(AerodynamicConstants.CT);
	FVector MomentsBF = CalculateMoments(AerodynamicConstants.CP, SystemOmega);

	// Rotate into the WF
	FVector ForcesWF = TransformFromBodyToWorld(ForcesBF);
	FVector MomentsWF = TransformFromBodyToWorld(MomentsBF);

	// ******************************* Debug ******************************* //

	//FVector Location = GetComponentLocation();
	//float scaling = 1000.0f;

	// Forces
	//DrawDebugDirectionalArrow(GetWorld(), Location, Location + ForcesWF * scaling, 50.0f, FColor(255, 0, 0), false, 0, 0, 5.0f);
	// DrawDebugDirectionalArrow(GetWorld(), Location, Location + FVector(ForcesWF.X, 0, 0) * scaling, 50.0f, FColor(255, 0, 0), false, 0, 0, 5.0f);
	// DrawDebugDirectionalArrow(GetWorld(), Location, Location + FVector(0, ForcesWF.Y, 0) * scaling * 20, 50.0f, FColor(0, 255, 0), false, 0, 0, 5.0f);
	// DrawDebugDirectionalArrow(GetWorld(), Location, Location + FVector(0, 0, ForcesWF.Z) * scaling * 20, 50.0f, FColor(0, 0, 255), false, 0, 0, 5.0f);

	// Moments
	// DrawDebugDirectionalArrow(GetWorld(), Location, Location + MomentsWF * scaling * 10, 200.0f, FColor(255, 0, 0), false, 0, 0, 5.0f);
	//DrawDebugDirectionalArrow(GetWorld(), Location, Location + FVector(MomentsWF.X, 0, 0) * scaling * 50, 200.0f, FColor(255, 0, 0), false, 0, 0, 5.0f);
	//DrawDebugDirectionalArrow(GetWorld(), Location, Location + FVector(0, MomentsWF.Y, 0) * scaling * 50, 200.0f, FColor(255, 255, 0), false, 0, 0, 5.0f);
	//DrawDebugDirectionalArrow(GetWorld(), Location, Location + FVector(0, 0, MomentsWF.Z) * scaling * 50, 200.0f, FColor(0, 255, 255), false, 0, 0, 5.0f);

	// ********************************************************************* //

	return FForcesAndMoments(ForcesWF, MomentsWF);
}

float UPropellerPropulsionStaticMeshComponent::GetMotionState()
{
	return PropellerState.omega;
}

void UPropellerPropulsionStaticMeshComponent::InitializePropellerPhysics()
{
	// We initialize the parameters which we want to use for interpolation, as well as any other precalculated parameters which we want to only do once.

	if (!PropellerPhysicsCalculationParameters.bPhysicsParametersInitialized)
	{
		// For each combination of n and J we add these to the lookup arrays, as well as the resulting CT and CP
		for (FConstantSpeedPropellerPhysicsParameters ConstantSpeedParametersIter : PhysicsParameters.ConstantSpeedPropellerPhysicsParameters)
		{
			PropellerPhysicsCalculationParameters.NArray.Add(ConstantSpeedParametersIter.n);
		}

		PropellerPhysicsCalculationParameters.bPhysicsParametersInitialized = true;
	}
}

void UPropellerPropulsionStaticMeshComponent::UpdatePropellerState(float Rho, FVector Vw)
{
	// Note that we can't just do this in a component tick because we need Vw and Rho from the parent.

	// Get the current transform and rotations
	FTransform WorldT = GetComponentTransform();

	// Unreal to world frame
	FRotator Ruw = WorldT.Rotator();

	// Inverse to get Rwu
	FRotator Rwu = Ruw.GetInverse();

	// Save these
	PropellerState.Ruw = Ruw;
	PropellerState.Rwu = Rwu;

	// Get our current airspeed velocity (in m/s).
	FVector V = GetComponentVelocity()/100.0f;
	FVector Vaw = V - Vw;

	FVector Vab = TransformFromWorldToBody(Vaw);
	Vab = SkyPhysHelpers::RemoveNumericalErrors(Vab);
	PropellerState.V = Vab;

	// Set the air density
	PropellerState.Rho = Rho;

	// Calculate and assign the advance ratio
	float n = PropellerState.omega / (2 * PI);
	// Set advance ratio to either -FLT_MAX (arbitrarily large negative number that should cause us to clip with our interp method) or to the actual calculated value based on rotational speed,
	// unless V is zero, then set it to 0.
	PropellerState.J = 0.0f;
	if (!FMath::IsNearlyZero(PropellerState.V.Size()))
	{
		PropellerState.J = FMath::IsNearlyZero(n) ? -FLT_MAX : PropellerState.V.Size() / (n * PhysicsParameters.D);
	}

	// Calculate and assign the aerodynamic constant
	PropellerState.AerodynamicConstant = Rho * pow(n, 2.0f) * pow(PhysicsParameters.D, 4.0f);
}

FVector UPropellerPropulsionStaticMeshComponent::CalculateForces(float CT)
{
	// Here we calculate our forces in the propeller body frame. 
	// This will be defined as the same rotation as the airframe body frame for a multirotor due to the Z axis being downward, 
	// but will be rotated by 90 degrees around Y for a fixed wing due to the thrust (Z axis) being aligned with the airframe X axis.

	// Our forces generated by the propeller include our thrust,
	FVector T = CalculateThrustForces(CT);
	// As well as our side/hub force.
	FVector H = CalculateSideForces(T.Size());
	return T + H;
}

FVector UPropellerPropulsionStaticMeshComponent::CalculateThrustForces(float CT)
{
	// Calculate thrust magnitude from coefficient
	float T = CT * PropellerState.AerodynamicConstant;

	// We know that thrust will always be in the negative Z direction in the propeller body frame.
	return FVector(0, 0, -T);
}

FVector UPropellerPropulsionStaticMeshComponent::CalculateSideForces(float T)
{
	// We calculate side forces (or hub forces), H, based on a lumped drag model as derived in:
	// M. Bangura, Aerodynamics and Control of Quadrotors, The Australian National University, 2017

	// This includes:
	
	// Induced drag, Di, which is the drag due to having semi or fully rigid propeller blades which do not flap:
	// Di = -T*Ki*Vh

	// Translational drag, Dt, which is the drag due to the bending of the induced velocity streamtube of the airflow as it goes through
	// the rotor during translational motion.
	// Dt = -T*Kt*Vh

	// Profile drag, Dp, which is the drag caused by the transverse velocity of the rotor blades as they move through the air.
	// Dp = -T*Kp*Vg

	// The total drag therefore is:
	// D = Di + Dt + Dp
	// D = -T*Kr*V

	// Where:
	// Kr = [c, 0, 0;
	//		 0, c, 0;
	//	     0, 0, 0]
	// Where c is a lumped drag coefficient (by default we assume 0.01 as per the above reference), where it should be noted that we have divided by 4 as the author
	// has lumped all rotor effects for the quadrotor into a single coefficient and we only want this for a single propeller.

	// It does not include blade flapping, as we assume rigid blades are being modelled.

	FVector D(0.0f);

	if (!FMath::IsNearlyZero(T) && !FMath::IsNearlyZero(PropellerState.V.Size()))
	{
		{
			// We do our matrix stuff within an eigen block
			using namespace Eigen;
			// We know that Kr won't change during runtime, so we can safely make it const
			const Matrix3d Kr{
				{PhysicsParameters.Cd, 0						, 0},
				{0					 , PhysicsParameters.Cd		, 0},
				{0					 , 0						, 0}
			};
			Vector3d V(PropellerState.V.X, PropellerState.V.Y, PropellerState.V.Z);
			Vector3d DEig = -T * Kr * V;
			// and now convert to the expected form
			D.X = DEig(0);
			D.Y = DEig(1);
			D.Z = DEig(2);
		}
	}

	// Our lumped drag model is now returned as our H force.
	return D;
}

FVector UPropellerPropulsionStaticMeshComponent::CalculateMoments(float CP, FVector RootOmegaw)
{
	// Here we calculate our forces in the propeller body frame. 
	// This will be defined as the same rotation as the airframe body frame for a multirotor due to the Z axis being downward, 
	// but will be rotated by 90 degrees around Y for a fixed wing due to the thrust (Z axis) being aligned with the airframe X axis.

	// Our moments will comprise of aerodynamic moments
	FVector Q = CalculateAerodynamicMoments(CP);
	// As well as gyroscopic moments due to rotation of a very quickly rotating body (a propeller) about another rotation axis (the airframe).
	FVector G = CalculateGyroscopicMoments(RootOmegaw);
	return Q + G;
}

FVector UPropellerPropulsionStaticMeshComponent::CalculateAerodynamicMoments(float CP)
{
	// Get torque coefficient from the pre-calculated power coefficient
	float CQ = CP / (2.0f * PI);
	// Calculate torque magnitude from coefficient
	float Q = CQ * PropellerState.AerodynamicConstant * PhysicsParameters.D;
	// Adjust our direction based on the rotation of the propeller (torque will be in the opposite direction).
	int8 MomentDirection = -(int8)PhysicsParameters.RotationDirection;
	Q *= MomentDirection;
	return FVector(0, 0, Q);
}

FVector UPropellerPropulsionStaticMeshComponent::CalculateGyroscopicMoments(FVector SystemOmega)
{
	// Gyroscopic effect is calculated as:
	// G = I*omega*(OMEGA x k) (as can be found here: http://www.gyroscopes.org/math2.asp)
	// where:
	// I is the moment of inertia of our spinning body (so our propeller in this case)
	// omega is the propeller rotational velocity (rad/s)
	// SystemOmega is the airframe rotational velocity (rad/s)
	// k is the "z" direction of the propeller (which should be the only rotation axis for the propeller), which we cross with OMEGA to get the direction of the effect.

	// We need to first get OMEGA in the body frame of the propeller.
	FVector OMEGA = TransformFromWorldToBody(SystemOmega);

	// Now calculate the Gyroscopic moment as per the above.
	int8 MomentDirection = (int8)PhysicsParameters.RotationDirection;
	FVector G = MomentDirection * PhysicsParameters.Izz * PropellerState.omega * (FVector::CrossProduct(OMEGA, FVector::ZAxisVector));

	return G;
}

FAerodynamicConstantResults UPropellerPropulsionStaticMeshComponent::GetAerodynamicConstants(float n, float J)
{
	// Only run this if we actually have a defined array
	
	if (PropellerPhysicsCalculationParameters.NArray.Num() > 0)
	{
		// First get where we are in our NArray
		int n2i = FMath::Clamp(Algo::UpperBound(PropellerPhysicsCalculationParameters.NArray, n), 0, PropellerPhysicsCalculationParameters.NArray.Num() - 1); // This returns the index of the first value > the value provided, so will be our second element.
		int n1i = FMath::Clamp((n2i - 1), 0, PropellerPhysicsCalculationParameters.NArray.Num() - 1);

		float n1 = PropellerPhysicsCalculationParameters.NArray[n1i];
		float n2 = PropellerPhysicsCalculationParameters.NArray[n2i];

		// Use these indices to get the associated arrays we will use for calculations.
		TArray<float> jn1 = PhysicsParameters.ConstantSpeedPropellerPhysicsParameters[n1i].J;
		TArray<float> CTn1 = PhysicsParameters.ConstantSpeedPropellerPhysicsParameters[n1i].CT;
		TArray<float> CPn1 = PhysicsParameters.ConstantSpeedPropellerPhysicsParameters[n1i].CP;

		// First get our N1 J values by extracting which indices are on either side of our value.
		int j12i = FMath::Clamp(Algo::UpperBound(jn1, J), 0, jn1.Num() - 1);
		int j11i = FMath::Clamp((j12i - 1), 0, jn1.Num() - 1);

		float CT1 = CTn1[j11i];
		float CP1 = CPn1[j11i];

		// Now only interpolate between j11 and j12 if they are not the same (similar to N1 and N2)
		if (j11i != j12i)
		{
			// Then get the actual J values at these indices
			float j11 = jn1[j11i];
			float j12 = jn1[j12i];

			// Now get the N1 J fraction, which we will use for interpolation along this axis
			float j1Frac = FMath::Clamp(FMath::IsNearlyZero(j12 - j11) ? 0.0f : (j12 - J) / (j12 - j11), 0.0f, 1.0f);

			// First interpolate along the J axis
			CT1 += j1Frac * (CTn1[j12i] - CTn1[j11i]);
			CP1 += j1Frac * (CPn1[j12i] - CPn1[j11i]);
		}

		// Then we will set CT/CP to CT1/CP1, and interpolate along the N axis if needed
		float CT = CT1;
		float CP = CP1;

		// We only need to interpolate along the N axis if N1 != N2
		// So only repeat the above for N2 if required
		if (n1i != n2i)
		{
			// Now get this fraction, which we will use to interpolate a little later.
			float nFrac = FMath::Clamp((n - n1) / (n2 - n1), 0.0f, 1.0f);

			TArray<float> jn2 = PhysicsParameters.ConstantSpeedPropellerPhysicsParameters[n2i].J;
			TArray<float> CTn2 = PhysicsParameters.ConstantSpeedPropellerPhysicsParameters[n2i].CT;
			TArray<float> CPn2 = PhysicsParameters.ConstantSpeedPropellerPhysicsParameters[n2i].CP;

			// Do the same for the N2 J values
			int j22i = FMath::Clamp(Algo::UpperBound(jn2, J), 0, jn2.Num() - 1);
			int j21i = FMath::Clamp((j22i - 1), 0, jn2.Num() - 1);

			float CT2 = CTn2[j21i];
			float CP2 = CPn2[j21i];

			// Similar to the above, we only need to interpolate along J for N2 if J21 != J22.
			if (j21i != j22i)
			{
				float j21 = jn2[j21i];
				float j22 = jn2[j22i];

				float j2Frac = FMath::Clamp(FMath::IsNearlyZero(j22 - j21) ? 0.0f : (j22 - J) / (j22 - j21), 0.0f, 1.0f);

				CT2 += j2Frac * (CTn2[j22i] - CTn2[j21i]);
				CP2 += j2Frac * (CPn2[j22i] - CPn2[j21i]);
			}

			// Then finally interpolate along the N axis between N1 and N2
			CT += nFrac * (CT2 - CT1);
			CP += nFrac * (CP2 - CP1);
		}

		return FAerodynamicConstantResults(CT, CP);
	}

	return FAerodynamicConstantResults(0.0f, 0.0f);
}

FVector UPropellerPropulsionStaticMeshComponent::TransformFromWorldToBody(FVector WorldVector)
{
	// To get from the world to the body, we first go from world to unreal
	FVector UnrealFrame = PropellerState.Rwu.RotateVector(WorldVector);
	// We then expect the propeller frame to be oriented correctly in unreal so no flipping required.
	return FVector(UnrealFrame.X, UnrealFrame.Y, UnrealFrame.Z);
}

FVector UPropellerPropulsionStaticMeshComponent::TransformFromBodyToWorld(FVector BodyVector)
{
	// We expect the propeller frame to be oriented correctly in unreal so no flipping required.
	FVector UnrealFrame(BodyVector.X, BodyVector.Y, BodyVector.Z);
	// Then we rotate from unreal to world
	return PropellerState.Ruw.RotateVector(UnrealFrame);
}
