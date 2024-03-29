// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Common/Types.h"

#include "Propulsion.generated.h"

// Forward declaration
class UActuatorModel;

UCLASS(Abstract, ClassGroup = "Propulsion")
class SKYPHYS_API UPropulsionStaticMeshComponent : public UStaticMeshComponent
{
	GENERATED_BODY()

public:
	UPropulsionStaticMeshComponent();

	// Apply the actuator command
	// This will then invoke the actuator driving the propulsor and handle the dynamics associated with this
	//
	// @param dtCmd: The unitless command signal (expect this to be between 0 and 1)
	// @param DeltaTime: The amount of time since the last command signal (s)
	virtual void ApplyActuatorCommand(const float dtCmd, const float DeltaTime) PURE_VIRTUAL(UPropulsionStaticMeshComponent::ApplyActuatorCommand, );

	// Get Propulsion Forces and Moments in the world frame. 
	// Note that the propulsion frame will generally be with the z-axis pointing down for a multirotor, or toward the back of the aircraft for a fixed wing. 
	// Right hand rule will then determine what "clockwise" and "anticlockwise" mean. 
	// "Positive thrust", or in other words actually actively generating thrust, will result in a negative Z thrust in the propulsor body frame. 
	//
	// Calculate the forces and moments generated by this propulsor in the world frame, at the origin of the propulsor frame (ie. you will need to get the moments yourself!)
	// 
	// @param Rho: Air density (kg/m^3)
	// @param SystemOmega: Root body rotational velocity in the world frame (rad/s)
	// @param Vw: Wind velocity in the world frame (NEU) (m/s)
	// 
	// @return The forces and moments generated by this propulsor in the world frame (N, Nm)
	virtual FForcesAndMoments GetForcesAndMoments(float Rho, FVector Vw, FVector SystemOmega) PURE_VIRTUAL(UPropulsionStaticMeshComponent::GetForcesAndMoments, return FForcesAndMoments(););

	// Get the current motion state of the propulsor (eg. propeller speed in SI units for a propeller)
	//
	// @return The current motion state of the propulsion element
	virtual float GetMotionState() PURE_VIRTUAL(UPropulsionStaticMeshComponent::GetMotionState, return 0.0f;);

	// Associate an actuator component to this propulsion model.
	// This actuator model will be responsible for managing the dynamics of the propulsion model.
	//
	// @param ActuatorModel: The actuator model to associate to this propulsion model.
	void AssociateActuatorComponent(UActuatorModel* pActuatorModel);

protected:

	// Methods

	// Utility functions

	// Helper function to get from RPM to RPS
	// 
	// @param RPM Value in RPM
	// 
	// @return The value in RPS
	float RPMToRPS(float RPM);

	// Helper function to get from rad/s to RPM
	// 
	// @param RadPerS Value in rad/s
	// 
	// @return The value in RPM
	float RadPerSToRPM(float RadPerS);

	// The actuator model to be used (if specified, otherwise just straight feedthrough with no dynamics)
	UActuatorModel* ActuatorModel;
};