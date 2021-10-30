// Fill out your copyright notice in the Description page of Project Settings.

#include "Actuation/ControlSurfaces/ControlSurface.h"

#include "Actuation/Actuators/ActuatorModel.h"

void UCtrlSurfaceStaticMeshComponent::ApplyActuatorCommand(float Cmd, float DeltaTime)
{
	if (ActuatorModel)
	{
		ControlSurfaceState.Deflection = ActuatorModel->ApplyActuatorCommand(Cmd, DeltaTime);
	}
	else
	{
		ControlSurfaceState.Deflection = FMath::Clamp((Cmd * MaxDeflection), -MaxDeflection, MaxDeflection);
	}
}

void UCtrlSurfaceStaticMeshComponent::AssociateActuatorComponent(UActuatorModel* pActuatorModel)
{
	ActuatorModel = pActuatorModel;
}

float UCtrlSurfaceStaticMeshComponent::GetMotionState()
{
	return ControlSurfaceState.Deflection;
}
