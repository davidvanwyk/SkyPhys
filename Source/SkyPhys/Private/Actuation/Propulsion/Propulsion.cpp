// Fill out your copyright notice in the Description page of Project Settings.

#include "Actuation/Propulsion/Propulsion.h"

UPropulsionStaticMeshComponent::UPropulsionStaticMeshComponent()
{
}

void UPropulsionStaticMeshComponent::AssociateActuatorComponent(UActuatorModel* pActuatorModel)
{
	ActuatorModel = pActuatorModel;
}

float UPropulsionStaticMeshComponent::RPMToRPS(float RPM)
{
	return RPM / 60.0f;
}

float UPropulsionStaticMeshComponent::RadPerSToRPM(float RadPerS)
{
	return (RadPerS / (2.0f*PI)) * 60.0f;
}
