// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

namespace SkyPhysHelpers
{
	FVector RemoveNumericalErrors(FVector TestVector)
	{
		// Return 0 if our vector contains NaNs... as something has then gone wrong.
		if (TestVector.ContainsNaN()) {
			return FVector(0.0f);
		}

		// Remove small numerical errors due to floating points, given that we're integrating these.
		float ErrorTolerance = 0.0001f;
		TestVector.X = FMath::IsNearlyZero(TestVector.X, ErrorTolerance) ? 0.0f : TestVector.X;
		TestVector.Y = FMath::IsNearlyZero(TestVector.Y, ErrorTolerance) ? 0.0f : TestVector.Y;
		TestVector.Z = FMath::IsNearlyZero(TestVector.Z, ErrorTolerance) ? 0.0f : TestVector.Z;

		return TestVector;
	}
}