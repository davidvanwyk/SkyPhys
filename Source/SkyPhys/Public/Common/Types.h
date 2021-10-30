// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

struct FForcesAndMoments
{
	FVector		Forces; // (Fx, Fy, Fz)
	FVector		Moments; // (l, m, n)

	FForcesAndMoments(FVector Forces = FVector(0.0f), FVector Moments = FVector(0.0f)) : Forces(Forces), Moments(Moments) {}

	inline FForcesAndMoments operator+(FForcesAndMoments Sum)
	{
		Forces += Sum.Forces;
		Moments += Sum.Moments;
		return *this;
	};

	inline FForcesAndMoments operator+=(FForcesAndMoments Sum)
	{
		Forces += Sum.Forces;
		Moments += Sum.Moments;
		return *this;
	};
};