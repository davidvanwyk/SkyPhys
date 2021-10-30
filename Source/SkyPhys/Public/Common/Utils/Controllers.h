// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

#include "Common/Utils/Integrator.h"

class SKYPHYS_API PIController
{
public:
    PIController() 
    {
        Intgrtr = Integrator(InitialState, DtMin);
    };
    PIController(float Kp, float Ki, float InitialState) : Kp(Kp), Ki(Ki), InitialState(InitialState) 
    {
        Intgrtr = Integrator(InitialState, DtMin);
    };
    PIController(float Kp, float Ki, float InitialState, float DtMin) : Kp(Kp), Ki(Ki), InitialState(InitialState), DtMin(DtMin) 
    {
        Intgrtr = Integrator(InitialState, DtMin);
    };

    float CalculateControllerOutput(const float Dt, const float U)
    {
        // Controller output for a PI controller is simply (Kp + Ki/s)*U.

        return Kp * U + Intgrtr.Integrate(Dt, Ki * U);

    }

private:

    float Kp = 0.0f;
    float Ki = 0.0f;

    float DtMin = 0.01f;
    float InitialState = 0.0f;

    Integrator Intgrtr;
};