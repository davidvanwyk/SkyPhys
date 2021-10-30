// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

class SKYPHYS_API Integrator
{
public:
	Integrator() {};
    Integrator(float InitialState) : X(InitialState), UPrev(InitialState), Y(InitialState) {};
	Integrator(float InitialState, float DtMin) : X(InitialState), UPrev(InitialState), Y(InitialState), DtMin(DtMin) {};

	float Integrate(const float Dt, const float U)
	{
        // Integrate multiple time-steps worth if our dt is larger than the minimum dt we have defined.
        if (Dt > DtMin) {
            float dtCumulative = 0;
            float y = 0;
            while (dtCumulative < Dt)
            {
                float tIncrement = (Dt - dtCumulative) > DtMin ? DtMin : (Dt - dtCumulative);
                y = Run(tIncrement, U);
                dtCumulative += tIncrement;
            }
            return y;
        }
        else
        {
            return Run(Dt, U);
        }
	}

	float X = 0.0f;
	float UPrev = 0;
	float Y = 0.0f;

private:

    float Run(const float Dt, const float U)
    {
        //Trapezoidal integration as per https://www.mathworks.com/help/simulink/slref/discretetimeintegrator.html with variable T
        Y = X + (Dt / 2.0f) * (U + UPrev);

        X = Y; //ie. x(n+1) = y(n)
        UPrev = U; //ie. u(n-1) = u(n)

        return Y;
    }

	float DtMin = 0.01f;

};
