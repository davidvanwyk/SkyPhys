#pragma once

#include <random>

#include "CoreMinimal.h"
#include "Common/Utils/Integrator.h"

#include "Dryden.generated.h"

// Base class for the Dryden model transfer function. Contains all common parameters that all 3 axes use.
UCLASS(EditInlineNew, Abstract)
class SKYPHYS_API UDrydenModelTFBase : public UObject
{
	GENERATED_BODY()

public:
	UDrydenModelTFBase() {};

	float GetTurbulence(float Va, float Dt, float L, float Sigma)
	{
		if (!IsInitialized)
		{
			Initialize();
		}
		// Ensure that Va is sensible before passing it through
		Va = (FMath::IsNaN(Va) || FMath::IsNearlyZero(Va)) ? 0 : Va;
		// Generate our noise using the RNG engine, passed into a normal distribution.
		// This is band limited white noise, which is scaled to sigma/sqrt(Ts) in order to have correct scaling in a discrete sim.
		// More information on this process can be found here: https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model
		// And this is also what is done in the Simulink White Noise model as part of the Dryden Wind Turbulence block.
		// Note: The Pi scaling comes from Simulink - not 100% sure where they got this from.
		float noise = sqrtf(PI / Ts) * WhiteNoise(RNGEngine);
		float turbulenceFts = Filter(Dt, Va, L, Sigma, noise);
		turbulenceFts = (FMath::IsNaN(turbulenceFts) || FMath::IsNearlyZero(turbulenceFts)) ? 0.0f : turbulenceFts;
		return turbulenceFts;
	}

private:
	// Methods
	void Initialize()
	{
		RNGEngine = std::default_random_engine(Seed);
		WhiteNoise = std::normal_distribution<float>(0.0f, 1.0f); // Normal distribution of Mean = 0, Variance/StdDev = 1
		InitialiseIntegrators();
		IsInitialized = true;
	}

	// Parameters
	bool IsInitialized = false;
	std::default_random_engine RNGEngine;
	std::normal_distribution<float> WhiteNoise;

protected:
	// Editor Properties
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "White Noise Generator RNG Seed"))
	int Seed;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "Sample Time (s)"))
	float Ts;

	// Initialize all integrators required.
	virtual void InitialiseIntegrators() PURE_VIRTUAL(UDrydenModelTFBase::InitialiseIntegrators);
	// Perform the actual filtering step (to provide the turbulence value).
	virtual float Filter(float Dt, float Va, float L, float Sigma, float Noise) PURE_VIRTUAL(UDrydenModelTFBase::Filter, return 0.0f;);
};

// Dryden Model Hu transfer function implementation.
UCLASS()
class SKYPHYS_API UDrydenModelTFHu : public UDrydenModelTFBase
{
	GENERATED_BODY()

public:
	UDrydenModelTFHu() {};

private:
	Integrator _Integrator;

protected:
	// Initialize our integrators
	virtual void InitialiseIntegrators() override
	{
		_Integrator = Integrator(0.0f, Ts);
	}

	//Dryden Model for our Forward Velocity (Imperial Units)
	virtual float Filter(float Dt, float Va, float L, float Sigma, float Noise) override
	{
		//	//This process matches that found in Hugw(s) within the Dryden Wind Turbulence Model (Continuous) in Simulink
		//I've tried to keep the variable names consistent with Simulink where possible.

		// Make Lug_over_Va as large as possible without being NaN if Va is nearly zero.
		float Lug_over_Va = FLT_MAX;
		if (!FMath::IsNearlyZero(Va))
		{
			Lug_over_Va = L / Va;
		}
		float feedback_input = sqrtf((Lug_over_Va) * (2.0f / PI)) * Noise;
		float summing_output = feedback_input - _Integrator.X;
		float w = summing_output / Lug_over_Va;
		float ug_p = _Integrator.Integrate(Dt, w);
		return Sigma * ug_p;
	}
};

// Dryden Model Hv transfer function implementation.
UCLASS()
class SKYPHYS_API UDrydenModelTFHv : public UDrydenModelTFBase
{
	GENERATED_BODY()

public:
	UDrydenModelTFHv() {};

private:
	Integrator _Integrator_Vg_P1;
	Integrator _Integrator_Vg_P2;

protected:
	// Initialize our integrators
	virtual void InitialiseIntegrators() override
	{
			_Integrator_Vg_P1 = Integrator(0.0f, Ts);
			_Integrator_Vg_P2 = Integrator(0.0f, Ts);
	}

	////Dryden Model for our Side Velocity (Imperial Units)
	virtual float Filter(float Dt, float Va, float L, float Sigma, float Noise) override
	{
		//This process matches that found in Hvgw(s) within the Dryden Wind Turbulence Model (Continuous) in Simulink
		//I've tried to keep the variable names consistent with Simulink where possible.
	
		//This is the first stage, up until the integrator named vg_p1
	
		// Make Lvg_over_Va as large as possible without being NaN if Va is nearly zero.
		float Lvg_over_Va = FLT_MAX;
		if (!FMath::IsNearlyZero(Va))
		{
			Lvg_over_Va = L / Va;
		}
		float feedback_input = sqrtf((Lvg_over_Va) * (1.0f / PI)) * Noise;
		float summing1_output = feedback_input - _Integrator_Vg_P1.X;
		float w1 = summing1_output / Lvg_over_Va; //They've used the name "w" twice, so w1 and w2 for us
		float vg_p1 = _Integrator_Vg_P1.Integrate(Dt, w1);
		// Second part of the integration, from vg_p1 onward
		float summing2_input_2 = sqrt(3.0f) * Lvg_over_Va * w1; //Second summing input to the second summing junction.
		float summing2_output = vg_p1 + summing2_input_2 - _Integrator_Vg_P2.X;
		float w2 = summing2_output / Lvg_over_Va;
		float vg_p2 = _Integrator_Vg_P2.Integrate(Dt, w2);
		return Sigma * vg_p2;
	}
};

// Dryden Model Hw transfer function implementation.
UCLASS()
class SKYPHYS_API UDrydenModelTFHw : public UDrydenModelTFBase
{
	GENERATED_BODY()

public:
	UDrydenModelTFHw() {};

private:
	Integrator _Integrator_Wg_P1;
	Integrator _Integrator_Wg_P2;

protected:
	// Initialize our integrators
	virtual void InitialiseIntegrators() override
	{
		_Integrator_Wg_P1 = Integrator(0.0f, Ts);
		_Integrator_Wg_P2 = Integrator(0.0f, Ts);
	}

	//Dryden Model for our Side Velocity (Imperial Units)
	float Filter(float Dt, float Va, float L, float Sigma, float Noise)
	{
		//This process matches that found in Hwgw(s) within the Dryden Wind Turbulence Model (Continuous) in Simulink
		//I've tried to keep the variable names consistent with Simulink where possible.
	
		//This is the first stage, up until the integrator named wg_p1
	
		// Make Lug_over_Va as large as possible without being NaN if Va is nearly zero.
		float Lwg_over_Va = FLT_MAX;
		if (!FMath::IsNearlyZero(Va))
		{
			Lwg_over_Va = L / Va;
		}
		float feedback_input = sqrtf((Lwg_over_Va) * (1.0f / PI)) * Noise;
		float summing1_output = feedback_input - _Integrator_Wg_P1.X;
		float w1 = summing1_output / Lwg_over_Va; // They've used the name "w" twice, so w1 and w2 for us
		float wg_p1 = _Integrator_Wg_P1.Integrate(Dt, w1);
		// Second part of the integration, from wg_p1 onward
		float summing2_input_2 = sqrt(3.0f) * Lwg_over_Va * w1; //Second summing input to the second summing junction.
		float summing2_output = wg_p1 + summing2_input_2 - _Integrator_Wg_P2.X;
		float w2 = summing2_output / Lwg_over_Va;
		float wg_p2 = _Integrator_Wg_P2.Integrate(Dt, w2);
		return Sigma * wg_p2;
	}
};