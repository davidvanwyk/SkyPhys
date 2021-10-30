// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Pawns/FlyingPawn.h"
#include "FixedWingPawn.generated.h"

// ***************** Declarations ***************** //

// Forward declaration

class UPropulsionPhysicsConstraint;
class UPropellerPropulsionStaticMeshComponent;
class UFirstOrderActuator;

// ################# Aerodynamics ################# //

// Force Coefficients

// Lift Force Control Derivatives
USTRUCT()
struct FCLControlDerivatives
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "CL de", Tooltip = "CL de"))
	float CLde = 0;
};

// Drag Force Control Derivatives
USTRUCT()
struct FCDControlDerivatives
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "CD de", Tooltip = "CD de"))
	float CDde = 0;
};

// Side Force Control Derivatives
USTRUCT()
struct FCYControlDerivatives
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "CY da", Tooltip = "CY da"))
	float CYda = 0;

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "CY dr", Tooltip = "CY dr"))
	float CYdr = 0;
};

// Moment Coefficients

// Roll Moment Control Derivatives (Actually Cl, but use "CI" for unique name purposes)
USTRUCT()
struct FCIControlDerivatives
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Cl da", Tooltip = "Cl da"))
	float CIda = 0;

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Cl dr", Tooltip = "Cl dr"))
	float CIdr = 0;
};

// Pitch Moment Control Derivatives
USTRUCT()
struct FCmControlDerivatives
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Cm de", Tooltip = "Cm de"))
	float Cmde = 0;
};

// Yaw Moment Control Derivatives
USTRUCT()
struct FCnControlDerivatives
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Cn da", Tooltip = "Cn da"))
	float Cnda = 0;

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Cn dr", Tooltip = "Cn dr"))
	float Cndr = 0;
};

// Aerodynamic Control Derivatives
USTRUCT()
struct FAerodynamicControlDerivatives
{
	GENERATED_BODY()

	// Force Control Derivatices

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "CL", Tooltip = "Lift Force Control Derivatives"))
	FCLControlDerivatives CL;

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "CD", Tooltip = "Drag Force Control Derivatives"))
	FCDControlDerivatives CD;

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "CY", Tooltip = "Side Force Control Derivatives"))
	FCYControlDerivatives CY;

	// Moment Control Derivatives

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Cl", Tooltip = "Roll Moment Control Derivatives"))
	FCIControlDerivatives CI;

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Cm", Tooltip = "Pitch Moment Control Derivatives"))
	FCmControlDerivatives Cm;

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Cn", Tooltip = "Yaw Moment Control Derivatives"))
	FCnControlDerivatives Cn;
};


// Stall Model
USTRUCT()
struct FAerodynamicStallParameters
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Enable Stall Model", Tooltip = "Whether to Enable the Stall Model"))
	bool bEnableStallModel = false;

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Alpha 0 (rad)", Tooltip = "Stall angle, Alpha 0 (rad)", EditCondition = "bEnableStallModel"))
	float Alpha0;

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "M", Tooltip = "Transition Rate, M", EditCondition = "bEnableStallModel"))
	float M;

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Cmfp", Tooltip = "Pitching Moment Flat Plate Coefficient, Cmfp", EditCondition = "bEnableStallModel"))
	float Cmfp;
};

// ################################################ //

// State Structs

// The actual actuator states
struct FFixedWingActuatorState
{
	float de = 0.0f; // Elevator angle (rad)
	float da = 0.0f; // Aileron angle (rad)
	float dr = 0.0f; // Rudder angle (rad)
};

// The current actuator commands
struct FFixedWingActuatorCommandState
{
	float de = 0.0f; // Elevator angle (rad)
	float da = 0.0f; // Aileron angle (rad)
	float dr = 0.0f; // Rudder angle (rad)
	float dt = 0.0f; // Propulsion level (Unitless)
};

// Calculation Structs
struct FAerodynamicStallCalculationParameters
{
	float SigmaAlpha = 0.0f;
};

// ************************************************ //

UCLASS(Abstract, NotBlueprintable)
class SKYPHYS_API AFixedWingPawn : public AFlyingPawn
{
	GENERATED_BODY()
	
public:
	// Sets default values for this pawn's properties
	AFixedWingPawn();

private:
	// Variables
	UPROPERTY(EditAnywhere, Category = "Aerodynamic Parameters")
	FAerodynamicControlDerivatives AerodynamicControlDerivatives;

	UPROPERTY(EditAnywhere, Category = "Aerodynamic Parameters")
	FAerodynamicStallParameters AerodynamicStallParameters;

protected:

	// Components
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	UPropellerPropulsionStaticMeshComponent* PropellerMesh; // Expect all fixed wings to have at least one propeller element.

	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	UFirstOrderActuator* PropellerMotor;

	// Called to update the current actuator state
	virtual void UpdateActuatorState(float DeltaTime) override;

	// Called to calculate forces and moments for this body
	virtual FForcesAndMoments CalculateAirframeForcesAndMoments() override;
	
	// Augment our forces in the body frame
	virtual float CalculateAdditionalCDParameters() const override;
	virtual float CalculateAdditionalCLParameters() const override;
	virtual float CalculateAdditionalCYParameters() const override;

	// Augment our moments in the body frame
	virtual float CalculateAdditionalClParameters() const override;
	virtual float CalculateAdditionalCmParameters() const override;
	virtual float CalculateAdditionalCnParameters() const override;

	// Stall Modelling
	void UpdateAerodynamicStallCalculationParameters();
	virtual FVector2D CalculateAdjustedCDAlphaCLAlphaForStall(float CDAlpha, float CLAlpha) const override;
	virtual float CalculateAdjustedCmAlphaForStall(float CmAlpha) const override;

	// Input Calculations

	// Apply the thrust command (expected Value of 0 -> 1)
	virtual void ApplyThrustCommand(float Value) override;

	// Parameters
	FFixedWingActuatorState ActuatorState;
	FFixedWingActuatorCommandState ActuatorCommandState;

	FAerodynamicStallCalculationParameters AerodynamicStallCalculationParameters;
};
