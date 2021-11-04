SkyPhys Unreal Plugin
=====================

The SkyPhys Plugin for UE4 aims to allow the user to (relatively) accurately simulate 6-DoF flight systems (this could also be extended to aquatic systems relatively easily).

Current Features
----------------

1. 6 DoF Kinematics, which has been coupled into the internal PhysX engine (which handles collisions etc.) using the recommended Physics subticking for frame-rate independent updates.
    * This includes allowing one to define custom system characteristics (mass and moments of inertia, including for asymmetric bodies where Ixz != 0)

1. General flight system aerodynamics using standard aerodynamic and control coefficients which one can get from wind tunnel testing (or the literature).
    * Note that these are assumed constant, as it's assumed that this will be used for RC simulations where Reynold's and Mach number variations will be negligible.
    * Control coefficients are only include for fixed wing aircraft where control surfaces might be present.
    * These equations are linear combinations, such that Cx(a, b, c) = Cx0 + a\*Cxa + b\*Cxb + c\*Cxc. (It should be noted that rotational rates, pqr, are non-dimensionalised by a length (ie. quarter-chord or wing span) and the current airspeed. This is excluded in the relationships to follow for the sake of brevity)
        * CL(alpha, q, de*)
        * CD(alpha, alpha^2, q, beta, beta^2, de*)
        * CY(beta, p, r, da*, dr*)
        * Cl(beta, p, r, da*, dr*)
        * Cm(alpha, q, de*)
        * Cn(beta, p, r, da*, dr*)

1. Stall modelling (at stall angle, alpha0)
    * Both lift force and pitching moment stall included.
    * Stall angle definable.
    * Pitching moment flate plate stall coefficient, Cmfp, configurable.
    * Stall models based on a sigmoid blending function that blends between a flate plate stall model, and the standard lift/pitching moment models based on the current angle of attack relative to the stall angle, with a definable transition rate, M.

1. Wind effect modelling 
    * The effect of wind is implicitly coupled into the aerodynamics on the airframe by directly impacting the airspeed. This then propagates into all equations via dynamic pressure, angle of attack and sideslip angle, which are all calculated based on the airspeed.
    * This currently uses UDS as an input to get the wind speed + direction, which can be scaled. The benefit of this is that the graphical weather effects from UDS (eg. slanted rain/snow etc.) will propogate through to the airframe physics.

1. Turbulence modelling for low altitude flight.

    * Dryden wind model with a customisable seed input for repeatable tests (if so desired).
    * Turbulence couples into the airframe dynamics in a similar way to wind, and is simply seen as an additional wind parameter which is calculated in the airframe body frame and added to the static wind after it has been rotated into the body frame as well. In other words: Vw = Rvb*Vwi + Vt, where Vw is the wind in the body frame, Rvb is the rotation from the vehicle to the body frame, Vwi is the inertial wind vector and Vt is the turbulence velocity.

1. Propeller modelling including:

    * Ability to specify propeller parameters based on propeller speed and advance ratio (as per [UIUC propeller data site](https://m-selig.ae.illinois.edu/props/propDB.html)).
    * Forces 
        * Thrust force based on the thrust coefficient from the propeller data (linearly interpolated)
        * Side force based on a lumped drag model for the propeller, with a configurable drag coefficient. As per: M. Bangura, Aerodynamics and Control of Quadrotors, The Australian National University, 2017
    * Moments 
        * Aerodynamic drag torque, based on the torque coefficient (derived from power coefficient) from the propeller data (linearly interpolated)
        * Gyroscopic moments (based on airframe angular velocity).
        * Moments due to propulsion forces at a distance
            * The drag and side forces are used to calculate a moment at the CoG of the airframe model by default, this depends on the location of the propeller (relative to the CoG) in the model itself.
    * All propeller forces and moments depend on the relative airspeed of the propeller (including wind, if relevant), and the air density.

1. Actuator modelling:

    * There is the ability to assign actuators to control surfaces (ie. servo motors), and propellers (ie. motors). These can either be defined as first or second order systems, and are essentially just standard filters of this type (ie. wn/(s+wn) and wn^2/(s^2 + 2\*zeta\*wn + wn^2)).
    * Actuator models have the following config:
        * Lower + Upper Saturation Limits
        * Rate Limits
        * Initial State

1. Animation

    * Animation is expected to be handled by the user, but some convenience utility is provided.
    * All animatable surfaces (eg. propellers + control surfaces) are exposed as blueprint variables. These can be independently scaled relative to their physical values (eg. 10000 RPM as a physical parameter can be scaled down to 1000 RPM if it's more practical to animate in this way). These variables can be used at the user's discretion for animating (eg. rotating movement components for propellers, adjusting the static mesh rotation for control surfaces etc.)

1. Pawns

    * Numerous flying pawns have been created and configured.

        * Fixed Wing Aircraft:
            * Flying Wing Configuration
            * V Tail Configuration
            * Standard Configuration
        * Multirotor Aircraft:
            * Quadrotor
            * Hexarotor
            * Octarotor

        These can all be subclassed (including the parent fixed wing and multirotor classes) for custom airframe types not included above.
        
        All of these systems are intended to be inherited from when creating your own custom airframe. For example, should you wish to create a Flying Wing aircraft, the recommendation would be to inherit from the Flying Wing Pawn as a base class, after which you can associate any custom implementations with with your airframe in this child class (eg. attaching meshes to sockets etc.). From this, the recommendation would be to create a blueprint class from this c++ pawn class and to handle all the "game" elements within this (eg. animation, sound, etc.).
    