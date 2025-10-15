using System;
using System.Numerics;
using BepuPhysics;
using Vector3 = Microsoft.Xna.Framework.Vector3;

namespace TGC.MonoGame.TP;

/// <summary>
/// Applies control inputs to a tank instance.
/// </summary>
public struct TankController(
    Tank tank,
    float speed,
    float force,
    float zoomMultiplier,
    float idleForce,
    float brakeForce)
{
    //While the Tank instance contains references to all the simulation-contained stuff, none of it actually defines how fast or strong the tank is.
    //We store that here in the controller so it can be modified on the fly.

    //Track the previous state to force wakeups if the constraint targets have changed.
    private float _previousLeftTargetSpeed;
    private float _previousLeftForce;
    private float _previousRightTargetSpeed;
    private float _previousRightForce;
    private float _previousTurretSwivel;
    private float _previousBarrelPitch;

    /// <summary>
    /// Updates constraint targets for an input state.
    /// </summary>
    /// <param name="simulation">Simulation containing the tank.</param>
    /// <param name="leftTargetSpeedFraction">Target speed fraction of the maximum speed for the left tread.</param>
    /// <param name="rightTargetSpeedFraction">Target speed fraction of the maximum speed for the right tread.</param>
    /// <param name="zoom">Whether or not to use the boost mulitplier.</param>
    /// <param name="brakeLeft">Whether the left tread should brake.</param>
    /// <param name="brakeRight">Whether the right tread should brake.</param>
    /// <param name="aimDirection">Direction that the tank's barrel should point.</param>
    public void UpdateMovementAndAim(Simulation simulation,
        float leftTargetSpeedFraction, float rightTargetSpeedFraction,
        bool zoom, bool brakeLeft, bool brakeRight, Vector3 aimDirection)
    {
        if (tank.IsDead) return;

        var leftTargetSpeed = brakeLeft ? 0 : leftTargetSpeedFraction * speed;
        var rightTargetSpeed = brakeRight ? 0 : rightTargetSpeedFraction * speed;

        if (zoom)
        {
            leftTargetSpeed *= zoomMultiplier;
            rightTargetSpeed *= zoomMultiplier;
        }
        var leftForce  = brakeLeft  ? brakeForce : leftTargetSpeedFraction  == 0 ? idleForce : force;
        var rightForce = brakeRight ? brakeForce : rightTargetSpeedFraction == 0 ? idleForce : force;

        // Siempre calculá los ángulos objetivo
        var (targetSwivelAngle, targetPitchAngle) = tank.ComputeTurretAngles(simulation, -aimDirection);

        // Detectar cambios
        bool motorsChanged =
            leftTargetSpeed != _previousLeftTargetSpeed || rightTargetSpeed != _previousRightTargetSpeed ||
            leftForce != _previousLeftForce || rightForce != _previousRightForce;

        const float eps = 1e-4f;
        bool aimChanged =
            MathF.Abs(targetSwivelAngle - _previousTurretSwivel) > eps ||
            MathF.Abs(targetPitchAngle  - _previousBarrelPitch)  > eps;

        // Actualizar motores si hizo falta
        if (motorsChanged)
        {
            tank.SetSpeed(tank.LeftMotors, leftTargetSpeed, leftForce);
            tank.SetSpeed(tank.RightMotors, rightTargetSpeed, rightForce);
            _previousLeftTargetSpeed  = leftTargetSpeed;
            _previousRightTargetSpeed = rightTargetSpeed;
            _previousLeftForce  = leftForce;
            _previousRightForce = rightForce;
        }

        // IMPORTANTE: aplicar siempre que cambie el OBJETIVO de torreta/cañón
        if (aimChanged)
        {
            tank.SetAim(simulation, targetSwivelAngle, targetPitchAngle);
            _previousTurretSwivel = targetSwivelAngle;
            _previousBarrelPitch  = targetPitchAngle;
        }
    }

}