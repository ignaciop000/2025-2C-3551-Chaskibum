using System;
using System.Numerics;
using BepuPhysics;
using Microsoft.Xna.Framework.Input;
using Vector3 = Microsoft.Xna.Framework.Vector3;

namespace TGC.MonoGame.TP;

/// <summary>
/// Applies control inputs to a tank instance.
/// </summary>
public struct TankController(
    Tank tank,
    float speed,
    float force,
    float turboMultiplier,
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

    public float FactorVelocidadIzquierda;
    public float FactorVelocidadDerecha;
    public bool Turbo;
    public bool BrakeLeft;
    public bool BrakeRight;

    /// <summary>
    /// Updates constraint targets for an input state.
    /// </summary>
    /// <param name="simulation">Simulation containing the tank.</param>
    /// <param name="aimDirection">Direction that the tank's barrel should point.</param>
    public void UpdateMovementAndAim(Simulation simulation, Vector3 aimDirection)
    {
        if (tank.IsDead) return;

        var leftTargetSpeed = BrakeLeft ? 0 : FactorVelocidadIzquierda * speed;
        var rightTargetSpeed = BrakeRight ? 0 : FactorVelocidadDerecha * speed;

        if (Turbo)
        {
            leftTargetSpeed *= turboMultiplier;
            rightTargetSpeed *= turboMultiplier;
        }

        var leftForce = BrakeLeft ? brakeForce : FactorVelocidadIzquierda == 0 ? idleForce : force;
        var rightForce = BrakeRight ? brakeForce : FactorVelocidadDerecha == 0 ? idleForce : force;
        
        //validamos si hubo cambios en la velocidadesFinales o fuerzas
        if (leftTargetSpeed != _previousLeftTargetSpeed 
            || rightTargetSpeed != _previousRightTargetSpeed 
            || leftForce != _previousLeftForce 
            || rightForce != _previousRightForce)
        {
            tank.SetSpeed(tank.LeftMotors, leftTargetSpeed, leftForce);
            tank.SetSpeed(tank.RightMotors, rightTargetSpeed, rightForce);
            _previousLeftTargetSpeed = leftTargetSpeed;
            _previousRightTargetSpeed = rightTargetSpeed;
            _previousLeftForce = leftForce;
            _previousRightForce = rightForce;
        }

        // Siempre calculá los ángulos objetivo
        var (targetSwivelAngle, targetPitchAngle) = tank.ComputeTurretAngles(simulation, aimDirection);
        if ( targetSwivelAngle != _previousTurretSwivel
             || targetPitchAngle != _previousBarrelPitch)
        {
            tank.SetAim(simulation, targetSwivelAngle, targetPitchAngle);
            _previousTurretSwivel = targetSwivelAngle;
            _previousBarrelPitch = targetPitchAngle;
        }
    }
    
    public void UpdateControls(KeyboardState keyboardState)
    {
        FactorVelocidadIzquierda = 0; //-1..0..1
        FactorVelocidadDerecha = 0; //-1..0..1
        var izquierda = keyboardState.IsKeyDown(Keys.A);
        var derecha = keyboardState.IsKeyDown(Keys.D);
        var adelante = keyboardState.IsKeyDown(Keys.W);
        var atras = keyboardState.IsKeyDown(Keys.S);

        if (adelante)
        {
            if ((izquierda && derecha) || (!izquierda && !derecha))
            {
                FactorVelocidadIzquierda = 1f;
                FactorVelocidadDerecha = 1f;
            }
            else if (izquierda)
            {
                FactorVelocidadIzquierda = 0.5f;
                FactorVelocidadDerecha = 1f;
            }
            else
            {
                FactorVelocidadIzquierda = 1f;
                FactorVelocidadDerecha = 0.5f;
            }
        }
        else if (atras)
        {
            if ((izquierda && derecha) || (!izquierda && !derecha))
            {
                FactorVelocidadIzquierda = -1f;
                FactorVelocidadDerecha = -1f;
            }
            else if (izquierda)
            {
                FactorVelocidadIzquierda = -0.5f;
                FactorVelocidadDerecha = -1f;
            }
            else
            {
                FactorVelocidadIzquierda = -1f;
                FactorVelocidadDerecha = -0.5f;
            }
        }
        else
        {
            if (izquierda && !derecha)
            {
                FactorVelocidadIzquierda = -1f;
                FactorVelocidadDerecha = 1f;
            }
            else if (derecha && !izquierda)
            {
                FactorVelocidadIzquierda = 1f;
                FactorVelocidadDerecha = -1f;
            }
        }

        Turbo = keyboardState.IsKeyDown(Keys.LeftShift);
        BrakeRight = BrakeLeft = keyboardState.IsKeyDown(Keys.Space);
    }
}