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

    public float factorVelocidadIzquierda;
    public float factorVelocidadDerecha;
    public bool turbo;
    public bool brakeLeft;
    public bool brakeRight;

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
    public void UpdateMovementAndAim(Simulation simulation, Vector3 aimDirection)
    {
        if (tank.IsDead) return;

        var leftTargetSpeed = brakeLeft ? 0 : factorVelocidadIzquierda * speed;
        var rightTargetSpeed = brakeRight ? 0 : factorVelocidadDerecha * speed;

        if (turbo)
        {
            leftTargetSpeed *= turboMultiplier;
            rightTargetSpeed *= turboMultiplier;
        }

        var leftForce = brakeLeft ? brakeForce : factorVelocidadIzquierda == 0 ? idleForce : force;
        var rightForce = brakeRight ? brakeForce : factorVelocidadDerecha == 0 ? idleForce : force;
        
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
        var (targetSwivelAngle, targetPitchAngle) = tank.ComputeTurretAngles(simulation, -aimDirection);
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
        factorVelocidadIzquierda = 0; //-1..0..1
        factorVelocidadDerecha = 0; //-1..0..1
        var izquierda = keyboardState.IsKeyDown(Keys.A);
        var derecha = keyboardState.IsKeyDown(Keys.D);
        var adelante = keyboardState.IsKeyDown(Keys.W);
        var atras = keyboardState.IsKeyDown(Keys.S);

        if (adelante)
        {
            if ((izquierda && derecha) || (!izquierda && !derecha))
            {
                factorVelocidadIzquierda = 1f;
                factorVelocidadDerecha = 1f;
            }
            else if (izquierda)
            {
                factorVelocidadIzquierda = 0.5f;
                factorVelocidadDerecha = 1f;
            }
            else
            {
                factorVelocidadIzquierda = 1f;
                factorVelocidadDerecha = 0.5f;
            }
        }
        else if (atras)
        {
            if ((izquierda && derecha) || (!izquierda && !derecha))
            {
                factorVelocidadIzquierda = -1f;
                factorVelocidadDerecha = -1f;
            }
            else if (izquierda)
            {
                factorVelocidadIzquierda = -0.5f;
                factorVelocidadDerecha = -1f;
            }
            else
            {
                factorVelocidadIzquierda = -1f;
                factorVelocidadDerecha = -0.5f;
            }
        }
        else
        {
            if (izquierda && !derecha)
            {
                factorVelocidadIzquierda = -1f;
                factorVelocidadDerecha = 1f;
            }
            else if (derecha && !izquierda)
            {
                factorVelocidadIzquierda = 1f;
                factorVelocidadDerecha = -1f;
            }
        }

        turbo = keyboardState.IsKeyDown(Keys.LeftShift);
        brakeRight = brakeLeft = keyboardState.IsKeyDown(Keys.Space);
    }
}