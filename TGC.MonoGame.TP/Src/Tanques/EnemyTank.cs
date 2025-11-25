using System;
using System.Collections.Generic;
using BepuPhysics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using TGC.MonoGame.TP.Fisicas;
using TGC.MonoGame.TP.Proyectiles;

namespace TGC.MonoGame.TP.Tanques;

public class EnemyTank(Vector3 initialPosition, PlayerTank playerTank, float initialRotation = 0f, float scale = 0.1f) : Tank(initialPosition, initialRotation, scale)
{
    // Método auxiliar
    private static float GetTankYaw(System.Numerics.Quaternion qBody)
    {
        // Forward local del tanque es (0,0,-1)
        var fwd = System.Numerics.Vector3.Transform(new System.Numerics.Vector3(0, 0, -1), qBody);
        return MathF.Atan2(fwd.X, fwd.Z) + MathF.PI; // 0 cuando mira -Z mundial
    }
    
    public void Update(float dt)
    {
        if (IsDead)
            return;
        
        if(!Simulation.Bodies.BodyExists(Body)) return;
        var body = Simulation.Bodies.GetBodyReference(Body);
        body.Awake = true;
            
        SteerRotation = Math.Clamp(SteerRotation, MinSteer, MaxSteer);
        
        FireCooldown = MathF.Max(0f, FireCooldown - dt);

        // Actualizar sonido del motor basado en la velocidad
        var velocity = body.Velocity.Linear;
        var speed = MathF.Sqrt(velocity.X * velocity.X + velocity.Z * velocity.Z);
        Audio?.UpdateEngine(speed, dt);

        // Girar ruedas según distancia recorrida
        UpdateWheelSpinByDistance();
            
        UpdateCanonAndTurretTowards();
            
        UpdateWorldMatrix();
    }

    public void UpdateAI(Vector3 playerPosition, TankController tankController, Simulation sim, List<Projectile> projectiles, Effect projectileEffect, CollidableProperty<TankBodyProperties> properties)
    {
        if (IsDead) return;
        
        var toPlayer = playerPosition - Position;
        var distance = toPlayer.Length();
        if (distance < 1e-4f) return;

        // Para que el tanque apunte un poco más arriba al jugador, en función de la distancia
        var toPlayerFixed = new Vector3(toPlayer.X, toPlayer.Y + distance / 30f, toPlayer.Z);
        if(distance < 1200f)
            AimDirectionWorld = Vector3.Normalize(toPlayerFixed);
        
        if (distance is > 350f and < 1200f)
        {
            // Yaw objetivo (misma convención: +PI para que frente -Z sea 0)
            var targetYaw = MathF.Atan2(AimDirectionWorld.X, AimDirectionWorld.Z) + MathF.PI;

            if(!Simulation.Bodies.BodyExists(Body)) return;
            var bodyRef = Simulation.Bodies.GetBodyReference(Body);
            var currentYaw = GetTankYaw(bodyRef.Pose.Orientation);

            // Diferencia envuelta a [-PI, PI]
            var angleDiff = MathHelper.WrapAngle(targetYaw - currentYaw);

            const float turnThreshold = 0.30f;
            
            if (MathF.Abs(angleDiff) > turnThreshold)
            {
                // Giro en el lugar: orugas opuestas, misma magnitud
                const float turnSpeed = 0.5f;

                if (angleDiff > 0f)
                {
                    // girar izquierda
                    tankController.FactorVelocidadIzquierda = -turnSpeed;
                    tankController.FactorVelocidadDerecha =  turnSpeed;
                }
                else
                {
                    // girar derecha
                    tankController.FactorVelocidadIzquierda =  turnSpeed;
                    tankController.FactorVelocidadDerecha = -turnSpeed;
                }
            }
            else
            {
                // Avanzar hacia el jugador
                tankController.FactorVelocidadIzquierda = 1.0f;
                tankController.FactorVelocidadDerecha = 1.0f;
            }

            tankController.Turbo = false;
            tankController.BrakeLeft = false;
            tankController.BrakeRight = false;
            tankController.UpdateMovementAndAim(Simulation, AimDirectionWorld);
            if(distance < 550f)
                Shoot(sim, projectiles, projectileEffect, properties);
            WasBraking = false;
        }
        else if(distance < 350f)
        {
            // Frenar cerca
            tankController.FactorVelocidadIzquierda = 0f;
            tankController.FactorVelocidadDerecha = 0f;
            tankController.Turbo = false;
            tankController.BrakeLeft = true;
            tankController.BrakeRight = true;
            tankController.UpdateMovementAndAim(Simulation, AimDirectionWorld);
            
            Shoot(sim, projectiles, projectileEffect, properties);

            if (!WasBraking)
                Audio?.PlayBrake();

            WasBraking = true;
        }
    }
    
    protected override void ResetCooldown()
    {
        FireCooldown = TipoProyectilActual.MaxCooldown * 3; // Para que disparen mas lento que el jugador
    }
    
    public override void Kill()
    {
        if (IsDead) return;
        base.Kill();
        playerTank.Curar(20);
    }
}