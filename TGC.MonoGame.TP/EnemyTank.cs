using System;
using System.Collections.Generic;
using BepuPhysics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class EnemyTank(Vector3 initialPosition, float initialRotation = 0f, float scale = 0.1f) : Tank(initialPosition, initialRotation, scale)
{
    // Método auxiliar
    private static float GetTankYaw(System.Numerics.Quaternion qBody)
    {
        // Forward local del tanque es (0,0,-1)
        var fwd = System.Numerics.Vector3.Transform(new System.Numerics.Vector3(0, 0, -1), qBody);
        return MathF.Atan2(fwd.X, fwd.Z) + MathF.PI; // 0 cuando mira -Z mundial
    }
    
    public void Update(GameTime gameTime)
    {
        if (IsDead)
            return;
            
        var dt = (float)gameTime.ElapsedGameTime.TotalSeconds;

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
        
        //playerPosition = new Vector3(playerPosition.X, playerPosition.Y + 15, playerPosition.Z);

        var toPlayer = playerPosition - Position;
        var distance = toPlayer.Length();
        if (distance < 1e-4f) return;
        
        AimDirectionWorld = Vector3.Normalize(toPlayer);
        
        if (distance > 350f)
        {
            // Yaw objetivo (misma convención: +PI para que frente -Z sea 0)
            var targetYaw = MathF.Atan2(AimDirectionWorld.X, AimDirectionWorld.Z) + MathF.PI;

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
            WasBraking = false;
        }
        else
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
        FireCooldown = TipoProyectilActual.MaxCooldown * 2; // Para que disparen mas lento que el jugador
    }
    
    public override void Kill()
    {
        base.Kill();
        
        Audio?.Dispose();
            
        foreach(var handle in BodyHandles)
        {
            if (Simulation.Bodies.BodyExists(handle))
            {
                Simulation.Bodies.Remove(handle);
            }
            CollisionHandler.HandleToTank.Remove(handle);
        }
    }
}