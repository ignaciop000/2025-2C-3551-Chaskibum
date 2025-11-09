using System;
using Microsoft.Xna.Framework;

namespace TGC.MonoGame.TP;

public class EnemyTank(Vector3 initialPosition, float initialRotation = 0f, float scale = 1f) : Tank(initialPosition, initialRotation, scale)
{
    public override void RecibirAtaque(float danio)
    {
        Vida -= danio;
            
        if (Vida <= 0f)
            Kill();
    }
    
    private void Kill()
    {
        if (IsDead) return;
        IsDead = true;
            
        // Detener todos los sonidos del tanque
        Audio?.StopAll();
        Audio?.Dispose();
            
        foreach(var body in BodyHandles)
        {
            Simulation.Bodies.Remove(body);
            CollisionHandler.HandleToTank.Remove(body);
        }
    }
    
    public void Update(GameTime gameTime)
    {
        if (IsDead)
            return;
            
        var dt = (float)gameTime.ElapsedGameTime.TotalSeconds;

        var body = Simulation.Bodies.GetBodyReference(Body);
        body.Awake = true;
            
        SteerRotation = Math.Clamp(SteerRotation, MinSteer, MaxSteer);

        // Actualizar sonido del motor basado en la velocidad
        var velocity = body.Velocity.Linear;
        var speed = MathF.Sqrt(velocity.X * velocity.X + velocity.Z * velocity.Z);
        Audio?.UpdateEngine(speed, dt);

        // Girar ruedas según distancia recorrida
        UpdateWheelSpinByDistance();
            
        UpdateCanonAndTurretTowards();
            
        UpdateWorldMatrix();
    }
    
    public void UpdateAI(Vector3 playerPosition, TankController tankController)
    {
        if (IsDead) return;

        // Calculate distance to player
        var direction = playerPosition - Position;
        var distance = direction.Length();

        // Always aim at player
        if (distance > 0.001f)
        {
            direction.Normalize();
            var aimDirection = new System.Numerics.Vector3(direction.X, 0, direction.Z);
            AimDirectionWorld = aimDirection;
        }

        // Simple approach: Only move if far enough, and calculate proper turning
        if (distance > 8f) // Reduced from 15f to 8f - much closer approach
        {
            // Calculate the angle we need to turn to face the player
            var directionToPlayer = new Vector3(direction.X, 0, direction.Z);
            directionToPlayer = Vector3.Normalize(directionToPlayer);

            var targetYaw = (float)Math.Atan2(directionToPlayer.X, directionToPlayer.Z);

            // Get current tank rotation from physics body
            var body = Simulation.Bodies.GetBodyReference(Body);
            var currentRotation = body.Pose.Orientation;
            var currentYaw = (float)Math.Atan2(
                2 * (currentRotation.W * currentRotation.Y + currentRotation.X * currentRotation.Z),
                1 - 2 * (currentRotation.Y * currentRotation.Y + currentRotation.Z * currentRotation.Z)
            );

            // Calculate angle difference
            var angleDiff = targetYaw - currentYaw;
            while (angleDiff > Math.PI) angleDiff -= 2 * (float)Math.PI;
            while (angleDiff < -Math.PI) angleDiff += 2 * (float)Math.PI;

            // Tank movement logic similar to player controls
            float leftSpeed, rightSpeed;

            if (Math.Abs(angleDiff) > 0.3f) // Need to turn significantly
            {
                // Turn towards the player (inverted controls)
                if (angleDiff > 0) // Turn left
                {
                    leftSpeed = 0.7f;   // Forward left track
                    rightSpeed = -0.7f; // Reverse right track
                }
                else // Turn right
                {
                    leftSpeed = -0.7f;  // Reverse left track
                    rightSpeed = 0.7f;  // Forward right track
                }
            }
            else // Go forward (roughly facing player)
            {
                leftSpeed = 1.0f;  // Full speed forward
                rightSpeed = 1.0f;
            }

            tankController.FactorVelocidadIzquierda = leftSpeed;
            tankController.FactorVelocidadDerecha = rightSpeed;
            tankController.Turbo = false;
            tankController.BrakeLeft = false;
            tankController.BrakeRight = false;
            tankController.UpdateMovementAndAim(Simulation, directionToPlayer);
        }
        else
        {
            // Stop completely
            var directionToPlayer = new Vector3(direction.X, 0, direction.Z);
            if (directionToPlayer.Length() > 0.001f)
                directionToPlayer = Vector3.Normalize(directionToPlayer);

            tankController.FactorVelocidadIzquierda = 0f;
            tankController.FactorVelocidadDerecha = 0f;
            tankController.Turbo = false;
            tankController.BrakeLeft = true;
            tankController.BrakeRight = true;
            tankController.UpdateMovementAndAim(Simulation, directionToPlayer);
        }
    }
}