using System;
using System.Collections.Generic;
using System.Linq;
using BepuPhysics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

/// <summary>
/// Enemy tanks with AI - chase player and attack when in range
/// </summary>
public class EnemyTanks(Terrain terrain, Simulation simulation) : ModelGroup(Colors, terrain, simulation)
{
    private static readonly List<Color> Colors = new List<Color>
    {
        new Color(150, 50, 50), // Dark red for enemy tanks
    };
    
    // AI properties
    private Tank _playerTank;
    private const float DetectionRange = 300f; // Distance to start chasing player
    private const float ChaseSpeed = 15f; // Units per second when chasing
    private const float RotationSpeed = 1.5f; // Radians per second rotation speed
    
    /// <summary>
    /// Sets the player tank reference for AI targeting
    /// </summary>
    public void SetPlayerTank(Tank playerTank)
    {
        _playerTank = playerTank;
    }
    
    public void CrearObjetos()
    {
        var parametros = new[]
        { 
            // (altura, escalaMin, escalaMax)
            (0f, 1.0f, 1.0f), // Enemy tanks - same size as player
        };

        base.CrearObjetos(parametros);
        
        var parametrosRigidBodies = new[]
        { 
            // (ancho, alto, profundidad, yawEnGrados)
            (3f, 2f, 5f, 0f), // Tank dimensions
        };
        
        CrearRigidBodies(parametrosRigidBodies);
    }

    public void CargarModelos(Effect efecto, ContentManager content)
    {
        var paths = new string[]
        {
            "panzer", // Use panzer.xnb model from content/models/panzer/
        };
        
        base.CargarModelos(efecto, content, paths);
    }
    
    /// <summary>
    /// Update AI behavior - chase player if in range
    /// </summary>
    public void Update(GameTime gameTime)
    {
        if (_playerTank == null || Models == null) return;
        
        var deltaTime = (float)gameTime.ElapsedGameTime.TotalSeconds;
        var playerPosition = _playerTank.Position;
        
        // Update AI for each model group (though we typically have only one for enemy tanks)
        foreach (var modelInstance in Models)
        {
            var handles = modelInstance.Handles;
            
            for (int i = 0; i < handles.Count; i++)
            {
                var bodyHandle = handles[i];
                
                if (!Simulation.Statics.StaticExists(bodyHandle)) continue;
                
                var staticReference = Simulation.Statics[bodyHandle];
                var enemyPosition = new Vector3(
                    staticReference.Pose.Position.X,
                    staticReference.Pose.Position.Y,
                    staticReference.Pose.Position.Z
                );
                
                // Calculate distance to player
                var direction = playerPosition - enemyPosition;
                var distance = direction.Length();
                
                // Only chase if player is within detection range
                if (distance <= DetectionRange && distance > 5f) // Don't move if too close
                {
                    // Normalize direction for movement
                    direction.Normalize();
                    
                    // Calculate target rotation to face player
                    var targetYaw = (float)Math.Atan2(direction.X, direction.Z);
                    
                    // Get current rotation
                    var currentRotation = staticReference.Pose.Orientation;
                    var currentYaw = (float)Math.Atan2(
                        2 * (currentRotation.W * currentRotation.Y + currentRotation.X * currentRotation.Z),
                        1 - 2 * (currentRotation.Y * currentRotation.Y + currentRotation.Z * currentRotation.Z)
                    );
                    
                    // Smooth rotation towards target
                    var yawDifference = targetYaw - currentYaw;
                    
                    // Handle angle wrapping
                    while (yawDifference > Math.PI) yawDifference -= 2f * (float)Math.PI;
                    while (yawDifference < -Math.PI) yawDifference += 2f * (float)Math.PI;
                    
                    // Apply rotation with speed limit
                    var rotationAmount = Math.Sign(yawDifference) * Math.Min(Math.Abs(yawDifference), RotationSpeed * deltaTime);
                    var newYaw = currentYaw + rotationAmount;
                    
                    // Move towards player
                    var movement = direction * ChaseSpeed * deltaTime;
                    var newPosition = enemyPosition + movement;
                    
                    // Get terrain height at new position
                    var terrainHeight = Terrain.GetHeightAtPosition(newPosition.X, newPosition.Z);
                    newPosition.Y = terrainHeight + 1f; // Slightly above ground
                    
                    // Update physics body position and rotation
                    var newPose = new RigidPose(
                        new System.Numerics.Vector3(newPosition.X, newPosition.Y, newPosition.Z),
                        System.Numerics.Quaternion.CreateFromYawPitchRoll(newYaw, 0f, 0f)
                    );
                    
                    staticReference.Pose = newPose;
                }
            }
        }
    }
}