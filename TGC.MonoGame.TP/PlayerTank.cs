using System;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;

namespace TGC.MonoGame.TP;

public class PlayerTank(Vector3 initialPosition, float initialRotation = 0f, float scale = 0.1f) : Tank(initialPosition, initialRotation, scale)
{
    private const float SteerSpeed = 90f;
    
    public void Update(GameTime gameTime, KeyboardState keyboardState)
    {
        if (IsDead) return;
        var dt = (float)gameTime.ElapsedGameTime.TotalSeconds;

        var body = Simulation.Bodies.GetBodyReference(Body);
        body.Awake = true;

        UpdateProjectileConfig(keyboardState);
            
        if (keyboardState.IsKeyDown(Keys.A)) SteerRotation += SteerSpeed * dt;
        else if(keyboardState.IsKeyDown(Keys.D))  SteerRotation -= SteerSpeed * dt;
        else{ SteerRotation = MathHelper.Lerp(SteerRotation, 0f, dt * 5f); }
            
        SteerRotation = Math.Clamp(SteerRotation, MinSteer, MaxSteer);
            
        FireCooldown = MathF.Max(0f, FireCooldown - dt);

        // Detectar turbo (Shift)
        bool isTurbo = keyboardState.IsKeyDown(Keys.LeftShift);

        // Actualizar sonido del motor basado en la velocidad y turbo
        var velocity = body.Velocity.Linear;
        var speed = MathF.Sqrt(velocity.X * velocity.X + velocity.Z * velocity.Z);
        Audio?.UpdateEngine(speed, dt, isTurbo);

        // Detectar freno
        bool isBraking = keyboardState.IsKeyDown(Keys.Space);
        if (isBraking && !WasBraking && speed > 1f)
        {
            Audio?.PlayBrake();
        }
        WasBraking = isBraking;

        // Girar ruedas según distancia recorrida
        UpdateWheelSpinByDistance();
            
        UpdateCanonAndTurretTowards();
            
        UpdateWorldMatrix();
    }
    
    private void UpdateProjectileConfig(KeyboardState keyboardState)
    {
        if (keyboardState.IsKeyDown(Keys.D1))
        {
            if (TipoProyectilActual == ProjectileTypes.Light) return;
                
            TipoProyectilActual = ProjectileTypes.Light;
            FireCooldown = TipoProyectilActual.MaxCooldown;
                
        } else if (keyboardState.IsKeyDown(Keys.D2))
        {
            if (TipoProyectilActual == ProjectileTypes.Heavy) return;
                
            TipoProyectilActual = ProjectileTypes.Heavy;
            FireCooldown = TipoProyectilActual.MaxCooldown;
        }
    }
    
    /// <summary>
    /// Indica hacia donde hay que apuntar, en base a la posicion del mouse
    /// </summary>
    public void UpdateAim(MouseState mouseState, Camera camera, Viewport vp)
    {
        var aimDir = camera.FrontDirection; 
        var hit = PickOnTerrain(mouseState.Position, vp, camera); //Rayo para ver donde impacta en el terreno
        if (hit.HasValue)
        {
            aimDir = Vector3.Normalize(hit.Value - Position);
        }
        AimDirectionWorld = aimDir;
    }
    
    private Vector3? PickOnTerrain(Point mouse, Viewport viewport, Camera camera)
    {
        // Desarma matrices
        var view = camera.View;
        var proj = camera.Projection;

        // Dos puntos en NDC (near/far) -> espacio mundo
        var nearPoint = viewport.Unproject(new Vector3(mouse.X, mouse.Y, 0f), proj, view, Matrix.Identity);
        var farPoint  = viewport.Unproject(new Vector3(mouse.X, mouse.Y, 1f), proj, view, Matrix.Identity);

        var dir = Vector3.Normalize(farPoint - nearPoint);
        var origin = nearPoint;

        // Busco intersección por búsqueda binaria contra la altura del terreno
        var tMin = 0f;
        var tMax = 5000f; // alcance del rayo
        for (var i = 0; i < 48; i++) // precisión suficiente
        {
            var tMid = 0.5f * (tMin + tMax);
            var rayoPrueba = origin + dir * tMid;
            var alturaTerreno = Terrain.GetHeightAtPosition(rayoPrueba.X, rayoPrueba.Z);
            if (rayoPrueba.Y > alturaTerreno)
            {
                tMin = tMid;
            }
            else
            {
                tMax = tMid;
            }
        }

        var hit = origin + dir * tMax;

        // Si estamos muy lejos o fuera del mapa, descartamos
        if (float.IsNaN(hit.X)) return null;
        return hit;
    }
    
    public void Reset()
    {
        base.Reset(); // Llamar al Reset de la clase base (Tank) para limpiar impactos
        IsDead = false;
        Vida = 100f;
        WasBraking = false;
        TipoProyectilActual = ProjectileTypes.Light;
        BrakeTime = 0f;
        RecoilTime = 0f;
        ResetCooldown();
        
        VolverAlCentro();
    }
    
    private void VolverAlCentro()
    {
        // Obtener referencia al cuerpo físico BEPU
        var centerY = Terrain.GetHeightAtPosition(0, 0);
        var bodyHandle = Simulation.Bodies.GetBodyReference(Body);
        var tankPos = bodyHandle.Pose.Position;
        var offset = new System.Numerics.Vector3(0, centerY + 25, 0) - tankPos;
            
        foreach (var handle in BodyHandles)
        {
            var bodyRef = Simulation.Bodies.GetBodyReference(handle);    
            var pose = bodyRef.Pose;
            pose.Position += offset;
            pose.Orientation = System.Numerics.Quaternion.Identity;
            bodyRef.Pose = pose;
                
            var vel = bodyRef.Velocity;
            vel.Linear = System.Numerics.Vector3.Zero;
            vel.Angular = System.Numerics.Vector3.Zero;
            bodyRef.Velocity = vel;
        }
            
        UpdateWorldMatrix();
        
        LastPos = Position;
    }
    
    public override void Kill()
    {
        base.Kill();

        Texture = null;
    }

    public void Curar(int cantidad)
    {
        Vida = Math.Min(VidaMax, Vida + cantidad);
    }
}