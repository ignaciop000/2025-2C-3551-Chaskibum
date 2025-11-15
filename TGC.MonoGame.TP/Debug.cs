using System;
using System.Collections.Generic;
using BepuPhysics;
using BepuPhysics.Collidables;
using ImGuiNET;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using TGC.MonoGame.Samples.Collisions;
using TGC.MonoGame.Samples.Viewer.GUI.ImGuiNET;
using TGC.MonoGame.TP.Viewer.Gizmos;

namespace TGC.MonoGame.TP;

public class Debug
{
    private Effect DebugEffect { get; set; }
    //private SpriteBatch _spriteBatch;
    //private SpriteFont _debugFont;
    private List<Tank> _tanks;
    private List<Projectile> _projectiles;
    public List<Tank> Tanks;
    private bool _showTerrainMeshDebug;
    //private bool _showTankTelemetry = false;
    private GraphicsDevice _graphicsDevice;
    private Terrain _terrain;
    private Simulation _simulation;
    private SpriteBatch _spriteBatch;
    
    private BoundingFrustum _boundingFrustum;
    private Matrix View;
    private Matrix Projection;
    private Gizmos Gizmos { get; set; }

    //private Vector3 _debugBoxSize;
    //private VertexBuffer _debugBoxVB;
    //private IndexBuffer _debugBoxIB;
    //private bool _debugBuffersReady;

    // Debug mesh (físico) del terreno
    private VertexBuffer _physDbgVb;
    private IndexBuffer _physDbgIb;
    private int _physDbgIndexCount;
    private bool _physDbgReady;

    public void LoadContent(ContentManager content, string contentEffectsFolder,
        GraphicsDevice graphicsDevice, OrbitCamera orbitCamera, Simulation simulation, Terrain terrain,
    public void LoadContent(ContentManager content, string contentEffectsFolder, string contentSpriteFolder,
        GraphicsDevice graphicsDevice, OrbitCamera orbitCamera, Simulation simulation, Terrain terrain,
        Gizmos gizmos)
    {
        
        _tanks = new List<Tank>();
        _projectiles = new List<Projectile>();
        _simulation = simulation;
        _terrain = terrain;
        _graphicsDevice = graphicsDevice;
        Gizmos = gizmos;
        DebugEffect = content.Load<Effect>(contentEffectsFolder + "Debug");
        DebugEffect.Parameters["DebugColor"]?.SetValue(Color.Red.ToVector4());
        _boundingFrustum = new BoundingFrustum(orbitCamera.View * orbitCamera.Projection);
        View = orbitCamera.View;
        Projection = orbitCamera.Projection;
        _spriteBatch = new SpriteBatch(_graphicsDevice);
        //_spriteBatch = new SpriteBatch(graphicsDevice);
        //_debugFont = content.Load<SpriteFont>(contentSpriteFolder + "CascadiaCode/CascadiaCodePL");
    }

    public void Update(KeyboardState keyboardState, KeyboardState kbPrev, float dt, OrbitCamera camera)
    {
        if (keyboardState.IsKeyDown(Keys.F2) && !kbPrev.IsKeyDown(Keys.F2))
        {
            _showTerrainMeshDebug = !_showTerrainMeshDebug;
        }
        
        _boundingFrustum= new BoundingFrustum(camera.View * camera.Projection) ;
    }

    public void Draw(Camera camera, OrbitCamera orbitCamera, TargetCamera targetLightCamera, Gizmos gizmos, GameTime gameTime, Terrain terrain)
    {
        if (_showTerrainMeshDebug)
        {
            DebugEffect.Parameters["View"]?.SetValue(camera.View);
            DebugEffect.Parameters["Projection"]?.SetValue(camera.Projection);
            DebugEffect.Parameters["World"]?.SetValue(Matrix.Identity);

            var oldRS = _graphicsDevice.RasterizerState;
            _graphicsDevice.RasterizerState = new RasterizerState
                { CullMode = CullMode.None, FillMode = FillMode.WireFrame };

            DrawCollider();

            DrawChunks(terrain);
            _graphicsDevice.RasterizerState = oldRS;

            // --- DEBUG: ver el mesh físico del terreno ---
            //_debugEffect.Parameters["DebugColor"]?.SetValue(Color.Yellow.ToVector4()); 
            //DrawPhysicsMeshDebug(DebugEffect, camera.View, camera.Projection);

            // Mostrar TODAS las cajas del simulador (dinámicas + estáticas)
            DebugEffect.Parameters["View"]?.SetValue(camera.View);
            DebugEffect.Parameters["Projection"]?.SetValue(camera.Projection);

            var oldRS2 = _graphicsDevice.RasterizerState;
            _graphicsDevice.RasterizerState = new RasterizerState
            {
                CullMode = CullMode.None,
                FillMode = FillMode.WireFrame
            };

            // -------- ESTÁTICOS (terreno) --------
            for (int i = 0; i < _simulation.Statics.Count; i++)
            {
                var handle = new StaticHandle(i);

                _simulation.Statics.GetDescription(handle, out var desc);

                if (desc.Shape.Type == Box.Id)
                {
                    var shape = _simulation.Shapes.GetShape<Box>(desc.Shape.Index);
                    
                    var transformation = Matrix.CreateFromQuaternion(new Quaternion(
                                            desc.Pose.Orientation.X,
                                            desc.Pose.Orientation.Y,
                                            desc.Pose.Orientation.Z,
                                            desc.Pose.Orientation.W)) *
                                             Matrix.CreateTranslation(desc.Pose.Position.X, desc.Pose.Position.Y, desc.Pose.Position.Z);

                    var worldMatrix =
                        Matrix.CreateScale(shape.Width, shape.Height, shape.Length) *
                        transformation;
                        
                    DebugEffect.Parameters["World"]?.SetValue(worldMatrix);
                    
                    var halfExtents = new Vector3(shape.Width / 2f, shape.Height / 2f, shape.Length / 2f);
                    var localBox = new BoundingBox(-halfExtents, halfExtents);
                    var transformedCorners = new Vector3[8];
                    
                    var corners = localBox.GetCorners();
                    for (int j = 0; j < 8; j++)
                    {
                        transformedCorners[j] = Vector3.Transform(corners[j], transformation);
                    }
                    
                    var worldBox = BoundingBox.CreateFromPoints(transformedCorners);
                    
                    if (_boundingFrustum.Intersects(worldBox))
                    {
                        foreach (var pass in DebugEffect.CurrentTechnique.Passes)
                        {
                            pass.Apply();
                            DebugPrimitiveRenderer.DrawCube(_graphicsDevice);
                        }
                    }
                }
            }
            gizmos.DrawFrustum(orbitCamera.View * orbitCamera.Projection, Color.Yellow);
            gizmos.DrawFrustum(targetLightCamera.View * targetLightCamera.Projection, Color.Black);
            gizmos.Draw();
            
            _graphicsDevice.RasterizerState = oldRS2;
        }
    }

    private void DrawCollider()
    {
        foreach (var tank in Tanks)
        {
            if (tank.IsDead) continue;
            tank.DrawDebug();
            foreach (var bodyHandle in tank.BodyHandles)
            {
                if(!_simulation.Bodies.BodyExists(bodyHandle))
                    continue;
                
                if (_simulation == null || bodyHandle.Value < 0) return;

                if(!_simulation.Bodies.BodyExists(bodyHandle)) continue;
                
                var body = _simulation.Bodies.GetBodyReference(bodyHandle);
                
                var bodyPose = body.Pose;

                // Convertir a MonoGame
                var posMg = new Vector3(bodyPose.Position.X, bodyPose.Position.Y, bodyPose.Position.Z);
                var rotMg = new Quaternion(bodyPose.Orientation.X, bodyPose.Orientation.Y, bodyPose.Orientation.Z,
                    bodyPose.Orientation.W);

                // Obtener el tipo de shape
                var shapeIndex = body.Collidable.Shape;
                int typeId = shapeIndex.Type;

                if (typeId == default(Box).TypeId)
                {
                    var box = _simulation.Shapes.GetShape<Box>(shapeIndex.Index);
                    var size = new Vector3(
                        box.HalfWidth * 2f,
                        box.HalfHeight * 2f,
                        box.HalfLength * 2f
                    );

                    var worldMatrix =
                        Matrix.CreateScale(size) *
                        Matrix.CreateFromQuaternion(rotMg) *
                        Matrix.CreateTranslation(posMg);

                    Gizmos.DrawCube(worldMatrix, Color.Yellow);
                }
                else if (typeId == default(Cylinder).TypeId)
                {
                    var cylinder = _simulation.Shapes.GetShape<Cylinder>(shapeIndex.Index);
                    float height = cylinder.HalfLength;

                    var worldMatrix =
                        Matrix.CreateScale(cylinder.Radius, height, cylinder.Radius) *
                        Matrix.CreateFromQuaternion(rotMg) *
                        Matrix.CreateTranslation(posMg);

                    Gizmos.DrawCylinder(worldMatrix, Color.Orange);
                }
            }
        }
        
        if (_projectiles != null)
        {
            foreach (var projectile in _projectiles)
            {
                if (projectile.IsDead) continue;

                var bodyHandleProyectile = projectile.Body;
                if (_simulation == null || bodyHandleProyectile.Value < 0) continue;

                var bodyProyectile = _simulation.Bodies.GetBodyReference(bodyHandleProyectile);
                var bodyPoseProyectile = bodyProyectile.Pose;
                var posProyectile = new Vector3(bodyPoseProyectile.Position.X, bodyPoseProyectile.Position.Y, bodyPoseProyectile.Position.Z);
                
                var shapeIndex = bodyProyectile.Collidable.Shape;
                if (shapeIndex.Type == default(Sphere).TypeId)
                {
                    var sphere = _simulation.Shapes.GetShape<Sphere>(shapeIndex.Index);
                    var worldMatrix = Matrix.CreateScale(sphere.Radius * 2f) * Matrix.CreateTranslation(posProyectile);
                    Gizmos.DrawCube(worldMatrix, Color.Red);
                }
            }
        }


        Gizmos.Draw();
    }

    private void EnsurePhysicsDebugBuffers()
    {
        if (_physDbgReady) return;

        int width = _terrain.HeightmapData.GetLength(0);
        int length = _terrain.HeightmapData.GetLength(1);

        // 1) VERTS (grid compartido)
        var verts = new VertexPositionColor[width * length];
        int vi = 0;
        for (int z = 0; z < length; z++)
        {
            for (int x = 0; x < width; x++)
            {
                float h = _terrain.HeightmapData[x, z] * _terrain.ScaleY;
                var p = new Vector3(
                    _terrain.Center.X + x * _terrain.ScaleXz,
                    _terrain.Center.Y + h,
                    _terrain.Center.Z + z * _terrain.ScaleXz
                );
                verts[vi++] = new VertexPositionColor(p, Color.Yellow);
            }
        }

        // 2) ÍNDICES (2 triángulos por celda)
        int quadsX = width - 1;
        int quadsZ = length - 1;
        int triCount = quadsX * quadsZ * 2;
        int indexCount = triCount * 3;

        bool need32 = (width * length) > 65535;
        if (!need32)
        {
            var idx = new ushort[indexCount];
            int k = 0;
            for (int z = 0; z < quadsZ; z++)
            {
                for (int x = 0; x < quadsX; x++)
                {
                    int i0 = z * width + x;
                    int i1 = z * width + (x + 1);
                    int i2 = (z + 1) * width + x;
                    int i3 = (z + 1) * width + (x + 1);

                    // t0: i0, i2, i1
                    idx[k++] = (ushort)i0;
                    idx[k++] = (ushort)i2;
                    idx[k++] = (ushort)i1;
                    // t1: i1, i2, i3
                    idx[k++] = (ushort)i1;
                    idx[k++] = (ushort)i2;
                    idx[k++] = (ushort)i3;
                }
            }

            var gd = DebugEffect.GraphicsDevice;
            _physDbgVb = new VertexBuffer(gd, VertexPositionColor.VertexDeclaration, verts.Length,
                BufferUsage.WriteOnly);
            _physDbgVb.SetData(verts);

            _physDbgIb = new IndexBuffer(gd, IndexElementSize.SixteenBits, idx.Length, BufferUsage.WriteOnly);
            _physDbgIb.SetData(idx);

            _physDbgIndexCount = idx.Length;
        }
        else
        {
            var idx = new int[indexCount];
            int k = 0;
            for (int z = 0; z < quadsZ; z++)
            {
                for (int x = 0; x < quadsX; x++)
                {
                    int i0 = z * width + x;
                    int i1 = z * width + (x + 1);
                    int i2 = (z + 1) * width + x;
                    int i3 = (z + 1) * width + (x + 1);

                    // t0: i0, i2, i1
                    idx[k++] = i0;
                    idx[k++] = i2;
                    idx[k++] = i1;
                    // t1: i1, i2, i3
                    idx[k++] = i1;
                    idx[k++] = i2;
                    idx[k++] = i3;
                }
            }

            var gd = DebugEffect.GraphicsDevice;
            _physDbgVb = new VertexBuffer(gd, VertexPositionColor.VertexDeclaration, verts.Length,
                BufferUsage.WriteOnly);
            _physDbgVb.SetData(verts);

            _physDbgIb = new IndexBuffer(gd, IndexElementSize.ThirtyTwoBits, idx.Length, BufferUsage.WriteOnly);
            _physDbgIb.SetData(idx);

            _physDbgIndexCount = idx.Length;
        }

        _physDbgReady = true;
    }

    private void DrawPhysicsMeshDebug(Effect debugEffect, Matrix view, Matrix projection)
    {
        EnsurePhysicsDebugBuffers();

        var gd = DebugEffect.GraphicsDevice;

        debugEffect.Parameters["View"]?.SetValue(view);
        debugEffect.Parameters["Projection"]?.SetValue(projection);
        debugEffect.Parameters["World"]?.SetValue(Matrix.Identity);

        var old = gd.RasterizerState;
        gd.RasterizerState = new RasterizerState { CullMode = CullMode.None, FillMode = FillMode.WireFrame };

        gd.SetVertexBuffer(_physDbgVb);
        gd.Indices = _physDbgIb;

        foreach (var pass in debugEffect.CurrentTechnique.Passes)
        {
            pass.Apply();
            gd.DrawIndexedPrimitives(PrimitiveType.TriangleList, 0, 0, _physDbgIndexCount / 3);
        }

        gd.RasterizerState = old;
    }

    public void actualizarTanks(List<Tank> tanks)
    {
        _tanks = tanks;
    }

    private void DrawChunks(Terrain terrain)
    {
        foreach (var chunk in terrain.Chunks)
        {
            Vector3 origin = (chunk.BoundingBox.Min + chunk.BoundingBox.Max) / 2f;
            Vector3 size = chunk.BoundingBox.Max - chunk.BoundingBox.Min;
            Gizmos.DrawCube(origin, size);
        }
    }

    public void actualizarProyectiles(List<Projectile> projectiles)
    {
        _projectiles = projectiles;
    }
}