using BepuPhysics;
using BepuPhysics.Collidables;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using TGC.MonoGame.Samples.Viewer.Gizmos;

namespace TGC.MonoGame.TP;

public class Debug
{
    public Effect DebugEffect { get; private set; }
    private SpriteBatch _spriteBatch;
    private SpriteFont _debugFont;
    private Tank _tank;
    private bool _showTerrainMeshDebug = false;
    private bool _showTankTelemetry = false;
    private Camera _camera;
    private GraphicsDevice _graphicsDevice;
    private Terrain _terrain;
    private Simulation _simulation;
    
    public Gizmos Gizmos { get; set;}
    
    private Vector3 _debugBoxSize;
    private VertexBuffer _debugBoxVB;
    private IndexBuffer _debugBoxIB;
    private bool _debugBuffersReady;
    
    // Debug mesh (físico) del terreno
    private VertexBuffer _physDbgVB;
    private IndexBuffer _physDbgIB;
    private int _physDbgIndexCount;
    private bool _physDbgReady;

    public void LoadContent(ContentManager content, string contentEffectsFolder, string contentSpriteFolder,
        GraphicsDevice graphicsDevice, Tank tank, OrbitCamera orbitCamera, Simulation simulation, Terrain terrain,
        Gizmos gizmos)
    {
        _tank = tank;
        _camera = orbitCamera;
        _simulation = simulation;
        _terrain = terrain;
        _graphicsDevice = graphicsDevice;
        Gizmos = gizmos;
        DebugEffect = content.Load<Effect>(contentEffectsFolder + "Debug");
        DebugEffect.Parameters["DebugColor"]?.SetValue(Color.Red.ToVector4());

        _spriteBatch = new SpriteBatch(graphicsDevice);
        _debugFont = content.Load<SpriteFont>(contentSpriteFolder + "CascadiaCode/CascadiaCodePL");
    }

    public void Update(KeyboardState keyboardState, KeyboardState kbPrev, float dt, Camera camera)
    {
        if (keyboardState.IsKeyDown(Keys.F2) && !kbPrev.IsKeyDown(Keys.F2))
        {
            _showTerrainMeshDebug = !_showTerrainMeshDebug;
        }

        if (keyboardState.IsKeyDown(Keys.F3) && !kbPrev.IsKeyDown(Keys.F3))
        {
            _showTankTelemetry = !_showTankTelemetry;
            _tank.DebugTelemetry = _showTankTelemetry;
        }

        _camera = camera;
    }

    public void Draw()
    {
        if (_showTerrainMeshDebug)
        {
            DebugEffect.Parameters["View"].SetValue(_camera.View);
            DebugEffect.Parameters["Projection"].SetValue(_camera.Projection);
            DebugEffect.Parameters["World"].SetValue(Matrix.Identity);

            var oldRS = _graphicsDevice.RasterizerState;
            _graphicsDevice.RasterizerState = new RasterizerState
                { CullMode = CullMode.None, FillMode = FillMode.WireFrame };

            DrawCollider(_graphicsDevice, _camera.View, _camera.Projection, DebugEffect, wireframe: false);
            
            _graphicsDevice.RasterizerState = oldRS;

            // --- DEBUG: ver el mesh físico del terreno ---
            //_debugEffect.Parameters["DebugColor"]?.SetValue(Color.Yellow.ToVector4()); // si tu .fx lo usa
            DrawPhysicsMeshDebug(DebugEffect, _camera.View, _camera.Projection);

            // Mostrar TODAS las cajas del simulador (dinámicas + estáticas)
            DebugEffect.Parameters["View"].SetValue(_camera.View);
            DebugEffect.Parameters["Projection"].SetValue(_camera.Projection);

            var oldRS2 = _graphicsDevice.RasterizerState;
            _graphicsDevice.RasterizerState = new RasterizerState
            {
                CullMode = CullMode.None,
                FillMode = FillMode.WireFrame
            };

            // -------- ESTÁTICOS (terreno) --------
            for (int i = 0; i < _simulation.Statics.Count; i++)
            {
                var handle = new BepuPhysics.StaticHandle(i);

                BepuPhysics.StaticDescription desc;
                _simulation.Statics.GetDescription(handle, out desc);

                if (desc.Shape.Type == BepuPhysics.Collidables.Box.Id)
                {
                    var shape = _simulation.Shapes.GetShape<BepuPhysics.Collidables.Box>(desc.Shape.Index);

                    var worldMatrix =
                        Matrix.CreateScale(shape.Width, shape.Height, shape.Length) *
                        Matrix.CreateFromQuaternion(new Microsoft.Xna.Framework.Quaternion(
                            desc.Pose.Orientation.X,
                            desc.Pose.Orientation.Y,
                            desc.Pose.Orientation.Z,
                            desc.Pose.Orientation.W)) *
                        Matrix.CreateTranslation(desc.Pose.Position.X, desc.Pose.Position.Y, desc.Pose.Position.Z);

                    DebugEffect.Parameters["World"].SetValue(worldMatrix);

                    foreach (var pass in DebugEffect.CurrentTechnique.Passes)
                    {
                        pass.Apply();
                        DebugPrimitiveRenderer.DrawCube(_graphicsDevice);
                    }
                }
            }

            // -------- DINÁMICOS (tanque, etc.) --------
            var activeSet = _simulation.Bodies.ActiveSet;

            for (int i = 0; i < activeSet.Count; i++)
            {
                var handle = activeSet.IndexToHandle[i];
                var bodyRef = _simulation.Bodies.GetBodyReference(handle);

                var shapeIndex = bodyRef.Collidable.Shape.Index;
                if (shapeIndex < 0)
                    continue;

                var shape = _simulation.Shapes.GetShape<BepuPhysics.Collidables.Box>(shapeIndex);

                var worldMatrix =
                    Matrix.CreateScale(shape.Width, shape.Height, shape.Length) *
                    Matrix.CreateFromQuaternion(new Microsoft.Xna.Framework.Quaternion(
                        bodyRef.Pose.Orientation.X,
                        bodyRef.Pose.Orientation.Y,
                        bodyRef.Pose.Orientation.Z,
                        bodyRef.Pose.Orientation.W)) *
                    Matrix.CreateTranslation(bodyRef.Pose.Position.X, bodyRef.Pose.Position.Y, bodyRef.Pose.Position.Z);

                DebugEffect.Parameters["World"].SetValue(worldMatrix);

                foreach (var pass in DebugEffect.CurrentTechnique.Passes)
                {
                    pass.Apply();
                    DebugPrimitiveRenderer.DrawCube(_graphicsDevice);
                }
            }

            _graphicsDevice.RasterizerState = oldRS2;
        }

        if (_showTankTelemetry && _debugFont != null)
        {
            _spriteBatch.Begin();
            _spriteBatch.DrawString(_debugFont, _tank.TelemetryText ?? "", new Vector2(14, 14), Color.LimeGreen);
            _spriteBatch.End();
        }
    }
    
    private void EnsureDebugCube(GraphicsDevice gd)
        {
            if (_debugBuffersReady) return;

            // Un cubo unitario centrado en el origen (vértices -0.5..+0.5)
            var v = new[]
            {
                new VertexPositionColor(new Microsoft.Xna.Framework.Vector3(-0.5f, -0.5f, -0.5f), Color.Red),
                new VertexPositionColor(new Microsoft.Xna.Framework.Vector3(0.5f, -0.5f, -0.5f), Color.Red),
                new VertexPositionColor(new Microsoft.Xna.Framework.Vector3(0.5f, 0.5f, -0.5f), Color.Red),
                new VertexPositionColor(new Microsoft.Xna.Framework.Vector3(-0.5f, 0.5f, -0.5f), Color.Red),
                new VertexPositionColor(new Microsoft.Xna.Framework.Vector3(-0.5f, -0.5f, 0.5f), Color.Red),
                new VertexPositionColor(new Microsoft.Xna.Framework.Vector3(0.5f, -0.5f, 0.5f), Color.Red),
                new VertexPositionColor(new Microsoft.Xna.Framework.Vector3(0.5f, 0.5f, 0.5f), Color.Red),
                new VertexPositionColor(new Microsoft.Xna.Framework.Vector3(-0.5f, 0.5f, 0.5f), Color.Red),
            };

            var idx = new ushort[]
            {
                // caras (triángulos) – sólido
                0, 1, 2, 0, 2, 3, // z-
                4, 6, 5, 4, 7, 6, // z+
                0, 4, 5, 0, 5, 1, // y-
                3, 2, 6, 3, 6, 7, // y+
                0, 3, 7, 0, 7, 4, // x-
                1, 5, 6, 1, 6, 2 // x+
            };

            _debugBoxVB = new VertexBuffer(gd, VertexPositionColor.VertexDeclaration, v.Length, BufferUsage.WriteOnly);
            _debugBoxVB.SetData(v);
            _debugBoxIB = new IndexBuffer(gd, IndexElementSize.SixteenBits, idx.Length, BufferUsage.WriteOnly);
            _debugBoxIB.SetData(idx);

            _debugBuffersReady = true;
        }
        
        public void DrawCollider(GraphicsDevice gd, Matrix view, Matrix projection, Effect effect, bool wireframe = false)
        {
            if (_simulation == null || _tank.PhysicsBody.Value < 0 || _tank.bodyShapeCompound.Children.Length == 0) return;
            
            var body = _simulation.Bodies.GetBodyReference(_tank.PhysicsBody);
            var bodyPose = body.Pose;

            for (int i = 0; i < _tank.bodyShapeCompound.Children.Length; i++)
            {
                var child = _tank.bodyShapeCompound.Children[i];
                var localPose = child.LocalPose;

                // Transformar posición local al mundo
                var localPos = localPose.Position;
                var worldPos = bodyPose.Position + System.Numerics.Vector3.Transform(localPos, bodyPose.Orientation);

                // Combinar orientaciones
                var worldOrientation = System.Numerics.Quaternion.Concatenate(localPose.Orientation, bodyPose.Orientation);

                // Convertir a MonoGame
                var posMG = new Microsoft.Xna.Framework.Vector3(worldPos.X, worldPos.Y, worldPos.Z);
                var rotMG = new Microsoft.Xna.Framework.Quaternion(worldOrientation.X, worldOrientation.Y, worldOrientation.Z, worldOrientation.W);

                // Obtener el tipo de shape
                var shapeIndex = child.ShapeIndex;
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
                        Matrix.CreateFromQuaternion(rotMG) *
                        Matrix.CreateTranslation(posMG);

                    Gizmos.DrawCube(worldMatrix, Color.Red);
                }
                else if (typeId == default(Cylinder).TypeId)
                {
                    var cylinder = _simulation.Shapes.GetShape<Cylinder>(shapeIndex.Index);
                    float height = cylinder.HalfLength;

                    var worldMatrix =
                        Matrix.CreateScale(cylinder.Radius, height, cylinder.Radius) *
                        Matrix.CreateFromQuaternion(rotMG) *
                        Matrix.CreateTranslation(posMG);

                    Gizmos.DrawCylinder(worldMatrix, Color.Orange);
                }
            }
            Gizmos.Draw();
        }
        
        private void EnsurePhysicsDebugBuffers()
{
    if (_physDbgReady) return;

    int width  = _terrain.HeightmapData.GetLength(0);
    int length = _terrain.HeightmapData.GetLength(1);

    // 1) VERTS (grid compartido)
    var verts = new VertexPositionColor[width * length];
    int vi = 0;
    for (int z = 0; z < length; z++)
    {
        for (int x = 0; x < width; x++)
        {
            float h = _terrain.HeightmapData[x, z] * _terrain._scaleY;
            var p = new Microsoft.Xna.Framework.Vector3(
                _terrain.Center.X + x * _terrain._scaleXZ,
                _terrain.Center.Y + h,
                _terrain.Center.Z + z * _terrain._scaleXZ
            );
            verts[vi++] = new VertexPositionColor(p, Color.Yellow);
        }
    }

    // 2) ÍNDICES (2 triángulos por celda)
    int quadsX = width  - 1;
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
                int i0 =  z      * width + x;
                int i1 =  z      * width + (x + 1);
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
        _physDbgVB = new VertexBuffer(gd, VertexPositionColor.VertexDeclaration, verts.Length, BufferUsage.WriteOnly);
        _physDbgVB.SetData(verts);

        _physDbgIB = new IndexBuffer(gd, IndexElementSize.SixteenBits, idx.Length, BufferUsage.WriteOnly);
        _physDbgIB.SetData(idx);

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
                int i0 =  z      * width + x;
                int i1 =  z      * width + (x + 1);
                int i2 = (z + 1) * width + x;
                int i3 = (z + 1) * width + (x + 1);

                // t0: i0, i2, i1
                idx[k++] = i0; idx[k++] = i2; idx[k++] = i1;
                // t1: i1, i2, i3
                idx[k++] = i1; idx[k++] = i2; idx[k++] = i3;
            }
        }

        var gd = DebugEffect.GraphicsDevice;
        _physDbgVB = new VertexBuffer(gd, VertexPositionColor.VertexDeclaration, verts.Length, BufferUsage.WriteOnly);
        _physDbgVB.SetData(verts);

        _physDbgIB = new IndexBuffer(gd, IndexElementSize.ThirtyTwoBits, idx.Length, BufferUsage.WriteOnly);
        _physDbgIB.SetData(idx);

        _physDbgIndexCount = idx.Length;
    }

    _physDbgReady = true;
}

    public void DrawPhysicsMeshDebug(Effect debugEffect, Matrix view, Matrix projection)
    {
        EnsurePhysicsDebugBuffers();

        var gd = DebugEffect.GraphicsDevice;

        debugEffect.Parameters["View"]?.SetValue(view);
        debugEffect.Parameters["Projection"]?.SetValue(projection);
        debugEffect.Parameters["World"]?.SetValue(Matrix.Identity);

        var old = gd.RasterizerState;
        gd.RasterizerState = new RasterizerState { CullMode = CullMode.None, FillMode = FillMode.WireFrame };

        gd.SetVertexBuffer(_physDbgVB);
        gd.Indices = _physDbgIB;

        foreach (var pass in debugEffect.CurrentTechnique.Passes)
        {
            pass.Apply();
            gd.DrawIndexedPrimitives(PrimitiveType.TriangleList, 0, 0, _physDbgIndexCount / 3);
        }

        gd.RasterizerState = old;
    }

}