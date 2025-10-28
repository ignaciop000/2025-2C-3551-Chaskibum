using System;
using System.Collections.Generic;
using BepuPhysics;
using BepuPhysics.Collidables;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using NumVector3 = System.Numerics.Vector3;
using NumQuaternion = System.Numerics.Quaternion;
using XnaVector3 = Microsoft.Xna.Framework.Vector3;
using XnaMatrix = Microsoft.Xna.Framework.Matrix;
using XnaQuaternion = Microsoft.Xna.Framework.Quaternion;

namespace TGC.MonoGame.TP
{
    public class WorldBorder
    {
        private readonly List<BorderPlane> _planes = new();
        private readonly GraphicsDevice _graphicsDevice;
        private readonly Effect _effect;
        private readonly float _mapWidth;
        private readonly float _mapLength;

        private readonly RasterizerState _cullNone = new RasterizerState { CullMode = CullMode.None };

        public WorldBorder(GraphicsDevice graphicsDevice, Effect effect, Simulation simulation, float mapWidth, float mapLength)
        {
            _graphicsDevice = graphicsDevice;
            _effect = effect;
            _mapWidth = mapWidth;
            _mapLength = mapLength;

            CreatePlanes(simulation);
        }

        private void CreatePlanes(Simulation sim)
        {
            float margin = 3000f;
            float x = (_mapWidth / 2f) - margin;
            float z = (_mapLength / 2f) - margin;
            float height = 2000f;

            // Frente (-Z) y Fondo (+Z) → normales ±Z (Identity)
            _planes.Add(new BorderPlane(
                new NumVector3(0f, 0f, -z),
                NumQuaternion.Identity,
                _mapWidth, height,
                sim, _graphicsDevice, _effect));

            _planes.Add(new BorderPlane(
                new NumVector3(0f, 0f, z),
                NumQuaternion.Identity,
                _mapWidth, height,
                sim, _graphicsDevice, _effect));

            // Izquierda (-X) y Derecha (+X) → normales ±X (rotación 90° en Y)
            var rotY = NumQuaternion.CreateFromAxisAngle(NumVector3.UnitY, MathF.PI / 2f);

            _planes.Add(new BorderPlane(
                new NumVector3(-x, 0f, 0f),
                rotY,
                _mapLength, height,
                sim, _graphicsDevice, _effect));

            _planes.Add(new BorderPlane(
                new NumVector3(x, 0f, 0f),
                rotY,
                _mapLength, height,
                sim, _graphicsDevice, _effect));
        }

        public void Update(NumVector3 tankPosition, float deltaTime)
        {
            foreach (var plane in _planes)
                plane.Update(tankPosition, deltaTime);
        }

        public void Draw(XnaMatrix view, XnaMatrix projection)
        {
            var old = _graphicsDevice.RasterizerState;
            _graphicsDevice.RasterizerState = _cullNone;

            foreach (var plane in _planes)
                plane.Draw(view, projection);

            _graphicsDevice.RasterizerState = old;
        }
    }

    class BorderPlane
    {
        private readonly VertexBuffer _vb;
        private readonly IndexBuffer _ib;
        private readonly Effect _effect;

        private readonly NumVector3 _position;
        private readonly NumQuaternion _orientation;
        private readonly NumVector3 _normal;
        private readonly float _d;

        private bool _isNear;

        public BorderPlane(NumVector3 position, NumQuaternion orientation,
            float width, float height,
            Simulation sim, GraphicsDevice gd, Effect effect)
        {
            _position = position;
            _orientation = orientation;
            _effect = effect;

            // 1) Forma y registro en BEPU (pared delgada)
            var shape = new Box(width, height, 1f);
            var shapeIndex = sim.Shapes.Add(shape);

            // 2) Pose con orientación correcta
            sim.Statics.Add(new StaticDescription(position, orientation, shapeIndex));

            // 3) Normal del plano en mundo (Z local transformada)
            _normal = NumVector3.Normalize(NumVector3.Transform(NumVector3.UnitZ, orientation));
            _d = -NumVector3.Dot(_normal, position);

            // 4) Quad de debug en espacio local (X–Y, Z=0)
            var verts = new[]
            {
                new VertexPositionColor(new XnaVector3(-width * 0.5f, 0f, 0f), Color.Black),     // 0 BL
                new VertexPositionColor(new XnaVector3( width * 0.5f, 0f, 0f), Color.Black),     // 1 BR
                new VertexPositionColor(new XnaVector3(-width * 0.5f, height, 0f), Color.Black), // 2 TL
                new VertexPositionColor(new XnaVector3( width * 0.5f, height, 0f), Color.Black), // 3 TR
            };
            short[] indices = { 0, 1, 2, 1, 3, 2 };

            _vb = new VertexBuffer(gd, typeof(VertexPositionColor), verts.Length, BufferUsage.WriteOnly);
            _vb.SetData(verts);
            _ib = new IndexBuffer(gd, IndexElementSize.SixteenBits, indices.Length, BufferUsage.WriteOnly);
            _ib.SetData(indices);
        }

        public void Update(NumVector3 tankPos, float dt)
        {
            // Distancia al plano: |n·p + d|
            float dist = Math.Abs(NumVector3.Dot(tankPos, _normal) + _d);
            const float threshold = 200f;
            _isNear = dist < threshold;
        }

        public void Draw(XnaMatrix view, XnaMatrix projection)
        {
            // Dibuja siempre el borde: blanco si lejos, rojo si cerca
            var q = new XnaQuaternion(_orientation.X, _orientation.Y, _orientation.Z, _orientation.W);
            var world = XnaMatrix.CreateFromQuaternion(q) *
                        XnaMatrix.CreateTranslation(new XnaVector3(_position.X, _position.Y, _position.Z));

            // Requisitos: tu shader debe tener estos parámetros
            _effect.Parameters["View"]?.SetValue(view);
            _effect.Parameters["Projection"]?.SetValue(projection);
            _effect.Parameters["World"]?.SetValue(world);
            _effect.Parameters["UseTexture"]?.SetValue(false);

            // Tu shader usa DiffuseColor como float3
            var color = _isNear ? new NumVector3(1f, 0f, 0f) : new NumVector3(0f, 0f, 0f);
            _effect.Parameters["DiffuseColor"]?.SetValue(color);

            foreach (var pass in _effect.CurrentTechnique.Passes)
            {
                pass.Apply();
                _vb.GraphicsDevice.SetVertexBuffer(_vb);
                _vb.GraphicsDevice.Indices = _ib;
                _vb.GraphicsDevice.DrawIndexedPrimitives(PrimitiveType.TriangleList, 0, 0, 2);
            }
        }
    }
}
