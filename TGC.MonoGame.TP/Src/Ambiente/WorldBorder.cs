using System;
using System.Collections.Generic;
using BepuPhysics;
using BepuPhysics.Collidables;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using NumVector3 = System.Numerics.Vector3;
using NumVector4 = System.Numerics.Vector4;
using NumQuaternion = System.Numerics.Quaternion;
using XnaVector3 = Microsoft.Xna.Framework.Vector3;
using XnaMatrix = Microsoft.Xna.Framework.Matrix;
using XnaQuaternion = Microsoft.Xna.Framework.Quaternion;

namespace TGC.MonoGame.TP.Ambiente
{
    public class WorldBorder
    {
        public readonly List<BorderPlane> Planes = new();
        private readonly GraphicsDevice _graphicsDevice;
        private readonly Effect _effect;
        private readonly float _mapWidth;
        private readonly float _mapLength;

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
            float width = _mapWidth - 2 * margin;
            float length = _mapLength - 2 * margin;
            float height = 2000f;

            // Frente (-Z)
            Planes.Add(new BorderPlane(
                new NumVector3(0f, 0f, -(_mapLength / 2f) + margin),
                NumQuaternion.Identity,
                width, height,
                sim, _graphicsDevice, _effect));

            // Fondo (+Z)
            var rot180Y = NumQuaternion.CreateFromAxisAngle(NumVector3.UnitY, MathF.PI);
            Planes.Add(new BorderPlane(
                new NumVector3(0f, 0f, +(_mapLength / 2f) - margin),
                rot180Y,
                width, height,
                sim, _graphicsDevice, _effect));

            // Izquierda (-X)
            var rotYLeft = NumQuaternion.CreateFromAxisAngle(NumVector3.UnitY, MathF.PI / 2f);
            Planes.Add(new BorderPlane(
                new NumVector3(-(_mapWidth / 2f) + margin, 0f, 0f),
                rotYLeft,
                length, height,
                sim, _graphicsDevice, _effect));

            // Derecha (+X)
            var rotYRight = NumQuaternion.CreateFromAxisAngle(NumVector3.UnitY, -MathF.PI / 2f);
            Planes.Add(new BorderPlane(
                new NumVector3(+(_mapWidth / 2f) - margin, 0f, 0f),
                rotYRight,
                length, height,
                sim, _graphicsDevice, _effect));
        }


        public void Update(NumVector3 tankPosition)
        {
            foreach (var plane in Planes)
                plane.Update(tankPosition);
        }

        public void Draw(XnaMatrix view, XnaMatrix projection)
        {
            foreach (var plane in Planes)
                plane.Draw(view, projection);
        }
    }

    public class BorderPlane
    {
        private readonly VertexBuffer _vb;
        private readonly IndexBuffer _ib;
        private readonly Effect _effect;
        
        private readonly NumVector3 _position;
        private readonly NumQuaternion _orientation;
        public readonly NumVector3 Normal;
        public readonly float D;

        private float _alpha;
        private float _distance;
        private const float MaxDistance = 300f;
        private const float MaxAlpha = 0.9f;

        public BorderPlane(NumVector3 position, NumQuaternion orientation,
            float width, float height,
            Simulation sim, GraphicsDevice gd, Effect effect)
        {
            _position = position;
            _orientation = orientation;
            _effect = effect;
            
            Normal = NumVector3.Normalize(NumVector3.Transform(NumVector3.UnitZ, orientation)); // Normal del plano en mundo (Z local transformada)
            D = -NumVector3.Dot(Normal, position); // D de la ecuación del plano

            // Forma y registro en BEPU (pared delgada)
            var shape = new Box(width, height, 1f);
            var shapeIndex = sim.Shapes.Add(shape);
            sim.Statics.Add(new StaticDescription(position, orientation, shapeIndex));
            
            // Quad en espacio local (X–Y, Z=0)
            var verts = new[]
            {
                new VertexPositionColor(new XnaVector3(-width * 0.5f, 0f, 0f), Color.Black),     // 0 BL
                new VertexPositionColor(new XnaVector3( width * 0.5f, 0f, 0f), Color.Black),     // 1 BR
                new VertexPositionColor(new XnaVector3(-width * 0.5f, height, 0f), Color.Black), // 2 TL
                new VertexPositionColor(new XnaVector3( width * 0.5f, height, 0f), Color.Black), // 3 TR
            };
            short[] indices = [0, 2, 1, 1, 2, 3];

            _vb = new VertexBuffer(gd, typeof(VertexPositionColor), verts.Length, BufferUsage.WriteOnly);
            _vb.SetData(verts);
            _ib = new IndexBuffer(gd, IndexElementSize.SixteenBits, indices.Length, BufferUsage.WriteOnly);
            _ib.SetData(indices);
        }

        public void Update(NumVector3 tankPos)
        {
            _distance = Math.Abs(NumVector3.Dot(tankPos, Normal) + D); // Evaluar la posicion del tanque en la ecuación del plano

            float factor = 1f - MathHelper.Clamp(_distance / MaxDistance, 0f, 1f); // Que tan cerca o lejos está el tanque

            _alpha = factor * MaxAlpha;
        }


        public void Draw(XnaMatrix view, XnaMatrix projection)
        {
            if (_distance > MaxDistance) return; // No dibujarlo si esta demasiado lejos
            
            var quaternion = new XnaQuaternion(_orientation.X, _orientation.Y, _orientation.Z, _orientation.W);
            var world = XnaMatrix.CreateFromQuaternion(quaternion) *
                        XnaMatrix.CreateTranslation(new XnaVector3(_position.X, _position.Y, _position.Z));
            
            _effect.Parameters["View"]?.SetValue(view);
            _effect.Parameters["Projection"]?.SetValue(projection);
            _effect.Parameters["World"]?.SetValue(world);
            
            var color = new NumVector4(1f, 0f, 0f, _alpha);
            _effect.Parameters["TintColor"]?.SetValue(color);

            var gd = _vb.GraphicsDevice;
            var oldBlend = gd.BlendState;
            gd.BlendState = BlendState.NonPremultiplied;

            foreach (var pass in _effect.CurrentTechnique.Passes)
            {
                pass.Apply();
                gd.SetVertexBuffer(_vb);
                gd.Indices = _ib;
                gd.DrawIndexedPrimitives(PrimitiveType.TriangleList, 0, 0, 2);
            }

            gd.BlendState = oldBlend;
        }
    }
}
