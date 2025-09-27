using BepuPhysics;
using BepuPhysics.Collidables;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using XnaVector3 = Microsoft.Xna.Framework.Vector3;
using XnaQuaternion = Microsoft.Xna.Framework.Quaternion;
using SysVector3 = System.Numerics.Vector3;
using SysQuaternion = System.Numerics.Quaternion;

namespace TGC.MonoGame.TP
{
    public class Projectile
    {
        private Simulation _simulation;
        private Terrain _terrain;
        private Effect _effect;

        private float _radius;
        private float _lifeMax;
        private float _life;

        private BodyHandle _body;
        private XnaVector3 _pos;
        private XnaQuaternion _rot = XnaQuaternion.Identity;

        // Debug-geom: cubito rápido
        private static VertexBuffer _vb;
        private static IndexBuffer _ib;
        private static bool _primReady;

        public bool IsDead { get; private set; }
        public Projectile(
            Simulation simulation,
            Terrain terrain,
            Effect effect,
            XnaVector3 spawnPos,
            XnaVector3 direction,
            float speed = 120f,
            float radius = 0.5f,
            float mass = 2f,
            float lifeSeconds = 6f)
        {
            Init(simulation, terrain, effect, spawnPos, direction, speed, radius, mass, lifeSeconds);
        }
        
        private void Init(
            Simulation simulation,
            Terrain terrain,
            Effect effect,
            XnaVector3 spawnPos,
            XnaVector3 direction,
            float speed,
            float radius,
            float mass,
            float lifeSeconds)
        {
            _simulation = simulation;
            _terrain = terrain;
            _effect = effect;
            _radius = radius;
            _lifeMax = lifeSeconds;
            _life = 0f;

            // Cuerpo BEPU: esfera dinámica
            var sphere = new Sphere(radius);
            var shape = _simulation.Shapes.Add(sphere);
            var inertia = sphere.ComputeInertia(mass);

            var pose = new RigidPose(ToSys(spawnPos), SysQuaternion.Identity);
            var vel = new BodyVelocity(linear: ToSys(direction) * speed, angular: SysVector3.Zero);
            
            var collidable = new CollidableDescription(shape);

            var bodyDesc = BodyDescription.CreateDynamic(
                pose, vel, inertia,
                collidable,
                new BodyActivityDescription(0.01f)
            );

            _body = _simulation.Bodies.Add(bodyDesc);
            _pos = spawnPos;

            EnsurePrimitive(effect.GraphicsDevice);
        }

        public void Update(float dt)
        {
            if (IsDead) return;

            _life += dt;
            if (_life > _lifeMax)
            {
                Kill();
                return;
            }

            // sync pose desde física
            var bodyRef = _simulation.Bodies.GetBodyReference(_body);
            var p = bodyRef.Pose.Position;
            _pos = ToXna(p);

            // impacto simple contra terreno
            float ground = _terrain.GetHeightAtPosition(_pos.X, _pos.Z);
            if (_pos.Y - ground <= _radius * 0.5f)
            {
                Kill();
            }
        }

        private void Kill()
        {
            if (IsDead) return;
            IsDead = true;

            // Quitar el cuerpo una sola vez (sin Exists/HandleExists)
            _simulation.Bodies.Remove(_body);
        }

        public void Draw(GraphicsDevice gd, Matrix view, Matrix proj)
        {
            if (IsDead) return;

            _effect.Parameters["World"]?.SetValue(
                Matrix.CreateScale(_radius * 2f) *
                Matrix.CreateFromQuaternion(_rot) *
                Matrix.CreateTranslation(_pos)
            );
            _effect.Parameters["View"]?.SetValue(view);
            _effect.Parameters["Projection"]?.SetValue(proj);

            gd.SetVertexBuffer(_vb);
            gd.Indices = _ib;
            foreach (var pass in _effect.CurrentTechnique.Passes)
            {
                pass.Apply();
                gd.DrawIndexedPrimitives(PrimitiveType.TriangleList, 0, 0, _ib.IndexCount / 3);
            }
        }

        private static void EnsurePrimitive(GraphicsDevice gd)
        {
            if (_primReady) return;
            var v = new[]
            {
                new VertexPositionColor(new XnaVector3(-0.5f, -0.5f, -0.5f), Color.White),
                new VertexPositionColor(new XnaVector3(0.5f, -0.5f, -0.5f), Color.White),
                new VertexPositionColor(new XnaVector3(0.5f, 0.5f, -0.5f), Color.White),
                new VertexPositionColor(new XnaVector3(-0.5f, 0.5f, -0.5f), Color.White),
                new VertexPositionColor(new XnaVector3(-0.5f, -0.5f, 0.5f), Color.White),
                new VertexPositionColor(new XnaVector3(0.5f, -0.5f, 0.5f), Color.White),
                new VertexPositionColor(new XnaVector3(0.5f, 0.5f, 0.5f), Color.White),
                new VertexPositionColor(new XnaVector3(-0.5f, 0.5f, 0.5f), Color.White),
            };
            var idx = new ushort[]
            {
                0, 1, 2, 0, 2, 3,
                4, 6, 5, 4, 7, 6,
                0, 4, 5, 0, 5, 1,
                3, 2, 6, 3, 6, 7,
                0, 3, 7, 0, 7, 4,
                1, 5, 6, 1, 6, 2
            };

            _vb = new VertexBuffer(gd, VertexPositionColor.VertexDeclaration, v.Length, BufferUsage.WriteOnly);
            _vb.SetData(v);
            _ib = new IndexBuffer(gd, IndexElementSize.SixteenBits, idx.Length, BufferUsage.WriteOnly);
            _ib.SetData(idx);
            _primReady = true;
        }

        // ===== Helpers de conversión =====
        private static SysVector3 ToSys(XnaVector3 v) => new SysVector3(v.X, v.Y, v.Z);
        private static XnaVector3 ToXna(SysVector3 v) => new XnaVector3(v.X, v.Y, v.Z);
    }
}