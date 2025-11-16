using System;
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
    public class Projectile : GeometricPrimitive
    {
        private Simulation _simulation;
        private Effect _effect;

        private float _radius;
        private float _lifeMax;
        private float _life;

        private BodyHandle _body;
        public BodyHandle Body => _body;
        private XnaVector3 _pos;
        private readonly XnaQuaternion _rot = XnaQuaternion.Identity;

        // Debug-geom: cubito rápido
        //private static VertexBuffer _vb;
        //private static IndexBuffer _ib;
        //private static bool _primReady;

        public Tank TanqueDisparador;

        public bool IsDead { get; private set; }

        public float Damage;
        
        public Projectile(
            Simulation simulation,
            Effect effect,
            XnaVector3 spawnPos,
            XnaVector3 direction,
            ProjectileType type,
            Tank tank,
            CollidableProperty<TankBodyProperties> properties,
            float lifeSeconds = 4f)
        {
            Init(simulation, effect, spawnPos, direction, type.Speed, type.Radius, type.Mass, tank, properties, lifeSeconds);
        }
        
        private void Init(
            Simulation simulation,
            Effect effect,
            XnaVector3 spawnPos,
            XnaVector3 direction,
            float speed,
            float radius,
            float mass,
            Tank tank,
            CollidableProperty<TankBodyProperties> properties,
            float lifeSeconds)
        {
            _simulation = simulation;
            _effect = effect;
            _radius = radius;
            _lifeMax = lifeSeconds;
            _life = 0f;
            TanqueDisparador = tank;

            Damage = mass * 10f; // Masa * Multiplicador de daño

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
            ref var props = ref properties.Allocate(_body);
            props = new TankBodyProperties { Friction = 0.5f, TankPart = false };
            props.Filter = new SubgroupCollisionFilter(groupId: 99, subgroupId: 0);
            CollisionHandler.HandleToProjectile[_body] = this;
            
            _pos = spawnPos;

            CreatePrimitive(effect.GraphicsDevice, radius * 2, 16, Color.Black);
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

            /*
            // impacto simple contra terreno
            float ground = _terrain.GetHeightAtPosition(_pos.X, _pos.Z);
            if (_pos.Y - ground <= _radius * 0.5f)
            {
                Kill();
            }
            */
        }

        public void Kill()
        {
            if (IsDead) return;
            IsDead = true;

            if (_simulation.Bodies.BodyExists(_body))
            {
                _simulation.Bodies.Remove(_body);
            }
            CollisionHandler.HandleToProjectile.Remove(_body);
        }

        public void Draw(Effect effect, Matrix view, Matrix proj)
        {
            if (IsDead) return;

            _effect.Parameters["World"]?.SetValue(
                Matrix.CreateScale(_radius * 2f) *
                Matrix.CreateFromQuaternion(_rot) *
                Matrix.CreateTranslation(_pos)
            );
            _effect.Parameters["View"]?.SetValue(view);
            _effect.Parameters["Projection"]?.SetValue(proj);
            _effect.Parameters["UseTexture"]?.SetValue(false);
            _effect.Parameters["TintColor"]?.SetValue(Color.Black.ToVector4());

            base.Draw(effect);
        }

        private void CreatePrimitive(GraphicsDevice graphicsDevice, float diameter, int tessellation, Color color)
        {
            if (tessellation < 3)
                throw new ArgumentOutOfRangeException("tessellation");

            var verticalSegments = tessellation;
            var horizontalSegments = tessellation * 2;

            var radius = diameter / 2;

            // Start with a single vertex at the bottom of the sphere.
            AddVertex(Vector3.Down * radius, color, Vector3.Down);

            // Create rings of vertices at progressively higher latitudes.
            for (var i = 0; i < verticalSegments - 1; i++)
            {
                var latitude = (i + 1) * MathHelper.Pi /
                    verticalSegments - MathHelper.PiOver2;

                var dy = (float) Math.Sin(latitude);
                var dxz = (float) Math.Cos(latitude);

                // Create a single ring of vertices at this latitude.
                for (var j = 0; j < horizontalSegments; j++)
                {
                    var longitude = j * MathHelper.TwoPi / horizontalSegments;

                    var dx = (float) Math.Cos(longitude) * dxz;
                    var dz = (float) Math.Sin(longitude) * dxz;

                    var normal = new Vector3(dx, dy, dz);

                    AddVertex(normal * radius, color, normal);
                }
            }

            // Finish with a single vertex at the top of the sphere.
            AddVertex(Vector3.Up * radius, color, Vector3.Up);

            // Create a fan connecting the bottom vertex to the bottom latitude ring.
            for (var i = 0; i < horizontalSegments; i++)
            {
                AddIndex(0);
                AddIndex(1 + (i + 1) % horizontalSegments);
                AddIndex(1 + i);
            }

            // Fill the sphere body with triangles joining each pair of latitude rings.
            for (var i = 0; i < verticalSegments - 2; i++)
            for (var j = 0; j < horizontalSegments; j++)
            {
                var nextI = i + 1;
                var nextJ = (j + 1) % horizontalSegments;

                AddIndex(1 + i * horizontalSegments + j);
                AddIndex(1 + i * horizontalSegments + nextJ);
                AddIndex(1 + nextI * horizontalSegments + j);

                AddIndex(1 + i * horizontalSegments + nextJ);
                AddIndex(1 + nextI * horizontalSegments + nextJ);
                AddIndex(1 + nextI * horizontalSegments + j);
            }

            // Create a fan connecting the top vertex to the top latitude ring.
            for (var i = 0; i < horizontalSegments; i++)
            {
                AddIndex(CurrentVertex - 1);
                AddIndex(CurrentVertex - 2 - (i + 1) % horizontalSegments);
                AddIndex(CurrentVertex - 2 - i);
            }

            InitializePrimitive(graphicsDevice);
        }

        // ===== Helpers de conversión =====
        private static SysVector3 ToSys(XnaVector3 v) => new(v.X, v.Y, v.Z);
        private static XnaVector3 ToXna(SysVector3 v) => new(v.X, v.Y, v.Z);
    }
}