using System;
using BepuPhysics;
using BepuPhysics.Collidables;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using System.Numerics;
using Quaternion = Microsoft.Xna.Framework.Quaternion;
using Vector3 = Microsoft.Xna.Framework.Vector3;
using BepuUtilities;
using MathHelper = Microsoft.Xna.Framework.MathHelper;
using Matrix = Microsoft.Xna.Framework.Matrix;

namespace TGC.MonoGame.TP
{
    public class Tank
    {
        private Vector3 _lastPos;
        private const float WheelRadius = 2.0f; // ajustá según tu modelo/escala

        private Model _model;
        private Effect _effect;
        private Matrix _world;

        private ModelBone _leftBackWheelBone;
        private ModelBone _rightBackWheelBone;
        private ModelBone _leftFrontWheelBone;
        private ModelBone _rightFrontWheelBone;
        private ModelBone _leftSteerBone;
        private ModelBone _rightSteerBone;
        private ModelBone _turretBone;
        private ModelBone _cannonBone;
        private ModelBone _hatchBone;

        private Matrix _leftBackWheelTransform;
        private Matrix _rightBackWheelTransform;
        private Matrix _leftFrontWheelTransform;
        private Matrix _rightFrontWheelTransform;
        private Matrix _leftSteerTransform;
        private Matrix _rightSteerTransform;
        private Matrix _turretTransform;
        private Matrix _cannonTransform;
        private Matrix _hatchTransform;

        private Matrix[] _boneTransforms;

        private Vector3 _debugBoxSize;
        private VertexBuffer _debugBoxVB;
        private IndexBuffer _debugBoxIB;
        private bool _debugBuffersReady;

        public Quaternion RotationQuaternion { get; private set; } = Quaternion.Identity;


        // Propiedades de movimiento
        public Vector3 Position { get; private set; }
        public float Rotation { get; private set; }
        public float Scale { get; private set; }

        // Agregar estas propiedades si no las tienes
        public float PitchRotation { get; private set; } // Inclinación hacia adelante/atrás
        public float RollRotation { get; private set; } // Inclinación lateral

        // ===== Control por fuerzas (tuneables) =====
        public float EngineAccel = 45f; // m/s^2 (empuje hacia adelante)
        public float BrakeAccel = 45f; // m/s^2 (empuje hacia atrás)
        public float MaxSpeed = 30f; // m/s (límite velocidad plano XZ)
        public float LateralGrip = 12f; // 1/s (mata deriva lateral)
        public float LinearDrag = 0.6f; // 1/s (freno aerodinámico simple)

        public float TurnAccelYaw = 8.0f; // rad/s^2 (acel. angular deseada)
        public float MaxYawRate = 2.8f; // rad/s (límite giro)
        public float YawInertia = 350f; // kg·m^2 (fallback si no querés usar tensor)

        public float ColliderWidth = 4f;
        public float ColliderHeight = 2f; // <<--- importante para el spawn
        public float ColliderLength = 8f;

        // Pequeño “clearance” para evitar nacer en penetración
        public float SpawnClearance = 0.05f;

        // Parámetros de movimiento
        private const float MovementSpeed = 200f;
        private const float RotationSpeed = 2f;

        // Física
        private BodyHandle _physicsBody;
        private Simulation _simulation;
        private Terrain _terrain;

        // --- Debug / Telemetría ---
        public bool DebugTelemetry = false;
        public float SteerSign = -1f; // ← por defecto invierto A/D (cámbialo en runtime con F4)
        private float _telemetryTimer = 0f;
        public string TelemetryText = "";

        /// <summary>
        /// Gets or sets the rotation of the wheels.
        /// </summary>
        public float WheelRotation { get; set; }

        /// <summary>
        ///     Gets or sets the steering rotation amount.
        /// </summary>
        public float SteerRotation { get; set; }

        /// <summary>
        ///     Gets or sets the turret rotation amount.
        /// </summary>
        public float TurretRotation { get; set; }

        /// <summary>
        ///     Gets or sets the cannon rotation amount.
        /// </summary>
        public float CannonRotation { get; set; }

        /// <summary>
        ///     Gets or sets the entry hatch rotation amount.
        /// </summary>
        public float HatchRotation { get; set; }


        public Tank(Vector3 initialPosition, float initialRotation = 0f, float scale = 20f)
        {
            Position = initialPosition;
            Rotation = initialRotation;
            Scale = scale;
            _lastPos = Position;
        }

        public void CargarModelo(string rutaRelativa, Effect efecto, ContentManager content, Simulation simulation,
            Terrain terrain = null)
        {
            _effect = efecto;
            _simulation = simulation;
            _terrain = terrain; // Guardar referencia al terreno

            // Ajustar Y inicial del tanque al terreno para no arrancar “flotando”
            if (_terrain != null)
            {
                var h = _terrain.GetHeightAtPosition(Position.X, Position.Z);
                // ¡OJO! Esto es la POSICIÓN FÍSICA del centro de masa, no la matriz de mundo.
                // Sumamos un pequeño offset para que no nazca interpenetrado.
                Position = new Vector3(Position.X, h + Scale * 0.05f, Position.Z);
            }

            // Cargar modelo
            _model = content.Load<Model>(TGCGame.ContentFolder3D + rutaRelativa);

            // Look up shortcut references to the bones we are going to animate.
            _leftBackWheelBone = _model.Bones["l_back_wheel_geo"];
            _rightBackWheelBone = _model.Bones["r_back_wheel_geo"];
            _leftFrontWheelBone = _model.Bones["l_front_wheel_geo"];
            _rightFrontWheelBone = _model.Bones["r_front_wheel_geo"];
            _leftSteerBone = _model.Bones["l_steer_geo"];
            _rightSteerBone = _model.Bones["r_steer_geo"];
            _turretBone = _model.Bones["turret_geo"];
            _cannonBone = _model.Bones["canon_geo"];
            _hatchBone = _model.Bones["hatch_geo"];

            // Store the original transform matrix for each animating bone.
            _leftBackWheelTransform = _leftBackWheelBone.Transform;
            _rightBackWheelTransform = _rightBackWheelBone.Transform;
            _leftFrontWheelTransform = _leftFrontWheelBone.Transform;
            _rightFrontWheelTransform = _rightFrontWheelBone.Transform;
            _leftSteerTransform = _leftSteerBone.Transform;
            _rightSteerTransform = _rightSteerBone.Transform;
            _turretTransform = _turretBone.Transform;
            _cannonTransform = _cannonBone.Transform;
            _hatchTransform = _hatchBone.Transform;

            // Allocate the transform matrix array.
            _boneTransforms = new Matrix[_model.Bones.Count];

            // Asignar efecto a todas las partes del modelo
            foreach (var mesh in _model.Meshes)
            {
                foreach (var meshPart in mesh.MeshParts)
                {
                    meshPart.Effect = efecto;
                }
            }

            // Crear cuerpo físico
            CreatePhysicsBody();

            // Calcular matriz inicial del mundo
            UpdateWorldMatrix();
        }

        private void CreatePhysicsBody()
        {
            if (_simulation == null) return;

            // Altura del suelo en (X,Z)
            float terrainY = _terrain != null ? _terrain.GetHeightAtPosition(Position.X, Position.Z) : Position.Y;
            // Y de spawn: suelo + media altura del collider + clearance
            float spawnY = terrainY + ColliderHeight * 0.5f + SpawnClearance;

            // Calcular orientación inicial según el terreno
            CalculateTerrainRotation(out var orientationMatrix);
            var orientationQuat = Quaternion.CreateFromRotationMatrix(orientationMatrix);

            // Dimensiones del collider (ajusta a tu modelo)
            var boxShape = new Box(Scale * ColliderWidth, Scale * ColliderHeight, Scale * ColliderLength);
            // Dimensiones del box (ya están escaladas por Scale)
            float W = boxShape.Width;
            float L = boxShape.Length;
            float massKg = 1200f; // ej: 1.2 toneladas
// Inercia de yaw para una caja alrededor del eje vertical: I = (1/12) * m * (W^2 + L^2)
            YawInertia = (1f / 12f) * massKg * (W * W + L * L);

// (Opcional) log para verificar:
            System.Diagnostics.Debug.WriteLine($"[TANK] YawInertia computed = {YawInertia:0.##}");
            var shapeIndex = _simulation.Shapes.Add(boxShape);
            _debugBoxSize = new Vector3(boxShape.Width, boxShape.Height, boxShape.Length); // para dibujarlo
            // Masa e inercia física del tanque (ajustable)
            var inertia = boxShape.ComputeInertia(massKg);

            // Pose inicial (posición + orientación que ya calculaste)
            var pose = new RigidPose(
                new System.Numerics.Vector3(Position.X, spawnY, Position.Z),
                new System.Numerics.Quaternion(orientationQuat.X, orientationQuat.Y, orientationQuat.Z,
                    orientationQuat.W)
            );

            // Velocidad inicial en cero
            var velocity = new BodyVelocity(System.Numerics.Vector3.Zero, System.Numerics.Vector3.Zero);

            // Collidable + margen especulativo un poco mayor si nacía rozando el suelo
            var collidable = new CollidableDescription(shapeIndex, 0.25f);

            // Actividad (umbral de “sueño” bajo para que se mueva enseguida)
            var activity = new BodyActivityDescription(0.01f);

            // ✔️ Versión correcta para tu Bepu:
            var bodyDesc = BodyDescription.CreateDynamic(pose, velocity, inertia, collidable, activity);

            // Agregar a la simulación
            _physicsBody = _simulation.Bodies.Add(bodyDesc);

            // Inicializar rotación visual también
            RotationQuaternion = orientationQuat;
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

        public void ControlForces(float throttle, float steer, float dt)
        {
            if (_simulation == null || _physicsBody.Value < 0) return;
            var body = _simulation.Bodies.GetBodyReference(_physicsBody);

            // --- Ejes del tanque en mundo (modelo mira +Z) ---
            var q = body.Pose.Orientation;

            // up del cuerpo en mundo
            var up = System.Numerics.Vector3.Transform(new System.Numerics.Vector3(0, 1, 0), q);
            up = up.LengthSquared() > 1e-12f
                ? System.Numerics.Vector3.Normalize(up)
                : new System.Numerics.Vector3(0, 1, 0);

            // forward “raw” (+Z local) y proyección al plano perpendicular a up
            var fwdRaw = System.Numerics.Vector3.Transform(new System.Numerics.Vector3(0, 0, 1), q);
            var fwd = fwdRaw - System.Numerics.Vector3.Dot(fwdRaw, up) * up;
            if (fwd.LengthSquared() < 1e-12f) fwd = new System.Numerics.Vector3(0, 0, 1);
            else fwd = System.Numerics.Vector3.Normalize(fwd);

            // base ortonormal (right,fwd) en el plano del suelo
            var right = System.Numerics.Vector3.Normalize(System.Numerics.Vector3.Cross(up, fwd));
            fwd = System.Numerics.Vector3.Normalize(System.Numerics.Vector3.Cross(right, up));

            // Velocidades
            var v = body.Velocity.Linear;
            var vPlanar = v - System.Numerics.Vector3.Dot(v, up) * up; // velocidad en el plano
            float speed = vPlanar.Length();

            float invMass = body.LocalInertia.InverseMass;
            if (invMass <= 0f) return;
            float mass = 1f / invMass;

            // 1) Empuje motor W/S -> impulso lineal sobre 'fwd'
            float accel = (throttle >= 0f ? EngineAccel : BrakeAccel) * throttle; // [-1..1]
            if (speed > MaxSpeed && System.Numerics.Vector3.Dot(vPlanar, fwd) * MathF.Sign(throttle) > 0f)
                accel = 0f;

            float J_engine = mass * accel * dt;
            if (J_engine != 0f) body.ApplyLinearImpulse(fwd * J_engine);

            // 2) Grip lateral (reduce deriva sobre 'right')
            float vSide = System.Numerics.Vector3.Dot(vPlanar, right);
            float kill = MathF.Min(1f, LateralGrip * dt);
            if (MathF.Abs(vSide) > 1e-4f && kill > 0f)
            {
                float J_side = mass * vSide * kill;
                body.ApplyLinearImpulse(-right * J_side);
            }

            // 3) Drag lineal suave en el plano
            if (LinearDrag > 0f && speed > 1e-3f)
            {
                var dir = vPlanar / speed;
                float J_drag = mass * (LinearDrag * dt) * speed;
                body.ApplyLinearImpulse(-dir * J_drag);
            }

            // 4) Giro A/D alrededor de 'up' (no Y global)
            float yawTarget = SteerSign * steer * MaxYawRate;
            var w = body.Velocity.Angular;

            // componente de giro actual alrededor de 'up'
            float yawNow = System.Numerics.Vector3.Dot(w, up);
            float dOmega = Math.Clamp(yawTarget - yawNow, -TurnAccelYaw * dt, TurnAccelYaw * dt);

            if (MathF.Abs(dOmega) > 1e-6f)
            {
                float J_ang = YawInertia * dOmega;
                body.ApplyAngularImpulse(up * J_ang);
            }

            // Clamps de seguridad (solo plano)
            v = body.Velocity.Linear;
            vPlanar = v - System.Numerics.Vector3.Dot(v, up) * up;
            speed = vPlanar.Length();
            if (speed > MaxSpeed * 1.5f)
            {
                var newPlanar = vPlanar * (MaxSpeed * 1.5f / speed);
                body.Velocity.Linear = newPlanar + up * System.Numerics.Vector3.Dot(v, up);
            }

            // ---- Telemetría (HUD + consola) ----
            if (DebugTelemetry)
            {
                _telemetryTimer += dt;
                if (_telemetryTimer > 0.20f) // 5 Hz
                {
                    _telemetryTimer = 0f;
                    float kmh = speed * 3.6f;

                    TelemetryText =
                        $"thr {throttle:+0.00;-0.00;0}  steer {steer:+0.00;-0.00;0}  sign {SteerSign:+0.0;-0.0;0}\n" +
                        $"speed {speed:0.00} m/s ({kmh:0} km/h)   vSide {vSide:+0.00;-0.00;0}\n" +
                        $"yawNow {yawNow:+0.00;-0.00;0} rad/s   yawTgt {yawTarget:+0.00;-0.00;0}   d {dOmega:+0.00;-0.00;0}";

                    System.Diagnostics.Debug.WriteLine("[TANK] " + TelemetryText);
                }
            }
        }


        public void DrawCollider(GraphicsDevice gd, Matrix view, Matrix projection, Effect effect,
            bool wireframe = true)
        {
            if (_simulation == null || _physicsBody.Value < 0) return;
            EnsureDebugCube(gd);

            var body = _simulation.Bodies.GetBodyReference(_physicsBody);
            var p = body.Pose.Position;
            var q = body.Pose.Orientation;

            var world =
                Matrix.CreateScale(_debugBoxSize) *
                Matrix.CreateFromQuaternion(new Microsoft.Xna.Framework.Quaternion(q.X, q.Y, q.Z, q.W)) *
                Matrix.CreateTranslation(new Microsoft.Xna.Framework.Vector3(p.X, p.Y, p.Z));

            // Set de matrices
            effect.Parameters["World"]?.SetValue(world);
            effect.Parameters["View"]?.SetValue(view);
            effect.Parameters["Projection"]?.SetValue(projection);

            // Rasterizer en wireframe si querés ver sólo el contorno
            var old = gd.RasterizerState;
            if (wireframe)
                gd.RasterizerState =
                    RasterizerState.CullNone; // y luego setear WireFrame en el Pass si tu efecto lo soporta

            gd.SetVertexBuffer(_debugBoxVB);
            gd.Indices = _debugBoxIB;

            foreach (var pass in effect.CurrentTechnique.Passes)
            {
                pass.Apply();
                gd.DrawIndexedPrimitives(PrimitiveType.TriangleList, 0, 0, _debugBoxIB.IndexCount / 3);
            }

            gd.RasterizerState = old;
        }

        public void Update(GameTime gameTime, KeyboardState keyboardState)
        {
            float dt = (float)gameTime.ElapsedGameTime.TotalSeconds;

            bool hasSim = _simulation != null && _physicsBody.Value >= 0;

            if (hasSim)
            {
                var body = _simulation.Bodies.GetBodyReference(_physicsBody);
                body.Awake = true;

                // Orientación actual del cuerpo físico -> ejes locales
                var q = body.Pose.Orientation;
                var rot = Matrix.CreateFromQuaternion(new Microsoft.Xna.Framework.Quaternion(q.X, q.Y, q.Z, q.W));
                var forward = Vector3.Transform(-Vector3.UnitZ, rot); // tu “frente”
                var forward3 = new System.Numerics.Vector3(forward.X, 0, forward.Z);
/*
                // Parámetros de control (ajustá a tu escala)
                const float MotorForce = 25000f; // N
                const float TurnForce = 3000f; // N·m
                const float MaxSpeed = 120f; // límite simple en XZ

                // Impulsos = fuerza * dt
                float motorImpulse = MotorForce * dt;
                float turnImpulse = TurnForce * dt;

                // Límite de velocidad horizontal
                var v = body.Velocity.Linear;
                float speedXZ = MathF.Sqrt(v.X * v.X + v.Z * v.Z);

                // W/S: impulso lineal hacia adelante/atrás
                if (keyboardState.IsKeyDown(Keys.W) && speedXZ < MaxSpeed)
                    body.ApplyLinearImpulse(forward3 * motorImpulse);
                if (keyboardState.IsKeyDown(Keys.S) && speedXZ < MaxSpeed)
                    body.ApplyLinearImpulse(-forward3 * motorImpulse);

                // A/D: impulso angular alrededor de Y (giro)
                if (keyboardState.IsKeyDown(Keys.A))
                    body.ApplyAngularImpulse(new System.Numerics.Vector3(0, turnImpulse, 0));
                if (keyboardState.IsKeyDown(Keys.D))
                    body.ApplyAngularImpulse(new System.Numerics.Vector3(0, -turnImpulse, 0));

                // Amortiguación suave (ajustable)
                body.Velocity.Linear *= 0.999f;
                body.Velocity.Angular *= 0.999f;

                // ¡No pises la altura con el terreno cuando usás física!
*/
                var kb = Keyboard.GetState();

                float throttle = 0f; // W adelante, S atrás
                if (kb.IsKeyDown(Keys.W)) throttle += 1f;
                if (kb.IsKeyDown(Keys.S)) throttle -= 1f;

                float steer = 0f; // A izquierda, D derecha
                if (kb.IsKeyDown(Keys.A)) steer -= 1f;
                if (kb.IsKeyDown(Keys.D)) steer += 1f;

                throttle = Math.Clamp(throttle, -1f, 1f);
                steer = Math.Clamp(steer, -1f, 1f);

                ControlForces(throttle, steer, dt);
                // Sincronizar pose visual desde la física
                SyncFromPhysics(); // ya lo tenés implementado
            }
            else
            {
                // Fallback sin física (tu comportamiento original)
                bool moved = false;

                if (keyboardState.IsKeyDown(Keys.W))
                {
                    MoveForward(dt);
                    moved = true;
                }

                if (keyboardState.IsKeyDown(Keys.S))
                {
                    MoveBackward(dt);
                    moved = true;
                }

                if (keyboardState.IsKeyDown(Keys.A))
                {
                    RotateLeft(dt);
                    moved = true;
                }

                if (keyboardState.IsKeyDown(Keys.D))
                {
                    RotateRight(dt);
                    moved = true;
                }

                // Sólo si NO hay física, ajustá la Y al terreno
                if (_terrain != null && !hasSim)
                {
                    float terrainHeight = _terrain.GetHeightAtPosition(Position.X, Position.Z);
                    Position = new Vector3(Position.X, terrainHeight + Scale * 0.02f, Position.Z); // <- + (no -)
                }

                if (moved)
                    UpdatePhysicsBody(); // escribe la pose en la física sólo en este modo
            }

            // Girar ruedas según distancia recorrida
            UpdateWheelSpinByDistance(dt);

            UpdateWorldMatrix();
        }

        private void UpdateWheelSpinByDistance(float dt)
        {
            var delta = Position - _lastPos;
            var dist = delta.Length();

            // Dirección “forward” actual para signo (+ avanza / - retrocede)
            var forward = Vector3.Transform(-Vector3.UnitZ, Matrix.CreateRotationY(Rotation));
            float sign = 0f;
            if (dist > 0.0001f)
            {
                var dir = Vector3.Normalize(delta);
                sign = MathF.Sign(Vector3.Dot(dir, forward));
            }

            WheelRotation += sign * (dist / WheelRadius);

            if (WheelRotation > MathHelper.TwoPi) WheelRotation -= MathHelper.TwoPi;
            else if (WheelRotation < -MathHelper.TwoPi) WheelRotation += MathHelper.TwoPi;

            _lastPos = Position;
        }

        private void MoveForward(float deltaTime)
        {
            Vector3 forward = Vector3.Transform(-Vector3.UnitZ, Matrix.CreateRotationY(Rotation));
            Position += forward * MovementSpeed * deltaTime;
        }

        private void MoveBackward(float deltaTime)
        {
            Vector3 forward = Vector3.Transform(-Vector3.UnitZ, Matrix.CreateRotationY(Rotation));
            Position -= forward * MovementSpeed * deltaTime;
        }

        private void RotateLeft(float deltaTime)
        {
            Rotation += RotationSpeed * deltaTime; // Cambiado de -= a +=
        }

        private void RotateRight(float deltaTime)
        {
            Rotation -= RotationSpeed * deltaTime; // Cambiado de += a -=
        }

        private void UpdatePhysicsBody()
        {
            if (_simulation != null && _physicsBody.Value >= 0)
            {
                // Actualizar posición en la física
                var bodyReference = _simulation.Bodies.GetBodyReference(_physicsBody);
                bodyReference.Pose.Position = new System.Numerics.Vector3(Position.X, Position.Y, Position.Z);
                bodyReference.Pose.Orientation = System.Numerics.Quaternion.CreateFromYawPitchRoll(Rotation, 0, 0);

                // Activar el cuerpo para que se actualice
                bodyReference.Activity.SleepThreshold = -1;
            }
        }

        private void SyncFromPhysics()
        {
            var bodyReference = _simulation.Bodies.GetBodyReference(_physicsBody);
            var physicsPosition = bodyReference.Pose.Position;

            Position = new Vector3(physicsPosition.X, physicsPosition.Y, physicsPosition.Z);

            // 🔧 Ajustar para apoyar sobre el terreno (solo si estás sobre él)
/*            if (_terrain != null)
            {
                float terrainHeight = _terrain.GetHeightAtPosition(Position.X, Position.Z);
                float offset = _debugBoxSize.Y * 0.5f; // mitad de la caja física real
                Position = new Vector3(Position.X, terrainHeight + offset, Position.Z);
            }
*/
            // Extraer rotación Y del quaternion
            var orientation = bodyReference.Pose.Orientation;
            Rotation = MathF.Atan2(2.0f * (orientation.W * orientation.Y + orientation.X * orientation.Z),
                1.0f - 2.0f * (orientation.Y * orientation.Y + orientation.Z * orientation.Z));
        }

        public void SyncFromPhysics2()
        {
            var body = _simulation.Bodies.GetBodyReference(_physicsBody);
            var pose = body.Pose;

            Position = new Vector3(pose.Position.X, pose.Position.Y, pose.Position.Z);
            var q = pose.Orientation;
            RotationQuaternion = new Quaternion(q.X, q.Y, q.Z, q.W);
        }

        private void UpdateWorldMatrix()
        {
            // 🔁 Sincronizar posición y rotación desde la física
            var bodyReference = _simulation.Bodies.GetBodyReference(_physicsBody);
            ref var body = ref bodyReference;
            var pose = body.Pose;

            Position = new Vector3(pose.Position.X, pose.Position.Y, pose.Position.Z);
            RotationQuaternion = new Quaternion(pose.Orientation.X, pose.Orientation.Y, pose.Orientation.Z,
                pose.Orientation.W);

            // 📦 Construir la matriz del mundo
            _world =
                Matrix.CreateScale(Scale) *
                Matrix.CreateFromQuaternion(RotationQuaternion) *
                Matrix.CreateTranslation(Position);
        }

        private void CalculateTerrainRotation(out Matrix orientation)
        {
            // 1) Normal del terreno por diferencias centrales (usar un paso acorde a tu scaleXZ)
            float h = 50f; // medio “paso” de muestreo (~ scaleXZ/2)
            float x = Position.X, z = Position.Z;

            float hL = _terrain.GetHeightAtPosition(x - h, z);
            float hR = _terrain.GetHeightAtPosition(x + h, z);
            float hD = _terrain.GetHeightAtPosition(x, z - h);
            float hU = _terrain.GetHeightAtPosition(x, z + h);

            // Gradientes (tangentes del heightfield)
            var tangentX = new Microsoft.Xna.Framework.Vector3(2f * h, hR - hL, 0f); // (Δx, Δy, 0)
            var tangentZ = new Microsoft.Xna.Framework.Vector3(0f, hU - hD, 2f * h); // (0, Δy, Δz)

            // Normal “up” del terreno
            var up = Microsoft.Xna.Framework.Vector3.Normalize(
                Microsoft.Xna.Framework.Vector3.Cross(tangentZ, tangentX));

            // 2) Direcciones objetivo: mantené la YAW de la física
            var yawForward = Microsoft.Xna.Framework.Vector3.Transform(
                -Microsoft.Xna.Framework.Vector3.UnitZ,
                Matrix.CreateRotationY(Rotation));

            // Proyectá el forward sobre el plano del terreno para que siga la pendiente
            var forwardOnPlane = yawForward - Microsoft.Xna.Framework.Vector3.Dot(yawForward, up) * up;
            if (forwardOnPlane.LengthSquared() < 1e-6f)
                forwardOnPlane = Microsoft.Xna.Framework.Vector3.Normalize(
                    Microsoft.Xna.Framework.Vector3.Cross(
                        new Microsoft.Xna.Framework.Vector3(1, 0, 0), up));
            else
                forwardOnPlane.Normalize();

            var right = Microsoft.Xna.Framework.Vector3.Normalize(
                Microsoft.Xna.Framework.Vector3.Cross(forwardOnPlane, up));
            var forward = Microsoft.Xna.Framework.Vector3.Normalize(
                Microsoft.Xna.Framework.Vector3.Cross(up, right));

            // 3) Matriz de orientación a partir de la base R-U-F
            orientation = new Matrix(
                right.X, right.Y, right.Z, 0f,
                up.X, up.Y, up.Z, 0f,
                forward.X, forward.Y, forward.Z, 0f,
                0f, 0f, 0f, 1f
            );

            // (Opcional) si aún querés ‘PitchRotation’ y ‘RollRotation’ para depurar:
            PitchRotation = MathF.Asin(-forward.Y);
            RollRotation = MathF.Asin(right.Y);
        }


        public void Draw()
        {
            if (_model == null || _effect == null) return;

            var wheelRotation = Matrix.CreateRotationX(WheelRotation);
            var steerRotation = Matrix.CreateRotationY(SteerRotation);
            var turretRotation = Matrix.CreateRotationY(TurretRotation);
            var cannonRotation = Matrix.CreateRotationX(CannonRotation);
            var hatchRotation = Matrix.CreateRotationX(HatchRotation);

            _leftBackWheelBone.Transform = wheelRotation * _leftBackWheelTransform;
            _rightBackWheelBone.Transform = wheelRotation * _rightBackWheelTransform;
            _leftFrontWheelBone.Transform = wheelRotation * _leftFrontWheelTransform;
            _rightFrontWheelBone.Transform = wheelRotation * _rightFrontWheelTransform;

            _leftSteerBone.Transform = steerRotation * _leftSteerTransform;
            _rightSteerBone.Transform = steerRotation * _rightSteerTransform;

            _turretBone.Transform = turretRotation * _turretTransform;
            _cannonBone.Transform = cannonRotation * _cannonTransform;
            _hatchBone.Transform = hatchRotation * _hatchTransform;

            var absBones = new Matrix[_model.Bones.Count];
            _model.CopyAbsoluteBoneTransformsTo(absBones);

            foreach (var mesh in _model.Meshes)
            {
                var worldPerMesh = absBones[mesh.ParentBone.Index] * _world;

                foreach (var part in mesh.MeshParts)
                {
                    part.Effect.Parameters["World"]?.SetValue(worldPerMesh);
                }

                mesh.Draw();
            }
        }
    }
}