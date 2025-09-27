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
        public float EngineAccel = 40f; // m/s^2 empuje "motor" (adelante/atrás)
        public float BrakeAccel = 40f; // m/s^2 si querés distinto para reversa, usá otro valor
        public float MaxSpeed = 35f; // m/s límite de velocidad planar
        public float LateralGrip = 8f; // 1/s cuánto mata la deriva lateral (0=sin grip, > mata deslizamiento)
        public float LinearDrag = 0.5f; // 1/s freno aerodinámico simple (plano XZ)

        public float TurnAccelYaw = 4.0f; // rad/s^2 aceleración angular deseada (yaw)
        public float MaxYawRate = 2.5f; // rad/s límite de giro
        public float YawInertia = 300f; // kg·m^2 aprox (inercia de yaw usada para impulse = I * dOmega)

        // Parámetros de movimiento
        private const float MovementSpeed = 200f;
        private const float RotationSpeed = 2f;

        // Física
        private BodyHandle _physicsBody;
        private Simulation _simulation;
        private Terrain _terrain;

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
/*
            // Crear una caja para representar el tanque en la física
            var box = new Box(Scale * 6.5f, Scale * 1f, Scale * 7f);
            _debugBoxSize = new Vector3(box.Width, box.Height, box.Length); // para dibujarlo
            var boxIndex = _simulation.Shapes.Add(box);

            const float mass = 1000f; // Masa de 1000kg
            var inertia = box.ComputeInertia(mass); // <— inercia real del box

            // Crear cuerpo dinámico
            var bodyDescription = BodyDescription.CreateDynamic(
                new System.Numerics.Vector3(Position.X, Position.Y, Position.Z),
                inertia,
                new CollidableDescription(boxIndex, 0.1f),
                new BodyActivityDescription(0.01f)
            );

            _physicsBody = _simulation.Bodies.Add(bodyDescription);*/

            // Calcular orientación inicial según el terreno
            CalculateTerrainRotation(out var orientationMatrix);
            var orientationQuat = Quaternion.CreateFromRotationMatrix(orientationMatrix);

            // Crear la forma física del tanque (por ejemplo, una caja)
            var box = new Box(Scale * 6.5f, Scale * 1f, Scale * 7f);
            _debugBoxSize = new Vector3(box.Width, box.Height, box.Length); // para dibujarlo
            var boxIndex = _simulation.Shapes.Add(box);

            const float mass = 1000f; // Masa de 1000kg
            var inertia = box.ComputeInertia(mass); // <— inercia real del box

            // Descripción física
            var bodyDescription = BodyDescription.CreateDynamic(
                new System.Numerics.Vector3(Position.X, Position.Y, Position.Z),
                inertia,
                new CollidableDescription(boxIndex, 0.1f),
                new BodyActivityDescription(0.01f)
            );

            // 🔧 Aplicar la misma orientación física que la visual
            bodyDescription.Pose.Orientation = new System.Numerics.Quaternion(
                orientationQuat.X,
                orientationQuat.Y,
                orientationQuat.Z,
                orientationQuat.W
            );

            // Agregar el cuerpo al simulador
            _physicsBody = _simulation.Bodies.Add(bodyDescription);

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

            var bodyReference = _simulation.Bodies.GetBodyReference(_physicsBody);
            ref var body = ref bodyReference;

            // --- Base: vectores del tanque en mundo (adelante = -Z) ---
            var q = body.Pose.Orientation;
            var qMat = System.Numerics.Matrix4x4.CreateFromQuaternion(q);

            var forward = System.Numerics.Vector3.Transform(new System.Numerics.Vector3(0, 0, 1), qMat);
            forward.Y = 0;
            float fLen = forward.Length();
            if (fLen > 1e-6f) forward /= fLen;
            else forward = new System.Numerics.Vector3(0, 0, 1);

            // right queda igual (cross(up, forward) -> (f.z, 0, -f.x))
            var right = new System.Numerics.Vector3(forward.Z, 0, -forward.X);


            // --- Estado actual (solo plano) ---
            var vLin = body.Velocity.Linear;
            var vPlanar = new System.Numerics.Vector3(vLin.X, 0, vLin.Z);
            float speed = vPlanar.Length();

            // Masa (de la inercia local del cuerpo)
            float invMass = body.LocalInertia.InverseMass;
            if (invMass <= 0f) return; // no dinámico
            float mass = 1f / invMass;

            // ===========================================================
            // 1) EMPUJE DEL MOTOR (impulso lineal en dirección forward)
            // ===========================================================
            float accel = (throttle >= 0f ? EngineAccel : BrakeAccel) * throttle; // throttle ∈ [-1..1]
            // Evitar seguir acelerando si ya superó MaxSpeed en el mismo sentido:
            if (speed > MaxSpeed && System.Numerics.Vector3.Dot(vPlanar, forward) * MathF.Sign(throttle) > 0f)
                accel = 0f;

            float J_engine = mass * accel * dt; // impulso = m * dv
            if (J_engine != 0f)
                body.ApplyLinearImpulse(forward * J_engine);

            // ===========================================================
            // 2) GRIP LATERAL (mata deriva / drifting en XZ)
            // ===========================================================
            float vSide = System.Numerics.Vector3.Dot(vPlanar, right);
            float kill = MathF.Min(1f, LateralGrip * dt); // 0..1
            if (MathF.Abs(vSide) > 1e-4f && kill > 0f)
            {
                float J_side = mass * vSide * kill; // queremos reducir vSide hacia 0
                body.ApplyLinearImpulse(-right * J_side); // opuesto a la deriva
            }

            // ===========================================================
            // 3) DRAG LINEAL (freno suave aerodinámico en XZ)
            // ===========================================================
            if (LinearDrag > 0f && speed > 1e-3f)
            {
                var dir = vPlanar / speed;
                float J_drag = mass * (LinearDrag * dt) * speed; // reduce proporcional al módulo
                body.ApplyLinearImpulse(-dir * J_drag);
            }

            // ===========================================================
            // 4) GIRO (impulso angular alrededor de Y)
            // ===========================================================
            float yawTarget = -steer * MaxYawRate;
            float yawNow = body.Velocity.Angular.Y;
            float errYaw = yawTarget - yawNow;

            float dOmega = Math.Clamp(errYaw, -TurnAccelYaw * dt, TurnAccelYaw * dt); // delta ω permitido
            if (MathF.Abs(dOmega) > 1e-5f)
            {
                // Impulso angular J = I * dΩ (I ≈ YawInertia tuneable)
                float J_ang = YawInertia * dOmega;
                body.ApplyAngularImpulse(new System.Numerics.Vector3(0, J_ang, 0));
            }

            // (Opcional) Clamps finales por seguridad
            vLin = body.Velocity.Linear;
            vPlanar = new System.Numerics.Vector3(vLin.X, 0, vLin.Z);
            speed = vPlanar.Length();
            if (speed > MaxSpeed * 1.5f) // límite duro de seguridad
            {
                vPlanar *= (MaxSpeed * 1.5f / speed);
                body.Velocity.Linear = new System.Numerics.Vector3(vPlanar.X, vLin.Y, vPlanar.Z);
            }

            // Limitar yaw rate por seguridad
            var w = body.Velocity.Angular;
            if (MathF.Abs(w.Y) > MaxYawRate * 1.5f)
                body.Velocity.Angular = new System.Numerics.Vector3(w.X, MathF.Sign(w.Y) * MaxYawRate * 1.5f, w.Z);
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
                float throttle = 0f; // W adelante, S atrás
                if (keyboardState.IsKeyDown(Keys.W)) throttle += 1f;
                if (keyboardState.IsKeyDown(Keys.S)) throttle -= 1f;

                float steer = 0f; // A izquierda, D derecha
                if (keyboardState.IsKeyDown(Keys.A)) steer -= 1f;
                if (keyboardState.IsKeyDown(Keys.D)) steer += 1f;

// Normalizar por si se pisan opuestos
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
            if (_terrain != null)
            {
                float terrainHeight = _terrain.GetHeightAtPosition(Position.X, Position.Z);
                float offset = _debugBoxSize.Y * 0.5f; // mitad de la caja física real
                Position = new Vector3(Position.X, terrainHeight + offset, Position.Z);
            }

            // Extraer rotación Y del quaternion
            var orientation = bodyReference.Pose.Orientation;
            Rotation = MathF.Atan2(2.0f * (orientation.W * orientation.Y + orientation.X * orientation.Z),
                1.0f - 2.0f * (orientation.Y * orientation.Y + orientation.Z * orientation.Z));
        }

        public void SyncFromPhysics2()
        {
            var bodyReference = _simulation.Bodies.GetBodyReference(_physicsBody);
            ref var body = ref bodyReference;
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