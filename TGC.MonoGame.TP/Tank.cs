using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using Demos.Demos.Tanks;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Quaternion = Microsoft.Xna.Framework.Quaternion;
using Vector3 = Microsoft.Xna.Framework.Vector3;
using TGC.MonoGame.Samples.Cameras;
using TGC.MonoGame.Samples.Viewer.Gizmos;
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

        private ModelBone[] _wheelBones;
        private ModelBone _turretBone;
        private ModelBone _cannonBone;
        
        public BodyHandle Body;
        public BodyHandle Turret;
        public BodyHandle Barrel;

        private Matrix[] _wheelTransforms;
        private Matrix _turretTransform;
        private Matrix _cannonTransform;

        private Matrix[] _boneTransforms;
        
        TwistServo BarrelServoDescription;
        TwistServo TurretServoDescription;
        public ConstraintHandle TurretServo;
        public ConstraintHandle BarrelServo;
        public Buffer<BodyHandle> WheelHandles;
        public Buffer<ConstraintHandle> Constraints;
        System.Numerics.Quaternion FromBodyLocalToTurretBasisLocal;
        System.Numerics.Vector3 BarrelLocalDirection;
        public System.Numerics.Quaternion BodyLocalOrientation;
        
        public Gizmos Gizmos { get; set;}
        
        public Quaternion RotationQuaternion { get; private set; } = Quaternion.Identity;

        // Propiedades de movimiento
        public Vector3 Position { get; private set; }
        public float Rotation { get; private set; }
        public float Scale { get; private set; }

        // Agregar estas propiedades si no las tienes
        public float PitchRotation { get; private set; } // Inclinación hacia adelante/atrás
        public float RollRotation { get; private set; } // Inclinación lateral

        // ===== Control por fuerzas (tuneables) =====
        public float EngineAccel = 20f; // m/s^2 (empuje hacia adelante)
        public float BrakeAccel = 20f; // m/s^2 (empuje hacia atrás)
        public float MaxSpeed = 100f; // m/s (límite velocidad plano XZ)
        public float LateralGrip = 12f; // 1/s (mata deriva lateral)
        public float LinearDrag = 0.6f; // 1/s (freno aerodinámico simple)
        
        public Buffer<ConstraintHandle> LeftMotors;
        public Buffer<ConstraintHandle> RightMotors;
        
        public float TurnAccelYaw = 8.0f; // rad/s^2 (acel. angular deseada)
        public float MaxYawRate = 2.8f; // rad/s (límite giro)
        public float YawInertia = 350f; // kg·m^2 (fallback si no querés usar tensor)

        public readonly float ColliderWidth = 6f;
        public readonly float ColliderHeight = 4f;
        public readonly float ColliderLength = 7f;

        // Pequeño “clearance” para evitar nacer en penetración
        public float SpawnClearance = 0.05f;

        // Parámetros de movimiento
        private const float MovementSpeed = 200f;
        private const float _steerSpeed = 90f;
        float maxSteer = 45f;
        float minSteer = -45f;


        // Física
        public BodyHandle PhysicsBody;
        private Simulation _simulation;
        private Terrain _terrain;

        // --- Debug / Telemetría ---
        public bool DebugTelemetry = false;
        public float SteerSign = -1f; // ← por defecto invierto A/D (cámbialo en runtime con F4)
        private float _telemetryTimer = 0f;
        public string TelemetryText = "";
        public QuickList<BodyHandle> bodyHandles;

        // Recoil
        private float _recoilTime = 0f;
        private float _recoilDuration = 0.12f; // seg: cuánto dura el empujón
        private System.Numerics.Vector3 _recoilAccelSys = System.Numerics.Vector3.Zero;

        // Brake (freno por “arrastre”)
        private float _brakeTime = 0f;
        private float _brakeDuration = 0.18f; // seg
        private float _brakeK = 10f; // coeficiente de frenado (tunable)

        private Camera _camera;
        
        public float VisualYOffset = 85.0f;
        public float VisualZOffset = 25f;

        // --- Helpers para extraer pos/axes de una Matrix ---
        private static Vector3 GetTranslation(in Matrix m) => new Vector3(m.M41, m.M42, m.M43);
        private static Vector3 GetRight(in Matrix m) => new Vector3(m.M11, m.M12, m.M13);
        private static Vector3 GetUp(in Matrix m) => new Vector3(m.M21, m.M22, m.M23);
        private static Vector3 GetForward(in Matrix m) => new Vector3(m.M31, m.M32, m.M33);


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

        public Compound bodyShapeCompound;
        
        public System.Numerics.Vector3 AimDirectionWorld { get; set; } = new System.Numerics.Vector3(0, 0, 1);


        public Tank(Vector3 initialPosition, Camera camera, float initialRotation = 0f, float scale = 1f)
        {
            Position = initialPosition;
            Rotation = initialRotation;
            Scale = scale;
            _lastPos = Position;
            _camera = camera;
        }
        
        /// <summary>
        /// Computes the swivel and pitch angles required to aim in a given direction based on the tank's current pose.
        /// </summary>
        /// <param name="simulation">Simulation containing the tank.</param>
        /// <param name="aimDirection">Direction to aim in.</param>
        /// <returns>Swivel and pitch angles to point in the given direction.</returns>
        public (float targetSwivelAngle, float targetPitchAngle) ComputeTurretAngles(Simulation simulation, Vector3 aimDirection)
        {
            //Decompose the aim direction into target angles for the turret and barrel servos.
            //First, we need to compute the frame of reference and transform the aim direction into the tank's local space.
            //aimDirection * inverse(body.Pose.Orientation) * Tank.LocalBodyPose.Orientation * inverse(Tank.TurretBasis)
            QuaternionEx.ConcatenateWithoutOverlap(QuaternionEx.Conjugate(simulation.Bodies[Body].Pose.Orientation), FromBodyLocalToTurretBasisLocal, out var toTurretBasis);
            //-Z in the turret basis points along the 0 angle direction for both swivel and pitch.
            //+Y is 90 degrees for pitch.
            //+X is 90 degres for swivel.
            //We'll compute the swivel angle first.
            var aimdirection = new System.Numerics.Vector3(aimDirection.X, aimDirection.Y, aimDirection.Z);
            QuaternionEx.TransformWithoutOverlap(aimdirection, toTurretBasis, out var aimDirectionInTurretBasis);
            var targetSwivelAngle = MathF.Atan2(aimDirectionInTurretBasis.X, -aimDirectionInTurretBasis.Z);

            //Barrel pitching is measured against the +Y axis and an axis created from the target swivel angle.
            var targetPitchAngle = MathF.Asin(MathF.Max(-1f, MathF.Min(1f, -aimDirectionInTurretBasis.Y)));
            return (targetSwivelAngle, targetPitchAngle);
        }
        
        /// <summary>
        /// Applies a target swivel and pitch angle to the turret's servos.
        /// </summary>
        /// <param name="simulation">Simulation containing the tank.</param>
        /// <param name="targetSwivelAngle">Target swivel angle of the turret.</param>
        /// <param name="targetPitchAngle">Target pitch angle of the barrel.</param>
        public void SetAim(Simulation simulation, float targetSwivelAngle, float targetPitchAngle)
        {
            var turretDescription = TurretServoDescription;
            turretDescription.TargetAngle = targetSwivelAngle;
            simulation.Solver.ApplyDescription(TurretServo, turretDescription);
            var barrelDescription = BarrelServoDescription;
            barrelDescription.TargetAngle = targetPitchAngle;
            simulation.Solver.ApplyDescription(BarrelServo, barrelDescription);

        }
        
        public void SetSpeed(Buffer<ConstraintHandle> motors, float speed, float maximumForce)
        {
            //This sets all properties of a motor at once; it's possible to create a custom description that only assigns a subset of properties if you find this to be somehow expensive.
            var motorDescription = new AngularAxisMotor
            {
                //Assuming the wheels are cylinders oriented in the obvious way.
                LocalAxisA = new System.Numerics.Vector3(0, -1, 0),
                Settings = new MotorSettings(maximumForce, 1e-6f),
                TargetVelocity = speed
            };
            
            for (int i = 0; i < motors.Length; ++i)
            {
                _simulation.Solver.ApplyDescription(motors[i], motorDescription);
            }
        }
        
        public void CargarModelo(string rutaRelativa, Effect efecto, ContentManager content, Simulation simulation, BufferPool bufferPool,
            GraphicsDevice graphicsDevice,Gizmos gizmos, CollidableProperty<TankBodyProperties> properties, Terrain terrain = null)
        {
            Gizmos = gizmos;
            Gizmos.LoadContent(graphicsDevice, new ContentManager(content.ServiceProvider, "Content"));
            _effect = efecto;
            _simulation = simulation;
            _terrain = terrain; // Guardar referencia al terreno
            bodyHandles = new QuickList<BodyHandle>(11, bufferPool);

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
            _wheelBones = new ModelBone[16];

            for (int i = 0; i < 16; i++)
            {
                string boneName = $"Wheel{i + 1}";
                _wheelBones[i] = _model.Bones[boneName];
            }

            _turretBone = _model.Bones["Turret"];
            _cannonBone = _model.Bones["Cannon"];
            _cannonBone.Parent = _turretBone;
            
            foreach (var bone in _model.Bones)
            {
                Console.WriteLine($"Bone: {bone.Name}");
            }


            // Store the original transform matrix for each animating bone.
            
            _wheelTransforms = new Matrix[16];

            for (int i = 0; i < 16; i++)
            {
                _wheelTransforms[i] = _wheelBones[i].Transform;
            }
            _turretTransform = _turretBone.Transform;
            _cannonTransform = _cannonBone.Transform;
            

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
            CreatePhysicsBody(bufferPool, properties);
            
            // Calcular matriz inicial del mundo
            UpdateWorldMatrix();
        }
        
        static ref SubgroupCollisionFilter CreatePart(Simulation simulation, in TankPartDescription part, RigidPose pose, CollidableProperty<TankBodyProperties> properties, ref QuickList<BodyHandle> bodyhandles, out BodyHandle handle)
        {
            RigidPose.MultiplyWithoutOverlap(part.Pose, pose, out var bodyPose);
            handle = simulation.Bodies.Add(BodyDescription.CreateDynamic(bodyPose, part.Inertia, part.Shape, 0.01f));
            bodyhandles.AllocateUnsafely() = handle;
            ref var partProperties = ref properties.Allocate(handle);
            partProperties = new TankBodyProperties { Friction = part.Friction, TankPart = true };
            return ref partProperties.Filter;
        }
        
        static BodyHandle CreateWheel(Simulation simulation, CollidableProperty<TankBodyProperties> properties, in RigidPose tankPose, in RigidPose bodyLocalPose,
        TypedIndex wheelShape, BodyInertia wheelInertia, float wheelFriction, BodyHandle bodyHandle, ref SubgroupCollisionFilter bodyFilter, System.Numerics.Vector3 bodyToWheelSuspension, float suspensionLength,
        in SpringSettings suspensionSettings, System.Numerics.Quaternion localWheelOrientation,
        ref QuickList<BodyHandle> wheelHandles, ref QuickList<ConstraintHandle> constraints, ref QuickList<ConstraintHandle> motors, ref QuickList<BodyHandle> bodyhandles)
        {
            RigidPose wheelPose;
            QuaternionEx.TransformUnitX(localWheelOrientation, out var suspensionDirection);
            RigidPose.Transform(bodyToWheelSuspension + suspensionDirection * suspensionLength, tankPose, out wheelPose.Position);
            QuaternionEx.ConcatenateWithoutOverlap(localWheelOrientation, tankPose.Orientation, out wheelPose.Orientation);
            
            var wheelHandle = simulation.Bodies.Add(BodyDescription.CreateDynamic(wheelPose, wheelInertia, wheelShape, 0.01f));
            wheelHandles.AllocateUnsafely() = wheelHandle;
            bodyhandles.AllocateUnsafely() = wheelHandle;

            //We need a LinearAxisServo to act as the suspension spring, pushing the wheel down.
            constraints.AllocateUnsafely() = simulation.Solver.Add(bodyHandle, wheelHandle, new LinearAxisServo
            {
                LocalPlaneNormal = suspensionDirection,
                TargetOffset = suspensionLength,
                LocalOffsetA = bodyToWheelSuspension,
                LocalOffsetB = default,
                ServoSettings = ServoSettings.Default,
                SpringSettings = suspensionSettings
            });
            //A PointOnLineServo keeps the wheel on a fixed track. Note that it does not constrain the angular behavior of the wheel at all.
            constraints.AllocateUnsafely() = simulation.Solver.Add(bodyHandle, wheelHandle, new PointOnLineServo
            {
                LocalDirection = suspensionDirection,
                LocalOffsetA = bodyToWheelSuspension,
                LocalOffsetB = default,
                ServoSettings = ServoSettings.Default,
                SpringSettings = new SpringSettings(30, 1)
            });
            //The angular component is handled by a hinge. Note that we only use the angular component of a hinge constraint here- the PointOnLineServo handles the linear degrees of freedom.
            //We're assuming the wheels will be cylinders. Pretty safe bet. A cylinder rolls around its local Y axis, so the motor will act along that axis.
            QuaternionEx.TransformUnitY(localWheelOrientation, out var wheelRotationAxis);
            constraints.AllocateUnsafely() = simulation.Solver.Add(bodyHandle, wheelHandle, new AngularHinge
            {
                LocalHingeAxisA = QuaternionEx.Transform(wheelRotationAxis, QuaternionEx.Conjugate(bodyLocalPose.Orientation)),
                LocalHingeAxisB = new System.Numerics.Vector3(0, 1, 0),
                SpringSettings = new SpringSettings(30, 1)
            });
            //We'll need a velocity motor to actually make the tank move.
            var motorHandle = simulation.Solver.Add(wheelHandle, bodyHandle, new AngularAxisMotor
            {
                //(All these are technically set on the fly during the update right now, but a custom constraint description could set only the Settings and TargetVelocity,
                //leaving the LocalAxisA unchanged, so we'll go ahead and set it to a reasonable value.)
                LocalAxisA = new System.Numerics.Vector3(0, 1, 0),
                Settings = default,
                TargetVelocity = default
            });
            motors.AllocateUnsafely() = motorHandle;
            constraints.AllocateUnsafely() = motorHandle;
            ref var wheelProperties = ref properties.Allocate(wheelHandle);
            wheelProperties = new TankBodyProperties { Filter = new SubgroupCollisionFilter(bodyHandle.Value, 3), Friction = wheelFriction, TankPart = true };
            //The wheels don't need to be tested against the body or each other.
            SubgroupCollisionFilter.DisableCollision(ref wheelProperties.Filter, ref bodyFilter);
            SubgroupCollisionFilter.DisableCollision(ref wheelProperties.Filter, ref wheelProperties.Filter);
            
            
            
            return wheelHandle;
        }
        
        private void CreatePhysicsBody(BufferPool bufferPool, CollidableProperty<TankBodyProperties> properties)
        {
            /*
            var builder = new CompoundBuilder(bufferPool, _simulation.Shapes, 2);
            builder.Add(new Box(50f, 27f, 62f), new RigidPose(new System.Numerics.Vector3(0f, -10f, -4f)), 10);
            builder.Add(
                new Cylinder(9f, 55f),
                new RigidPose(new System.Numerics.Vector3(0f, -23f, 22f), 
                    System.Numerics.Quaternion.CreateFromAxisAngle(System.Numerics.Vector3.UnitZ, MathF.PI / 2) // rotar para que gire sobre X
                ), 3);
            builder.Add(
                new Cylinder(12f, 59f),
                new RigidPose(new System.Numerics.Vector3(0f, -20f, -26f), 
                    System.Numerics.Quaternion.CreateFromAxisAngle(System.Numerics.Vector3.UnitZ, MathF.PI / 2)
                ), 5);
            builder.BuildDynamicCompound(out var children, out var bodyInertia, out _);
            builder.Dispose();
            var bodyShapeCompound = new Compound(children);
            var bodyShapeIndex = _simulation.Shapes.Add(bodyShapeCompound);
            _debugCompoundShape = bodyShapeCompound;
            */
            
            var wheelShape = new Cylinder(3.4f, 1f);
            var wheelInertia = wheelShape.ComputeInertia(0.25f);
            var wheelShapeIndex = _simulation.Shapes.Add(wheelShape);
            
            var tankDescription = new TankDescription
            {
                Body = TankPartDescription.Create(1, new Box(36f, 9, 60), new RigidPose(new System.Numerics.Vector3(0, 0, 0), System.Numerics.Quaternion.Identity), 0.5f, _simulation.Shapes),
                Turret = TankPartDescription.Create(1, new Cylinder(15f, 7f), new System.Numerics.Vector3(0, 8.5f, 4f), 0.5f, _simulation.Shapes),
                Barrel = TankPartDescription.Create(0.5f, new Box(2f, 2f, 30f), new System.Numerics.Vector3(0, 8.5f, 4f - 10f - 15f), 0.5f, _simulation.Shapes),
                TurretAnchor = new System.Numerics.Vector3(0f, 0.5f, 0.4f),
                BarrelAnchor = new System.Numerics.Vector3(0, 0.5f + 0.35f, 0.4f - 1f),
                TurretBasis = System.Numerics.Quaternion.CreateFromAxisAngle(
                    System.Numerics.Vector3.UnitY, MathF.PI),
                TurretServo = new ServoSettings(1e7f, 0f, 1000),
                TurretSpring = new SpringSettings(10f, 10.0f),
                BarrelServo = new ServoSettings(1e7f, 0f, 4000),
                BarrelSpring = new SpringSettings(10f, 10.0f),

                LeftTreadOffset = new System.Numerics.Vector3(-15f, 0f, 0),
                RightTreadOffset = new System.Numerics.Vector3(15f, 0f, 0),
                SuspensionLength = 3f,
                SuspensionSettings = new SpringSettings(5f, 3f),
                WheelShape = wheelShapeIndex,
                WheelInertia = wheelInertia,
                WheelFriction = 3f,
                TreadSpacing = 8f,
                WheelCountPerTread = 6,
                WheelOrientation = QuaternionEx.CreateFromAxisAngle(System.Numerics.Vector3.UnitZ, MathF.PI * -0.5f),
            };
            
            if (_simulation == null) return;

            // Altura del suelo en (X,Z)
            float terrainY = _terrain != null ? _terrain.GetHeightAtPosition(Position.X, Position.Z) : Position.Y;
            
            // Calcular orientación inicial según el terreno
            CalculateTerrainRotation(out var orientationMatrix);
            var orientationQuat = Quaternion.CreateFromRotationMatrix(orientationMatrix);

            System.Diagnostics.Debug.WriteLine($"[TANK] YawInertia computed = {YawInertia:0.##}");
            
            //float lowestY = GetLowestYFromCompound(bodyShapeCompound);
            //float spawnY = terrainY - lowestY + SpawnClearance + 100;
            
            // Pose inicial
            var pose = new RigidPose(
                new System.Numerics.Vector3(Position.X + 1300, terrainY + 200 , Position.Z),
                new System.Numerics.Quaternion(orientationQuat.X, orientationQuat.Y, orientationQuat.Z,
                    orientationQuat.W)
            );
            /*
            var material = new 
            {
                FrictionCoefficient = 0.8f, // ← acá definís la fricción
                MaximumRecoveryVelocity = 2f,
                SpringSettings = new SpringSettings(30, 1)
            };
             */
            
            var wheelHandles = new QuickList<BodyHandle>(tankDescription.WheelCountPerTread * 2, bufferPool);
            var constraints = new QuickList<ConstraintHandle>(tankDescription.WheelCountPerTread * 2 * 6 + 4, bufferPool);
            var leftMotors = new QuickList<ConstraintHandle>(tankDescription.WheelCountPerTread, bufferPool);
            var rightMotors = new QuickList<ConstraintHandle>(tankDescription.WheelCountPerTread, bufferPool);
            
            ref var bodyFilter = ref CreatePart(_simulation, tankDescription.Body, pose, properties, ref bodyHandles, out Body);
            ref var turretFilter = ref CreatePart(_simulation, tankDescription.Turret, pose, properties, ref bodyHandles, out Turret);
            ref var barrelFilter = ref CreatePart(_simulation, tankDescription.Barrel, pose, properties, ref bodyHandles, out Barrel);
            
            bodyFilter = new SubgroupCollisionFilter(Body.Value, 0);
            turretFilter = new SubgroupCollisionFilter(Body.Value, 1);
            barrelFilter = new SubgroupCollisionFilter(Body.Value, 2);
            SubgroupCollisionFilter.DisableCollision(ref bodyFilter, ref turretFilter);
            SubgroupCollisionFilter.DisableCollision(ref turretFilter, ref barrelFilter);
            
            Matrix3x3.CreateFromQuaternion(tankDescription.TurretBasis, out var turretBasis);

            //Attach the turret to the body.
            QuaternionEx.Transform(turretBasis.Y, QuaternionEx.Conjugate(tankDescription.Body.Pose.Orientation), out var bodyLocalSwivelAxis);
            QuaternionEx.Transform(turretBasis.Y, QuaternionEx.Conjugate(tankDescription.Turret.Pose.Orientation), out var turretLocalSwivelAxis);
            RigidPose.TransformByInverse(tankDescription.TurretAnchor, tankDescription.Body.Pose, out var bodyLocalTurretAnchor);
            RigidPose.TransformByInverse(tankDescription.TurretAnchor, tankDescription.Turret.Pose, out var turretLocalTurretAnchor);
            constraints.AllocateUnsafely() = _simulation.Solver.Add(Body, Turret,
                new Hinge
                {
                    LocalHingeAxisA = bodyLocalSwivelAxis,
                    LocalHingeAxisB = turretLocalSwivelAxis,
                    LocalOffsetA = bodyLocalTurretAnchor,
                    LocalOffsetB = turretLocalTurretAnchor,
                    SpringSettings = new SpringSettings(30, 1)
                });
            
            Matrix3x3 turretSwivelBasis;
            turretSwivelBasis.Z = -turretBasis.Y;
            turretSwivelBasis.X = -turretBasis.Z;
            turretSwivelBasis.Y = turretBasis.X;
            //Debug.Assert(turretSwivelBasis.Determinant() > 0.999f && turretSwivelBasis.Determinant() < 1.0001f, "The turret swivel axis and forward axis should be perpendicular and unit length.");
            QuaternionEx.CreateFromRotationMatrix(turretSwivelBasis, out var turretSwivelBasisQuaternion);
            QuaternionEx.ConcatenateWithoutOverlap(turretSwivelBasisQuaternion, QuaternionEx.Conjugate(tankDescription.Body.Pose.Orientation), out var bodyLocalTurretBasis);
            QuaternionEx.ConcatenateWithoutOverlap(turretSwivelBasisQuaternion, QuaternionEx.Conjugate(tankDescription.Turret.Pose.Orientation), out var turretLocalTurretBasis);
            TurretServoDescription = new TwistServo
            {
                LocalBasisA = bodyLocalTurretBasis,
                LocalBasisB = turretLocalTurretBasis,
                SpringSettings = tankDescription.TurretSpring,
                ServoSettings = tankDescription.TurretServo
            };
            TurretServo = _simulation.Solver.Add(Body, Turret, TurretServoDescription);
            constraints.AllocateUnsafely() = TurretServo;
            
            
            //Attach the barrel to the turret.
            QuaternionEx.Transform(turretBasis.X, QuaternionEx.Conjugate(tankDescription.Turret.Pose.Orientation), out var turretLocalPitchAxis);
            QuaternionEx.Transform(turretBasis.X, QuaternionEx.Conjugate(tankDescription.Barrel.Pose.Orientation), out var barrelLocalPitchAxis);
            RigidPose.TransformByInverse(tankDescription.BarrelAnchor, tankDescription.Turret.Pose, out var turretLocalBarrelAnchor);
            RigidPose.TransformByInverse(tankDescription.BarrelAnchor, tankDescription.Barrel.Pose, out var barrelLocalBarrelAnchor);
            constraints.AllocateUnsafely() = _simulation.Solver.Add(Turret, Barrel,
                new Hinge
                {
                    LocalHingeAxisA = turretLocalPitchAxis,
                    LocalHingeAxisB = barrelLocalPitchAxis,
                    LocalOffsetA = turretLocalBarrelAnchor,
                    LocalOffsetB = barrelLocalBarrelAnchor,
                    SpringSettings = new SpringSettings(30, 1)
                });
            //The twist servo might seem like an odd choice to control 1 angular degree of freedom, but servo-like control over 1DOF requires a measurement basis to interpret the target angle.
            //Hence the apparent complexity.
            Matrix3x3 barrelPitchBasis;
            barrelPitchBasis.Z = -turretBasis.X;
            barrelPitchBasis.X = -turretBasis.Z;
            barrelPitchBasis.Y = -turretBasis.Y;
            //Debug.Assert(barrelPitchBasis.Determinant() > 0.999f && barrelPitchBasis.Determinant() < 1.0001f, "The barrel axis and forward axis should be perpendicular and unit length.");
            QuaternionEx.CreateFromRotationMatrix(barrelPitchBasis, out var barrelPitchBasisQuaternion);
            QuaternionEx.ConcatenateWithoutOverlap(barrelPitchBasisQuaternion, QuaternionEx.Conjugate(tankDescription.Turret.Pose.Orientation), out var turretLocalBarrelBasis);
            QuaternionEx.ConcatenateWithoutOverlap(barrelPitchBasisQuaternion, QuaternionEx.Conjugate(tankDescription.Barrel.Pose.Orientation), out var barrelLocalBarrelBasis);
            BarrelServoDescription = new TwistServo
            {
                LocalBasisA = turretLocalBarrelBasis,
                LocalBasisB = barrelLocalBarrelBasis,
                SpringSettings = tankDescription.BarrelSpring,
                ServoSettings = tankDescription.BarrelServo
            };
            BarrelServo = _simulation.Solver.Add(Turret, Barrel, BarrelServoDescription);
            constraints.AllocateUnsafely() = BarrelServo;
            
            QuaternionEx.TransformUnitY(tankDescription.WheelOrientation, out var wheelAxis);
            QuaternionEx.TransformUnitZ(tankDescription.WheelOrientation, out var treadDirection);
            var treadStart = tankDescription.TreadSpacing * (tankDescription.WheelCountPerTread - 1) * -0.5f;
            BodyHandle previousLeftWheelHandle = default, previousRightWheelHandle = default;
            for (int i = 0; i < tankDescription.WheelCountPerTread; ++i)
            {
                var wheelOffsetFromTread = treadDirection * (treadStart + i * tankDescription.TreadSpacing);
                var rightWheelHandle = CreateWheel(_simulation, properties, pose, tankDescription.Body.Pose,
                    tankDescription.WheelShape, tankDescription.WheelInertia, tankDescription.WheelFriction, Body, ref properties[Body].Filter,
                    tankDescription.RightTreadOffset + wheelOffsetFromTread - tankDescription.Body.Pose.Position,
                    tankDescription.SuspensionLength, tankDescription.SuspensionSettings, tankDescription.WheelOrientation,
                    ref wheelHandles, ref constraints, ref rightMotors, ref bodyHandles);
                var leftWheelHandle = CreateWheel(_simulation, properties, pose, tankDescription.Body.Pose,
                    tankDescription.WheelShape, tankDescription.WheelInertia, tankDescription.WheelFriction, Body, ref properties[Body].Filter,
                    tankDescription.LeftTreadOffset + wheelOffsetFromTread - tankDescription.Body.Pose.Position,
                    tankDescription.SuspensionLength, tankDescription.SuspensionSettings, tankDescription.WheelOrientation,
                    ref wheelHandles, ref constraints, ref leftMotors, ref bodyHandles);

                if (i >= 1)
                {
                    //Connect wheels in a tread to each other to distribute the drive forces.
                    //The motor will always just target 0 velocity. The wheel orientations will be allowed to drift from each other.
                    //(If you didn't want to allow drift, you could use an AngularServo or TwistServo. AngularServo constrains all 3 degrees of freedom, but for these purposes, that'd be fine.)
                    var motorDescription = new AngularAxisMotor { LocalAxisA = new System.Numerics.Vector3(0, 1, 0), Settings = new MotorSettings(float.MaxValue, 1e-4f) };
                    constraints.AllocateUnsafely() = _simulation.Solver.Add(previousLeftWheelHandle, leftWheelHandle, motorDescription);
                    constraints.AllocateUnsafely() = _simulation.Solver.Add(previousRightWheelHandle, rightWheelHandle, motorDescription);
                }
                previousLeftWheelHandle = leftWheelHandle;
                previousRightWheelHandle = rightWheelHandle;

            }
            
            WheelHandles = wheelHandles.Span.Slice(wheelHandles.Count);
            Constraints = constraints.Span.Slice(constraints.Count);
            LeftMotors = leftMotors.Span.Slice(leftMotors.Count);
            RightMotors = rightMotors.Span.Slice(rightMotors.Count);
            
            QuaternionEx.ConcatenateWithoutOverlap(tankDescription.Body.Pose.Orientation, QuaternionEx.Conjugate(tankDescription.TurretBasis), out FromBodyLocalToTurretBasisLocal);
            BodyLocalOrientation = tankDescription.Body.Pose.Orientation;
            QuaternionEx.Transform(-turretBasis.Z, QuaternionEx.Conjugate(tankDescription.Barrel.Pose.Orientation), out BarrelLocalDirection);
            /*
            var velocity = new BodyVelocity(System.Numerics.Vector3.Zero, System.Numerics.Vector3.Zero);
            var collidable = new CollidableDescription(bodyShapeIndex, 0.25f);
            var activity = new BodyActivityDescription(0.01f);
            var bodyDesc = BodyDescription.CreateDynamic(pose, velocity, bodyInertia, collidable, activity);
            _physicsBody = _simulation.Bodies.Add(bodyDesc);
            */
            RotationQuaternion = orientationQuat;

        }
        
        private float GetLowestYFromCompound(Compound compoundShape)
        {
            float lowestY = float.MaxValue;

            for (int i = 0; i < compoundShape.Children.Length; i++)
            {
                ref var child = ref compoundShape.Children[i];
                var localPose = child.LocalPose;

                // Obtener bounds del shape
                _simulation.Shapes.UpdateBounds(localPose, ref child.ShapeIndex, out var bounds);

                // Punto más bajo en espacio local
                float localBottomY = bounds.Min.Y;

                // Transformar al espacio del cuerpo (solo rotación + posición local)
                var rotatedOffset = System.Numerics.Vector3.Transform(new System.Numerics.Vector3(0, localBottomY, 0), localPose.Orientation);
                float worldOffsetY = localPose.Position.Y + rotatedOffset.Y;

                if (worldOffsetY < lowestY)
                    lowestY = worldOffsetY;
            }

            return lowestY;
        }
        
        public void ControlForces(float throttle, float steer, float dt)
        {
            if (_simulation == null || PhysicsBody.Value < 0) return;
            var body = _simulation.Bodies.GetBodyReference(PhysicsBody);

            // --- Ejes del tanque en mundo (modelo mira +Z) ---
            var q = body.Pose.Orientation;

            // up local del cuerpo
            var up = System.Numerics.Vector3.Transform(new System.Numerics.Vector3(0, 1, 0), q);
            up = up.LengthSquared() > 1e-12f
                ? System.Numerics.Vector3.Normalize(up)
                : new System.Numerics.Vector3(0, 1, 0);

            // forward proyectado al plano perpendicular a up
            var fwdRaw = System.Numerics.Vector3.Transform(new System.Numerics.Vector3(0, 0, 1), q);
            var fwd = fwdRaw - System.Numerics.Vector3.Dot(fwdRaw, up) * up;
            fwd = fwd.LengthSquared() < 1e-12f ? new System.Numerics.Vector3(0, 0, 1) : System.Numerics.Vector3.Normalize(fwd);

            // right ortogonal
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
            float accel = (throttle >= 0f ? EngineAccel : BrakeAccel) * throttle ; // [-1..1]
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
                        $"Position X{_lastPos.X:+0.00;-0.00;0} Y{_lastPos.Y:+0.00;-0.00;0} Z{_lastPos.Z:+0.0;-0.0;0}\n" +
                        $"thr {throttle:+0.00;-0.00;0}  steer {steer:+0.00;-0.00;0}  sign {SteerSign:+0.0;-0.0;0}\n" +
                        $"speed {speed:0.00} m/s ({kmh:0} km/h)   vSide {vSide:+0.00;-0.00;0}\n" +
                        $"yawNow {yawNow:+0.00;-0.00;0} rad/s   yawTgt {yawTarget:+0.00;-0.00;0}   d {dOmega:+0.00;-0.00;0}";

                    System.Diagnostics.Debug.WriteLine("[TANK] " + TelemetryText);
                }
            }
        }

        public void Update(GameTime gameTime, KeyboardState keyboardState,MouseState mouseState,Vector3 cameraForward)
        {
            var dt = (float)gameTime.ElapsedGameTime.TotalSeconds;

            var body = _simulation.Bodies.GetBodyReference(PhysicsBody);
            body.Awake = true;

            //var steer = 0f;
            if (keyboardState.IsKeyDown(Keys.A)) SteerRotation += _steerSpeed * dt;
            else if(keyboardState.IsKeyDown(Keys.D))  SteerRotation -= _steerSpeed * dt;
            else{ SteerRotation = MathHelper.Lerp(SteerRotation, 0f, dt * 5f); }
            
            SteerRotation = Math.Clamp(SteerRotation, minSteer, maxSteer);

            // Girar ruedas según distancia recorrida
            UpdateWheelSpinByDistance(dt);
            
            UpdateCanonAndTurretTowards(new Vector3(AimDirectionWorld.X, AimDirectionWorld.Y, AimDirectionWorld.Z), dt);
            
            UpdateWorldMatrix();
        }

        public void UpdateCanonAndTurretTowards(Vector3 worldAimDir, float dt)
        {

            // 1) Leer las poses actuales del cuerpo, torreta y cañón desde la simulación
            var bodyRef   = _simulation.Bodies.GetBodyReference(Body);
            var turretRef = _simulation.Bodies.GetBodyReference(Turret);
            var barrelRef = _simulation.Bodies.GetBodyReference(Barrel);

            var qBody   = bodyRef.Pose.Orientation;
            var qBarrel = barrelRef.Pose.Orientation;

            // 2) Dirección "forward" del cañón físico en MUNDO
            //    (BarrelLocalDirection ya es el forward del cañón en su espacio local)
            System.Numerics.Vector3 fwdWorldN;
            QuaternionEx.Transform(BarrelLocalDirection, qBarrel, out fwdWorldN);

            // Normalizar por seguridad
            var len2 = fwdWorldN.LengthSquared();
            if (len2 < 1e-12f) return;
            fwdWorldN /= MathF.Sqrt(len2);

            // 3) Yaw del tanque (desde la física). Ojo: tu tanque "mira" -Z.
            var bodyFwd = System.Numerics.Vector3.Transform(new System.Numerics.Vector3(0, 0, -1), qBody);
            var tankYaw = MathF.Atan2(bodyFwd.X, bodyFwd.Z) + MathF.PI; 
            // 4) Yaw de la torreta a partir del cañón físico
            var flat = new System.Numerics.Vector3(fwdWorldN.X, 0, fwdWorldN.Z);
            var flatLen2 = flat.LengthSquared();
            if (flatLen2 < 1e-12f) return;
            flat /= MathF.Sqrt(flatLen2);

            var globalTurretYaw = MathF.Atan2(flat.X, flat.Z);

            // Diferencia de yaw entre el cuerpo y el cañón físico
            var swivel = MathHelper.WrapAngle(globalTurretYaw - tankYaw);

            // 5) Pitch del cañón físico (misma convención que venías usando)
            var pitch = -MathF.Asin(Math.Clamp(fwdWorldN.Y, -1f, 1f));

            // 6) Asignar directo a los ángulos visuales
            //    Nota: tu rig dibuja yaw en Z con signo invertido en Draw(): Matrix.CreateRotationZ(-TurretRotation)
            //    Mantenemos TurretRotation = swivel y CannonRotation = -pitch para respetar ese rig.
            TurretRotation = swivel;
            CannonRotation = pitch;
        }
        private float InterpolateAngle(float current, float target, float maxStep)
        {
            float delta = MathHelper.WrapAngle(target - current);
            delta = Math.Clamp(delta, -maxStep, maxStep);
            return MathHelper.WrapAngle(current + delta);
        }
        
        private static float GetTankYawFromQuaternion(Quaternion q)
        {
            // Extrae el yaw (rotación sobre Y) del quaternion
            Vector3 forward = Vector3.Transform(-Vector3.UnitZ, q);
            return MathF.Atan2(forward.X, forward.Z);
        }
        
        private void UpdateWheelSpinByDistance(float dt)
        {
            var delta = Position - _lastPos;
            var dist = delta.Length();

            // Dirección “forward” actual para signo (+ avanza / - retrocede)
            var forward = Vector3.Transform(-Vector3.UnitZ, Matrix.CreateFromQuaternion(RotationQuaternion));
            float sign = 0f;
            if (dist > 0.0001f)
            {
                var dir = Vector3.Normalize(delta);
                sign = MathF.Sign(Vector3.Dot(dir, forward));
            }
            
            WheelRotation += sign * (dist / (WheelRadius ));

            if (WheelRotation > MathHelper.TwoPi) WheelRotation -= MathHelper.TwoPi;
            else if (WheelRotation < -MathHelper.TwoPi) WheelRotation += MathHelper.TwoPi;

            _lastPos = Position;
        }

        public void SyncFromPhysics()
        {
            var body = _simulation.Bodies.GetBodyReference(PhysicsBody);
            var pose = body.Pose;

            Position = new Vector3(pose.Position.X, pose.Position.Y, pose.Position.Z);
            var q = pose.Orientation;
            RotationQuaternion = new Quaternion(q.X, q.Y, q.Z, q.W);
        }

        private void UpdateWorldMatrix()
        {
            // Sincronizar posición y rotación desde la física
            var bodyReference = _simulation.Bodies.GetBodyReference(PhysicsBody);
            ref var body = ref bodyReference;
            var pose = body.Pose;

            Position = new Vector3(pose.Position.X, pose.Position.Y, pose.Position.Z);
            RotationQuaternion = new Quaternion(pose.Orientation.X, pose.Orientation.Y, pose.Orientation.Z,
                pose.Orientation.W);
            
            // Construir offset visual en espacio local del modelo
            // Lo rotamos por la orientación del cuerpo para llevarlo al espacio mundo
            Vector3 localOffsetScaled = new Vector3(0f, VisualYOffset * Scale, VisualZOffset * Scale);
            // Transformar por la rotación del body
            var offsetWorld = Vector3.Transform(localOffsetScaled, Matrix.CreateFromQuaternion(RotationQuaternion));

            // Posición que usamos para dibujar el modelo
            var visualPosition = Position + offsetWorld;
            
            // Construir la matriz del mundo
            _world =
                Matrix.CreateScale(Scale) *
                Matrix.CreateFromQuaternion(RotationQuaternion) *
                Matrix.CreateTranslation(visualPosition);
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
            var tangentX = new Vector3(2f * h, hR - hL, 0f); // (Δx, Δy, 0)
            var tangentZ = new Vector3(0f, hU - hD, 2f * h); // (0, Δy, Δz)

            // Normal “up” del terreno
            var up = Vector3.Normalize(
                Vector3.Cross(tangentZ, tangentX));

            // 2) Direcciones objetivo: mantené la YAW de la física
            var yawForward = Vector3.Transform(
                -Vector3.UnitZ,
                Matrix.CreateRotationY(Rotation));

            // Proyectá el forward sobre el plano del terreno para que siga la pendiente
            var forwardOnPlane = yawForward - Vector3.Dot(yawForward, up) * up;
            if (forwardOnPlane.LengthSquared() < 1e-6f)
                forwardOnPlane = Vector3.Normalize(
                    Vector3.Cross(
                        new Vector3(1, 0, 0), up));
            else
                forwardOnPlane.Normalize();

            var right = Vector3.Normalize(
                Vector3.Cross(forwardOnPlane, up));
            var forward = Vector3.Normalize(
                Vector3.Cross(up, right));

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
            var turretRotation = Matrix.CreateRotationZ(TurretRotation);
            var cannonRotation = Matrix.CreateRotationX(CannonRotation);
            
            
            for (int i = 0; i < 16; i++)
            {
                _wheelBones[i].Transform = wheelRotation * _wheelTransforms[i]; 
            }
            
            _turretBone.Transform = turretRotation * _turretTransform;
            _cannonBone.Transform = cannonRotation * _cannonTransform;
            
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
        
        // Devuelve posición y dirección de la boca del cañón, tomando el hueso real del cañón
        public (Vector3 pos, Vector3 dir) GetMuzzle(float muzzleOffsetLocal = 300.2f)
        {
            var turretRotation = Matrix.CreateRotationZ(TurretRotation);
            var cannonRotation = Matrix.CreateRotationX(CannonRotation); // mismo eje/signo que Draw
            _turretBone.Transform = turretRotation * _turretTransform;
            _cannonBone.Transform = cannonRotation * _cannonTransform;
            
            if (_boneTransforms == null || _boneTransforms.Length != _model.Bones.Count)
                _boneTransforms = new Matrix[_model.Bones.Count];

            _model.CopyAbsoluteBoneTransformsTo(_boneTransforms);

            // 3) Transform del cañón en espacio mundo
            var cannonAbs = _boneTransforms[_cannonBone.Index];
            var cannonWorld = cannonAbs * _world; // mismo patrón que en Draw()

            // 4) La dirección “forward” del cañón (en el modelo, -Z es hacia adelante del tanque)
            //    Recordá que en matrices de XNA la tercera fila es el "forward" local del transform.
            //    Según tu pipeline, el tanque mira -Z: usamos -Forward del hueso.
            var f = -Vector3.Normalize(GetUp(cannonWorld));

            // 5) Posición del muzzle: origen del hueso + un corrimiento a lo largo del cañón
            var origin = GetTranslation(cannonWorld);
            var muzzle = origin + f * (muzzleOffsetLocal * Scale);

            return (muzzle, f);
        }

        // Dispara un pulso de retroceso y, opcionalmente, activa freno momentáneo.
        public void TriggerRecoil(Vector3 fireDirXna,
            float projectileMass = 2f,
            float muzzleSpeed = 120f,
            float intensity = 1f,
            bool withBrake = true)
        {
            // Dirección normalizada en espacio mundo (hacia donde sale el disparo)
            var dir = Vector3.Normalize(fireDirXna);

            // “Magnitud” base: momento del proyectil (m * v), escalado un poco
            var recoilMagnitude = projectileMass * muzzleSpeed * 2.0f * intensity;

            // Aceleración de retroceso (la aplicamos por un corto lapso)
            _recoilAccelSys = -new System.Numerics.Vector3(dir.X, dir.Y, dir.Z) * recoilMagnitude;
            _recoilTime = _recoilDuration;

            if (withBrake)
            {
                _brakeTime = _brakeDuration;
            }

            var amplitude = 0.001f * projectileMass * muzzleSpeed; 
            var rotational = amplitude * 0.06f;
            _camera.StartShake(amplitude, 0.12f, rotational);
        }

        public void ApplyRecoilAndBrake(float dt, Simulation simulation)
        {
            // Referencia al cuerpo físico del tanque
            var bodyRef = simulation.Bodies.GetBodyReference(PhysicsBody); // usa tu handle del tanque

            // Retroceso: empuja en dirección opuesta por un tiempo corto
            if (_recoilTime > 0f)
            {
                // Δv = a * dt — aplicamos sobre la velocidad lineal
                bodyRef.Velocity.Linear += _recoilAccelSys * dt;
                _recoilTime -= dt;
            }

            // Freno: arrastre proporcional a la velocidad actual (como un damping temporal)
            if (_brakeTime > 0f)
            {
                var v = bodyRef.Velocity.Linear;
                var drag = -v * (_brakeK * dt); // más K => frena más fuerte
                bodyRef.Velocity.Linear += drag;
                _brakeTime -= dt;
            }
        }
    }
}