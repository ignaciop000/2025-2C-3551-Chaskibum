using System;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using TGC.MonoGame.TP.Viewer.Gizmos;
using Quaternion = Microsoft.Xna.Framework.Quaternion;
using Vector3 = Microsoft.Xna.Framework.Vector3;
using MathHelper = Microsoft.Xna.Framework.MathHelper;
using Matrix = Microsoft.Xna.Framework.Matrix;

namespace TGC.MonoGame.TP 
{
    public class Tank 
    {
        public bool IsDead { get; private set; }
        private Vector3 _lastPos;
        private const float WheelRadius = 2.0f; // ajustá según tu modelo/escala

        private Model _model;
        private Effect _effect;
        private Matrix _world;

        private ModelBone[] _wheelBones;
        private ModelBone _turretBone;
        private ModelBone _cannonBone;

        private BodyHandle _body;
        private BodyHandle _turret;
        private BodyHandle _barrel;

        private Matrix[] _wheelTransforms;
        private Matrix _turretTransform;
        private Matrix _cannonTransform;

        private Matrix[] _boneTransforms;

        private TwistServo _barrelServoDescription;
        private TwistServo _turretServoDescription;
        private ConstraintHandle _turretServo;
        private ConstraintHandle _barrelServo;
        private System.Numerics.Quaternion _fromBodyLocalToTurretBasisLocal;
        private System.Numerics.Vector3 _barrelLocalDirection;

        private Gizmos Gizmos { get; set;}

        private Quaternion RotationQuaternion { get; set; } = Quaternion.Identity;

        // Propiedades de movimiento
        public Vector3 Position { get; private set; }
        public float Rotation { get; }
        private float Scale { get; }

        public float PitchRotation; // Inclinación hacia adelante/atrás
        public float RollRotation; // Inclinación lateral
        
        public Buffer<ConstraintHandle> LeftMotors;
        public Buffer<ConstraintHandle> RightMotors;

        private const float YawInertia = 350f; // kg·m^2 (fallback si no querés usar tensor)

        // Parámetros de movimiento
        private const float SteerSpeed = 90f;
        private const float MaxSteer = 45f;
        private const float MinSteer = -45f;

        // Física
        private Simulation _simulation;
        private Terrain _terrain;

        public QuickList<BodyHandle> BodyHandles;

        // Recoil
        private float _recoilTime;
        private const float RecoilDuration = 0.12f; // seg: cuánto dura el empujón
        private System.Numerics.Vector3 _recoilAccelSys = System.Numerics.Vector3.Zero;

        // Brake (freno por “arrastre”)
        private float _brakeTime;
        private const float BrakeDuration = 0.18f; // seg
        private const float BrakeK = 10f; // coeficiente de frenado (tunable)

        private readonly Camera _camera;

        private const float VisualYOffset = 85.0f;
        private const float VisualZOffset = 25f;

        // --- Helpers para extraer pos/axes de una Matrix ---
        private static Vector3 GetTranslation(in Matrix m) => new(m.M41, m.M42, m.M43);
        private static Vector3 GetUp(in Matrix m) => new(m.M21, m.M22, m.M23);

        /// <summary>
        /// Gets or sets the rotation of the wheels.
        /// </summary>
        private float WheelRotation { get; set; }

        /// <summary>
        ///     Gets or sets the steering rotation amount.
        /// </summary>
        private float SteerRotation { get; set; }

        /// <summary>
        ///     Gets or sets the turret rotation amount.
        /// </summary>
        private float TurretRotation { get; set; }

        /// <summary>
        ///     Gets or sets the cannon rotation amount.
        /// </summary>
        private float CannonRotation { get; set; }

        public Vector3 AimDirectionWorld { get; set; } = new(0, 0, 1);

        public float Vida = 100f;
        public ProjectileConfig TipoProyectilActual = ProjectilePresets.Light;

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
            QuaternionEx.ConcatenateWithoutOverlap(QuaternionEx.Conjugate(simulation.Bodies[_body].Pose.Orientation), _fromBodyLocalToTurretBasisLocal, out var toTurretBasis);
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
            var turretDescription = _turretServoDescription;
            turretDescription.TargetAngle = targetSwivelAngle;
            simulation.Solver.ApplyDescription(_turretServo, turretDescription);
            var barrelDescription = _barrelServoDescription;
            barrelDescription.TargetAngle = targetPitchAngle;
            simulation.Solver.ApplyDescription(_barrelServo, barrelDescription);

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
            BodyHandles = new QuickList<BodyHandle>(11, bufferPool);

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
            
            // Cargar texturas del T90
            var hullATexture = content.Load<Texture2D>(TGCGame.ContentFolder3D + "t90/textures_mod/hullA");
            var hullBTexture = content.Load<Texture2D>(TGCGame.ContentFolder3D + "t90/textures_mod/hullB");
            var hullCTexture = content.Load<Texture2D>(TGCGame.ContentFolder3D + "t90/textures_mod/hullC");
            var treadmillsTexture = content.Load<Texture2D>(TGCGame.ContentFolder3D + "t90/textures_mod/treadmills");
                
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

            // Asignar efecto y texturas a todas las partes del modelo
            foreach (var mesh in _model.Meshes)
            {
                foreach (var meshPart in mesh.MeshParts)
                {
                    meshPart.Effect = efecto;
                    
                    // Asignar textura según el mesh
                    // Las ruedas usan treadmills, el cuerpo usa hullA/B/C
                    if (mesh.Name.Contains("Wheel") || mesh.Name.Contains("wheel"))
                    {
                        efecto.Parameters["ModelTexture"]?.SetValue(treadmillsTexture);
                    }
                    else if (mesh.Name.Contains("Turret") || mesh.Name.Contains("turret"))
                    {
                        efecto.Parameters["ModelTexture"]?.SetValue(hullBTexture);
                    }
                    else if (mesh.Name.Contains("Cannon") || mesh.Name.Contains("cannon") || mesh.Name.Contains("Barrel"))
                    {
                        efecto.Parameters["ModelTexture"]?.SetValue(hullCTexture);
                    }
                    else
                    {
                        efecto.Parameters["ModelTexture"]?.SetValue(hullATexture);
                    }
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
                TargetVelocity = 0
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
            var wheelShape = new Cylinder(3.4f, 1f);
            var wheelInertia = wheelShape.ComputeInertia(0.25f);
            var wheelShapeIndex = _simulation.Shapes.Add(wheelShape);
            
            var tankDescription = new TankDescription
            {
                Body = TankPartDescription.Create(1, new Box(36f, 9, 60), new RigidPose(new System.Numerics.Vector3(0, 0, 0), System.Numerics.Quaternion.Identity), 0.5f, _simulation.Shapes),
                Turret = TankPartDescription.Create(1, new Cylinder(15f, 7f), new System.Numerics.Vector3(0, 8.5f, 4f), 0.5f, _simulation.Shapes),
                Barrel = TankPartDescription.Create(0.5f, new Box(2f, 2f, 40f), new System.Numerics.Vector3(0, 8.5f, 4f - 10f - 15f), 0.5f, _simulation.Shapes),
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
                SuspensionLength = 4f,
                SuspensionSettings = new SpringSettings(5f, 3f),
                WheelShape = wheelShapeIndex,
                WheelInertia = wheelInertia,
                WheelFriction = 2f,
                TreadSpacing = 8f,
                WheelCountPerTread = 6,
                WheelOrientation = QuaternionEx.CreateFromAxisAngle(System.Numerics.Vector3.UnitZ, MathF.PI * -0.5f),
            };
            //PhysicsBody = tankDescription.Body.;
            if (_simulation == null) return;

            // Altura del suelo en (X,Z)
            float terrainY = _terrain?.GetHeightAtPosition(Position.X, Position.Z) ?? Position.Y;
            
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
            
            var wheelHandles = new QuickList<BodyHandle>(tankDescription.WheelCountPerTread * 2, bufferPool);
            var constraints = new QuickList<ConstraintHandle>(tankDescription.WheelCountPerTread * 2 * 6 + 4, bufferPool);
            var leftMotors = new QuickList<ConstraintHandle>(tankDescription.WheelCountPerTread, bufferPool);
            var rightMotors = new QuickList<ConstraintHandle>(tankDescription.WheelCountPerTread, bufferPool);
            
            ref var bodyFilter = ref CreatePart(_simulation, tankDescription.Body, pose, properties, ref BodyHandles, out _body);
            ref var turretFilter = ref CreatePart(_simulation, tankDescription.Turret, pose, properties, ref BodyHandles, out _turret);
            ref var barrelFilter = ref CreatePart(_simulation, tankDescription.Barrel, pose, properties, ref BodyHandles, out _barrel);
            
            bodyFilter = new SubgroupCollisionFilter(_body.Value, 0);
            turretFilter = new SubgroupCollisionFilter(_body.Value, 1);
            barrelFilter = new SubgroupCollisionFilter(_body.Value, 2);
            SubgroupCollisionFilter.DisableCollision(ref bodyFilter, ref turretFilter);
            SubgroupCollisionFilter.DisableCollision(ref turretFilter, ref barrelFilter);
            
            Matrix3x3.CreateFromQuaternion(tankDescription.TurretBasis, out var turretBasis);

            //Attach the turret to the body.
            QuaternionEx.Transform(turretBasis.Y, QuaternionEx.Conjugate(tankDescription.Body.Pose.Orientation), out var bodyLocalSwivelAxis);
            QuaternionEx.Transform(turretBasis.Y, QuaternionEx.Conjugate(tankDescription.Turret.Pose.Orientation), out var turretLocalSwivelAxis);
            RigidPose.TransformByInverse(tankDescription.TurretAnchor, tankDescription.Body.Pose, out var bodyLocalTurretAnchor);
            RigidPose.TransformByInverse(tankDescription.TurretAnchor, tankDescription.Turret.Pose, out var turretLocalTurretAnchor);
            constraints.AllocateUnsafely() = _simulation.Solver.Add(_body, _turret,
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
            _turretServoDescription = new TwistServo
            {
                LocalBasisA = bodyLocalTurretBasis,
                LocalBasisB = turretLocalTurretBasis,
                SpringSettings = tankDescription.TurretSpring,
                ServoSettings = tankDescription.TurretServo
            };
            _turretServo = _simulation.Solver.Add(_body, _turret, _turretServoDescription);
            constraints.AllocateUnsafely() = _turretServo;
            
            
            //Attach the barrel to the turret.
            QuaternionEx.Transform(turretBasis.X, QuaternionEx.Conjugate(tankDescription.Turret.Pose.Orientation), out var turretLocalPitchAxis);
            QuaternionEx.Transform(turretBasis.X, QuaternionEx.Conjugate(tankDescription.Barrel.Pose.Orientation), out var barrelLocalPitchAxis);
            RigidPose.TransformByInverse(tankDescription.BarrelAnchor, tankDescription.Turret.Pose, out var turretLocalBarrelAnchor);
            RigidPose.TransformByInverse(tankDescription.BarrelAnchor, tankDescription.Barrel.Pose, out var barrelLocalBarrelAnchor);
            constraints.AllocateUnsafely() = _simulation.Solver.Add(_turret, _barrel,
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
            _barrelServoDescription = new TwistServo
            {
                LocalBasisA = turretLocalBarrelBasis,
                LocalBasisB = barrelLocalBarrelBasis,
                SpringSettings = tankDescription.BarrelSpring,
                ServoSettings = tankDescription.BarrelServo
            };
            _barrelServo = _simulation.Solver.Add(_turret, _barrel, _barrelServoDescription);
            constraints.AllocateUnsafely() = _barrelServo;
            
            QuaternionEx.TransformUnitY(tankDescription.WheelOrientation, out var wheelAxis);
            QuaternionEx.TransformUnitZ(tankDescription.WheelOrientation, out var treadDirection);
            var treadStart = tankDescription.TreadSpacing * (tankDescription.WheelCountPerTread - 1) * -0.5f;
            BodyHandle previousLeftWheelHandle = default, previousRightWheelHandle = default;
            for (int i = 0; i < tankDescription.WheelCountPerTread; ++i)
            {
                var wheelOffsetFromTread = treadDirection * (treadStart + i * tankDescription.TreadSpacing);
                var rightWheelHandle = CreateWheel(_simulation, properties, pose, tankDescription.Body.Pose,
                    tankDescription.WheelShape, tankDescription.WheelInertia, tankDescription.WheelFriction, _body, ref properties[_body].Filter,
                    tankDescription.RightTreadOffset + wheelOffsetFromTread - tankDescription.Body.Pose.Position,
                    tankDescription.SuspensionLength, tankDescription.SuspensionSettings, tankDescription.WheelOrientation,
                    ref wheelHandles, ref constraints, ref rightMotors, ref BodyHandles);
                var leftWheelHandle = CreateWheel(_simulation, properties, pose, tankDescription.Body.Pose,
                    tankDescription.WheelShape, tankDescription.WheelInertia, tankDescription.WheelFriction, _body, ref properties[_body].Filter,
                    tankDescription.LeftTreadOffset + wheelOffsetFromTread - tankDescription.Body.Pose.Position,
                    tankDescription.SuspensionLength, tankDescription.SuspensionSettings, tankDescription.WheelOrientation,
                    ref wheelHandles, ref constraints, ref leftMotors, ref BodyHandles);

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
            
            wheelHandles.Span.Slice(wheelHandles.Count);
            constraints.Span.Slice(constraints.Count);
            LeftMotors = leftMotors.Span.Slice(leftMotors.Count);
            RightMotors = rightMotors.Span.Slice(rightMotors.Count);
            
            QuaternionEx.ConcatenateWithoutOverlap(tankDescription.Body.Pose.Orientation, QuaternionEx.Conjugate(tankDescription.TurretBasis), out _fromBodyLocalToTurretBasisLocal);
            QuaternionEx.Transform(-turretBasis.Z, QuaternionEx.Conjugate(tankDescription.Barrel.Pose.Orientation), out _barrelLocalDirection);
            
            RotationQuaternion = orientationQuat;

        }
       
        // Actualizar tanque jugador
        public void Update(GameTime gameTime, KeyboardState keyboardState)
        {
            var dt = (float)gameTime.ElapsedGameTime.TotalSeconds;

            var body = _simulation.Bodies.GetBodyReference(_body);
            body.Awake = true;

            UpdateProjectileConfig(keyboardState);
            
            if (keyboardState.IsKeyDown(Keys.A)) SteerRotation += SteerSpeed * dt;
            else if(keyboardState.IsKeyDown(Keys.D))  SteerRotation -= SteerSpeed * dt;
            else{ SteerRotation = MathHelper.Lerp(SteerRotation, 0f, dt * 5f); }
            
            SteerRotation = Math.Clamp(SteerRotation, MinSteer, MaxSteer);

            // Girar ruedas según distancia recorrida
            UpdateWheelSpinByDistance();
            
            UpdateCanonAndTurretTowards(new Vector3(AimDirectionWorld.X, AimDirectionWorld.Y, AimDirectionWorld.Z), dt);
            
            UpdateWorldMatrix();
        }
        
        // Actualizar tanque enemigo
        public void Update(GameTime gameTime)
        {
            if (IsDead)
                return;
            
            var dt = (float)gameTime.ElapsedGameTime.TotalSeconds;

            var body = _simulation.Bodies.GetBodyReference(_body);
            body.Awake = true;
            
            SteerRotation = Math.Clamp(SteerRotation, MinSteer, MaxSteer);

            // Girar ruedas según distancia recorrida
            UpdateWheelSpinByDistance();
            
            UpdateCanonAndTurretTowards(new Vector3(AimDirectionWorld.X, AimDirectionWorld.Y, AimDirectionWorld.Z), dt);
            
            UpdateWorldMatrix();
        }

        private void UpdateCanonAndTurretTowards(Vector3 worldAimDir, float dt)
        {
            if (IsDead) return;
            // Leer las poses actuales del cuerpo, torreta y cañón desde la simulación
            var bodyRef   = _simulation.Bodies.GetBodyReference(_body);
            var turretRef = _simulation.Bodies.GetBodyReference(_turret);
            var barrelRef = _simulation.Bodies.GetBodyReference(_barrel);

            var qBody   = bodyRef.Pose.Orientation;
            var qBarrel = barrelRef.Pose.Orientation;

            // Dirección "forward" del cañón físico en MUNDO
            //    (BarrelLocalDirection ya es el forward del cañón en su espacio local)
            QuaternionEx.Transform(_barrelLocalDirection, qBarrel, out var fwdWorldN);

            // Normalizar por seguridad
            var len2 = fwdWorldN.LengthSquared();
            if (len2 < 1e-12f) return;
            fwdWorldN /= MathF.Sqrt(len2);

            // Yaw del tanque (desde la física). Ojo: tu tanque "mira" -Z.
            var bodyFwd = System.Numerics.Vector3.Transform(new System.Numerics.Vector3(0, 0, -1), qBody);
            var tankYaw = MathF.Atan2(bodyFwd.X, bodyFwd.Z) + MathF.PI; 
            // Yaw de la torreta a partir del cañón físico
            var flat = new System.Numerics.Vector3(fwdWorldN.X, 0, fwdWorldN.Z);
            var flatLen2 = flat.LengthSquared();
            if (flatLen2 < 1e-12f) return;
            flat /= MathF.Sqrt(flatLen2);

            var globalTurretYaw = MathF.Atan2(flat.X, flat.Z);

            // Diferencia de yaw entre el cuerpo y el cañón físico
            var swivel = MathHelper.WrapAngle(globalTurretYaw - tankYaw);

            // Pitch del cañón físico 
            var pitch = -MathF.Asin(Math.Clamp(fwdWorldN.Y, -1f, 1f));

            // Asignar directo a los ángulos visuales
            
            TurretRotation = swivel;
            CannonRotation = pitch;
        }
        
        private void UpdateWheelSpinByDistance()
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
            if (IsDead) return;
            var body = _simulation.Bodies.GetBodyReference(_body);
            var pose = body.Pose;

            Position = new Vector3(pose.Position.X, pose.Position.Y, pose.Position.Z);
            var q = pose.Orientation;
            RotationQuaternion = new Quaternion(q.X, q.Y, q.Z, q.W);
        }

        private void UpdateWorldMatrix()
        {
            if (IsDead) return;
            // Sincronizar posición y rotación desde la física
            var bodyReference = _simulation.Bodies.GetBodyReference(_body);
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
            // Normal del terreno por diferencias centrales 
            float h = 50f;
            float x = Position.X, z = Position.Z;

            float hL = _terrain.GetHeightAtPosition(x - h, z);
            float hR = _terrain.GetHeightAtPosition(x + h, z);
            float hD = _terrain.GetHeightAtPosition(x, z - h);
            float hU = _terrain.GetHeightAtPosition(x, z + h);

            // Gradientes 
            var tangentX = new Vector3(2f * h, hR - hL, 0f); // (Δx, Δy, 0)
            var tangentZ = new Vector3(0f, hU - hD, 2f * h); // (0, Δy, Δz)

            // Normal “up” del terreno
            var up = Vector3.Normalize(
                Vector3.Cross(tangentZ, tangentX));
            
            var yawForward = Vector3.Transform(
                -Vector3.UnitZ,
                Matrix.CreateRotationY(Rotation));

            // Proyecta el forward sobre el plano del terreno para que siga la pendiente
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

            // Matriz de orientación a partir de la base R-U-F
            orientation = new Matrix(
                right.X, right.Y, right.Z, 0f,
                up.X, up.Y, up.Z, 0f,
                forward.X, forward.Y, forward.Z, 0f,
                0f, 0f, 0f, 1f
            );
            
            PitchRotation = MathF.Asin(-forward.Y);
            RollRotation = MathF.Asin(right.Y);
        }


        public void Draw()
        {
            if (_model == null || _effect == null || IsDead) return;

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
                    var effect = part.Effect;
                    effect.Parameters["World"]?.SetValue(worldPerMesh);
                    
                    // CRÍTICO: Configurar View y Projection desde la cámara
                    if (_camera != null)
                    {
                        effect.Parameters["View"]?.SetValue(_camera.View);
                        effect.Parameters["Projection"]?.SetValue(_camera.Projection);
                    }
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

            // Transform del cañón en espacio mundo
            var cannonAbs = _boneTransforms[_cannonBone.Index];
            var cannonWorld = cannonAbs * _world; // mismo patrón que en Draw()

            // La dirección “forward” del cañón 
            var f = -Vector3.Normalize(GetUp(cannonWorld));

            // Posición del muzzle: origen del hueso + corrimiento a lo largo del cañón
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

            // magnitud  base: momento del proyectil escalado un poco
            var recoilMagnitude = projectileMass * muzzleSpeed * 2.0f * intensity;

            // Aceleración de retroceso 
            _recoilAccelSys = -new System.Numerics.Vector3(dir.X, dir.Y, dir.Z) * recoilMagnitude;
            _recoilTime = RecoilDuration;

            if (withBrake)
            {
                _brakeTime = BrakeDuration;
            }

            var amplitude = 0.001f * projectileMass * muzzleSpeed; 
            var rotational = amplitude * 0.06f;
            if(_camera != null)
                _camera.StartShake(amplitude, 0.12f, rotational);
        }

        public void ApplyRecoilAndBrake(float dt, Simulation simulation)
        {
            if (IsDead) return;
            // Referencia al cuerpo físico del tanque
            var bodyRef = simulation.Bodies.GetBodyReference(_body); // usa tu handle del tanque

            // Retroceso: empuja en dirección opuesta por un tiempo corto
            if (_recoilTime > 0f)
            {
                bodyRef.Velocity.Linear += _recoilAccelSys * dt;
                _recoilTime -= dt;
            }

            // Freno: arrastre proporcional a la velocidad actual 
            if (_brakeTime > 0f)
            {
                var v = bodyRef.Velocity.Linear;
                var drag = -v * (BrakeK * dt); 
                bodyRef.Velocity.Linear += drag;
                _brakeTime -= dt;
            }
        }

        public void Kill()
        {
            if (IsDead) return;
            IsDead = true;
            
            foreach(var body in BodyHandles)
            {
                _simulation.Bodies.Remove(body);
                CollisionHandler.HandleToTank.Remove(body);
            }
        }

        public void UpdateEnemyTankAI(Vector3 playerPosition, TankController tankController)
        {
            if (IsDead) return;

            // Calculate distance to player
            var direction = playerPosition - Position;
            var distance = direction.Length();

            // Always aim at player
            if (distance > 0.001f)
            {
                direction.Normalize();
                var aimDirection = new System.Numerics.Vector3(direction.X, 0, direction.Z);
                AimDirectionWorld = aimDirection;
            }

            // Simple approach: Only move if far enough, and calculate proper turning
            if (distance > 8f) // Reduced from 15f to 8f - much closer approach
            {
                // Calculate the angle we need to turn to face the player
                var directionToPlayer = new Vector3(direction.X, 0, direction.Z);
                directionToPlayer = Vector3.Normalize(directionToPlayer);

                var targetYaw = (float)Math.Atan2(directionToPlayer.X, directionToPlayer.Z);

                // Get current tank rotation from physics body
                var body = _simulation.Bodies.GetBodyReference(_body);
                var currentRotation = body.Pose.Orientation;
                var currentYaw = (float)Math.Atan2(
                    2 * (currentRotation.W * currentRotation.Y + currentRotation.X * currentRotation.Z),
                    1 - 2 * (currentRotation.Y * currentRotation.Y + currentRotation.Z * currentRotation.Z)
                );

                // Calculate angle difference
                var angleDiff = targetYaw - currentYaw;
                while (angleDiff > Math.PI) angleDiff -= 2 * (float)Math.PI;
                while (angleDiff < -Math.PI) angleDiff += 2 * (float)Math.PI;

                // Tank movement logic similar to player controls
                float leftSpeed, rightSpeed;

                if (Math.Abs(angleDiff) > 0.3f) // Need to turn significantly
                {
                    // Turn towards the player (inverted controls)
                    if (angleDiff > 0) // Turn left
                    {
                        leftSpeed = 0.7f;   // Forward left track
                        rightSpeed = -0.7f; // Reverse right track
                    }
                    else // Turn right
                    {
                        leftSpeed = -0.7f;  // Reverse left track
                        rightSpeed = 0.7f;  // Forward right track
                    }
                }
                else // Go forward (roughly facing player)
                {
                    leftSpeed = 1.0f;  // Full speed forward
                    rightSpeed = 1.0f;
                }

                tankController.leftTargetSpeedFraction = leftSpeed;
                tankController.rightTargetSpeedFraction = rightSpeed;
                tankController.zoom = false;
                tankController.brakeLeft = false;
                tankController.brakeRight = false;
                tankController.UpdateMovementAndAim(_simulation, directionToPlayer);
            }
            else
            {
                // Stop completely
                var directionToPlayer = new Vector3(direction.X, 0, direction.Z);
                if (directionToPlayer.Length() > 0.001f)
                    directionToPlayer = Vector3.Normalize(directionToPlayer);

                tankController.leftTargetSpeedFraction = 0f;
                tankController.rightTargetSpeedFraction = 0f;
                tankController.zoom = false;
                tankController.brakeLeft = true;
                tankController.brakeRight = true;
                tankController.UpdateMovementAndAim(_simulation, directionToPlayer);
            }
        }

        private void UpdateProjectileConfig(KeyboardState keyboardState)
        {
            if (keyboardState.IsKeyDown(Keys.D1))
            {
                TipoProyectilActual = ProjectilePresets.Light;
            } else if (keyboardState.IsKeyDown(Keys.D2))
            {
                TipoProyectilActual = ProjectilePresets.Heavy;
            }
        }
        
        // Ojo, esta es la versión del método para enemigos, no jugador!
        public void RecibirAtaque(float danio)
        {
            Vida -= danio;
            
            if (Vida <= 0f)
                Kill();
        }

        public void UpdateAim(MouseState mouseState, Camera camera, Viewport vp)
        {
            var aimDir = camera.FrontDirection; 
            var hit = PickOnTerrain(mouseState.Position, vp); //Rayo para ver donde impacta en el terreno
            if (hit.HasValue)
            {
                var aimXna = hit.Value - this.Position;
                aimXna.Normalize();
                aimDir = aimXna;
            }
            this.AimDirectionWorld = aimDir;
        }
        
        private Vector3? PickOnTerrain(Point mouse, Viewport viewport)
        {
            // Desarma matrices
            var view = _camera.View;
            var proj = _camera.Projection;

            // Dos puntos en NDC (near/far) -> espacio mundo
            var nearPoint = viewport.Unproject(new Microsoft.Xna.Framework.Vector3(mouse.X, mouse.Y, 0f), proj, view, Matrix.Identity);
            var farPoint  = viewport.Unproject(new Microsoft.Xna.Framework.Vector3(mouse.X, mouse.Y, 1f), proj, view, Matrix.Identity);

            var dir = Microsoft.Xna.Framework.Vector3.Normalize(farPoint - nearPoint);
            var origin = nearPoint;

            // Busco intersección por búsqueda binaria contra la altura del terreno
            float tMin = 0f, tMax = 5000f; // alcance del rayo
            for (int i = 0; i < 48; i++) // precisión suficiente
            {
                float tMid = 0.5f * (tMin + tMax);
                var p = origin + dir * tMid;
                float terrainY = _terrain.GetHeightAtPosition(p.X, p.Z); // ← tu helper
                if (p.Y > terrainY) tMin = tMid; else tMax = tMid;
            }

            var hit = origin + dir * tMax;

            // Si estamos muy lejos o fuera del mapa, descartamos
            if (float.IsNaN(hit.X)) return null;
            return hit;
        }
    }
}

