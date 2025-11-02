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

        public Model Model;
        public Effect _effect;
        public Matrix _world;

        private ModelBone[] _wheelBones;
        private ModelBone _turretBone;
        private ModelBone _cannonBone;
        
        private Texture2D hullATexture;
        private Texture2D hullBTexture;
        private Texture2D hullCTexture;
        private Texture2D treadmillsTexture;
        
        private BodyHandle _body;
        private BodyHandle _secBody;
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

        // Sistema de audio del tanque
        public TankAudio Audio { get; private set; }
        private bool _wasBraking = false;

        // Propiedades de movimiento
        public Vector3 Position { get; private set; }
        public float Rotation { get; }
        private float Scale { get; }
        
        public Buffer<ConstraintHandle> LeftMotors;
        public Buffer<ConstraintHandle> RightMotors;
        
        // Parámetros de movimiento
        private const float SteerSpeed = 90f;
        private const float MaxSteer = 45f;
        private const float MinSteer = -45f;
        private const float MaxPitch = MathF.PI / 6f;
        private const float MinPitch = -MathF.PI / 40f;

        // Física
        private Simulation _simulation;
        private Terrain _terrain;

        public QuickList<BodyHandle> BodyHandles;

        private TankDescription tankDescription;
        // Recoil
        private float _recoilTime;
        private const float RecoilDuration = 0.12f; // seg: cuánto dura el empujón
        private System.Numerics.Vector3 _recoilAccelSys = System.Numerics.Vector3.Zero;

        // Brake (freno por “arrastre”)
        private float _brakeTime;
        private const float BrakeDuration = 0.18f; // seg
        private const float BrakeK = 10f; // coeficiente de frenado (tunable)

        private float VisualYOffset = 38f;
        private float VisualZOffset = 23f;

        private float _previousSwivel;

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
        public ProjectileType TipoProyectilActual = ProjectileTypes.Light;
        public float FireCooldown;

        public Texture2D Texture;
        
        public Tank(Vector3 initialPosition, float initialRotation = 0f, float scale = 1f)
        {
            _lastPos = Position = initialPosition;
            Rotation = initialRotation;
            Scale = scale;
        }
        
        /// <summary>
        /// Obtiene el angulo de yaw y pitch apartir de la direccion de mira
        /// </summary>
        public (float targetYaw, float targetPitch) ComputeTurretAngles(Simulation simulation, Vector3 aimDirection)
        {
            // Descomponemos el vector de puntería en dos ángulos: yaw de torreta y pitch de cañón.
            // Primero llevamos 'aimDirection' al sistema de referencia de la torreta (su "turret basis").
            QuaternionEx.ConcatenateWithoutOverlap(QuaternionEx.Conjugate(simulation.Bodies[_body].Pose.Orientation), _fromBodyLocalToTurretBasisLocal, out var toTurretBasis);
            var aimdirection = new System.Numerics.Vector3(aimDirection.X, aimDirection.Y, aimDirection.Z);
            QuaternionEx.TransformWithoutOverlap(aimdirection, toTurretBasis, out var aimDirectionInTurretBasis);
            
            var yaw = MathF.Atan2(aimDirectionInTurretBasis.X, -aimDirectionInTurretBasis.Z);
            var pitch = MathF.Asin(MathF.Max(-1f, MathF.Min(1f, -aimDirectionInTurretBasis.Y)));
            return (yaw, pitch);
        }
        
        public void SetAim(Simulation simulation, float targetSwivelAngle, float targetPitchAngle)
        {
            
            targetPitchAngle = Math.Clamp(targetPitchAngle, MinPitch, MaxPitch);
            var turretDescription = _turretServoDescription;
            turretDescription.TargetAngle = targetSwivelAngle;
            simulation.Solver.ApplyDescription(_turretServo, turretDescription);
            var barrelDescription = _barrelServoDescription;
            barrelDescription.TargetAngle = targetPitchAngle;
            simulation.Solver.ApplyDescription(_barrelServo, barrelDescription);

        }
        
        public void SetSpeed(Buffer<ConstraintHandle> motors, float speed, float maximumForce)
        {
            var motorDescription = new AngularAxisMotor //BEPU
            {
                LocalAxisA = new System.Numerics.Vector3(0, -1, 0),
                Settings = new MotorSettings(maximumForce, 1e-6f),
                TargetVelocity = speed
            };
            
            for (var i = 0; i < motors.Length; ++i)
            {
                _simulation.Solver.ApplyDescription(motors[i], motorDescription);
            }
        }
        
        public void CargarModelo(string rutaRelativa, Effect efecto, ContentManager content, Simulation simulation, BufferPool bufferPool,
            GraphicsDevice graphicsDevice,Gizmos gizmos, CollidableProperty<TankBodyProperties> properties, Terrain terrain = null)
        {
            //debug
            Gizmos = gizmos;
            Gizmos.LoadContent(graphicsDevice, new ContentManager(content.ServiceProvider, "Content"));
            
            _effect = efecto;
            _simulation = simulation;
            _terrain = terrain;
            
            // Inicializar sistema de audio
            Audio = new TankAudio();
            Audio.LoadContent(content);
            
            BodyHandles = new QuickList<BodyHandle>(11, bufferPool);
            // Cargar modelo
            Model = content.Load<Model>(TGCGame.ContentFolder3D + rutaRelativa);
            
            // Cargar texturas del T90
            hullATexture = content.Load<Texture2D>(TGCGame.ContentFolder3D + "t90/textures_mod/hullA");
            hullBTexture = content.Load<Texture2D>(TGCGame.ContentFolder3D + "t90/textures_mod/hullB");
            hullCTexture = content.Load<Texture2D>(TGCGame.ContentFolder3D + "t90/textures_mod/hullC");
            treadmillsTexture = content.Load<Texture2D>(TGCGame.ContentFolder3D + "t90/textures_mod/treadmills");
                
            // Look up shortcut references to the bones we are going to animate.
            _wheelBones = new ModelBone[16];

            for (int i = 0; i < 16; i++)
            {
                string boneName = $"Wheel{i + 1}";
                _wheelBones[i] = Model.Bones[boneName];
            }

            _turretBone = Model.Bones["Turret"];
            _cannonBone = Model.Bones["Cannon"];
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
            _boneTransforms = new Matrix[Model.Bones.Count];

            // Asignar efecto y texturas a todas las partes del modelo
            foreach (var mesh in Model.Meshes)
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
        TypedIndex wheelShape, BodyInertia wheelInertia, float wheelFriction, BodyHandle bodyHandle, ref SubgroupCollisionFilter bodyFilter, ref SubgroupCollisionFilter secBodyFilter, System.Numerics.Vector3 bodyToWheelSuspension, float suspensionLength,
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
            SubgroupCollisionFilter.DisableCollision(ref wheelProperties.Filter, ref secBodyFilter);
           
                      
            return wheelHandle;
        }

        private void CreatePhysicsBody(BufferPool bufferPool, CollidableProperty<TankBodyProperties> properties)
        {
            if (_simulation == null || _terrain == null) return;

            var length = 4f;
            // Geometría de una rueda
            var wheelShape = new Cylinder(3.4f, length);
            var smallWheelShape = new Cylinder(3f, length);
            var smallerWheelShape = new Cylinder(2.3f, length);
            // Calcula el tensor de inercia para masa
            var wheelInertia = wheelShape.ComputeInertia(0.25f);
            // Registra la forma en el repositorio de Shapes de la simulación
            var wheelShapeIndex = _simulation.Shapes.Add(wheelShape);
            var smallWheelShapeIndex = _simulation.Shapes.Add(smallWheelShape);
            var smallerWheelShapeIndex = _simulation.Shapes.Add(smallerWheelShape);

            tankDescription = new TankDescription
            {
                //Cuerpo del tanque
                Body = TankPartDescription.Create(1, new Box(36f, 5f, 60),
                    new RigidPose(new System.Numerics.Vector3(0, 0, 0), System.Numerics.Quaternion.Identity), 0.5f, _simulation.Shapes),
                SecondaryBody = TankPartDescription.Create(0.1f, new Box(22f, 5f, 52.5f),
                    new RigidPose(new System.Numerics.Vector3(0, -5, 0), System.Numerics.Quaternion.Identity), 0.5f, _simulation.Shapes),
                // Torreta, desplazado hacia arriba y adelante
                Turret = TankPartDescription.Create(1, new Cylinder(15f, 7f), new System.Numerics.Vector3(0f, 6f, 2.5f),
                    0.5f, _simulation.Shapes),
                // Cañón
                Barrel = TankPartDescription.Create(0.1f, new Box(2f, 2f, 46f), new System.Numerics.Vector3(0, 6f, -34),
                    0.5f, _simulation.Shapes),
                // Punto de anclaje de la torreta respecto al cuerpo.
                TurretAnchor = new System.Numerics.Vector3(1f, 3f, 2.5f),
                // Punto de anclaje del cañón respecto a la torreta.
                BarrelAnchor = new System.Numerics.Vector3(0, 6f, -11),
                // Base/orientación de referencia de la torreta (rotada 180° sobre Y)
                TurretBasis = System.Numerics.Quaternion.CreateFromAxisAngle(
                    System.Numerics.Vector3.UnitY, MathF.PI),
                // Control de servo y resorte para la torreta
                TurretServo = new ServoSettings(1e7f, 0f, 1000),
                TurretSpring = new SpringSettings(10f, 10.0f),
                // Control de servo y resorte para el cañón
                BarrelServo = new ServoSettings(1e7f, 0f, 4000),
                BarrelSpring = new SpringSettings(10f, 10.0f),
                // Offset lateral de las orugas izquierda y derecha.
                LeftTreadOffset = new System.Numerics.Vector3(-15f, 0f, 0),
                RightTreadOffset = new System.Numerics.Vector3(15f, 0f, 0),
                // Longitud de suspensión y parámetros de resorte/amortiguación de las ruedas.
                SuspensionLength = 6.5f,
                SuspensionSettings = new SpringSettings(5f, 3f),
                // Forma, inercia y fricción de las ruedas.
                WheelShape = wheelShapeIndex,
                SmallWheelShape = smallWheelShapeIndex,
                SmallerWheelShape = smallerWheelShapeIndex,
                WheelInertia = wheelInertia,
                WheelFriction = 2f,
                // Espaciado entre ruedas y cantidad de ruedas por oruga.
                TreadSpacing = 8f,
                WheelCountPerTread = 8,
                // Orientación local de la rueda (ejes del cilindro alineados con el mundo de rueda).
                WheelOrientation = QuaternionEx.CreateFromAxisAngle(System.Numerics.Vector3.UnitZ, MathF.PI * -0.5f),
            };

            var alturaTerreno = _terrain.GetHeightAtPosition(Position.X, Position.Z);
            
            // Orienta el tanque para que “asiente” sobre la normal del terreno.
            var orientationQuat = _terrain.CalculateRotation(Position, Rotation );
            
            // Pose inicial
            var pose = new RigidPose(
                new System.Numerics.Vector3(Position.X + 1300, alturaTerreno + 100, Position.Z),
                new System.Numerics.Quaternion(orientationQuat.X, orientationQuat.Y, orientationQuat.Z,
                    orientationQuat.W)
            );
            // Posición inicial y orientación alineada al terreno.
            var wheelHandles = new QuickList<BodyHandle>(tankDescription.WheelCountPerTread * 2, bufferPool);
            var constraints =
                new QuickList<ConstraintHandle>(tankDescription.WheelCountPerTread * 2 * 6 + 4, bufferPool);
            var leftMotors = new QuickList<ConstraintHandle>(tankDescription.WheelCountPerTread, bufferPool);
            var rightMotors = new QuickList<ConstraintHandle>(tankDescription.WheelCountPerTread, bufferPool);
            // Estructuras temporales para guardar handles de ruedas, constraints y motores. Se usan pools para evitar GC.
            ref var bodyFilter = ref CreatePart(_simulation, tankDescription.Body, pose, properties, ref BodyHandles,
                out _body);
            ref var secBodyFilter = ref CreatePart(_simulation, tankDescription.SecondaryBody, pose, properties,
                ref BodyHandles, out _secBody);
            ref var turretFilter = ref CreatePart(_simulation, tankDescription.Turret, pose, properties,
                ref BodyHandles, out _turret);
            ref var barrelFilter = ref CreatePart(_simulation, tankDescription.Barrel, pose, properties,
                ref BodyHandles, out _barrel);
            // Crea los cuerpos rígidos (body/torreta/cañón) en la simulación y obtiene sus filtros de colisión y handles.

            bodyFilter = new SubgroupCollisionFilter(_body.Value, 0);
            turretFilter = new SubgroupCollisionFilter(_body.Value, 1);
            barrelFilter = new SubgroupCollisionFilter(_body.Value, 2);
            secBodyFilter = new SubgroupCollisionFilter(_body.Value, 3);
            SubgroupCollisionFilter.DisableCollision(ref bodyFilter, ref turretFilter);
            SubgroupCollisionFilter.DisableCollision(ref turretFilter, ref barrelFilter);
            SubgroupCollisionFilter.DisableCollision(ref bodyFilter, ref secBodyFilter);
            // Define subgrupos de colisión para evitar colisiones internas entre cuerpo-torreta y torreta-cañón.
            Matrix3x3.CreateFromQuaternion(tankDescription.TurretBasis, out var turretBasis);
            // Convierte la base de la torreta (quaternion) a una matriz 3x3.
            
            constraints.AllocateUnsafely() = _simulation.Solver.Add(_body, _secBody,
                new Weld
                {
                    LocalOffset = new System.Numerics.Vector3(0f, -5f, 2.5f),
                    LocalOrientation = System.Numerics.Quaternion.Identity,
                    SpringSettings = new SpringSettings(30f,1f)
                }
            );

            // Convierte la base de la torreta (quaternion) a una matriz 3x3.
            QuaternionEx.Transform(turretBasis.Y, QuaternionEx.Conjugate(tankDescription.Body.Pose.Orientation), out var bodyLocalSwivelAxis);
            QuaternionEx.Transform(turretBasis.Y, QuaternionEx.Conjugate(tankDescription.Turret.Pose.Orientation), out var turretLocalSwivelAxis);
            RigidPose.TransformByInverse(tankDescription.TurretAnchor, tankDescription.Body.Pose, out var bodyLocalTurretAnchor);
            RigidPose.TransformByInverse(tankDescription.TurretAnchor, tankDescription.Turret.Pose, out var turretLocalTurretAnchor);
            // Calcula ejes locales de giro (swivel) y puntos de anclaje en los espacios locales de cuerpo y torreta.

            constraints.AllocateUnsafely() = _simulation.Solver.Add(_body, _turret,
                new Hinge
                {
                    LocalHingeAxisA = bodyLocalSwivelAxis,
                    LocalHingeAxisB = turretLocalSwivelAxis,
                    LocalOffsetA = bodyLocalTurretAnchor,
                    LocalOffsetB = turretLocalTurretAnchor,
                    SpringSettings = new SpringSettings(30, 1)
                });
            // Agrega una bisagra (hinge) entre cuerpo y torreta, con ejes/offsets ya calculados.
            
            Matrix3x3 turretSwivelBasis;
            turretSwivelBasis.Z = -turretBasis.Y;
            turretSwivelBasis.X = -turretBasis.Z;
            turretSwivelBasis.Y = turretBasis.X;
            // Construye una base ortonormal para medir el ángulo de la torreta (ejes perpendiculares a hinge).
            
            QuaternionEx.CreateFromRotationMatrix(turretSwivelBasis, out var turretSwivelBasisQuaternion);
            QuaternionEx.ConcatenateWithoutOverlap(turretSwivelBasisQuaternion, QuaternionEx.Conjugate(tankDescription.Body.Pose.Orientation), out var bodyLocalTurretBasis);
            QuaternionEx.ConcatenateWithoutOverlap(turretSwivelBasisQuaternion, QuaternionEx.Conjugate(tankDescription.Turret.Pose.Orientation), out var turretLocalTurretBasis);
            // Convierte esa base a los espacios locales de cuerpo y torreta.
            
            _turretServoDescription = new TwistServo
            {
                LocalBasisA = bodyLocalTurretBasis,
                LocalBasisB = turretLocalTurretBasis,
                SpringSettings = tankDescription.TurretSpring,
                ServoSettings = tankDescription.TurretServo
            };
            _turretServo = _simulation.Solver.Add(_body, _turret, _turretServoDescription);
            constraints.AllocateUnsafely() = _turretServo;
            // Crea y agrega un TwistServo para controlar el ángulo de la torreta (yaw), usando esa base de medida.
            
            QuaternionEx.Transform(turretBasis.X, QuaternionEx.Conjugate(tankDescription.Turret.Pose.Orientation), out var turretLocalPitchAxis);
            QuaternionEx.Transform(turretBasis.X, QuaternionEx.Conjugate(tankDescription.Barrel.Pose.Orientation), out var barrelLocalPitchAxis);
            RigidPose.TransformByInverse(tankDescription.BarrelAnchor, tankDescription.Turret.Pose, out var turretLocalBarrelAnchor);
            RigidPose.TransformByInverse(tankDescription.BarrelAnchor, tankDescription.Barrel.Pose, out var barrelLocalBarrelAnchor);
            // Prepara ejes locales de pitch y puntos de anclaje para el cañón respecto a la torreta.
            
            constraints.AllocateUnsafely() = _simulation.Solver.Add(_turret, _barrel,
                new Hinge
                {
                    LocalHingeAxisA = turretLocalPitchAxis,
                    LocalHingeAxisB = barrelLocalPitchAxis,
                    LocalOffsetA = turretLocalBarrelAnchor,
                    LocalOffsetB = barrelLocalBarrelAnchor,
                    SpringSettings = new SpringSettings(30, 1)
                });
            // Agrega una bisagra entre torreta y cañón (pitch).

            Matrix3x3 barrelPitchBasis;
            barrelPitchBasis.Z = -turretBasis.X;
            barrelPitchBasis.X = -turretBasis.Z;
            barrelPitchBasis.Y = -turretBasis.Y;
            // Base ortonormal para medir el pitch del cañón.
            
            QuaternionEx.CreateFromRotationMatrix(barrelPitchBasis, out var barrelPitchBasisQuaternion);
            QuaternionEx.ConcatenateWithoutOverlap(barrelPitchBasisQuaternion, QuaternionEx.Conjugate(tankDescription.Turret.Pose.Orientation), out var turretLocalBarrelBasis);
            QuaternionEx.ConcatenateWithoutOverlap(barrelPitchBasisQuaternion, QuaternionEx.Conjugate(tankDescription.Barrel.Pose.Orientation), out var barrelLocalBarrelBasis);
            // Bases convertidas a espacios locales.
            
            _barrelServoDescription = new TwistServo
            {
                LocalBasisA = turretLocalBarrelBasis,
                LocalBasisB = barrelLocalBarrelBasis,
                SpringSettings = tankDescription.BarrelSpring,
                ServoSettings = tankDescription.BarrelServo
            };
            // Agrega el TwistServo que controla el pitch del cañón.
            _barrelServo = _simulation.Solver.Add(_turret, _barrel, _barrelServoDescription);
            constraints.AllocateUnsafely() = _barrelServo;
            // Agrega el TwistServo que controla el pitch del cañón.
            
            QuaternionEx.TransformUnitY(tankDescription.WheelOrientation, out var wheelAxis);
            QuaternionEx.TransformUnitZ(tankDescription.WheelOrientation, out var treadDirection);
            // Obtiene los ejes “rueda” (giro) y “oruga” (dirección de avance) en mundo local de rueda.

            var treadStart = tankDescription.TreadSpacing * (tankDescription.WheelCountPerTread - 1) * -0.5f;
            // Punto inicial para distribuir ruedas centradas a lo largo de la oruga.
            BodyHandle previousLeftWheelHandle = default, previousRightWheelHandle = default;
            for (var i = 0; i < tankDescription.WheelCountPerTread; ++i)
            {
                var wheelOffsetFromTread = treadDirection * (treadStart + i * tankDescription.TreadSpacing);
                // Offset longitudinal de cada rueda a lo largo de la oruga.
                var verticalLift = 0f;
                var wheelShapeToUse = tankDescription.WheelShape;
                if (i == 0 )
                {
                    wheelOffsetFromTread += treadDirection * (tankDescription.TreadSpacing / 4);
                    verticalLift = 3f;
                    wheelShapeToUse = tankDescription.SmallerWheelShape;
                }else if (i == tankDescription.WheelCountPerTread - 1)
                {
                    verticalLift = 3f;
                    wheelShapeToUse = tankDescription.SmallWheelShape;
                }
                
                var rightSuspensionOffset = tankDescription.RightTreadOffset + wheelOffsetFromTread - tankDescription.Body.Pose.Position;
                var leftSuspensionOffset = tankDescription.LeftTreadOffset + wheelOffsetFromTread - tankDescription.Body.Pose.Position;

                // Aplicar elevación en eje Y
                rightSuspensionOffset.Y += verticalLift;
                leftSuspensionOffset.Y += verticalLift;
                
                var rightWheelHandle = CreateWheel(_simulation, properties, pose, tankDescription.Body.Pose,
                    wheelShapeToUse, tankDescription.WheelInertia, tankDescription.WheelFriction, _body, ref properties[_body].Filter, ref properties[_secBody].Filter,
                    rightSuspensionOffset, tankDescription.SuspensionLength, tankDescription.SuspensionSettings, tankDescription.WheelOrientation,
                    ref wheelHandles, ref constraints, ref rightMotors, ref BodyHandles);
                
                // Crea una rueda derecha con suspensión, fricción y orientación definidos; guarda handles y constraints.

                var leftWheelHandle = CreateWheel(_simulation, properties, pose, tankDescription.Body.Pose,
                    wheelShapeToUse, tankDescription.WheelInertia, tankDescription.WheelFriction, _body, ref properties[_body].Filter, ref properties[_secBody].Filter,
                    leftSuspensionOffset, tankDescription.SuspensionLength, tankDescription.SuspensionSettings, tankDescription.WheelOrientation,
                    ref wheelHandles, ref constraints, ref leftMotors, ref BodyHandles);
                // Crea la rueda izquierda correspondiente.

                if (i >= 1)
                {
                    // Conecta ruedas consecutivas de una misma oruga para compartir esfuerzo de tracción.
                    // Motor angular que busca velocidad relativa 0 (igualar giro, permitiendo algo de deriva).
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
            // “Cierra” las QuickList exposando los spans finales (ajusta las vistas a su tamaño real).

            QuaternionEx.ConcatenateWithoutOverlap(tankDescription.Body.Pose.Orientation, QuaternionEx.Conjugate(tankDescription.TurretBasis), out _fromBodyLocalToTurretBasisLocal);
            // Guarda conversión de base local del cuerpo a la base de la torreta (útil para apuntado).

            QuaternionEx.Transform(-turretBasis.Z, QuaternionEx.Conjugate(tankDescription.Barrel.Pose.Orientation), out _barrelLocalDirection);
            // Direccion “hacia adelante” del cañón en su espacio local (para raycasts/disparo).

            RotationQuaternion = orientationQuat;
            // Guarda la orientación inicial del tanque (alineada al terreno).
        }
       
        // Actualizar tanque jugador
        public void Update(GameTime gameTime, KeyboardState keyboardState)
        {
            if (IsDead) return;
            var dt = (float)gameTime.ElapsedGameTime.TotalSeconds;

            var body = _simulation.Bodies.GetBodyReference(_body);
            body.Awake = true;

            UpdateProjectileConfig(keyboardState);
            
            if (keyboardState.IsKeyDown(Keys.A)) SteerRotation += SteerSpeed * dt;
            else if(keyboardState.IsKeyDown(Keys.D))  SteerRotation -= SteerSpeed * dt;
            else{ SteerRotation = MathHelper.Lerp(SteerRotation, 0f, dt * 5f); }
            
            SteerRotation = Math.Clamp(SteerRotation, MinSteer, MaxSteer);
            
            FireCooldown = MathF.Max(0f, FireCooldown - dt);

            // Detectar turbo (Shift)
            bool isTurbo = keyboardState.IsKeyDown(Keys.LeftShift);

            // Actualizar sonido del motor basado en la velocidad y turbo
            var velocity = body.Velocity.Linear;
            var speed = MathF.Sqrt(velocity.X * velocity.X + velocity.Z * velocity.Z);
            Audio?.UpdateEngine(speed, dt, isTurbo);

            // Detectar freno
            bool isBraking = keyboardState.IsKeyDown(Keys.Space);
            if (isBraking && !_wasBraking && speed > 1f)
            {
                Audio?.PlayBrake();
            }
            _wasBraking = isBraking;

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

            // Actualizar sonido del motor basado en la velocidad
            var velocity = body.Velocity.Linear;
            var speed = MathF.Sqrt(velocity.X * velocity.X + velocity.Z * velocity.Z);
            Audio?.UpdateEngine(speed, dt);

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
            QuaternionEx.Transform(_barrelLocalDirection, qBarrel, out var fwdWorldN);

            // Normalizar por seguridad
            var len2 = fwdWorldN.LengthSquared();
            if (len2 < 1e-12f) return;
            fwdWorldN /= MathF.Sqrt(len2);

            // Yaw del tanque 
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
            var qTurret = turretRef.Pose.Orientation;
            QuaternionEx.Conjugate(qTurret, out var qInvTurret);
            QuaternionEx.Transform(fwdWorldN, qInvTurret, out var fwdLocalToTurret);
            var pitch = -MathF.Atan2(fwdLocalToTurret.Y, fwdLocalToTurret.Z);


            // Asignar directo a los ángulos visuales
            
            TurretRotation = swivel + (1 * (swivel - _previousSwivel));
            
            _previousSwivel = swivel;
            CannonRotation = pitch - MathHelper.ToRadians(0f);
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
            var pose = _simulation.Bodies.GetBodyReference(_body).Pose;

            Position = new Vector3(pose.Position.X, pose.Position.Y, pose.Position.Z);
            RotationQuaternion = new Quaternion(pose.Orientation.X, pose.Orientation.Y, pose.Orientation.Z,
                pose.Orientation.W);
            
            // Construir offset visual en espacio local del modelo
            // Lo rotamos por la orientación del cuerpo para llevarlo al espacio mundo
            var localOffsetScaled = new Vector3(0f, VisualYOffset * Scale, VisualZOffset * Scale);
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
        

        public void DrawDebug()
        {
            RigidPose pose = _simulation.Bodies.GetBodyReference(_body).Pose;
            var scale = Matrix.CreateScale(1f, 1f, 1f); // tamaño del cuerpo (Box)
            var rotation = Matrix.CreateFromQuaternion(pose.Orientation);
            var translation = Matrix.CreateTranslation(pose.Position);
    
            var matriz = scale * rotation * translation;
            Gizmos.DrawCube(matriz, Color.Blue);


            var turretAnchor = Matrix.CreateTranslation(tankDescription.TurretAnchor);
            var barrelAnchor = Matrix.CreateTranslation(tankDescription.BarrelAnchor);
            Gizmos.DrawCube(turretAnchor * matriz  , Color.Green);
            Gizmos.DrawCube(barrelAnchor * matriz, Color.Red);
            
            Matrix[] boneTransforms = new Matrix[Model.Bones.Count];
            Model.CopyAbsoluteBoneTransformsTo(boneTransforms);
            
            var boneWorldTransform = boneTransforms[_turretBone.Index] * _world;
            var boneWorldPosition = boneWorldTransform.Translation;
            
            var boneMatrix = Matrix.CreateScale(1f) * Matrix.CreateTranslation(boneWorldPosition);
            Gizmos.DrawCube(boneMatrix, Color.Orange);
                
            Gizmos.Draw();
        }
        
        public void Draw(Camera camera)
        {
            if (Model == null || _effect == null || IsDead) return;

            var wheelRotation = Matrix.CreateRotationX(WheelRotation);
            var turretRotation = Matrix.CreateRotationZ(TurretRotation);
            var cannonRotation = Matrix.CreateRotationX(CannonRotation);
            
            for (int i = 0; i < 16; i++)
            {
                _wheelBones[i].Transform = wheelRotation * _wheelTransforms[i]; 
            }
            
            _turretBone.Transform = turretRotation * _turretTransform;
            _cannonBone.Transform = cannonRotation * _cannonTransform;
            
            var absBones = new Matrix[Model.Bones.Count];
            Model.CopyAbsoluteBoneTransformsTo(absBones);
    
            foreach (var mesh in Model.Meshes)
            {
                var worldPerMesh = absBones[mesh.ParentBone.Index] * _world;
                _effect.Parameters["ModelTexture"]?.SetValue(hullATexture);
                if (mesh.Name.Contains("Treadmill"))
                {
                    _effect.Parameters["ModelTexture"]?.SetValue(treadmillsTexture);
                }

                foreach (var part in mesh.MeshParts)
                {
                    
                    part.Effect = _effect;
                    _effect.Parameters["World"]?.SetValue(worldPerMesh);
                    
                    // CRÍTICO: Configurar View y Projection desde la cámara
                    _effect.Parameters["View"]?.SetValue(camera.View);
                    _effect.Parameters["Projection"]?.SetValue(camera.Projection);
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
            
            if (_boneTransforms == null || _boneTransforms.Length != Model.Bones.Count)
                _boneTransforms = new Matrix[Model.Bones.Count];

            Model.CopyAbsoluteBoneTransformsTo(_boneTransforms);

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
            Camera camera,
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
            camera.StartShake(amplitude, 0.12f, rotational);
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
            
            // Detener todos los sonidos del tanque
            Audio?.StopAll();
            Audio?.Dispose();
            
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

                tankController.factorVelocidadIzquierda = leftSpeed;
                tankController.factorVelocidadDerecha = rightSpeed;
                tankController.turbo = false;
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

                tankController.factorVelocidadIzquierda = 0f;
                tankController.factorVelocidadDerecha = 0f;
                tankController.turbo = false;
                tankController.brakeLeft = true;
                tankController.brakeRight = true;
                tankController.UpdateMovementAndAim(_simulation, directionToPlayer);
            }
        }

        private void UpdateProjectileConfig(KeyboardState keyboardState)
        {
            if (keyboardState.IsKeyDown(Keys.D1))
            {
                if (TipoProyectilActual == ProjectileTypes.Light) return;
                
                TipoProyectilActual = ProjectileTypes.Light;
                FireCooldown = TipoProyectilActual.MaxCooldown;
                
            } else if (keyboardState.IsKeyDown(Keys.D2))
            {
                if (TipoProyectilActual == ProjectileTypes.Heavy) return;
                
                TipoProyectilActual = ProjectileTypes.Heavy;
                FireCooldown = TipoProyectilActual.MaxCooldown;
            }
        }
        
        // Ojo, esta es la versión del método para enemigos, no jugador!
        public void RecibirAtaque(float danio)
        {
            Vida -= danio;
            
            if (Vida <= 0f)
                Kill();
        }

        /// <summary>
        /// Indica hacia donde hay que apuntar, en base a la posicion del mouse
        /// </summary>
        public void UpdateAim(MouseState mouseState, Camera camera, Viewport vp)
        {
            var aimDir = camera.FrontDirection; 
            var hit = PickOnTerrain(mouseState.Position, vp, camera); //Rayo para ver donde impacta en el terreno
            if (hit.HasValue)
            {
                aimDir = Vector3.Normalize(hit.Value - Position);
            }
            AimDirectionWorld = aimDir;
        }
        
        private Vector3? PickOnTerrain(Point mouse, Viewport viewport, Camera camera)
        {
            // Desarma matrices
            var view = camera.View;
            var proj = camera.Projection;

            // Dos puntos en NDC (near/far) -> espacio mundo
            var nearPoint = viewport.Unproject(new Vector3(mouse.X, mouse.Y, 0f), proj, view, Matrix.Identity);
            var farPoint  = viewport.Unproject(new Vector3(mouse.X, mouse.Y, 1f), proj, view, Matrix.Identity);

            var dir = Vector3.Normalize(farPoint - nearPoint);
            var origin = nearPoint;

            // Busco intersección por búsqueda binaria contra la altura del terreno
            var tMin = 0f;
            var tMax = 5000f; // alcance del rayo
            for (var i = 0; i < 48; i++) // precisión suficiente
            {
                var tMid = 0.5f * (tMin + tMax);
                var rayoPrueba = origin + dir * tMid;
                var alturaTerreno = _terrain.GetHeightAtPosition(rayoPrueba.X, rayoPrueba.Z);
                if (rayoPrueba.Y > alturaTerreno)
                {
                    tMin = tMid;
                }
                else
                {
                    tMax = tMid;
                }
            }

            var hit = origin + dir * tMax;

            // Si estamos muy lejos o fuera del mapa, descartamos
            if (float.IsNaN(hit.X)) return null;
            return hit;
        }

        public void ResetCooldown()
        {
            FireCooldown = TipoProyectilActual.MaxCooldown;
            Audio?.PlayShoot(TipoProyectilActual);
        }
        
        public void VolverAlCentro()
        {
            // Obtener referencia al cuerpo físico BEPU
            var centerY = _terrain.GetHeightAtPosition(0, 0);
            var bodyHandle = _simulation.Bodies.GetBodyReference(_body);
            var tankPos = bodyHandle.Pose.Position;
            var offset = new System.Numerics.Vector3(1300, centerY + 200, 0) - tankPos;
            
            foreach (var handle in BodyHandles)
            {
                var bodyRef = _simulation.Bodies.GetBodyReference(handle);    
                var pose = bodyRef.Pose;
                pose.Position += offset;
                bodyRef.Pose = pose;
                
                var vel = bodyRef.Velocity;
                vel.Linear = System.Numerics.Vector3.Zero;
                vel.Angular = System.Numerics.Vector3.Zero;
                bodyRef.Velocity = vel;
            }
            
            UpdateWorldMatrix();
        }
    }
}
