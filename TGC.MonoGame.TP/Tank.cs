using System;
using System.Collections.Generic;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;
using TGC.MonoGame.TP.Viewer.Gizmos;
using Quaternion = Microsoft.Xna.Framework.Quaternion;
using Vector3 = Microsoft.Xna.Framework.Vector3;
using MathHelper = Microsoft.Xna.Framework.MathHelper;
using Matrix = Microsoft.Xna.Framework.Matrix;

namespace TGC.MonoGame.TP;

public abstract class Tank
{
    public bool IsDead;
    protected Vector3 LastPos;
    private const float WheelRadius = 2.0f;

    public Model Model;
    public Effect _effect;
    public Matrix _world;
    public Matrix World;

    private ModelBone[] _wheelBones;
    private ModelBone _turretBone;
    private ModelBone _cannonBone;

    private Texture2D hullATexture;
    private Texture2D hullBTexture;
    private Texture2D hullCTexture;
    public Texture2D treadmillsTexture;

    private BodyHandle _body;
    private BodyHandle _secBody;
    private BodyHandle _turret;
    private BodyHandle _barrel;

    protected BodyHandle Body;

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

    private Gizmos Gizmos { get; set; }

    private Quaternion RotationQuaternion { get; set; } = Quaternion.Identity;

    // Sistema de audio del tanque
    public TankAudio Audio { get; private set; }
    protected bool WasBraking = false;

    // Propiedades de movimiento
    public Vector3 Position { get; private set; }
    private float Rotation { get; }
    private float Scale { get; }

    public Buffer<ConstraintHandle> LeftMotors;
    public Buffer<ConstraintHandle> RightMotors;

    // Parámetros de movimiento
    protected const float MaxSteer = 45f;
    protected const float MinSteer = -45f;
    private const float MaxPitch = MathF.PI / 6f;
    private const float MinPitch = -MathF.PI / 40f;

    // Física
    protected Simulation Simulation;
    protected Terrain Terrain;

    public QuickList<BodyHandle> BodyHandles;

    private TankDescription _tankDescription;

    // Recoil
    protected float RecoilTime;
    private const float RecoilDuration = 0.12f; // seg: cuánto dura el empujón
    private System.Numerics.Vector3 _recoilAccelSys = System.Numerics.Vector3.Zero;

    // Brake (freno por “arrastre”)
    protected float BrakeTime;
    private const float BrakeDuration = 0.18f; // seg
    private const float BrakeK = 10f; // coeficiente de frenado (tunable)

    private const float VisualYOffset = 38f;
    private const float VisualZOffset = 23f;

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
    protected float SteerRotation { get; set; }

    /// <summary>
    ///     Gets or sets the turret rotation amount.
    /// </summary>
    private float TurretRotation { get; set; }

    /// <summary>
    ///     Gets or sets the cannon rotation amount.
    /// </summary>
    private float CannonRotation { get; set; }

    public Vector3 AimDirectionWorld { get; protected set; } = new(0, 0, 1);

    public const float VidaMax = 100f;
    public float Vida = VidaMax;
    public ProjectileType TipoProyectilActual = ProjectileTypes.Light;
    public float FireCooldown;
    public Texture2D Texture;

    private const int MaxImpacts = 10;

    public struct ImpactLocal
    {
        public Vector3 Local; // posición en espacio local del hueso
        public float Radius; // en unidades del modelo (luego escala)
        public int BoneIndex; // hueso dueño (ej: hull)
    }

    public readonly List<ImpactLocal> ImpactsLocal = new();
    private const float ImpactRadius = 15f;

    public Tank(Vector3 initialPosition, float initialRotation, float scale)
    {
        LastPos = Position = initialPosition;
        Rotation = initialRotation;
        Scale = scale;
    }
    
    public void AddImpact(Vector3 worldImpactPosition, int boneIndex)
    {
        // Obtené matrices absolutas actuales
        var absBones = new Matrix[Model.Bones.Count];
        Model.CopyAbsoluteBoneTransformsTo(absBones);

        // WORLD del hueso actual
        var boneWorld = absBones[boneIndex] * World;

        // Local = world * inverse(boneWorld)
        var inv = Matrix.Invert(boneWorld);
        var local = Vector3.Transform(worldImpactPosition, inv);

        // Guardar local + radio + hueso
        ImpactsLocal.Add(new ImpactLocal { Local = local, Radius = ImpactRadius, BoneIndex = boneIndex });
    }

    /// <summary>
    /// Obtiene el angulo de yaw y pitch apartir de la direccion de mira
    /// </summary>
    public (float targetYaw, float targetPitch) ComputeTurretAngles(Simulation simulation, Vector3 aimDirection)
    {
        aimDirection = -aimDirection; // Se invierte pues los cálculos necesitan el vector que va hacia el cañón
        // Descomponemos el vector de puntería en dos ángulos: yaw de torreta y pitch de cañón.
        // Primero llevamos 'aimDirection' al sistema de referencia de la torreta (su "turret basis").
        QuaternionEx.ConcatenateWithoutOverlap(QuaternionEx.Conjugate(simulation.Bodies[Body].Pose.Orientation),
            _fromBodyLocalToTurretBasisLocal, out var toTurretBasis);
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
            Simulation.Solver.ApplyDescription(motors[i], motorDescription);
        }
    }

    public void CargarModelo(string rutaRelativa, Effect efecto, ContentManager content, Simulation simulation,
        BufferPool bufferPool,
        GraphicsDevice graphicsDevice, Gizmos gizmos, CollidableProperty<TankBodyProperties> properties,
        Terrain terrain = null)
    {
        //debug
        Gizmos = gizmos;
        Gizmos.LoadContent(graphicsDevice, new ContentManager(content.ServiceProvider, "Content"));

        _effect = efecto;
        Simulation = simulation;
        Terrain = terrain;

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

    static ref SubgroupCollisionFilter CreatePart(Simulation simulation, in TankPartDescription part, RigidPose pose,
        CollidableProperty<TankBodyProperties> properties, ref QuickList<BodyHandle> bodyhandles, out BodyHandle handle)
    {
        RigidPose.MultiplyWithoutOverlap(part.Pose, pose, out var bodyPose);
        handle = simulation.Bodies.Add(BodyDescription.CreateDynamic(bodyPose, part.Inertia, part.Shape, 0.01f));
        bodyhandles.AllocateUnsafely() = handle;
        ref var partProperties = ref properties.Allocate(handle);
        partProperties = new TankBodyProperties { Friction = part.Friction, TankPart = true };
        return ref partProperties.Filter;
    }

    static BodyHandle CreateWheel(Simulation simulation, CollidableProperty<TankBodyProperties> properties,
        in RigidPose tankPose, in RigidPose bodyLocalPose,
        TypedIndex wheelShape, BodyInertia wheelInertia, float wheelFriction, BodyHandle bodyHandle,
        ref SubgroupCollisionFilter bodyFilter, ref SubgroupCollisionFilter secBodyFilter,
        System.Numerics.Vector3 bodyToWheelSuspension, float suspensionLength,
        in SpringSettings suspensionSettings, System.Numerics.Quaternion localWheelOrientation,
        ref QuickList<BodyHandle> wheelHandles, ref QuickList<ConstraintHandle> constraints,
        ref QuickList<ConstraintHandle> motors, ref QuickList<BodyHandle> bodyhandles)
    {
        RigidPose wheelPose;
        QuaternionEx.TransformUnitX(localWheelOrientation, out var suspensionDirection);
        RigidPose.Transform(bodyToWheelSuspension + suspensionDirection * suspensionLength, tankPose,
            out wheelPose.Position);
        QuaternionEx.ConcatenateWithoutOverlap(localWheelOrientation, tankPose.Orientation, out wheelPose.Orientation);

        var wheelHandle =
            simulation.Bodies.Add(BodyDescription.CreateDynamic(wheelPose, wheelInertia, wheelShape, 0.01f));
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
            LocalHingeAxisA =
                QuaternionEx.Transform(wheelRotationAxis, QuaternionEx.Conjugate(bodyLocalPose.Orientation)),
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
        wheelProperties = new TankBodyProperties
            { Filter = new SubgroupCollisionFilter(bodyHandle.Value, 3), Friction = wheelFriction, TankPart = true };
        //The wheels don't need to be tested against the body or each other.
        SubgroupCollisionFilter.DisableCollision(ref wheelProperties.Filter, ref bodyFilter);
        SubgroupCollisionFilter.DisableCollision(ref wheelProperties.Filter, ref wheelProperties.Filter);
        SubgroupCollisionFilter.DisableCollision(ref wheelProperties.Filter, ref secBodyFilter);


        return wheelHandle;
    }

    private void CreatePhysicsBody(BufferPool bufferPool, CollidableProperty<TankBodyProperties> properties)
    {
        if (Simulation == null || Terrain == null) return;

        var length = 4f;
        // Geometría de una rueda
        var wheelShape = new Cylinder(3.4f, length);
        var smallWheelShape = new Cylinder(3f, length);
        var smallerWheelShape = new Cylinder(2.3f, length);
        // Calcula el tensor de inercia para masa
        var wheelInertia = wheelShape.ComputeInertia(0.25f);
        // Registra la forma en el repositorio de Shapes de la simulación
        var wheelShapeIndex = Simulation.Shapes.Add(wheelShape);
        var smallWheelShapeIndex = Simulation.Shapes.Add(smallWheelShape);
        var smallerWheelShapeIndex = Simulation.Shapes.Add(smallerWheelShape);

        _tankDescription = new TankDescription
        {
            //Cuerpo del tanque
            Body = TankPartDescription.Create(1, new Box(36f, 5f, 60),
                new RigidPose(new System.Numerics.Vector3(0, 0, 0), System.Numerics.Quaternion.Identity), 0.5f,
                Simulation.Shapes),
            SecondaryBody = TankPartDescription.Create(0.1f, new Box(22f, 5f, 52.5f),
                new RigidPose(new System.Numerics.Vector3(0, -5, 0), System.Numerics.Quaternion.Identity), 0.5f,
                Simulation.Shapes),
            // Torreta, desplazado hacia arriba y adelante
            Turret = TankPartDescription.Create(1, new Cylinder(15f, 7f), new System.Numerics.Vector3(0f, 6f, 2.5f),
                0.5f, Simulation.Shapes),
            // Cañón
            Barrel = TankPartDescription.Create(0.1f, new Box(2f, 2f, 46f), new System.Numerics.Vector3(0, 6f, -34),
                0.5f, Simulation.Shapes),
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

        var alturaTerreno = Terrain.GetHeightAtPosition(Position.X, Position.Z);

        // Orienta el tanque para que “asiente” sobre la normal del terreno.
        var orientationQuat = Terrain.CalculateRotation(Position, Rotation);

        // Pose inicial
        var pose = new RigidPose(
            new System.Numerics.Vector3(Position.X, alturaTerreno + 25, Position.Z),
            new System.Numerics.Quaternion(orientationQuat.X, orientationQuat.Y, orientationQuat.Z,
                orientationQuat.W)
        );
        // Posición inicial y orientación alineada al terreno.
        var wheelHandles = new QuickList<BodyHandle>(_tankDescription.WheelCountPerTread * 2, bufferPool);
        var constraints =
            new QuickList<ConstraintHandle>(_tankDescription.WheelCountPerTread * 2 * 6 + 4, bufferPool);
        var leftMotors = new QuickList<ConstraintHandle>(_tankDescription.WheelCountPerTread, bufferPool);
        var rightMotors = new QuickList<ConstraintHandle>(_tankDescription.WheelCountPerTread, bufferPool);
        // Estructuras temporales para guardar handles de ruedas, constraints y motores. Se usan pools para evitar GC.
        ref var bodyFilter = ref CreatePart(Simulation, _tankDescription.Body, pose, properties, ref BodyHandles,
            out Body);
        ref var secBodyFilter = ref CreatePart(Simulation, _tankDescription.SecondaryBody, pose, properties,
            ref BodyHandles, out _secBody);
        ref var turretFilter = ref CreatePart(Simulation, _tankDescription.Turret, pose, properties,
            ref BodyHandles, out _turret);
        ref var barrelFilter = ref CreatePart(Simulation, _tankDescription.Barrel, pose, properties,
            ref BodyHandles, out _barrel);
        // Crea los cuerpos rígidos (body/torreta/cañón) en la simulación y obtiene sus filtros de colisión y handles.

        bodyFilter = new SubgroupCollisionFilter(Body.Value, 0);
        turretFilter = new SubgroupCollisionFilter(Body.Value, 1);
        barrelFilter = new SubgroupCollisionFilter(Body.Value, 2);
        secBodyFilter = new SubgroupCollisionFilter(Body.Value, 3);
        SubgroupCollisionFilter.DisableCollision(ref bodyFilter, ref turretFilter);
        SubgroupCollisionFilter.DisableCollision(ref turretFilter, ref barrelFilter);
        SubgroupCollisionFilter.DisableCollision(ref bodyFilter, ref secBodyFilter);
        // Define subgrupos de colisión para evitar colisiones internas entre cuerpo-torreta y torreta-cañón.
        Matrix3x3.CreateFromQuaternion(_tankDescription.TurretBasis, out var turretBasis);
        // Convierte la base de la torreta (quaternion) a una matriz 3x3.

        constraints.AllocateUnsafely() = Simulation.Solver.Add(Body, _secBody,
            new Weld
            {
                LocalOffset = new System.Numerics.Vector3(0f, -5f, 2.5f),
                LocalOrientation = System.Numerics.Quaternion.Identity,
                SpringSettings = new SpringSettings(30f, 1f)
            }
        );

        // Convierte la base de la torreta (quaternion) a una matriz 3x3.
        QuaternionEx.Transform(turretBasis.Y, QuaternionEx.Conjugate(_tankDescription.Body.Pose.Orientation),
            out var bodyLocalSwivelAxis);
        QuaternionEx.Transform(turretBasis.Y, QuaternionEx.Conjugate(_tankDescription.Turret.Pose.Orientation),
            out var turretLocalSwivelAxis);
        RigidPose.TransformByInverse(_tankDescription.TurretAnchor, _tankDescription.Body.Pose,
            out var bodyLocalTurretAnchor);
        RigidPose.TransformByInverse(_tankDescription.TurretAnchor, _tankDescription.Turret.Pose,
            out var turretLocalTurretAnchor);
        // Calcula ejes locales de giro (swivel) y puntos de anclaje en los espacios locales de cuerpo y torreta.

        constraints.AllocateUnsafely() = Simulation.Solver.Add(Body, _turret,
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
        QuaternionEx.ConcatenateWithoutOverlap(turretSwivelBasisQuaternion,
            QuaternionEx.Conjugate(_tankDescription.Body.Pose.Orientation), out var bodyLocalTurretBasis);
        QuaternionEx.ConcatenateWithoutOverlap(turretSwivelBasisQuaternion,
            QuaternionEx.Conjugate(_tankDescription.Turret.Pose.Orientation), out var turretLocalTurretBasis);
        // Convierte esa base a los espacios locales de cuerpo y torreta.

        _turretServoDescription = new TwistServo
        {
            LocalBasisA = bodyLocalTurretBasis,
            LocalBasisB = turretLocalTurretBasis,
            SpringSettings = _tankDescription.TurretSpring,
            ServoSettings = _tankDescription.TurretServo
        };
        _turretServo = Simulation.Solver.Add(Body, _turret, _turretServoDescription);
        constraints.AllocateUnsafely() = _turretServo;
        // Crea y agrega un TwistServo para controlar el ángulo de la torreta (yaw), usando esa base de medida.

        QuaternionEx.Transform(turretBasis.X, QuaternionEx.Conjugate(_tankDescription.Turret.Pose.Orientation),
            out var turretLocalPitchAxis);
        QuaternionEx.Transform(turretBasis.X, QuaternionEx.Conjugate(_tankDescription.Barrel.Pose.Orientation),
            out var barrelLocalPitchAxis);
        RigidPose.TransformByInverse(_tankDescription.BarrelAnchor, _tankDescription.Turret.Pose,
            out var turretLocalBarrelAnchor);
        RigidPose.TransformByInverse(_tankDescription.BarrelAnchor, _tankDescription.Barrel.Pose,
            out var barrelLocalBarrelAnchor);
        // Prepara ejes locales de pitch y puntos de anclaje para el cañón respecto a la torreta.

        constraints.AllocateUnsafely() = Simulation.Solver.Add(_turret, _barrel,
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
        QuaternionEx.ConcatenateWithoutOverlap(barrelPitchBasisQuaternion,
            QuaternionEx.Conjugate(_tankDescription.Turret.Pose.Orientation), out var turretLocalBarrelBasis);
        QuaternionEx.ConcatenateWithoutOverlap(barrelPitchBasisQuaternion,
            QuaternionEx.Conjugate(_tankDescription.Barrel.Pose.Orientation), out var barrelLocalBarrelBasis);
        // Bases convertidas a espacios locales.

        _barrelServoDescription = new TwistServo
        {
            LocalBasisA = turretLocalBarrelBasis,
            LocalBasisB = barrelLocalBarrelBasis,
            SpringSettings = _tankDescription.BarrelSpring,
            ServoSettings = _tankDescription.BarrelServo
        };
        // Agrega el TwistServo que controla el pitch del cañón.
        _barrelServo = Simulation.Solver.Add(_turret, _barrel, _barrelServoDescription);
        constraints.AllocateUnsafely() = _barrelServo;
        // Agrega el TwistServo que controla el pitch del cañón.

        QuaternionEx.TransformUnitY(_tankDescription.WheelOrientation, out _);
        QuaternionEx.TransformUnitZ(_tankDescription.WheelOrientation, out var treadDirection);
        // Obtiene los ejes “rueda” (giro) y “oruga” (dirección de avance) en mundo local de rueda.

        var treadStart = _tankDescription.TreadSpacing * (_tankDescription.WheelCountPerTread - 1) * -0.5f;
        // Punto inicial para distribuir ruedas centradas a lo largo de la oruga.
        BodyHandle previousLeftWheelHandle = default, previousRightWheelHandle = default;
        for (var i = 0; i < _tankDescription.WheelCountPerTread; ++i)
        {
            var wheelOffsetFromTread = treadDirection * (treadStart + i * _tankDescription.TreadSpacing);
            // Offset longitudinal de cada rueda a lo largo de la oruga.
            var verticalLift = 0f;
            var wheelShapeToUse = _tankDescription.WheelShape;
            if (i == 0)
            {
                wheelOffsetFromTread += treadDirection * (_tankDescription.TreadSpacing / 4);
                verticalLift = 3f;
                wheelShapeToUse = _tankDescription.SmallerWheelShape;
            }
            else if (i == _tankDescription.WheelCountPerTread - 1)
            {
                verticalLift = 3f;
                wheelShapeToUse = _tankDescription.SmallWheelShape;
            }

            var rightSuspensionOffset = _tankDescription.RightTreadOffset + wheelOffsetFromTread -
                                        _tankDescription.Body.Pose.Position;
            var leftSuspensionOffset = _tankDescription.LeftTreadOffset + wheelOffsetFromTread -
                                       _tankDescription.Body.Pose.Position;

            // Aplicar elevación en eje Y
            rightSuspensionOffset.Y += verticalLift;
            leftSuspensionOffset.Y += verticalLift;

            var rightWheelHandle = CreateWheel(Simulation, properties, pose, _tankDescription.Body.Pose,
                wheelShapeToUse, _tankDescription.WheelInertia, _tankDescription.WheelFriction, Body,
                ref properties[Body].Filter, ref properties[_secBody].Filter,
                rightSuspensionOffset, _tankDescription.SuspensionLength, _tankDescription.SuspensionSettings,
                _tankDescription.WheelOrientation,
                ref wheelHandles, ref constraints, ref rightMotors, ref BodyHandles);

            // Crea una rueda derecha con suspensión, fricción y orientación definidos; guarda handles y constraints.

            var leftWheelHandle = CreateWheel(Simulation, properties, pose, _tankDescription.Body.Pose,
                wheelShapeToUse, _tankDescription.WheelInertia, _tankDescription.WheelFriction, Body,
                ref properties[Body].Filter, ref properties[_secBody].Filter,
                leftSuspensionOffset, _tankDescription.SuspensionLength, _tankDescription.SuspensionSettings,
                _tankDescription.WheelOrientation,
                ref wheelHandles, ref constraints, ref leftMotors, ref BodyHandles);
            // Crea la rueda izquierda correspondiente.

            if (i >= 1)
            {
                // Conecta ruedas consecutivas de una misma oruga para compartir esfuerzo de tracción.
                // Motor angular que busca velocidad relativa 0 (igualar giro, permitiendo algo de deriva).
                var motorDescription = new AngularAxisMotor
                {
                    LocalAxisA = new System.Numerics.Vector3(0, 1, 0),
                    Settings = new MotorSettings(float.MaxValue, 1e-4f)
                };
                constraints.AllocateUnsafely() =
                    Simulation.Solver.Add(previousLeftWheelHandle, leftWheelHandle, motorDescription);
                constraints.AllocateUnsafely() =
                    Simulation.Solver.Add(previousRightWheelHandle, rightWheelHandle, motorDescription);
            }

            previousLeftWheelHandle = leftWheelHandle;
            previousRightWheelHandle = rightWheelHandle;
        }

        wheelHandles.Span.Slice(wheelHandles.Count);
        constraints.Span.Slice(constraints.Count);
        LeftMotors = leftMotors.Span.Slice(leftMotors.Count);
        RightMotors = rightMotors.Span.Slice(rightMotors.Count);
        // “Cierra” las QuickList exposando los spans finales (ajusta las vistas a su tamaño real).

        QuaternionEx.ConcatenateWithoutOverlap(_tankDescription.Body.Pose.Orientation,
            QuaternionEx.Conjugate(_tankDescription.TurretBasis), out _fromBodyLocalToTurretBasisLocal);
        // Guarda conversión de base local del cuerpo a la base de la torreta (útil para apuntado).

        QuaternionEx.Transform(-turretBasis.Z, QuaternionEx.Conjugate(_tankDescription.Barrel.Pose.Orientation),
            out _barrelLocalDirection);
        // Direccion “hacia adelante” del cañón en su espacio local (para raycasts/disparo).

        RotationQuaternion = orientationQuat;
        // Guarda la orientación inicial del tanque (alineada al terreno).
    }

    protected void UpdateCanonAndTurretTowards()
    {
        if (IsDead) return;
        // Leer las poses actuales del cuerpo, torreta y cañón desde la simulación
        var bodyRef = Simulation.Bodies.GetBodyReference(Body);
        var turretRef = Simulation.Bodies.GetBodyReference(_turret);
        var barrelRef = Simulation.Bodies.GetBodyReference(_barrel);

        var qBody = bodyRef.Pose.Orientation;
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

    protected void UpdateWheelSpinByDistance()
    {
        var delta = Position - LastPos;
        var dist = delta.Length();

        // Dirección “forward” actual para signo (+ avanza / - retrocede)
        var forward = Vector3.Transform(-Vector3.UnitZ, Matrix.CreateFromQuaternion(RotationQuaternion));
        float sign = 0f;
        if (dist > 0.0001f)
        {
            var dir = Vector3.Normalize(delta);
            sign = MathF.Sign(Vector3.Dot(dir, forward));
        }

        WheelRotation += sign * (dist / (WheelRadius));

        if (WheelRotation > MathHelper.TwoPi) WheelRotation -= MathHelper.TwoPi;
        else if (WheelRotation < -MathHelper.TwoPi) WheelRotation += MathHelper.TwoPi;

        LastPos = Position;
    }

    public void SyncFromPhysics()
    {
        if (IsDead) return;
        var body = Simulation.Bodies.GetBodyReference(Body);
        var pose = body.Pose;

        Position = new Vector3(pose.Position.X, pose.Position.Y, pose.Position.Z);
        var q = pose.Orientation;
        RotationQuaternion = new Quaternion(q.X, q.Y, q.Z, q.W);
    }

    protected void UpdateWorldMatrix()
    {
        if (IsDead) return;

        // Sincronizar posición y rotación desde la física
        var pose = Simulation.Bodies.GetBodyReference(Body).Pose;

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
        World =
            Matrix.CreateScale(Scale) *
            Matrix.CreateFromQuaternion(RotationQuaternion) *
            Matrix.CreateTranslation(visualPosition);
    }

    public void DrawDebug()
    {
        RigidPose pose = Simulation.Bodies.GetBodyReference(Body).Pose;
        var scale = Matrix.CreateScale(1f, 1f, 1f); // tamaño del cuerpo (Box)
        var rotation = Matrix.CreateFromQuaternion(pose.Orientation);
        var translation = Matrix.CreateTranslation(pose.Position);

        var matriz = scale * rotation * translation;
        Gizmos.DrawCube(matriz, Color.Blue);


        var turretAnchor = Matrix.CreateTranslation(_tankDescription.TurretAnchor);
        var barrelAnchor = Matrix.CreateTranslation(_tankDescription.BarrelAnchor);
        Gizmos.DrawCube(turretAnchor * matriz, Color.Green);
        Gizmos.DrawCube(barrelAnchor * matriz, Color.Red);

        Matrix[] boneTransforms = new Matrix[Model.Bones.Count];
        Model.CopyAbsoluteBoneTransformsTo(boneTransforms);

        var boneWorldTransform = boneTransforms[_turretBone.Index] * World;
        var boneWorldPosition = boneWorldTransform.Translation;

        var boneMatrix = Matrix.CreateScale(1f) * Matrix.CreateTranslation(boneWorldPosition);
        Gizmos.DrawCube(boneMatrix, Color.Orange);

        Gizmos.Draw();
    }

    public void Draw(Camera camera, RenderTarget2D shadowMapRenderTarget, int shadowmapSize,
        TargetCamera targetLightCamera)
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

        _effect.CurrentTechnique = _effect.Techniques["BasicDrawing"];
        _effect.Parameters["shadowMap"]?.SetValue(shadowMapRenderTarget);
        _effect.Parameters["shadowMapSize"]?.SetValue(Vector2.One * shadowmapSize);
        _effect.Parameters["LightViewProjection"]?.SetValue(targetLightCamera.View * targetLightCamera.Projection);

        var impactPointsArray = new Vector4[MaxImpacts];
        var used = Math.Min(ImpactsLocal.Count, MaxImpacts);
        for (int i = 0; i < used; i++)
        {
            var imp = ImpactsLocal[i];
            var boneWorld = absBones[imp.BoneIndex] * World;
            var worldPos = Vector3.Transform(imp.Local, boneWorld);
            impactPointsArray[i] = new Vector4(worldPos, imp.Radius);
        }

        _effect.Parameters["ImpactPoints"]?.SetValue(impactPointsArray);

        _effect.Parameters["View"]?.SetValue(camera.View);
        _effect.Parameters["Projection"]?.SetValue(camera.Projection);

        foreach (var mesh in Model.Meshes)
        {
            foreach (var part in mesh.MeshParts)
                part.Effect = _effect;

            _effect.Parameters["ModelTexture"]?.SetValue(Texture);
            if (mesh.Name.Contains("Treadmill"))
            {
                _effect.Parameters["ModelTexture"]?.SetValue(treadmillsTexture);
            }

            var worldPerMesh = absBones[mesh.ParentBone.Index] * World;
            
            _effect.Parameters["World"]?.SetValue(worldPerMesh);
            _effect.Parameters["InverseTransposeWorld"]?.SetValue(Matrix.Transpose(Matrix.Invert(worldPerMesh)));

            mesh.Draw();
        }
    }

    // Devuelve posición y dirección de la boca del cañón, tomando el hueso real del cañón
    private (Vector3 pos, Vector3 dir) GetMuzzle(float muzzleOffsetLocal = 300.2f)
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
        var cannonWorld = cannonAbs * World; // mismo patrón que en Draw()

        // La dirección “forward” del cañón 
        var f = -Vector3.Normalize(GetUp(cannonWorld));

        // Posición del muzzle: origen del hueso + corrimiento a lo largo del cañón
        var origin = GetTranslation(cannonWorld);
        var muzzle = origin + f * (muzzleOffsetLocal * Scale);

        return (muzzle, f);
    }
    
    // Dispara un pulso de retroceso y, opcionalmente, activa freno momentáneo.
    private void TriggerRecoil(Vector3 fireDirXna,
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
        RecoilTime = RecoilDuration;

        if (withBrake)
        {
            BrakeTime = BrakeDuration;
        }
    }

    public void ApplyRecoilAndBrake(float dt, Simulation simulation)
    {
        if (IsDead) return;
        // Referencia al cuerpo físico del tanque
        var bodyRef = simulation.Bodies.GetBodyReference(Body); // usa tu handle del tanque

        // Retroceso: empuja en dirección opuesta por un tiempo corto
        if (RecoilTime > 0f)
        {
            bodyRef.Velocity.Linear += _recoilAccelSys * dt;
            RecoilTime -= dt;
        }

        // Freno: arrastre proporcional a la velocidad actual 
        if (BrakeTime > 0f)
        {
            var v = bodyRef.Velocity.Linear;
            var drag = -v * (BrakeK * dt);
            bodyRef.Velocity.Linear += drag;
            BrakeTime -= dt;
        }
    }

    public void RecibirAtaque(float danio)
    {
        Vida -= danio;

        if (Vida <= 0f)
            Kill();
    }

    public virtual void Kill()
    {
        if (IsDead) return;
        IsDead = true;

        // Detener todos los sonidos del tanque
        Audio?.StopAll();
    }

    protected virtual void ResetCooldown()
    {
        FireCooldown = TipoProyectilActual.MaxCooldown;
    }

    public void Shoot(Simulation simulation, List<Projectile> projectiles, Effect projectileEffect, CollidableProperty<TankBodyProperties> properties)
    {
        if (IsDead) return;
        if (FireCooldown > 0f) return;

        var (muzzle, dir) = GetMuzzle();

        var proj = new Projectile(simulation, projectileEffect, muzzle, dir, TipoProyectilActual, this, properties);
        projectiles.Add(proj);

        /*TriggerRecoil(
            dir,
            projectileMass: TipoProyectilActual.Mass,
            muzzleSpeed: TipoProyectilActual.Speed,
            intensity: 1f,
            withBrake: true);*/

        ResetCooldown();

        Audio?.PlayShoot(TipoProyectilActual);
    }
}