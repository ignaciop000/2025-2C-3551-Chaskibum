// LISTA DE TAREAS
// - Agregar Postes [SANTI]
// - Incorporar otros tanques (sin IA) [SANTI]
// - Mejorar movimiento tanque [MATEO]
// - Poner las texturas de tanque y otros objetos [SANTI]
// - Disparar proyectiles [NACHO]
// - Colisión entre tanques y objetos (con disparos se rompen todos, pero arboles y arbustos se rompen tambien con el tanque) [AGUS] [COMPLETADO]

// - Opcionales:
// - Corregir angulo arbustos, rocas y casas para que sigan el piso
// - Los objetos solo spawnean si el angulo es menor a X°, dependiendo del objeto. Casas muy bajo, el resto un poco mas, rocas no tienen restriccion
// - Suavizar el mapa [COMPLETADO]
// - Que la camara no traspase el piso
// - Arreglar los arbustos, que a veces vuelan y otras estan bajo tierra
// - Que las casas se coloquen manualmente

using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;
using Demos.Demos.Tanks;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using TGC.MonoGame.Samples.Cameras;
using TGC.MonoGame.Samples.Viewer.Gizmos;
using MathHelper = BepuUtilities.MathHelper;
using Matrix = Microsoft.Xna.Framework.Matrix;
using Vector2 = System.Numerics.Vector2;
using Vector3 = System.Numerics.Vector3;

namespace TGC.MonoGame.TP;

/// <summary>
///     Esta es la clase principal del juego.
///     Inicialmente puede ser renombrado o copiado para hacer mas ejemplos chicos, en el caso de copiar para que se
///     ejecute el nuevo ejemplo deben cambiar la clase que ejecuta Program <see cref="Program.Main()" /> linea 10.
/// </summary>
public class TGCGame : Game
{
    private float _escalaMapa = 20;
    
    public const string ContentFolder3D = "Models/";
    public const string ContentFolderEffects = "Effects/";
    public const string ContentFolderMusic = "Music/";
    public const string ContentFolderSounds = "Sounds/";
    public const string ContentFolderSpriteFonts = "SpriteFonts/";
    public const string ContentFolderTextures = "Textures/";

    private readonly GraphicsDeviceManager _graphics;
    private readonly Random _rnd = new Random();
    private OrbitCamera _orbitCamera;
    private Camera _camera;
    public Vector3 DesiredLookAt;
    public bool hay_lookAt;
    public Vector3 LookAt;
    public Vector2 pos;
    private Effect _terrainEffect;
    private Effect _effect;
    private Effect _debugEffect;
    private Simulation _simulation;
    public Gizmos Gizmos { get; set;}
    public BufferPool bufferPool { get; private set; }

    private PositionGenerator _positionGenerator;
    private float angle;
    public Terrain terrain;

    private bool _showTerrainMeshDebug = false;
    private KeyboardState _kbPrev;
    private bool _showTankTelemetry = false;
    private SpriteBatch _spriteBatch;
    private SpriteFont _debugFont; 
    public TankController PlayerController;
    
    private Tank _tank;
    // Proyectiles
    private readonly List<Projectile> _missiles = new();
    private MouseState _mousePrev;
    private float _fireCooldown = 0f;

    //private ModelInstances _tank2 = new ModelInstances(new Color(15, 15, 15));
    //private ModelInstances _panzer = new ModelInstances(new Color(0, 39, 77));
    //private ModelInstances _t90 = new ModelInstances(new Color(95, 96, 98));

    private Houses _houses;
    private Rocks _rocks;
    private Trees _trees;
    private Bushes _bushes;
    
    // Diccionario para mapear StaticHandle a ModelGroup
    public static readonly Dictionary<StaticHandle, ModelGroup> HandleToGroup = new();

    private Debug _debug;

    /// <summary>
    ///     Constructor del juego.
    /// </summary>
    public TGCGame()
    {
        // Maneja la configuracion y la administracion del dispositivo grafico.
        _graphics = new GraphicsDeviceManager(this);

        _graphics.PreferredBackBufferWidth = GraphicsAdapter.DefaultAdapter.CurrentDisplayMode.Width - 100;
        _graphics.PreferredBackBufferHeight = GraphicsAdapter.DefaultAdapter.CurrentDisplayMode.Height - 100;
        // Para que el juego sea pantalla completa se puede usar Graphics IsFullScreen.
        
        // Carpeta raiz donde va a estar toda la Media.
        Content.RootDirectory = "Content";
        
        // Hace que el mouse sea visible.
        IsMouseVisible = true;
    }

    /// <summary>
    ///     Se llama una sola vez, al principio cuando se ejecuta el ejemplo.
    ///     Escribir aqui el codigo de inicializacion: el procesamiento que podemos pre calcular para nuestro juego.
    /// </summary>
    protected override void Initialize()
    {
        Gizmos = new Gizmos();
        bufferPool = new BufferPool();
        DesiredLookAt = Vector3.Zero;
        pos = Vector2.Zero;
        
        // Inicialización de cámaras
        
        _orbitCamera = new OrbitCamera(
            GraphicsDevice.Viewport.AspectRatio, 
            Vector3.Zero, 
            800f, 
            5, 
            50000
            );
        
        // Seteo la cámara inicial como la orbital
        _camera = _orbitCamera;
        
        var narrowPhase = new NarrowPhaseCallbacks();
        narrowPhase.OnCollision = pair =>
        {
            CollidableReference estatico, movil;

            if (pair.A.Mobility == CollidableMobility.Static && pair.B.Mobility != CollidableMobility.Static)
            {
                estatico = pair.A;
                movil = pair.B;
            }
            else if (pair.B.Mobility == CollidableMobility.Static && pair.A.Mobility != CollidableMobility.Static)
            {
                estatico = pair.B;
                movil = pair.A;
            }
            else
            {
                return; // no nos interesa este caso
            }

            var staticHandle = estatico.StaticHandle;
            if (!HandleToGroup.TryGetValue(staticHandle, out var group))
                return;

            if (movil.BodyHandle == _tank.PhysicsBody)
            {
                group.OnCollisionWithTank(staticHandle);
            }
            else if (_missiles.Select(projectil => projectil.Body).Contains(movil.BodyHandle))
            {
                group.OnCollisionWithProjectile(staticHandle);
            }
        };

        
        _simulation = Simulation.Create(bufferPool, narrowPhase,
            new PoseIntegratorCallbacks(new Vector3(0, -120, 0)), new SolveDescription(8, 1));
        
        _tank = new Tank(new Vector3(0, 0, 1000), _orbitCamera, 0f, 10f );
        
        _debug = new Debug();
        base.Initialize();
    }

    /// <summary>
    /// Se ejecuta una vez al inicio del juego, inmediatamente después de Initialize.
    /// Carga y configura los recursos esenciales necesarios para el juego, como efectos, texturas, modelos y estructuras de optimización.
    /// Aquí también se puede realizar cualquier preprocesamiento requerido antes del inicio del ciclo del juego.
    /// </summary>
    protected override void LoadContent()
    {
        _terrainEffect = Content.Load<Effect>(ContentFolderEffects + "Terrain");
        _effect = Content.Load<Effect>(ContentFolderEffects + "BasicShader");
        
        // heights
        var terrainHeigthmap = Content.Load<Texture2D>(ContentFolderTextures + "heightmaps/heightmap");
        // basic color
        var terrainColorMap = Content.Load<Texture2D>(ContentFolderTextures + "heightmaps/colormap");
        // blend texture 1
        var terrainGrass = Content.Load<Texture2D>(ContentFolderTextures + "grass");
        // blend texture 2
        var terrainGround = Content.Load<Texture2D>(ContentFolderTextures + "ground");

        terrain = new Terrain(GraphicsDevice, 
            terrainHeigthmap, 
            terrainColorMap, 
            terrainGrass, 
            terrainGround, 
            _terrainEffect,
            _simulation,
            _escalaMapa
            );

        _tank.CargarModelo("tank/tank", _terrainEffect, Content, _simulation, bufferPool, GraphicsDevice, Gizmos,terrain);
        PlayerController = new TankController(_tank, 20, 5, 2, 1, 3.5f);
        //_tank2.CargarModelo("tank/tank", _effect, Content);
        //_panzer.CargarModelo("panzer/Panzer", _effect, Content);
        //_t90.CargarModelo("t90/T90", _effect, Content);
        _trees = new Trees(terrain, _simulation);
        _houses = new Houses(terrain, _simulation);
        _rocks = new Rocks(terrain, _simulation);
        _bushes = new Bushes(terrain, _simulation);

        // Generacion de posiciones de modelos

        var anchoMapa = (terrain.HeightmapData.GetLength(0) - 1) * _escalaMapa;
        var largoMapa = (terrain.HeightmapData.GetLength(1) - 1) * _escalaMapa;
        
        _positionGenerator = new PositionGenerator(anchoMapa, largoMapa);
        var modelos = _trees.GetModelosConPorcentaje(0.60) // Arboles
            .Concat(_rocks.GetModelosConPorcentaje(0.35)) // Rocas
            .Concat(_houses.GetModelosConPorcentaje(0.05)) // Casas
            .ToList();
        _positionGenerator.AgregarPosiciones(modelos);

        // Genero otros puntos para los arbustos
        var arbustos = _bushes.GetModelosConPorcentaje(1.0);
        _positionGenerator.AgregarPosiciones(arbustos, 450);

        _trees.CrearObjetos();
        _rocks.CrearObjetos();
        _houses.CrearObjetos();
        _bushes.CrearObjetos();
        
        _trees.CargarModelos(_effect, Content);
        _houses.CargarModelos(_effect, Content);
        _rocks.CargarModelos(_effect, Content);
        _bushes.CargarModelos(_effect, Content);
        
        _debug.LoadContent(
            Content, 
            ContentFolderEffects, 
            ContentFolderSpriteFonts, 
            GraphicsDevice, 
            _tank, 
            _orbitCamera,
            _simulation, 
            terrain,
            Gizmos
        );
        
        base.LoadContent();
    }

    /// <summary>
    ///     Se llama en cada frame.
    ///     Se debe escribir toda la logica de computo del modelo, asi como tambien verificar entradas del usuario y reacciones
    ///     ante ellas.
    /// </summary>
    protected override void Update(GameTime gameTime)
    {
        var keyboardState = Keyboard.GetState();
        var mouseState = Mouse.GetState();
        var deltaTime = (float)gameTime.ElapsedGameTime.TotalSeconds;
        
        // cooldown
        _fireCooldown = MathF.Max(0f, _fireCooldown - deltaTime);
        
        _tank?.Update(gameTime, keyboardState, mouseState, _orbitCamera.FrontDirection);
        
        // click izquierdo: dispara
        if (_fireCooldown <= 0f && mouseState.LeftButton == ButtonState.Pressed && _mousePrev.LeftButton == ButtonState.Released)
        {

            // Velocidad configurable acá:
            float speed = 300f;
            float projMass = 2f;
            // Uso el mismo efecto de debug para dibujar el proyectil sin assets extra
            var (muzzle, dir) = _tank.GetMuzzle(3.2f); // offset local del cañón (ajustá a tu modelo)
            var proj = new Projectile(_simulation, terrain, _debug.DebugEffect, muzzle, dir, speed);
            _missiles.Add(proj);
            
            // Retroceso + freno breve
            _tank.TriggerRecoil(dir, projectileMass: projMass, muzzleSpeed: speed, intensity: 1f, withBrake: true);

            _fireCooldown = 1f; // 4 disparos/seg
        }
        
        // update de todos los proyectiles
        for (int i = _missiles.Count - 1; i >= 0; --i)
        {
            _missiles[i].Update(deltaTime);
            if (_missiles[i].IsDead) _missiles.RemoveAt(i);
        }

        
        // Actualizar simulación física
        if (_simulation != null && deltaTime > 0.0f && deltaTime < 0.1f) // Máximo 100ms por frame
        {
            _simulation.Timestep(deltaTime);
            _tank?.SyncFromPhysics();
            _tank.ApplyRecoilAndBrake(deltaTime, _simulation);
        }

        // Capturar Input teclado
        if (keyboardState.IsKeyDown(Keys.Escape))
        {
            //Salgo del juego.
            Exit();
        }

        if (keyboardState.IsKeyDown(Keys.F4) && !_kbPrev.IsKeyDown(Keys.F4))
        {
            if (_camera == _orbitCamera)
            {
                var size = GraphicsDevice.Viewport.Bounds.Size;
                size.X /= 2;
                size.Y /= 2;
                _camera = new FreeCamera(GraphicsDevice.Viewport.AspectRatio, _orbitCamera.Position, _orbitCamera.FrontDirection, size);
            } else {
                _camera = _orbitCamera;
            }
        }
        
        _debug.Update(keyboardState, _kbPrev, deltaTime, _camera);

        // Actualizar cámara para seguir al tanque
        if (_tank != null)
        {
            // Usar la posición y rotación del tanque
            var targetHeight = terrain.GetHeightAtPosition(_tank.Position.X, _tank.Position.Z) + 50f; 
            _orbitCamera.SetTarget(new Vector3(_tank.Position.X, targetHeight, _tank.Position.Z));
            var dir = new Vector2(MathF.Cos(_tank.Rotation), MathF.Sin(_tank.Rotation));

            DesiredLookAt = new Vector3(_tank.Position.X, terrain.GetHeightAtPosition(_tank.Position.X, _tank.Position.Z), _tank.Position.Z);
            if (!hay_lookAt)
            {
                LookAt = DesiredLookAt;
                hay_lookAt = true;
            }
            else
            {
                var lamda = 0.05f;
                LookAt = DesiredLookAt * lamda + LookAt * (1 - lamda);
            }

            var tankPos2D = new Vector2(_tank.Position.X, _tank.Position.Z);
            var cameraPos2D = tankPos2D - dir * 800; // Distancia de 800 unidades detrás del tanque

            // Calcular la altura máxima entre la cámara y el tanque
            float H = 0;
            for (var i = 0; i < 10; ++i)
            {
                var t = i / 10.0f;
                var p = cameraPos2D * t + tankPos2D * (1 - t);
                var Hi = terrain.GetHeightAtPosition(p.X, p.Y) + 50;
                if (Hi > H) H = Hi;
            }
            
            // Actualizar la cámara (maneja el input del mouse)
            _camera.Update(gameTime);
            Gizmos.UpdateViewProjection(_camera.View, _camera.Projection);
        }
        
        _kbPrev = keyboardState;
        _mousePrev = mouseState;
        base.Update(gameTime);
    }

    /// <summary>
    ///     Se llama cada vez que hay que refrescar la pantalla.
    ///     Escribir aqui el codigo referido al renderizado.
    /// </summary>
    protected override void Draw(GameTime gameTime)
    {
        GraphicsDevice.Clear(Color.Black);
        // Limpia también el depth buffer
        GraphicsDevice.Clear(ClearOptions.Target | ClearOptions.DepthBuffer, Color.Black, 1f, 0);

        // Estados por defecto para 3D
        GraphicsDevice.BlendState = BlendState.Opaque;
        GraphicsDevice.DepthStencilState = DepthStencilState.Default;
        GraphicsDevice.RasterizerState = RasterizerState.CullCounterClockwise;
        GraphicsDevice.SamplerStates[0] = SamplerState.LinearWrap;

        // Verificar que el efecto y el terreno no sean nulos antes de dibujar
        if (_terrainEffect == null || terrain == null)
            return;

        // Para dibujar el modelo necesitamos pasarle informacion que el efecto esta esperando.
        _terrainEffect.Parameters["View"].SetValue(_camera.View);
        _terrainEffect.Parameters["Projection"].SetValue(_camera.Projection);
        
        _effect.Parameters["View"].SetValue(_camera.View);
        _effect.Parameters["Projection"].SetValue(_camera.Projection);

        var oldRasterizerState = GraphicsDevice.RasterizerState;
        GraphicsDevice.RasterizerState = RasterizerState.CullNone;
        terrain.Draw(Matrix.Identity, _camera.View, _camera.Projection);
        GraphicsDevice.RasterizerState = oldRasterizerState;
        
        _tank.Draw();
        //_panzer.Dibujar();
        //_t90.Dibujar();

        _trees.Dibujar();
        _houses.Dibujar();
        _rocks.Dibujar();
        _bushes.Dibujar();
        
        foreach (var s in _missiles)
            s.Draw(GraphicsDevice, _camera.View, _camera.Projection);

        _debug.Draw();

    }

    /// <summary>
    ///     Libero los recursos que se cargaron en el juego.
    /// </summary>
    protected override void UnloadContent()
    {
        // Libero los recursos.
        Content.Unload();

        base.UnloadContent();
    }
}