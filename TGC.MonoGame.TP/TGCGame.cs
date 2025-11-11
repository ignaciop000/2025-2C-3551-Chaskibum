// ARREGLOS
// - Separar en clases distintas TanqueJugador y TanqueEnemigo (ambas heredan de Tanque) [AGUS]
// - Los tanques no deben spawnear donde hay obstaculos + no deben spawnear volando [AGUS]
// - Que se vea el debug de los proyectiles [NACHO]
// - Bajar volumenes altos [SANTI]
// - Arreglar el árbol raro [SANTI]
// - Cielo y niebla [SANTI]

// NUEVAS TAREAS
// - Terminar Shadow Map [MATEO]
// - Modo God [MATEO]
// - Verificar iluminación Blinn-Phong sobre todos los vehículos, elementos del entorno y terreno [MATEO]
// - Mejorar IA tanques: deben disparar al jugador (y este debe perder vida) [AGUS]
// - Deformación tanques [NACHO]
// - Animación ruedas con Texture Scrolling [SANTI]

// FINALMENTE [TODOS]
// - Separar los .cs en carpetas
// - Emprolijar código (principalmente físicas, pero no estaria mal diseñar mejor algunas cosas)
// - Emprolijar shaders (borrarles cosas innecesarias o unificarlos si se puede)

// ERRORES A SOLUCIONAR [TODOS]
// - A veces algunos proyectiles rebotan en el piso
// - Ver si se solucionó el bug del suicidio
// - Error de BEPU de que un valor es nan o infinito
// - Error de index out of boundaries en GetHeightAtPosition llamado desde TGCGame

// OPCIONALES [EL QUE QUIERA]
// - Hacer volumen configurable
// - Imagen tutorial [SANTI]
// - Textura terreno con normales
// - Que los cambios en el menú sean no solo skin, si no tmb tamaño, vida, daño base, velocidad
// - Crear algún sistema de partículas que muestren humo, fuego, chispas, etc.
// - Delineado tanques ocultos
// - Agregar un efecto de Bloom para los disparos, y el humo/fuego si fue implementado.
// - Modo versus

using System;
using System.Collections.Generic;
using System.Linq;
using BepuPhysics;
using BepuUtilities.Memory;
using ImGuiNET;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework.Media;
using TGC.MonoGame.Samples.Viewer.GUI.ImGuiNET;
using TGC.MonoGame.TP.Viewer.Gizmos;
using Matrix = Microsoft.Xna.Framework.Matrix;
using Vector3 = System.Numerics.Vector3;

namespace TGC.MonoGame.TP;

/// <summary>
///     Esta es la clase principal del juego.
///     Inicialmente puede ser renombrado o copiado para hacer mas ejemplos chicos, en el caso de copiar para que se
///     ejecute el nuevo ejemplo deben cambiar la clase que ejecuta Program <see cref="Program.Main()" /> linea 10.
/// </summary>
public class TGCGame : Game
{
    //Debug
    private Gizmos Gizmos { get; set;}
    private ImGuiRenderer _imGuiRenderer;
    private bool dibujar = true;
    private BoundingFrustum _boundingFrustum;
    
    public const string ContentFolder3D = "Models/";
    public const string ContentFolderEffects = "Effects/";
    public const string ContentFolderMusic = "Music/";
    public const string ContentFolderSounds = "Sounds/";
    public const string ContentFolderSpriteFonts = "SpriteFonts/";
    public const string ContentFolderTextures = "Textures/";

    private enum GameState { MainMenu, Playing }
    private GameState _state = GameState.MainMenu;
    
    private const float EscalaMapa = 30;
    private readonly GraphicsDeviceManager _graphics;
    private RenderTarget2D _shadowMapRenderTarget;
    
    private TargetCamera _targetLightCamera;
    private const int ShadowmapSize = 4096;
    
    private Camera _camera;                 // Cámara activa
    private OrbitCamera _orbitCamera;       // Cámara que sigue al tanque
    private const float LightCameraFarPlaneDistance = 20000f;
    private const float LightCameraNearPlaneDistance = 5f;
    
    private Effect _terrainEffect;          //Shader Terreno
    private Effect _effect;                 //Shader Basico
    private Effect _shadowEffect;
    private Effect _worldBorderEffect;      //Shader WorldBorder
    private Vector3 _lightPosition;
    private Simulation _simulation;
    private CollidableProperty<TankBodyProperties> _bodyProperties;     // Propiedades por colisionable (tanques)
    private TankCallbacks _callbacks;                                   // Callbacks de BEPU para fuerzas/colisiones
    private CollisionHandler _collisionHandler;                         // Maneja eventos de colisión de juego
    
    private BufferPool BufferPool { get; set; }                         // Pool de buffers BEPU para performance

    private PositionGenerator _positionGenerator;
    private Terrain _terrain;
    private WorldBorder _worldBorder;
    
    private KeyboardState _kbPrev;
    
    private TankController _playerController;
    private int _enemyCount;

    private Tank _tank;
    private List<Tank> _enemyTanks;
    private List<TankController> _enemyControllers;
    private List<Tank> _tanks;

    private Effect _tankShader;
    // Proyectiles
    private readonly List<Projectile> _projectiles = [];
    private MouseState _mousePrev;

    private Houses _houses;
    private Rocks _rocks;
    private Trees _trees;
    private Bushes _bushes;
    private LightPoles _lightPoles;
    
    private float _matchTimeSeconds;
    private bool _hasLost;
    private bool _hasWon;
    
    private float _playerHealth = 100f;
    private float _playerMaxHealth = 100f;

    private List<TankEntry> _tankEntries = new();   
    
    private static readonly Random _random = new Random();
    
    private Debug _debug;
    private HUD _hud;
    private Menu _menu;
    
    // Audio
    private Song _gameplayMusic;
    private bool _gameplayMusicStarted = false;
    
    /// <summary>
    ///     Constructor del juego.
    /// </summary>
    public TGCGame()
    {
        
        // Maneja la configuración y la administración del dispositivo gráfico.
        _graphics = new GraphicsDeviceManager(this);

        //Le restamos un valor arbitrario para descartar para de titulo y barra de tareas
        _graphics.PreferredBackBufferWidth = GraphicsAdapter.DefaultAdapter.CurrentDisplayMode.Width - 100; 
        _graphics.PreferredBackBufferHeight = GraphicsAdapter.DefaultAdapter.CurrentDisplayMode.Height - 100;
        // Para que el juego sea pantalla completa se puede usar Graphics IsFullScreen.
        
        // Carpeta raíz donde va a estar toda la Media.
        Content.RootDirectory = "Content";
        
        // Hace que el mouse sea visible.
        IsMouseVisible = true;
    }

    /// <summary>
    ///     Se llama una sola vez, al principio cuando se ejecuta el ejemplo.
    ///     Escribir aquí el código de inicialización: el procesamiento que podemos pre calcular para nuestro juego.
    /// </summary>
    protected override void Initialize()
    {
        //DEBUG
        Gizmos = new Gizmos();
        
        BufferPool = new BufferPool();

        // Inicialización de cámaras
        _orbitCamera = new OrbitCamera(
            GraphicsDevice.Viewport.AspectRatio,
            Vector3.Zero,
            800f,
            5,
            3000
        );
        // Seteo la cámara inicial como la orbital
        _camera = _orbitCamera;

        _collisionHandler = new CollisionHandler();
        _bodyProperties = new CollidableProperty<TankBodyProperties>(); //BEPU
        _callbacks = new TankCallbacks() { Properties = _bodyProperties };
        _callbacks.SetCollisionHandler(_collisionHandler);

        _simulation = Simulation.Create(BufferPool, _callbacks,
            new PoseIntegratorCallbacks(new Vector3(0, -120, 0)), new SolveDescription(8, 1)); //TODO

        _tank = new Tank(new Vector3(1300, 0, 0), 0f, 0.1f);
        
        _tanks = [_tank];

        _debug = new Debug();
        _menu = new Menu();
        _hud = new HUD(GraphicsDevice);
        _enemyTanks = new List<Tank>();
        _enemyControllers = new List<TankController>();
        _lightPosition = new Vector3(1300, 8000, 0);
        _targetLightCamera = new TargetCamera(1f, _lightPosition, new Vector3(1300,0,0));
        _targetLightCamera.BuildProjection(1f, LightCameraNearPlaneDistance, LightCameraFarPlaneDistance,
            MathHelper.Pi / 5);
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
        _shadowEffect = Content.Load<Effect>(ContentFolderEffects + "ShadowMap");
        _effect.Parameters["lightPosition"].SetValue(_lightPosition);
        
        // Cargar shader específico para tanques
        _tankShader = Content.Load<Effect>(ContentFolderEffects + "TankShader");
        
        // Cargar shader específico para el World Border
        _worldBorderEffect = Content.Load<Effect>(ContentFolderEffects + "WorldBorderShader");
        
        // Cargar shader específico para árboles
        var treeShader = Content.Load<Effect>(ContentFolderEffects + "TreeShader");
        
        // heights
        var terrainHeigthmap = Content.Load<Texture2D>(ContentFolderTextures + "heightmaps/heightmap");
        // basic color
        var terrainColorMap = Content.Load<Texture2D>(ContentFolderTextures + "heightmaps/colormap");
        // blend texture 1
        var terrainGrass = Content.Load<Texture2D>(ContentFolderTextures + "grass");
        // blend texture 2
        var terrainGround = Content.Load<Texture2D>(ContentFolderTextures + "ground");

        _terrain = new Terrain(GraphicsDevice, 
            terrainHeigthmap, 
            terrainColorMap, 
            terrainGrass, 
            terrainGround, 
            _terrainEffect,
            _simulation,
            EscalaMapa,
            _camera.Position
            );
        
        // Registrar el handle del terreno en el CollisionHandler para excluirlo de sonidos
        CollisionHandler.TerrainHandle = _terrain.Handle;
        
        var tankT90 = Content.Load<Model>(ContentFolder3D + "t90/T90");
        var hullATexture = Content.Load<Texture2D>(TGCGame.ContentFolder3D + "t90/textures_mod/hullA");
        _tankEntries.Add(new TankEntry("T-90-A", tankT90, hullATexture, 0.002f, 0.5f, _tankShader));
        
        var hullBTexture = Content.Load<Texture2D>(TGCGame.ContentFolder3D + "t90/textures_mod/hullB");
        _tankEntries.Add(new TankEntry("T-90-B", tankT90, hullBTexture, 0.002f, 0.5f, _tankShader));
        
        var hullCTexture = Content.Load<Texture2D>(TGCGame.ContentFolder3D + "t90/textures_mod/hullC");
        _tankEntries.Add(new TankEntry("T-90-C", tankT90, hullCTexture, 0.002f, 0.5f, _tankShader));
         
        _tank.CargarModelo("t90/T90", _tankShader, Content, _simulation, BufferPool, GraphicsDevice, Gizmos, _bodyProperties, _terrain);

        // Construyo el diccionario BodyHandle → Tank
        var tankMap = new Dictionary<BodyHandle, Tank>();

        foreach (var tank in _tanks)
        {
            foreach (var handle in tank.BodyHandles)
            {
                tankMap[handle] = tank;
            }
        }

        // Se lo paso al handler
        CollisionHandler.HandleToTank = tankMap;

        _playerController = new TankController(_tank, 20, 200, 2, 100, 200f);

        _trees = new Trees(_terrain, _simulation);
        _houses = new Houses(_terrain, _simulation);
        _rocks = new Rocks(_terrain, _simulation);
        _bushes = new Bushes(_terrain, _simulation);
        _lightPoles =  new LightPoles(_terrain, _simulation);
        
        _houses.SetPlacementRules(5f,  false); // ≤ 5°, NO se inclinan
        _trees.SetPlacementRules(20f,  true);  // ≤ 20°, se inclinan
        _bushes.SetPlacementRules(25f, true);  // ≤ 25°, se inclinan
        _rocks.SetPlacementRules(null, true);  // sin restricción, se inclinan
        _lightPoles.SetPlacementRules(10f, true);

        // Generacion de posiciones de modelos

        var anchoMapa = (_terrain.HeightmapData.GetLength(0) - 1) * EscalaMapa; // Ancho terreno en mundo
        var largoMapa = (_terrain.HeightmapData.GetLength(1) - 1) * EscalaMapa; // Largo terreno en mundo
        
        _positionGenerator = new PositionGenerator(anchoMapa, largoMapa);
        var modelos = _trees.GetModelosConPorcentaje(0.50) // Arboles
            .Concat(_rocks.GetModelosConPorcentaje(0.30)) // Rocas
            .Concat(_houses.GetModelosConPorcentaje(0.05)) // Casas
            .Concat(_lightPoles.GetModelosConPorcentaje(0.15))
            .ToList();
        _positionGenerator.AgregarPosiciones(modelos);

        // Genero otros puntos para los arbustos
        var arbustos = _bushes.GetModelosConPorcentaje(1.0);
        _positionGenerator.AgregarPosiciones(arbustos, 450);

        _trees.CrearObjetos();
        _rocks.CrearObjetos();
        _houses.CrearObjetos();
        _bushes.CrearObjetos();
        _lightPoles.CrearObjetos();
        
        _trees.CargarModelos(treeShader, Content);
        _houses.CargarModelos(_effect, Content);
        _rocks.CargarModelos(_effect, Content);
        _bushes.CargarModelos(_effect, Content);
        _lightPoles.CargarModelos(_effect, Content);
        
        _worldBorder = new WorldBorder(GraphicsDevice, _worldBorderEffect, _simulation, anchoMapa, largoMapa);
        
        _debug.LoadContent(
            Content, 
            ContentFolderEffects, 
            ContentFolderSpriteFonts, 
            GraphicsDevice, 
            _tanks, 
            _orbitCamera,
            _simulation, 
            _terrain,
            Gizmos
        );
        
        _menu.LoadContent(Content, ContentFolderTextures, GraphicsDevice, ContentFolderSpriteFonts);
        _hud.LoadContent(Content, ContentFolderTextures, GraphicsDevice, ContentFolderSpriteFonts);
        //_font = Content.Load<SpriteFont>(ContentFolderSpriteFonts + "CascadiaCode/CascadiaCodePL");
        
        
        // Cargar música de gameplay (opcional - no falla si no existe)
        try
        {
            _gameplayMusic = Content.Load<Song>("Music/gameplay_music");
        }
        catch
        {
            // Música de gameplay no encontrada, continuar sin ella
        }
        
        _shadowMapRenderTarget = new RenderTarget2D(GraphicsDevice, ShadowmapSize, ShadowmapSize, false,
            SurfaceFormat.Single, DepthFormat.Depth24, 0, RenderTargetUsage.PlatformContents);

        _terrain.LightPosition = _lightPosition;
        _effect.Parameters["lightPosition"].SetValue(_lightPosition);
        _targetLightCamera.Position = _lightPosition;
        _targetLightCamera.BuildView();
        
        _imGuiRenderer = new ImGuiRenderer(this);
        _imGuiRenderer.RebuildFontAtlas();
        _boundingFrustum = new BoundingFrustum(_orbitCamera.View * _orbitCamera.Projection);
        base.LoadContent();
    }

    /// <summary>
    ///     Se llama en cada frame.
    ///     Se debe escribir toda la logica de computo del modelo, asi como tambien verificar entradas del usuario y reacciones
    ///     ante ellas.
    /// </summary>
    protected override void Update(GameTime gameTime)
    {
        var deltaTime = (float)gameTime.ElapsedGameTime.TotalSeconds;
        var keyboardState = Keyboard.GetState();
        var mouseState = Mouse.GetState();
        
        //Salgo del juego
        if (keyboardState.IsKeyDown(Keys.Escape))
        {
            Exit();
        }
        
        // ------------------------------
        //  MODO MENU
        // ------------------------------

        if (_state == GameState.MainMenu)
        {
            _menu.Update(keyboardState,_kbPrev, gameTime, this, _tankEntries);
            _kbPrev = keyboardState;
            _mousePrev = mouseState;
            return; // >>> NO actualizar lógica de juego mientras estás en el menú
        }
        
        // ------------------------------
        //  MODO JUEGO
        // ------------------------------
        
        //si se acabo el tiempo perdemos
        _matchTimeSeconds -= deltaTime;
        if (_matchTimeSeconds <= 0f && !_hasLost && !_hasWon)
        {
            _hasLost = true;
            _matchTimeSeconds = 5;
        }
        
        if (_hasLost || _hasWon)
        {
            if (_matchTimeSeconds <= 0)
            {
                _state = GameState.MainMenu;
                _hasLost = false;
                _hasWon = false;
                _tank.VolverAlCentro();
            }
        }
        else
        {
            
            _playerController.UpdateControls(keyboardState);
            _tank.UpdateAim(mouseState, _camera, GraphicsDevice.Viewport);
            _playerController.UpdateMovementAndAim(_simulation, _tank.AimDirectionWorld);
            
            // click izquierdo: dispara
            if (_tank.FireCooldown <= 0f 
                && mouseState.LeftButton == ButtonState.Pressed 
                && _mousePrev.LeftButton == ButtonState.Released)
            {
                var tipoProyectilActual = _tank.TipoProyectilActual;

                var (muzzle, dir) = _tank.GetMuzzle(); // offset local del cañón 
                var proj = new Projectile(_simulation, _effect, muzzle, dir, tipoProyectilActual, _tank);
                _projectiles.Add(proj);

                // Retroceso + freno breve
                _tank.TriggerRecoil(
                    dir,
                    _camera,
                    projectileMass: tipoProyectilActual.Mass, 
                    muzzleSpeed: tipoProyectilActual.Speed, 
                    intensity: 1f, 
                    withBrake: true);

                _tank.ResetCooldown();
            }
            
            for (int i = _enemyTanks.Count - 1; i >= 0; i--)
            {
                var enemyTank = _enemyTanks[i];
                if (enemyTank.IsDead)
                {
                    _enemyTanks.RemoveAt(i);
                    _enemyControllers.RemoveAt(i);
                    _enemyCount--;
                    continue;
                }
                
                var enemyController = _enemyControllers[i];
                enemyTank.UpdateEnemyTankAI(_tank.Position, enemyController);
                enemyTank.Update(gameTime);
            }

            if (_enemyCount == 0)
            {
                _hasWon = true;
                _matchTimeSeconds = 5;
            }
            
            //DEBUG
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
            
            if (Keyboard.GetState().IsKeyDown(Keys.L))
            {
                _lightPosition += new Vector3(0, 100, 0);
                Console.WriteLine(_lightPosition);
            }
        
            if (Keyboard.GetState().IsKeyDown(Keys.K))
            {
                _lightPosition -= new Vector3(0, 100, 0);
                Console.WriteLine(_lightPosition);
            }
            if (Keyboard.GetState().IsKeyDown(Keys.O))
            {
                _lightPosition += new Vector3(100, 0, 0);
                Console.WriteLine(_lightPosition);
            }
        
            if (Keyboard.GetState().IsKeyDown(Keys.P))
            {
                _lightPosition -= new Vector3(100, 0, 0);
                Console.WriteLine(_lightPosition);
            }
            if (Keyboard.GetState().IsKeyDown(Keys.U))
            {
                _lightPosition += new Vector3(0, 0, 100);
                Console.WriteLine(_lightPosition);
            }
        
            if (Keyboard.GetState().IsKeyDown(Keys.I))
            {
                _lightPosition -= new Vector3(0, 0, 100);
                Console.WriteLine(_lightPosition);
            }
            _targetLightCamera.Position = _lightPosition;
            _terrain.LightPosition = _lightPosition;
            
            var forward = Microsoft.Xna.Framework.Vector3.Normalize(_orbitCamera.FrontDirection);
            _targetLightCamera.TargetPosition = _tank.Position + forward * 1100;
            _targetLightCamera.BuildView();
            _terrain.EyePosition = _camera.Position;
            
            _boundingFrustum= new BoundingFrustum(_orbitCamera.View * _orbitCamera.Projection) ;
        }
        
        _tank?.Update(gameTime, keyboardState);

        // update de todos los proyectiles
        for (var i = _projectiles.Count - 1; i >= 0; --i)
        {
            _projectiles[i].Update(deltaTime);
            if (_projectiles[i].IsDead) _projectiles.RemoveAt(i);
        }
        
        // Actualizar World Border
        _worldBorder.Update(_tank.Position.ToNumerics());
        
        // Actualizar simulación física
        if (_simulation != null && deltaTime is > 0.0f and < 0.1f) // Máximo 100ms por frame
        {
            _simulation.Timestep(deltaTime);
            _collisionHandler.HandleCollisions();
            
            _tank?.SyncFromPhysics();
            _tank?.ApplyRecoilAndBrake(deltaTime, _simulation);
        }
        
        _debug.Update(keyboardState, _kbPrev, deltaTime, _orbitCamera);

        // Actualizar cámara para seguir al tanque
        if (_tank != null)
        {
            // Usar la posición y rotación del tanque
            var alturaTerreno = _terrain.GetHeightAtPosition(_tank.Position.X, _tank.Position.Z); 
            _orbitCamera.SetTarget(new Vector3(_tank.Position.X, alturaTerreno + 50f, _tank.Position.Z));

            // Actualizar la cámara (maneja el input del mouse)
            _camera.Update(gameTime);
            _orbitCamera.ConstrainAboveTerrain(_terrain, clearance: 50f, samples: 16);
            _orbitCamera.ConstrainInsideWorldBorder(_worldBorder);
            
            //debug
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
        DrawShadows();
        GraphicsDevice.SetRenderTarget(null);
        GraphicsDevice.Clear(ClearOptions.Target | ClearOptions.DepthBuffer, Color.CornflowerBlue, 1f, 0);
        
        if (_state == GameState.MainMenu)
        {
            _menu.Draw(_tankEntries);
            return; // no dibujamos el juego
        }

        // Estados por defecto para 3D
        GraphicsDevice.BlendState = BlendState.Opaque;
        GraphicsDevice.DepthStencilState = DepthStencilState.Default;
        GraphicsDevice.RasterizerState = RasterizerState.CullCounterClockwise;
        GraphicsDevice.SamplerStates[0] = SamplerState.LinearWrap;

        // Verificar que el efecto y el terreno no sean nulos antes de dibujar
        if (_terrainEffect == null || _terrain == null)
            return;
        
        _effect.Parameters["View"].SetValue(_camera.View);
        _effect.Parameters["Projection"].SetValue(_camera.Projection);
        _effect.Parameters["eyePosition"].SetValue(_camera.Position);
        
        //_terrain.Draw(Matrix.Identity, _camera.View, _camera.Projection);
        DibujarTerreno();    
        
        
        _tank.Draw(_camera);
        if (_state != GameState.MainMenu)
        {
            foreach (var enemyTank in _enemyTanks)
            {
                enemyTank.Draw(_camera);
            }
        }

        if (dibujar)
        {
            _houses.Draw(_effect, _boundingFrustum, Gizmos, "Casa");
            _rocks.Draw(_effect, _boundingFrustum, Gizmos, "Piedra");
            _bushes.Draw(_effect,_boundingFrustum, Gizmos, "Arbusto");
            _lightPoles.Draw(_effect, _boundingFrustum, Gizmos, "Poste de luz");
        }
       
        foreach (var projectile in _projectiles)
            projectile.Draw(_effect, _camera.View, _camera.Projection);
        
        //GraphicsDevice.BlendState = BlendState.AlphaBlend;
        //GraphicsDevice.DepthStencilState = DepthStencilState.DepthRead;
        
        _trees.Draw(_effect, _boundingFrustum, Gizmos, "Arbol");

        GraphicsDevice.BlendState = BlendState.Opaque;
        GraphicsDevice.DepthStencilState = DepthStencilState.Default;

        _worldBorder.Draw(_camera.View, _camera.Projection);

        _debug.Draw(_camera, _orbitCamera, Gizmos, _shadowMapRenderTarget, _imGuiRenderer, gameTime);
        
        _hud.Begin();
        if (!_hasLost && !_hasWon)
        {
            foreach (var enemyTank in _enemyTanks)
            {
                var healthBarPosition3D = enemyTank.Position + Microsoft.Xna.Framework.Vector3.Up * 50f;
                var projectedPosition = GraphicsDevice.Viewport.Project(healthBarPosition3D, _camera.Projection, _camera.View, Matrix.Identity);
                if (projectedPosition.Z < 1) //Si esta visible
                {
                    var healthPercentage = enemyTank.Vida / enemyTank.MaxVida;
                    _hud.DrawHealthBar(new Vector2(projectedPosition.X, projectedPosition.Y), healthPercentage, 100, 10);
                }
            }
            _hud.Draw(_matchTimeSeconds, _tank.FireCooldown, _tank.TipoProyectilActual.MaxCooldown, _tank.TipoProyectilActual, _playerHealth, _playerMaxHealth, _enemyCount, gameTime);
        }
        
        if (_hasLost)
        {
            _hud.DrawMensaje("PERDISTE", Color.Red);
        }

        if (_hasWon)
        {
            _hud.DrawMensaje("GANASTE", Color.Green);
        }
        _hud.End();
    }

    public void StartGame(TimeSpan tiempoPartida, int cantidadEnemigos, int indiceSeleccionado)
    {
        _matchTimeSeconds = (float)tiempoPartida.TotalSeconds;
        SpawnearTanks(cantidadEnemigos);
        _enemyCount=cantidadEnemigos;
        
        // Iniciar música de gameplay
        StartGameplayMusic();
        _tank.Texture = _tankEntries[indiceSeleccionado].Texture;
        _state = GameState.Playing;
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
    
    public void SpawnearTanks(int cantTanks)
    {
        for (int i = 0; i < cantTanks; i++)
        {
            
            var random = new Random();
            var radio = (float)(1000f + random.NextDouble() * 1000f); 
            var angulo = (float)(random.NextDouble() * Math.PI * 2); 
            
            var offsetX = (float)Math.Cos(angulo) * radio;
            var offsetZ = (float)Math.Sin(angulo) * radio;
            var spawnPosition = new Vector3(offsetX + _tank.Position.X, 0, offsetZ + _tank.Position.Z);
            
            
            var enemyTank = new Tank(spawnPosition, 0f, 0.1f);
            
            
            enemyTank.CargarModelo("t90/T90", _tankShader, Content, _simulation, BufferPool, GraphicsDevice, Gizmos,
                _bodyProperties, _terrain);
            int index = _random.Next(_tankEntries.Count); // índice aleatorio entre 0 y Count-1
            enemyTank.Texture = _tankEntries[index].Texture;
            var enemyController = new TankController(enemyTank, 20, 200, 2, 100, 200f);
            _enemyTanks.Add(enemyTank);
            _enemyControllers.Add(enemyController);
        }
        
        _tanks.AddRange(_enemyTanks);
        
        _debug.actualizarTanks(_tanks);
        
        var tankMap = new Dictionary<BodyHandle, Tank>();

        foreach (var tank in _tanks)
        {
            foreach (var handle in tank.BodyHandles)
            {
                tankMap[handle] = tank;
            }
        }
        
        CollisionHandler.HandleToTank = tankMap;
    }
    
    /// <summary>
    /// Inicia la música de gameplay
    /// </summary>
    private void StartGameplayMusic()
    {
        if (_gameplayMusic != null && !_gameplayMusicStarted)
        {
            try
            {
                MediaPlayer.IsRepeating = true;
                MediaPlayer.Volume = 0.33f; // 33% del volumen (música de fondo)
                MediaPlayer.Play(_gameplayMusic);
                _gameplayMusicStarted = true;
            }
            catch
            {
                // Si hay error al reproducir, continuar sin música
            }
        }
    }
    
    /// <summary>
    /// Detiene la música de gameplay
    /// </summary>
    private void StopGameplayMusic()
    {
        try
        {
            if (MediaPlayer.State == MediaState.Playing)
            {
                MediaPlayer.Stop();
            }
            _gameplayMusicStarted = false;
        }
        catch
        {
            // Ignorar errores
        }
    }
    
    private void DrawShadows()
        {
            GraphicsDevice.BlendState = BlendState.Opaque;
            GraphicsDevice.DepthStencilState = DepthStencilState.Default;
            // Set the render target as our shadow map, we are drawing the depth into this texture
            GraphicsDevice.SetRenderTarget(_shadowMapRenderTarget);
            GraphicsDevice.Clear(ClearOptions.Target | ClearOptions.DepthBuffer, Color.Black, 1f, 0);

            _shadowEffect.CurrentTechnique = _shadowEffect.Techniques["DepthPass"];

            List<ModelInstances> allInstances = new List<ModelInstances>();
            
            allInstances.AddRange(_rocks.Models);
            allInstances.AddRange(_bushes.Models);
            allInstances.AddRange(_houses.Models);
            allInstances.AddRange(_lightPoles.Models);
            allInstances.AddRange(_trees.Models);
            
            foreach (var instance in allInstances)
            {
                var model =  instance.Model;
                var modelMeshesBaseTransforms = new Matrix[model.Bones.Count];
                model.CopyAbsoluteBoneTransformsTo(modelMeshesBaseTransforms);
                var worlds = instance._worlds;
                
                foreach (var world in worlds)
                {
                    if (!instance.EsVisible(world, _boundingFrustum))
                    {
                        continue;
                    }
                    foreach (var modelMesh in model.Meshes)
                    {
                        foreach (var part in modelMesh.MeshParts)
                            part.Effect = _shadowEffect;
                        
                        var worldMatrix = modelMeshesBaseTransforms[modelMesh.ParentBone.Index] * world;
                        // WorldViewProjection is used to transform from model space to clip space
                        _shadowEffect.Parameters["WorldViewProjection"].SetValue(worldMatrix * _targetLightCamera.View * _targetLightCamera.Projection);

                        // Once we set these matrices we draw
                        modelMesh.Draw();
                    }
                }
            }
            
            DrawTankShadow(_tank);
            foreach (var tank in _enemyTanks)
            {
                DrawTankShadow(tank);
            }
        }

    private void DrawTankShadow(Tank tank)
    {
        var modelMeshesBaseTransforms = new Matrix[tank.Model.Bones.Count];
        tank.Model.CopyAbsoluteBoneTransformsTo(modelMeshesBaseTransforms);
        foreach (var modelMesh in tank.Model.Meshes)
        {
            foreach (var part in modelMesh.MeshParts)
                part.Effect = _shadowEffect;

            // We set the main matrices for each mesh to draw
            var worldMatrix = modelMeshesBaseTransforms[modelMesh.ParentBone.Index] * tank._world;

            // WorldViewProjection is used to transform from model space to clip space
            _shadowEffect.Parameters["WorldViewProjection"].SetValue(worldMatrix * _targetLightCamera.View * _targetLightCamera.Projection);

            // Once we set these matrices we draw
            modelMesh.Draw();
        }
    }

    private void DibujarTerreno()
    {
        _terrainEffect.CurrentTechnique = _terrainEffect.Techniques["DrawShadowedPCF"];
        _terrainEffect.Parameters["shadowMap"].SetValue(_shadowMapRenderTarget);
        _terrainEffect.Parameters["lightPosition"].SetValue(_lightPosition);
        _terrainEffect.Parameters["shadowMapSize"].SetValue(Vector2.One * ShadowmapSize);
        _terrainEffect.Parameters["LightViewProjection"].SetValue(_targetLightCamera.View * _targetLightCamera.Projection);
        
        _terrain.Draw(Matrix.Identity, _camera.View, _camera.Projection);
    }
}