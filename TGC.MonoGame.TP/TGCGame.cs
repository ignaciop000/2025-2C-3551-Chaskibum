// FINALMENTE [TODOS]
// - Separar los .cs en carpetas
// - Emprolijar código (principalmente físicas, pero no estaria mal diseñar mejor algunas cosas)
// - Emprolijar shaders (borrarles cosas innecesarias o unificarlos si se puede)

// ERRORES A SOLUCIONAR [TODOS]
// - Error de BEPU de que un valor es nan o infinito

// OPCIONALES [EL QUE QUIERA]
// - Hacer volumen configurable
// - Que los cambios en el menú sean no solo skin, si no tmb tamaño, vida, daño base, velocidad
// - Crear algún sistema de partículas que muestren humo, fuego, chispas, etc.
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
    private Gizmos Gizmos { get; set; }
    private ImGuiRenderer _imGuiRenderer;
    private BoundingFrustum _boundingFrustum;
    private BoundingFrustum _lightBoundingFrustum;

    public const string ContentFolder3D = "Models/";
    private const string ContentFolderEffects = "Effects/";
    public const string ContentFolderMusic = "Music/";
    public const string ContentFolderSounds = "Sounds/";
    private const string ContentFolderSpriteFonts = "SpriteFonts/";
    private const string ContentFolderTextures = "Textures/";

    private enum GameState
    {
        MainMenu,
        Playing
    }

    private GameState _state = GameState.MainMenu;

    private const float EscalaMapa = 30;
    private readonly GraphicsDeviceManager _graphics;
    private RenderTarget2D _shadowMapRenderTarget;
    
    // Post-procesado de niebla
    private RenderTarget2D _sceneRenderTarget;
    private Effect _fogEffect;
    private VertexBuffer _fullScreenQuad;

    private TargetCamera _targetLightCamera;
    private const int ShadowmapSize = 4096;

    private Camera _camera; // Cámara activa
    private OrbitCamera _orbitCamera; // Cámara que sigue al tanque
    private const float LightCameraFarPlaneDistance = 7500f;
    private const float LightCameraNearPlaneDistance = 1f;
    private float _elapsedTime;
    private Point _screenCenter;

    private Effect _terrainEffect; //Shader Terreno
    private Effect _effect; //Shader Basico

    private Effect _shadowEffect;
    private Effect _worldBorderEffect; //Shader WorldBorder
    private Vector3 _lightPosition;
    private Simulation _simulation;
    private CollidableProperty<TankBodyProperties> _bodyProperties; // Propiedades por colisionable (tanques)
    private TankCallbacks _callbacks; // Callbacks de BEPU para fuerzas/colisiones
    private CollisionHandler _collisionHandler; // Maneja eventos de colisión de juego

    private BufferPool BufferPool { get; set; } // Pool de buffers BEPU para performance

    private PositionGenerator _positionGenerator;
    private Terrain _terrain;
    private WorldBorder _worldBorder;

    private KeyboardState _kbPrev;

    private TankController _playerController;
    private int _enemyCount;

    private bool _slowmotion = false;
    private bool _stopTime = false;
    private bool _apuntar = true;

    private PlayerTank _tank;
    private List<EnemyTank> _enemyTanks;
    private List<TankController> _enemyControllers;
    private List<Tank> _tanks;
    List<ModelInstances> _allInstances;

    private Effect _tankShader;
    private Effect _pastoShader;

    // Proyectiles
    private readonly List<Projectile> _projectiles = [];
    private MouseState _mousePrev;

    private Houses _houses;
    private Rocks _rocks;
    private Trees _trees;
    private Bushes _bushes;
    private LightPoles _lightPoles;

    private Pasto _pasto;

    private float _matchTimeSeconds;
    private bool _hasLost;
    private bool _hasWon;
    private bool _usarNormalMapping;
    private bool _dibujarSombras;

    private float _playerHealth = 100f;
    private float _playerMaxHealth = 100f;

    private Vector3 _offset;

    private List<TankEntry> _tankEntries = new();

    private static readonly Random Random = new Random();

    private Debug _debug;
    private HUD _hud;
    private Menu _menu;

    // Estado de la ayuda
    private bool _showHelp = false;

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
        //_graphics.IsFullScreen = true;
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
        _usarNormalMapping = true;
        _dibujarSombras = true;
        //DEBUG
        Gizmos = new Gizmos();

        // Inicialización de cámaras
        _orbitCamera = new OrbitCamera(
            GraphicsDevice.Viewport.AspectRatio,
            Vector3.Zero,
            300f,
            5,
            3000
        );
        // Seteo la cámara inicial como la orbital
        _camera = _orbitCamera;
        _screenCenter = new Point(
            GraphicsDevice.Viewport.Width / 2,
            GraphicsDevice.Viewport.Height / 2);
        
        _offset = new Vector3(2700f, 4600f, -2000f);
        
        _collisionHandler = new CollisionHandler();
        
        //BEPU
        BufferPool = new BufferPool();
        _bodyProperties = new CollidableProperty<TankBodyProperties>();
        _callbacks = new TankCallbacks() { Properties = _bodyProperties };
        _callbacks.SetCollisionHandler(_collisionHandler);

        _simulation = Simulation.Create(BufferPool, _callbacks,
            new PoseIntegratorCallbacks(new Vector3(0, -120, 0)), new SolveDescription(8, 1));
        
        _tank = new PlayerTank(new Vector3(0, 0, 0));
        _tanks = [_tank];
        
        _debug = new Debug();
        _menu = new Menu();
        _hud = new HUD(GraphicsDevice);
        _enemyTanks = new List<EnemyTank>();
        _enemyControllers = new List<TankController>();
        _lightPosition = new Vector3(1300, 8000, 0);
        _targetLightCamera = new TargetCamera(1f, _lightPosition, new Vector3(1300, 0, 0));
        _targetLightCamera.BuildProjection(1f, LightCameraNearPlaneDistance, LightCameraFarPlaneDistance,
            MathHelper.Pi / 4.5f);
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
        _effect.Parameters["lightPosition"]?.SetValue(_lightPosition);
        _effect.Parameters["ambientColor"]?.SetValue(new Vector3(1, 1, 1));
        _effect.Parameters["Ka"]?.SetValue(0.1f);
        _effect.Parameters["diffuseColor"]?.SetValue(new Vector3(1, 1, 1));
        _effect.Parameters["Kd"]?.SetValue(0.8f);
        _effect.Parameters["specularColor"]?.SetValue(new Vector3(1, 1, 1));
        _effect.Parameters["Ks"]?.SetValue(0.1f);
        _effect.Parameters["shininess"]?.SetValue(4f);

        _pastoShader = Content.Load<Effect>(ContentFolderEffects + "Pasto");
        // Cargar shader específico para tanques
        _tankShader = Content.Load<Effect>(ContentFolderEffects + "TankShader");
        _tankShader.Parameters["Ka"]?.SetValue(1f);
        
        // Cargar efecto de niebla para post-procesado
        _fogEffect = Content.Load<Effect>(ContentFolderEffects + "Fog");

        // Cargar shader específico para el World Border
        _worldBorderEffect = Content.Load<Effect>(ContentFolderEffects + "WorldBorderShader");

        // Cargar shader específico para árboles
        var treeShader = Content.Load<Effect>(ContentFolderEffects + "TreeShader");

        // heights
        var terrainHeigthmap = Content.Load<Texture2D>(ContentFolderTextures + "heightmaps/heightmap");
        // basic color
        var terrainColorMap = Content.Load<Texture2D>(ContentFolderTextures + "heightmaps/colormap");
        var spawnMap = Content.Load<Texture2D>(ContentFolderTextures + "heightmaps/spawnmap");

        // blend texture 1
        var terrainGrass = Content.Load<Texture2D>(ContentFolderTextures + "grass");
        // blend texture 2
        var terrainGround = Content.Load<Texture2D>(ContentFolderTextures + "ground");
        //normal map piso
        var normalMap = Content.Load<Texture2D>(ContentFolderTextures + "normal");

        var normalMapRock = Content.Load<Texture2D>(ContentFolder3D + "rocks/Textures/Rock_Normal");
        var normalMapHouse = Content.Load<Texture2D>(ContentFolder3D + "house/city_house_2_Nor");
        var normalMapTree2Leaves =
            Content.Load<Texture2D>(ContentFolder3D + "tree2/TexturesCom_Branches0018_1_alphamasked_Snor");
        var normalMapTree2Bark =
            Content.Load<Texture2D>(ContentFolder3D + "tree2/tileable_tree_bark_texture_by_ftourini-d3l69hznor");
        var normalMapTreeLeaves = Content.Load<Texture2D>(ContentFolder3D + "tree/Tree.fbm/DB2X2_L01_Nor");

        _pasto = new Pasto(_simulation, GraphicsDevice);
        _pasto.CargarModelos(_pastoShader, Content);
        _pasto.Models[0]._effect = _pastoShader;

        _positionGenerator = new PositionGenerator();
        _terrain = new Terrain(GraphicsDevice,
            terrainHeigthmap,
            terrainColorMap,
            terrainGrass,
            terrainGround,
            normalMap,
            _terrainEffect,
            _simulation,
            EscalaMapa,
            _camera.Position,
            _positionGenerator,
            spawnMap,
            _pasto
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

        _tank.CargarModelo("t90/T90", _tankShader, Content, _simulation, BufferPool, GraphicsDevice, Gizmos,
            _bodyProperties, _terrain);

        _playerController = new TankController(_tank, 20, 200, 2, 100, 200f);

        _trees = new Trees(_simulation);
        _houses = new Houses(_simulation);
        _rocks = new Rocks(_simulation);
        _bushes = new Bushes(_simulation);
        _lightPoles = new LightPoles(_simulation);


        _houses.SetPlacementRules(5f, false); // ≤ 5°, NO se inclinan
        _trees.SetPlacementRules(20f, true); // ≤ 20°, se inclinan
        _bushes.SetPlacementRules(25f, true); // ≤ 25°, se inclinan
        _rocks.SetPlacementRules(null, true); // sin restricción, se inclinan
        _lightPoles.SetPlacementRules(10f, true);
        _pasto.SetPlacementRules(30, true);

        // Generacion de posiciones de modelos


        var colorMap = _terrain.LoadColorMap(spawnMap);

        var modelos = _trees.GetModelosConPorcentaje(0.50) // Arboles
            .Concat(_rocks.GetModelosConPorcentaje(0.30)) // Rocas
            .Concat(_houses.GetModelosConPorcentaje(0.05)) // Casas
            .Concat(_lightPoles.GetModelosConPorcentaje(0.15))
            .ToList();

        _positionGenerator.GenerarPosicionesReservadas();


        _positionGenerator.AgregarPosiciones(modelos, colorMap, EscalaMapa, 450);

        // Genero otros puntos para los arbustos
        var arbustos = _bushes.GetModelosConPorcentaje(1.0);
        _positionGenerator.AgregarPosiciones(arbustos, colorMap, EscalaMapa, 450);

        _trees.CrearObjetos(normalMapTree2Leaves, normalMapTree2Bark, normalMapTreeLeaves, _terrain);
        _rocks.CrearObjetos(normalMapRock, _terrain);
        _houses.CrearObjetos(normalMapHouse, _terrain);
        _bushes.CrearObjetos(_terrain);
        _lightPoles.CrearObjetos(_terrain);
        _pasto.CrearObjetos(_terrain);

        _trees.CargarModelos(treeShader, Content);
        _houses.CargarModelos(_effect, Content);
        _rocks.CargarModelos(_effect, Content);
        _bushes.CargarModelos(_effect, Content);
        _lightPoles.CargarModelos(_effect, Content);

        var anchoMapa = (_terrain.HeightmapData.GetLength(0) - 1) * EscalaMapa; // Ancho terreno en mundo
        var largoMapa = (_terrain.HeightmapData.GetLength(1) - 1) * EscalaMapa; // Largo terreno en mundo

        _worldBorder = new WorldBorder(GraphicsDevice, _worldBorderEffect, _simulation, anchoMapa, largoMapa);

        _debug.LoadContent(
            Content,
            ContentFolderEffects,
            ContentFolderSpriteFonts,
            GraphicsDevice,
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

        // Crear render target para la escena (con profundidad para niebla)
        _sceneRenderTarget = new RenderTarget2D(GraphicsDevice, 
            GraphicsDevice.PresentationParameters.BackBufferWidth,
            GraphicsDevice.PresentationParameters.BackBufferHeight,
            false,
            SurfaceFormat.Color,
            DepthFormat.Depth24);

        // Crear quad de pantalla completa para post-procesado
        var vertices = new[]
        {
            new VertexPositionTexture(new Microsoft.Xna.Framework.Vector3(-1, 1, 0), new Vector2(0, 0)),
            new VertexPositionTexture(new Microsoft.Xna.Framework.Vector3(1, 1, 0), new Vector2(1, 0)),
            new VertexPositionTexture(new Microsoft.Xna.Framework.Vector3(-1, -1, 0), new Vector2(0, 1)),
            new VertexPositionTexture(new Microsoft.Xna.Framework.Vector3(1, -1, 0), new Vector2(1, 1))
        };
        _fullScreenQuad = new VertexBuffer(GraphicsDevice, typeof(VertexPositionTexture), 4, BufferUsage.WriteOnly);
        _fullScreenQuad.SetData(vertices);

        _terrain.LightPosition = _lightPosition;
        _effect.Parameters["lightPosition"]?.SetValue(_lightPosition);
        _targetLightCamera.Position = _lightPosition;
        _targetLightCamera.BuildView();

        _imGuiRenderer = new ImGuiRenderer(this);
        _imGuiRenderer.RebuildFontAtlas();
        _boundingFrustum = new BoundingFrustum(_orbitCamera.View * _orbitCamera.Projection);
        _lightBoundingFrustum = new BoundingFrustum(_targetLightCamera.View * _targetLightCamera.Projection);

        _allInstances = new List<ModelInstances>();

        _allInstances.AddRange(_rocks.Models);
        //_allInstances.AddRange(_bushes.Models);
        _allInstances.AddRange(_houses.Models);
        _allInstances.AddRange(_lightPoles.Models);

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
        if (_slowmotion)
        {
            deltaTime /= 20;
        }

        if (_stopTime)
            deltaTime = 0;
        var keyboardState = Keyboard.GetState();
        var mouseState = Mouse.GetState();

        //Salgo del juego
        if (keyboardState.IsKeyDown(Keys.Escape))
        {
            Exit();
        }
        if (keyboardState.IsKeyUp(Keys.R) && _kbPrev.IsKeyDown(Keys.R))
        {
            _slowmotion = !_slowmotion;
        }
        if (keyboardState.IsKeyUp(Keys.T) && _kbPrev.IsKeyDown(Keys.T))
        {
            _stopTime = !_stopTime;
        }
        if (keyboardState.IsKeyUp(Keys.Y) && _kbPrev.IsKeyDown(Keys.Y))
        {
            _camera.Position = _targetLightCamera.Position;
        }
        // ------------------------------
        //  MODO MENU
        // ------------------------------

        if (_state == GameState.MainMenu)
        {
            _tank._effect.CurrentTechnique = _tank._effect.Techniques["MenuDrawing"];
            _menu.Update(keyboardState, _kbPrev, gameTime, this, _tankEntries);
            _kbPrev = keyboardState;
            _mousePrev = mouseState;
            return; // >>> NO actualizar lógica de juego mientras estás en el menú
        }

        // ------------------------------
        //  MODO JUEGO
        // ------------------------------

        //si se acabo el tiempo perdemos
        _matchTimeSeconds -= deltaTime;
        if (!_hasLost && !_hasWon && (_matchTimeSeconds <= 0f || _tank.IsDead))
        {
            _hasLost = true;
            _matchTimeSeconds = 5;
        }

        if (_hasLost || _hasWon)
        {
            if (_matchTimeSeconds <= 0)
            {
                _state = GameState.MainMenu;
                _tank._effect.CurrentTechnique = _tank._effect.Techniques["MenuDrawing"];
                _hasLost = false;
                _hasWon = false;
                foreach (var tank in _enemyTanks)
                {
                    tank.Kill();
                }

                _enemyTanks.Clear();
                _enemyControllers.Clear();
            }
        }
        else
        {
            if (mouseState.RightButton == ButtonState.Pressed || _camera != _orbitCamera)
            {
                IsMouseVisible = false;
            }
            else
            {
                IsMouseVisible = true;
            }

            if (_camera != _orbitCamera && mouseState.RightButton == ButtonState.Pressed)
            {
                _orbitCamera.Update(gameTime, _screenCenter);
            }

            if (keyboardState.IsKeyUp(Keys.N) && _kbPrev.IsKeyDown(Keys.N))
            {
                _usarNormalMapping = !_usarNormalMapping;
            }

            if (keyboardState.IsKeyUp(Keys.M) && _kbPrev.IsKeyDown(Keys.M))
            {
                _dibujarSombras = !_dibujarSombras;
            }

            if (keyboardState.IsKeyUp(Keys.H) && _kbPrev.IsKeyDown(Keys.H))
            {
                _showHelp = !_showHelp;
            }

            _playerController.UpdateControls(keyboardState);
            if(_apuntar)
                _tank.UpdateAim(mouseState, _camera, GraphicsDevice.Viewport);
            _playerController.UpdateMovementAndAim(_simulation, _tank.AimDirectionWorld);

            // click izquierdo: dispara
            if (_tank.FireCooldown <= 0f
                && mouseState.LeftButton == ButtonState.Pressed
                && _mousePrev.LeftButton == ButtonState.Released)
            {
                _tank.Shoot(_simulation, _projectiles, _effect, _bodyProperties);

                var tipoProyectilActual = _tank.TipoProyectilActual;
                var amplitude = 0.001f * tipoProyectilActual.Mass * tipoProyectilActual.Speed;
                var rotational = amplitude * 0.06f;
                _camera.StartShake(amplitude, 0.12f, rotational);
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
                enemyTank.UpdateAI(_tank.Position, enemyController, _simulation, _projectiles, _effect, _bodyProperties);
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
                    _camera = new FreeCamera(GraphicsDevice.Viewport.AspectRatio, _orbitCamera.Position,
                        _orbitCamera.FrontDirection, size);
                }
                else
                {
                    _camera = _orbitCamera;
                }
            }
            
            if (keyboardState.IsKeyDown(Keys.F5) && !_kbPrev.IsKeyDown(Keys.F5))
            {
                _tank.ModoGod = !_tank.ModoGod;
            }

            if (Keyboard.GetState().IsKeyDown(Keys.L))
            {
                _offset += new Vector3(0, 100, 0);
                Console.WriteLine(_offset);
            }

            if (Keyboard.GetState().IsKeyDown(Keys.K))
            {
                _offset -= new Vector3(0, 100, 0);
                Console.WriteLine(_offset);
            }
/*
            if (Keyboard.GetState().IsKeyDown(Keys.O))
            {
                _offset += new Vector3(100, 0, 0);
                Console.WriteLine(_offset);
            }

            if (Keyboard.GetState().IsKeyDown(Keys.P))
            {
                _offset -= new Vector3(100, 0, 0);
                Console.WriteLine(_offset);
            }
*/
            if (keyboardState.IsKeyUp(Keys.P) && _kbPrev.IsKeyDown(Keys.P))
            {
                _state = GameState.MainMenu;
            }
            if (Keyboard.GetState().IsKeyDown(Keys.U))
            {
                _offset += new Vector3(0, 0, 100);
                Console.WriteLine(_offset);
            }

            if (Keyboard.GetState().IsKeyDown(Keys.I))
            {
                _offset -= new Vector3(0, 0, 100);
                Console.WriteLine(_offset);
            }

            var forward = Microsoft.Xna.Framework.Vector3.Normalize(_orbitCamera.FrontDirection);
            var targetPos = _orbitCamera.Position + forward * 2000;
            var nuevaPos = _tank.Position + _offset;
            _lightPosition = new Vector3(nuevaPos.X, nuevaPos.Y, nuevaPos.Z);
            _targetLightCamera.Position = nuevaPos;
            _terrain.LightPosition = nuevaPos;


            _targetLightCamera.TargetPosition = _orbitCamera.Position + forward * 2000;
            _targetLightCamera.BuildView();
            _terrain.EyePosition = _camera.Position;

            _boundingFrustum = new BoundingFrustum(_orbitCamera.View * _orbitCamera.Projection);
            _lightBoundingFrustum = new BoundingFrustum(_targetLightCamera.View * _targetLightCamera.Projection);
            _elapsedTime += (float)gameTime.ElapsedGameTime.TotalSeconds;

            // Si pasó 1 segundo
            /* if (_elapsedTime >= 1.0)
             {
                 Console.WriteLine("offset: " + _offset);
                 _elapsedTime = 0;
             }*/
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
            foreach (var tank in _enemyTanks)
            {
                tank.SyncFromPhysics();
                tank.ApplyRecoilAndBrake(deltaTime, _simulation);
            }
        }

        _debug.Update(keyboardState, _kbPrev, deltaTime, _orbitCamera);
        _debug.actualizarTanks(_tanks);
        _debug.actualizarProyectiles(_projectiles);

        // Actualizar cámara para seguir al tanque
        if (_tank != null)
        {
            // Usar la posición y rotación del tanque
            _orbitCamera.SetTarget(new Vector3(_tank.Position.X, _tank.Position.Y + 25, _tank.Position.Z));

            // Actualizar la cámara (maneja el input del mouse)
            _camera.Update(gameTime, _screenCenter);
            _orbitCamera.ConstrainAboveTerrain(_terrain, clearance: 30f, samples: 16);
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
        if (_dibujarSombras)
            DrawShadows();
        GraphicsDevice.SetRenderTarget(null);
        GraphicsDevice.Clear(ClearOptions.Target | ClearOptions.DepthBuffer, new Color(new Vector3(0.5f, 0.6f, 0.7f)), 1f, 0);

        if (_state == GameState.MainMenu)
        {
            _tank.Audio.StopAll();
            foreach (var enemyTank in _enemyTanks)
            {
                enemyTank.Audio.StopAll();
            }
            _menu.Draw(_tankEntries, _tank.treadmillsTexture);
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

        _effect.Parameters["View"]?.SetValue(_camera.View);
        _effect.Parameters["Projection"]?.SetValue(_camera.Projection);
        _effect.Parameters["eyePosition"]?.SetValue(_camera.Position);

        _debug.Draw(_camera, _orbitCamera, _targetLightCamera, Gizmos, _shadowMapRenderTarget, _imGuiRenderer, gameTime,
            _terrain);

        _tankShader.Parameters["lightPosition"].SetValue(_lightPosition);
        _tankShader.Parameters["eyePosition"].SetValue(_camera.Position);
        if (_state != GameState.MainMenu)
        {
            DibujarTerreno();
            DibujarElementos();
            DibujarTanques(); // Acá se dibuja el outline también
            DibujarPasto();
            DibujarProyectiles();
            _worldBorder.Draw(_camera.View, _camera.Projection);
            
            _imGuiRenderer.BeforeLayout(gameTime);

            ImGui.SetNextWindowPos(new System.Numerics.Vector2(20, 60), ImGuiCond.Always);
            ImGui.SetNextWindowSize(new System.Numerics.Vector2(300, 60), ImGuiCond.Always);
            ImGui.Begin("Performance");
            ImGui.TextWrapped(
                $"Application average {1000f / ImGui.GetIO().Framerate:F3} ms/frame ({ImGui.GetIO().Framerate:F1} FPS)");
            ImGui.End();

            _imGuiRenderer.AfterLayout();

            if (_state != GameState.MainMenu && !_hasLost && !_hasWon)
                _debug.Draw(_camera, _orbitCamera, _targetLightCamera, Gizmos, _shadowMapRenderTarget, _imGuiRenderer,
                    gameTime, _terrain);

            _hud.Begin();
            if (!_hasLost && !_hasWon)
            {
                foreach (var enemyTank in _enemyTanks)
                {
                    var healthBarPosition3D = enemyTank.Position + Microsoft.Xna.Framework.Vector3.Up * 50f;
                    var projectedPosition = GraphicsDevice.Viewport.Project(healthBarPosition3D, _camera.Projection,
                        _camera.View, Matrix.Identity);
                    if (projectedPosition.Z < 1) //Si esta visible
                    {
                        var healthPercentage = enemyTank.Vida / Tank.VidaMax;
                        _hud.DrawHealthBar(new Vector2(projectedPosition.X, projectedPosition.Y), healthPercentage, 100,
                            10);
                    }
                }
                _hud.Draw(_matchTimeSeconds, _tank.FireCooldown, _tank.TipoProyectilActual.MaxCooldown, 
                    _tank.TipoProyectilActual, _tank.Vida, Tank.VidaMax, _enemyCount, gameTime);
            }

            if (_hasLost)
            {
                _hud.DrawMensaje("PERDISTE", Color.Red);
            }

            if (_hasWon)
            {
                _hud.DrawMensaje("GANASTE", Color.Green);
            }

            // Mostrar panel de ayuda si está activado
            if (_showHelp)
            {
                _hud.DrawHelp();
            }

            _hud.End();
        }
    }

    public void StartGame(TimeSpan tiempoPartida, int cantidadEnemigos, int indiceSeleccionado)
    {
        _tank.Reset(); // Resetear el tanque del jugador al inicio de una nueva partida
        _matchTimeSeconds = (float)tiempoPartida.TotalSeconds;
        SpawnearTanks(cantidadEnemigos);
        _enemyCount = cantidadEnemigos;

        // Iniciar música de gameplay
        StartGameplayMusic();
        _tank.Texture = _tankEntries[indiceSeleccionado].Texture;
        _state = GameState.Playing;
        _tank._effect.CurrentTechnique = _tank._effect.Techniques["BasicDrawing"];
        _tank._effect.Parameters["Ka"]?.SetValue(0.2f);
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

    private void SpawnearTanks(int cantTanks)
    {
        for (int i = 1; i < cantTanks + 1; i++) // Desde 1 porque la posición 0 es la del jugador
        {
            var generatedPosition = _positionGenerator.ReservedPositions[i];
            var spawnPosition = new Vector3(generatedPosition.X, 0, generatedPosition.Y);

            var enemyTank = new EnemyTank(spawnPosition, _tank);

            enemyTank.CargarModelo("t90/T90", _tankShader, Content, _simulation, BufferPool, GraphicsDevice, Gizmos,
                _bodyProperties, _terrain);
            int index = Random.Next(_tankEntries.Count); // índice aleatorio entre 0 y Count-1
            enemyTank.Texture = _tankEntries[index].Texture;
            var enemyController = new TankController(enemyTank, 20, 200, 2, 100, 200f);
            _enemyTanks.Add(enemyTank);
            _enemyControllers.Add(enemyController);
        }

        _debug.Tanks = [.._enemyTanks, _tank];

        // Construyo el diccionario BodyHandle → Tank
        var tankMap = new Dictionary<BodyHandle, Tank>();

        foreach (var handle in _tank.BodyHandles)
        {
            tankMap[handle] = _tank;
        }

        foreach (var tank in _enemyTanks)
        {
            foreach (var handle in tank.BodyHandles)
            {
                tankMap[handle] = tank;
            }
        }

        // Se lo paso al handler
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
                MediaPlayer.Volume = 0.34f; // 34% del volumen (música de fondo)
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
        GraphicsDevice.Clear(ClearOptions.Target | ClearOptions.DepthBuffer, Color.White, 1f, 0);

        _shadowEffect.CurrentTechnique = _shadowEffect.Techniques["DepthPass"];


        foreach (var enemyTank in _enemyTanks)
        {
            enemyTank.DrawShadow(_shadowEffect, _targetLightCamera, _lightBoundingFrustum);
        }

        _tank.DrawShadow(_shadowEffect, _targetLightCamera, _lightBoundingFrustum);

        foreach (var instance in _allInstances)
        {
            var model = instance.Model;
            var modelMeshesBaseTransforms = new Matrix[model.Bones.Count];
            model.CopyAbsoluteBoneTransformsTo(modelMeshesBaseTransforms);
            var worlds = instance.Worlds;
            if (instance.Texturas.Length > 0)
                _shadowEffect.Parameters["ModelTexture"]?.SetValue(instance.Texturas[0]);

            foreach (var world in worlds)
            {
                if (!instance.EsVisible(world, _lightBoundingFrustum))
                {
                    continue;
                }

                foreach (var modelMesh in model.Meshes)
                {
                    foreach (var part in modelMesh.MeshParts)
                        part.Effect = _shadowEffect;

                    var worldMatrix = modelMeshesBaseTransforms[modelMesh.ParentBone.Index] * world;
                    // WorldViewProjection is used to transform from model space to clip space
                    _shadowEffect.Parameters["WorldViewProjection"]
                        .SetValue(worldMatrix * _targetLightCamera.View * _targetLightCamera.Projection);

                    // Once we set these matrices we draw
                    modelMesh.Draw();
                }
            }
        }

        _trees.DrawSombra(_lightBoundingFrustum, _shadowEffect, _targetLightCamera, _elapsedTime);
        _terrain.DrawPastoShadow(_lightBoundingFrustum, GraphicsDevice, _targetLightCamera, _pasto, _elapsedTime);
    }

    

    private void DibujarTerreno()
    {
        if (_dibujarSombras)
        {
            _terrainEffect.CurrentTechnique = _terrainEffect.Techniques["DrawShadowedPCF"];
            _terrainEffect.Parameters["shadowMap"]?.SetValue(_shadowMapRenderTarget);
            _terrainEffect.Parameters["shadowMapSize"]?.SetValue(Vector2.One * ShadowmapSize);
        }
        else
        {
            _terrainEffect.CurrentTechnique = _terrainEffect.Techniques["RenderTerrain"];
        }

        _terrainEffect.Parameters["lightPosition"]?.SetValue(_lightPosition);
        _terrainEffect.Parameters["LightViewProjection"]?.SetValue(_targetLightCamera.View * _targetLightCamera.Projection);
        
        // Configurar parámetros de niebla
        _terrainEffect.Parameters["FogColor"]?.SetValue(new Vector3(0.5f, 0.6f, 0.7f));
        _terrainEffect.Parameters["FogStart"]?.SetValue(2000f);
        _terrainEffect.Parameters["FogEnd"]?.SetValue(3000f);

        _terrain.Draw(Matrix.Identity, _camera, _boundingFrustum);
    }

    private void DibujarElementos()
    {
        _effect.Parameters["lightPosition"]?.SetValue(_lightPosition);
        
        // Configurar parámetros de niebla
        _effect.Parameters["FogColor"]?.SetValue(new Vector3(0.5f, 0.6f, 0.7f));
        _effect.Parameters["FogStart"]?.SetValue(2000f);
        _effect.Parameters["FogEnd"]?.SetValue(3000f);

        _rocks.Draw(_effect, _boundingFrustum, "Piedra", _usarNormalMapping, _elapsedTime);

        GraphicsDevice.RasterizerState = RasterizerState.CullNone;
        _houses.Draw(_effect, _boundingFrustum, "Casa", _usarNormalMapping, _elapsedTime);

        _trees.Draw(_effect, _boundingFrustum, "Arbol", _usarNormalMapping, _elapsedTime);

        //_bushes.Draw(_effect,_boundingFrustum, "Arbusto", _usarNormalMapping);
        GraphicsDevice.RasterizerState = RasterizerState.CullCounterClockwise;

        _lightPoles.Draw(_effect, _boundingFrustum, "Poste de luz", _usarNormalMapping, _elapsedTime);
    }

    private void DibujarProyectiles()
    {
        foreach (var projectile in _projectiles)
            projectile.Draw(_effect, _camera.View, _camera.Projection);
    }

    private void DibujarPasto()
    {
        _terrain.DibujarPasto(_pasto, _elapsedTime, _camera);
    }

    private void DibujarTanques()
    {
        foreach (var enemyTank in _enemyTanks)
        {
            enemyTank.DrawOutline(_camera, GraphicsDevice, Color.Red.ToVector3(), _boundingFrustum);
        }
        _tank.DrawOutline(_camera, GraphicsDevice, Color.White.ToVector3(), _boundingFrustum);
        
        foreach (var enemyTank in _enemyTanks)
        {
            enemyTank.Draw(_camera, _shadowMapRenderTarget, ShadowmapSize, _targetLightCamera, _boundingFrustum);
        }
        _tank.Draw(_camera, _shadowMapRenderTarget, ShadowmapSize, _targetLightCamera, _boundingFrustum);

    }
}