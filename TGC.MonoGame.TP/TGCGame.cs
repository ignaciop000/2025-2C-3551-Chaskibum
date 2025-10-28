// LISTA DE TAREAS:
// - Solucionar Bug Camara Free [AGUS] [COMPLETADO]
// - Arreglar collider Roca 9 [SANTI Y AGUS]
// - Solucionar y Completar Texturas [SANTI]


// - No quedarte atascado ni dado vuelta / Solucion atascamientos [MATEO]
// - Agregar mas Tanques IA (T90 o Panzer), aparecen de a poco hasta llegar al límite [MATEO]
// - Contador, enemigos restantes, victoria/derrota [MATEO]

// - Incluir Panzer, distintas caracteristicas que T90: vida, daño base, velocidad [NACHO]
// - Postes de luz [AGUS]
// - Vida, Cooldown, tipos proyectiles [AGUS] [COMPLETADO]
// - HUD (Vida, Cooldown de disparo, Tipo proyectil: + velocidad => - daño, Ganaste/Perdiste/Juga de nuevo) [NACHO] [COMPLETADO]
// - Menú (Iniciar -> Elegir tanque, los 2 tienen distintas caracteristicas;
//          Configurar tiempo y cant. enemigos;
//          Configurar volumenes;
//          Integrantes) [NACHO] [COMPLETADO]
// - Música (menu y juego) y Sonidos (acciones de hud, daño, choque, disparo y motor) [SANTI]
// - Imagen tutorial [SANTI]
// - Agregar World Border [AGUS]
// - Emprolijar código físicas [OPCIONAL, POR AHORA]
// - Que se vea el debug de los proyectiles [OPCIONAL]
// - Cielo y fog [OPCIONAL]


using System;
using System.Collections.Generic;
using System.Linq;
using BepuPhysics;
using BepuUtilities.Memory;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using TGC.MonoGame.TP.Viewer.Gizmos;
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
    //Debug
    private Gizmos Gizmos { get; set;}
    
    
    public const string ContentFolder3D = "Models/";
    public const string ContentFolderEffects = "Effects/";
    public const string ContentFolderMusic = "Music/";
    public const string ContentFolderSounds = "Sounds/";
    public const string ContentFolderSpriteFonts = "SpriteFonts/";
    public const string ContentFolderTextures = "Textures/";
    private SpriteBatch _spriteBatch;
    private SpriteFont _font;

    private enum GameState { MainMenu, Playing }
    private GameState _state = GameState.MainMenu;
    
    private const float EscalaMapa = 30;
    private readonly GraphicsDeviceManager _graphics;
    
    private Camera _camera;                 // Cámara activa
    private OrbitCamera _orbitCamera;       // Cámara que sigue al tanque
    
    private Effect _terrainEffect;          //Shader Terreno
    private Effect _effect;                 //Shader Basico
    
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
    private TankController _enemyController;
    private int _enemyCount;

    private Tank _tank;
    private List<Tank> enemyTanks;
    private List<TankController> enemyControllers;
    private List<Tank> _tanks;

    private Effect tankShader;
    // Proyectiles
    private readonly List<Projectile> _projectiles = [];
    private MouseState _mousePrev;

    private Houses _houses;
    private Rocks _rocks;
    private Trees _trees;
    private Bushes _bushes;
    private LightPoles _lightPoles;
    
    private float _matchTimeSeconds = 0f;
    private bool _hasLost = false;
    private bool _hasWon = false;
    
    private float _playerHealth = 100f;
    private float _playerMaxHealth = 100f;

    private List<TankEntry> _tankEntries = new();   
    private Debug _debug;
    private HUD _hud;
    private Menu _menu;
    
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
            50000
        );
        // Seteo la cámara inicial como la orbital
        _camera = _orbitCamera;

        _collisionHandler = new CollisionHandler();
        _bodyProperties = new CollidableProperty<TankBodyProperties>(); //BEPU
        _callbacks = new TankCallbacks() { Properties = _bodyProperties };
        _callbacks.SetCollisionHandler(_collisionHandler);

        _simulation = Simulation.Create(BufferPool, _callbacks,
            new PoseIntegratorCallbacks(new Vector3(0, -120, 0)), new SolveDescription(8, 1)); //TODO

        _tank = new Tank(new Vector3(0, 0, 0), 0f, 0.1f);
        
        _tanks = [_tank];

        _debug = new Debug();
        _menu = new Menu();
        _hud = new HUD(GraphicsDevice);
        enemyTanks = new List<Tank>();
        enemyControllers = new List<TankController>();
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
        
        // Cargar shader específico para tanques
        tankShader = Content.Load<Effect>(ContentFolderEffects + "TankShader");
        
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
            EscalaMapa
            );
        
        var tankT90 = Content.Load<Model>(ContentFolder3D + "t90/T90");
        var hullATexture = Content.Load<Texture2D>(TGCGame.ContentFolder3D + "t90/textures_mod/hullA");
        _tankEntries.Add(new TankEntry("T-90-A", tankT90, hullATexture, 0.002f, 0.5f, tankShader));
        
        var hullBTexture = Content.Load<Texture2D>(TGCGame.ContentFolder3D + "t90/textures_mod/hullB");
        _tankEntries.Add(new TankEntry("T-90-B", tankT90, hullBTexture, 0.002f, 0.5f, tankShader));
        
        var hullCTexture = Content.Load<Texture2D>(TGCGame.ContentFolder3D + "t90/textures_mod/hullC");
        _tankEntries.Add(new TankEntry("T-90-C", tankT90, hullCTexture, 0.002f, 0.5f, tankShader));
         
        _tank.CargarModelo("t90/T90", tankShader, Content, _simulation, BufferPool, GraphicsDevice, Gizmos, _bodyProperties, _terrain);

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
        
        _trees.CargarModelos(_effect, Content);
        _houses.CargarModelos(_effect, Content);
        _rocks.CargarModelos(_effect, Content);
        _bushes.CargarModelos(_effect, Content);
        _lightPoles.CargarModelos(_effect, Content);
        
        _worldBorder = new WorldBorder(GraphicsDevice, _effect, _simulation, anchoMapa, largoMapa);
        
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
        _font = Content.Load<SpriteFont>(ContentFolderSpriteFonts + "CascadiaCode/CascadiaCodePL");
        _spriteBatch = new SpriteBatch(GraphicsDevice);
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
                var proj = new Projectile(_simulation, _effect, muzzle, dir, tipoProyectilActual);
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
            
            for (int i = enemyTanks.Count - 1; i >= 0; i--)
            {
                var enemyTank = enemyTanks[i];
                if (enemyTank.IsDead)
                {
                    enemyTanks.RemoveAt(i);
                    enemyControllers.RemoveAt(i);
                    _enemyCount--;
                    continue;
                }
                
                var enemyController = enemyControllers[i];
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
        }
        
        _tank?.Update(gameTime, keyboardState);

        // update de todos los proyectiles
        for (var i = _projectiles.Count - 1; i >= 0; --i)
        {
            _projectiles[i].Update(deltaTime);
            if (_projectiles[i].IsDead) _projectiles.RemoveAt(i);
        }
        
        // Actualizar World Border
        _worldBorder.Update(_tank.Position.ToNumerics(), deltaTime);
        
        // Actualizar simulación física
        if (_simulation != null && deltaTime is > 0.0f and < 0.1f) // Máximo 100ms por frame
        {
            _simulation.Timestep(deltaTime);
            _collisionHandler.HandleCollisions();
            
            _tank?.SyncFromPhysics();
            _tank?.ApplyRecoilAndBrake(deltaTime, _simulation);
        }
        
        _debug.Update(keyboardState, _kbPrev, deltaTime, _camera);

        // Actualizar cámara para seguir al tanque
        if (_tank != null)
        {
            // Usar la posición y rotación del tanque
            var alturaTerreno = _terrain.GetHeightAtPosition(_tank.Position.X, _tank.Position.Z); 
            _orbitCamera.SetTarget(new Vector3(_tank.Position.X, alturaTerreno + 50f, _tank.Position.Z));

            // Actualizar la cámara (maneja el input del mouse)
            _camera.Update(gameTime);
            _camera.ConstrainAboveTerrain(_terrain, clearance: 50f, samples: 16);
            
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
        
        GraphicsDevice.Clear(Color.Black);
        // Limpia también el depth buffer
        GraphicsDevice.Clear(ClearOptions.Target | ClearOptions.DepthBuffer, Color.Black, 1f, 0);
        
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

        var oldRasterizerState = GraphicsDevice.RasterizerState;
        GraphicsDevice.RasterizerState = RasterizerState.CullNone;
        _terrain.Draw(Matrix.Identity, _camera.View, _camera.Projection);
        GraphicsDevice.RasterizerState = oldRasterizerState;
        
        _tank.Draw(_camera);
        if (_state != GameState.MainMenu)
        {
            foreach (var enemyTank in enemyTanks)
            {
                enemyTank.Draw(_camera);
            }
        }

        _trees.Dibujar();
        _houses.Dibujar();
        _rocks.Dibujar();
        _bushes.Dibujar();
        //_lightPoles.Dibujar();
        
        foreach (var projectile in _projectiles)
            projectile.Draw(_effect, _camera.View, _camera.Projection);
        
        _worldBorder.Draw(_camera.View, _camera.Projection);

        _debug.Draw(_camera);
        if (!_hasLost && !_hasWon)
        {
            _hud.Draw(_matchTimeSeconds, _tank.FireCooldown, _tank.TipoProyectilActual.MaxCooldown, _tank.TipoProyectilActual, _playerHealth, _playerMaxHealth, _enemyCount);
        }
        
        if (_hasLost)
        {
            _hud.DrawMensaje("PERDISTE", Color.Red);
            return;
        }

        if (_hasWon)
        {
            _hud.DrawMensaje("GANASTE", Color.Green);
        }
    }

    public void StartGame(TimeSpan tiempoPartida, int cantidadEnemigos, int tanqueSeleccionado)
    {
        _state = GameState.Playing;
        _matchTimeSeconds = (float)tiempoPartida.TotalSeconds;
        spawnearTanks(cantidadEnemigos);
        _enemyCount=cantidadEnemigos;
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
    
    public void spawnearTanks(int cantTanks)
    {
        for (int i = 0; i < cantTanks; i++)
        {
            var enemyTank = new  Tank(new Vector3(500 + 100 * i, 0, 50 * i), 0f, 0.1f);
            enemyTank.CargarModelo("t90/T90", tankShader, Content, _simulation, BufferPool, GraphicsDevice, Gizmos,
                _bodyProperties, _terrain);
            var enemyController = new TankController(enemyTank, 20, 200, 2, 100, 200f);
            enemyTanks.Add(enemyTank);
            enemyControllers.Add(enemyController);
        }
        
        _tanks.AddRange(enemyTanks);
        
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
}