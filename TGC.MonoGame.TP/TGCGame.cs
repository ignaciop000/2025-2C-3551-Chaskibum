// LISTA DE TAREAS:
// - Solucionar Bug Camara Free [AGUS]
// - Mejorar torreta (que siga mejor al mouse y que no se desincronice el collider) [MATEO]
// - Arreglar collider Roca 9 [SANTI Y AGUS]
// - Solucionar y Completar Texturas [SANTI]
// - No quedarte atascado ni dado vuelta / Solucion atascamientos [MATEO]

// - Incluir Panzer, distintas caracteristicas que T90: vida, daño base, velocidad [NACHO]
// - Agregar mas Tanques IA (T90 o Panzer), aparecen de a poco hasta llegar al límite [MATEO]
// - Contador, enemigos restantes, victoria/derrota [MATEO]
// - Postes de luz [AGUS]
// - Vida, Cooldown, tipos proyectiles [AGUS]
// - HUD (Vida, Cooldown de disparo, Tipo proyectil: + velocidad => - daño, Ganaste/Perdiste/Juga de nuevo) [NACHO]
// - Menú (Iniciar -> Elegir tanque, los 2 tienen distintas caracteristicas;
//          Configurar tiempo y cant. enemigos;
//          Configurar volumenes;
//          Integrantes) [NACHO]
// - Música (menu y juego) y Sonidos (acciones de hud, daño, choque, disparo y motor) [SANTI]
// - Imagen tutorial [SANTI]


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
    private const float EscalaMapa = 20;

    public const string ContentFolder3D = "Models/";
    public const string ContentFolderEffects = "Effects/";
    public const string ContentFolderMusic = "Music/";
    public const string ContentFolderSounds = "Sounds/";
    public const string ContentFolderSpriteFonts = "SpriteFonts/";
    public const string ContentFolderTextures = "Textures/";

    private readonly GraphicsDeviceManager _graphics;
    
    private Camera _camera;
    private OrbitCamera _orbitCamera;
    
    private Vector3 _desiredLookAt;
    private bool _hayLookAt;
    private Vector3 _lookAt;
    
    private Effect _terrainEffect;
    private Effect _effect;
    
    private Simulation _simulation;
    private CollidableProperty<TankBodyProperties> _bodyProperties;
    private TankCallbacks _callbacks;
    private CollisionHandler _collisionHandler;
    private Gizmos Gizmos { get; set;}
    private BufferPool BufferPool { get; set; }

    private PositionGenerator _positionGenerator;
    private Terrain _terrain;
    
    private KeyboardState _kbPrev;
    
    private TankController _playerController;
    private TankController _enemyController;
    
    private Tank _tank;
    private Tank _tank2;
    private List<Tank> _tanks;
    
    // Proyectiles
    private readonly List<Projectile> _projectiles = [];
    private MouseState _mousePrev;
    private float _fireCooldown;

    private Houses _houses;
    private Rocks _rocks;
    private Trees _trees;
    private Bushes _bushes;

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
        BufferPool = new BufferPool();
        _desiredLookAt = Vector3.Zero;

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
        
        _bodyProperties = new CollidableProperty<TankBodyProperties>();
        
        _callbacks = new TankCallbacks() { Properties = _bodyProperties};
        _callbacks.SetCollisionHandler(_collisionHandler);
        
        _simulation = Simulation.Create(BufferPool, _callbacks,
            new PoseIntegratorCallbacks(new Vector3(0, -120, 0)), new SolveDescription(8, 1));
        
        _tank = new Tank(new Vector3(0, 0, 0), _orbitCamera, 0f, 0.1f );
        _tank2 = new Tank(new Vector3(500, 0, 0), null, 0f, 0.1f );
        
        _tanks = [_tank, _tank2];
        
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
        
        // Cargar shader específico para tanques
        var tankShader = Content.Load<Effect>(ContentFolderEffects + "TankShader");
        
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

        _tank.CargarModelo("t90/T90", tankShader, Content, _simulation, BufferPool, GraphicsDevice, Gizmos, _bodyProperties, _terrain);
        _tank2.CargarModelo("t90/T90", tankShader, Content, _simulation, BufferPool, GraphicsDevice, Gizmos, _bodyProperties, _terrain);
        
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
        
        _playerController = new TankController(_tank, 20,200 , 2, 100, 200f);
        _enemyController = new TankController(_tank2, 20, 200, 2, 100, 200f);

        _trees = new Trees(_terrain, _simulation);
        _houses = new Houses(_terrain, _simulation);
        _rocks = new Rocks(_terrain, _simulation);
        _bushes = new Bushes(_terrain, _simulation);
        
        _houses.SetPlacementRules(5f,  false); // ≤ 5°, NO se inclinan
        _trees.SetPlacementRules(20f,  true);  // ≤ 20°, se inclinan
        _bushes.SetPlacementRules(25f, true);  // ≤ 25°, se inclinan
        _rocks.SetPlacementRules(null, true);  // sin restricción, se inclinan


        // Generacion de posiciones de modelos

        var anchoMapa = (_terrain.HeightmapData.GetLength(0) - 1) * EscalaMapa;
        var largoMapa = (_terrain.HeightmapData.GetLength(1) - 1) * EscalaMapa;
        
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
        [_tank, _tank2], 
            _orbitCamera,
            _simulation, 
            _terrain,
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
        var deltaTime = (float)gameTime.ElapsedGameTime.TotalSeconds;
        var keyboardState = Keyboard.GetState();
        var mouseState = Mouse.GetState();



        float leftTargetSpeedFraction = 0;
        float rightTargetSpeedFraction = 0;
        var left = keyboardState.IsKeyDown(Keys.A);
        var right = keyboardState.IsKeyDown(Keys.D);
        var forward = keyboardState.IsKeyDown(Keys.W);
        var backward = keyboardState.IsKeyDown(Keys.S);

        if (forward)
        {
            if ((left && right) || (!left && !right))
            {
                leftTargetSpeedFraction = 1f;
                rightTargetSpeedFraction = 1f;
            }
            //Note turns require a bit of help from the opposing track to overcome friction.
            else if (left)
            {
                leftTargetSpeedFraction = 0.5f;
                rightTargetSpeedFraction = 1f;
            }
            else
            {
                leftTargetSpeedFraction = 1f;
                rightTargetSpeedFraction = 0.5f;
            }
        }
        else if (backward)
        {
            if ((left && right) || (!left && !right))
            {
                leftTargetSpeedFraction = -1f;
                rightTargetSpeedFraction = -1f;
            }
            else if (left)
            {
                leftTargetSpeedFraction = -0.5f;
                rightTargetSpeedFraction = -1f;
            }
            else
            {
                leftTargetSpeedFraction = -1f;
                rightTargetSpeedFraction = -0.5f;
            }
        }
        else
        {
            //Not trying to move. Turn?
            if (left && !right)
            {
                leftTargetSpeedFraction = -1f;
                rightTargetSpeedFraction = 1f;
            }
            else if (right && !left)
            {
                leftTargetSpeedFraction = 1f;
                rightTargetSpeedFraction = -1f;
            }
        }

        var zoom = keyboardState.IsKeyDown(Keys.LeftShift);
        var brake = keyboardState.IsKeyDown(Keys.Space);
        var frontDirection = new Vector3(_camera.FrontDirection.X, _camera.FrontDirection.Y, _camera.FrontDirection.Z);

        // Dirección de mira a partir del mouse (si hay hit en el terreno)
        var aimDir = frontDirection; // fallback
        var hit = PickOnTerrain(mouseState.Position);
        if (hit.HasValue)
        {
            var aimXna = hit.Value -
                         new Microsoft.Xna.Framework.Vector3(_tank.Position.X, _tank.Position.Y, _tank.Position.Z);
            if (aimXna.LengthSquared() > 1e-6f)
                aimXna.Normalize();
            aimDir = new Vector3(aimXna.X, aimXna.Y, aimXna.Z);
        }

        _tank.AimDirectionWorld = aimDir;
        _playerController.UpdateMovementAndAim(_simulation, leftTargetSpeedFraction, rightTargetSpeedFraction, zoom,
            brake, brake, aimDir);

        // cooldown
        _fireCooldown = MathF.Max(0f, _fireCooldown - deltaTime);

        _tank?.Update(gameTime, keyboardState);

        // Update AI for enemy tank (_tank2)
        _tank2?.UpdateEnemyTankAI(_tank.Position, _enemyController);

        _tank2?.Update(gameTime);

        // click izquierdo: dispara
        if (_fireCooldown <= 0f && mouseState.LeftButton == ButtonState.Pressed &&
            _mousePrev.LeftButton == ButtonState.Released)
        {
            // Velocidad configurable acá:
            float speed = 300f;
            float projMass = 2f;

            var (muzzle, dir) = _tank.GetMuzzle(); // offset local del cañón 
            var proj = new Projectile(_simulation, _effect, muzzle, dir, speed);
            _projectiles.Add(proj);

            // Retroceso + freno breve
            _tank.TriggerRecoil(dir, projectileMass: projMass, muzzleSpeed: speed, intensity: 1f, withBrake: true);

            _fireCooldown = 1f; // 4 disparos/seg
        }

        // update de todos los proyectiles
        for (int i = _projectiles.Count - 1; i >= 0; --i)
        {
            _projectiles[i].Update(deltaTime);
            if (_projectiles[i].IsDead) _projectiles.RemoveAt(i);
        }
        
        // Actualizar simulación física
        if (_simulation != null && deltaTime is > 0.0f and < 0.1f) // Máximo 100ms por frame
        {
            _simulation.Timestep(deltaTime);
            
            // Procesar colisiones
            _collisionHandler.HandleCollisions();
            
            _tank?.SyncFromPhysics();
            _tank?.ApplyRecoilAndBrake(deltaTime, _simulation);
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
            var targetHeight = _terrain.GetHeightAtPosition(_tank.Position.X, _tank.Position.Z) + 50f; 
            _orbitCamera.SetTarget(new Vector3(_tank.Position.X, targetHeight, _tank.Position.Z));
            var dir = new Vector2(MathF.Cos(_tank.Rotation), MathF.Sin(_tank.Rotation));

            _desiredLookAt = new Vector3(_tank.Position.X, _terrain.GetHeightAtPosition(_tank.Position.X, _tank.Position.Z), _tank.Position.Z);
            if (!_hayLookAt)
            {
                _lookAt = _desiredLookAt;
                _hayLookAt = true;
            }
            else
            {
                var lamda = 0.05f;
                _lookAt = _desiredLookAt * lamda + _lookAt * (1 - lamda);
            }

            var tankPos2D = new Vector2(_tank.Position.X, _tank.Position.Z);
            var cameraPos2D = tankPos2D - dir * 800; // Distancia de 800 unidades detrás del tanque

            // Calcular la altura máxima entre la cámara y el tanque
            float H = 0;
            for (var i = 0; i < 10; ++i)
            {
                var t = i / 10.0f;
                var p = cameraPos2D * t + tankPos2D * (1 - t);
                var Hi = _terrain.GetHeightAtPosition(p.X, p.Y) + 50;
                if (Hi > H) H = Hi;
            }
            
            // Actualizar la cámara (maneja el input del mouse)
            _camera.Update(gameTime);
            _camera.ConstrainAboveTerrain(_terrain, clearance: 50f, samples: 16);
            Gizmos.UpdateViewProjection(_camera.View, _camera.Projection);
        }
        
        _kbPrev = keyboardState;
        _mousePrev = mouseState;
        base.Update(gameTime);
    }
    
    /// <summary>
    /// Updates the AI behavior for the enemy tank (_tank2)
    /// </summary>
    

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
        if (_terrainEffect == null || _terrain == null)
            return;

        // Para dibujar el modelo necesitamos pasarle informacion que el efecto esta esperando.
        _terrainEffect.Parameters["View"].SetValue(_camera.View);
        _terrainEffect.Parameters["Projection"].SetValue(_camera.Projection);
        
        _effect.Parameters["View"].SetValue(_camera.View);
        _effect.Parameters["Projection"].SetValue(_camera.Projection);

        var oldRasterizerState = GraphicsDevice.RasterizerState;
        GraphicsDevice.RasterizerState = RasterizerState.CullNone;
        _terrain.Draw(Matrix.Identity, _camera.View, _camera.Projection);
        GraphicsDevice.RasterizerState = oldRasterizerState;
        
        _tank.Draw();
        _tank2?.Draw();

        _trees.Dibujar();
        _houses.Dibujar();
        _rocks.Dibujar();
        _bushes.Dibujar();
        
        foreach (var s in _projectiles)
            s.Draw(GraphicsDevice, _camera.View, _camera.Projection);

        _debug.Draw(_camera);

    }
    
    private Microsoft.Xna.Framework.Vector3? PickOnTerrain(Point mouse)
    {
        // Desarma matrices
        var view = _camera.View;
        var proj = _camera.Projection;
        var vp = GraphicsDevice.Viewport;

        // Dos puntos en NDC (near/far) -> espacio mundo
        var nearPoint = vp.Unproject(new Microsoft.Xna.Framework.Vector3(mouse.X, mouse.Y, 0f), proj, view, Matrix.Identity);
        var farPoint  = vp.Unproject(new Microsoft.Xna.Framework.Vector3(mouse.X, mouse.Y, 1f), proj, view, Matrix.Identity);

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