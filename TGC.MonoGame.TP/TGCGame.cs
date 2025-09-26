using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using System.Numerics;
using System.Transactions;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using TGC.MonoGame.Samples.Cameras;
using MathHelper = BepuUtilities.MathHelper;
using Matrix = Microsoft.Xna.Framework.Matrix;
using Vector2 = System.Numerics.Vector2;
using Vector3 = Microsoft.Xna.Framework.Vector3;

namespace TGC.MonoGame.TP;

/// <summary>
///     Esta es la clase principal del juego.
///     Inicialmente puede ser renombrado o copiado para hacer mas ejemplos chicos, en el caso de copiar para que se
///     ejecute el nuevo ejemplo deben cambiar la clase que ejecuta Program <see cref="Program.Main()" /> linea 10.
/// </summary>
public class TGCGame : Game
{
    /// <summary>
    /// NarrowPhaseCallbacks es una estructura que define las operaciones relacionadas con la detección de colisiones
    /// en la fase estrecha dentro de la simulación física. Su implementación permite personalizar cómo se generan
    /// los contactos entre los objetos, cómo se configuran los parámetros de los contactos y cómo se administran los
    /// recursos asociados a esta fase en la simulación.
    /// </summary>
    struct NarrowPhaseCallbacks : INarrowPhaseCallbacks
    {
        /// <summary>
        /// Propiedad que define la elasticidad del resorte utilizado para manejar los contactos
        /// en la simulación física. Esta elasticidad está representada por los parámetros
        /// de frecuencia angular y razón de amortiguamiento dos veces, que controlan cómo
        /// la simulación responde a las fuerzas de contacto entre los objetos.
        /// Los valores de ContactSpringiness afectan directamente la estabilidad y el comportamiento
        /// físico de los objetos al interactuar en colisión. Configuraciones que no son óptimas pueden
        /// resultar en comportamientos no deseados, como penetraciones excesivas o explosiones numéricas.
        /// </summary>
        private SpringSettings ContactSpringiness { get; set; }

        /// <summary>
        /// Propiedad que define la velocidad máxima de recuperación permitida entre los objetos en contacto
        /// durante la simulación física. Este valor limita la magnitud de la velocidad relativa con la que
        /// los objetos intentan separarse después de una colisión, ayudando a controlar comportamientos
        /// indeseados, como rebotes excesivos o respuestas físicamente inestables.
        /// </summary>
        private float MaximumRecoveryVelocity { get; set; }

        /// <summary>
        /// Propiedad que define el coeficiente de fricción utilizado en la simulación física.
        /// Este valor determina la resistencia relativa al deslizamiento entre dos superficies
        /// en contacto. Un coeficiente más bajo representará superficies más resbaladizas, mientras
        /// que un coeficiente más alto representará mayor resistencia al deslizamiento.
        /// El uso de valores adecuados para FrictionCoefficient contribuye a lograr un comportamiento
        /// físico más realista en las interacciones entre los objetos.
        /// </summary>
        private float FrictionCoefficient { get; set; }

        /// <summary>
        /// Inicializa la simulación física configurando valores predeterminados
        /// como la elasticidad del resorte, la velocidad máxima de recuperación y el coeficiente de fricción,
        /// en caso de no haber sido previamente inicializados.
        /// </summary>
        /// <param name="simulation">Simulación física que será configurada.</param>
        public void Initialize(Simulation simulation)
        {
            if (ContactSpringiness.AngularFrequency == 0 && ContactSpringiness.TwiceDampingRatio == 0)
            {
                ContactSpringiness = new SpringSettings(30, 1);
                MaximumRecoveryVelocity = 2f;
                FrictionCoefficient = 1f;
            }
        }

        /// <summary>
        /// Determina si se permite la generación de contactos entre dos colisionadores específicos
        /// durante la simulación física.
        /// </summary>
        /// <param name="workerIndex">Índice del trabajador que ejecuta esta llamada en el contexto de la simulación multi-hilo.</param>
        /// <param name="a">Primera referencia al colisionador involucrado en la posible colisión.</param>
        /// <param name="b">Segunda referencia al colisionador involucrado en la posible colisión.</param>
        /// <param name="speculativeMargin">Margen especulativo utilizado para extender el rango de detección de colisiones
        /// entre los colisionadores.</param>
        /// <returns>Devuelve true si se permite la generación de contactos entre los colisionadores, de lo contrario, devuelve false.</returns>
        public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin)
        {
            return a.Mobility == CollidableMobility.Dynamic || b.Mobility == CollidableMobility.Dynamic;
        }

        /// <summary>
        /// Configura las propiedades de un conjunto de contactos generados en la simulación física,
        /// estableciendo parámetros como el coeficiente de fricción, la velocidad máxima de recuperación
        /// y las configuraciones del resorte, necesarias para calcular las interacciones entre los objetos.
        /// </summary>
        /// <param name="workerIndex">Índice del trabajador que procesa este contacto en el sistema multihilo.</param>
        /// <param name="pair">Par de colisionadores que están interactuando.</param>
        /// <param name="manifold">Conjunto de contactos detectados entre los colisionadores.</param>
        /// <param name="pairMaterial">Propiedades del material del par que serán configuradas.</param>
        /// <typeparam name="TManifold">Tipo específico de la estructura que representa el conjunto de contactos.</typeparam>
        /// <returns>Un valor booleano que indica si deben generarse restricciones de contacto basadas en el conjunto configurado.</returns>
        public bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold,
            [UnscopedRef] out PairMaterialProperties pairMaterial) where TManifold : unmanaged, IContactManifold<TManifold>
        {
            pairMaterial.FrictionCoefficient = 1f;
            pairMaterial.MaximumRecoveryVelocity = 2f;
            pairMaterial.SpringSettings = new SpringSettings(30, 1);
            //For the purposes of the demo, contact constraints are always generated.
            return true;
        }

        /// <summary>
        /// Determina si se permite la generación de contactos en la simulación física entre dos objetos colisionables dados.
        /// </summary>
        /// <param name="workerIndex">Índice del trabajador que procesa esta operación, generalmente utilizado en simulaciones multihilo.</param>
        /// <param name="pair">Par de objetos colisionables que se están evaluando.</param>
        /// <param name="childIndexA">Índice del subcomponente del objeto A, en caso de colisionadores compuestos.</param>
        /// <param name="childIndexB">Índice del subcomponente del objeto B, en caso de colisionadores compuestos.</param>
        /// <returns>Un valor booleano que indica si se permite o no la generación de contactos entre los objetos evaluados.</returns>
        public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
        {
            return true;
        }

        /// <summary>
        /// Configura el colector de contactos especificando las propiedades materiales del par de colisionadores,
        /// como la fricción y la restitución, ajustando los contactos detectados según sea necesario.
        /// </summary>
        /// <param name="workerIndex">Índice del hilo de trabajo que genera el colector de contactos.</param>
        /// <param name="pair">Par de colisionadores cuya interacción está siendo procesada.</param>
        /// <param name="childIndexA">Índice del hijo del primer colisionador en el par.</param>
        /// <param name="childIndexB">Índice del hijo del segundo colisionador en el par.</param>
        /// <param name="manifold">Colector de contactos que contiene información de los puntos de contacto detectados.</param>
        /// <returns>
        /// Devuelve un valor booleano indicando si la configuración del colector de contactos fue exitosa.
        /// </returns>
        public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB,
            ref ConvexContactManifold manifold)
        {
            return true;
        }

        public void Dispose()
        {
            throw new NotImplementedException();
        }
    }

    /// <summary>
    /// Estructura utilizada para implementar los "callbacks" necesarios para la integración de las poses en el sistema físico.
    /// Define la aplicación de fuerzas externas, como la gravedad, y permite personalizar el comportamiento de integración
    /// de velocidades y posición de los cuerpos durante el tiempo de simulación.
    /// </summary>
    public struct PoseIntegratorCallbacks(Vector3 gravity) : IPoseIntegratorCallbacks
    {
        /// <summary>
        /// Propiedad que define el vector de gravedad usado en la simulación física.
        /// Representa la fuerza gravitacional que se
        public Vector3 Gravity;
        /// <summary>
        /// Propiedad que define el coeficiente de amortiguamiento lineal aplicado a los cuerpos en la simulación física.
        /// Este coeficiente determina la tasa a la que se reduce la velocidad lineal de los objetos con el tiempo.
        /// Un valor más alto indica un desaceleramiento más rápido, simulando efectos como la resistencia del aire
        /// o la fricción generalizada en un entorno simulado.
        /// </summary>
        public float LinearDamping;

        /// <summary>
        /// Propiedad que define el coeficiente de amortiguación angular aplicado a los cuerpos
        /// en la simulación física. Este coeficiente controla la cantidad de atenuación que se
        /// aplica a las velocidades angulares de los cuerpos a lo largo del tiempo, reduciendo
        /// gradualmente la rotación de los objetos hasta que cesa por completo.
        /// Un mayor valor de AngularDamping resultará en una desaceleración más rápida de los
        /// movimientos rotacionales, mientras que valores menores permitirán que los objetos
        /// mantengan su rotación por más tiempo. Es una herramienta crucial para modelar el
        /// comportamiento físico deseado, como evitar rotaciones perpetuas en sistemas sin fricción.
        /// </summary>
        public float AngularDamping;
        
        /// <summary>
        /// Variable que almacena los valores de gravedad multiplicados por el intervalo de tiempo (dt) en formato de vectores anchos (SIMD).
        /// Esta optimización permite precalcular y evitar cómputos redundantes durante cada integración de las velocidades.
        /// Es utilizada dentro de los "callbacks" del integrador de poses para aplicar las fuerzas gravitatorias de manera eficiente
        /// a múltiples cuerpos simultáneamente durante la simulación física.
        /// </summary>
        private Vector3Wide GravityWideDt;

        /// <summary>
        /// Variable que representa el factor de amortiguamiento lineal aplicado por unidad de tiempo en la simulación.
        /// Este valor se calcula a partir de un coeficiente inicial de amortiguamiento lineal y el paso temporal
        /// de integración (dt). Controla la reducción progresiva de la velocidad lineal de los cuerpos en la simulación,
        /// simulando la resistencia o pérdida de energía debido a factores como la fricción con el medio ambiente.
        /// Ajustar este valor impacta directamente la estabilidad del sistema físico y la forma en la que los objetos
        /// desaceleran a lo largo del tiempo.
        /// </summary>
        private Vector<float> LinearDampingDt;

        /// <summary>
        /// Variable que define el amortiguamiento angular aplicado durante la integración
        /// temporal en el sistema de simulación física. Este valor se utiliza para reducir
        /// gradualmente las velocidades angulares de los cuerpos, simulando el efecto de
        /// fricción rotacional o pérdida de energía angular.
        /// El cálculo de AngularDampingDt depende del valor del amortiguamiento angular
        /// configurado y del intervalo de tiempo de simulación. Este parámetro es crucial
        /// para mantener la estabilidad de la simulación y controlar el comportamiento
        /// dinámico de los objetos en rotación.
        /// </summary>
        private Vector<float> AngularDampingDt;

        public void Initialize(Simulation simulation)
        {
            
        }

        /// <summary>
        /// Prepara los cálculos previos necesarios para la integración del sistema físico.
        /// Calcula y almacena las amortiguaciones lineal y angular ajustadas al intervalo de tiempo.
        /// </summary>
        /// <param name="dt">Intervalo de tiempo para el cual se realizan los cálculos de integración.</param>
        public void PrepareForIntegration(float dt)
        {
            //No reason to recalculate gravity * dt for every body; just cache it ahead of time.
            //Since these callbacks don't use per-body damping values, we can precalculate everything.
            LinearDampingDt = new Vector<float>(MathF.Pow(MathHelper.Clamp(1 - LinearDamping, 0, 1), dt));
            AngularDampingDt = new Vector<float>(MathF.Pow(MathHelper.Clamp(1 - AngularDamping, 0, 1), dt));
            //GravityWideDt = Vector3Wide.Broadcast(Gravity * dt);
            
        }

        public void IntegrateVelocity(Vector<int> bodyIndices, Vector3Wide position, QuaternionWide orientation,
            BodyInertiaWide localInertia, Vector<int> integrationMask, int workerIndex, Vector<float> dt, ref BodyVelocityWide velocity)
        {
            // Aplicar gravedad usando vectores SIMD
            velocity.Linear.Y = velocity.Linear.Y + new Vector<float>(gravity.Y) * dt;
        }

        public AngularIntegrationMode AngularIntegrationMode { get; } = AngularIntegrationMode.Nonconserving;
        public bool AllowSubstepsForUnconstrainedBodies { get; } = false;
        public bool IntegrateVelocityForKinematics { get; } = false;
    }
    public const string ContentFolder3D = "Models/";
    public const string ContentFolderEffects = "Effects/";
    public const string ContentFolderMusic = "Music/";
    public const string ContentFolderSounds = "Sounds/";
    public const string ContentFolderSpriteFonts = "SpriteFonts/";
    public const string ContentFolderTextures = "Textures/";
    
    private readonly GraphicsDeviceManager _graphics;
    private readonly Random _rnd = new Random();
    private OrbitCamera _camera;
    public Vector3 DesiredLookAt;
    public bool hay_lookAt;
    public Vector3 LookAt;
    public Vector2 pos;
    private Effect _effect;
    private Simulation _simulation;
    
    private PositionGenerator _positionGenerator;
    private float angle;
    private Ground _ground;
    public Terrain terrain;
    
    private Tank _tank;

    private ModelInstances _tank2 = new ModelInstances();
    private ModelInstances _panzer = new ModelInstances();
    private ModelInstances _t90 = new ModelInstances();
    
    private Houses _houses = new Houses();
    private Rocks _rocks = new Rocks();
    private Trees _trees = new Trees();
    private Bushes _bushes = new Bushes();

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
        // La logica de inicializacion que no depende del contenido se recomienda poner en este metodo.
        
        // Inicializacion de camara
        var size = GraphicsDevice.Viewport.Bounds.Size;
        size.X /= 2;
        size.Y /= 2;
        //_camera = new FreeCamera(GraphicsDevice.Viewport.AspectRatio, new Vector3(300, -100, 500), size);
        DesiredLookAt = Vector3.Zero;
        pos = Vector2.Zero;
        _camera = new OrbitCamera(GraphicsDevice.Viewport.AspectRatio, Vector3.Zero, 800f, 5, 50000);
        
        // Generacion de posiciones de modelos
        _positionGenerator = new PositionGenerator();
        
        var modelos = _trees.GetModelosConPorcentaje(0.60) // Arboles
            .Concat(_rocks.GetModelosConPorcentaje(0.35)) // Rocas
            .Concat(_houses.GetModelosConPorcentaje(0.05)) // Casas
            .ToList();

        _positionGenerator.AgregarPosiciones(modelos);

        // Genero otros puntos para los arbustos
        var arbustos = _bushes.GetModelosConPorcentaje(1.0);
        
        _positionGenerator.AgregarPosiciones(arbustos, 450);
        
        _tank = new Tank(new Vector3(300, 500, 300), 0f, 20f);
        // Configuramos nuestras matrices de la escena.
        //_tank2.CrearObjetoUnico(20f,  0f, new Vector3(-300, 0, 300));
        //_panzer.CrearObjetoUnico(0.25f,  0f, new Vector3(0, 0, 300));
        //_t90.CrearObjetoUnico(0.25f,  180f, new Vector3(300, 37, 300));
        //_houses.CrearObjetos();
        //_trees.CrearObjetos();
        //_rocks.CrearObjetos();
        //_bushes.CrearObjetos();
        
        base.Initialize();
    }

    /// <summary>
    /// Se ejecuta una vez al inicio del juego, inmediatamente después de Initialize.
    /// Carga y configura los recursos esenciales necesarios para el juego, como efectos, texturas, modelos y estructuras de optimización.
    /// Aquí también se puede realizar cualquier preprocesamiento requerido antes del inicio del ciclo del juego.
    /// </summary>
    protected override void LoadContent()
    {
        var bufferPool = new BufferPool();
        _simulation = Simulation.Create(bufferPool, new NarrowPhaseCallbacks(), new PoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));
        
        // Aca es donde deberiamos cargar todos los contenido necesarios antes de iniciar el juego.

        // Cargo un efecto basico propio declarado en el Content pipeline.
        // En el juego no pueden usar BasicEffect de MG, deben usar siempre efectos propios.
        _effect = Content.Load<Effect>(ContentFolderEffects + "Terrain");
        
        // heights
        var terrainHeigthmap = Content.Load<Texture2D>(ContentFolderTextures + "heightmaps/heightmap");
        // basic color
        var terrainColorMap = Content.Load<Texture2D>(ContentFolderTextures + "heightmaps/colormap");
        // blend texture 1
        var terrainGrass = Content.Load<Texture2D>(ContentFolderTextures + "grass");
        // blend texture 2
        var terrainGround = Content.Load<Texture2D>(ContentFolderTextures + "ground");
        
        terrain = new Terrain(GraphicsDevice, terrainHeigthmap, terrainColorMap, terrainGrass, terrainGround, _effect, _simulation);
        
        _tank.CargarModelo("tank/tank", _effect, Content, _simulation, terrain);
        //_tank2.CargarModelo("tank/tank", _effect, Content);
        //_panzer.CargarModelo("panzer/Panzer", _effect, Content);
        //_t90.CargarModelo("t90/T90", _effect, Content);
        //_trees.CargarModelos(_effect, Content);
        //_houses.CargarModelos(_effect, Content);
        //_rocks.CargarModelos(_effect, Content);
        //_bushes.CargarModelos(_effect, Content);

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
        _tank?.Update(gameTime, keyboardState);
        
        // Actualizar simulación física
        float deltaTime = (float)gameTime.ElapsedGameTime.TotalSeconds;
        if (_simulation != null && deltaTime > 0.0f && deltaTime < 0.1f) // Máximo 100ms por frame
        {
            _simulation.Timestep(deltaTime);
        }

        // Capturar Input teclado
        if (Keyboard.GetState().IsKeyDown(Keys.Escape))
        {
            //Salgo del juego.
            Exit();
        }
        
        // Actualizar cámara para seguir al tanque
        if (_tank != null)
        {
            // Usar la posición y rotación del tanque
            var tankPosition = _tank.Position;
            var tankRotation = _tank.Rotation;
            
            var targetHeight = terrain.GetHeightAtPosition(tankPosition.X, tankPosition.Z) + 50f; // Un poco arriba del tanque
            _camera.SetTarget(new Vector3(tankPosition.X, targetHeight, tankPosition.Z));

        
            var X = tankPosition.X;
            var Z = tankPosition.Z;
            var dir = new Vector2(MathF.Cos(tankRotation), MathF.Sin(tankRotation));
        
            DesiredLookAt = new Vector3(X, terrain.GetHeightAtPosition(X, Z), Z);
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

            var tankPos2D = new Vector2(X, Z);
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
        
            //var Position = new Vector3(cameraPos2D.X, DesiredLookAt.Y + H, cameraPos2D.Y);
            //_camera.View = Matrix.CreateLookAt(Position, LookAt, new Vector3(0, 1, 0));
            
            // Actualizar la cámara (maneja el input del mouse)
            _camera.Update(gameTime);

        }

        base.Update(gameTime);
    }

    /// <summary>
    ///     Se llama cada vez que hay que refrescar la pantalla.
    ///     Escribir aqui el codigo referido al renderizado.
    /// </summary>
    protected override void Draw(GameTime gameTime)
    {
        // Aca deberiamos poner toda la logia de renderizado del juego.
        GraphicsDevice.Clear(Color.Black);
        
        // Verificar que el efecto y el terreno no sean nulos antes de dibujar
        if (_effect == null || terrain == null)
            return;

        
        // Para dibujar le modelo necesitamos pasarle informacion que el efecto esta esperando.
        _effect.Parameters["View"].SetValue(_camera.View);
        _effect.Parameters["Projection"].SetValue(_camera.Projection);
        
        //_effect.Parameters["DiffuseColor"].SetValue(Color.ForestGreen.ToVector3());
        //_ground.Draw(GraphicsDevice, _camera.View, _camera.Projection);
        
        var oldRasterizerState = GraphicsDevice.RasterizerState;
        GraphicsDevice.RasterizerState = RasterizerState.CullNone;
        terrain.Draw(Matrix.Identity, _camera.View, _camera.Projection);
        GraphicsDevice.RasterizerState = oldRasterizerState;
        
        //_effect.Parameters["DiffuseColor"].SetValue(new Color(15, 15, 15).ToVector3());
        _tank.Draw();
        
        //_effect.Parameters["DiffuseColor"].SetValue(new Color(0, 39, 77).ToVector3());
        //_panzer.Dibujar();
        
        //_effect.Parameters["DiffuseColor"].SetValue(new Color(95, 96, 98).ToVector3());
        //_t90.Dibujar();
        
        //_trees.Dibujar();
        //_houses.Dibujar();
        //_rocks.Dibujar();
        //_bushes.Dibujar();
        
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
