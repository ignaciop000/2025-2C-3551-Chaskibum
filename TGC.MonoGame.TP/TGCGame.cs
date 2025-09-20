// LISTA DE TAREAS

// Agregar los 2 modelos de casas (alternar entre los 2, pero muy separadas)
// Buscar modelo de camino de tierra y poner muchos
// Buscar modelo de hangar y poner 2 (uno cerca y otro lejos)
// Buscar otros modelos de arboles y ponerlos
// Buscar modelos de arbustos y ponerlos

using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;

namespace TGC.MonoGame.TP;

/// <summary>
///     Esta es la clase principal del juego.
///     Inicialmente puede ser renombrado o copiado para hacer mas ejemplos chicos, en el caso de copiar para que se
///     ejecute el nuevo ejemplo deben cambiar la clase que ejecuta Program <see cref="Program.Main()" /> linea 10.
/// </summary>
public class TGCGame : Game
{
    public const string ContentFolder3D = "Models/";
    public const string ContentFolderEffects = "Effects/";
    public const string ContentFolderMusic = "Music/";
    public const string ContentFolderSounds = "Sounds/";
    public const string ContentFolderSpriteFonts = "SpriteFonts/";
    public const string ContentFolderTextures = "Textures/";
    
    private readonly GraphicsDeviceManager _graphics;
    private readonly Random _rnd = new Random();
    private Effect _effect;
    private FreeCamera Camera { get; set; }
    
    private Ground Ground { get; set; }
    
    private ModelInstances _tank = new ModelInstances();
    private ModelInstances _panzer = new ModelInstances();
    private ModelInstances _t90 = new ModelInstances();
    private ModelInstances _trees = new ModelInstances();
    
    private Rocks _rocks = new Rocks();

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
        Camera = new FreeCamera(GraphicsDevice.Viewport.AspectRatio, new Vector3(0, 100, 150), size);

        // Generacion de posiciones de modelos
        var positionGenerator = new PositionGenerator();
        var modelosRocas = _rocks._rocks
            .Select(m => (modelo: m, valor: 0.045f))
            .ToList();

        List<(ModelInstances modelo, float porcentaje)> modelos = [(_trees, 0.505f)];
        modelos.AddRange(modelosRocas);
        
        positionGenerator.AgregarPosiciones(modelos);
        
        // Configuramos nuestras matrices de la escena.
        _tank.CrearObjetoUnico(Matrix.CreateScale(20f, 20f, 20f) * Matrix.CreateTranslation(0, 0, 0));
        _panzer.CrearObjetoUnico(Matrix.CreateScale(0.25f, 0.25f, 0.25f) * Matrix.CreateTranslation(200, 0, 0));
        _t90.CrearObjetoUnico(Matrix.CreateScale(0.25f, 0.25f, 0.25f) * Matrix.CreateTranslation(-200, 37, 0));
        
        _trees.CrearObjetos(0f, 25f, 50f);
        _rocks.CrearObjetos();
        
        base.Initialize();
    }

    /// <summary>
    ///     Se llama una sola vez, al principio cuando se ejecuta el ejemplo, despues de Initialize.
    ///     Escribir aqui el codigo de inicializacion: cargar modelos, texturas, estructuras de optimizacion, el procesamiento
    ///     que podemos pre calcular para nuestro juego.
    /// </summary>
    protected override void LoadContent()
    {
        // Aca es donde deberiamos cargar todos los contenido necesarios antes de iniciar el juego.

        // Cargo un efecto basico propio declarado en el Content pipeline.
        // En el juego no pueden usar BasicEffect de MG, deben usar siempre efectos propios.
        _effect = Content.Load<Effect>(ContentFolderEffects + "BasicShader");
        
        Ground = new Ground(GraphicsDevice)
        {
            Effect = _effect
        };
        
        _tank.CargarModelo("tank/tank", _effect, Content);
        _panzer.CargarModelo("panzer/Panzer", _effect, Content);
        _t90.CargarModelo("t90/T90", _effect, Content);
        _trees.CargarModelo("tree/Tree", _effect, Content);
        
        _rocks.CargarModelos(_effect, Content);

        base.LoadContent();
    }

    /// <summary>
    ///     Se llama en cada frame.
    ///     Se debe escribir toda la logica de computo del modelo, asi como tambien verificar entradas del usuario y reacciones
    ///     ante ellas.
    /// </summary>
    protected override void Update(GameTime gameTime)
    {
        // Aca deberiamos poner toda la logica de actualizacion del juego.

        Camera.Update(gameTime);
        
        // Capturar Input teclado
        if (Keyboard.GetState().IsKeyDown(Keys.Escape))
        {
            //Salgo del juego.
            Exit();
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
        
        // Para dibujar le modelo necesitamos pasarle informacion que el efecto esta esperando.
        _effect.Parameters["View"].SetValue(Camera.View);
        _effect.Parameters["Projection"].SetValue(Camera.Projection);
        
        _effect.Parameters["DiffuseColor"].SetValue(Color.ForestGreen.ToVector3());
        Ground.Draw(GraphicsDevice, Camera.View, Camera.Projection);
        
        _effect.Parameters["DiffuseColor"].SetValue(new Color(15, 15, 15).ToVector3());
        _tank.Dibujar();
        
        _effect.Parameters["DiffuseColor"].SetValue(new Color(0, 39, 77).ToVector3());
        _panzer.Dibujar();
        
        _effect.Parameters["DiffuseColor"].SetValue(new Color(95, 96, 98).ToVector3());
        _t90.Dibujar();
        
        _effect.Parameters["DiffuseColor"].SetValue(Color.SaddleBrown.ToVector3());
        _trees.Dibujar();
        
        _effect.Parameters["DiffuseColor"].SetValue(Color.Gray.ToVector3());
        _rocks.Dibujar();
        
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
