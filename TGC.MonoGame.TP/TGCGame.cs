// LISTA DE TAREAS

// Usar los otros modelos de rocas (alternar entre los 11)
// Usar el otro modelo de arbol (alternar entre los 2)
// Agregar los 2 modelos de casas (alternar entre los 2, pero muy separadas)
// Buscar modelo de camino de tierra y poner muchos
// Buscar modelo de hangar y poner 2 (uno cerca y otro lejos)

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

    private Random rnd = new Random();
    
    private Effect _effect;
    
    private Model _tankModel;
    private Matrix _tankWorld;
    
    private Model _panzerModel;
    private Matrix _panzerWorld;
    
    private Model _t90Model;
    private Matrix _t90World;

    private Model _rockModel;
    private List<Matrix> rockWorlds = new List<Matrix>();
    
    private Model _treeModel;
    private List<Matrix> treeWorlds = new List<Matrix>();
    
    private FreeCamera Camera { get; set; }
    
    private Ground Ground { get; set; }

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

        // Configuramos nuestras matrices de la escena.
        _tankWorld = Matrix.CreateScale(20f, 20f, 20f) * Matrix.CreateTranslation(0, 0, 0);
        _panzerWorld = Matrix.CreateScale(0.25f, 0.25f, 0.25f) * Matrix.CreateTranslation(200, 0, 0);
        _t90World = Matrix.CreateScale(0.25f, 0.25f, 0.25f) * Matrix.CreateTranslation(-200, 37, 0);
        crearObjetos(1000, treeWorlds, 25f, 0f);
        crearObjetos(500, rockWorlds, 0.1f, 5f);
        
        base.Initialize();
    }

    private void crearObjetos(int cantidad, List<Matrix> lista, float escala, float altura)
    {
        for (int i = 0; i < cantidad; i++)
        {
            float x = rnd.Next(-5000, 5000);
            float z = rnd.Next(-5000, 5000);
            
            float yaw = MathHelper.ToRadians(rnd.Next(0, 360));

            Matrix world = Matrix.CreateScale(escala, escala, escala) *
                           Matrix.CreateFromYawPitchRoll(yaw, 0f, 0f) *
                           Matrix.CreateTranslation(x, altura, z);

            lista.Add(world);
        } 
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

        _tankModel = CargarModeloConEfecto("tank/tank", _effect);
        _panzerModel = CargarModeloConEfecto("panzer/Panzer", _effect);
        _t90Model = CargarModeloConEfecto("t90/T90", _effect);
        _treeModel = CargarModeloConEfecto("tree/Tree", _effect);
        _rockModel = CargarModeloConEfecto("rocks/Rock8", _effect);
        
        Ground = new Ground(GraphicsDevice);
        Ground.Effect = _effect;

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
        DrawModeloConEstructura(_tankModel, _tankWorld);
        
        _effect.Parameters["DiffuseColor"].SetValue(new Color(0, 39, 77).ToVector3());
        DrawModeloConEstructura(_panzerModel, _panzerWorld);
        
        _effect.Parameters["DiffuseColor"].SetValue(new Color(95, 96, 98).ToVector3());
        DrawModeloConEstructura(_t90Model, _t90World);
        
        _effect.Parameters["DiffuseColor"].SetValue(Color.SaddleBrown.ToVector3());
        foreach (var world in treeWorlds)
        {
            Model[] opciones = { _treeModel, _treeModel};
            int index = rnd.Next(opciones.Length);
            DrawModeloConEstructura(opciones[index], world);
        }
        
        _effect.Parameters["DiffuseColor"].SetValue(Color.Gray.ToVector3());
        foreach (var world in rockWorlds)
        {
            DrawModeloConEstructura(_rockModel, world);
        }
        
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
    
    // Funciones auxiliares
    private Model CargarModeloConEfecto(String rutaRelativa, Effect efecto)
    {
        var modelo = Content.Load<Model>(ContentFolder3D + rutaRelativa);
        
        foreach (var mesh in modelo.Meshes)
        {
            foreach (var meshPart in mesh.MeshParts)
            {
                meshPart.Effect = efecto;
            }
        }

        return modelo;
    }

    private void DrawModeloConEstructura(Model modelo, Matrix world)
    {
        var modelMeshesBaseTransforms = new Matrix[modelo.Bones.Count];
        modelo.CopyAbsoluteBoneTransformsTo(modelMeshesBaseTransforms);
        foreach (var mesh in modelo.Meshes)
        {
            var relativeTransform = modelMeshesBaseTransforms[mesh.ParentBone.Index];
            _effect.Parameters["World"].SetValue(relativeTransform * world);
            mesh.Draw();
        }
    }
}