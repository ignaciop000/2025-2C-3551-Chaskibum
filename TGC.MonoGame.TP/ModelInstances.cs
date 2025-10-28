using System;
using System.Collections.Generic;
using BepuPhysics;
using BepuPhysics.Collidables;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class ModelInstances(Color color, Terrain terrain, Simulation simulation)
{
    private Model _model;
    private readonly List<Matrix> _worlds = [];
    public readonly List<StaticHandle> Handles = [];
    private Color _color = color;
    private Effect _effect;
    private float _altura;
    private Texture2D _texture;
    private Texture2D _texture2; // Segunda textura (para hojas en árboles)
    private int _modelIndex = 0; // Índice del modelo (para TreeType: 0, 1, 2)
    
    public List<Vector2> Positions { get; set; } = [];

    private const string ContentFolder3D = TGCGame.ContentFolder3D;
    private readonly Random _random = new();
    
    public float? MaxSlopeDegrees { get; set; }
    public bool AlignToTerrain { get; set; } = true;

    public void CrearObjetoUnico(float escala, float yawInDegrees, Vector3 position)
    { 
        Matrix world = Matrix.CreateScale(escala, escala, escala) * 
                       Matrix.CreateFromYawPitchRoll(MathHelper.ToRadians(yawInDegrees), 0f, 0f) * 
                       Matrix.CreateTranslation(position.X, position.Y, position.Z);

        _worlds.Add(world);
    }
    
    public void CrearObjetos(float altura, float escalaMin, float escalaMax)
    {
        foreach (var posicion in Positions)
        {
            //Filtro por inclinación
            var slopeDeg = terrain.GetSlopeDegreesAt(posicion.X, posicion.Y);
            if (MaxSlopeDegrees.HasValue && slopeDeg > MaxSlopeDegrees.Value)
                continue;

            //Altura en el mapa
            var alturaMapa = terrain.GetHeightAtPosition(posicion.X, posicion.Y);
            var escala = NextFloat(escalaMin, escalaMax); //elegimos la escala al azar en base al min y max
            var yaw = MathHelper.ToRadians(_random.Next(0, 360)); //giramos de manera aleatoria el objeto para que sean diferentes

            Matrix world;
            if (AlignToTerrain)
            {
                // Alinear al plano del terreno
                var pos3 = new Vector3(posicion.X, altura + alturaMapa, posicion.Y);
                var q = terrain.CalculateRotation(pos3, yaw);
                world = Matrix.CreateScale(escala) * Matrix.CreateFromQuaternion(q) * Matrix.CreateTranslation(pos3);
            }
            else
            {
                // Mantener vertical (solo yaw)
                world = Matrix.CreateScale(escala) *
                        Matrix.CreateFromYawPitchRoll(yaw, 0f, 0f) *
                        Matrix.CreateTranslation(posicion.X, altura + alturaMapa, posicion.Y);
            }
            _worlds.Add(world);
        }

        // Me guardo la altura para luego escalar correctamente el RigidBody
        _altura = altura;
    }

    public List<StaticHandle> CrearRigidBodies(float ancho, float alto, float profundidad, float yawInDegrees)
    {
        List<StaticHandle> handles = [];
        
        foreach (var world in _worlds)
        {
            world.Decompose(out var scale, out var rotation, out var translation);

            float anchoEscalado = ancho * scale.X;
            float altoEscalado = alto * scale.Y;
            float profundidadEscalada = profundidad * scale.Z;

            var shape = new Box(anchoEscalado, altoEscalado, profundidadEscalada);
            var shapeIndex = simulation.Shapes.Add(shape);

            float offsetY = (altoEscalado / 2f) - _altura;

            var correctedPos = new System.Numerics.Vector3(
                translation.X,
                translation.Y + offsetY,
                translation.Z
            );

            // Aplicar rotación extra en yaw
            var yawRotation = Quaternion.CreateFromAxisAngle(Vector3.Up, MathHelper.ToRadians(yawInDegrees));
            var finalRotation = rotation * yawRotation;

            var desc = new StaticDescription(
                new RigidPose(correctedPos, new System.Numerics.Quaternion(finalRotation.X, finalRotation.Y, finalRotation.Z, finalRotation.W)),
                shapeIndex
            );

            var handle = simulation.Statics.Add(desc);
            Handles.Add(handle);
            handles.Add(handle);
        }
        
        return handles;
    }

    public void CargarModelo(string rutaRelativa, Effect efecto, ContentManager content)
    {
        _model = content.Load<Model>(ContentFolder3D + rutaRelativa);

        foreach (var mesh in _model.Meshes)
        {
            foreach (var meshPart in mesh.MeshParts)
            {
                meshPart.Effect = efecto;
            }
        }

        // Me lo guardo para usar en el dibujado
        _effect = efecto;
    }
    
    public void CargarModelo(string rutaRelativa, Effect efecto, ContentManager content, string texturaPath)
    {
        CargarModelo(rutaRelativa, efecto, content, texturaPath, null, 0);
    }
    
    public void CargarModelo(string rutaRelativa, Effect efecto, ContentManager content, string texturaPath, string textura2Path)
    {
        CargarModelo(rutaRelativa, efecto, content, texturaPath, textura2Path, 0);
    }
    
    public void CargarModelo(string rutaRelativa, Effect efecto, ContentManager content, string texturaPath, string textura2Path, int modelIndex)
    {
        _model = content.Load<Model>(ContentFolder3D + rutaRelativa);
        _modelIndex = modelIndex; // Guardar el índice del modelo

        // Cargar textura si se proporciona
        if (!string.IsNullOrEmpty(texturaPath))
        {
            _texture = content.Load<Texture2D>(ContentFolder3D + texturaPath);
        }
        
        // Cargar segunda textura si se proporciona (para hojas)
        if (!string.IsNullOrEmpty(textura2Path))
        {
            _texture2 = content.Load<Texture2D>(ContentFolder3D + textura2Path);
        }

        foreach (var mesh in _model.Meshes)
        {
            foreach (var meshPart in mesh.MeshParts)
            {
                meshPart.Effect = efecto;
            }
        }

        // Me lo guardo para usar en el dibujado
        _effect = efecto;
    }

    public void Dibujar(Matrix view, Matrix projection)
    {
        // Configurar todos los parámetros del shader ANTES de asignar a mesh parts
        _effect.Parameters["View"]?.SetValue(view);
        _effect.Parameters["Projection"]?.SetValue(projection);
        _effect.Parameters["TintColor"]?.SetValue(_color.ToVector4());
        
        // Configurar TreeType para el shader de árboles
        _effect.Parameters["TreeType"]?.SetValue(_modelIndex);
        
        // Si hay textura, activarla
        if (_texture != null)
        {
            _effect.Parameters["UseTexture"]?.SetValue(true);
            _effect.Parameters["ModelTexture"]?.SetValue(_texture);
            
            // Para TreeShader: textura de corteza
            _effect.Parameters["BarkTexture"]?.SetValue(_texture);
        }
        else
        {
            _effect.Parameters["UseTexture"]?.SetValue(false);
        }
        
        // Segunda textura (hojas para árboles) - CRUCIAL
        if (_texture2 != null)
        {
            _effect.Parameters["LeavesTexture"]?.SetValue(_texture2);
        }
        
        foreach (var world in _worlds)
        {
            var modelMeshesBaseTransforms = new Matrix[_model.Bones.Count];
            _model.CopyAbsoluteBoneTransformsTo(modelMeshesBaseTransforms);
            foreach (var mesh in _model.Meshes)
            {
                var relativeTransform = modelMeshesBaseTransforms[mesh.ParentBone.Index];
                _effect.Parameters["World"]?.SetValue(relativeTransform * world);
                
                // Aplicar el efecto a cada mesh part para que use los parámetros actualizados
                foreach (var meshPart in mesh.MeshParts)
                {
                    meshPart.Effect = _effect;
                }
                
                mesh.Draw();
            }
        }
    }

    public void DestruirInstancia(StaticHandle handle)
    {
        var index = Handles.IndexOf(handle);
        simulation.Statics.Remove(handle);
        Handles.Remove(handle);
        _worlds.RemoveAt(index);
    }

    private float NextFloat(float min, float max)
    {
        return (float)(min + (max - min) * _random.NextDouble());
    }
}

