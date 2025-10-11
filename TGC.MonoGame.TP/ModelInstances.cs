using System;
using System.Collections.Generic;
using System.Linq;
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
    private readonly List<StaticHandle> _handles = [];
    private Color _color = color;
    private Terrain _terrain = terrain;
    private Simulation _simulation = simulation;
    private Effect _effect;
    private float _altura;
    public List<Vector2> Positions { get; set; } = [];

    private const string ContentFolder3D = TGCGame.ContentFolder3D;
    private readonly Random _random = new Random();

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
            float escala = NextFloat(escalaMin, escalaMax); //elegimos la escala al azar en base al min y max

            float yaw = MathHelper.ToRadians(_random.Next(0, 360)); //giramos de manera aletaria el objeto para que sean diferentes
            var alturaMapa = _terrain.GetHeightAtPosition(posicion.X, posicion.Y); // Es Y porque es un Vector2
            Matrix world = Matrix.CreateScale(escala, escala, escala) *
                           Matrix.CreateFromYawPitchRoll(yaw, 0f, 0f) *
                           Matrix.CreateTranslation(posicion.X, altura + alturaMapa, posicion.Y);

            _worlds.Add(world);
        }

        // Me guardo la altura para luego escalar correctamente el RigidBody
        _altura = altura;
    }

    public void CrearRigidBodies(float ancho, float alto, float profundidad)
    {
        foreach (var world in _worlds)
        {
            // Extraer escala, rotacion y traslacion de la world matrix
            world.Decompose(out var scale, out var rotation, out var translation);

            // Escalar dimensiones
            float anchoEscalado  = ancho  * scale.X;
            float altoEscalado = alto * scale.Y;
            float profundidadEscalada  = profundidad  * scale.Z;

            // Crear shape escalado
            var shape = new Box(anchoEscalado, altoEscalado, profundidadEscalada);
            var shapeIndex = _simulation.Shapes.Add(shape);

            // Ajustar posición: subir el centro para que la base quede en el piso
            // Diferencia entre la altura escalada y la altura base
            float offsetY = (altoEscalado / 2f) - _altura;

            // Aplicar offset al centro
            var correctedPos = new System.Numerics.Vector3(
                translation.X,
                translation.Y + offsetY,
                translation.Z
            );
            
            // Crear StaticDescription con el índice del shape
            var desc = new StaticDescription(
                new RigidPose(
                    correctedPos,
                    new System.Numerics.Quaternion(rotation.X, rotation.Y, rotation.Z, rotation.W)
                ),
                shapeIndex
            );

            // Agregar a la simulación
            var handle = _simulation.Statics.Add(desc);
            
            // Guardar referencia para poder borrarlo después
            _handles.Add(handle);
        }
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

    public void Dibujar()
    {
        _effect.Parameters["DiffuseColor"].SetValue(_color.ToVector3());
        
        foreach (var world in _worlds)
        {
            var modelMeshesBaseTransforms = new Matrix[_model.Bones.Count];
            _model.CopyAbsoluteBoneTransformsTo(modelMeshesBaseTransforms);
            foreach (var mesh in _model.Meshes)
            {
                var relativeTransform = modelMeshesBaseTransforms[mesh.ParentBone.Index];
                _effect.Parameters["World"].SetValue(relativeTransform * world);
                mesh.Draw();
            }
        }
    }

    private float NextFloat(float min, float max)
    {
        return (float)(min + (max - min) * _random.NextDouble());
    }
}