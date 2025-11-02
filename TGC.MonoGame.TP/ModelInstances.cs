using System;
using System.Collections.Generic;
using BepuPhysics;
using BepuPhysics.Collidables;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;
using TGC.MonoGame.Samples.Collisions;
using TGC.MonoGame.TP.Viewer.Gizmos;

namespace TGC.MonoGame.TP;

public class ModelInstances(Color color, Terrain terrain, Simulation simulation)
{
    public Model Model;
    private BoundingBox _box;
    public readonly List<Matrix> _worlds = [];
    public readonly List<StaticHandle> Handles = [];
    private Color _color = color;
    private float _altura;
    private Texture2D[] _texturas;
    private bool _usaNormalMapping;
    private Texture2D[] _normalMaps;
    
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
    
    public void CrearObjetos(float altura, float escalaMin, float escalaMax, bool usaNormalMapping, Texture2D[] normalMaps)
    {
        _texturas = new Texture2D[2];
        _usaNormalMapping = usaNormalMapping;
        _normalMaps = normalMaps;
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
        Model = content.Load<Model>(ContentFolder3D + rutaRelativa);
        _box = BoundingVolumesExtensions.CreateAABBFrom(Model);
        
        foreach (var mesh in Model.Meshes)
        {
            foreach (var meshPart in mesh.MeshParts)
            {
                meshPart.Effect = efecto;
            }
        }
        
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
        Model = content.Load<Model>(ContentFolder3D + rutaRelativa);
        _box = BoundingVolumesExtensions.CreateAABBFrom(Model);
       
        // Cargar textura si se proporciona
        if (!string.IsNullOrEmpty(texturaPath))
        {
            _texturas[0] = content.Load<Texture2D>(ContentFolder3D + texturaPath);
        }
        
        // Cargar segunda textura si se proporciona (para hojas)
        if (!string.IsNullOrEmpty(textura2Path))
        {
            _texturas[1] = content.Load<Texture2D>(ContentFolder3D + textura2Path);
        }
        if(rutaRelativa.Contains("tree"))
            Console.WriteLine($"Path: {rutaRelativa}");
        foreach (var mesh in Model.Meshes)
        {
            if(rutaRelativa.Contains("tree"))
                Console.WriteLine($"-Mesh: {mesh.Name}");
            foreach (var meshPart in mesh.MeshParts)
            {
                meshPart.Effect = efecto;
            }
        }
    }

    public void Draw(Effect effect, BoundingFrustum boundingFrustum, Gizmos gizmos, string texto)
    {
        // Configurar todos los parámetros del shader ANTES de asignar a mesh parts
        
        effect.Parameters["TintColor"]?.SetValue(_color.ToVector4());
        effect.Parameters["UseTexture"]?.SetValue(true);
        effect.Parameters["ModelTexture"].SetValue(_texturas[0]);

        foreach (var world in _worlds)
        {
            //Continuar si no esta dentro del frustum
            if (!EsVisible(world, boundingFrustum))
                continue;
            
            var modelMeshesBaseTransforms = new Matrix[Model.Bones.Count];
            Model.CopyAbsoluteBoneTransformsTo(modelMeshesBaseTransforms);

            SetShader(effect);
            
            foreach (var mesh in Model.Meshes)
            {
                if (texto.Equals("Arbol"))
                    ElegirTexturaYNormal(mesh, effect);
                
                var relativeTransform = modelMeshesBaseTransforms[mesh.ParentBone.Index];
                effect.Parameters["World"]?.SetValue(relativeTransform * world);
                
                foreach (var meshPart in mesh.MeshParts)
                {
                    meshPart.Effect = effect;
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

    public bool EsVisible(Matrix world, BoundingFrustum boundingFrustum)
    {
        var corners = _box.GetCorners();
        for (int i = 0; i < corners.Length; i++)
            corners[i] = Vector3.Transform(corners[i], world);

        var boundingBox = BoundingBox.CreateFromPoints(corners);
        return boundingFrustum.Intersects(boundingBox);
    }

    private void SetShader(Effect effect)
    {
        
        if (_usaNormalMapping)
        {
            effect.CurrentTechnique = effect.Techniques["NormalMapping"];
            effect.Parameters["NormalTexture"].SetValue(_normalMaps[0]);
        }
        else
        {
            effect.CurrentTechnique = effect.Techniques["BasicColorDrawing"];
        }
    }

    private void ElegirTexturaYNormal(ModelMesh mesh, Effect effect)
    {
        if (mesh.Name.Contains("Plane") || mesh.Name.Contains("leaves") || mesh.Name.Contains("polySurface1.001"))
        {
            effect.Parameters["ModelTexture"].SetValue(_texturas[1]);
            if(mesh.Name.Contains("Plane"))
                effect.Parameters["NormalTexture"].SetValue(_normalMaps[0]);
        }
        else if(mesh.Name.Contains("Trunk") || mesh.Name.Contains("Branch"))
        {
            effect.CurrentTechnique = effect.Techniques["NormalMapping"];
            effect.Parameters["NormalTexture"].SetValue(_normalMaps[1]);
            effect.Parameters["ModelTexture"].SetValue(_texturas[0]);
        }
        else
        {
            effect.CurrentTechnique = effect.Techniques["BasicColorDrawing"];
            effect.Parameters["ModelTexture"].SetValue(_texturas[0]);
        }
    }
}

