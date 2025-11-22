using System;
using System.Collections.Generic;
using BepuPhysics;
using BepuPhysics.Collidables;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;
using TGC.MonoGame.TP.Cameras;
using TGC.MonoGame.TP.Fisicas;

namespace TGC.MonoGame.TP.Ambiente.Objetos;

public class ModelInstances(Color color , Simulation simulation, String tipo)
{
    public Model Model;
    private BoundingBox _box;
    public readonly List<Matrix> Worlds = [];
    public List<Matrix> WorldsVisibles = [];
    public readonly List<StaticHandle> Handles = [];
    private Color _color = color;
    private float _altura;
    public readonly Texture2D[] Texturas = new Texture2D[2];
    private bool _usaNormalMapping;
    private Texture2D[] _normalMaps;
    //para el pasto
    private VertexBuffer _vertexBuffer;
    private IndexBuffer _indexBuffer;
    private int _primitiveCount;
    private VertexDeclaration _instanceVertexDeclaration;
    public Effect Effect;
    private String _tipo = tipo;
    
    public List<Vector2> Positions { get; set; } = [];

    private const string ContentFolder3D = TGCGame.ContentFolder3D;
    private readonly Random _random = new();
    
    public float? MaxSlopeDegrees { get; set; }
    public bool AlignToTerrain { get; set; } = true;
    
    public void CrearObjetos(float altura, float escalaMin, float escalaMax, bool usaNormalMapping, Texture2D[] normalMaps, Terrain terrain)
    {
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
            Worlds.Add(world);
        }

        // Me guardo la altura para luego escalar correctamente el RigidBody
        _altura = altura;
    }

    public List<StaticHandle> CrearRigidBodies(float ancho, float alto, float profundidad, float yawInDegrees)
    {
        List<StaticHandle> handles = [];
        
        foreach (var world in Worlds)
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
    
    public VertexBuffer GenerarInstanceBuffer(InstanceData[] instances, GraphicsDevice graphicsDevice)
    {
         return new VertexBuffer(graphicsDevice, _instanceVertexDeclaration, instances.Length, BufferUsage.WriteOnly);
    }
    
    public void CargarModeloPasto(string rutaRelativa, Effect efecto, ContentManager content, string texturaPath)
    {
        Effect = efecto;
        Model = content.Load<Model>(ContentFolder3D + rutaRelativa);
        if (!string.IsNullOrEmpty(texturaPath))
        {
            Texturas[0] = content.Load<Texture2D>(ContentFolder3D + texturaPath);
        }
        var mesh = Model.Meshes[0]; 
        var part = mesh.MeshParts[0];

        _vertexBuffer = part.VertexBuffer;

        _indexBuffer = part.IndexBuffer;

        _primitiveCount = part.PrimitiveCount;
        
        
        VertexElement[] instanceElements =
        [
            new(0, VertexElementFormat.Vector3, VertexElementUsage.Position, 1), // posición
            new(12, VertexElementFormat.Single, VertexElementUsage.TextureCoordinate, 1), // Escala
            new(16, VertexElementFormat.Vector4, VertexElementUsage.TextureCoordinate, 2), // Rotacion
            new(32, VertexElementFormat.Vector4, VertexElementUsage.TextureCoordinate, 3),
            new(48, VertexElementFormat.Vector4, VertexElementUsage.TextureCoordinate, 4),
            new(64, VertexElementFormat.Vector4, VertexElementUsage.TextureCoordinate, 5)
        ]; 
        
        _instanceVertexDeclaration = new VertexDeclaration(instanceElements);
        
    }
    
    public void CargarModelo(string rutaRelativa, Effect efecto, ContentManager content, string texturaPath, string textura2Path = null)
    {
        Model = content.Load<Model>(ContentFolder3D + rutaRelativa);
        _box = BoundingVolumesExtensions.CreateAABBFrom(Model);
       
        // Cargar textura si se proporciona
        if (!string.IsNullOrEmpty(texturaPath))
        {
            Texturas[0] = content.Load<Texture2D>(ContentFolder3D + texturaPath);
        }
        
        // Cargar segunda textura si se proporciona (para hojas)
        if (!string.IsNullOrEmpty(textura2Path))
        {
            Texturas[1] = content.Load<Texture2D>(ContentFolder3D + textura2Path);
        }
        if(rutaRelativa.Contains("tree"))
            foreach (var mesh in Model.Meshes)
            {
                if(rutaRelativa.Contains("tree"))
                    foreach (var meshPart in mesh.MeshParts)
                    {
                        meshPart.Effect = efecto;
                    }
            }
    }

    public void DrawPasto(GraphicsDevice graphicsDevice, Camera camera, VertexBuffer instanceBuffer, 
        float gameTime, Vector3 lightPosition, Vector3 eyePosition, TargetCamera targetLightCamera, float shadowmapSize, RenderTarget2D shadowMap)
    {
        //graphicsDevice.BlendState = BlendState.AlphaBlend;
        graphicsDevice.RasterizerState = RasterizerState.CullNone;
        
        graphicsDevice.SetVertexBuffers(
            new VertexBufferBinding(_vertexBuffer),
            new VertexBufferBinding(instanceBuffer, 0, 1)
        );
        graphicsDevice.Indices = _indexBuffer;

        Effect.Parameters["World"]?.SetValue(Matrix.Identity);
        Effect.Parameters["View"]?.SetValue(camera.View);
        Effect.Parameters["Projection"]?.SetValue(camera.Projection);
        Effect.Parameters["ModelTexture"]?.SetValue(Texturas[0]);
        Effect.Parameters["Time"]?.SetValue(gameTime);
        Effect.Parameters["eyePosition"]?.SetValue(eyePosition);
        Effect.Parameters["lightPosition"]?.SetValue(lightPosition);
        Effect.Parameters["LightViewProjection"]?.SetValue(targetLightCamera.View * targetLightCamera.Projection);
        Effect.Parameters["shadowMapSize"]?.SetValue(Vector2.One * shadowmapSize);
        Effect.Parameters["shadowMap"]?.SetValue(shadowMap);
        
        
        // Configurar parámetros de niebla
        Effect.Parameters["FogColor"]?.SetValue(Color.CornflowerBlue.ToVector3());
        Effect.Parameters["FogStart"]?.SetValue(2500f);
        Effect.Parameters["FogEnd"]?.SetValue(3000f);

        foreach (EffectPass pass in Effect.CurrentTechnique.Passes)
        {
            pass.Apply();
            graphicsDevice.DrawInstancedPrimitives(
                PrimitiveType.TriangleList,
                0,
                0,
                _primitiveCount,
                instanceBuffer.VertexCount
            );
        }
        graphicsDevice.BlendState = BlendState.Opaque;
        graphicsDevice.RasterizerState = RasterizerState.CullCounterClockwise;    
    }

    public void DrawPastoShadow(GraphicsDevice graphicsDevice, VertexBuffer instanceBuffer, float time, Effect effect)
    {
        graphicsDevice.RasterizerState = RasterizerState.CullNone;
        
        graphicsDevice.SetVertexBuffers(
            new VertexBufferBinding(_vertexBuffer),
            new VertexBufferBinding(instanceBuffer, 0, 1)
        );
        graphicsDevice.Indices = _indexBuffer;
        
        effect.Parameters["Time"]?.SetValue(time);
        effect.Parameters["checkInvisible"]?.SetValue(true);
        effect.Parameters["ModelTexture"]?.SetValue(Texturas[0]);
        effect.Parameters["World"].SetValue(Matrix.Identity);
        
        var modelMeshesBaseTransforms = new Matrix[Model.Bones.Count];
        Model.CopyAbsoluteBoneTransformsTo(modelMeshesBaseTransforms);

        foreach (EffectPass pass in effect.CurrentTechnique.Passes)
        {
            pass.Apply();
            graphicsDevice.DrawInstancedPrimitives(
                PrimitiveType.TriangleList,
                0,
                0,
                _primitiveCount,
                instanceBuffer.VertexCount
            );
        }
        effect.Parameters["checkInvisible"]?.SetValue(false);
        graphicsDevice.BlendState = BlendState.Opaque;
        graphicsDevice.RasterizerState = RasterizerState.CullCounterClockwise;    
    }
    
    public void Draw(Effect effect, BoundingFrustum boundingFrustum, bool usarNormalMapping, float time)
    {
        // Configurar todos los parámetros del shader ANTES de asignar a mesh parts
        
        effect.Parameters["TintColor"]?.SetValue(_color.ToVector4());
        effect.Parameters["UseTexture"]?.SetValue(true);
        if(_tipo.Equals("Poste de luz"))
            effect.Parameters["UseTexture"]?.SetValue(false);
        effect.Parameters["ModelTexture"]?.SetValue(Texturas[0]);

        foreach (var world in WorldsVisibles)
        {
            //Continuar si no esta dentro del frustum
            if (!EsVisible(world, boundingFrustum))
                continue;
            
            var modelMeshesBaseTransforms = new Matrix[Model.Bones.Count];
            Model.CopyAbsoluteBoneTransformsTo(modelMeshesBaseTransforms);

            if (usarNormalMapping)
            { SetTechnique(effect);  }
            else 
            { effect.CurrentTechnique = effect.Techniques["BasicColorDrawing"]; }
            
            foreach (var mesh in Model.Meshes)
            {
                if (_tipo.Equals("Arbol"))
                {
                    ElegirTexturaYNormal(mesh, effect, usarNormalMapping);
                    effect.Parameters["Time"]?.SetValue(time);
                }
                
                var relativeTransform = modelMeshesBaseTransforms[mesh.ParentBone.Index];
                effect.Parameters["World"]?.SetValue(relativeTransform * world);
                
                foreach (var meshPart in mesh.MeshParts)
                {
                    meshPart.Effect = effect;
                }

                mesh.Draw();
            }
            effect.Parameters["Sway"]?.SetValue(false);
        }
    }

    public void DestruirInstancia(StaticHandle handle)
    {
        var index = Handles.IndexOf(handle);

        if (simulation.Statics.StaticExists(handle))
        {
            simulation.Statics.Remove(handle);
        }
        
        Handles.Remove(handle);
        Worlds.RemoveAt(index);
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

    private void SetTechnique(Effect effect)
    {
        
        if (_usaNormalMapping)
        {
            effect.CurrentTechnique = effect.Techniques["NormalMapping"];
            effect.Parameters["NormalTexture"]?.SetValue(_normalMaps[0]);
        }
        else
        {
            effect.CurrentTechnique = effect.Techniques["BasicColorDrawing"];
        }
    }

    private void ElegirTexturaYNormal(ModelMesh mesh, Effect effect, bool usarNormalMapping)
    {
        if (mesh.Name.Contains("Plane") ||  mesh.Name.Contains("leaves"))
        {
            effect.Parameters["ModelTexture"]?.SetValue(Texturas[1]);
            effect.Parameters["Sway"]?.SetValue(true);
            if(usarNormalMapping)
                effect.Parameters["NormalTexture"]?.SetValue(_normalMaps[0]);
        }
        else 
        {
            
            effect.Parameters["Sway"]?.SetValue(false);
            effect.Parameters["ModelTexture"]?.SetValue(Texturas[0]);
            
            if (usarNormalMapping)
            {
                effect.CurrentTechnique = effect.Techniques["NormalMapping"];
                effect.Parameters["NormalTexture"]?.SetValue(_normalMaps[1]);
            }
        }
        
    }
}

public struct InstanceData
{
    public Vector3 Position;
    public float Scale;
    public Matrix Rotation;
}
