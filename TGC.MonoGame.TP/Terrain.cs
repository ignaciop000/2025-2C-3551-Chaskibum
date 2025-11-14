using System;
using System.Collections.Generic;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities.Memory;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class Terrain
{
    private readonly Effect _effect;
    public float ScaleXz = 1;
    public float ScaleY = 1;
    public List<TerrainChunk> Chunks;
    private readonly Texture2D _colorMapTexture;
    private readonly Texture2D _terrainTexture;
    private readonly Texture2D _terrainTexture2;
    private readonly Texture2D _normalMap;
    public StaticHandle Handle;
    public Vector3 LightPosition;
    public Vector3 EyePosition;
    private PositionGenerator _positionGenerator;
    public Color[,] ColorMap;
    
    /// <summary>
    /// Datos del mapa de altura (Heightmap) utilizados para representar la topografía de un terreno.
    /// Este array bidimensional almacena los valores de altura para cada punto del terreno,
    /// donde cada valor representa la elevación relativa en ese punto.
    /// </summary>
    public int[,] HeightmapData { get; private set; }

    /// <summary>
    /// Representa el centro del terreno en coordenadas espaciales.
    /// Este punto central se utiliza como referencia para posicionar y ajustar el terreno en el espacio tridimensional.
    /// </summary>
    public Vector3 Center { get; private set; }


    /// Representa una clase para modelar y renderizar un terreno en base a un mapa de alturas.
    /// Contiene funcionalidad para cargar un mapa de alturas, mapas de textura y generar la estructura del terreno.
    public Terrain(GraphicsDevice graphicsDevice, Texture2D heightMap, Texture2D colorMap,
        Texture2D diffuseMap, Texture2D diffuseMap2, Texture2D normalMap, Effect effect,
        Simulation simulation, float scaleXZ, Vector3 eyePos, PositionGenerator positionGenerator, 
        Texture2D spawnMap, Pasto pasto)
    {
        Chunks = [];
        //Shader
        _effect = effect;
        ColorMap = LoadColorMap(spawnMap);
        // cargo el heightmap
        // textura con el color Map
        _colorMapTexture = colorMap;
        // diffuse maps auxiliares
        _terrainTexture = diffuseMap;
        _terrainTexture2 = diffuseMap2;
        _normalMap = normalMap;
        LightPosition =  new Vector3(1000, 1000, 1000);
        EyePosition = eyePos;
        _positionGenerator = positionGenerator;
        LoadHeightmap(graphicsDevice, heightMap, scaleXZ, 1, Vector3.Zero, simulation, pasto);
    }

    /// Genera la estructura del terreno cargando un mapa de alturas y creando los vértices necesarios.
    /// <param name="graphicsDevice">El dispositivo gráfico utilizado para generar el terreno.</param>
    /// <param name="heightmap">La textura que representa el mapa de alturas del terreno.</param>
    /// <param name="scaleXZ">La escala horizontal utilizada para ajustar el tamaño del terreno en los ejes X y Z.</param>
    /// <param name="scaleY">La escala vertical utilizada para ajustar la altura del terreno en el eje Y.</param>
    /// <param name="center">El punto central del terreno en coordenadas tridimensionales.</param>
    /// <param name="simulation">La simulación de física donde se agregará el mesh de colisión.</param>
    private void LoadHeightmap(GraphicsDevice graphicsDevice, Texture2D heightmap, float scaleXZ, float scaleY,
        Vector3 center, Simulation simulation, Pasto pasto)
    {
        
        ScaleXz = scaleXZ;
        ScaleY = scaleY;
        const float txScale = 1;

        //cargar heightmap
        HeightmapData = LoadHeightMap(heightmap);
        
        var width = HeightmapData.GetLength(0);
        
        int chunkSize = 64;
        var length = HeightmapData.GetLength(1);
        
        // Ajuste del centro
        center.X = center.X * scaleXZ - width / 2f * scaleXZ;
        center.Y = center.Y * scaleY;
        center.Z = center.Z * scaleXZ - length / 2f * scaleXZ;
        Center = center;

        for (int chunkX = 0; chunkX < width; chunkX += chunkSize)
        {
            for (int chunkZ = 0; chunkZ < length; chunkZ += chunkSize)
            {
                int localWidth = Math.Min(chunkSize + 1, width - chunkX);
                int localLength = Math.Min(chunkSize + 1, width - chunkZ);
                
                var localVertices = new VertexPositionNormalTexture[localWidth * localLength];
                var localIndices = new int[(localWidth - 1) * (localLength - 1) * 6];
                
                // Generar vértices
                for (int i = 0; i < localWidth; i++)
                {
                    for (int j = 0; j < localLength; j++)
                    {
                        int globalX = chunkX + i;
                        int globalZ = chunkZ + j;
                        
                        var pos = new Vector3(center.X + globalX * scaleXZ, 
                                              center.Y + HeightmapData[globalX, globalZ] * scaleY, 
                                              center.Z + globalZ * scaleXZ);
                        
                        var tex = new Vector2(globalX / (float)(width - 1), 
                                                     globalZ / (float)(length - 1)) * txScale;
                        localVertices[i * localLength + j] = new VertexPositionNormalTexture(pos, Vector3.Zero, tex);
                    }
                }
                
                // Generar índices y normales
                int idx = 0;
                for (int i = 0; i < localWidth - 1; i++)
                {
                    for (int j = 0; j < localLength - 1; j++)
                    {
                        int v1 = i * localLength + j;
                        int v2 = i * localLength + (j + 1);
                        int v3 = (i + 1) * localLength + j;
                        int v4 = (i + 1) * localLength + (j + 1);

                        // Triángulo 1: v1, v2, v4
                        var n0 = Vector3.Normalize(Vector3.Cross(localVertices[v2].Position - localVertices[v1].Position, 
                                                                        localVertices[v4].Position - localVertices[v1].Position));
                        localVertices[v1].Normal += n0;
                        localVertices[v2].Normal += n0;
                        localVertices[v4].Normal += n0;

                        localIndices[idx++] = v1;
                        localIndices[idx++] = v4;
                        localIndices[idx++] = v2;

                        // Triángulo 2: v1, v4, v3
                        var n1 = Vector3.Normalize(Vector3.Cross(localVertices[v4].Position - localVertices[v1].Position, 
                                                                        localVertices[v3].Position - localVertices[v1].Position));
                        localVertices[v1].Normal += n1;
                        localVertices[v4].Normal += n1;
                        localVertices[v3].Normal += n1;

                        localIndices[idx++] = v1;
                        localIndices[idx++] = v3;
                        localIndices[idx++] = v4;
                    }
                }
                
                // Normalizar normales
                for (int i = 0; i < localVertices.Length; i++)
                    localVertices[i].Normal = Vector3.Normalize(localVertices[i].Normal);

                // Crear buffers
                var _vbChunk = new VertexBuffer(graphicsDevice, VertexPositionNormalTexture.VertexDeclaration, localVertices.Length, BufferUsage.WriteOnly);
                _vbChunk.SetData(localVertices);

                var _ibChunk = new IndexBuffer(graphicsDevice, IndexElementSize.ThirtyTwoBits, localIndices.Length, BufferUsage.WriteOnly);
                _ibChunk.SetData(localIndices);
                
                // Calcular bounding box del chunk
                var min = new Vector3(float.MaxValue);
                var max = new Vector3(float.MinValue);
                foreach (var v in localVertices)
                {
                    min = Vector3.Min(min, v.Position);
                    max = Vector3.Max(max, v.Position);
                }
                
                var grassPoints = _positionGenerator.GenerarPuntosPasto( 
                    100f, 
                    ColorMap, 
                    scaleXZ, 
                    chunkX, 
                    chunkZ, 
                    localWidth,
                    localLength,
                    center,
                    width,
                    length,
                    30
                );
                
                var random = new Random();
                var instances = new InstanceData[grassPoints.Count];
                
                for (int i = 0; i < instances.Length; i++)
                {
                    var posX = grassPoints[i].X;
                    var posZ = grassPoints[i].Y;
                    var posY = GetHeightAtPosition(posX, posZ);
                    instances[i].Position = new Vector3(posX, posY, posZ);
                    instances[i].Scale = random.Next(11,16); 
                    instances[i].Rotation = GetRotationMatrix(GetNormalAtPosition(posX, posZ));
                }
                
                var instanceBuffer = pasto.Models[0].GenerarInstanceBuffer(instances, graphicsDevice);
                instanceBuffer.SetData(instances);
                
                var chunk = new TerrainChunk
                {
                    VertexBuffer = _vbChunk,
                    IndexBuffer = _ibChunk,
                    BoundingBox = new BoundingBox(min, max),
                    InstanceBuffer = instanceBuffer
                };
                
                Chunks.Add(chunk);
            }
        }
        Console.WriteLine($"Chunks generados: {Chunks.Count}");

        // Crear colisión física
        CreatePhysicsCollision(simulation);
    }

    /// <summary>
    /// Crea el mesh de colisión para la física del terreno usando un heightfield más simple
    /// </summary>
    /// <param name="simulation">La simulación de física</param>
    private void CreatePhysicsCollision(Simulation simulation)
    {
        int width = HeightmapData.GetLength(0);
        int length = HeightmapData.GetLength(1);

        // Crear triángulos del terreno
        var triangleCount = (width - 1) * (length - 1) * 2;
        simulation.BufferPool.Take(triangleCount, out Buffer<Triangle> triangles);

        int index = 0;
        for (int z = 0; z < length - 1; z++)
        {
            for (int x = 0; x < width - 1; x++)
            {
                // Alturas del heightmap
                float h11 = HeightmapData[x, z] * ScaleY;
                float h12 = HeightmapData[x, z + 1] * ScaleY;
                float h21 = HeightmapData[x + 1, z] * ScaleY;
                float h22 = HeightmapData[x + 1, z + 1] * ScaleY;

                // Posiciones de los 4 vértices de la celda
                var v00 = new System.Numerics.Vector3(Center.X + x * ScaleXz, Center.Y + h11, Center.Z + z * ScaleXz);
                var v01 = new System.Numerics.Vector3(Center.X + x * ScaleXz, Center.Y + h12,
                    Center.Z + (z + 1) * ScaleXz);
                var v10 = new System.Numerics.Vector3(Center.X + (x + 1) * ScaleXz, Center.Y + h21,
                    Center.Z + z * ScaleXz);
                var v11 = new System.Numerics.Vector3(Center.X + (x + 1) * ScaleXz, Center.Y + h22,
                    Center.Z + (z + 1) * ScaleXz);

                // Crear los triángulos de la celda
                triangles[index++] = new Triangle(v00, v10, v01);
                triangles[index++] = new Triangle(v10, v11, v01);
            }
        }

        // Crear el mesh con escala unitaria
        var mesh = new Mesh(triangles, new System.Numerics.Vector3(1, 1, 1), simulation.BufferPool);

        // Agregar el mesh al simulador
        var meshHandle = simulation.Shapes.Add(mesh);

        // Crear una descripción estática (el terreno no se mueve)
        var staticDescription = new StaticDescription(
            new System.Numerics.Vector3(0, 0, 0),
            System.Numerics.Quaternion.Identity,
            meshHandle
        );

        Handle = simulation.Statics.Add(staticDescription);

        // Liberar el buffer (ya fue copiado internamente)
        simulation.BufferPool.Return(ref triangles);
    }
    
    /// Carga un mapa de alturas desde una textura para ser utilizado como datos de elevación.
    /// <param name="texture">La textura que contiene el mapa de alturas.</param>
    /// <returns>Una matriz bidimensional de enteros que representa los valores de altura extraídos de la textura.</returns>
    private int[,] LoadHeightMap(Texture2D texture)
    {
        var width = texture.Width;
        var height = texture.Height;
        var rawData = new Color[width * height];
        texture.GetData(rawData);
        var heightmap = new int[width, height];

        for (var i = 0; i < width; i++)
        {
            for (var j = 0; j < height; j++)
            {
                //(j, i) invertido para primero barrer filas y despues columnas
                var pixel = rawData[j * texture.Width + i];
                var intensity = pixel.R * 0.299f + pixel.G * 0.587f + pixel.B * 0.114f;
                heightmap[i, j] = (int)intensity;
            }
        }

        return heightmap;
    }
    
    public Color[,] LoadColorMap(Texture2D texture)
    {
        var width = texture.Width;
        var height = texture.Height;
        var rawData = new Color[width * height];
        texture.GetData(rawData);
        var colorMap = new Color[width, height];

        for (var i = 0; i < width; i++)
        {
            for (var j = 0; j < height; j++)
            {
                //(j, i) invertido para primero barrer filas y despues columnas
                colorMap[i, j]  = rawData[j * texture.Width + i];
            }
        }

        return colorMap;
    }

    /// Dibuja el terreno en la pantalla utilizando los parámetros proporcionados para las matrices de transformación.
    /// Configura el efecto y los recursos necesarios para el renderizado, aplicando cada paso del shader.
    /// <param name="world">Matriz de transformación para coordenadas del mundo.</param>
    /// <param name="view">Matriz de vista de la cámara para determinar cómo se observa la escena.</param>
    /// <param name="projection">Matriz de proyección utilizada para la perspectiva 3D.</param>
        public void Draw(Matrix world, Matrix view, Matrix projection, BoundingFrustum boundingFrustum, Pasto pasto, float time)
        {
            var graphicsDevice = _effect.GraphicsDevice;
            
            _effect.Parameters["World"]?.SetValue(world);
            _effect.Parameters["View"]?.SetValue(view);
            _effect.Parameters["Projection"]?.SetValue(projection);
            _effect.Parameters["texColorMap"]?.SetValue(_colorMapTexture);
            _effect.Parameters["texDiffuseMap"]?.SetValue(_terrainTexture);
            _effect.Parameters["texDiffuseMap2"]?.SetValue(_terrainTexture2);
            _effect.Parameters["NormalTexture"]?.SetValue(_normalMap);
            _effect.Parameters["lightPosition"]?.SetValue(LightPosition);
            _effect.Parameters["eyePosition"]?.SetValue(EyePosition);

            _effect.Parameters["WorldViewProjection"]?.SetValue(world * view * projection);
            _effect.Parameters["InverseTransposeWorld"]?.SetValue(Matrix.Transpose(Matrix.Invert(world)));

            foreach (var chunk in Chunks)
            {
                if (boundingFrustum.Intersects(chunk.BoundingBox))
                {
                    pasto.DrawPasto(graphicsDevice,view,projection, chunk.InstanceBuffer, time);
                    
                    graphicsDevice.SetVertexBuffer(chunk.VertexBuffer);
                    graphicsDevice.Indices = chunk.IndexBuffer;

                    foreach (var pass in _effect.CurrentTechnique.Passes)
                    {
                        pass.Apply();
                        graphicsDevice.DrawIndexedPrimitives(PrimitiveType.TriangleList, 0, 0, chunk.IndexBuffer.IndexCount / 3);
                    }
                }
            }
        }

    /// <summary>
    /// Obtiene la altura del terreno en una posición específica (X, Z)
    /// </summary>
    /// <param name="worldX">Posición X en coordenadas del mundo</param>
    /// <param name="worldZ">Posición Z en coordenadas del mundo</param>
    /// <returns>La altura Y del terreno en esa posición</returns>
    public float GetHeightAtPosition(float worldX, float worldZ)
    {
        if (HeightmapData == null) return 0f;

        var width = HeightmapData.GetLength(0);
        var height = HeightmapData.GetLength(1);

        // Convertir coordenadas del mundo a coordenadas del heightmap
        float mapX = (worldX - Center.X) / ScaleXz;
        float mapZ = (worldZ - Center.Z) / ScaleXz;

        // Verificar límites
        if (mapX < 0 || mapX >= width - 1 || mapZ < 0 || mapZ >= height - 1)
            return 0f;

        // Obtener las coordenadas de la celda
        int x1 = (int)MathF.Floor(mapX);
        int z1 = (int)MathF.Floor(mapZ);
        int x2 = Math.Min(x1 + 1, width - 1);
        int z2 = Math.Min(z1 + 1, height - 1);

        // Calcular factores de interpolación
        float fx = mapX - x1;
        float fz = mapZ - z1;
        
        int xMax = HeightmapData.GetLength(0);
        int zMax = HeightmapData.GetLength(1);

        bool indicesValidos =
            x1 >= 0 && x1 < xMax &&
            x2 >= 0 && x2 < xMax &&
            z1 >= 0 && z1 < zMax &&
            z2 >= 0 && z2 < zMax;

        if (!indicesValidos)
            return 0f;
        // Obtener alturas de las 4 esquinas
        float h11 = HeightmapData[x1, z1] * ScaleY;
        float h21 = HeightmapData[x2, z1] * ScaleY;
        float h12 = HeightmapData[x1, z2] * ScaleY;
        float h22 = HeightmapData[x2, z2] * ScaleY;

        // Interpolación bilineal
        float h1 = MathHelper.Lerp(h11, h21, fx);
        float h2 = MathHelper.Lerp(h12, h22, fx);
        float finalHeight = MathHelper.Lerp(h1, h2, fz);

        return Center.Y + finalHeight;
    }

    
    public Quaternion CalculateRotation(Vector3 position, float rotation)
    {
        //obtenemos la normal
        var normalHaciaArriba = GetNormalAtPosition(position.X, position.Z);
            
        // Direcciones objetivo: mantené la YAW de la física
        var yawForward = Vector3.Transform(
            -Vector3.UnitZ,
            Matrix.CreateRotationY(rotation));

        // Proyectá el forward sobre el plano del terreno para que siga la pendiente
        var forwardOnPlane = yawForward - Vector3.Dot(yawForward, normalHaciaArriba) * normalHaciaArriba;
        if (forwardOnPlane.LengthSquared() < 1e-6f)
            forwardOnPlane = Vector3.Normalize(
                Vector3.Cross(
                    new Vector3(1, 0, 0), normalHaciaArriba));
        else
            forwardOnPlane.Normalize();

        var right = Vector3.Normalize(
            Vector3.Cross(forwardOnPlane, normalHaciaArriba));
        var forward = Vector3.Normalize(
            Vector3.Cross(normalHaciaArriba, right));

        // Matriz de orientación a partir de la base R-U-F
        var orientationMatrix = new Matrix(
            right.X, right.Y, right.Z, 0f,
            normalHaciaArriba.X, normalHaciaArriba.Y, normalHaciaArriba.Z, 0f,
            forward.X, forward.Y, forward.Z, 0f,
            0f, 0f, 0f, 1f
        );
        
        return Quaternion.CreateFromRotationMatrix(orientationMatrix);
    }

    private Vector3 GetNormalAtPosition(float worldX, float worldZ)
    {
        var h = ScaleXz * 0.5f; 
        var x = worldX;
        var z = worldZ;

        var hL = GetHeightAtPosition(x - h, z);
        var hR = GetHeightAtPosition(x + h, z);
        var hD = GetHeightAtPosition(x, z - h);
        var hU = GetHeightAtPosition(x, z + h);

        var tangentX = new Vector3(2f * h, hR - hL, 0f);
        var tangentZ = new Vector3(0f, hU - hD, 2f * h);
        
        return  Vector3.Normalize(Vector3.Cross(tangentZ, tangentX));
    }

    public Matrix GetRotationMatrix(Vector3 normal)
    {
        var up = normal;
        var reference = Math.Abs(up.Y) < 0.99f ? Vector3.Up : Vector3.Right;
        var right = Vector3.Normalize(Vector3.Cross(reference, up));
        var forward = Vector3.Cross(up, right);
        return new Matrix(
            right.X,   right.Y,   right.Z,   0,
            up.X,      up.Y,      up.Z,      0,
            forward.X, forward.Y, forward.Z, 0,
            0,         0,         0,         1
        );
    }
    
    public float GetSlopeDegreesAt(float worldX, float worldZ)
    {
        var n = GetNormalAtPosition(worldX, worldZ);
        var dot = Vector3.Dot(n, Vector3.Up);
        dot = MathHelper.Clamp(dot, -1f, 1f);
        var radians = MathF.Acos(dot);
        return MathHelper.ToDegrees(radians);
    }

    public void DrawPastoShadow(BoundingFrustum boundingFrustum, GraphicsDevice graphicsDevice, TargetCamera targetLightCamera, Pasto pasto, float time)
    {
        foreach (var chunk in Chunks)
        {
            if (boundingFrustum.Intersects(chunk.BoundingBox))
            {
                pasto.DrawPasto(graphicsDevice, targetLightCamera.View, targetLightCamera.Projection , chunk.InstanceBuffer, time);
            }
        }
    }

    public class TerrainChunk
    {
        public VertexBuffer VertexBuffer;
        public IndexBuffer IndexBuffer;
        public BoundingBox BoundingBox;
        public VertexBuffer InstanceBuffer;
    }
}