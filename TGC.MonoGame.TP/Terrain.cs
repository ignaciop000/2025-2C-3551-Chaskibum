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
    private float _scaleXZ = 1;
    private float _scaleY = 1;
    private VertexBuffer _vbTerrain;
    private IndexBuffer _ibTerrain;
    private readonly Texture2D _colorMapTexture;
    private readonly Texture2D _terrainTexture;
    private readonly Texture2D _terrainTexture2;

    // Debug mesh (físico) del terreno
    private VertexBuffer _physDbgVB;
    private IndexBuffer _physDbgIB;
    private int _physDbgIndexCount;
    private bool _physDbgReady;

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
        Texture2D diffuseMap, Texture2D diffuseMap2, Effect effect, Simulation simulation, float scaleXZ)
    {
        //Shader
        _effect = effect;
        // cargo el heightmap
        LoadHeightmap(graphicsDevice, heightMap, scaleXZ, 4, Vector3.Zero, simulation);
        // textura con el color Map
        _colorMapTexture = colorMap;
        // diffuse maps auxiliares
        _terrainTexture = diffuseMap;
        _terrainTexture2 = diffuseMap2;
    }

    /// Genera la estructura del terreno cargando un mapa de alturas y creando los vértices necesarios.
    /// <param name="graphicsDevice">El dispositivo gráfico utilizado para generar el terreno.</param>
    /// <param name="heightmap">La textura que representa el mapa de alturas del terreno.</param>
    /// <param name="scaleXZ">La escala horizontal utilizada para ajustar el tamaño del terreno en los ejes X y Z.</param>
    /// <param name="scaleY">La escala vertical utilizada para ajustar la altura del terreno en el eje Y.</param>
    /// <param name="center">El punto central del terreno en coordenadas tridimensionales.</param>
    /// <param name="simulation">La simulación de física donde se agregará el mesh de colisión.</param>
    public void LoadHeightmap(GraphicsDevice graphicsDevice, Texture2D heightmap, float scaleXZ, float scaleY,
        Vector3 center, Simulation simulation)
    {
        _scaleXZ = scaleXZ;
        _scaleY = scaleY;
        float tx_scale = 1;

        //cargar heightmap
        HeightmapData = LoadHeightMap(heightmap);

        var width = HeightmapData.GetLength(0);
        var length = HeightmapData.GetLength(1);

        // Ajuste del centro
        center.X = center.X * scaleXZ - width / 2f * scaleXZ;
        center.Y = center.Y * scaleY;
        center.Z = center.Z * scaleXZ - length / 2f * scaleXZ;
        Center = center;

        // Crear vértices únicos
        var vertices = new VertexPositionNormalTexture[width * length];
        for (int i = 0; i < width; i++)
        {
            for (int j = 0; j < length; j++)
            {
                var pos = new Vector3(center.X + i * scaleXZ, center.Y + HeightmapData[i, j] * scaleY, center.Z + j * scaleXZ);
                var tex = new Vector2(i / (float)(width - 1), j / (float)(length - 1)) * tx_scale;
                vertices[i * length + j] = new VertexPositionNormalTexture(pos, Vector3.Zero, tex);
            }
        }

        // Crear índices y calcular normales por triángulo
        var indices = new int[(width - 1) * (length - 1) * 6];
        int idx = 0;
        for (int i = 0; i < width - 1; i++)
        {
            for (int j = 0; j < length - 1; j++)
            {
                int v1 = i * length + j;
                int v2 = i * length + (j + 1);
                int v3 = (i + 1) * length + j;
                int v4 = (i + 1) * length + (j + 1);

                // Triángulo 1: v1, v2, v4
                var n0 = Vector3.Normalize(Vector3.Cross(vertices[v2].Position - vertices[v1].Position, vertices[v4].Position - vertices[v1].Position));
                vertices[v1].Normal += n0;
                vertices[v2].Normal += n0;
                vertices[v4].Normal += n0;

                indices[idx++] = v1;
                indices[idx++] = v2;
                indices[idx++] = v4;

                // Triángulo 2: v1, v4, v3
                var n1 = Vector3.Normalize(Vector3.Cross(vertices[v4].Position - vertices[v1].Position, vertices[v3].Position - vertices[v1].Position));
                vertices[v1].Normal += n1;
                vertices[v4].Normal += n1;
                vertices[v3].Normal += n1;

                indices[idx++] = v1;
                indices[idx++] = v4;
                indices[idx++] = v3;
            }
        }

        // Normalizar normales
        for (int i = 0; i < vertices.Length; i++)
            vertices[i].Normal = Vector3.Normalize(vertices[i].Normal);

        // Crear buffers
        _vbTerrain = new VertexBuffer(graphicsDevice, VertexPositionNormalTexture.VertexDeclaration, vertices.Length, BufferUsage.WriteOnly);
        _vbTerrain.SetData(vertices);

        _ibTerrain = new IndexBuffer(graphicsDevice, IndexElementSize.ThirtyTwoBits, indices.Length, BufferUsage.WriteOnly);
        _ibTerrain.SetData(indices);

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
                float h11 = HeightmapData[x, z] * _scaleY;
                float h12 = HeightmapData[x, z + 1] * _scaleY;
                float h21 = HeightmapData[x + 1, z] * _scaleY;
                float h22 = HeightmapData[x + 1, z + 1] * _scaleY;

                // Posiciones de los 4 vértices de la celda
                var v00 = new System.Numerics.Vector3(Center.X + x * _scaleXZ, Center.Y + h11, Center.Z + z * _scaleXZ);
                var v01 = new System.Numerics.Vector3(Center.X + x * _scaleXZ, Center.Y + h12,
                    Center.Z + (z + 1) * _scaleXZ);
                var v10 = new System.Numerics.Vector3(Center.X + (x + 1) * _scaleXZ, Center.Y + h21,
                    Center.Z + z * _scaleXZ);
                var v11 = new System.Numerics.Vector3(Center.X + (x + 1) * _scaleXZ, Center.Y + h22,
                    Center.Z + (z + 1) * _scaleXZ);

                // Crear los triángulos de la celda
                triangles[index++] = new Triangle(v00, v10, v01);
                triangles[index++] = new Triangle(v10, v11, v01);
            }
        }

        // ⚙️ Crear el mesh con escala unitaria
        var mesh = new Mesh(triangles, new System.Numerics.Vector3(1, 1, 1), simulation.BufferPool);

        // Agregar el mesh al simulador
        var meshHandle = simulation.Shapes.Add(mesh);

        // Crear una descripción estática (el terreno no se mueve)
        var staticDescription = new StaticDescription(
            new System.Numerics.Vector3(0, 0, 0),
            System.Numerics.Quaternion.Identity,
            meshHandle
        );

        simulation.Statics.Add(staticDescription);

        // Liberar el buffer (ya fue copiado internamente)
        simulation.BufferPool.Return(ref triangles);
    }


    /// <summary>
    /// Crea boxes estáticas para representar el terreno
    /// </summary>
    private void CreateTerrainBoxes(Simulation simulation, int width, int height)
    {
        // Crear boxes pequeñas para cada celda del heightmap
        for (int x = 0; x < width - 1; x++)
        {
            for (int z = 0; z < height - 1; z++)
            {
                float h11 = HeightmapData[x, z] * _scaleY;
                float h21 = HeightmapData[x + 1, z] * _scaleY;
                float h12 = HeightmapData[x, z + 1] * _scaleY;
                float h22 = HeightmapData[x + 1, z + 1] * _scaleY;
                float heightY = MathF.Max(0.1f, MathF.Max(MathF.Max(h11, h21), MathF.Max(h12, h22)));
                var position = new System.Numerics.Vector3(
                    Center.X + (x + 0.5f) * _scaleXZ,
                    Center.Y + heightY * 0.5f,
                    Center.Z + (z + 0.5f) * _scaleXZ
                );
                var box = new BepuPhysics.Collidables.Box(_scaleXZ, heightY, _scaleXZ);

                var shapeIndex = simulation.Shapes.Add(box);
                simulation.Statics.Add(new StaticDescription(position, System.Numerics.Quaternion.Identity,
                    shapeIndex));
            }
        }
    }

    /// Carga un mapa de alturas desde una textura para ser utilizado como datos de elevación.
    /// <param name="texture">La textura que contiene el mapa de alturas.</param>
    /// <returns>Una matriz bidimensional de enteros que representa los valores de altura extraídos de la textura.</returns>
    protected int[,] LoadHeightMap(Texture2D texture)
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

    /// Dibuja el terreno en la pantalla utilizando los parámetros proporcionados para las matrices de transformación.
    /// Configura el efecto y los recursos necesarios para el renderizado, aplicando cada paso del shader.
    /// <param name="world">Matriz de transformación para coordenadas del mundo.</param>
    /// <param name="view">Matriz de vista de la cámara para determinar cómo se observa la escena.</param>
    /// <param name="projection">Matriz de proyección utilizada para la perspectiva 3D.</param>
    public void Draw(Matrix world, Matrix view, Matrix projection)
    {
        var graphicsDevice = _effect.GraphicsDevice;

        _effect.Parameters["World"].SetValue(world);
        _effect.Parameters["View"].SetValue(view);
        _effect.Parameters["Projection"].SetValue(projection);
        _effect.Parameters["texColorMap"].SetValue(_colorMapTexture);
        _effect.Parameters["texDiffuseMap"].SetValue(_terrainTexture);
        _effect.Parameters["texDiffuseMap2"].SetValue(_terrainTexture2);

        graphicsDevice.SetVertexBuffer(_vbTerrain);

        //Render con shader
        graphicsDevice.SetVertexBuffer(_vbTerrain);
        graphicsDevice.Indices = _ibTerrain;

        foreach (var pass in _effect.CurrentTechnique.Passes)
        {
            pass.Apply();
            graphicsDevice.DrawIndexedPrimitives(PrimitiveType.TriangleList, 0, 0, _ibTerrain.IndexCount / 3);
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
        float mapX = (worldX - Center.X) / _scaleXZ;
        float mapZ = (worldZ - Center.Z) / _scaleXZ;

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

        // Obtener alturas de las 4 esquinas
        float h11 = HeightmapData[x1, z1] * _scaleY;
        float h21 = HeightmapData[x2, z1] * _scaleY;
        float h12 = HeightmapData[x1, z2] * _scaleY;
        float h22 = HeightmapData[x2, z2] * _scaleY;

        // Interpolación bilineal
        float h1 = MathHelper.Lerp(h11, h21, fx);
        float h2 = MathHelper.Lerp(h12, h22, fx);
        float finalHeight = MathHelper.Lerp(h1, h2, fz);

        return Center.Y + finalHeight;
    }

    private void EnsurePhysicsDebugBuffers()
{
    if (_physDbgReady) return;

    int width  = HeightmapData.GetLength(0);
    int length = HeightmapData.GetLength(1);

    // 1) VERTS (grid compartido)
    var verts = new VertexPositionColor[width * length];
    int vi = 0;
    for (int z = 0; z < length; z++)
    {
        for (int x = 0; x < width; x++)
        {
            float h = HeightmapData[x, z] * _scaleY;
            var p = new Microsoft.Xna.Framework.Vector3(
                Center.X + x * _scaleXZ,
                Center.Y + h,
                Center.Z + z * _scaleXZ
            );
            verts[vi++] = new VertexPositionColor(p, Color.Yellow);
        }
    }

    // 2) ÍNDICES (2 triángulos por celda)
    int quadsX = width  - 1;
    int quadsZ = length - 1;
    int triCount = quadsX * quadsZ * 2;
    int indexCount = triCount * 3;

    bool need32 = (width * length) > 65535;
    if (!need32)
    {
        var idx = new ushort[indexCount];
        int k = 0;
        for (int z = 0; z < quadsZ; z++)
        {
            for (int x = 0; x < quadsX; x++)
            {
                int i0 =  z      * width + x;
                int i1 =  z      * width + (x + 1);
                int i2 = (z + 1) * width + x;
                int i3 = (z + 1) * width + (x + 1);

                // t0: i0, i2, i1
                idx[k++] = (ushort)i0;
                idx[k++] = (ushort)i2;
                idx[k++] = (ushort)i1;
                // t1: i1, i2, i3
                idx[k++] = (ushort)i1;
                idx[k++] = (ushort)i2;
                idx[k++] = (ushort)i3;
            }
        }

        var gd = _effect.GraphicsDevice;
        _physDbgVB = new VertexBuffer(gd, VertexPositionColor.VertexDeclaration, verts.Length, BufferUsage.WriteOnly);
        _physDbgVB.SetData(verts);

        _physDbgIB = new IndexBuffer(gd, IndexElementSize.SixteenBits, idx.Length, BufferUsage.WriteOnly);
        _physDbgIB.SetData(idx);

        _physDbgIndexCount = idx.Length;
    }
    else
    {
        var idx = new int[indexCount];
        int k = 0;
        for (int z = 0; z < quadsZ; z++)
        {
            for (int x = 0; x < quadsX; x++)
            {
                int i0 =  z      * width + x;
                int i1 =  z      * width + (x + 1);
                int i2 = (z + 1) * width + x;
                int i3 = (z + 1) * width + (x + 1);

                // t0: i0, i2, i1
                idx[k++] = i0; idx[k++] = i2; idx[k++] = i1;
                // t1: i1, i2, i3
                idx[k++] = i1; idx[k++] = i2; idx[k++] = i3;
            }
        }

        var gd = _effect.GraphicsDevice;
        _physDbgVB = new VertexBuffer(gd, VertexPositionColor.VertexDeclaration, verts.Length, BufferUsage.WriteOnly);
        _physDbgVB.SetData(verts);

        _physDbgIB = new IndexBuffer(gd, IndexElementSize.ThirtyTwoBits, idx.Length, BufferUsage.WriteOnly);
        _physDbgIB.SetData(idx);

        _physDbgIndexCount = idx.Length;
    }

    _physDbgReady = true;
}

    public void DrawPhysicsMeshDebug(Effect debugEffect, Matrix view, Matrix projection)
    {
        EnsurePhysicsDebugBuffers();

        var gd = _effect.GraphicsDevice;

        debugEffect.Parameters["View"]?.SetValue(view);
        debugEffect.Parameters["Projection"]?.SetValue(projection);
        debugEffect.Parameters["World"]?.SetValue(Matrix.Identity);

        var old = gd.RasterizerState;
        gd.RasterizerState = new RasterizerState { CullMode = CullMode.None, FillMode = FillMode.WireFrame };

        gd.SetVertexBuffer(_physDbgVB);
        gd.Indices = _physDbgIB;

        foreach (var pass in debugEffect.CurrentTechnique.Passes)
        {
            pass.Apply();
            gd.DrawIndexedPrimitives(PrimitiveType.TriangleList, 0, 0, _physDbgIndexCount / 3);
        }

        gd.RasterizerState = old;
    }
}