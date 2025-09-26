using System;
using System.Collections.Generic;
using BepuPhysics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class Terrain
{
    private readonly Effect _effect;
    private float _scaleXZ = 1;
    private float _scaleY = 1;
    private VertexBuffer _vbTerrain;
    private readonly Texture2D _colorMapTexture;
    private readonly Texture2D _terrainTexture;
    private readonly Texture2D _terrainTexture2;

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
        Texture2D diffuseMap, Texture2D diffuseMap2, Effect effect, Simulation simulation)
    {
        //Shader
        _effect = effect;
        // cargo el heightmap
        LoadHeightmap(graphicsDevice, heightMap, 100, 4, Vector3.Zero, simulation);
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
        Center = center;
        _scaleXZ = scaleXZ;
        _scaleY = scaleY;
        float tx_scale = 1; // 50f;

        //cargar heightmap
        HeightmapData = LoadHeightMap(heightmap);
        var width = HeightmapData.GetLength(0);
        var length = HeightmapData.GetLength(1);
        float min_h = 256;
        float max_h = 0;
        for (var i = 0; i < width; i++)
        for (var j = 0; j < length; j++)
        {
            //HeightmapData[i, j] = 256 - HeightmapData[i, j];
            if (HeightmapData[i, j] > max_h)
                max_h = HeightmapData[i, j];
            if (HeightmapData[i, j] < min_h)
                min_h = HeightmapData[i, j];
        }

        //Cargar vertices
        var totalVertices = 2 * 3 * (HeightmapData.GetLength(0) - 1) * (HeightmapData.GetLength(1) - 1);
        var dataIdx = 0;
        var data = new VertexPositionNormalTexture[totalVertices];

        center.X = center.X * scaleXZ - width / 2f * scaleXZ;
        center.Y = center.Y * scaleY;
        center.Z = center.Z * scaleXZ - length / 2f * scaleXZ;

        var N = new Vector3[width, length];
        for (var i = 0; i < width - 1; i++)
        for (var j = 0; j < length - 1; j++)
        {
            var v1 = new Vector3(center.X + i * scaleXZ, center.Y + HeightmapData[i, j] * scaleY,
                center.Z + j * scaleXZ);
            var v2 = new Vector3(center.X + i * scaleXZ, center.Y + HeightmapData[i, j + 1] * scaleY,
                center.Z + (j + 1) * scaleXZ);
            var v3 = new Vector3(center.X + (i + 1) * scaleXZ, center.Y + HeightmapData[i + 1, j] * scaleY,
                center.Z + j * scaleXZ);
            N[i, j] = Vector3.Normalize(Vector3.Cross(v2 - v1, v3 - v1));
        }

        for (var i = 0; i < width - 1; i++)
        for (var j = 0; j < length - 1; j++)
        {
            //Vertices
            var v1 = new Vector3(center.X + i * scaleXZ, center.Y + HeightmapData[i, j] * scaleY,
                center.Z + j * scaleXZ);
            var v2 = new Vector3(center.X + i * scaleXZ, center.Y + HeightmapData[i, j + 1] * scaleY,
                center.Z + (j + 1) * scaleXZ);
            var v3 = new Vector3(center.X + (i + 1) * scaleXZ, center.Y + HeightmapData[i + 1, j] * scaleY,
                center.Z + j * scaleXZ);
            var v4 = new Vector3(center.X + (i + 1) * scaleXZ, center.Y + HeightmapData[i + 1, j + 1] * scaleY,
                center.Z + (j + 1) * scaleXZ);

            //Coordendas de textura
            var t1 = new Vector2(i / (float)width, j / (float)length) * tx_scale;
            var t2 = new Vector2(i / (float)width, (j + 1) / (float)length) * tx_scale;
            var t3 = new Vector2((i + 1) / (float)width, j / (float)length) * tx_scale;
            var t4 = new Vector2((i + 1) / (float)width, (j + 1) / (float)length) * tx_scale;

            //Cargar triangulo 1
            data[dataIdx] = new VertexPositionNormalTexture(v1, N[i, j], t1);
            data[dataIdx + 1] = new VertexPositionNormalTexture(v2, N[i, j + 1], t2);
            data[dataIdx + 2] = new VertexPositionNormalTexture(v4, N[i + 1, j + 1], t4);

            //Cargar triangulo 2
            data[dataIdx + 3] = new VertexPositionNormalTexture(v1, N[i, j], t1);
            data[dataIdx + 4] = new VertexPositionNormalTexture(v4, N[i + 1, j + 1], t4);
            data[dataIdx + 5] = new VertexPositionNormalTexture(v3, N[i + 1, j], t3);

            dataIdx += 6;
        }

        //Crear vertexBuffer
        _vbTerrain = new VertexBuffer(graphicsDevice, VertexPositionNormalTexture.VertexDeclaration, totalVertices,
            BufferUsage.WriteOnly);
    _vbTerrain.SetData(data);

    // **CREAR MESH DE COLISIÓN PARA BEPU PHYSICS**
    CreatePhysicsCollision(simulation);
}

/// <summary>
/// Crea el mesh de colisión para la física del terreno usando un heightfield más simple
/// </summary>
/// <param name="simulation">La simulación de física</param>
private void CreatePhysicsCollision(Simulation simulation)
{
    var width = HeightmapData.GetLength(0);
    var height = HeightmapData.GetLength(1);
    
    // Crear un array de alturas para BepuPhysics
    var heights = new float[width * height];
    for (int z = 0; z < height; z++)
    {
        for (int x = 0; x < width; x++)
        {
            heights[z * width + x] = HeightmapData[x, z] * _scaleY;
        }
    }
    
    // Crear el heightfield de Bepu
    // En lugar de Mesh, usamos un enfoque más simple con boxes para cada celda
    // Esto es más eficiente para terrenos grandes
    CreateTerrainBoxes(simulation, width, height);
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
            // Obtener la altura promedio de esta celda
            float avgHeight = (HeightmapData[x, z] + HeightmapData[x + 1, z] + 
                              HeightmapData[x, z + 1] + HeightmapData[x + 1, z + 1]) / 4.0f;
            
            // Posición del centro de la box
            var position = new System.Numerics.Vector3(
                Center.X + (x + 0.5f) * _scaleXZ,
                Center.Y + avgHeight * _scaleY * 0.5f,
                Center.Z + (z + 0.5f) * _scaleXZ
            );
            
            // Crear una box
            var box = new BepuPhysics.Collidables.Box(_scaleXZ, avgHeight * _scaleY, _scaleXZ);
            var boxIndex = simulation.Shapes.Add(box);
            
            // Crear descripción estática
            var staticDescription = new BepuPhysics.StaticDescription(
                position,
                boxIndex
            );
            
            simulation.Statics.Add(staticDescription);
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
        foreach (var pass in _effect.CurrentTechnique.Passes)
        {
            pass.Apply();
            graphicsDevice.DrawPrimitives(PrimitiveType.TriangleList, 0, _vbTerrain.VertexCount / 3);
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
        float mapX = (worldX - Center.X + (width / 2f * _scaleXZ)) / _scaleXZ;
        float mapZ = (worldZ - Center.Z + (height / 2f * _scaleXZ)) / _scaleXZ;

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

}