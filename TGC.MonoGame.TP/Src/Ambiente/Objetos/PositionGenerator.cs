using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;

namespace TGC.MonoGame.TP.Ambiente.Objetos;

public class PositionGenerator
{
    private readonly Random _random = new();
    public List<Vector2> ReservedPositions;

    // Aux: chequeo por distancia al cuadrado (evita sqrt)
    private static bool AnyCloserThan(IList<Vector2> list, Vector2 p, float minDistSquared)
    {
        for (int i = 0; i < list.Count; i++)
        {
            if (Vector2.DistanceSquared(list[i], p) < minDistSquared)
                return true;
        }
        return false;
    }
    
    public void GenerarPosicionesReservadas(float minDistReservadas = 350f, int maxIntentosPorPunto = 100)
    {
        ReservedPositions = [new Vector2(0f, 0f)];

        float minDist2 = minDistReservadas * minDistReservadas;

        // Máximo 20 tanques enemigos
        for (int i = 0; i < 20; i++)
        {
            bool colocado = false;

            for (int intento = 0; intento < maxIntentosPorPunto; intento++)
            {
                float radio = 1000f + (float)_random.NextDouble() * 1000f;
                float angulo = (float)(_random.NextDouble() * Math.PI * 2.0);

                var pos = new Vector2(
                    (float)Math.Cos(angulo) * radio,
                    (float)Math.Sin(angulo) * radio
                );

                if (!AnyCloserThan(ReservedPositions, pos, minDist2))
                {
                    ReservedPositions.Add(pos);
                    colocado = true;
                    break;
                }
            }

            // En el caso extremadamente raro de no poder colocar, se relaja levemente (falla tolerable)
            if (!colocado)
            {
                // Último intento: separa un poco más el radio para destrabar
                float radio = 1600f + (float)_random.NextDouble() * 800f;
                float angulo = (float)(_random.NextDouble() * Math.PI * 2.0);
                var pos = new Vector2(
                    (float)Math.Cos(angulo) * radio,
                    (float)Math.Sin(angulo) * radio
                );
                ReservedPositions.Add(pos);
            }
        }
    }
    
    public void AgregarPosiciones(List<(ModelInstances modelo, double porcentaje)> modelos, Color[,] colorMap, float escalaMap, float distanciaMinima = 550)
    {
        // Generar posiciones
        List<Vector2> posiciones = GenerarPuntos(distanciaMinima, colorMap, escalaMap);
        int total = posiciones.Count;
        int modelosCount = modelos.Count;

        // Mezclar posiciones usando Fisher–Yates
        Random rand = new Random();
        for (int i = posiciones.Count - 1; i > 0; i--)
        {
            int j = rand.Next(i + 1);
            (posiciones[i], posiciones[j]) = (posiciones[j], posiciones[i]);
        }

        // Calcular cuántas posiciones le toca a cada modelo
        int[] cantidades = new int[modelosCount];
        int suma = 0;
        for (int i = 0; i < modelosCount; i++)
        {
            cantidades[i] = (int)Math.Round(total * modelos[i].porcentaje);
            suma += cantidades[i];
        }
        if (suma != total)
            cantidades[0] += total - suma;

        // Preparar listas y contadores
        int[] contadores = new int[modelosCount];
        var listasPorModelo = new List<Vector2>[modelosCount];
        for (int i = 0; i < modelosCount; i++)
            listasPorModelo[i] = new List<Vector2>(cantidades[i]);

        // Asignar posiciones cíclicamente
        int modeloIdx = 0;
        foreach (var pos in posiciones)
        {
            while (contadores[modeloIdx] >= cantidades[modeloIdx])
                modeloIdx = (modeloIdx + 1) % modelosCount;

            listasPorModelo[modeloIdx].Add(pos);
            contadores[modeloIdx]++;
            modeloIdx = (modeloIdx + 1) % modelosCount;
        }

        // Agregar posiciones a cada modelo
        for (int i = 0; i < modelosCount; i++)
            modelos[i].modelo.Positions = listasPorModelo[i];
    }
    
    private List<Vector2> GenerarPuntos(float minDist, Color[,] colorMap, float escalaMap, int attempts = 100)
    {
        List<Vector2> points = [];

        int mapWidth = colorMap.GetLength(0);
        int mapHeight = colorMap.GetLength(1);

        var currentAttempts = attempts;
        while (currentAttempts > 0)
        {
            float pointX = _random.NextSingle() * mapWidth;
            float pointY = _random.NextSingle() * mapHeight;
            var color = colorMap[(int)pointX , (int)pointY];

            var x = ((pointX - mapWidth / 2f) * escalaMap)/2;
            var y = ((pointY - mapHeight / 2f) * escalaMap)/2;
            if (color.R < 150)
            {
                Vector2 newPoint = new Vector2(x, y);
                    
                // Validar dentro del área y lejos de otros puntos
                    
                if (points.All(p => Vector2.Distance(p, newPoint) >= minDist))
                {
                    points.Add(newPoint);
                    currentAttempts = attempts;
                }
            }
                
            currentAttempts--;
        }

        return points;
    }

    
    // Generar posiciones aleatorias para el pasto
    public List<Vector2> GenerarPuntosPasto(float minDist, Color[,] colorMap, float escalaMap, int chunkX, int chunkZ, 
                                            int chunkWidth, int chunkLength, Vector3 center, int width, int length, int attempts = 100)
    {
        List<Vector2> points = [];

        int mapWidth = colorMap.GetLength(0);
        int mapHeight = colorMap.GetLength(1);
        
        var currentAttempts = attempts;
        while (currentAttempts > 0)
        {
            float localX = _random.NextSingle() * chunkWidth;
            float localZ = _random.NextSingle() * chunkLength;
            
            float globalX = chunkX + localX;
            float globalZ = chunkZ + localZ;
            
            if (globalX >= mapWidth || globalZ >= mapHeight)
            {
                currentAttempts--;
                continue;
            }
            
            int colorX = (int)(globalX * (mapWidth - 1) / (width - 1));
            int colorZ = (int)(globalZ * (mapHeight - 1) / (length - 1));

            colorX = Math.Min(colorX, mapWidth - 1);
            colorZ = Math.Min(colorZ, mapHeight - 1);

            var color = colorMap[colorX, colorZ];
            
            if (color.R < 150)
            {
                float x = center.X + globalX * escalaMap;
                float z = center.Z + globalZ * escalaMap;
                
                Vector2 newPoint = new Vector2(x, z);
                    
                // Validar dentro del área y lejos de otros puntos
                    
                if (points.All(p => Vector2.Distance(p, newPoint) >= minDist))
                {
                    points.Add(newPoint);

                    currentAttempts = attempts;
                }
            }
                
            currentAttempts--;
        }
        return points;
    }
    
}