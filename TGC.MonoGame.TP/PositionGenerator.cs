using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;

namespace TGC.MonoGame.TP;

public class PositionGenerator()
{
    private readonly Random _random = new();
    
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
    
    // Generar posiciones aleatorias que no se pisen
    private List<Vector2> GenerarPuntos(float minDist, Color[,] colorMap, float escalaMap, int attempts = 100)
    {
        List<Vector2> points = [];

        int mapWidth = colorMap.GetLength(0);
        int mapHeight = colorMap.GetLength(1);

            while (attempts > 0)
            {
                int pointX = _random.Next(0, mapWidth);
                int pointY = _random.Next(0, mapHeight);
                var color = colorMap[pointX, pointY];

                var x = ((pointX - mapWidth / 2f) * escalaMap)/2;
                var y = ((pointY - mapHeight / 2f) * escalaMap)/2;
                var noEsCalle = color.R < 150;
                
                Vector2 newPoint = new Vector2(x, y);
                
                // Validar dentro del área y lejos de otros puntos
                
                if (points.All(p => Vector2.Distance(p, newPoint) >= minDist) && noEsCalle)
                {
                    points.Add(newPoint);
                    attempts = 100;
                }
                attempts--;
            }

        return points;
    }
}