using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;

namespace TGC.MonoGame.TP;

public class PositionGenerator
{
    public void AgregarPosiciones(List<(ModelInstances modelo, double porcentaje)> modelos)
    {
        // Generar posiciones
        List<Vector2> posiciones = GenerarPuntos();
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
    private List<Vector2> GenerarPuntos(float minDist = 400, int width = 5000, int height = 5000, int attempts = 30)
    {
        Random rand = new Random();
        List<Vector2> points = new List<Vector2>();
        List<Vector2> active = new List<Vector2>();

        // Primer punto
        Vector2 first = new Vector2(100, 100);
        points.Add(first);
        active.Add(first);

        while (active.Count > 0)
        {
            int idx = rand.Next(active.Count);
            Vector2 center = active[idx];
            bool found = false;

            for (int i = 0; i < attempts; i++)
            {
                // Generar punto en un anillo entre [minDist, 110*minDist], para que esten bien separados
                double angle = rand.NextDouble() * Math.PI * 2;
                double radius = minDist * (1 + 109 * rand.NextDouble());
                Vector2 newPoint = center + new Vector2(
                    (float)(Math.Cos(angle) * radius),
                    (float)(Math.Sin(angle) * radius)
                );

                // Validar dentro del área y lejos de otros puntos
                if (newPoint.X >= -width && newPoint.X < width &&
                    newPoint.Y >= -height && newPoint.Y < height &&
                    points.All(p => Vector2.Distance(p, newPoint) >= minDist))
                {
                    points.Add(newPoint);
                    active.Add(newPoint);
                    found = true;
                    break;
                }
            }

            if (!found)
                active.RemoveAt(idx);
        }

        return points;
    }
}