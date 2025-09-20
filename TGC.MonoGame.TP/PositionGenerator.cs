using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;

namespace TGC.MonoGame.TP;

public class PositionGenerator
{
    private readonly List<Vector2> _posiciones = GenerarPuntos();

    public void AgregarPosiciones(List<(ModelInstances modelo, float porcentaje)> modelos)
{
    int total = _posiciones.Count;

    // Calcular cuántas posiciones le toca a cada modelo según el porcentaje
    var cantidades = modelos.Select(m => (m.modelo, Cantidad: (int)Math.Round(total * m.porcentaje))).ToList();

    // Índices para contar cuántas posiciones ya asignó a cada modelo
    var contadores = new int[modelos.Count];

    int modeloIdx = 0;

    foreach (var pos in _posiciones)
    {
        // Buscar el siguiente modelo que aún no haya completado su cantidad
        while (contadores[modeloIdx] >= cantidades[modeloIdx].Cantidad)
        {
            modeloIdx = (modeloIdx + 1) % modelos.Count;
        }

        // Asignar posición
        cantidades[modeloIdx].modelo.AgregarPosiciones(new List<Vector2> { pos });
        contadores[modeloIdx]++;

        // Pasar al siguiente modelo para la próxima iteración
        modeloIdx = (modeloIdx + 1) % modelos.Count;
    }
}

    
    // Generar posiciones aleatorias que no se pisen
    // minDist = 400, width = 5000, height = 5000
    private static List<Vector2> GenerarPuntos(float minDist = 400, int width = 5000, int height = 5000, int attempts = 30)
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
                // Generar punto en un anillo entre [minDist, 110*minDist]
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