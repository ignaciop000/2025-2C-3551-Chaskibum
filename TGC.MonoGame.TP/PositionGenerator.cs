using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;

namespace TGC.MonoGame.TP;

public class PositionGenerator(float anchoMapa, float largoMapa)
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
    
    public void AgregarPosiciones(List<(ModelInstances modelo, double porcentaje)> modelos, float distanciaMinima = 550)
    {
        // Generar posiciones
        List<Vector2> posiciones = GenerarPuntos(distanciaMinima);
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
    private List<Vector2> GenerarPuntos(float minDist, int attempts = 100, float reservedMinDist = 350f)
    {
        List<Vector2> points = [];
        List<Vector2> active = [];

        double width = anchoMapa / 2;
        double height = largoMapa / 2;
        
        float minDist2 = minDist * minDist;
        float reservedMinDist2 = reservedMinDist * reservedMinDist;

        // Semilla: usar (0,0) solo como centro de expansión, no como punto válido
        active.Add(Vector2.Zero);

        while (active.Count > 0)
        {
            int idx = _random.Next(active.Count);
            Vector2 center = active[idx];
            bool found = false;

            for (int i = 0; i < attempts; i++)
            {
                // Generar punto en un anillo entre [minDist, 6 * minDist]
                double angle = _random.NextDouble() * Math.PI * 2.0;
                double radius = minDist * (1 + 5 * Math.Pow(_random.NextDouble(), 1.5)); // tienden a estar más cerca
                Vector2 newPoint = center + new Vector2(
                    (float)(Math.Cos(angle) * radius),
                    (float)(Math.Sin(angle) * radius)
                );

                // Offset de ruido
                float maxOffset = minDist * 0.3f; // 30% de minDist
                float offsetX = (float)(_random.NextDouble() - 0.5) * 2f * maxOffset;
                float offsetY = (float)(_random.NextDouble() - 0.5) * 2f * maxOffset;
                newPoint += new Vector2(offsetX, offsetY);

                // Descarta temprano si está cerca de alguna reservada
                if (ReservedPositions.Count > 0 &&
                    AnyCloserThan(ReservedPositions, newPoint, reservedMinDist2))
                {
                    continue;
                }

                // Validar dentro del área
                if (newPoint.X < -width || newPoint.X >= width || newPoint.Y < -height || newPoint.Y >= height)
                    continue;

                // Validar lejos de otros puntos
                bool ok = true;
                for (int p = 0; p < points.Count; p++)
                {
                    if (Vector2.DistanceSquared(points[p], newPoint) < minDist2)
                    {
                        ok = false;
                        break;
                    }
                }

                if (!ok)
                    continue;

                points.Add(newPoint);
                active.Add(newPoint);
                found = true;
                break;
            }

            if (!found)
                active.RemoveAt(idx);
        }

        return points;
    }
}