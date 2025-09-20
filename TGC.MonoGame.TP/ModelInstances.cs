using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class ModelInstances()
{
    private Model _model;
    private readonly List<Matrix> _worlds = [];
    private Effect _effect;
    private List<Vector2> _positions = [];

    private const string ContentFolder3D = TGCGame.ContentFolder3D;
    private readonly Random _rnd = new Random();
    
    public void AgregarPosiciones(List<Vector2> posiciones)
    {
        _positions = posiciones;
    }

    public void CrearObjetoUnico(Matrix world)
    {
        _worlds.Add(world);
    }

    // Retorna los puntos generados
    public void CrearObjetos(float altura, float escalaMin, float escalaMax)
    {
        foreach (var posicion in _positions)
        {
            float escala = NextFloat(escalaMin, escalaMax); //elegimos la escala al azar en base al min y max

            float
                yaw = MathHelper.ToRadians(_rnd.Next(0,
                    360)); //giramos de manera aletaria el objeto para que sean diferentes

            Matrix world = Matrix.CreateScale(escala, escala, escala) *
                           Matrix.CreateFromYawPitchRoll(yaw, 0f, 0f) *
                           Matrix.CreateTranslation(posicion.X, altura, posicion.Y);

            _worlds.Add(world);

        }
    }

    public void CargarModelo(string rutaRelativa, Effect efecto, ContentManager content)
    {
        _model = content.Load<Model>(ContentFolder3D + rutaRelativa);

        foreach (var mesh in _model.Meshes)
        {
            foreach (var meshPart in mesh.MeshParts)
            {
                meshPart.Effect = efecto;
            }
        }

        _effect = efecto; // Me lo guardo para usar en el dibujado
    }

    public void Dibujar()
    {
        foreach (var world in _worlds)
        {
            var modelMeshesBaseTransforms = new Matrix[_model.Bones.Count];
            _model.CopyAbsoluteBoneTransformsTo(modelMeshesBaseTransforms);
            foreach (var mesh in _model.Meshes)
            {
                var relativeTransform = modelMeshesBaseTransforms[mesh.ParentBone.Index];
                _effect.Parameters["World"].SetValue(relativeTransform * world);
                mesh.Draw();
            }
        }
    }

    private float NextFloat(float min, float max)
    {
        return (float)(min + (max - min) * _rnd.NextDouble());
    }

    // Para que no se pisen los objetos
    private List<Vector2> GenerarPuntos(List<Vector2> ocupados, float minDist, int width = 5000, int height = 5000, int attempts = 30)
    {
        Random rand = new Random();
        List<Vector2> points = new List<Vector2>();
        List<Vector2> active = new List<Vector2>();

        // Primer punto random
        Vector2 first = new Vector2(rand.Next(width), rand.Next(height));
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
                    points.All(p => Vector2.Distance(p, newPoint) >= minDist) &&
                    (ocupados.All(o => Vector2.Distance(o, newPoint) >= minDist)))
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
/*
    public List<Vector2> GenerateRocks(float minDistRocks, List<Vector2> treePositions, float minDistTrees, float width = 5000, float height = 5000, int attempts = 30)
    {
        Random rand = new Random();
        List<Vector2> rocks = new List<Vector2>();
        List<Vector2> active = new List<Vector2>();

        // Primer punto random dentro del área
        Vector2 first = new Vector2(rand.Next(width), rand.Next(height));

        rocks.Add(first);
        active.Add(first);

        while (active.Count > 0)
        {
            int idx = rand.Next(active.Count);
            Vector2 center = active[idx];
            bool found = false;

            for (int i = 0; i < attempts; i++)
            {
                // Generar punto en un anillo entre [minDistRocks, 2*minDistRocks]
                double angle = rand.NextDouble() * Math.PI * 2;
                double radius = minDistRocks * (1 + rand.NextDouble());
                Vector2 newPoint = center + new Vector2(
                    (float)(Math.Cos(angle) * radius),
                    (float)(Math.Sin(angle) * radius)
                );

                // Validar dentro del área
                if (newPoint.X >= -width && newPoint.X < width &&
                    newPoint.Y >= -height && newPoint.Y < height)
                    continue;

                // Validar distancia con otras rocas
                if (rocks.Any(r => Vector2.Distance(r, newPoint) < minDistRocks))
                    continue;

                // Validar distancia con los árboles
                if (treePositions.Any(t => Vector2.Distance(t, newPoint) < minDistTrees))
                    continue;

                // Si pasa todas las validaciones, lo agregamos
                rocks.Add(newPoint);
                active.Add(newPoint);
                found = true;
                break;
            }

            if (!found)
                active.RemoveAt(idx);
        }

        return rocks;
    }
*/
}