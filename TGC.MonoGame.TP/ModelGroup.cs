using System.Collections.Generic;
using System.Linq;
using BepuPhysics;
using BepuPhysics.Collidables;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public abstract class ModelGroup
{
    protected readonly Simulation Simulation;
    protected readonly List<ModelInstances> Models = [];
    protected Terrain Terrain;

    protected ModelGroup(List<Color> colors, Terrain terrain, Simulation simulation)
    {
        Terrain = terrain;
        Simulation = simulation;
        foreach (var color in colors)
        {
            Models.Add(new ModelInstances(color, Terrain, Simulation));
        }
    }

    protected void CrearObjetos((float, float, float)[] parametros)
    {
        for (int i = 0; i < Models.Count; i++)
        {
            var (altura, escalaMin, escalaMax) = parametros[i];
            Models[i].CrearObjetos(altura, escalaMin, escalaMax);
        }
    }
    
    protected void CrearRigidBodies((float, float, float)[] parametros)
    {
        for (int i = 0; i < Models.Count; i++)
        {
            var (ancho, alto, profundidad) = parametros[i];
            Models[i].CrearRigidBodies(ancho, alto, profundidad);
        }
    }

    protected void CargarModelos(Effect efecto, ContentManager content, string[] pathsModelos)
    {
        for (int i = 0; i < Models.Count; i++)
        {
            Models[i].CargarModelo(pathsModelos[i], efecto, content);
        }
    }
    
    public void Dibujar()
    {
        foreach (var model in Models)
        {
            model.Dibujar();
        }
    }
    
    // Devuelve la lista para generar posiciones con porcentaje aplicado
    public List<(ModelInstances modelo, double porcentaje)> GetModelosConPorcentaje(double porcentajeTotal)
    {
        double porcentajePorModelo = porcentajeTotal / Models.Count;
        return Models
            .Select(m => (modelo: m, porcentaje: porcentajePorModelo))
            .ToList();
    }
}