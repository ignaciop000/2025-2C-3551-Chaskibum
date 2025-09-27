using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public abstract class ModelGroup
{
    protected readonly List<ModelInstances> Models = [];
    protected Terrain _terrain;

    protected ModelGroup(List<Color> colors, Terrain terrain)
    {
        _terrain = terrain;
        foreach (var color in colors)
        {
            Models.Add(new ModelInstances(color, _terrain));
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