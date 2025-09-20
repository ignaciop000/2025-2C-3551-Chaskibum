using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class Trees
{
    private List<ModelInstances> _trees = [];
    
    List<Vector3> _colors = new List<Vector3>
    {
        new Vector3(0.1f, 0.1f, 0.1f),      // Tree 0
        new Vector3(0.15f, 0.15f, 0.15f),   // Tree 1
    };
    
    public Trees()
    {
        for (int i = 0; i < 2; i++)
        {
            _trees.Add(new ModelInstances());
        }
    }
    
    public void CrearObjetos()
    {
        
        var parametros = new (float, float, float)[]
        { 
            // Por si se quiere configurar cada modelo en concreto de forma distinta
            // (altura, escalaMin, escalaMax)
            (0, 25f, 50f), // Tree 0
            (0, 10f, 25f), // Tree 1
        };

        for (int i = 0; i < _trees.Count; i++)
        {
            var (altura, escalaMin, escalaMax) = parametros[i];
            _trees[i].CrearObjetos(altura, escalaMin, escalaMax);
        }
    }
    
    public void CargarModelos(Effect efecto, ContentManager content)
    {
        var paths = new string[]
        {
            "/tree/Tree0",
            "/greenTree/Tree1",
        };
        for (int i = 0; i < _trees.Count; i++)
        {
            _trees[i].CargarModelo(paths[i], efecto, content);
        }
    }
    
    public void Dibujar()
    {
        for (int i = 0; i < _trees.Count; i++)
        {
            _trees[i].Effect.Parameters["DiffuseColor"].SetValue(_colors[i]);
            _trees[i].Dibujar();
        }
    }
    
    // Devuelve la lista lista para generar posiciones con porcentaje aplicado
    public List<(ModelInstances modelo, double porcentaje)> GetModelosConPorcentaje(double porcentajeTotal)
    {
        double porcentajePorRoca = porcentajeTotal / _trees.Count;
        return _trees
            .Select(m => (modelo: m, porcentaje: porcentajePorRoca))
            .ToList();
    }
}