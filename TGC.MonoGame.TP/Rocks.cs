using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class Rocks
{
    private readonly List<ModelInstances> _rocks = [];
    
    public Rocks()
    {
        for (int i = 0; i < 11; i++)
        {
            _rocks.Add(new ModelInstances());
        }
    }
    
    public void CrearObjetos(double cantidadTotal)
    {
        var cantidadPorRoca = (int) Math.Floor(cantidadTotal / 11);
        
        var parametros = new (float, float, float)[]
        { 
            // Por si se quiere configurar cada modelo en concreto de forma distinta
            // (altura, escalaMin, escalaMax)
            (5, 0.1f, 0.2f), // Roca 1
            (5, 0.1f, 0.2f), // Roca 0
            (5, 0.1f, 0.2f), // Roca 2
            (5, 0.1f, 0.2f), // Roca 3
            (5, 0.1f, 0.2f), // Roca 4
            (5, 0.1f, 0.2f), // Roca 5
            (5, 0.1f, 0.2f), // Roca 6
            (5, 0.1f, 0.2f), // Roca 7
            (5, 0.1f, 0.2f), // Roca 8
            (5, 0.1f, 0.2f), // Roca 9
            (5, 0.1f, 0.2f)  // Roca 10
        };

        for (int i = 0; i < _rocks.Count; i++)
        {
            var (altura, escalaMin, escalaMax) = parametros[i];
            _rocks[i].CrearObjetos(cantidadPorRoca, altura, escalaMin, escalaMax);
        }
    }
    
    public void CargarModelos(Effect efecto, ContentManager content)
    {
        for (int i = 0; i < _rocks.Count; i++)
        {
            string path = $"/rocks/Rock{i}";
            _rocks[i].CargarModelo(path, efecto, content);
        }
    }
    
    public void Dibujar()
    {
        foreach (var rock in _rocks)
        {
            rock.Dibujar();
        }
    }
}