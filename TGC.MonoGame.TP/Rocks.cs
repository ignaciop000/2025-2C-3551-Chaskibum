
using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class Rocks
{
    public List<ModelInstances> _rocks = [];
    private Effect _effect;
    
    List<Vector3> colores = new List<Vector3>
    {
        new Vector3(0.1f, 0.1f, 0.1f),  // Muy oscuro
        new Vector3(0.15f, 0.15f, 0.15f),
        new Vector3(0.2f, 0.2f, 0.2f),
        new Vector3(0.25f, 0.25f, 0.25f),
        new Vector3(0.35f, 0.35f, 0.35f),
        new Vector3(0.45f, 0.45f, 0.45f),
        new Vector3(0.55f, 0.55f, 0.55f),
        new Vector3(0.65f, 0.65f, 0.65f),
        new Vector3(0.75f, 0.75f, 0.75f),
        new Vector3(0.85f, 0.85f, 0.85f),
        new Vector3(0.95f, 0.95f, 0.95f) // Gris casi blanco
    };
    
    public Rocks()
    {
        for (int i = 0; i < 11; i++)
        {
            _rocks.Add(new ModelInstances());
        }
    }
    
    public void CrearObjetos()
    {
        
        var parametros = new (float, float, float)[]
        { 
            // Por si se quiere configurar cada modelo en concreto de forma distinta
            // (altura, escalaMin, escalaMax)
            (5, 0.1f, 0.2f), // Roca 0
            (5, 0.1f, 0.2f), // Roca 1
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
            _rocks[i].CrearObjetos(altura, escalaMin, escalaMax);
        }
    }
    
    public void CargarModelos(Effect efecto, ContentManager content)
    {
        for (int i = 0; i < _rocks.Count; i++)
        {
            string path = $"/rocks/Rock{i}";
            _rocks[i].CargarModelo(path, efecto, content);
        }
        
        _effect = efecto;
    }
    
    public void Dibujar()
    {
        for (int i = 0; i < _rocks.Count; i++)
        {
            _effect.Parameters["DiffuseColor"].SetValue(colores[i]);
            _rocks[i].Dibujar();
        }
    }
}