using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class Rocks
{
    private List<ModelInstances> _rocks = [];
    
    List<Vector3> _colors = new List<Vector3>
    {
        new Vector3(0.1f, 0.1f, 0.1f),      // Roca 0
        new Vector3(0.15f, 0.15f, 0.15f),   // Roca 1
        new Vector3(0.2f, 0.2f, 0.2f),      // Roca 2
        new Vector3(0.25f, 0.25f, 0.25f),   // Roca 3
        new Vector3(0.35f, 0.35f, 0.35f),   // Roca 4
        new Vector3(0.45f, 0.45f, 0.45f),   // Roca 5
        new Vector3(0.55f, 0.55f, 0.55f),   // Roca 6
        new Vector3(0.65f, 0.65f, 0.65f),   // Roca 7
        new Vector3(0.7f, 0.7f, 0.7f),      // Roca 8
        new Vector3(0.8f, 0.8f, 0.8f)       // Roca 9
    };
    
    public Rocks()
    {
        for (int i = 0; i < 10; i++)
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
            (5, 0.1f, 0.2f)  // Roca 9
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
    }
    
    public void Dibujar()
    {
        for (int i = 0; i < _rocks.Count; i++)
        {
            _rocks[i].Effect.Parameters["DiffuseColor"].SetValue(_colors[i]);
            _rocks[i].Dibujar();
        }
    }
    
    // Devuelve la lista lista para generar posiciones con porcentaje aplicado
    public List<(ModelInstances modelo, double porcentaje)> GetModelosConPorcentaje(double porcentajeTotal)
    {
        double porcentajePorRoca = porcentajeTotal / _rocks.Count;
        return _rocks
            .Select(m => (modelo: m, porcentaje: porcentajePorRoca))
            .ToList();
    }
}