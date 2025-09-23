using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class Bushes
{
    private List<ModelInstances> _bushes = [];
    
    List<Vector3> _colors = new List<Vector3>
    {
        new Color(0, 95, 12).ToVector3()  // Bush
    };
    
    public Bushes()
    {
        for (int i = 0; i < 1; i++)
        {
            _bushes.Add(new ModelInstances());
        }
    }
    
    public void CrearObjetos()
    {
        var parametros = new (float, float, float)[]
        { 
            // Por si se quiere configurar cada modelo en concreto de forma distinta
            // (altura, escalaMin, escalaMax)
            (-105, 1f, 1f) // Bush
        };

        for (int i = 0; i < _bushes.Count; i++)
        {
            var (altura, escalaMin, escalaMax) = parametros[i];
            _bushes[i].CrearObjetos(altura, escalaMin, escalaMax);
        }
    }
    
    public void CargarModelos(Effect efecto, ContentManager content)
    {
        var paths = new string[]
        {
            "/bush/IVY_FBX" // Bush
        };
        for (int i = 0; i < _bushes.Count; i++)
        {
            _bushes[i].CargarModelo(paths[i], efecto, content);
        }
    }
    
    public void Dibujar()
    {
        for (int i = 0; i < _bushes.Count; i++)
        {
            _bushes[i].Effect.Parameters["DiffuseColor"].SetValue(_colors[i]);
            _bushes[i].Dibujar();
        }
    }
    
    // Devuelve la lista lista para generar posiciones con porcentaje aplicado
    public List<(ModelInstances modelo, double porcentaje)> GetModelosConPorcentaje(double porcentajeTotal)
    {
        double porcentajePorRoca = porcentajeTotal / _bushes.Count;
        return _bushes
            .Select(m => (modelo: m, porcentaje: porcentajePorRoca))
            .ToList();
    }
}