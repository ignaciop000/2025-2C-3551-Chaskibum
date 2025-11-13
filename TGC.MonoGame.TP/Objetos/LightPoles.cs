using System.Collections.Generic;
using System.Linq;
using BepuPhysics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class LightPoles(Simulation simulation) : ModelGroup(Colors, simulation)
{
    private static readonly List<Color> Colors = [Color.White];
    
    public void CrearObjetos(Terrain terrain)
    {
        var parametros = new (float, float, float, bool, Texture2D[])[]
        { 
            // Por si se quiere configurar cada modelo en concreto de forma distinta
            // (altura, escalaMin, escalaMax)
            (0f, 0.25f, 0.25f, false, null) // SimpleStreetLight
        };

        base.CrearObjetos(parametros, terrain);
        
        var parametrosRigidBodies = new[]
        { 
            // Por si se quiere configurar cada modelo en concreto de forma distinta
            // (ancho, alto, profundidad, yawEnGrados)
            (7f, 400f, 7f, 0f) // SimpleStreetLight
        };
        
        CrearRigidBodies(parametrosRigidBodies);
    }
    
    public void CargarModelos(Effect efecto, ContentManager content)
    {
        var paths = new[]
        {
            "/LightPole/SimpleStreetLight" // SimpleStreetLight
        };
        
        base.CargarModelos(efecto, content, paths);
    }
    
    public override void OnCollisionWithTank(StaticHandle handle)
    {
        // Rompe el objeto
        var model = Models.FirstOrDefault(m => m.Handles.Contains(handle));
        model?.DestruirInstancia(handle);
    }
}