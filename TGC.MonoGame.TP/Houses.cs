using System.Collections.Generic;
using System.Linq;
using BepuPhysics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class Houses(Terrain terrain, Simulation simulation) : ModelGroup(Colors, terrain, simulation)
{
    private static readonly List<Color> Colors = new List<Color>
    {
        new Color(255, 94, 45),    // house
        new Color(252, 209, 86)    // cottage
    };
    
    public void CrearObjetos()
    {
        var parametros = new[]
        { 
            // Por si se quiere configurar cada modelo en concreto de forma distinta
            // (altura, escalaMin, escalaMax)
            (0f, 34f, 34f),  // house
            (0f, 0.1f, 0.1f) // cottage
        };

        base.CrearObjetos(parametros);
        
        var parametrosRigidBodies = new[]
        { 
            // Por si se quiere configurar cada modelo en concreto de forma distinta
            // (ancho, alto, profundidad, yawEnGrados)
            (3.75f, 4.5f, 3.25f, 0f), // house
            (3500f, 1500f, 1725f, 5f) // cottage
        };
        
        CrearRigidBodies(parametrosRigidBodies);
    }

    public void CargarModelos(Effect efecto, ContentManager content)
    {
        var paths = new string[]
        {
            "/house/City_House_2_BI",
            "cottage/cottage_fbx",
        };
        
        base.CargarModelos(efecto, content, paths);
    }
}