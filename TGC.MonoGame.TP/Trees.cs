using System.Collections.Generic;
using System.Linq;
using BepuPhysics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class Trees(Terrain terrain, Simulation simulation) : ModelGroup(Colors, terrain, simulation)
{
    private static readonly List<Color> Colors = new List<Color>
    {
        new Color(100, 50, 40),  // Tree
        new Color(90, 158, 42),  // Tree 2
        new Color(25, 60, 40)    // Tree 3
    };
    
    public void CrearObjetos()
    {
        var parametros = new[]
        { 
            // Por si se quiere configurar cada modelo en concreto de forma distinta
            // (altura, escalaMin, escalaMax)
            (0f, 25f, 50f), // Tree
            (0f, 0.15f, 0.3f), // Tree 2
            (0f, 10f, 25f) // Tree 3
        };

        base.CrearObjetos(parametros);
        
        var parametrosRigidBodies = new[]
        { 
            // Por si se quiere configurar cada modelo en concreto de forma distinta
            // (ancho, alto, profundidad)
            (0.5f, 2.5f, 0.3f), // Tree
            (65f, 400f, 75f), // Tree 2
            (0.6f, 12f, 0.6f) // Tree 3
        };
        
        CrearRigidBodies(parametrosRigidBodies);
    }
    
    public void CargarModelos(Effect efecto, ContentManager content)
    {
        var paths = new string[]
        {
            "/tree/Tree0",
            "/tree2/Leaf_Oak",
            "/tree3/Tree"
        };
        
        base.CargarModelos(efecto, content, paths);
    }
}