using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class Trees(Terrain terrain) : ModelGroup(Colors, terrain)
{
    private static readonly List<Color> Colors = new List<Color>
    {
        new Color(100, 50, 40),  // Tree
        new Color(90, 158, 42),  // Tree 2
        new Color(25, 60, 40)    // Tree 3
    };
    
    public void CrearObjetos()
    {
        var parametros = new (float, float, float)[]
        { 
            // Por si se quiere configurar cada modelo en concreto de forma distinta
            // (altura, escalaMin, escalaMax)
            (0, 25f, 50f), // Tree
            (0, 0.15f, 0.3f), // Tree 2
            (0, 10f, 25f) // Tree 3
        };

        base.CrearObjetos(parametros);
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