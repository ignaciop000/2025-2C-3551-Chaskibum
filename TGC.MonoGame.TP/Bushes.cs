using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class Bushes() : ModelGroup(Colors)
{
    private static readonly List<Color> Colors = new List<Color>
    {
        new Color(0, 95, 12)  // Bush
    };
    
    public void CrearObjetos()
    {
        var parametros = new (float, float, float)[]
        { 
            // Por si se quiere configurar cada modelo en concreto de forma distinta
            // (altura, escalaMin, escalaMax)
            (-105, 1f, 1f) // Bush
        };

        base.CrearObjetos(parametros);
    }
    
    public void CargarModelos(Effect efecto, ContentManager content)
    {
        var paths = new string[]
        {
            "/bush/IVY_FBX" // Bush
        };
        
        base.CargarModelos(efecto, content, paths);
    }
    
}