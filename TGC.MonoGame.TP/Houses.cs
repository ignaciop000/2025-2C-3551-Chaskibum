using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class Houses() : ModelGroup(Colors)
{
    private static readonly List<Color> Colors = new List<Color>
    {
        new Color(255, 94, 45),    // house
        new Color(252, 209, 86)    // cottage
    };
    
    public void CrearObjetos()
    {
        var parametros = new (float, float, float)[]
        { 
            // Por si se quiere configurar cada modelo en concreto de forma distinta
            // (altura, escalaMin, escalaMax)
            (0, 34f, 34f),  // house
            (0, 0.1f, 0.1f) // cottage
        };

        base.CrearObjetos(parametros);
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