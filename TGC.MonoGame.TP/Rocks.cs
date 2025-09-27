using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class Rocks(Terrain terrain) : ModelGroup(Colors,terrain)
{
    private static readonly List<Color> Colors = new List<Color>
    {
        new Color(25, 25, 25),    // Roca 0
        new Color(38, 38, 38),    // Roca 1
        new Color(51, 51, 51),    // Roca 2
        new Color(64, 64, 64),    // Roca 3
        new Color(89, 89, 89),    // Roca 4
        new Color(115, 115, 115), // Roca 5
        new Color(140, 140, 140), // Roca 6
        new Color(166, 166, 166), // Roca 7
        new Color(179, 179, 179), // Roca 8
        new Color(204, 204, 204)  // Roca 9
    };

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

        base.CrearObjetos(parametros);
    }
    
    public void CargarModelos(Effect efecto, ContentManager content)
    {
        int count = Models.Count;
        string[] paths = new string[count];

        for (int i = 0; i < count; i++)
        {
            paths[i] = $"/rocks/Rock{i}";
        }
        
        base.CargarModelos(efecto, content, paths);
    }

}