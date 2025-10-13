using System.Collections.Generic;
using System.Linq;
using BepuPhysics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class Rocks(Terrain terrain, Simulation simulation) : ModelGroup(Colors, terrain, simulation)
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
        new Color(0, 0, 255)  // Roca 9
    };

    public void CrearObjetos()
    {
        var parametros = new[]
        { 
            // Por si se quiere configurar cada modelo en concreto de forma distinta
            // (altura, escalaMin, escalaMax)
            (5f, 0.1f, 0.2f), // Roca 0
            (5f, 0.1f, 0.2f), // Roca 1
            (5f, 0.1f, 0.2f), // Roca 2
            (5f, 0.1f, 0.2f), // Roca 3
            (5f, 0.1f, 0.2f), // Roca 4
            (5f, 0.1f, 0.2f), // Roca 5
            (5f, 0.1f, 0.2f), // Roca 6
            (5f, 0.1f, 0.2f), // Roca 7
            (5f, 0.1f, 0.2f), // Roca 8
            (5f, 0.1f, 0.2f)  // Roca 9
        };

        base.CrearObjetos(parametros);
        
        var parametrosRigidBodies = new[]
        { 
            // Por si se quiere configurar cada modelo en concreto de forma distinta
            // (ancho, alto, profundidad, yawEnGrados)
            (1300f, 250f, 700f, 30f), // Roca 0
            (1400f, 400f, 800f, 30f), // Roca 1
            (800f, 200f, 1000f, 0f), // Roca 2
            (850f, 350f, 950f, 30f), // Roca 3
            (400f, 350f, 500f, 25f), // Roca 4
            (450f, 300f, 500f, 35f), // Roca 5
            (1700f, 150f, 850f, -60f), // Roca 6
            (600f, 150f, 350f, -7f), // Roca 7
            (500f, 200f, 950f, 55f), // Roca 8
            (750f, 250f, 450f, -75f)  // Roca 9
        };
        
        CrearRigidBodies(parametrosRigidBodies);
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