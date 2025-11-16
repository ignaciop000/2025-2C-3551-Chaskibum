using System.Collections.Generic;
using System.Net.Mime;
using BepuPhysics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class Rocks(Simulation simulation) : ModelGroup(Colors, simulation)
{
    private static readonly List<Color> Colors =
    [
        Color.White, // Roca 0
        Color.White, // Roca 1
        Color.White, // Roca 2
        Color.White, // Roca 3
        Color.White, // Roca 4
        Color.White, // Roca 5
        Color.White, // Roca 6
        Color.White, // Roca 7
        Color.White, // Roca 8
        Color.White  // Roca 9
    ];

    public void CrearObjetos(Texture2D normalMap, Terrain terrain)
    {
        var normalMaps = new[]{normalMap};
        var parametros = new[]
        { 
            // Por si se quiere configurar cada modelo en concreto de forma distinta
            // (altura, escalaMin, escalaMax)
            (5f, 0.1f, 0.2f, true, normalMaps), // Roca 0
            (5f, 0.1f, 0.2f, true, normalMaps), // Roca 1
            (5f, 0.1f, 0.2f, true, normalMaps), // Roca 2
            (5f, 0.1f, 0.2f, true, normalMaps), // Roca 3
            (5f, 0.1f, 0.2f, true, normalMaps), // Roca 4
            (5f, 0.1f, 0.2f, true, normalMaps), // Roca 5
            (5f, 0.1f, 0.2f, true, normalMaps), // Roca 6
            (5f, 0.1f, 0.2f, true, normalMaps), // Roca 7
            (5f, 0.1f, 0.2f, true, normalMaps), // Roca 8
            (5f, 0.1f, 0.2f, true, normalMaps)  // Roca 9
        };

        base.CrearObjetos(parametros, terrain);
        
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
            (700f, 420f, 800f, -40f)  // Roca 9
        };
        
        CrearRigidBodies(parametrosRigidBodies);
    }
    
    public void CargarModelos(Effect efecto, ContentManager content)
    {
        int count = Models.Count;
        string[] paths = new string[count];
        string[] texturas = new string[count];

        for (int i = 0; i < count; i++)
        {
            paths[i] = $"/rocks/Rock{i}";
            // Asignar texturas correspondientes (Rock1_Diffuse a Rock9_Diffuse)
            texturas[i] = i == 0 ? "/rocks/Textures/Rock1_Diffuse" : $"/rocks/Textures/Rock{i}_Diffuse";
        }
        
        base.CargarModelos(efecto, content, paths, texturas);
    }

}