using System;
using System.Collections.Generic;
using System.Linq;
using BepuPhysics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

/// <summary>
/// Simple enemy tanks - exactly like rocks but with tank model
/// </summary>
public class EnemyTanks(Terrain terrain, Simulation simulation) : ModelGroup(Colors, terrain, simulation)
{
    private static readonly List<Color> Colors = new List<Color>
    {
        new Color(150, 50, 50), // Dark red for enemy tanks
    };
    
    public void CrearObjetos()
    {
        var parametros = new[]
        { 
            // (altura, escalaMin, escalaMax)
            (0f, 1.0f, 1.0f), // Enemy tanks - same size as player
        };

        base.CrearObjetos(parametros);
        
        var parametrosRigidBodies = new[]
        { 
            // (ancho, alto, profundidad, yawEnGrados)
            (3f, 2f, 5f, 0f), // Tank dimensions
        };
        
        CrearRigidBodies(parametrosRigidBodies);
    }

    public void CargarModelos(Effect efecto, ContentManager content)
    {
        var paths = new string[]
        {
            "panzer", // Use panzer.xnb model from content/models/panzer/
        };
        
        base.CargarModelos(efecto, content, paths);
    }
}