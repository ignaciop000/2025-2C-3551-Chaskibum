using System.Collections.Generic;
using System.Linq;
using BepuPhysics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class Houses(Terrain terrain, Simulation simulation) : ModelGroup(Colors, terrain, simulation)
{
    private static readonly List<Color> Colors =
    [
        Color.White // house
    ];
    
    public void CrearObjetos()
    {
        var parametros = new[]
        { 
            // Por si se quiere configurar cada modelo en concreto de forma distinta
            // (altura, escalaMin, escalaMax)
            (0f, 34f, 34f)  // house
        };

        base.CrearObjetos(parametros);
        
        var parametrosRigidBodies = new[]
        { 
            // Por si se quiere configurar cada modelo en concreto de forma distinta
            // (ancho, alto, profundidad, yawEnGrados)
            (3.75f, 4.5f, 3.25f, 0f) // house
        };
        
        CrearRigidBodies(parametrosRigidBodies);
    }

    public void CargarModelos(Effect efecto, ContentManager content)
    {
        var paths = new[]
        {
            "/house/City_House_2_BI"
        };
        
        var texturas = new[]
        {
            "/house/city_house_2_Col", // Textura de color para la casa
        };
        
        base.CargarModelos(efecto, content, paths, texturas);
    }
}