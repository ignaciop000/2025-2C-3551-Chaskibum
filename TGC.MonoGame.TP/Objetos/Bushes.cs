using System.Collections.Generic;
using System.Linq;
using BepuPhysics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class Bushes(Simulation simulation) : ModelGroup(Colors, simulation)
{
    private static readonly List<Color> Colors = [Color.White];
    
    public void CrearObjetos(Terrain terrain)
    {
        var parametros = new (float, float, float, bool, Texture2D[])[]
        { 
            // Por si se quiere configurar cada modelo en concreto de forma distinta
            // (altura, escalaMin, escalaMax)
            (-15f, 1f, 2f, false, null) // Bush
        };

        base.CrearObjetos(parametros, terrain);
        
        var parametrosRigidBodies = new[]
        { 
            // Por si se quiere configurar cada modelo en concreto de forma distinta
            // (ancho, alto, profundidad, yawEnGrados)
            (20f, 10f, 20f, 0f) // Bush
        };
        
        CrearRigidBodies(parametrosRigidBodies);
    }
    
    public void CargarModelos(Effect efecto, ContentManager content)
    {
        var paths = new[]
        {
            "/bush/IVY_FBX" // Bush
        };
        
        var texturas = new[]
        {
            "/bush/tex/leaf01_D" // Textura de hojas para el arbusto
        };
        
        base.CargarModelos(efecto, content, paths, texturas);
    }
    
    public override void OnCollisionWithTank(StaticHandle handle)
    {
        // Rompe el objeto
        var model = Models.FirstOrDefault(m => m.Handles.Contains(handle));
        model?.DestruirInstancia(handle);
    }
    
}