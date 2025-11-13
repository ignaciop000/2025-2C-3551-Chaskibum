using System.Collections.Generic;
using System.Linq;
using BepuPhysics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class Pasto(Simulation simulation, GraphicsDevice graphicsDevice) : ModelGroup(Colors, simulation)
{
    
    private static readonly List<Color> Colors = [Color.White];
    
    public void CrearObjetos(Terrain terrain)
    {
        var parametros = new (float, float, float, bool, Texture2D[])[]
        { 
            // (altura, escalaMin, escalaMax)
            (0f, 0.25f, 0.25f, false, null) 
        };

        base.CrearObjetos(parametros, terrain);
        /*
        var parametrosRigidBodies = new[]
        { 
            // (ancho, alto, profundidad, yawEnGrados)
            (7f, 400f, 7f, 0f) 
        };
        
        CrearRigidBodies(parametrosRigidBodies);*/
    }
    
    public void CargarModelos(Effect efecto, ContentManager content)
    {
        var paths = new[]
        {
            "/pasto/pasto" 
        };
        
        var texturas = new[]
        {
            "/pasto/isolated-wild-grasses-png" // Textura de hojas para el arbusto
        };
        
        base.CargarModelosPasto(efecto, content, paths, texturas, graphicsDevice);
    }
    
    public override void OnCollisionWithTank(StaticHandle handle)
    {
        // Rompe el objeto
        var model = Models.FirstOrDefault(m => m.Handles.Contains(handle));
        model?.DestruirInstancia(handle);
    }

    public void AgregarPosicionesPasto(List<Vector2> posiciones)
    {
        
    }
}