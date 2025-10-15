using System.Collections.Generic;
using System.Linq;
using BepuPhysics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class Trees(Terrain terrain, Simulation simulation) : ModelGroup(Colors, terrain, simulation)
{
    private static readonly List<Color> Colors =
    [
        Color.White, // Tree
        Color.White, // Tree 2
        Color.White  // Tree 3
    ];
    
    public void CrearObjetos()
    {
        var parametros = new[]
        { 
            // Por si se quiere configurar cada modelo en concreto de forma distinta
            // (altura, escalaMin, escalaMax)
            (0f, 25f, 50f), // Tree
            (0f, 0.15f, 0.3f), // Tree 2
            (0f, 10f, 25f) // Tree 3
        };

        base.CrearObjetos(parametros);
        
        var parametrosRigidBodies = new[]
        { 
            // Por si se quiere configurar cada modelo en concreto de forma distinta
            // (ancho, alto, profundidad, yawEnGrados)
            (0.5f, 2.5f, 0.3f, 0f), // Tree
            (65f, 400f, 75f, 0f), // Tree 2
            (0.6f, 12f, 0.6f, 0f) // Tree 3
        };
        
        CrearRigidBodies(parametrosRigidBodies);
    }
    
    public void CargarModelos(Effect efecto, ContentManager content)
    {
        var paths = new[]
        {
            "/tree/Tree",
            "/tree2/Leaf_Oak",
            "/tree3/Tree"
        };
        
        // Texturas de corteza (BarkTexture)
        var barkTextures = new[]
        {
            "/tree/Tree.fbm/bark_0021",  // Tree 1 - corteza
            "/tree2/tileable_tree_bark_texture_by_ftourini-d3l69hz", // Tree 2 - corteza
            null  // Tree 3 - por ahora sin textura específica
        };
        
        // Texturas de hojas (LeavesTexture)
        var leavesTextures = new[]
        {
            "/tree/Tree.fbm/DB2X2_L01", // Tree 1 - hojas
            "/tree2/TexturesCom_Branches0018_1_alphamasked_S", // Tree 2 - hojas
            null  // Tree 3 - por ahora sin textura específica
        };
        
        base.CargarModelos(efecto, content, paths, barkTextures, leavesTextures);
    }
    
    public override void OnCollisionWithTank(StaticHandle handle)
    {
        // Rompe el objeto
        var model = Models.FirstOrDefault(m => m.Handles.Contains(handle));
        model?.DestruirInstancia(handle);
    }
}