using System.Collections.Generic;
using System.Linq;
using BepuPhysics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class Trees(Simulation simulation) : ModelGroup(Colors, simulation)
{
    private static readonly List<Color> Colors =
    [
        Color.White, // Tree
        Color.White // Tree 2
    ];
    
    public void CrearObjetos(Texture2D normalMapTree2Leaves, Texture2D normalMapTree2Bark, Texture2D normalMapTreeLeaves, Terrain terrain)
    {
        var normalMapsTree1 = new[]{normalMapTreeLeaves};
        var normalMapsTree2 = new[]{normalMapTree2Leaves, normalMapTree2Bark};
        var parametros = new []
        { 
            // Por si se quiere configurar cada modelo en concreto de forma distinta
            // (altura, escalaMin, escalaMax)
            (0f, 40f, 75f, true, normalMapsTree1), // Tree
            (0f, 0.22f, 0.45f, true, normalMapsTree2) // Tree 2
        };

        base.CrearObjetos(parametros, terrain);
        
        var parametrosRigidBodies = new[]
        { 
            // Por si se quiere configurar cada modelo en concreto de forma distinta
            // (ancho, alto, profundidad, yawEnGrados)
            (0.5f, 2.5f, 0.3f, 0f), // Tree
            (65f, 400f, 75f, 0f) // Tree 2
        };
        
        CrearRigidBodies(parametrosRigidBodies);
    }
    
    public void CargarModelos(Effect efecto, ContentManager content)
    {
        var paths = new[]
        {
            "/tree/Tree",
            "/tree2/Leaf_Oak"
        };
        
        // Texturas de corteza (BarkTexture)
        var barkTextures = new[]
        {
            "/tree/Tree.fbm/bark_0021",  // Tree 1 - corteza
            "/tree2/tileable_tree_bark_texture_by_ftourini-d3l69hz" // Tree 2 - corteza
        };
        
        // Texturas de hojas (LeavesTexture)
        var leavesTextures = new[]
        {
            "/tree/Tree.fbm/DB2X2_L01", // Tree 1 - hojas
            "/tree2/TexturesCom_Branches0018_1_alphamasked_S" // Tree 2 - hojas
        };
        
        base.CargarModelos(efecto, content, paths, barkTextures, leavesTextures);
    }

    public void DrawSombra(BoundingFrustum boundingFrustum, Effect effect, TargetCamera targetLightCamera, float time)
    {
        effect.Parameters["Time"]?.SetValue(time);
        
        foreach (var instance in Models)
        {
            effect.Parameters["checkInvisible"]?.SetValue(true);
            var model =  instance.Model;
            var modelMeshesBaseTransforms = new Matrix[model.Bones.Count];
            model.CopyAbsoluteBoneTransformsTo(modelMeshesBaseTransforms);
            var worlds = instance.Worlds;
            
            foreach (var world in worlds)
            {
                if (!instance.EsVisible(world, boundingFrustum))
                {
                    continue;
                }
                foreach (var modelMesh in model.Meshes)
                {
                    foreach (var part in modelMesh.MeshParts)
                        part.Effect = effect;
                    var worldMatrix = modelMeshesBaseTransforms[modelMesh.ParentBone.Index] * world;
                    SetVariables(modelMesh, effect, instance, worldMatrix, targetLightCamera);
                    
                    
                    modelMesh.Draw();
                }
            }
        }
        effect.Parameters["checkInvisible"]?.SetValue(false);
        effect.Parameters["Sway"]?.SetValue(0);
    }

    private void SetVariables(ModelMesh mesh, Effect effect, ModelInstances model, Matrix worldMatrix, TargetCamera targetLightCamera)
    {
        if (mesh.Name.Contains("Plane") || mesh.Name.Contains("leaves") || mesh.Name.Contains("polySurface1.001"))
        {
            effect.Parameters["Sway"]?.SetValue(1);
            effect.Parameters["ModelTexture"]?.SetValue(model.Texturas[1]);
            effect.Parameters["ViewProjection"]?.SetValue(targetLightCamera.View * targetLightCamera.Projection);
            effect.Parameters["World"]?.SetValue(worldMatrix);
            
        }
        else
        {
            effect.Parameters["Sway"]?.SetValue(0);
            effect.Parameters["ModelTexture"]?.SetValue(model.Texturas[0]);
            effect.Parameters["WorldViewProjection"]?.SetValue(worldMatrix * targetLightCamera.View * targetLightCamera.Projection);
        }
    }
}