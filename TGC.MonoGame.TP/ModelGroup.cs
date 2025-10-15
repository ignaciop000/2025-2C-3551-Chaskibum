using System.Collections.Generic;
using System.Linq;
using BepuPhysics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public abstract class ModelGroup
{
    protected readonly List<ModelInstances> Models = [];

    protected ModelGroup(List<Color> colors, Terrain terrain, Simulation simulation)
    {
        foreach (var color in colors)
        {
            Models.Add(new ModelInstances(color, terrain, simulation));
        }
    }

    protected void CrearObjetos((float, float, float)[] parametros)
    {
        for (int i = 0; i < Models.Count; i++)
        {
            var (altura, escalaMin, escalaMax) = parametros[i];
            Models[i].CrearObjetos(altura, escalaMin, escalaMax);
        }
    }
    
    protected void CrearRigidBodies((float, float, float, float)[] parametros)
    {
        for (int i = 0; i < Models.Count; i++)
        {
            var (ancho, alto, profundidad, yaw) = parametros[i];
            var handles = Models[i].CrearRigidBodies(ancho, alto, profundidad, yaw);

            foreach (var handle in handles)
            {
                CollisionHandler.HandleToGroup[handle] = this;
            }
        }
    }

    protected void CargarModelos(Effect efecto, ContentManager content, string[] pathsModelos)
    {
        for (int i = 0; i < Models.Count; i++)
        {
            Models[i].CargarModelo(pathsModelos[i], efecto, content);
        }
    }
    
    protected void CargarModelos(Effect efecto, ContentManager content, string[] pathsModelos, string[] pathsTexturas)
    {
        for (int i = 0; i < Models.Count; i++)
        {
            string texturaPath = i < pathsTexturas.Length ? pathsTexturas[i] : null;
            Models[i].CargarModelo(pathsModelos[i], efecto, content, texturaPath);
        }
    }
    
    protected void CargarModelos(Effect efecto, ContentManager content, string[] pathsModelos, string[] pathsTexturas, string[] pathsTexturas2)
    {
        for (int i = 0; i < Models.Count; i++)
        {
            string texturaPath = i < pathsTexturas.Length ? pathsTexturas[i] : null;
            string textura2Path = i < pathsTexturas2.Length ? pathsTexturas2[i] : null;
            Models[i].CargarModelo(pathsModelos[i], efecto, content, texturaPath, textura2Path);
        }
    }
    
    public void Dibujar()
    {
        foreach (var model in Models)
        {
            model.Dibujar();
        }
    }
    
    public List<(ModelInstances modelo, double porcentaje)> GetModelosConPorcentaje(double porcentajeTotal)
    {
        double porcentajePorModelo = porcentajeTotal / Models.Count;
        return Models
            .Select(m => (modelo: m, porcentaje: porcentajePorModelo))
            .ToList();
    }

    public virtual void OnCollisionWithTank(StaticHandle handle)
    {
        // Por defecto, solo choca con el objeto
    }
    
    public void OnCollisionWithProjectile(StaticHandle handle)
    {
        // Por defecto, rompe el objeto
        var model = Models.FirstOrDefault(m => m.Handles.Contains(handle));
        model?.DestruirInstancia(handle);
    }
    
    public void SetPlacementRules(float? maxSlopeDegrees, bool alignToTerrain)
    {
        foreach (var m in Models)
        {
            m.MaxSlopeDegrees = maxSlopeDegrees;
            m.AlignToTerrain = alignToTerrain;
        }
    }
}