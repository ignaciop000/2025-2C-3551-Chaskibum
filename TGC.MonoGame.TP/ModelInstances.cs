using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class ModelInstances()
{
    private Model _model;
    private readonly List<Matrix> _worlds = [];
    private Effect _effect;
    
    private const string ContentFolder3D = TGCGame.ContentFolder3D;
    private readonly Random _rnd = new Random();

    public void CrearObjetoUnico(Matrix world)
    {
        _worlds.Add(world);
    }
    
    public void CrearObjetos(int cantidad, float altura, float escalaMin, float escalaMax)
    {
        for (int i = 0; i < cantidad; i++)
        {
            float x = _rnd.Next(-5000, 5000);
            float z = _rnd.Next(-5000, 5000);
            float escala = NextFloat(escalaMin, escalaMax);
            
            float yaw = MathHelper.ToRadians(_rnd.Next(0, 360));

            Matrix world = Matrix.CreateScale(escala, escala, escala) *
                           Matrix.CreateFromYawPitchRoll(yaw, 0f, 0f) *
                           Matrix.CreateTranslation(x, altura, z);

            _worlds.Add(world);
        } 
    }
    
    public void CargarModelo(string rutaRelativa, Effect efecto, ContentManager content)
    {
        _model = content.Load<Model>(ContentFolder3D + rutaRelativa);
        
        foreach (var mesh in _model.Meshes)
        {
            foreach (var meshPart in mesh.MeshParts)
            {
                meshPart.Effect = efecto;
            }
        }
        
        _effect = efecto; // Me lo guardo para usar en el dibujado
    }

    public void Dibujar()
    {
        foreach (var world in _worlds)
        {
            var modelMeshesBaseTransforms = new Matrix[_model.Bones.Count];
            _model.CopyAbsoluteBoneTransformsTo(modelMeshesBaseTransforms);
            foreach (var mesh in _model.Meshes)
            {
                var relativeTransform = modelMeshesBaseTransforms[mesh.ParentBone.Index];
                _effect.Parameters["World"].SetValue(relativeTransform * world);
                mesh.Draw();
            }
        }
    }
    
    private float NextFloat(float min, float max)
    {
        return (float)(min + (max - min) * _rnd.NextDouble());
    }
}