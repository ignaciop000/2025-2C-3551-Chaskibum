using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class Houses
{
    private List<ModelInstances> _houses = [];
    
    public Houses()
    {
    }
    public void CrearObjetos()
    {
        createHouse(34f,0f, new Vector3(500f,0f, 150f));
        createHouse(34f,0.5f, new Vector3(800f,0f, 500f));
        createHouse(34f,0.5f, new Vector3(800f,0f, 200f));
        createHouse(34f,1f, new Vector3(1000f,0f, 200f));
        createHouse(0.1f,0f, new Vector3(1200f,0f, 400f));
    }

    private void createHouse(float escala, float yaw, Vector3 position)
    {
        ModelInstances _house = new ModelInstances();
        _house.CrearObjeto(escala, yaw, position);
        _houses.Add(_house);
    }

    public void CargarModelos(Effect efecto, ContentManager content)
    {
        for (int i = 0; i < _houses.Count - 1; i++)
        {
            string path = $"/house/City_House_2_BI";
            _houses[i].CargarModelo(path, efecto, content);
        }
        _houses[^1].CargarModelo("cottage/cottage_fbx", efecto, content);
    }
    
    public void Dibujar()
    {
        for (int i = 0; i < _houses.Count; i++)
        {
            _houses[i].Effect.Parameters["DiffuseColor"].SetValue(Color.Red.ToVector3());
            _houses[i].Dibujar();
        }
    }
 
}