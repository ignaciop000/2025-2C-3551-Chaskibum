using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class TankEntry
{
    public string Name;
    public Model Model;
    public Texture2D Texture;
    public Matrix initialTransformation;
    public float scale;
    public float posY;

    public TankEntry(string n, Model m, Texture2D t, Matrix it, float scale, float posY)
    {
        Name = n;
        Model = m;
        Texture = t;
        initialTransformation = it;
        this.scale = scale;
        this.posY = posY;
    }
}