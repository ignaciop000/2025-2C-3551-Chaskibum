using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class TankEntry
{
    public string Name;
    public Model Model;
    public Texture2D Texture;
    public float scale;
    public float posY;
    public Effect effect;

    public TankEntry(string n, Model m, Texture2D t, float scale, float posY, Effect effect)
    {
        Name = n;
        Model = m;
        Texture = t;
        this.scale = scale;
        this.posY = posY;
        this.effect = effect;
    }
}