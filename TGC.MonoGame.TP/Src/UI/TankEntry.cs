using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP.UI;

public class TankEntry(Model m, Texture2D t, float scale, float posY, Effect effect)
{
    public readonly Model Model = m;
    public readonly Texture2D Texture = t;
    public readonly float Scale = scale;
    public readonly float PosY = posY;
    public readonly Effect Effect = effect;
}