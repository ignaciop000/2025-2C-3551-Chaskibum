using Vector3 = Microsoft.Xna.Framework.Vector3;

namespace TGC.MonoGame.TP;

public class PanzerTank(Vector3 initialPosition, Camera camera, float initialRotation = 0, float scale = 1)
    : Tank(initialPosition, camera, initialRotation, scale)
{
    
}