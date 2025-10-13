namespace TGC.MonoGame.TP;

public struct TankBodyProperties
{
    /// <summary>
    /// Controls which collidables the body can collide with.
    /// </summary>
    public SubgroupCollisionFilter Filter;
    /// <summary>
    /// Friction coefficient to use for the body.
    /// </summary>
    public float Friction;
    /// <summary>
    /// True if the body is a projectile and should explode on contact.
    /// </summary>
    public bool Projectile;
    /// <summary>
    /// True if the body is part of a tank.
    /// </summary>
    public bool TankPart;
}