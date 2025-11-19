namespace TGC.MonoGame.TP.Proyectiles;

public record ProjectileType(float Speed, float Mass, float Radius, float MaxCooldown)
{
    public readonly float Speed = Speed;
    public readonly float Mass = Mass;
    public readonly float Radius = Radius;
    public readonly float MaxCooldown = MaxCooldown;
}

public static class ProjectileTypes
{
    public static readonly ProjectileType Light = new(1000f, 0.5f, 0.85f, 1f);
    public static readonly ProjectileType Heavy = new(800f, 1f, 1.15f, 3f);
}