namespace TGC.MonoGame.TP;

public record ProjectileConfig(float Speed, float Mass, float Radius, float Cooldown)
{
    public readonly float Speed = Speed;
    public readonly float Mass = Mass;
    public readonly float Radius = Radius;
    public readonly float Cooldown = Cooldown;
}

public static class ProjectilePresets
{
    public static readonly ProjectileConfig Light = new(500f, 2f, 0.85f, 1f);
    public static readonly ProjectileConfig Heavy = new(400f, 5f, 1.15f, 3f);
}