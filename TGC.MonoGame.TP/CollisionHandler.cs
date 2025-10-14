using System.Collections.Generic;
using System.Linq;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;

namespace TGC.MonoGame.TP;

public struct ImpactoDinamico(BodyHandle impactador, BodyHandle impactado)
{
    public BodyHandle Impactador = impactador;
    public BodyHandle Impactado = impactado;
}

public struct ImpactoEstatico(BodyHandle impactador, StaticHandle impactado)
{
    public BodyHandle Impactador = impactador;
    public StaticHandle Impactado = impactado;
}

public class CollisionHandler()
{
    // Diccionario para mapear StaticHandle a ModelGroup
    public static readonly Dictionary<StaticHandle, ModelGroup> HandleToGroup = new();
    public static readonly Dictionary<BodyHandle, Projectile> HandleToProjectile = new();
    public static Dictionary<BodyHandle, Tank> HandleToTank;
    public static readonly List<ImpactoEstatico> ImpactosEstaticos = [];
    public static readonly List<ImpactoDinamico> ImpactosDinamicos = [];

    public static void AgregarImpactoEstatico(BodyHandle impactador, StaticHandle impactado)
    {
        ImpactosEstaticos.Add(new ImpactoEstatico(impactador, impactado));
    }
    
    public static void AgregarImpactoDinamico(BodyHandle impactador, BodyHandle impactado)
    {
        ImpactosDinamicos.Add(new ImpactoDinamico(impactador, impactado));
    }

    public void HandleCollisions()
    {
        foreach (var impacto in ImpactosEstaticos)
        {
            HandleStaticCollision(impacto.Impactador, impacto.Impactado);
        }
        
        ImpactosEstaticos.Clear();

        foreach (var impacto in ImpactosDinamicos)
        {
            HandleDynamicCollision(impacto.Impactador, impacto.Impactado);
        }
        
        ImpactosDinamicos.Clear();
    }

    
    private void HandleDynamicCollision(BodyHandle a, BodyHandle b)
    {
        if (HandleToProjectile.TryGetValue(a, out var projA)) // Si a es proyectil
        {
            if (HandleToTank.ContainsKey(b)) // Y b es tanque
            {
                HandleToTank[b].Kill();
                projA.Kill();
                return;
            }
        }
        
        if (HandleToProjectile.TryGetValue(b, out var projB)) // Si b es proyectil
        {
            if (HandleToTank.ContainsKey(a)) // Y a es tanque
            {
                HandleToTank[a].Kill();
                projB.Kill();
            }
        }
    }

    private void HandleStaticCollision(BodyHandle impactador, StaticHandle impactado)
    {
        // ¿El estático pertenece a un grupo conocido?
        if (!HandleToGroup.TryGetValue(impactado, out var group))
        {
            // Si no, entonces es "piso" u otro estático sin grupo.
            // Si el móvil era un proyectil, lo matamos.
            if (HandleToProjectile.TryGetValue(impactador, out var projectile))
                projectile.Kill();
            return;
        }

        // Si el móvil es un tanque
        if (HandleToTank.ContainsKey(impactador))
        {
            group.OnCollisionWithTank(impactado);
            return;
        }

        // Si el móvil es un proyectil
        if (HandleToProjectile.TryGetValue(impactador, out var proj))
        {
            group.OnCollisionWithProjectile(impactado);
            proj.Kill();
        }
    }
}