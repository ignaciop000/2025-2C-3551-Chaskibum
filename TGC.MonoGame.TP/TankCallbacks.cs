using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Threading;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities.Collections;
using TGC.MonoGame.TP;

namespace Demos.Demos.Tanks;

/// <summary>
/// Stores properties about a body in the tank demo.
/// </summary>
public struct TankDemoBodyProperties
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

public struct ProjectileImpact
{
    /// <summary>
    /// Handle of the projectile body associated with this impact.
    /// </summary>
    public BodyHandle ProjectileHandle;
    /// <summary>
    /// Handle of the tank body associated with whatever the projectile hit. If the projectile didn't hit a tank, this is -1.
    /// </summary>
    public BodyHandle ImpactedTankBodyHandle;
}

/// <summary>
/// For the tank demo, we want both wheel-body collision filtering and different friction for wheels versus the tank body.
/// </summary>
struct TankCallbacks : INarrowPhaseCallbacks
{
    // Diccionario para mapear StaticHandle a ModelGroup
    public static readonly Dictionary<StaticHandle, ModelGroup> HandleToGroup = new();
    
    public CollidableProperty<TankBodyProperties> Properties;
    public SpinLock ProjectileLock;
    public QuickList<ProjectileImpact> ProjectileImpacts;
    
    private List<BodyHandle> _tankBodies;
    private List<BodyHandle> _projectileBodies;
    
    public void Initialize(Simulation simulation)
    {
        Properties.Initialize(simulation);
    }
    
    public void Configure(List<BodyHandle> tankBodies, List<BodyHandle> projectileBodies)
    {
        _tankBodies = tankBodies;
        _projectileBodies = projectileBodies;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin)
    {
        //It's impossible for two statics to collide, and pairs are sorted such that bodies always come before statics.
        if (b.Mobility != CollidableMobility.Static)
        {
            return SubgroupCollisionFilter.AllowCollision(Properties[a.BodyHandle].Filter, Properties[b.BodyHandle].Filter);
        }
        return a.Mobility == CollidableMobility.Dynamic || b.Mobility == CollidableMobility.Dynamic;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
    {
        //This function is called for children of compounds, triangles in meshes, and similar cases, but we don't perform any child-level filtering in the tank demo.
        //The top level filter will always run before this function has a chance to, so we don't have to do anything here.
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    void TryAddProjectileImpact(BodyHandle projectileHandle, CollidableReference impactedCollidable)
    {
        bool lockTaken = false;
        ProjectileLock.Enter(ref lockTaken);
        try
        {
            //Note that we have to protect against redundant adds- a projectile might hit multiple things in the same frame. Wouldn't want it to explode multiple times.
            for (int i = 0; i < ProjectileImpacts.Count; ++i)
            {
                ref var impact = ref ProjectileImpacts[i];
                //If the projectile has already been handled, ignore it.
                if (impact.ProjectileHandle.Value == projectileHandle.Value)
                    return;
            }
            //The exploding projectiles list should have been sized ahead of time to hold all projectiles, so no dynamic allocations should be required.
            ref var newImpact = ref ProjectileImpacts.AllocateUnsafely();
            newImpact.ProjectileHandle = projectileHandle;
            if (impactedCollidable.Mobility != CollidableMobility.Static)
            {
                //The filter's group id is the tank's main body handle. We use that to find the tank (if this body is related to a tank at all).
                ref var properties = ref Properties[impactedCollidable.BodyHandle];
                newImpact.ImpactedTankBodyHandle = new BodyHandle(properties.TankPart ? properties.Filter.GroupId : -1);
            }
            else
            {
                //It hit a static; tank's aren't static.
                newImpact.ImpactedTankBodyHandle = new BodyHandle(-1);
            }
        }
        finally
        {
            if (lockTaken)
                ProjectileLock.Exit();
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial) where TManifold : unmanaged, IContactManifold<TManifold>
    {
        //Different tank parts have different friction values. Wheels tend to stick more than the body of the tank.
        ref var propertiesA = ref Properties[pair.A.BodyHandle];
        pairMaterial.FrictionCoefficient = propertiesA.Friction;
        if (pair.B.Mobility != CollidableMobility.Static)
        {
            //If two bodies collide, just average the friction. Other options include min(a, b) or a * b.
            ref var propertiesB = ref Properties[pair.B.BodyHandle];
            pairMaterial.FrictionCoefficient = (pairMaterial.FrictionCoefficient + propertiesB.Friction) * 0.5f;
        }
        //These are just some nice standard values. Higher maximum velocities can result in more energy being introduced during deep contact.
        //Finite spring stiffness helps the solver converge to a solution in difficult cases. Try to keep the spring frequency at around half of the timestep frequency or less.
        pairMaterial.MaximumRecoveryVelocity = 2f;
        pairMaterial.SpringSettings = new SpringSettings(30, 1);

        //if (propertiesA.Projectile || (pair.B.Mobility != CollidableMobility.Static && Properties[pair.B.BodyHandle].Projectile))
        //{
            for (int i = 0; i < manifold.Count; ++i)
            {
                //This probably looks a bit odd. You can't return refs to the this instance in structs, and interfaces can't require static functions...
                //so we use this redundant construction to get a direct reference to a contact's depth with near zero overhead.
                //There's a more typical out parameter overload for contact properties too. And there's always the option of using the manifold pointers directly.
                //Note the use of a nonzero negative threshold: speculative contacts will bring incoming objects to a stop at the surface, but in some cases integrator/numerical issues can mean that they don't quite reach.
                //In most cases, this isn't a problem at all, but tank projectiles are moving very quickly and a single missed frame might be enough to not trigger an explosion.
                //A nonzero epsilon helps catch those cases.
                //(An alternative would be to check each projectile's contact constraints and cause an explosion if any contact has nonzero penetration impulse.)
                if (manifold.GetDepth(ref manifold, i) >= -1e-3f)
                { 
                    //An actual collision was found. 
                    HandleObstacleCollision(pair);
                    
                    if (propertiesA.Projectile)
                    {
                        TryAddProjectileImpact(pair.A.BodyHandle, pair.B);
                    }
                    if (pair.B.Mobility != CollidableMobility.Static && Properties[pair.B.BodyHandle].Projectile)
                    {
                        //Could technically combine the locks in the case that both bodies are projectiles, but that's not exactly common.
                        TryAddProjectileImpact(pair.B.BodyHandle, pair.A);
                    }
                    break;
                }
            }
        //}
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold)
    {
        return true;
    }

    public void Dispose()
    {
        Properties.Dispose();
    }
    
    private void HandleObstacleCollision(CollidablePair pair)
    {
        if (!TryGetStaticAndMovil(pair, out var estatico, out var movil))
            return;

        var staticHandle = estatico.StaticHandle;
        if (!HandleToGroup.TryGetValue(staticHandle, out var group))
            return;

        if (_tankBodies.Contains(movil.BodyHandle))
        {
            group.OnCollisionWithTank(staticHandle);
        }
        else if (_projectileBodies.Contains(movil.BodyHandle))
        {
            group.OnCollisionWithProjectile(staticHandle);
        }
    }
    
    private bool TryGetStaticAndMovil(CollidablePair pair, out CollidableReference estatico, out CollidableReference movil)
    {
        if (pair.A.Mobility == CollidableMobility.Static && pair.B.Mobility != CollidableMobility.Static)
        {
            estatico = pair.A;
            movil = pair.B;
            return true;
        }
        if (pair.B.Mobility == CollidableMobility.Static && pair.A.Mobility != CollidableMobility.Static)
        {
            estatico = pair.B;
            movil = pair.A;
            return true;
        }

        estatico = default;
        movil = default;
        return false;
    }
}