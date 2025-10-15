using System.Runtime.CompilerServices;
using System.Threading;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;

namespace TGC.MonoGame.TP;

struct TankCallbacks : INarrowPhaseCallbacks
{
    public CollidableProperty<TankBodyProperties> Properties;
    private SpinLock _lock;
    
    private CollisionHandler _collisionHandler;

    public void Initialize(Simulation simulation)
    {
        Properties.Initialize(simulation);
    }

    public void SetCollisionHandler(CollisionHandler collisionHandler)
    {
        _collisionHandler = collisionHandler;
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
    public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold)
    {
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool ConfigureContactManifold<TManifold>(
        int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial
    ) where TManifold : unmanaged, IContactManifold<TManifold>
    {
        ref var propertiesA = ref Properties[pair.A.BodyHandle];
        pairMaterial.FrictionCoefficient = propertiesA.Friction;
        if (pair.B.Mobility != CollidableMobility.Static)
        {
            ref var propertiesB = ref Properties[pair.B.BodyHandle];
            pairMaterial.FrictionCoefficient = (pairMaterial.FrictionCoefficient + propertiesB.Friction) * 0.5f;
        }
        pairMaterial.MaximumRecoveryVelocity = 2f;
        pairMaterial.SpringSettings = new SpringSettings(30, 1);

        for (int i = 0; i < manifold.Count; ++i)
        {
            if (manifold.GetDepth(ref manifold, i) >= -1e-3f)
            {
                if (pair.B.Mobility == CollidableMobility.Static) // Los Static van siempre después
                {
                    AgregarImpactoEstatico(pair.A.BodyHandle, pair.B.StaticHandle);
                }
                else
                {
                    AgregarImpactoDinamico(pair.A.BodyHandle, pair.B.BodyHandle);
                }
                break;
            }
        }
        return true;
    }

    public void Dispose()
    {
        Properties.Dispose();
    }
    
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void AgregarImpactoEstatico(BodyHandle impactadorHandle, StaticHandle impactadoHandle)
    {
        bool lockTaken = false;
        _lock.Enter(ref lockTaken);
        try
        {
            var impactosEstaticos = CollisionHandler.ImpactosEstaticos;
            // Evitar duplicados: si ya está registrado, no lo agregamos
            for (int i = 0; i < impactosEstaticos.Count; ++i)
            {
                if (impactosEstaticos[i].Impactador.Value == impactadorHandle.Value)
                    return;
            }

            // Guardamos el BodyHandle del proyectil
            _collisionHandler.AgregarImpactoEstatico(impactadorHandle, impactadoHandle);
        }
        finally
        {
            if (lockTaken)
                _lock.Exit();
        }
    }
    
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void AgregarImpactoDinamico(BodyHandle impactadorHandle, BodyHandle impactadoHandle)
    {
        bool lockTaken = false;
        _lock.Enter(ref lockTaken);
        try
        {
            var impactosEstaticos = CollisionHandler.ImpactosDinamicos;
            // Evitar duplicados: si ya está registrado, no lo agregamos
            for (int i = 0; i < impactosEstaticos.Count; ++i)
            {
                if (impactosEstaticos[i].Impactador.Value == impactadorHandle.Value)
                    return;
            }

            // Guardamos el BodyHandle del proyectil
            _collisionHandler.AgregarImpactoDinamico(impactadorHandle, impactadoHandle);
        }
        finally
        {
            if (lockTaken)
                _lock.Exit();
        }
    }
}