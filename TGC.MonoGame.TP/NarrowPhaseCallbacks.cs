using System;
using System.Diagnostics.CodeAnalysis;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;

namespace TGC.MonoGame.TP;

/// <summary>
/// NarrowPhaseCallbacks es una estructura que define las operaciones relacionadas con la detección de colisiones
/// en la fase estrecha dentro de la simulación física. Su implementación permite personalizar cómo se generan
/// los contactos entre los objetos, cómo se configuran los parámetros de los contactos y cómo se administran los
/// recursos asociados a esta fase en la simulación.
/// </summary>
public struct NarrowPhaseCallbacks : INarrowPhaseCallbacks
{
    /// <summary>
    /// Propiedad que define la elasticidad del resorte utilizado para manejar los contactos
    /// en la simulación física. Esta elasticidad está representada por los parámetros
    /// de frecuencia angular y razón de amortiguamiento dos veces, que controlan cómo
    /// la simulación responde a las fuerzas de contacto entre los objetos.
    /// Los valores de ContactSpringiness afectan directamente la estabilidad y el comportamiento
    /// físico de los objetos al interactuar en colisión. Configuraciones que no son óptimas pueden
    /// resultar en comportamientos no deseados, como penetraciones excesivas o explosiones numéricas.
    /// </summary>
    private SpringSettings ContactSpringiness { get; set; }

    /// <summary>
    /// Propiedad que define la velocidad máxima de recuperación permitida entre los objetos en contacto
    /// durante la simulación física. Este valor limita la magnitud de la velocidad relativa con la que
    /// los objetos intentan separarse después de una colisión, ayudando a controlar comportamientos
    /// indeseados, como rebotes excesivos o respuestas físicamente inestables.
    /// </summary>
    private float MaximumRecoveryVelocity { get; set; }

    /// <summary>
    /// Propiedad que define el coeficiente de fricción utilizado en la simulación física.
    /// Este valor determina la resistencia relativa al deslizamiento entre dos superficies
    /// en contacto. Un coeficiente más bajo representará superficies más resbaladizas, mientras
    /// que un coeficiente más alto representará mayor resistencia al deslizamiento.
    /// El uso de valores adecuados para FrictionCoefficient contribuye a lograr un comportamiento
    /// físico más realista en las interacciones entre los objetos.
    /// </summary>
    private float FrictionCoefficient { get; set; }

    // Evento para notificar colisiones a la lógica de juego
    public Action<CollidablePair> OnCollision;

    /// <summary>
    /// Inicializa la simulación física configurando valores predeterminados
    /// como la elasticidad del resorte, la velocidad máxima de recuperación y el coeficiente de fricción,
    /// en caso de no haber sido previamente inicializados.
    /// </summary>
    /// <param name="simulation">Simulación física que será configurada.</param>
    public void Initialize(Simulation simulation)
    {
        if (ContactSpringiness.AngularFrequency == 0 && ContactSpringiness.TwiceDampingRatio == 0)
        {
            ContactSpringiness = new SpringSettings(30, 1);
            MaximumRecoveryVelocity = 2f;
            FrictionCoefficient = 1f;
        }
    }

    /// <summary>
    /// Determina si se permite la generación de contactos entre dos colisionadores específicos
    /// durante la simulación física.
    /// </summary>
    /// <param name="workerIndex">Índice del trabajador que ejecuta esta llamada en el contexto de la simulación multi-hilo.</param>
    /// <param name="a">Primera referencia al colisionador involucrado en la posible colisión.</param>
    /// <param name="b">Segunda referencia al colisionador involucrado en la posible colisión.</param>
    /// <param name="speculativeMargin">Margen especulativo utilizado para extender el rango de detección de colisiones
    /// entre los colisionadores.</param>
    /// <returns>Devuelve true si se permite la generación de contactos entre los colisionadores, de lo contrario, devuelve false.</returns>
    public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b,
        ref float speculativeMargin)
    {
        return a.Mobility == CollidableMobility.Dynamic || b.Mobility == CollidableMobility.Dynamic;
    }

    /// <summary>
    /// Configura las propiedades de un conjunto de contactos generados en la simulación física,
    /// estableciendo parámetros como el coeficiente de fricción, la velocidad máxima de recuperación
    /// y las configuraciones del resorte, necesarias para calcular las interacciones entre los objetos.
    /// </summary>
    /// <param name="workerIndex">Índice del trabajador que procesa este contacto en el sistema multihilo.</param>
    /// <param name="pair">Par de colisionadores que están interactuando.</param>
    /// <param name="manifold">Conjunto de contactos detectados entre los colisionadores.</param>
    /// <param name="pairMaterial">Propiedades del material del par que serán configuradas.</param>
    /// <typeparam name="TManifold">Tipo específico de la estructura que representa el conjunto de contactos.</typeparam>
    /// <returns>Un valor booleano que indica si deben generarse restricciones de contacto basadas en el conjunto configurado.</returns>
    public bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold,
        [UnscopedRef] out PairMaterialProperties pairMaterial)
        where TManifold : unmanaged, IContactManifold<TManifold>
    {
        pairMaterial.FrictionCoefficient = 1f;
        pairMaterial.MaximumRecoveryVelocity = 2f;
        pairMaterial.SpringSettings = new SpringSettings(30, 1);

        // Notificar colisión a la lógica de juego
        OnCollision?.Invoke(pair);

        return true;
    }

    /// <summary>
    /// Determina si se permite la generación de contactos en la simulación física entre dos objetos colisionables dados.
    /// </summary>
    /// <param name="workerIndex">Índice del trabajador que procesa esta operación, generalmente utilizado en simulaciones multihilo.</param>
    /// <param name="pair">Par de objetos colisionables que se están evaluando.</param>
    /// <param name="childIndexA">Índice del subcomponente del objeto A, en caso de colisionadores compuestos.</param>
    /// <param name="childIndexB">Índice del subcomponente del objeto B, en caso de colisionadores compuestos.</param>
    /// <returns>Un valor booleano que indica si se permite o no la generación de contactos entre los objetos evaluados.</returns>
    public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
    {
        return true;
    }

    /// <summary>
    /// Configura el colector de contactos especificando las propiedades materiales del par de colisionadores,
    /// como la fricción y la restitución, ajustando los contactos detectados según sea necesario.
    /// </summary>
    /// <param name="workerIndex">Índice del hilo de trabajo que genera el colector de contactos.</param>
    /// <param name="pair">Par de colisionadores cuya interacción está siendo procesada.</param>
    /// <param name="childIndexA">Índice del hijo del primer colisionador en el par.</param>
    /// <param name="childIndexB">Índice del hijo del segundo colisionador en el par.</param>
    /// <param name="manifold">Colector de contactos que contiene información de los puntos de contacto detectados.</param>
    /// <returns>
    /// Devuelve un valor booleano indicando si la configuración del colector de contactos fue exitosa.
    /// </returns>
    public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB,
        ref ConvexContactManifold manifold)
    {
        return true;
    }

    public void Dispose()
    {
        throw new NotImplementedException();
    }
}