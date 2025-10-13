using System;
using System.Numerics;
using BepuPhysics;
using BepuUtilities;

namespace TGC.MonoGame.TP;

/// <summary>
/// Estructura utilizada para implementar los "callbacks" necesarios para la integración de las poses en el sistema físico.
/// Define la aplicación de fuerzas externas, como la gravedad, y permite personalizar el comportamiento de integración
/// de velocidades y posición de los cuerpos durante el tiempo de simulación.
/// </summary>
public struct PoseIntegratorCallbacks() : IPoseIntegratorCallbacks
{
    /// <summary>
    /// Propiedad que define el vector de gravedad usado en la simulación física.
    /// Representa la fuerza gravitacional que se
    public Vector3 Gravity;

    /// <summary>
    /// Propiedad que define el coeficiente de amortiguamiento lineal aplicado a los cuerpos en la simulación física.
    /// Este coeficiente determina la tasa a la que se reduce la velocidad lineal de los objetos con el tiempo.
    /// Un valor más alto indica un desaceleramiento más rápido, simulando efectos como la resistencia del aire
    /// o la fricción generalizada en un entorno simulado.
    /// </summary>
    public float LinearDamping;

    /// <summary>
    /// Propiedad que define el coeficiente de amortiguación angular aplicado a los cuerpos
    /// en la simulación física. Este coeficiente controla la cantidad de atenuación que se
    /// aplica a las velocidades angulares de los cuerpos a lo largo del tiempo, reduciendo
    /// gradualmente la rotación de los objetos hasta que cesa por completo.
    /// Un mayor valor de AngularDamping resultará en una desaceleración más rápida de los
    /// movimientos rotacionales, mientras que valores menores permitirán que los objetos
    /// mantengan su rotación por más tiempo. Es una herramienta crucial para modelar el
    /// comportamiento físico deseado, como evitar rotaciones perpetuas en sistemas sin fricción.
    /// </summary>
    public float AngularDamping;

    /// <summary>
    /// Variable que almacena los valores de gravedad multiplicados por el intervalo de tiempo (dt) en formato de vectores anchos (SIMD).
    /// Esta optimización permite precalcular y evitar cómputos redundantes durante cada integración de las velocidades.
    /// Es utilizada dentro de los "callbacks" del integrador de poses para aplicar las fuerzas gravitatorias de manera eficiente
    /// a múltiples cuerpos simultáneamente durante la simulación física.
    /// </summary>
    private Vector3Wide GravityWideDt;

    /// <summary>
    /// Variable que representa el factor de amortiguamiento lineal aplicado por unidad de tiempo en la simulación.
    /// Este valor se calcula a partir de un coeficiente inicial de amortiguamiento lineal y el paso temporal
    /// de integración (dt). Controla la reducción progresiva de la velocidad lineal de los cuerpos en la simulación,
    /// simulando la resistencia o pérdida de energía debido a factores como la fricción con el medio ambiente.
    /// Ajustar este valor impacta directamente la estabilidad del sistema físico y la forma en la que los objetos
    /// desaceleran a lo largo del tiempo.
    /// </summary>
    private Vector<float> LinearDampingDt;

    /// <summary>
    /// Variable que define el amortiguamiento angular aplicado durante la integración
    /// temporal en el sistema de simulación física. Este valor se utiliza para reducir
    /// gradualmente las velocidades angulares de los cuerpos, simulando el efecto de
    /// fricción rotacional o pérdida de energía angular.
    /// El cálculo de AngularDampingDt depende del valor del amortiguamiento angular
    /// configurado y del intervalo de tiempo de simulación. Este parámetro es crucial
    /// para mantener la estabilidad de la simulación y controlar el comportamiento
    /// dinámico de los objetos en rotación.
    /// </summary>
    private Vector<float> AngularDampingDt;

    public PoseIntegratorCallbacks(Vector3 gravity) : this()
    {
        Gravity = gravity;
        LinearDamping = 0f;
        AngularDamping = 0f;
        GravityWideDt = default;
        LinearDampingDt = default;
        AngularDampingDt = default;
    }

    public void Initialize(Simulation simulation)
    {
    }

    /// <summary>
    /// Prepara los cálculos previos necesarios para la integración del sistema físico.
    /// Calcula y almacena las amortiguaciones lineal y angular ajustadas al intervalo de tiempo.
    /// </summary>
    /// <param name="dt">Intervalo de tiempo para el cual se realizan los cálculos de integración.</param>
    public void PrepareForIntegration(float dt)
    {
        LinearDampingDt = new Vector<float>(MathF.Pow(MathHelper.Clamp(1 - LinearDamping, 0, 1), dt));
        AngularDampingDt = new Vector<float>(MathF.Pow(MathHelper.Clamp(1 - AngularDamping, 0, 1), dt));
        GravityWideDt = Vector3Wide.Broadcast(Gravity * dt);
    }

    public void IntegrateVelocity(Vector<int> bodyIndices, Vector3Wide position, QuaternionWide orientation,
        BodyInertiaWide localInertia, Vector<int> integrationMask, int workerIndex, Vector<float> dt,
        ref BodyVelocityWide velocity)
    {
        velocity.Linear = velocity.Linear + GravityWideDt;
        velocity.Linear *= LinearDampingDt;
        velocity.Angular *= AngularDampingDt;
    }

    public AngularIntegrationMode AngularIntegrationMode { get; } = AngularIntegrationMode.Nonconserving;
    public bool AllowSubstepsForUnconstrainedBodies { get; } = false;
    public bool IntegrateVelocityForKinematics { get; } = false;
}