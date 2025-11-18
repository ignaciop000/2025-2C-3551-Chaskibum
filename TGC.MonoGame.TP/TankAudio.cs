using Microsoft.Xna.Framework.Audio;
using Microsoft.Xna.Framework.Content;

namespace TGC.MonoGame.TP;

/// <summary>
/// Maneja todos los sonidos del tanque durante el gameplay
/// </summary>
public class TankAudio
{
    // Sonidos del tanque
    private SoundEffect _engineSound;
    private SoundEffect _turboSound;
    private SoundEffect _shootLightSound;  // Disparo ligero
    private SoundEffect _shootHeavySound;  // Disparo pesado
    private SoundEffect _brakeSound;
    private SoundEffect _collisionSound;

    // Instancia del motor (para loop)
    private SoundEffectInstance _engineInstance;
    private SoundEffectInstance _turboInstance;

    // Estado del motor
    private bool _enginePlaying = false;
    private bool _turboPlaying = false;
    private float _previousSpeed = 0f;

    // Cooldown para colisiones (evitar spam de sonido)
    private float _collisionCooldown = 0f;
    private const float CollisionCooldownTime = 3.0f; // 3 segundos entre colisiones para mayor separación

    /// <summary>
    /// Carga todos los sonidos del tanque
    /// </summary>
    public void LoadContent(ContentManager content)
    {
        try
        {
            _engineSound = content.Load<SoundEffect>("Sounds/tank_engine");
            _engineInstance = _engineSound.CreateInstance();
            _engineInstance.IsLooped = true;
            _engineInstance.Volume = 0.2f;
        }
        catch
        {
            // Motor no encontrado
        }

        try
        {
            _turboSound = content.Load<SoundEffect>("Sounds/tank_turbo");
            _turboInstance = _turboSound.CreateInstance();
            _turboInstance.IsLooped = true;
            _turboInstance.Volume = 0.25f;
        }
        catch
        {
            // Turbo no encontrado
        }

        try
        {
            _shootLightSound = content.Load<SoundEffect>("Sounds/tank_shoot_light");
        }
        catch
        {
            // Disparo ligero no encontrado
        }

        try
        {
            _shootHeavySound = content.Load<SoundEffect>("Sounds/tank_shoot_heavy");
        }
        catch
        {
            // Disparo pesado no encontrado
        }

        try
        {
            _brakeSound = content.Load<SoundEffect>("Sounds/tank_brake");
        }
        catch
        {
            // Freno no encontrado
        }

        try
        {
            _collisionSound = content.Load<SoundEffect>("Sounds/tank_collision");
        }
        catch
        {
            // Colisión no encontrada
        }
    }

    /// <summary>
    /// Actualiza el sonido del motor según el movimiento del tanque
    /// </summary>
    public void UpdateEngine(float currentSpeed, float deltaTime, bool isTurbo = false)
    {
        if (_engineInstance == null) return;

        // Actualizar cooldown de colisión
        if (_collisionCooldown > 0f)
            _collisionCooldown -= deltaTime;

        var isMoving = System.Math.Abs(currentSpeed) > 0.1f;

        if (isMoving)
        {
            if (isTurbo)
            {
                // Modo TURBO activado
                
                // Bajar volumen del motor normal
                if (_enginePlaying)
                {
                    _engineInstance.Volume = System.Math.Max(0.05f, _engineInstance.Volume - deltaTime * 2f);
                }
                
                // Activar sonido de turbo
                if (!_turboPlaying && _turboInstance != null)
                {
                    _turboInstance.Play();
                    _turboPlaying = true;
                }

                // Ajustar pitch y volumen del turbo
                if (_turboInstance != null)
                {
                    var normalizedSpeed = System.Math.Clamp(System.Math.Abs(currentSpeed) / 40f, 0f, 1f);
                    _turboInstance.Pitch = 0f + (normalizedSpeed * 0.3f); // 0.0 a 0.3 (más agudo)
                    _turboInstance.Volume = 0.2f + (normalizedSpeed * 0.15f); // 0.2 a 0.35
                }
            }
            else
            {
                // Modo NORMAL
                
                // Desactivar turbo si estaba activo
                if (_turboPlaying)
                {
                    _turboInstance?.Stop();
                    _turboPlaying = false;
                }
                
                // Arrancar motor si no está sonando
                if (!_enginePlaying)
                {
                    _engineInstance.Play();
                    _enginePlaying = true;
                }

                // Variar pitch según velocidad (0.8 a 1.2)
                var normalizedSpeed = System.Math.Clamp(System.Math.Abs(currentSpeed) / 30f, 0f, 1f);
                _engineInstance.Pitch = -0.2f + (normalizedSpeed * 0.4f);
                _engineInstance.Volume = 0.15f + (normalizedSpeed * 0.1f); // 0.15 a 0.25
            }
        }
        else
        {
            // Detener motor si estaba sonando
            if (_enginePlaying)
            {
                _engineInstance.Stop();
                _enginePlaying = false;
            }
            
            // Detener turbo si estaba sonando
            if (_turboPlaying)
            {
                _turboInstance?.Stop();
                _turboPlaying = false;
            }
        }

        _previousSpeed = currentSpeed;
    }

    /// <summary>
    /// Reproduce el sonido de disparo según el tipo de proyectil
    /// </summary>
    public void PlayShoot(ProjectileType projectileType)
    {
        try
        {
            // Determinar qué sonido reproducir según el tipo
            if (projectileType == ProjectileTypes.Heavy)
            {
                _shootHeavySound?.Play(volume: 0.4f, pitch: -0.1f, pan: 0f); // Más grave y fuerte
            }
            else // Light o cualquier otro
            {
                _shootLightSound?.Play(volume: 0.3f, pitch: 0.1f, pan: 0f); // Más agudo y suave
            }
        }
        catch
        {
            // Ignorar errores
        }
    }

    /// <summary>
    /// Reproduce el sonido de freno
    /// </summary>
    public void PlayBrake()
    {
        try
        {
            // Solo reproducir si el motor está sonando (tanque se está moviendo)
            if (_enginePlaying)
            {
                _brakeSound?.Play(volume: 0.3f, pitch: 0f, pan: 0f);
            }
        }
        catch
        {
            // Ignorar errores
        }
    }

    /// <summary>
    /// Reproduce el sonido de colisión
    /// </summary>
    public void PlayCollision(float intensity = 1f)
    {
        // Evitar spam de sonido de colisión
        if (_collisionCooldown > 0f) return;

        try
        {
            // Volumen muy reducido (0.05 a 0.10)
            var volume = System.Math.Clamp(0.05f + (intensity * 0.05f), 0.05f, 0.10f);
            _collisionSound?.Play(volume: volume, pitch: 0f, pan: 0f);
            _collisionCooldown = CollisionCooldownTime;
        }
        catch
        {
            // Ignorar errores
        }
    }

    /// <summary>
    /// Detiene todos los sonidos (para cuando el tanque muere o sale del juego)
    /// </summary>
    public void StopAll()
    {
        try
        {
            _engineInstance?.Stop();
            _enginePlaying = false;
            
            _turboInstance?.Stop();
            _turboPlaying = false;
        }
        catch
        {
            // Ignorar errores
        }
    }

    /// <summary>
    /// Limpia recursos
    /// </summary>
    public void Dispose()
    {
        StopAll();
        _engineInstance?.Dispose();
        _turboInstance?.Dispose();
    }
}
