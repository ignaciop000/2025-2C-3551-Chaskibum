using System;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;
using TGC.MonoGame.TP;

namespace TGC.MonoGame.Samples.Cameras
{
    /// <summary>
    /// Cámara que orbita alrededor de un objetivo específico usando controles de mouse.
    /// </summary>
    public class OrbitCamera : Camera
    {
        // --- Camera Shake ---
        private float _shakeTimeLeft = 0f, _shakeDuration = 0f;
        private float _shakeTranslation = 0f, _shakeRoll = 0f;
        private float _shakePhaseX = 0f, _shakePhaseY = 0f, _shakePhaseR = 0f;
        private float _shakeFreqX = 42f, _shakeFreqY = 33f, _shakeFreqR = 25f;
        private readonly System.Random _rng = new System.Random();

        public bool IsShaking => _shakeTimeLeft > 0f;
        // Propiedades de la órbita
        public Vector3 Target { get; set; }
        public float Distance { get; set; }
        public float MinDistance { get; set; } = 50f;
        public float MaxDistance { get; set; } = 2000f;
        
        // Ángulos de rotación
        private float _yaw = -50f;   // Rotación horizontal
        private float _pitch = 30f; // Rotación vertical (empezamos ligeramente hacia abajo)
        
        // Configuración de controles
        public float MouseSensitivity { get; set; } = 0.3f;
        public float ZoomSpeed { get; set; } = 500f;
        
        // Control de mouse
        private Vector2 _lastMousePosition;
        private bool _isMouseControlActive = false;

        public OrbitCamera(float aspectRatio, Vector3 target, float distance = 800f) : base(aspectRatio)
        {
            Target = target;
            Distance = MathHelper.Clamp(distance, MinDistance, MaxDistance);
            _lastMousePosition = Mouse.GetState().Position.ToVector2();
            UpdatePosition();
        }

        public OrbitCamera(float aspectRatio, Vector3 target, float distance, float nearPlane, float farPlane) 
            : base(aspectRatio, nearPlane, farPlane)
        {
            Target = target;
            Distance = MathHelper.Clamp(distance, MinDistance, MaxDistance);
            _lastMousePosition = Mouse.GetState().Position.ToVector2();
            UpdatePosition();
        }

        public override void Update(GameTime gameTime)
        {
            var deltaTime = (float)gameTime.ElapsedGameTime.TotalSeconds;
            var mouseState = Mouse.GetState();
            var currentMousePosition = mouseState.Position.ToVector2();
            
            // Activar control de cámara con clic derecho
            if (mouseState.RightButton == ButtonState.Pressed)
            {
                if (!_isMouseControlActive)
                {
                    _isMouseControlActive = true;
                    _lastMousePosition = currentMousePosition;
                }
                else
                {
                    // Calcular movimiento del mouse
                    var mouseDelta = currentMousePosition - _lastMousePosition;
                    
                    // Aplicar sensibilidad
                    mouseDelta *= MouseSensitivity;
                    
                    // Actualizar ángulos
                    _yaw += mouseDelta.X;
                    _pitch += mouseDelta.Y;
                    
                    // Limitar el pitch para evitar que la cámara se voltee
                    _pitch = MathHelper.Clamp(_pitch, -89f, 89f);
                    
                    UpdatePosition();
                }
                _lastMousePosition = currentMousePosition;
            }
            else
            {
                _isMouseControlActive = false;
                _lastMousePosition = currentMousePosition;
            }
            
            // Control de zoom con la rueda del mouse
            var scrollDelta = mouseState.ScrollWheelValue - _lastScrollValue;
            if (scrollDelta != 0)
            {
                Distance -= scrollDelta * ZoomSpeed * 0.001f;
                Distance = MathHelper.Clamp(Distance, MinDistance, MaxDistance);
                UpdatePosition();
            }
            _lastScrollValue = mouseState.ScrollWheelValue;
            
            if (_shakeTimeLeft > 0f)
            {
                _shakeTimeLeft = MathF.Max(0f, _shakeTimeLeft - deltaTime);
            }
        }
        
        private int _lastScrollValue = 0;

        private void UpdatePosition()
        {
            // Convertir ángulos a radianes
            float yawRad = MathHelper.ToRadians(_yaw);
            float pitchRad = MathHelper.ToRadians(_pitch);
            
            // Calcular posición de la cámara usando coordenadas esféricas
            float x = Distance * MathF.Cos(pitchRad) * MathF.Cos(yawRad);
            float y = Distance * MathF.Sin(pitchRad);
            float z = Distance * MathF.Cos(pitchRad) * MathF.Sin(yawRad);
            
            Position = Target + new Vector3(x, y, z);
            
            // Actualizar vectores de dirección
            FrontDirection = Vector3.Normalize(Target - Position);
            RightDirection = Vector3.Normalize(Vector3.Cross(Vector3.Up, FrontDirection));
            UpDirection = Vector3.Cross(FrontDirection, RightDirection);
            
            // Crear matriz de vista
            View = Matrix.CreateLookAt(Position, Target, UpDirection);

            //sacudida
            if (_shakeTimeLeft > 0f && (_shakeTranslation > 0f || _shakeRoll > 0f))
            {
                var t = 1f - (_shakeTimeLeft / _shakeDuration);  
                var falloff = MathF.Exp(-6f * t);             // decaimiento suave
                var elapsed = _shakeDuration - _shakeTimeLeft;

                var ox = MathF.Sin(_shakePhaseX + elapsed * _shakeFreqX) * _shakeTranslation * falloff;
                var oy = MathF.Cos(_shakePhaseY + elapsed * _shakeFreqY) * _shakeTranslation * 0.6f * falloff;
                var roll = MathF.Sin(_shakePhaseR + elapsed * _shakeFreqR) * _shakeRoll * falloff;

                var s = Matrix.CreateRotationZ(roll) * Matrix.CreateTranslation(ox, oy, 0f);
                View = s * View;           // premultiplico en espacio cámara
            }
        }

        /// <summary>
        /// Establece el objetivo de la cámara y actualiza la posición
        /// </summary>
        /// <param name="target">Nueva posición objetivo</param>
        public void SetTarget(Vector3 target)
        {
            Target = target;
            UpdatePosition();
        }
        
        /// <summary>
        /// Establece la distancia de la cámara al objetivo
        /// </summary>
        /// <param name="distance">Nueva distancia</param>
        public void SetDistance(float distance)
        {
            Distance = MathHelper.Clamp(distance, MinDistance, MaxDistance);
            UpdatePosition();
        }
        
        /// <summary>
        /// Inicia un sacudido de cámara en espacio de vista.
        /// amplitude: traslación en unidades de mundo (en el plano de la pantalla).
        /// duration: segundos.
        /// rotational: roll (radianes). 0 si no querés giro.
        /// </summary>
        public void StartShake(float amplitude, float duration, float rotational = 0f)
        {
            _shakeTranslation = amplitude;
            _shakeRoll  = rotational;
            _shakeDuration = MathF.Max(0.0001f, duration);
            _shakeTimeLeft = _shakeDuration;

            _shakePhaseX = (float)_rng.NextDouble() * MathF.Tau;
            _shakePhaseY = (float)_rng.NextDouble() * MathF.Tau;
            _shakePhaseR = (float)_rng.NextDouble() * MathF.Tau;
            
            _shakeFreqX = 42f;
            _shakeFreqY = 33f;
            _shakeFreqR = 25f;
        }
    }
}