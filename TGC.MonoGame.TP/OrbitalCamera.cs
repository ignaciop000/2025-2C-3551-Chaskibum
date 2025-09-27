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
        public float ZoomSpeed { get; set; } = 50f;
        
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
                    _yaw -= mouseDelta.X;
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
    }
}