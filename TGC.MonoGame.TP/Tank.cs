using System;
using BepuPhysics;
using BepuPhysics.Collidables;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using System.Numerics;
using Vector3 = Microsoft.Xna.Framework.Vector3;

namespace TGC.MonoGame.TP
{
    public class Tank
    {
        private Model _model;
        private Effect _effect;
        private Matrix _world;

        // Propiedades de movimiento
        public Vector3 Position { get; private set; }
        public float Rotation { get; private set; }
        public float Scale { get; private set; }

        // Agregar estas propiedades si no las tienes
        public float PitchRotation { get; private set; } // Inclinación hacia adelante/atrás
        public float RollRotation { get; private set; } // Inclinación lateral


        // Parámetros de movimiento
        private const float MovementSpeed = 200f;
        private const float RotationSpeed = 2f;

        // Física
        private BodyHandle _physicsBody;
        private Simulation _simulation;
        private Terrain _terrain;


        public Tank(Vector3 initialPosition, float initialRotation = 0f, float scale = 20f)
        {
            Position = initialPosition;
            Rotation = initialRotation;
            Scale = scale;
            UpdateWorldMatrix();
        }

        public void CargarModelo(string rutaRelativa, Effect efecto, ContentManager content, Simulation simulation,
            Terrain terrain = null)
        {
            _effect = efecto;
            _simulation = simulation;
            _terrain = terrain; // Guardar referencia al terreno


            // Cargar modelo
            _model = content.Load<Model>(TGCGame.ContentFolder3D + rutaRelativa);

            // Asignar efecto a todas las partes del modelo
            foreach (var mesh in _model.Meshes)
            {
                foreach (var meshPart in mesh.MeshParts)
                {
                    meshPart.Effect = efecto;
                }
            }

            // Crear cuerpo físico
            CreatePhysicsBody();
        }

        private void CreatePhysicsBody()
        {
            if (_simulation == null) return;

            // Crear una caja para representar el tanque en la física
            var box = new Box(Scale * 0.1f, Scale * 0.05f, Scale * 0.15f); // Ajusta el tamaño según necesites
            var boxIndex = _simulation.Shapes.Add(box);

            // Crear cuerpo dinámico
            var bodyDescription = BodyDescription.CreateDynamic(
                new System.Numerics.Vector3(Position.X, Position.Y, Position.Z),
                new BodyInertia { InverseMass = 1f / 1000f }, // Masa de 1000kg
                new CollidableDescription(boxIndex, 0.1f),
                new BodyActivityDescription(0.01f)
            );

            _physicsBody = _simulation.Bodies.Add(bodyDescription);
        }

        public void Update(GameTime gameTime, KeyboardState keyboardState)
        {
            float deltaTime = (float)gameTime.ElapsedGameTime.TotalSeconds;
            bool moved = false;

            // Movimiento hacia adelante/atrás
            if (keyboardState.IsKeyDown(Keys.W))
            {
                MoveForward(deltaTime);
                moved = true;
            }

            if (keyboardState.IsKeyDown(Keys.S))
            {
                MoveBackward(deltaTime);
                moved = true;
            }

            // Rotación
            if (keyboardState.IsKeyDown(Keys.A))
            {
                RotateLeft(deltaTime);
                moved = true;
            }

            if (keyboardState.IsKeyDown(Keys.D))
            {
                RotateRight(deltaTime);
                moved = true;
            }

            if (_terrain != null)
            {
                float terrainHeight = _terrain.GetHeightAtPosition(Position.X, Position.Z);
                System.Console.WriteLine($"Tank pos: {Position.X}, {Position.Z} - Terrain height: {terrainHeight}");
                // Reducir o eliminar el offset, o hacerlo negativo si el modelo está muy alto
                Position = new Vector3(Position.X, terrainHeight - Scale * 0.02f,
                    Position.Z); // Prueba con valores diferentes
            }


            // Actualizar física si se movió
            if (moved)
            {
                UpdatePhysicsBody();
            }

            // Sincronizar posición desde la física
            if (_simulation != null && _physicsBody.Value >= 0)
            {
                SyncFromPhysics();
            }

            UpdateWorldMatrix();
        }

        private void MoveForward(float deltaTime)
        {
            Vector3 forward = Vector3.Transform(-Vector3.UnitZ, Matrix.CreateRotationY(Rotation));
            Position += forward * MovementSpeed * deltaTime;
        }

        private void MoveBackward(float deltaTime)
        {
            Vector3 forward = Vector3.Transform(-Vector3.UnitZ, Matrix.CreateRotationY(Rotation));
            Position -= forward * MovementSpeed * deltaTime;
        }

        private void RotateLeft(float deltaTime)
        {
            Rotation += RotationSpeed * deltaTime; // Cambiado de -= a +=
        }

        private void RotateRight(float deltaTime)
        {
            Rotation -= RotationSpeed * deltaTime; // Cambiado de += a -=
        }

        private void UpdatePhysicsBody()
        {
            if (_simulation != null && _physicsBody.Value >= 0)
            {
                // Actualizar posición en la física
                var bodyReference = _simulation.Bodies.GetBodyReference(_physicsBody);
                bodyReference.Pose.Position = new System.Numerics.Vector3(Position.X, Position.Y, Position.Z);
                bodyReference.Pose.Orientation = System.Numerics.Quaternion.CreateFromYawPitchRoll(Rotation, 0, 0);

                // Activar el cuerpo para que se actualice
                bodyReference.Activity.SleepThreshold = -1;
            }
        }

        private void SyncFromPhysics()
        {
            var bodyReference = _simulation.Bodies.GetBodyReference(_physicsBody);
            var physicsPosition = bodyReference.Pose.Position;

            Position = new Vector3(physicsPosition.X, physicsPosition.Y, physicsPosition.Z);

            // Extraer rotación Y del quaternion
            var orientation = bodyReference.Pose.Orientation;
            Rotation = MathF.Atan2(2.0f * (orientation.W * orientation.Y + orientation.X * orientation.Z),
                1.0f - 2.0f * (orientation.Y * orientation.Y + orientation.Z * orientation.Z));
        }

        private void UpdateWorldMatrix()
        {
            // Calcular inclinación basada en el terreno
            CalculateTerrainRotation();

            // Ajustar la posición Y para compensar el pivot del modelo
            float modelHeightOffset = -Scale * 0.5f; // Ajusta este valor según tu modelo

            _world = Matrix.CreateScale(Scale, Scale, Scale) *
                     Matrix.CreateRotationX(PitchRotation) *
                     Matrix.CreateRotationZ(-RollRotation) *
                     Matrix.CreateRotationY(Rotation) *
                     Matrix.CreateTranslation(Position.X, Position.Y + modelHeightOffset, Position.Z);
        }

        private void CalculateTerrainRotation()
        {
            if (_terrain == null)
            {
                PitchRotation = 0f;
                RollRotation = 0f;
                return;
            }

            float sampleDistance = 10f; // Distancia para samplear el terreno (ajusta según necesites)

            // Obtener vectores de dirección basados en la rotación actual del tanque
            Vector3 forward = Vector3.Transform(Vector3.Forward, Matrix.CreateRotationY(Rotation));
            Vector3 right = Vector3.Transform(Vector3.Right, Matrix.CreateRotationY(Rotation));

            // Obtener alturas en diferentes puntos alrededor del tanque
            float heightCenter = _terrain.GetHeightAtPosition(Position.X, Position.Z);
            float heightForward = _terrain.GetHeightAtPosition(
                Position.X + forward.X * sampleDistance,
                Position.Z + forward.Z * sampleDistance);
            float heightRight = _terrain.GetHeightAtPosition(
                Position.X + right.X * sampleDistance,
                Position.Z + right.Z * sampleDistance);

            // Calcular inclinaciones usando arctangente
            PitchRotation = MathF.Atan2(heightForward - heightCenter, sampleDistance);
            RollRotation = MathF.Atan2(heightRight - heightCenter, sampleDistance);

            // Opcional: Limitar las inclinaciones máximas para evitar volcamientos extremos
            float maxTilt = MathHelper.ToRadians(45f); // Máximo 45 grados
            PitchRotation = MathHelper.Clamp(PitchRotation, -maxTilt, maxTilt);
            RollRotation = MathHelper.Clamp(RollRotation, -maxTilt, maxTilt);
        }


        public void Draw()
        {
            if (_model == null || _effect == null) return;

            var modelMeshesBaseTransforms = new Matrix[_model.Bones.Count];
            _model.CopyAbsoluteBoneTransformsTo(modelMeshesBaseTransforms);

            foreach (var mesh in _model.Meshes)
            {
                var relativeTransform = modelMeshesBaseTransforms[mesh.ParentBone.Index];
                _effect.Parameters["World"].SetValue(relativeTransform * _world);
                mesh.Draw();
            }
        }
    }
}