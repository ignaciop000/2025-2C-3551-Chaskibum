using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace TGC.MonoGame.TP;

public class Ground
{
    private VertexBuffer vertexBuffer {get;set;}
    private IndexBuffer indexBuffer {get;set;}
    public Effect Effect {get; set;}
    public Matrix World { get; set; } = Matrix.Identity;
    
    public Ground(GraphicsDevice graphicsDevice)
    {
        var vertices = new[]
        {
            new VertexPositionColor(new Vector3(-20000f, -0.5f, -20000f),Color.Red),
            new VertexPositionColor(new Vector3(-20000f, -0.5f, 20000f),Color.Red),
            new VertexPositionColor(new Vector3(20000f, -0.5f, 20000f),Color.Red),
            new VertexPositionColor(new Vector3(20000f, -0.5f, -20000f),Color.Red)
        };
        
        var indices = new ushort[]
        {
            2, 1, 0,
            0, 3, 2
        };
        
        vertexBuffer = new VertexBuffer(graphicsDevice, VertexPositionColor.VertexDeclaration, vertices.Length, BufferUsage.None);
        vertexBuffer.SetData(vertices);

        indexBuffer = new IndexBuffer(graphicsDevice, IndexElementSize.SixteenBits, 6, BufferUsage.None);
        indexBuffer.SetData(indices);
        
    }

    public void Draw(GraphicsDevice graphicsDevice, Matrix view, Matrix projection)
    {
        graphicsDevice.SetVertexBuffer(vertexBuffer);
        graphicsDevice.Indices = indexBuffer;
        
        Effect.Parameters["World"].SetValue(World);
        Effect.Parameters["View"].SetValue(view);
        Effect.Parameters["Projection"].SetValue(projection);
        
        foreach(var pass in Effect.CurrentTechnique.Passes)
        {
            pass.Apply();
            graphicsDevice.DrawIndexedPrimitives(PrimitiveType.TriangleList, 0, 0, 2);
        }
    }
}