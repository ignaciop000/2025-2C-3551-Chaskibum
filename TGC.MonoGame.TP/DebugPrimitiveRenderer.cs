namespace TGC.MonoGame.TP;

using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

public static class DebugPrimitiveRenderer
{
    private static VertexPositionColor[] _cubeVerts;
    private static short[] _cubeIndices;

    static DebugPrimitiveRenderer()
    {
        _cubeVerts = new VertexPositionColor[8];
        _cubeVerts[0].Position = new Vector3(-0.5f, -0.5f, -0.5f);
        _cubeVerts[1].Position = new Vector3(-0.5f, -0.5f,  0.5f);
        _cubeVerts[2].Position = new Vector3(-0.5f,  0.5f, -0.5f);
        _cubeVerts[3].Position = new Vector3(-0.5f,  0.5f,  0.5f);
        _cubeVerts[4].Position = new Vector3( 0.5f, -0.5f, -0.5f);
        _cubeVerts[5].Position = new Vector3( 0.5f, -0.5f,  0.5f);
        _cubeVerts[6].Position = new Vector3( 0.5f,  0.5f, -0.5f);
        _cubeVerts[7].Position = new Vector3( 0.5f,  0.5f,  0.5f);

        for (int i = 0; i < _cubeVerts.Length; i++)
            _cubeVerts[i].Color = Color.Red;

        _cubeIndices = new short[]
        {
            0,1,2, 1,3,2, // Left
            4,6,5, 5,6,7, // Right
            0,4,1, 1,4,5, // Bottom
            2,3,6, 3,7,6, // Top
            0,2,4, 2,6,4, // Back
            1,5,3, 3,5,7  // Front
        };
    }

    public static void DrawCube(GraphicsDevice device)
    {
        device.DrawUserIndexedPrimitives(
            PrimitiveType.TriangleList,
            _cubeVerts,
            0,
            8,
            _cubeIndices,
            0,
            _cubeIndices.Length / 3);
    }
}