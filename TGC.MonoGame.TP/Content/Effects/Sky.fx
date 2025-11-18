#if OPENGL
    #define SV_POSITION POSITION
    #define VS_SHADERMODEL vs_3_0
    #define PS_SHADERMODEL ps_3_0
#else
    #define VS_SHADERMODEL vs_4_0_level_9_1
    #define PS_SHADERMODEL ps_4_0_level_9_1
#endif

float4x4 WorldViewProjection;

// Colores del cielo
float3 SkyColorTop = float3(0.25, 0.45, 0.8);      // Azul cielo arriba
float3 SkyColorHorizon = float3(0.7, 0.8, 0.95);   // Azul claro en el horizonte

struct VSInput
{
    float4 Position : POSITION0;
};

struct VSOutput
{
    float4 Position : SV_POSITION;
    float3 Dir      : TEXCOORD0;
};

VSOutput MainVS(VSInput input)
{
    VSOutput output;

    // Usamos la posición en espacio local como dirección aproximada
    output.Dir = normalize(input.Position.xyz);

    output.Position = mul(input.Position, WorldViewProjection);
    return output;
}

float4 MainPS(VSOutput input) : COLOR
{
    float y = saturate(input.Dir.y * 0.5 + 0.5);  // -1..1 -> 0..1
    float3 color = lerp(SkyColorHorizon, SkyColorTop, y);
    return float4(color, 1.0);
}

technique Sky
{
    pass P0
    {
        VertexShader = compile VS_SHADERMODEL MainVS();
        PixelShader  = compile PS_SHADERMODEL MainPS();
    }
}
