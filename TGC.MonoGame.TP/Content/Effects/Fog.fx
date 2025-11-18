#if OPENGL
    #define SV_POSITION POSITION
    #define VS_SHADERMODEL vs_3_0
    #define PS_SHADERMODEL ps_3_0
#else
    #define VS_SHADERMODEL vs_4_0_level_9_1
    #define PS_SHADERMODEL ps_4_0_level_9_1
#endif

// Textura de la escena renderizada
texture SceneTexture;
sampler2D SceneSampler = sampler_state
{
    Texture = <SceneTexture>;
    MinFilter = Linear;
    MagFilter = Linear;
    MipFilter = Linear;
    AddressU = Clamp;
    AddressV = Clamp;
};

// Textura de profundidad del render target
texture DepthTexture;
sampler2D DepthSampler = sampler_state
{
    Texture = <DepthTexture>;
    MinFilter = Point;
    MagFilter = Point;
    MipFilter = Point;
    AddressU = Clamp;
    AddressV = Clamp;
};

// Parámetros de niebla
float3 FogColor = float3(0.3, 0.3, 0.3); // Color de niebla gris oscuro
float FogStart = 1500.0;
float FogEnd = 3500.0;
float3 CameraPosition;
float4x4 InverseViewProjection;
float CameraNearPlane;
float CameraFarPlane;

struct VertexShaderInput
{
    float4 Position : POSITION0;
    float2 TexCoord : TEXCOORD0;
};

struct VertexShaderOutput
{
    float4 Position : SV_POSITION;
    float2 TexCoord : TEXCOORD0;
};

VertexShaderOutput MainVS(VertexShaderInput input)
{
    VertexShaderOutput output;
    output.Position = input.Position;
    output.TexCoord = input.TexCoord;
    return output;
}

float4 MainPS(VertexShaderOutput input) : COLOR
{
    // Obtener el color de la escena
    float4 sceneColor = tex2D(SceneSampler, input.TexCoord);
    
    // Aproximación: usar la coordenada Y de la pantalla para estimar distancia
    // La parte superior (y=0) corresponde al horizonte (lejos)
    // La parte inferior (y=1) corresponde al suelo cercano
    
    float screenY = input.TexCoord.y;
    
    // Estimar distancia basada en la altura en pantalla
    // Mapear de forma que el horizonte (arriba) sea lejos
    float estimatedDistance = lerp(FogEnd * 2.0, 0.0, screenY);
    
    // Calcular factor de niebla
    float fogFactor = saturate((estimatedDistance - FogStart) / (FogEnd - FogStart));
    
    // Hacer que la transición sea más suave
    fogFactor = smoothstep(0.0, 1.0, fogFactor);
    
    // Mezclar color de escena con color de niebla
    float3 finalColor = lerp(sceneColor.rgb, FogColor, fogFactor * 0.8);
    
    return float4(finalColor, sceneColor.a);
}

technique Fog
{
    pass P0
    {
        VertexShader = compile VS_SHADERMODEL MainVS();
        PixelShader = compile PS_SHADERMODEL MainPS();
    }
}
