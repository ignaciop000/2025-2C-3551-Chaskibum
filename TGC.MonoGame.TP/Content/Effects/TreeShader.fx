#if OPENGL
	#define SV_POSITION POSITION
	#define VS_SHADERMODEL vs_3_0
	#define PS_SHADERMODEL ps_3_0
#else
	#define VS_SHADERMODEL vs_4_0_level_9_1
	#define PS_SHADERMODEL ps_4_0_level_9_1
#endif

float4x4 World;
float4x4 View;
float4x4 Projection;

// AÑADIDO: El parámetro que espera ModelInstances.cs
float3 DiffuseColor;

// NUEVO: Parámetro para identificar el tipo de árbol (0=tree, 1=tree2, 2=tree3)
int TreeType;

// Dos texturas: una para la corteza y otra para las hojas
texture BarkTexture;
texture LeavesTexture;

sampler BarkSampler = sampler_state
{
    Texture = <BarkTexture>;
    MinFilter = Linear;
    MagFilter = Linear;
    AddressU = Wrap;
    AddressV = Wrap;
};

sampler LeavesSampler = sampler_state
{
    Texture = <LeavesTexture>;
    MinFilter = Linear;
    MagFilter = Linear;
    AddressU = Wrap;
    AddressV = Wrap;
};

struct VertexShaderInput
{
    float4 Position : POSITION0;
    float2 TextureCoordinate : TEXCOORD0;
    float3 Normal : NORMAL0; // Cambiar de Color a Normal para que funcione con los modelos
};

struct VertexShaderOutput
{
    float4 Position : SV_POSITION;
    float2 TextureCoordinate : TEXCOORD0;
    float3 Normal : TEXCOORD1;
    float3 LocalPosition : TEXCOORD2; // Posición local del objeto
};

VertexShaderOutput MainVS(in VertexShaderInput input)
{
    VertexShaderOutput output = (VertexShaderOutput)0;
    
    float4 worldPosition = mul(input.Position, World);
    float4 viewPosition = mul(worldPosition, View);	
    output.Position = mul(viewPosition, Projection);

    output.TextureCoordinate = input.TextureCoordinate;
    output.Normal = mul(input.Normal, (float3x3)World);
    output.LocalPosition = input.Position.xyz;

    return output;
}

float4 MainPS(VertexShaderOutput input) : COLOR
{
    float2 uv = input.TextureCoordinate;
    float3 localPos = input.LocalPosition;
    
    // Calcular distancia radial desde el centro (eje Y vertical)
    float distanceFromCenter = length(float2(localPos.x, localPos.z));
    
    // POR DEFECTO: TODO ES HOJAS (verde)
    bool isTrunk = false;
    
    if (TreeType == 0) // Tree (tree/) - Funciona bien
    {
        isTrunk = (uv.y < 0.12) || (localPos.y < -1.0 && distanceFromCenter < 0.2);
    }
    else if (TreeType == 1) // Tree2 (tree2/) - Leaf_Oak
    {
    // Pattern: Trunk is UV middle, wider and taller
    float uvY = input.TextureCoordinate.y;
    bool isMidRangeUV = uvY > 0.2 && uvY < 0.8;
    bool isWiderCenter = distanceFromCenter < 1.0;
    bool isTaller = localPos.y < 1.5;
    isTrunk = isMidRangeUV && isWiderCenter && isTaller;
    }
    else if (TreeType == 2) // Tree3 (tree3/)
    {
        bool isVeryCenter = distanceFromCenter < 0.8;
        bool isVeryLow = localPos.y < -1.0;
        bool isBottomUV = uv.y < 0.20;
        isTrunk = (isVeryCenter && isVeryLow) || isBottomUV;
    }
    
    if (isTrunk)
    {
        // TRUNK: SOLO textura de corteza (sin mezcla con color base)
        float4 barkTexture = tex2D(BarkSampler, uv);
        return barkTexture;
    }
    else
    {
        // LEAVES: SOLO textura de hojas (sin mezcla con color base)
        float4 leavesTexture = tex2D(LeavesSampler, uv);
        return leavesTexture;
    }
}

technique BasicDrawing
{
    pass P0
    {
        VertexShader = compile VS_SHADERMODEL MainVS();
        PixelShader = compile PS_SHADERMODEL MainPS();
    }
}