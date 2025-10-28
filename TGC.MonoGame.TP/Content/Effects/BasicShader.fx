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

float4 TintColor;
float Time = 0;

// Textura principal del modelo
texture ModelTexture;
sampler TextureSampler = sampler_state
{
    Texture = <ModelTexture>;
    MinFilter = Linear;
    MagFilter = Linear;
    AddressU = Wrap;
    AddressV = Wrap;
};

// Flag para indicar si usamos textura o color sólido
bool UseTexture = false;

struct VertexShaderInput
{
	float4 Position : POSITION0;
    float2 TextureCoordinate : TEXCOORD0;
    float3 Normal : NORMAL0;
};

struct VertexShaderOutput
{
	float4 Position : SV_POSITION;
    float2 TextureCoordinate : TEXCOORD0;
    float3 Normal : TEXCOORD1;
    float3 WorldPos : TEXCOORD2;
};

VertexShaderOutput MainVS(in VertexShaderInput input)
{
	VertexShaderOutput output = (VertexShaderOutput)0;
    
    float4 worldPosition = mul(input.Position, World);
    float4 viewPosition = mul(worldPosition, View);	
    output.Position = mul(viewPosition, Projection);

    output.TextureCoordinate = input.TextureCoordinate;
    output.Normal = mul(input.Normal, (float3x3)World);
    output.WorldPos = worldPosition.xyz;

    return output;
}

float4 MainPS(VertexShaderOutput input) : COLOR
{
    // Iluminación básica
    float3 lightDir = normalize(float3(1, 1, 1));
    float3 normal = normalize(input.Normal);
    float diffuse = saturate(dot(normal, lightDir));
    float ambient = 0.3;
    float lighting = ambient + diffuse * 0.7;
    
    // Usar textura o color sólido basado en el flag
    float useTexAmount = UseTexture;
    float4 texColor = tex2D(TextureSampler, input.TextureCoordinate);
    float3 baseColor = lerp(TintColor.rgb, texColor.rgb, useTexAmount);
    
	float3 rgb = baseColor * lighting;
	return float4(rgb * TintColor.a, TintColor.a);
}

technique BasicColorDrawing
{
	pass P0
	{
		VertexShader = compile VS_SHADERMODEL MainVS();
		PixelShader = compile PS_SHADERMODEL MainPS();
	}
}