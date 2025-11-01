#if OPENGL
	#define SV_POSITION POSITION
	#define VS_SHADERMODEL vs_3_0
	#define PS_SHADERMODEL ps_3_0
#else
    #define VS_SHADERMODEL vs_4_0_level_9_1
    #define PS_SHADERMODEL ps_4_0_level_9_1
#endif

float4x4 WorldViewProjection;
float4x4 InverseTransposeWorld;
float4x4 World;
float4x4 LightViewProjection;

float3 lightPosition;

float2 shadowMapSize;

static const float modulatedEpsilon = 0.000041200182749889791011810302734375;
static const float maxEpsilon = 0.000023200045689009130001068115234375;

float4x4 View;
float4x4 Projection;

float alphaValue = 1;
float time = 0;
float3 eyePosition;

//---------------TEXTURAS---------------
texture texDiffuseMap;
sampler2D diffuseMap = sampler_state
{
    Texture = (texDiffuseMap);
    ADDRESSU = MIRROR;
    ADDRESSV = MIRROR;
    MINFILTER = LINEAR;
    MAGFILTER = LINEAR;
    MIPFILTER = LINEAR;
};

texture texDiffuseMap2;
sampler2D diffuseMap2 = sampler_state
{
    Texture = (texDiffuseMap2);
    ADDRESSU = MIRROR;
    ADDRESSV = MIRROR;
    MINFILTER = LINEAR;
    MAGFILTER = LINEAR;
    MIPFILTER = LINEAR;
};

texture texColorMap;
sampler2D colorMap = sampler_state
{
    Texture = (texColorMap);
    ADDRESSU = WRAP;
    ADDRESSV = WRAP;
    MINFILTER = LINEAR;
    MAGFILTER = LINEAR;
    MIPFILTER = LINEAR;
};

texture shadowMap;
sampler2D shadowMapSampler =
sampler_state
{
	Texture = <shadowMap>;
	MinFilter = Point;
	MagFilter = Point;
	MipFilter = Point;
	AddressU = Clamp;
	AddressV = Clamp;
};


//---------------STRUCTS---------------

//input VS normal
struct VS_INPUT
{
    float4 Position : POSITION0;
    float2 Texcoord : TEXCOORD0;
    float3 Normal : NORMAL0;
};

//output VS normal
struct VS_OUTPUT
{
    float4 Position : POSITION0;
    float2 Texcoord : TEXCOORD0;
    float3 WorldPos : TEXCOORD1;
    float3 WorldNormal : TEXCOORD2;
};

//input VS sombra
struct ShadowedVertexShaderInput
{
	float4 Position : POSITION0;
	float3 Normal : NORMAL;
	float2 TextureCoordinates : TEXCOORD0;
};

//output VS sombra
struct ShadowedVertexShaderOutput
{
	float4 Position : SV_POSITION;
	float2 TextureCoordinates : TEXCOORD0;
	float4 WorldSpacePosition : TEXCOORD1;
	float4 LightSpacePosition : TEXCOORD2;
    float4 Normal : TEXCOORD3;
};

//---------------SHADERS---------------
//vertex shader normal
VS_OUTPUT vs_RenderTerrain(VS_INPUT input)
{
    VS_OUTPUT output;

    //Proyectar posicion
    float4 worldPosition = mul(input.Position, World);
    float4 viewPosition = mul(worldPosition, View);
    output.Position = mul(viewPosition, Projection);

    //Enviar Texcoord directamente
    output.Texcoord = input.Texcoord;

    //todo: que le pase el inv trasp. word
    float4x4 matInverseTransposeWorld = World;
    output.WorldPos = worldPosition.xyz;
    output.WorldNormal = mul(input.Normal, matInverseTransposeWorld).xyz;

    return output;
}

//Pixel Shader normal
float4 ps_RenderTerrain(VS_OUTPUT input) : COLOR0
{
    float4 grassTex = tex2D(diffuseMap, input.Texcoord * 50);
    float4 dirtTex = tex2D(diffuseMap2, input.Texcoord * 50);
    float3 mapColor = tex2D(colorMap, input.Texcoord).rgb;
    float4 color = lerp(grassTex, dirtTex, mapColor.r);
    
    float3 lightDirection = normalize(lightPosition - input.WorldPos);
    float3 viewDirection = normalize(eyePosition - input.WorldPos);
    float3 halfVector = normalize(lightDirection + viewDirection);
    float3 normal = normalize(input.WorldNormal.xyz);
    
    float NdotL = saturate(dot(normal, lightDirection));
    float3 diffuseLight = 0.8 * color.rgb * NdotL;  
    
    float NdotH = dot(normal, halfVector);
    float3 specularLight = 0.05 * float3(1,1,1) * pow(saturate(NdotH),1.1);

    float4 finalColor = float4(saturate(float3(1,1,1) * 0.2 + diffuseLight) * color.rgb + specularLight, color.a);
       
   	return finalColor;
}

//vertex shader sombras
ShadowedVertexShaderOutput MainVS(in ShadowedVertexShaderInput input)
{
	ShadowedVertexShaderOutput output;
	output.Position = mul(input.Position, WorldViewProjection);
	output.TextureCoordinates = input.TextureCoordinates;
	output.WorldSpacePosition = mul(input.Position, World);
	output.LightSpacePosition = mul(output.WorldSpacePosition, LightViewProjection);
    output.Normal = mul(float4(input.Normal, 1), InverseTransposeWorld);
	return output;
}

//pixel shader sombras
float4 ShadowedPCFPS(in ShadowedVertexShaderOutput input) : COLOR
{
    float3 lightSpacePosition = input.LightSpacePosition.xyz / input.LightSpacePosition.w;
    float2 shadowMapTextureCoordinates = 0.5 * lightSpacePosition.xy + float2(0.5, 0.5);
    shadowMapTextureCoordinates.y = 1.0f - shadowMapTextureCoordinates.y;
	
    float3 normal = normalize(input.Normal.rgb);
    float3 lightDirection = normalize(lightPosition - input.WorldSpacePosition.xyz);
    float inclinationBias = max(modulatedEpsilon * (1.0 - dot(normal, lightDirection)), maxEpsilon);
	
    float notInShadow = 0.0;
    float2 texelSize = 1.0 / shadowMapSize;
    for (int x = -1; x <= 1; x++)
        for (int y = -1; y <= 1; y++)
        {
            float pcfDepth = tex2D(shadowMapSampler, shadowMapTextureCoordinates + float2(x, y) * texelSize).r + inclinationBias;
            notInShadow += step(lightSpacePosition.z, pcfDepth) / 9.0;
        }
	
	float4 grassTex = tex2D(diffuseMap, input.TextureCoordinates * 100);
    float4 dirtTex = tex2D(diffuseMap2, input.TextureCoordinates * 100);
    
    
    float3 mapColor = tex2D(colorMap, input.TextureCoordinates).rgb;
	float4 color =  lerp(grassTex, dirtTex, mapColor.r);
	
    float3 viewDirection = normalize(eyePosition - input.WorldSpacePosition);
    float3 halfVector = normalize(lightDirection + viewDirection);
    
    float NdotL = saturate(dot(normal, lightDirection));
    float3 diffuseLight = 0.8 * color.rgb * NdotL;  
    
    float NdotH = dot(normal, halfVector);
    float3 specularLight = 0.05 * float3(1,1,1) * pow(saturate(NdotH),1.1);

    float4 baseColor = float4(saturate(float3(1,1,1) * 0.2 + diffuseLight) * color.rgb + specularLight, color.a);
	
    baseColor.rgb *= 0.5 + 0.5 * (1.0 - notInShadow);
	return baseColor;
}

technique RenderTerrain
{
    pass Pass_0
    {
        VertexShader = compile VS_SHADERMODEL vs_RenderTerrain();
        PixelShader = compile PS_SHADERMODEL ps_RenderTerrain();
    }
}

technique DrawShadowedPCF
{
    pass Pass0
    {
        VertexShader = compile VS_SHADERMODEL MainVS();
        PixelShader = compile PS_SHADERMODEL ShadowedPCFPS();
    }
};
