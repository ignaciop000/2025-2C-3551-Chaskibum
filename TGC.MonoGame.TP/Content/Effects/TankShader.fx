#if OPENGL
	#define SV_POSITION POSITION
	#define VS_SHADERMODEL vs_3_0
	#define PS_SHADERMODEL ps_3_0
#else
	#define VS_SHADERMODEL vs_4_0_level_9_1
	#define PS_SHADERMODEL ps_4_0_level_9_1
#endif

float4x4 WorldViewProjection;
float4x4 LightViewProjection;
float4x4 World;
float4x4 InverseTransposeWorld;

float3 lightPosition;

float2 shadowMapSize;
float AmbientLight = 0.3;
float3 eyePosition;

static const float modulatedEpsilon = 0.000000541200182749889791011810302734375;
static const float maxEpsilon = 0.000000523200045689009130001068115234375;

float Ka;
uniform float4 ImpactPoints[10];

// Texturas del T90
texture ModelTexture;
sampler TextureSampler = sampler_state
{
    Texture = <ModelTexture>;
    MinFilter = Linear;
    MagFilter = Linear;
    AddressU = Wrap;
    AddressV = Wrap;
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
    float4 WorldPos : TEXCOORD2;
    float4 LightSpacePosition : TEXCOORD3;
};

struct MenuVSOutput
{
	float4 Position : SV_POSITION;
    float2 TextureCoordinate : TEXCOORD0;
};

void ApplyDeformation(inout float4 worldPosition, float4 impact) 
{
    float3 impactPos = impact.xyz;
    float impactRadius = impact.w;
    float displacementAmount = 10.0f;

    if (impactRadius > 0.0)
    {
        float3 delta = impactPos - worldPosition.xyz;
        float dist = length(delta);

        [branch]
        if (dist > 0.0001 && dist < impactRadius)
        {
            float deformationFactor = (1.0 - (dist / impactRadius));
            float displacement = deformationFactor * impactRadius * 0.5f;
            worldPosition.xyz += normalize(delta) * displacement;
        }
    }
}

VertexShaderOutput MainVS(in VertexShaderInput input)
{
	VertexShaderOutput output = (VertexShaderOutput)0;

    float4 worldPosition = mul(input.Position, World);
    ApplyDeformation(worldPosition, ImpactPoints[0]); 
    ApplyDeformation(worldPosition, ImpactPoints[1]); 
    ApplyDeformation(worldPosition, ImpactPoints[2]); 
    ApplyDeformation(worldPosition, ImpactPoints[3]); 
    ApplyDeformation(worldPosition, ImpactPoints[4]); 
    ApplyDeformation(worldPosition, ImpactPoints[5]); 
    ApplyDeformation(worldPosition, ImpactPoints[6]); 
    ApplyDeformation(worldPosition, ImpactPoints[7]); 
    ApplyDeformation(worldPosition, ImpactPoints[8]); 
    ApplyDeformation(worldPosition, ImpactPoints[9]);

        output.Position = mul(input.Position, WorldViewProjection);
        output.TextureCoordinate = input.TextureCoordinate;
        output.WorldPos = mul(input.Position, World);
        output.LightSpacePosition = mul(output.WorldPos, LightViewProjection);
        output.Normal = mul(float4(input.Normal, 1), InverseTransposeWorld);
    
        return output;
}

float4 MainPS(VertexShaderOutput input) : COLOR
{
    float3 lightDirection = normalize(lightPosition - input.WorldPos.xyz);
    float3 viewDirection = normalize(eyePosition - input.WorldPos.xyz);
    float3 halfVector = normalize(lightDirection + viewDirection);
    float3 normal = normalize(input.Normal.xyz);
    if (length(normal) < 0.001)
    {
        normal = float3(0, 1, 0); // normal hacia arriba
    }
    
    float3 lightSpacePosition = input.LightSpacePosition.xyz / input.LightSpacePosition.w;
    float2 shadowMapTextureCoordinates = 0.5 * lightSpacePosition.xy + float2(0.5, 0.5);
    shadowMapTextureCoordinates.y = 1.0f - shadowMapTextureCoordinates.y;
    
    float inclinationBias = max(modulatedEpsilon * (1.0 - dot(normal, lightDirection)), maxEpsilon);
    	
    float notInShadow = 0.0;
    float2 texelSize = 1.0 / shadowMapSize;
    for (int x = -1; x <= 1; x++)
        for (int y = -1; y <= 1; y++)
        {
            float pcfDepth = tex2D(shadowMapSampler, shadowMapTextureCoordinates + float2(x, y) * texelSize).r + inclinationBias;
            notInShadow += step(lightSpacePosition.z, pcfDepth) / 9.0;
        }
    
     float shadowMapDepth = tex2D(shadowMapSampler, shadowMapTextureCoordinates).r + inclinationBias;
     
    float4 texColor = tex2D(TextureSampler, input.TextureCoordinate);
   
    float3 color = texColor.rgb;
    
    float NdotL = saturate(dot(normal, lightDirection));
    float3 diffuseLight = 0.8 * float3(1,1,1) * NdotL;  
    
    float NdotH = dot(normal, halfVector);
    float3 specularLight = 0.3 * float3(1,1,1) * pow(saturate(NdotH),32);
    
    float4 finalColor = float4(saturate(float3(1,1,1) * Ka + diffuseLight) * color + specularLight, texColor.a);
    
    finalColor.rgb *= 0.5 + 0.5 * notInShadow;
    return finalColor;
}

MenuVSOutput MenuVS(in VertexShaderInput input)
{
	MenuVSOutput output = (MenuVSOutput)0;
	
    output.Position = mul(input.Position, WorldViewProjection);
    output.TextureCoordinate = input.TextureCoordinate;

    return output;
}

float4 MenuPS(MenuVSOutput input) : COLOR
{
    float4 texColor = tex2D(TextureSampler, input.TextureCoordinate);
   
    return float4(texColor.rgb, 1.0);
}

technique BasicDrawing
{
	pass P0
	{
		VertexShader = compile VS_SHADERMODEL MainVS();
		PixelShader = compile PS_SHADERMODEL MainPS();
	}
}

technique MenuDrawing
{
    pass P_0
    {
        VertexShader = compile VS_SHADERMODEL MenuVS();
        PixelShader = compile PS_SHADERMODEL MenuPS();
    }
}
