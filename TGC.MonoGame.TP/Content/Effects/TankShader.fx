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
    
    // Textura
    float4 texColor = tex2D(TextureSampler, input.TextureCoordinate);
    
    return float4(texColor.rgb * lighting, texColor.a);
}

technique BasicDrawing
{
	pass P0
	{
		VertexShader = compile VS_SHADERMODEL MainVS();
		PixelShader = compile PS_SHADERMODEL MainPS();
	}
}
