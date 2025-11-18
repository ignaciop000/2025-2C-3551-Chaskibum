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
float AmbientLight = 0.3;
float3 lightPosition;
float3 eyePosition;

float3 ambientColor;
float Ka;

float3 diffuseColor;
float Kd;

float3 specularColor;
float Ks;

float shininess;

// Parámetros de niebla volumétrica 3D
float3 FogColor = float3(0.5, 0.6, 0.7); // Color azulado por defecto
float FogStart = 700.0; // Distancia donde empieza la niebla
float FogEnd = 2200.0;   // Distancia donde la niebla es completa

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

//Textura para Normals
texture NormalTexture;
sampler2D normalSampler = sampler_state
{
    Texture = (NormalTexture);
    ADDRESSU = WRAP;
    ADDRESSV = WRAP;
    MINFILTER = LINEAR;
    MAGFILTER = LINEAR;
    MIPFILTER = LINEAR;
};

float3 getNormalFromMap(float2 textureCoordinates, float3 worldPosition, float3 worldNormal)
{
    float3 tangentNormal = tex2D(normalSampler, textureCoordinates).xyz * 2.0 - 1.0;

    float3 Q1 = ddx(worldPosition);
    float3 Q2 = ddy(worldPosition);
    float2 st1 = ddx(textureCoordinates);
    float2 st2 = ddy(textureCoordinates);

    worldNormal = normalize(worldNormal.xyz);
    float3 T = normalize(Q1 * st2.y - Q2 * st1.y);
    float3 B = -normalize(cross(worldNormal, T));
    float3x3 TBN = float3x3(T, B, worldNormal);

    return normalize(mul(tangentNormal, TBN));
}

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

    output.WorldPos = worldPosition.xyz;
    output.Normal = mul(input.Normal, (float3x3)World);
    output.TextureCoordinate = input.TextureCoordinate;

    return output;
}

float4 MainPS(VertexShaderOutput input) : COLOR
{

    float3 lightDirection = normalize(lightPosition - input.WorldPos);
    float3 viewDirection = normalize(eyePosition - input.WorldPos);
    float3 halfVector = normalize(lightDirection + viewDirection);
    float3 normal = normalize(input.Normal.xyz);
    
    float4 texColor = tex2D(TextureSampler, input.TextureCoordinate);
    clip(texColor.a - 0.2);
    float useTexAmount = UseTexture;
    float3 color = lerp(TintColor.rgb, texColor.rgb, useTexAmount);
    
    float NdotL = saturate(dot(normal, lightDirection));
    float3 diffuseLight = Kd * diffuseColor * NdotL;  
        
    float NdotH = dot(normal, halfVector);
    float3 specularLight = Ks * specularColor * pow(saturate(NdotH), shininess);
    
    float3 ambientLight = ambientColor * Ka;
    
    float4 finalColor = float4(saturate(ambientLight + diffuseLight) * color + specularLight, texColor.a);
    
    // Aplicar niebla volumétrica 3D
    float distanceToCamera = distance(input.WorldPos, eyePosition);
    float fogFactor = saturate((distanceToCamera - FogStart) / (FogEnd - FogStart));
    finalColor.rgb = lerp(finalColor.rgb, FogColor, fogFactor);
       
	return finalColor;
}

VertexShaderOutput NormalMapVS(in VertexShaderInput input)
{
    VertexShaderOutput output = (VertexShaderOutput) 0;

    float4 worldPosition = mul(input.Position, World);
    float4 viewPosition = mul(worldPosition, View);	
    output.Position = mul(viewPosition, Projection);
    
    output.WorldPos = worldPosition;
    output.Normal = mul(input.Normal, (float3x3)World);
    output.TextureCoordinate = input.TextureCoordinate;
	
    return output;
}

float4 NormalMapPS(VertexShaderOutput input) : COLOR
{
    // Base vectors
    float3 lightDirection = normalize(lightPosition - input.WorldPos.xyz);
    float3 viewDirection = normalize(eyePosition - input.WorldPos.xyz);
    float3 halfVector = normalize(lightDirection + viewDirection);
    float3 normal =  getNormalFromMap(input.TextureCoordinate, input.WorldPos.xyz, normalize(input.Normal.xyz));

	// Get the texture texel
    float4 texelColor = tex2D(TextureSampler, input.TextureCoordinate);
    clip(texelColor.a - 0.3);
	// Calculate the diffuse light
    float NdotL = saturate(dot(normal, lightDirection));
    float3 diffuseLight = Kd * diffuseColor * NdotL;

    // Calculate the specular light
    float NdotH = dot(normal, halfVector);
    float3 specularLight = Ks * specularColor * pow(NdotH, shininess);
    
    // Final calculation
    float4 finalColor = float4(saturate(ambientColor * Ka + diffuseLight) * texelColor.rgb + specularLight, texelColor.a);
    
    // Aplicar niebla volumétrica 3D
    float distanceToCamera = distance(input.WorldPos.xyz, eyePosition);
    float fogFactor = saturate((distanceToCamera - FogStart) / (FogEnd - FogStart));
    finalColor.rgb = lerp(finalColor.rgb, FogColor, fogFactor);
    
    return finalColor;

}

technique BasicColorDrawing
{
	pass P0
	{
		VertexShader = compile VS_SHADERMODEL MainVS();
		PixelShader = compile PS_SHADERMODEL MainPS();
	}
}

technique NormalMapping
{
    pass Pass0
    {
        VertexShader = compile VS_SHADERMODEL NormalMapVS();
        PixelShader = compile PS_SHADERMODEL NormalMapPS();
    }
};