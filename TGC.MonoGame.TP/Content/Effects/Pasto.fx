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
float Time; 

float3 lightPosition;
float3 eyePosition;

texture ModelTexture;
sampler TextureSampler = sampler_state
{
    Texture = <ModelTexture>;
    MinFilter = Linear;
    MagFilter = Linear;
    AddressU = Wrap;
    AddressV = Wrap;
};

struct VSInput
{
    float3 Position : POSITION0;   
    float2 TexCoord : TEXCOORD0;   
    float3 InstancePos : POSITION1;   
    float Scale : TEXCOORD1;
    float4 RotationRow0 : TEXCOORD2;
    float4 RotationRow1 : TEXCOORD3;
    float4 RotationRow2 : TEXCOORD4;
    float4 RotationRow3 : TEXCOORD5;
};

struct VSOutput
{
    float4 Position : SV_POSITION;
    float2 TexCoord : TEXCOORD0;
    float Scale: TEXCOORD1;
    float4 WorldPos: TEXCOORD2;
};

VSOutput VSMain(VSInput input)
{
    VSOutput output;
    
    float4x4 rotationMatrix = float4x4(
            input.RotationRow0,
            input.RotationRow1,
            input.RotationRow2,
            input.RotationRow3
    );
        
    float3 rotated = mul(input.Position, (float3x3)rotationMatrix);
    float sway = sin(Time * (2 / input.Scale) + input.InstancePos.x * 0.1 + input.InstancePos.z * 0.1) + 1;
    
    rotated.x += 1 * sway * 0.25 * input.Position.y ;
    
    rotated.y *= input.Scale;
    output.Scale = input.Scale;
    float3 posWorld = rotated + input.InstancePos;

    output.WorldPos = mul(float4(posWorld, 1.0), World);
    float4 posView  = mul(output.WorldPos, View);
    output.Position = mul(posView, Projection);
    output.TexCoord = input.TexCoord;
    return output;
}

float4 PSMain(VSOutput input) : SV_TARGET
{
    float4 texColor = tex2D(TextureSampler, input.TexCoord);    
    clip(texColor.a - 0.8);
    float gradient = (1 - input.TexCoord.y) * 1.8;
    texColor.r += (input.Scale - 1.3)  * 0.05;
    texColor.rgb = lerp(texColor.rgb * -0.3, texColor.rgb, gradient);
    
    float3 lightDirection = normalize(lightPosition - input.WorldPos.xyz);
    float3 viewDirection = normalize(eyePosition - input.WorldPos.xyz);
    float3 halfVector = normalize(lightDirection + viewDirection);
    float3 normal = normalize(lerp( float3(0,1,0), float3(0,0,1), pow(input.TexCoord.y,4)));
    
    float NdotL = dot(normal, lightDirection);
    float3 diffuseLight = 0.1 * float3(1,1,1) * saturate(NdotL);  
        
    float NdotH = dot(normal, halfVector);
    float3 specularLight = 0.15 * float3(1,1,1) * pow(saturate(NdotH), 6);
    specularLight *= 1-input.TexCoord.y;
   
    float3 ambientLight = float3(1,1,1) * 0.8;
    
    float4 finalColor = float4(saturate(ambientLight + diffuseLight) * texColor.rgb + specularLight, texColor.a);
    
    return finalColor;
}

technique Instancing
{
    pass P0
    {
        VertexShader = compile VS_SHADERMODEL VSMain();
        PixelShader = compile PS_SHADERMODEL PSMain();
    }
}