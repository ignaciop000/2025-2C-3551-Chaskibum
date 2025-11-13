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
    float RotationY : TEXCOORD1;    
    float Scale : TEXCOORD2;
};

struct VSOutput
{
    float4 Position : SV_POSITION;
    float2 TexCoord : TEXCOORD0;
};

VSOutput VSMain(VSInput input)
{
    VSOutput output;

    float cosR = cos(input.RotationY);
    float sinR = sin(input.RotationY);
    
    float3 rotated = float3(
        input.Position.x * cosR - input.Position.z * sinR,
        input.Position.y,
        input.Position.x * sinR + input.Position.z * cosR
    );
    
    rotated *= input.Scale;

    float3 worldPos = rotated + input.InstancePos;

    float4 posWorld = mul(float4(worldPos, 1.0), World);
    float4 posView  = mul(posWorld, View);
    output.Position = mul(posView, Projection);
    
    output.TexCoord = input.TexCoord;
    return output;
}

float4 PSMain(VSOutput input) : SV_TARGET
{
    float4 texColor = tex2D(TextureSampler, input.TexCoord);    
    clip(texColor.a - 0.8);
    return texColor;
}

technique Instancing
{
    pass P0
    {

        VertexShader = compile VS_SHADERMODEL VSMain();
        PixelShader = compile PS_SHADERMODEL PSMain();
    }
}