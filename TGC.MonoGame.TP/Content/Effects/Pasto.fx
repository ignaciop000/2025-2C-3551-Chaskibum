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
    float sway = sin(Time + input.InstancePos.x * 0.01 + input.InstancePos.z * 0.01);
    
    rotated.x += 1 * sway * 0.1 * input.Position.y * input.Position.y ;
    
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