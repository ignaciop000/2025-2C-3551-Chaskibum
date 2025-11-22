#if OPENGL
#define SV_POSITION POSITION
#define VS_SHADERMODEL vs_3_0
#define PS_SHADERMODEL ps_3_0
#else
#define VS_SHADERMODEL vs_4_0_level_9_1
#define PS_SHADERMODEL ps_4_0_level_9_1
#endif

float4x4 ViewProjection;
float4x4 World;
bool checkInvisible = false;
float Time;
int Sway = 0;


texture ModelTexture;
sampler TextureSampler = sampler_state
{
    Texture = <ModelTexture>;
    MinFilter = Linear;
    MagFilter = Linear;
    AddressU = Wrap;
    AddressV = Wrap;
};

struct DepthPassVertexShaderInput
{
	float4 Position : POSITION0;
	float2 TextureCoordinate : TEXCOORD0;
};

struct DepthPassVertexShaderInputPasto
{
	float4 Position : POSITION0;
	float2 TextureCoordinate : TEXCOORD0;
	float3 InstancePos : POSITION1;   
    float Scale : TEXCOORD1;
    float4 RotationRow0 : TEXCOORD2;
    float4 RotationRow1 : TEXCOORD3;
    float4 RotationRow2 : TEXCOORD4;
    float4 RotationRow3 : TEXCOORD5;
};

struct DepthPassVertexShaderOutput
{
	float4 Position : SV_POSITION;
	float2 TextureCoordinate : TEXCOORD0;
	float4 ScreenSpacePosition : TEXCOORD1;
};

DepthPassVertexShaderOutput DepthVS(in DepthPassVertexShaderInput input)
{
	DepthPassVertexShaderOutput output;
	
	float swayOffset = 0; 
    float4 worldPosition = mul(input.Position, World);
   if(Sway)
    {
        
        swayOffset = sin(Time + worldPosition.x * .5 + worldPosition.z * .5 + worldPosition.y * 2) * (((1-input.TextureCoordinate.y) + input.TextureCoordinate.x)/2);
        worldPosition.x += swayOffset * 10;
        
    }
    output.Position = mul(worldPosition, ViewProjection);
	output.ScreenSpacePosition = mul(input.Position, mul(World, ViewProjection));
	output.TextureCoordinate = input.TextureCoordinate;
	
	return output;
}

DepthPassVertexShaderOutput DepthVSPasto(in DepthPassVertexShaderInputPasto input)
{
	DepthPassVertexShaderOutput output;
	
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
    
    float3 posWorld = rotated + input.InstancePos;

    output.Position = mul(float4(posWorld, 1.0), ViewProjection);
	output.ScreenSpacePosition = mul(float4(posWorld, 1.0), ViewProjection);
	output.TextureCoordinate = input.TextureCoordinate;
	
	return output;
}

float4 DepthPS(in DepthPassVertexShaderOutput input) : COLOR
{
    float alpha = tex2D(TextureSampler, input.TextureCoordinate).a;
    if(checkInvisible)
        clip(alpha - 0.3);
    
    float depth = input.ScreenSpacePosition.z / input.ScreenSpacePosition.w;
    
    return float4(depth, depth, depth, 1.0);
}

technique DepthPass
{
	pass Pass0
	{
		VertexShader = compile VS_SHADERMODEL DepthVS();
		PixelShader = compile PS_SHADERMODEL DepthPS();
	}
};

technique DepthPassPasto
{
	pass P0
	{
		VertexShader = compile VS_SHADERMODEL DepthVSPasto();
		PixelShader = compile PS_SHADERMODEL DepthPS();
	}
};
