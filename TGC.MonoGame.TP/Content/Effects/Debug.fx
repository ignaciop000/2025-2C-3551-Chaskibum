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
float4 DebugColor;

struct VSInput { float4 Position : POSITION; float4 Color : COLOR0; };
struct VSOutput { float4 Position : SV_POSITION; float4 Color : COLOR0; };

VSOutput VSMain(VSInput input) {
    VSOutput o;
    o.Position = mul(mul(mul(input.Position, World), View), Projection);
    o.Color = DebugColor;
    return o;
}

float4 PSMain(VSOutput input) : SV_TARGET { return input.Color; }

technique DebugTech
{
    pass Pass_0
    {
        VertexShader = compile VS_SHADERMODEL VSMain();
        PixelShader = compile PS_SHADERMODEL PSMain();
    }
}

