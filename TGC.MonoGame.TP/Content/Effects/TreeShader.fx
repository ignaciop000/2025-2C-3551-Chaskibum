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

// AÑADIDO: El parámetro que espera ModelInstances.cs
float3 DiffuseColor;

// NUEVO: Parámetro para identificar el tipo de árbol (0=tree, 1=tree2, 2=tree3)
int TreeType;

// Dos texturas: una para la corteza y otra para las hojas
texture BarkTexture;
texture LeavesTexture;

sampler BarkSampler = sampler_state
{
    Texture = <BarkTexture>;
    MinFilter = Linear;
    MagFilter = Linear;
    AddressU = Wrap;
    AddressV = Wrap;
};

sampler LeavesSampler = sampler_state
{
    Texture = <LeavesTexture>;
    MinFilter = Linear;
    MagFilter = Linear;
    AddressU = Wrap;
    AddressV = Wrap;
};

struct VertexShaderInput
{
    float4 Position : POSITION0;
    float2 TextureCoordinate : TEXCOORD0;
    // Usaremos el color del vértice para decidir qué textura usar.
    // Asumimos que el modelador 3D pintó los vértices del tronco de un color y los de las hojas de otro.
    float4 Color : COLOR0; 
};

struct VertexShaderOutput
{
    float4 Position : SV_POSITION;
    float2 TextureCoordinate : TEXCOORD0;
    float4 Color : COLOR0;
    float3 LocalPosition : TEXCOORD1; // Posición local del objeto (antes de transformaciones)
};

VertexShaderOutput MainVS(in VertexShaderInput input)
{
    VertexShaderOutput output = (VertexShaderOutput)0;
    
    float4 worldPosition = mul(input.Position, World);
    float4 viewPosition = mul(worldPosition, View);	
    output.Position = mul(viewPosition, Projection);

    output.TextureCoordinate = input.TextureCoordinate;
    output.Color = input.Color; // Pasamos el color del vértice al Pixel Shader
    output.LocalPosition = input.Position.xyz; // CLAVE: posición local del objeto

    return output;
}

float4 MainPS(VertexShaderOutput input) : COLOR
{
    // Apply different logic based on tree type
    float3 localPos = input.LocalPosition;
    float2 uv = input.TextureCoordinate;
    bool isTrunk = false;
    
    // Check vertex color availability
    bool hasValidColor = (input.Color.r > 0.1 || input.Color.g > 0.1 || input.Color.b > 0.1);
    
    // Apply tree-specific detection logic
    if (TreeType == 0) // Tree (tree/)
    {
        // Tree works well with current logic, just fine-tune
        if (hasValidColor)
        {
            float colorSum = input.Color.r + input.Color.g + input.Color.b;
            bool isDarkish = colorSum < 1.8;
            bool isBrownish = input.Color.r > input.Color.g * 1.1;
            isTrunk = isDarkish && isBrownish;
        }
        else
        {
            float distanceFromCenter = length(float2(localPos.x, localPos.z));
            bool isNearCenter = distanceFromCenter < 0.4; // Tighter center for tree
            bool isLowerPart = localPos.y < 0.0;
            isTrunk = isNearCenter || isLowerPart;
        }
        // Force some trunk visibility
        if (uv.y < 0.25) isTrunk = true; // Only bottom 25% forced
    }
    else if (TreeType == 1) // Tree2 (tree2/)
    {
        // Tree2 needs less aggressive parameters
        if (hasValidColor)
        {
            // Less aggressive color detection for tree2
            float brightness = (input.Color.r + input.Color.g + input.Color.b) / 3.0;
            bool isDark = brightness < 0.2; // Much more restrictive
            bool isWarm = input.Color.r > input.Color.g * 1.5; // More restrictive
            isTrunk = isDark && isWarm; // Both conditions required
        }
        else
        {
            // Much more restrictive geometry detection for tree2
            float distanceFromCenter = length(float2(localPos.x, localPos.z));
            bool isNearCenter = distanceFromCenter < 0.2; // Much tighter center
            bool isLowerPart = localPos.y < -0.5; // Much lower threshold
            isTrunk = isNearCenter && isLowerPart; // Both conditions required
        }
        // Less forcing for tree2
        if (uv.y < 0.15) isTrunk = true; // Only bottom 15% forced
    }
    else if (TreeType == 2) // Tree3 (tree3/)
    {
        // Tree3 needs very conservative parameters
        if (hasValidColor)
        {
            // Very conservative color detection for tree3
            float colorSum = input.Color.r + input.Color.g + input.Color.b;
            bool isDarkish = colorSum < 0.8; // Much more restrictive
            bool isRedish = input.Color.r > (input.Color.g + input.Color.b) * 1.5; // More restrictive
            isTrunk = isDarkish && isRedish; // Both conditions required
        }
        else
        {
            // Very conservative geometry detection for tree3
            float distanceFromCenter = length(float2(localPos.x, localPos.z));
            bool isNearCenter = distanceFromCenter < 0.15; // Very tight center
            bool isLowerPart = localPos.y < -0.8; // Very low threshold
            isTrunk = isNearCenter && isLowerPart; // Both conditions required
        }
        // Minimal forcing for tree3
        if (uv.y < 0.1) isTrunk = true; // Only bottom 10% forced
    }
    
    if (isTrunk)
    {
        // TRUNK: brown base + bark texture
        float4 barkTexture = tex2D(BarkSampler, uv);
        float4 brownBase = float4(0.55, 0.35, 0.15, 1.0);
        return lerp(brownBase, barkTexture, 0.7);
    }
    else
    {
        // LEAVES: green base + leaves texture  
        float4 leavesTexture = tex2D(LeavesSampler, uv);
        float4 greenBase = float4(0.25, 0.7, 0.25, 1.0);
        return lerp(greenBase, leavesTexture, 0.7);
    }
}

technique BasicDrawing
{
    pass P0
    {
        VertexShader = compile VS_SHADERMODEL MainVS();
        PixelShader = compile PS_SHADERMODEL MainPS();
    }
}