#ifndef TILE_INSTANCED_INCLUDED
#define TILE_INSTANCED_INCLUDED

struct InstanceData {
	float4x4 m;
	float lifetime;
};

struct InstanceHairData
{
	float4x4 m;
	float3x3 normalMatrix;
	float lifetime;
};

StructuredBuffer<InstanceData> _PerInstanceData;
StructuredBuffer<InstanceHairData> _PerInstanceHairData;
float _MaxLifetime;

#if UNITY_ANY_INSTANCING_ENABLED

	void vertInstancingSetup() {
		
	}

#endif

// Shader Graph Functions
void GetInstanceID_float(out float Out) {
	Out = 0;
	#ifndef SHADERGRAPH_PREVIEW
	#if UNITY_ANY_INSTANCING_ENABLED
	Out = unity_InstanceID;
	#endif
	#endif
}

void Instancing_float(float3 Position, out float3 Out) {
    Out = Position;
    #if UNITY_ANY_INSTANCING_ENABLED
    InstanceData data = _PerInstanceData[unity_InstanceID];
    Out = mul(data.m, float4(Position.x, Position.y, Position.z, 1.0f)).xyz;
    #endif
}

void InstancingHair_float(float3 Position, float3 Normal, out float3 OutPos, out float3 OutNormal) {
    OutPos = Position;
	OutNormal = Normal;
    #if UNITY_ANY_INSTANCING_ENABLED
    InstanceHairData data = _PerInstanceHairData[unity_InstanceID];
    OutPos = mul(data.m, float4(Position.x, Position.y, Position.z, 1.0f)).xyz;
	OutNormal = mul(data.normalMatrix, Normal);
    #endif
}

void ColorLifetime_float(float4 ColorA, float4 ColorB, out float4 Out, out float alpha) {
	Out = ColorA;
	alpha = 1.0f;
	#if UNITY_ANY_INSTANCING_ENABLED
	float t = _PerInstanceData[unity_InstanceID].lifetime / _MaxLifetime;
	Out = lerp(ColorA, ColorB, 1.0f - t);
	alpha = smoothstep(0.0f, 1.0f, t);
	#endif
}

void ColorFixed_float(float4 ColorA, float4 ColorB, out float4 Out)
{
	Out = ColorA;
	#if UNITY_ANY_INSTANCING_ENABLED
	float t = _PerInstanceData[unity_InstanceID].lifetime;
	Out = lerp(ColorA, ColorB, step(1.0f, t));
	#endif
}

void ColorFixedHair_float(float4 ColorA, float4 ColorB, out float4 Out)
{
	Out = ColorA;
	#if UNITY_ANY_INSTANCING_ENABLED
	float t = _PerInstanceHairData[unity_InstanceID].lifetime;
	Out = lerp(ColorA, ColorB, step(1.0f, t));
	#endif
}

#endif