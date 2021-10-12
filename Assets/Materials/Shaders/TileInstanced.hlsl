#ifndef TILE_INSTANCED_INCLUDED
#define TILE_INSTANCED_INCLUDED

struct InstanceData {
	float4x4 m;
	float lifetime;
};

StructuredBuffer<InstanceData> _PerInstanceData;
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

void ColorLifetime_float(float4 ColorA, float4 ColorB, out float4 Out, out float alpha) {
	Out = ColorA;
	alpha = 1.0f;
	#if UNITY_ANY_INSTANCING_ENABLED
	float t = _PerInstanceData[unity_InstanceID].lifetime / _MaxLifetime;
	Out = lerp(ColorA, ColorB, 1.0f - t);
	alpha = smoothstep(0.0f, 1.0f, t);
	#endif
}

#endif