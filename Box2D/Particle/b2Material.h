

#ifndef B2_PARTICLE_MATERIAL
#define B2_PARTICLE_MATERIAL

#include <BOX2d/Particle/b2Particle.h>
#include <Box2D/Particle/b2ParticleSystem.h>
#include <Box2D/Common/b2GrowableBuffer.h>

class b2World;
class b2ParticleSystem;
class b2ParticleGroup;

#pragma once


enum b2MaterialFlag
{
	b2_extinguishingMaterial = 1 << 0,
	b2_meltingMaterial = 1 << 1,
	b2_boilingMaterial = 1 << 2,
	b2_flammableMaterial = 1 << 3,
	b2_heatConductingMaterial = 1 << 4,
	b2_electricityConductingMaterial = 1 << 5,
};


struct b2ParticleMaterialDef 
{
	b2ParticleMaterialDef()
	{
		partSystem = NULL;
		matFlags = 0;
		mass = 1.0f;
		stability = 1.0f;
		extinguishingPoint = -273.15f;
		meltingPoint = 0.0f;
		boilingPoint = 100.0f;
		ignitionPoint = 300.0f;
		heatConductivity = 1.0f;
	}

	b2ParticleSystem* partSystem;
	uint32 matFlags;
	float32 mass;
	float32 stability;
	float32 extinguishingPoint;
	float32 meltingPoint;
	float32 boilingPoint;
	float32 ignitionPoint;
	float32 heatConductivity;
};


class b2ParticleMaterial
{
public:
	b2ParticleMaterial(b2ParticleMaterialDef def);
	~b2ParticleMaterial();

	const b2ParticleSystem* partSys;

	const uint32 m_matFlags;
	const float32 m_mass;
	const float32 m_invMass;
	const float32 m_stability;
	const float32 m_invStability;
	const float32 m_extinguishingPoint;
	const float32 m_meltingPoint;
	const float32 m_boilingPoint;
	const float32 m_ignitionPoint;
	const float32 m_heatConductivity;

	b2ParticleMaterial* m_prev;
	b2ParticleMaterial* m_next;
};

inline b2ParticleMaterial::b2ParticleMaterial(b2ParticleMaterialDef def)
  : m_matFlags(def.matFlags),
	m_mass(def.mass), m_invMass(1 / def.mass),
	m_stability(def.stability), m_invStability(1 / def.stability),
	m_extinguishingPoint(def.extinguishingPoint),
	m_meltingPoint(def.meltingPoint),
	m_boilingPoint(def.boilingPoint),
	m_ignitionPoint(def.ignitionPoint),
	m_heatConductivity(def.heatConductivity)
{
}

inline b2ParticleMaterial::~b2ParticleMaterial()
{
}



struct b2BodyMaterialDef
{
	b2BodyMaterialDef()
	{
		matFlags = 0;
		density = 1.0f;
		friction = 0.1f;
		bounciness = 0.2f;
		stability = 1.0f;
		extinguishingPoint = 0.0f;
		meltingPoint = 1000.0f;
		ignitionPoint = 300.0f;
		heatConductivity = 0.0f;
	}

	uint32 matFlags;
	float32 density;
	float32 friction;
	float32 bounciness;
	float32 stability;
	float32 extinguishingPoint;
	float32 meltingPoint;
	float32 ignitionPoint;
	float32 heatConductivity;
};

class b2BodyMaterial
{
public:
	const uint32 m_matFlags;
	const float32 m_density;
	const float32 m_friction;
	const float32 m_bounciness;
	const float32 m_stability;
	const float32 m_invStability;
	const float32 m_extinguishingPoint;
	const float32 m_meltingPoint;
	const float32 m_ignitionPoint;
	const float32 m_heatConductivity;

	b2BodyMaterial* m_prev;
	b2BodyMaterial* m_next;

	b2BodyMaterial(b2BodyMaterialDef def);
	~b2BodyMaterial();
};

inline b2BodyMaterial::b2BodyMaterial(b2BodyMaterialDef def)
  : m_matFlags(def.matFlags),
	m_density(def.density),
	m_friction(def.friction),
	m_bounciness(def.bounciness),
	m_stability(def.stability), m_invStability(1 / def.stability),
	m_extinguishingPoint(def.extinguishingPoint),
	m_meltingPoint(def.meltingPoint),
	m_ignitionPoint(def.ignitionPoint),
	m_heatConductivity(def.heatConductivity)
{}

inline b2BodyMaterial::~b2BodyMaterial()
{}



#endif

