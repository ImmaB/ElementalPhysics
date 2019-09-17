/*
* Copyright (c) 2013 Google, Inc.
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/
#ifndef B2_PARTICLE
#define B2_PARTICLE

#include <Box2D/Common/b2Math.h>
#include <Box2D/Common/b2Settings.h>
#include <Box2D/Common/b2IntrusiveList.h>

struct ParticleGroup;

struct Particle
{
	struct Def
	{
		Def()
		{
			flags = 0;
			position = Vec3_zero;
			velocity = Vec3_zero;
			color = 0;
			groupIdx = INVALID_IDX;
			matIdx = INVALID_IDX;
			heat = 0.0f;
			health = 1.0f;
		}
		uint32 flags;
		Vec3 position;
		Vec3 velocity;
		int32 color;

		float32 heat;
		float32 health;

		int32 groupIdx;
		int32 matIdx;
	};

	enum Flag
	{
		Zombie = 1u << 0,			/// Removed after next simulation step.
		Reactive = 1u << 1,			/// Makes pairs or triads with other particles.
		Controlled = 1u << 2,		/// markes Particles that are currently controlled
		Burning = 1u << 3,			/// Burning down to other mat
	};

	struct Mat
	{
		struct Def
		{
			uint32  flags;
			float32 density;
			float32 mass;
			float32 stability;
			float32 heatConductivity;
			float32 strength;
		};
		struct ChangeDef
		{
			float32 coldThreshold;
			int32 coldMatIdx;
			float32 hotThreshold;
			int32 hotMatIdx;
			float32 ignitionThreshold;
			int32 burnedMatIdx;
			int32 fireMatIdx;
			int32 deadMatIdx;
		};

		enum Flag
		{
			Fluid = 1u << 8,					/// Fluid particle.
			Gas = 1u << 9,						/// Gas particle.
			Wall = 1u << 10,					/// Zero velocity.
			Spring = 1u << 11,					/// With restitution from stretching.
			Elastic = 1u << 12,					/// With restitution from deformation.
			Viscous = 1u << 13,					/// With viscosity.
			Powder = 1u << 14,					/// Without isotropic pressure.
			Tensile = 1u << 15,					/// With surface tension.
			ColorMixing = 1u << 16,				/// Mix color between contacting particles.
			Barrier = 1u << 17,					/// Prevents other particles from leaking.
			StaticPressure = 1u << 18,			/// Less compressibility.
			Repulsive = 1u << 19,				/// With high repulsive force.
			HeatLoosing = 1u << 20,				/// makes Particles loose heat over time
			Flame = 1u << 21,					/// makes Particle ignite other inflamable Materials
			Inflammable = 1u << 22,
			Extinguishing = 1u << 23,
			HeatConducting = 1u << 24,
			ElectricityConducting = 1u << 25,
			KillIfNotMoving = 1u << 26, 

			ChangeWhenCold = 1u << 30,
			ChangeWhenHot = 1u << 31,
		};
		/// All particle types that require creating pairs
		static const uint32 k_pairFlags = Flag::Spring | Flag::Barrier;
		/// All particle types that require creating triads
		static const uint32 k_triadFlags = Flag::Elastic;
		/// All particle types that do not produce dynamic pressure
		static const uint32 k_noPressureFlags = Flag::Powder | Flag::Tensile;
		/// All particle types that apply extra damping force with bodies
		static const uint32 k_extraDampingFlags = Flag::StaticPressure;
		/// All particle types that apply extra damping force with bodies
		static const uint32 k_wallOrSpringOrElasticFlags = Flag::Wall | Flag::Spring | Flag::Elastic;
		static const uint32 k_barrierWallFlags = Flag::Wall | Flag::Barrier;
		static const uint32 k_changeFlags = Flag::ChangeWhenCold | Flag::ChangeWhenHot | Particle::Flag::Burning;

		/// For only getting mat flags from uint32
		static const uint32 k_mask = 0xFFFFFF00;

		uint32 m_flags;

		float32 m_mass;
		float32 m_invMass;
		float32 m_stability;
		float32 m_invStability;
		float32 m_heatConductivity;
		float32 m_strength;

		float32 m_coldThreshold;
		int32 m_changeToColdMatIdx;
		float32 m_hotThreshold;
		int32 m_changeToHotMatIdx;
		float32 m_ignitionThreshold;
		int32 m_changeToBurnedMatIdx;
		int32 m_changeToFireMatIdx;
		int32 m_changeToDeadMatIdx;


		bool Compare(const Def& def)
		{
			return def.flags == m_flags && def.mass == m_mass &&
				def.stability == m_stability && def.heatConductivity == m_heatConductivity;
		}

		void Set(const Def& def)
		{
			m_flags = def.flags;
			m_mass = def.mass;
			m_invMass = 1 / def.mass;
			m_stability = def.stability;
			m_invStability = 1 / def.stability;
			m_heatConductivity = def.heatConductivity;
			m_strength = def.strength;
		}

		void SetMatChanges(const ChangeDef& changeDef)
		{
			m_coldThreshold = changeDef.coldThreshold;
			m_changeToColdMatIdx = changeDef.coldMatIdx;
			m_hotThreshold = changeDef.hotThreshold;
			m_changeToHotMatIdx = changeDef.hotMatIdx;
			m_ignitionThreshold = changeDef.ignitionThreshold;
			m_changeToBurnedMatIdx = changeDef.burnedMatIdx;
			m_changeToFireMatIdx = changeDef.fireMatIdx;
			m_changeToDeadMatIdx = changeDef.deadMatIdx;
		}

		inline bool HasFlag(Flag flag) const { return m_flags & flag; }
		inline bool HasFlag(Flag flag) const restrict(amp) { return m_flags & flag; }
		inline bool IsWallSpringOrElastic() const { return m_flags & k_wallOrSpringOrElasticFlags; }
		inline bool IsWallSpringOrElastic() const restrict(amp) { return m_flags & k_wallOrSpringOrElasticFlags; }
	};

	/// For only getting particle flags from uint32
	static const uint32 k_mask = 0x000000FF;
};

/// A helper function to calculate the optimal number of iterations.
int32 b2CalculateParticleIterations(
	float32 gravity, float32 radius, float32 timeStep);

/// Handle to a particle. Particle indices are ephemeral: the same index might
/// refer to a different particle, from frame-to-frame. If you need to keep a
/// reference to a particular particle across frames, you should acquire a
/// b2ParticleHandle. Use #b2ParticleSystem::GetParticleHandleFromIndex() to
/// retrieve the b2ParticleHandle of a particle from the particle system.
class b2ParticleHandle : public b2TypedIntrusiveListNode<b2ParticleHandle>
{
	// Allow b2ParticleSystem to use SetIndex() to associate particle handles
	// with particle indices.
	friend class b2ParticleSystem;

public:
	/// Initialize the index associated with the handle to an invalid index.
	b2ParticleHandle() : m_index(INVALID_IDX) { }
	/// Empty destructor.
	~b2ParticleHandle() { }

	/// Get the index of the particle associated with this handle.
	int32 GetIndex() const { return m_index; }

private:
	/// Set the index of the particle associated with this handle.
	void SetIndex(int32 index) { m_index = index; }

private:
	// Index of the particle within the particle system.
	int32 m_index;
};

#if LIQUIDFUN_EXTERNAL_LANGUAGE_API
inline void b2ParticleDef::SetPosition(float32 x, float32 y)
{
	position.Set(x, y);
}

inline void b2ParticleDef::SetColor(int32 r, int32 g, int32 b, int32 a)
{
	color.Set((uint8)r, (uint8)g, (uint8)b, (uint8)a);
}
#endif // LIQUIDFUN_EXTERNAL_LANGUAGE_API

#endif
