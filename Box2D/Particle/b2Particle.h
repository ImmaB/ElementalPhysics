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
#pragma once

#include <Box2D/Common/b2Math.h>
#include <Box2D/Common/b2Settings.h>
#include <Box2D/Common/b2IntrusiveList.h>
#include <Box2D/Amp/ampAlgorithms.h>
#include <vector>
#include <d3d11.h>
#include <amp.h>

struct ParticleGroup;

namespace Particle
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
	/// For only getting particle flags from uint32
	static const uint32 k_mask = 0x000000FF;

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
		static const uint32 k_nonSolidFlags = Flag::Fluid | Flag::Gas;
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

	struct ContactIdx
	{
		int32 i, j;
		ContactIdx(int32 i, int32 j) restrict(amp) : i(i), j(j) {}
		void Set(int32 newI, int32 newJ) restrict(amp) { i = newI; j = newJ; }
	};

	struct Contact
	{
		int32 idxA, idxB;
		/// Weight of the contact. A value between 0.0f and 1.0f.
		/// 0.0f ==> particles are just barely touching
		/// 1.0f ==> particles are perfectly on top of each other
		float32 weight;
		float32 mass;
		/// The normalized direction from A to B.
		Vec3 normal;
		/// The logical sum of the particle behaviors that have been set.
		/// See the b2ParticleFlag enum.
		uint32 flags;

		bool operator==(const Contact& rhs) const;
		bool operator!=(const Contact& rhs) const { return !operator==(rhs); }
		bool ApproximatelyEqual(const Contact& rhs) const;

		inline bool HasFlag(const uint32 f) const { return flags & f; }
		inline bool HasFlag(const uint32 f) const restrict(amp) { return flags & f; }
		inline bool HasFlags(const uint32 f) const { return (flags & f) == f; }
		inline bool HasFlags(const uint32 f) const restrict(amp) { return (flags & f) == f; }
		inline bool IsZombie() const restrict(amp) { return HasFlag(Particle::Flag::Zombie); }
	};
	struct Contacts
	{
		Contact contacts[MAX_CONTACTS_PER_PARTICLE];
	};
	struct BodyContact
	{
		int32 partIdx;
		/// The body making contact.
		int32 bodyIdx;
		/// The specific fixture making contact
		int32 fixtureIdx;
		/// Weight of the contact. A value between 0.0f and 1.0f.
		float32 weight;
		/// The normalized direction from the particle to the body.
		Vec2 normal;
		/// The effective mass used in calculating force.
		float32 mass;
	};
	struct GroundContact
	{
		int32 groundTileIdx;
		int32 groundChunkIdx;
		int32 groundMatIdx;
		float32 weight;
		Vec3 normal;
		float32 mass;

		void setInvalid() { groundTileIdx = INVALID_IDX; }
		void setInvalid() restrict(amp) { groundTileIdx = INVALID_IDX; }
		bool getValid() const { return groundTileIdx != INVALID_IDX; }
		bool getValid() const restrict(amp) { return groundTileIdx != INVALID_IDX; }
	};

	struct D11Buffers
	{
		ID3D11Buffer* flags, *matIdx, *position, *velocity, *weight,
			*heat, *health, *color, *contactIdx, *contact;

		void Set(ID3D11Buffer** bufPtrs, bool includeContacts);
	};

	struct Buffers
	{
		std::vector<uint32> flags;
		std::vector<Vec3> position, velocity, force, accumulationVec3;
		std::vector<float32> weight, heat, health, mass, invMass,
			staticPressure, accumulation, depth;
		std::vector<int32> matIdx, groupIdx, color, bodyContactCnt;
		std::vector<Proxy> proxy;

		Buffers(int32 cap);
		void Resize(int32 capacity);
	};

	struct AmpArrays
	{
		ampArray<uint32> flags;
		ampArray<Vec3> position, velocity, force, accumulationVec3;
		ampArray<float32> weight, heat, health, mass, invMass,
			staticPressure, accumulation, depth;
		ampArray<int32>	matIdx, groupIdx, color, contactCnt, bodyContactCnt;
		ampArray<Proxy> proxy;
		ampArray<ContactIdx> contactIdx;
		ampArray<Contacts> contact;
		ampArray2D<BodyContact> bodyContact;
		ampArray<GroundContact> groundContact;

		AmpArrays(int32 cap, const ampAccelView& accelView);
		void Resize(int32 capacity, int32 copyCnt);
	};

	void CopyBufferRangeToAmpArrays(Buffers& bufs,
		AmpArrays& arrs, int32 first, int32 last);
	void CopyAmpArraysToD11Buffers(D11Device& device, const AmpArrays& arrs,
		D11Buffers& d11Bufs, int32 cnt, int32 contactCnt = 0);
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
	friend class ParticleSystem;

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
