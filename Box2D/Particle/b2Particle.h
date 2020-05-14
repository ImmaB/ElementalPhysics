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
		uint32 color;

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

		Mat() {};
		Mat(const Def& def);

		bool Compare(const Def& def)
		{
			return def.flags == m_flags && def.mass == m_mass &&
				def.stability == m_stability && def.heatConductivity == m_heatConductivity;
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

	struct MatArray
	{
		int32 m_count, m_capacity;

		amp::Array<Mat> m_array;
		std::vector<Mat> m_vector;

		MatArray(const ampAccelView& accelView);

		int32 Add(Mat::Def& def);
		void AddChange(const int32 matIdx, const Particle::Mat::ChangeDef& changeDef);
		void Clear() { m_count = 0; }
	};

	struct Buffers
	{
		std::vector<uint32> flags, color;
		std::vector<Vec3> position, velocity, force;
		std::vector<float32> weight, heat, health, mass, invMass,
			staticPressure, accumulation, depth;
		std::vector<int32> matIdx, groupIdx;

		Buffers(int32 cap);
		void Resize(int32 capacity);
	};

	struct AmpArrays
	{
		int32 m_count, m_capacity, m_iterations;

		amp::Array<uint32> m_flags, m_color;
		amp::Array<Vec3> m_position, m_velocity;
		amp::Array<float32> m_weight, m_heat, m_health;
		amp::Array<int32> m_matIdx;

		amp::Array<Vec3> m_force, m_accumulationVec3;
		amp::Array<float32> m_mass, m_invMass, m_staticPressure, m_accumulation, m_depth;
		amp::Array<int32> m_groupIdx;
		
		amp::Array<Proxy> m_proxy;


		AmpArrays(const ampAccelView& accelView);
		bool Empty() const { return m_count == 0; }
		bool Resize(int32 size);
		void SetD11Buffers(ID3D11Buffer** ppNewBufs);
		void WaitForCopies();

		template<typename F1, typename F2> void ForEachWithZombies(const F1& fn, const F2& zFn) const
		{
			auto flags = m_flags.GetConstView();
			amp::forEach(m_count, [=](const int32 i) restrict(amp)
			{
				if (flags[i] & Particle::Flag::Zombie) zFn(i);
				else fn(i);
			});
		}
		template<typename F> void ForEach(const F& function) const
		{
			auto flags = m_flags.GetConstView();
			amp::forEach(m_count, [=](const int32 i) restrict(amp)
			{
				if (!(flags[i] & Particle::Flag::Zombie)) function(i);
			});
		}
		template<typename F> void ForEach(const uint32 flag, const F& function) const
		{
			auto flags = m_flags.GetConstView();
			ForEach([=](const int32 i) restrict(amp)
			{
				if (flags[i] & flag) function(i);
			});
		}
		template<typename F> void ForEachProxy(const F& function) const
		{
			auto flags = m_flags.GetConstView();
			auto proxies = m_proxy.GetConstView();
			amp::forEach(m_count, [=](const int32 i) restrict(amp)
			{
				const Proxy proxy = proxies[i];
				if (!(flags[proxy.idx] & Particle::Flag::Zombie)) function(proxy);
			});
		}
	};
	
	void CopyBufferRangeToAmpArrays(Buffers& bufs,
		AmpArrays& arrs, int32 first, int32 last);
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
