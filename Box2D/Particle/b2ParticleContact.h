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
#include <Box2D/Particle/b2Particle.h>
#include <Box2D/Amp/ampAlgorithms.h>
#include <vector>
#include <d3d11.h>
#include <amp.h>

namespace Particle
{
	struct ContactIdx
	{
		int32 i, j;
		ContactIdx(int32 i, int32 j) restrict(amp) : i(i), j(j) {}
		void Set(int32 newI, int32 newJ) restrict(amp) { i = newI; j = newJ; }
	};

	struct BodyContact
	{
		enum Flag
		{
			Touching = 1 << 0	// otherwise its a potential collision
		};

		uint32 flags;
		/// The body making contact.
		int32 bodyIdx;
		/// The specific fixture making contact
		int32 fixtureIdx;
		/// The specific fixture child making contact
		int32 childIdx;
		/// Weight of the contact. A value between 0.0f and 1.0f.
		float32 weight;
		/// The normalized direction from the particle to the body.
		Vec3 normal;
		/// The effective mass used in calculating force.
		float32 mass;

		inline void AddFlag(const uint32 f) restrict(amp) { flags |= f; }
		inline bool HasFlag(const uint32 f) const restrict(amp) { return flags & f; }
		inline bool IsReal() const restrict(amp) { return flags & Touching; }
	};
	struct BodyContacts
	{
		BodyContact contacts[MAX_BODY_CONTACTS_PER_PARTICLE];
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

		Contact() restrict(amp) : idxA(INVALID_IDX), idxB(INVALID_IDX), weight(0), mass(0), normal(), flags(0) {}
		Contact() : idxA(INVALID_IDX), idxB(INVALID_IDX), weight(0), mass(0), normal(), flags(0) {}

		Contact(int32 idxA, int32 idxB, float32 weight, float32 mass,
			Vec3 normal, uint32 flags) restrict(amp) :
			idxA(idxA), idxB(idxB), weight(weight), mass(mass),
			normal(normal), flags(flags)
		{}


		bool operator==(const Contact& rhs) const;
		bool operator!=(const Contact& rhs) const { return !operator==(rhs); }
		bool ApproximatelyEqual(const Contact& rhs) const;

		inline bool HasFlag(const uint32 f) const { return flags & f; }
		inline bool HasFlag(const uint32 f) const restrict(amp) { return flags & f; }
		inline bool HasFlags(const uint32 f) const { return (flags & f) == f; }
		inline bool HasFlags(const uint32 f) const restrict(amp) { return (flags & f) == f; }
		inline bool IsZombie() const restrict(amp) { return HasFlag(Particle::Flag::Zombie); }
	};

	class ContactArrays
	{
	public:
		const Particle::AmpArrays& m_particleArrays;
		int32 m_count, m_capacity;
		uint32 m_maxPerTile;

		amp::Array<uint32> m_idx;
		amp::Array<int32> m_cnt;
		amp::Array2D<Contact> m_perTile;

		ContactArrays(const ampAccelView& accelView, const Particle::AmpArrays& particleArrays);

		bool Empty() const { return m_count == 0; }
		bool Resize(int32 size);

		template<typename F> void ForEach(const F& function)  const
		{
			auto idxs = m_idx.GetConstView();
			auto contacts = m_perTile.GetConstView();
			const uint32 maxPerTile = m_maxPerTile;
			amp::forEach(m_count, [=](const int32 i) restrict(amp)
			{
				const uint32 idx = idxs[i];
				function(contacts[idx / maxPerTile][idx % maxPerTile]);
			});
		}
		template<typename F> void ForEach(const uint32 flag, const F& function) const
		{
			ForEach([=](const Particle::Contact& contact) restrict(amp)
			{
				if (contact.HasFlags(flag)) function(contact);
			});
		}
		template<typename F> void ShuffledForEach(const F& function) const
		{
			const int32 count = m_count;
			const uint32 blockSize = amp::getTileCount(count);

			//auto& contacts = m_ampContacts;
			auto idxs = m_idx.GetConstView();
			auto contacts = m_perTile.GetConstView();
			const uint32 maxPerTile = m_maxPerTile;
			amp::forEachTiledWithBarrier(m_count,
				[=](const ampTiledIdx<TILE_SIZE>& tIdx) restrict(amp)
			{
				const uint32 gi = tIdx.global[0];
				const uint32 li = gi % blockSize;
				const uint32 bi = gi / blockSize;

				const uint32 lis = bi % MAX_CONTACTS_PER_PARTICLE;
				const uint32 bis = bi / MAX_CONTACTS_PER_PARTICLE;

				const uint32 shuffledbi = bis * MAX_CONTACTS_PER_PARTICLE + lis;
				const uint32 shuffledIdx = shuffledbi * blockSize + li;

				if (shuffledIdx >= count) return;
				const uint32 idx = idxs[shuffledIdx];
				function(contacts[idx / maxPerTile][idx % maxPerTile]);
			});
		}
		template<typename F> void ShuffledForEach(const uint32 flag, const F& function) const
		{
			ShuffledForEach([=](const Particle::Contact& c) restrict(amp)
			{
				if (c.flags & flag) function(c);
			});
		}
	};

	class BodyContactArrays
	{
	public:
		const Particle::AmpArrays& m_particleArrays;
		int32 m_count, m_capacity;

		amp::Array<BodyContacts> m_array;
		amp::Array<ContactIdx> m_idx;
		amp::Array<int32> m_cnt;

		BodyContactArrays(const ampAccelView& accelView,
			const Particle::AmpArrays& particleArrays);

		bool Empty() const { return m_count == 0; }
		void Clear() { m_count = 0; }
		void Resize(int32 size);
		template<typename F> void ForEachPotential(const F& function) const
		{
			auto idxs = m_idx.GetConstView();
			auto bodyContacts = m_array.GetConstView();
			amp::forEach(m_count, [=](const int32 i) restrict(amp)
			{
				const Particle::ContactIdx idx = idxs[i];
				const Particle::BodyContact& c = bodyContacts[idx.i].contacts[idx.j];
				function(idx.i, c);
			});
		}
		template<typename F> void ForEach(const F& function) const
		{
			ForEachPotential([=](const int32 i, const Particle::BodyContact& c) restrict(amp)
			{
				if (c.IsReal()) function(i, c);
			});
		}
		template<typename F> void ForEach(const uint32 flag, const F& function) const
		{
			auto flags = m_particleArrays.m_flags.GetConstView();
			ForEach([=](const int32 i, const Particle::BodyContact& contact) restrict(amp)
			{
				if (flags[i] & flag) function(i, contact);
			});
		}
	};

	class GroundContactArrays
	{
	public:
		const Particle::AmpArrays& m_particleArrays;
		int32 m_count, m_capacity;
		amp::Array<GroundContact> m_array;

		GroundContactArrays(const ampAccelView& accelView,
			const Particle::AmpArrays& particleArrays);

		void Resize(int32 size);
		template<typename F> void ForEach(const F& function) const
		{
			auto groundContacts = m_array.GetConstView();
			amp::forEach(m_particleArrays.m_count, [=](const int32 i) restrict(amp)
			{
				const Particle::GroundContact& contact = groundContacts[i];
				if (!contact.getValid()) return;
				function(i, contact);
			});
		}
		template<typename F> void ForEach(const uint32 partFlag, const F& function) const
		{
			auto groundContacts = m_array.GetConstView();
			auto flags = m_particleArrays.m_flags.GetConstView();
			amp::forEach(m_particleArrays.m_count, [=](const int32 i) restrict(amp)
			{
				if (!(flags[i] & partFlag)) return;
				const Particle::GroundContact& contact = groundContacts[i];
				if (contact.getValid())
					function(i, contact);
			});
		}
	};
};
