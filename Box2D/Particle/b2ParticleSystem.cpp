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
#define NOMINMAX
#include <Box2D/Particle/b2ParticleSystem.h>
#include <Box2D/Particle/b2VoronoiDiagram.h>
#include <Box2D/Particle/b2ParticleAssembly.h>
#include <Box2D/Common/b2BlockAllocator.h>
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Dynamics/Ground.h>
#include <Box2D/Dynamics/b2WorldCallbacks.h>
#include <algorithm>
#include <intrin.h>
#include <ppl.h>

// Define LIQUIDFUN_SIMD_TEST_VS_REFERENCE to run both SIMD and reference
// versions, and assert that the results are identical. This is useful when
// modifying one of the functions, to help verify correctness.
// #define LIQUIDFUN_SIMD_TEST_VS_REFERENCE

// For ease of debugging, remove 'inline'. Then, when an assert hits in the
// test-vs-reference functions, you can easily jump the instruction pointer
// to the top of the function to re-run the test.
#define LIQUIDFUN_SIMD_INLINE inline

using namespace std;

static const uint32 xTruncBits = 12;
static const uint32 yTruncBits = 12;
static const uint32 tagBits = 8u * sizeof(uint32);
static const uint32 yOffset = 1u << (yTruncBits - 1u);
static const uint32 yShift = tagBits - yTruncBits;
static const uint32 xShift = tagBits - yTruncBits - xTruncBits;
static const uint32 xScale = 1u << xShift;
static const uint32 xOffset = xScale * (1u << (xTruncBits - 1u));
static const uint32 yMask = ((1u << yTruncBits) - 1u) << yShift;
static const uint32 xMask = ~yMask;
static const uint32 relativeTagRight = 1u << xShift;
static const uint32 relativeTagBottomLeft = (uint32)((1 << yShift) +
                                                    (-1 << xShift));

static const uint32 relativeTagBottomRight = (1u << yShift) + (1u << xShift);

// This functor is passed to std::remove_if in RemoveSpuriousBodyContacts
// to implement the algorithm described there.  It was hoisted out and friended
// as it would not compile with g++ 4.6.3 as a local class.  It is only used in
// that function.
class b2ParticleBodyContactRemovePredicate
{
public:
	b2ParticleBodyContactRemovePredicate(b2World& world, b2ParticleSystem& system,
										 int32* discarded)
		: m_world(world), m_system(system), m_lastIndex(-1), m_currentContacts(0),
		  m_discarded(discarded) {}

	/*bool operator()(const b2PartBodyContact& contact)
	{
		// This implements the selection criteria described in
		// RemoveSpuriousBodyContacts().
		// This functor is iterating through a list of Body contacts per
		// Particle, ordered from near to far.  For up to the maximum number of
		// contacts we allow per point per step, we verify that the contact
		// normal of the Body that genenerated the contact makes physical sense
		// by projecting a point back along that normal and seeing if it
		// intersects the fixture generating the contact.
		
		int32 particleIdx = contact.idx;
	
		if (particleIdx != m_lastIndex)
		{
			m_currentContacts = 0;
			m_lastIndex = particleIdx;
		}
	
		if (m_currentContacts++ > k_maxContactsPerPoint)
		{
			++(*m_discarded);
			return true;
		}
	
		// Project along inverse normal (as returned in the contact) to get the
		// point to check.
		b2Vec2 n = contact.normal;
		// weight is 1-(inv(diameter) * distance)
		n *= m_system->m_particleDiameter * (1 - contact.weight);
		b2Vec2 pos = n + (m_system->m_positionBuffer[particleIdx]);
	
		// pos is now a point projected back along the contact normal to the
		// contact distance. If the surface makes sense for a contact, pos will
		// now lie on or in the fixture generating
		b2Fixture* fixture = contact.fixture;
		if (!fixture->TestPoint(pos))
		{
			int32 childCount = fixture->GetShape()->GetChildCount();
			for (int32 childIndex = 0; childIndex < childCount; childIndex++)
			{
				float32 distance;
				b2Vec2 normal;
				fixture->ComputeDistance(pos, &distance, &normal,
																	childIndex);
				if (distance < b2_linearSlop)
				{
					return false;
				}
			}
			++(*m_discarded);
			return true;
		}
	
		return false;
	}*/
private:
	// Max number of contacts processed per particle, from nearest to farthest.
	// This must be at least 2 for correctness with concave shapes; 3 was
	// experimentally arrived at as looking reasonable.
	static const int32 k_maxContactsPerPoint = 3;
	const b2ParticleSystem& m_system;
	const b2World& m_world;
	// Index of last particle processed.
	int32 m_lastIndex;
	// Number of contacts processed for the current particle.
	int32 m_currentContacts;
	// Output the number of discarded contacts.
	int32* m_discarded;
};

namespace {

// Compares the expiration time of two particle indices.
class ExpirationTimeComparator
{
public:
	// Initialize the class with a pointer to an array of particle
	// lifetimes.
	ExpirationTimeComparator(const int32* const expirationTimes) :
		m_expirationTimes(expirationTimes)
	{
	}
	// Empty destructor.
	~ExpirationTimeComparator() { }

	// Compare the lifetime of particleIndexA and particleIndexB
	// returning true if the lifetime of A is greater than B for particles
	// that will expire.  If either particle's lifetime is infinite (<= 0.0f)
	// this function return true if the lifetime of A is lesser than B.
	// When used with std::sort() this results in an array of particle
	// indicies sorted in reverse order by particle lifetime.
	// For example, the set of lifetimes
	// (1.0, 0.7, 0.3, 0.0, -1.0, -2.0)
	// would be sorted as
	// (0.0, -1.0, -2.0, 1.0, 0.7, 0.3)
	bool operator() (const int32 particleIndexA,
					 const int32 particleIndexB) const
	{
		const int32 expirationTimeA = m_expirationTimes[particleIndexA];
		const int32 expirationTimeB = m_expirationTimes[particleIndexB];
		const bool infiniteExpirationTimeA = expirationTimeA <= 0.0f;
		const bool infiniteExpirationTimeB = expirationTimeB <= 0.0f;
		return infiniteExpirationTimeA == infiniteExpirationTimeB ?
			expirationTimeA > expirationTimeB : infiniteExpirationTimeA;
	}

private:
	const int32* m_expirationTimes;
};

// *Very* lightweight pair implementation.
template<typename A, typename B>
struct LightweightPair
{
	A first;
	B second;

	// Compares the value of two FixtureParticle objects returning
	// true if left is a smaller value than right.
	static bool Compare(const LightweightPair& left,
						const LightweightPair& right)
	{
		return left.first < right.first &&
			left.second < right.second;
	}

};

// Allocator for a fixed set of items.
class FixedSetAllocator
{
public:
	// Associate a memory allocator with this object.
	FixedSetAllocator(b2StackAllocator* allocator);
	// Deallocate storage for this class.
	~FixedSetAllocator()
	{
		Clear();
	}

	// Allocate internal storage for this object returning the size.
	int32 Allocate(const int32 itemSize, const int32 count);

	// Deallocate the internal buffer if it's allocated.
	void Clear();

	// Get the number of items in the set.
	int32 GetCount() const { return m_count; }

	// Invalidate an item from the set by index.
	void Invalidate(const int32 itemIndex)
	{
		b2Assert(m_valid);
		m_valid[itemIndex] = 0;
	}

	// Get the buffer which indicates whether items are valid in the set.
	const int8* GetValidBuffer() const { return m_valid; }

protected:
	// Get the internal buffer.
	void* GetBuffer() const { return m_buffer; }
	void* GetBuffer() { return m_buffer; }

	// Reduce the number of items in the set.
	void SetCount(int32 count)
	{
		b2Assert(count <= m_count);
		m_count = count;
	}

private:
	// Set buffer.
	void* m_buffer;
	// Array of size m_count which indicates whether an item is in the
	// corresponding index of m_set (1) or the item is invalid (0).
	int8* m_valid;
	// Number of items in m_set.
	int32 m_count;
	// Allocator used to allocate / free the set.
	b2StackAllocator* m_allocator;
};

// Allocator for a fixed set of objects.
template<typename T>
class TypedFixedSetAllocator : public FixedSetAllocator
{
public:
	// Initialize members of this class.
	TypedFixedSetAllocator(b2StackAllocator* allocator) :
		FixedSetAllocator(allocator) { }

	// Allocate a set of objects, returning the new size of the set.
	int32 Allocate(const int32 numberOfObjects)
	{
		Clear();
		return FixedSetAllocator::Allocate(sizeof(T), numberOfObjects);
	}

	// Get the index of an item in the set if it's valid return an index
	// >= 0, -1 otherwise.
	int32 GetIndex(const T* item) const
	{
		if (item)
		{
			b2Assert(item >= GetBuffer() &&
					 item < GetBuffer() + GetCount());
			const int32 index =
				(int32)(((uint8*)item - (uint8*)GetBuffer()) /
						sizeof(*item));
			if (GetValidBuffer()[index])
			{
				return index;
			}
		}
		return -1;
	}

	// Get the internal buffer.
	const T* GetBuffer() const
	{
		return (const T*)FixedSetAllocator::GetBuffer();
	}
	T* GetBuffer() { return (T*)FixedSetAllocator::GetBuffer(); }
};

// Associates a fixture with a particle index.
typedef LightweightPair<int32, int32> FixtureParticle;

// Associates a fixture with a particle index.
typedef LightweightPair<int32, int32> ParticlePair;

}  // namespace

// Set of fixture / particle indices.
class FixtureParticleSet :
	public TypedFixedSetAllocator<FixtureParticle>
{
public:
	// Initialize members of this class.
	FixtureParticleSet(b2StackAllocator* allocator) :
		TypedFixedSetAllocator<FixtureParticle>(allocator) { }


	// Initialize from a set of particle / body contacts for particles
	// that have the b2_fixtureContactListenerParticle flag set.
	void Initialize(const vector<b2PartBodyContact>& bodyContacts,
					const int32 numBodyContacts,
					const uint32 * const particleFlagsBuffer);

	// Find the index of a particle / fixture pair in the set or -1
	// if it's not present.
	// NOTE: This was not written as a template function to avoid
	// exposing any dependencies via this header.
	int32 Find(const FixtureParticle& fixtureParticle) const;
};

// Set of particle / particle pairs.
class b2ParticlePairSet : public TypedFixedSetAllocator<ParticlePair>
{
public:
	// Initialize members of this class.
	b2ParticlePairSet(b2StackAllocator* allocator) :
		TypedFixedSetAllocator<ParticlePair>(allocator) { }

	// Initialize from a set of particle contacts.
	void Initialize(const vector<int32> contactIdxAs, const vector<int32> contactIdxBs,
					const int32 numContacts,
					const vector<uint32> particleFlagsBuffer);

	// Find the index of a particle pair in the set or -1
	// if it's not present.
	// NOTE: This was not written as a template function to avoid
	// exposing any dependencies via this header.
	int32 Find(const ParticlePair& pair) const;
};

static inline uint32 computeTag(float32 x, float32 y)
{
	return ((uint32)(y + yOffset) << yShift) + (uint32)(xScale * x + xOffset);
}
static inline uint32 computeTag(float32 x, float32 y) restrict(amp)
{
	return ((uint32)(y + yOffset) << yShift) + (uint32)(xScale * x + xOffset);
}

static inline uint32 computeRelativeTag(uint32 tag, int32 x, int32 y)
{
	return tag + (y << yShift) + (x << xShift);
}
static inline uint32 computeRelativeTag(uint32 tag, uint32 x, uint32 y) restrict(amp)
{
	return tag + (y << yShift) + (x << xShift);
}

b2ParticleSystem::InsideBoundsEnumerator::InsideBoundsEnumerator(
	uint32 lower, uint32 upper, const Proxy* first, const Proxy* last)
{
	m_xLower = lower & xMask;
	m_xUpper = upper & xMask;
	m_yLower = lower & yMask;
	m_yUpper = upper & yMask;
	m_first = first;
	m_last = last;
	b2Assert(m_first <= m_last);
}

int32 b2ParticleSystem::InsideBoundsEnumerator::GetNext()
{
	while (m_first < m_last)
	{
		uint32 xTag = m_first->tag & xMask;
#if B2_ASSERT_ENABLED
		uint32 yTag = m_first->tag & yMask;
		b2Assert(yTag >= m_yLower);
		b2Assert(yTag <= m_yUpper);
#endif
		if (xTag >= m_xLower && xTag <= m_xUpper)
		{
			return (m_first++)->idx;
		}
		m_first++;
	}
	return b2_invalidIndex;
}

b2ParticleSystem::b2ParticleSystem(b2World& world, vector<Body>& bodyBuffer, vector<Fixture>& fixtureBuffer) :
	m_handleAllocator(b2_minParticleBufferCapacity),
	m_ampGroups(1, m_gpuAccelView),

	// Box2D
	m_ampBodies(TILE_SIZE, m_gpuAccelView),
	m_ampFixtures(TILE_SIZE, m_gpuAccelView),
	m_ampChainShapes(TILE_SIZE, m_gpuAccelView),
	m_ampCircleShapes(TILE_SIZE, m_gpuAccelView),
	m_ampEdgeShapes(TILE_SIZE, m_gpuAccelView),
	m_ampPolygonShapes(TILE_SIZE, m_gpuAccelView),

	// Contacts
	m_ampContacts(TILE_SIZE, m_gpuAccelView),
	m_ampBodyContacts(TILE_SIZE, m_gpuAccelView),

	m_ampProxies(TILE_SIZE, m_gpuAccelView),
	m_localContactCnts(TILE_SIZE, m_gpuAccelView),
	m_localContacts(TILE_SIZE, MAX_CONTACTS_PER_PARTICLE, m_gpuAccelView),
	m_localBodyContactCnts(TILE_SIZE, m_gpuAccelView),
	m_localBodyContacts(TILE_SIZE, TILE_SIZE, m_gpuAccelView),

	// Particles
	m_ampHealths(TILE_SIZE, m_gpuAccelView),
	m_ampHeats(TILE_SIZE, m_gpuAccelView),
	m_ampWeights(TILE_SIZE, m_gpuAccelView),
	m_ampStaticPressures(TILE_SIZE, m_gpuAccelView),
	m_ampFlags(TILE_SIZE, m_gpuAccelView),
	m_ampDepths(TILE_SIZE, m_gpuAccelView),
	m_ampAccumulations(TILE_SIZE, m_gpuAccelView),
	m_ampAccumulationVec3s(TILE_SIZE, m_gpuAccelView),
	m_ampPositions(TILE_SIZE, m_gpuAccelView),
	m_ampVelocities(TILE_SIZE, m_gpuAccelView),
	m_ampForces(TILE_SIZE, m_gpuAccelView),
	m_ampMatIdxs(TILE_SIZE, m_gpuAccelView),
	m_ampGroupIdxs(TILE_SIZE, m_gpuAccelView),
	m_ampColors(TILE_SIZE, m_gpuAccelView),

	// Materials
	m_ampMatFlags(1, m_gpuAccelView),
	m_ampMatPartFlags(1, m_gpuAccelView),
	m_ampMatMasses(1, m_gpuAccelView),
	m_ampMatInvMasses(1, m_gpuAccelView),
	m_ampMatStabilities(1, m_gpuAccelView),
	m_ampMatInvStabilities(1, m_gpuAccelView),
	m_ampMatColderThans(1, m_gpuAccelView),
	m_ampMatChangeToColdMats(1, m_gpuAccelView),
	m_ampMatHotterThans(1, m_gpuAccelView),
	m_ampMatChangeToHotMats(1, m_gpuAccelView),
	m_ampMatIgnitionPoints(1, m_gpuAccelView),
	m_ampMatBurnToMats(1, m_gpuAccelView),
	m_ampMatHeatConductivities(1, m_gpuAccelView),


	// pairs and triads
	m_ampPairs(TILE_SIZE, m_gpuAccelView),
	m_ampTriads(TILE_SIZE, m_gpuAccelView),

	m_world(world)
{
	m_paused = false;
	m_timestamp = 0;
	m_allParticleFlags = 0;
	m_needsUpdateAllParticleFlags = true;
	m_allGroupFlags = 0;
	m_needsUpdateAllGroupFlags = false;
	m_hasForce = false;
	m_hasDepth = false;
	m_iteration = 0;

	SetStrictContactCheck(false);
	SetDensity(1.0f);
	SetGravityScale(1.0f);
	SetRadius(1.0f);
	SetMaxParticleCount(0);

	m_freeGroupIdxs.reserve(256);

	m_count = 0;
	m_capacity = 0;
	m_groupCount = 0;
	m_groupCapacity = 0;
	m_partMatCount = 0;
	m_partMatCapacity = 0;
	m_contactCount = 0;
	m_contactCapacity = 0;
	m_bodyContactCount = 0;
	m_bodyContactCapacity = 0;
	m_pairCount = 0;
	m_pairCapacity = 0;
	m_triadCount = 0;
	m_triadCapacity = 0;

	m_hasColorBuf					 = false;
	hasHandleIndexBuffer			 = false;
	hasStaticPressureBuf			 = false;
	hasAccumulation2Buf				 = false;
	hasLastBodyContactStepBuffer	 = false;
	hasBodyContactCountBuffer		 = false;
	hasConsecutiveContactStepsBuffer = false;

	b2Assert(1.0f / 60.0f > 0.0f);

	m_stuckThreshold = 0;

	m_timeElapsed = 0;
	m_expirationTimeBufferRequiresSorting = false;

	SetDestructionByAge(m_def.destroyByAge);
}

b2ParticleSystem::~b2ParticleSystem()
{
	for (int i = 0; i < m_groupCount; i++)
		DestroyGroup(i, 0, true);

	amp::uninitialize();
}

template <typename T> void b2ParticleSystem::FreeBuffer(T** b, int capacity)
{
	if (*b == NULL)
		return;

	m_world.m_blockAllocator.Free(*b, sizeof(**b) * capacity);
	*b = NULL;
}

// Free buffer, if it was allocated with b2World's block allocator
template <typename T> void b2ParticleSystem::FreeUserOverridableBuffer(
	UserOverridableBuffer<T>* b)
{
	if (b->userSuppliedCapacity == 0)
		FreeBuffer(&b->data, m_capacity);
}

// Reallocate a buffer
template <typename T> T* b2ParticleSystem::ReallocateBuffer(
	T* oldBuffer, int32 oldCapacity, int32 newCapacity)
{
	b2Assert(newCapacity > oldCapacity);
	T* newBuffer = (T*) m_world.m_blockAllocator.Allocate(
		sizeof(T) * newCapacity);
	if (oldBuffer)
	{
		memcpy(newBuffer, oldBuffer, sizeof(T) * oldCapacity);
		m_world.m_blockAllocator.Free(oldBuffer, sizeof(T) * oldCapacity);
	}
	return newBuffer;
}

// Reallocate a buffer
template <typename T> T* b2ParticleSystem::ReallocateBuffer(
	T* buffer, int32 userSuppliedCapacity, int32 oldCapacity,
	int32 newCapacity, bool deferred)
{
	b2Assert(newCapacity > oldCapacity);
	// A 'deferred' buffer is reallocated only if it is not NULL.
	// If 'userSuppliedCapacity' is not zero, buffer is user supplied and must
	// be kept.
	b2Assert(!userSuppliedCapacity || newCapacity <= userSuppliedCapacity);
	if ((!deferred || buffer) && !userSuppliedCapacity)
	{
		buffer = ReallocateBuffer(buffer, oldCapacity, newCapacity);
	}
	return buffer;
}

// Reallocate a buffer
template <typename T> T* b2ParticleSystem::ReallocateBuffer(
	UserOverridableBuffer<T>* buffer, int32 oldCapacity, int32 newCapacity,
	bool deferred)
{
	b2Assert(newCapacity > oldCapacity);
	return ReallocateBuffer(buffer->data, buffer->userSuppliedCapacity,
							oldCapacity, newCapacity, deferred);
}

/// Reallocate the handle / index map and schedule the allocation of a new
/// pool for handle allocation.
void b2ParticleSystem::ReallocateHandleBuffers(int32 newCapacity)
{
	b2Assert(newCapacity > m_capacity);
	// Reallocate a new handle / index map buffer, copying old handle pointers
	// is fine since they're kept around.
	m_handleIndexBuffer.resize(newCapacity);
	// Set the size of the next handle allocation.
	m_handleAllocator.SetItemsPerSlab(newCapacity -
									  m_capacity);
}

template <typename T> 
void b2ParticleSystem::RequestBuffer(vector<T>& buf, bool& hasBuffer)
{
	if (!hasBuffer)
	{
		buf.resize(m_capacity);
		hasBuffer = true;
	}
}
template <typename T>
void b2ParticleSystem::AmpRequestBuffer(ampArray<T>& a, bool& hasBuffer)
{
	if (!hasBuffer)
	{
		amp::resize(a, m_capacity);
		hasBuffer = true;
	}
}

int32* b2ParticleSystem::GetColorBuffer()
{
	return m_colorBuffer.data();
}

template<typename F>
inline void b2ParticleSystem::AmpForEachParticle(const F& function) const
{
	auto& flags = m_ampFlags;
	const uint32 zombieFlag = b2_zombieParticle;
	amp::forEach(m_count, [=, &flags](const int32 i) restrict (amp)
	{
		if (flags[i] & zombieFlag) return;
		function(i);
	});
}
template<typename F>
inline void b2ParticleSystem::AmpForEachParticle(const uint32 filterFlag, const F& function) const
{
	auto& flags = m_ampFlags;
	const uint32 zombieFlag = b2_zombieParticle;
	amp::forEach(m_count, [=, &flags](const int32 i) restrict(amp)
	{
		if (flags[i] & zombieFlag) return;
		if (flags[i] & filterFlag)
			function(i);
	});
}
template<typename F>
inline void b2ParticleSystem::AmpForEachContact(const F& function) const
{
	auto& contacts = m_ampContacts;
	amp::forEach(m_contactCount, [=, &contacts](const int32 i) restrict(amp)
	{
		function(contacts[i]);
	});
}
template<typename F>
inline void b2ParticleSystem::AmpForEachContactShuffled(const F& function) const
{
	const int32 contactCnt = m_contactCount;
	const uint32 blockSize = amp::getTileCount(contactCnt);
	const uint32 subBlockSize = MAX_CONTACTS_PER_PARTICLE;

	auto& contacts = m_ampContacts;
	amp::forEachTiledWithBarrier(m_contactCount, [=,&contacts](const ampTiledIdx<TILE_SIZE>& tIdx) restrict(amp)
	{
		const uint32 gi = tIdx.global[0];
		const uint32 li = gi % blockSize;
		const uint32 bi = gi / blockSize;

		const uint32 lis = bi % subBlockSize;
		const uint32 bis = bi / subBlockSize;

		const uint32 shuffledbi = bis * subBlockSize + lis;
		const uint32 shuffledIdx = shuffledbi * blockSize + li;

		if (shuffledIdx < contactCnt)
			function(contacts[shuffledIdx]);
	});
}
template<typename F>
inline void b2ParticleSystem::AmpForEachBodyContact(const F& function) const
{
	auto& bodyContacts = m_ampBodyContacts;
	amp::forEach(m_bodyContactCount, [=, &bodyContacts](const int32 i) restrict(amp)
	{
		function(bodyContacts[i]);
	});
}
template<typename F>
inline void b2ParticleSystem::AmpForEachPair(F& function) const
{
	amp::forEach(m_pairCount, [=](const int32 i) restrict(amp)
	{
		function(i);
	});
}
template<typename F>
inline void b2ParticleSystem::AmpForEachTriad(F& function) const
{
	amp::forEach(m_triadCount, [=](const int32 i) restrict(amp)
	{
		function(i);
	});
}

void b2ParticleSystem::ResizePartMatBuffers(int32 size)
{
	if (size < b2_minGroupBufferCapacity) size = b2_minPartMatBufferCapacity;
	m_partMatFlagsBuf.resize(size);
	m_partMatPartFlagsBuf.resize(size);
	m_partMatMassBuf.resize(size);
	m_partMatInvMassBuf.resize(size);
	m_partMatStabilityBuf.resize(size);
	m_partMatInvStabilityBuf.resize(size);
	m_partMatColderThanBuf.resize(size);
	m_partMatChangeToColdMatBuf.resize(size);
	m_partMatHotterThanBuf.resize(size);
	m_partMatChangeToHotMatBuf.resize(size);
	m_partMatIgnitionPointBuf.resize(size);
	m_partMatBurnToMatBuf.resize(size);
	m_partMatHeatConductivityBuf.resize(size);

	amp::resize(m_ampMatFlags, size);
	amp::resize(m_ampMatPartFlags, size);
	amp::resize(m_ampMatMasses, size);
	amp::resize(m_ampMatInvMasses, size);
	amp::resize(m_ampMatStabilities, size);
	amp::resize(m_ampMatInvStabilities, size);
	amp::resize(m_ampMatColderThans, size);
	amp::resize(m_ampMatChangeToColdMats, size);
	amp::resize(m_ampMatHotterThans, size);
	amp::resize(m_ampMatChangeToHotMats, size);
	amp::resize(m_ampMatIgnitionPoints, size);
	amp::resize(m_ampMatBurnToMats, size);
	amp::resize(m_ampMatHeatConductivities, size);

	m_partMatCapacity = size;
}

inline boolean b2ParticleSystem::AdjustCapacityToSize(int32& capacity, int32 size, const int32 minCapacity) const
{
	if (size < minCapacity)
		size = minCapacity;
	if (size > capacity)		// grow if needed
	{
		capacity *= 2;
		if (capacity == 0) capacity = minCapacity;
		while (size > capacity)
			capacity *= 2;
		return true;
	}
	if (size < capacity / 4)	// shrink if quarter
	{
		capacity /= 2;
		while (size < capacity / 4)
			capacity /= 2;
		return true;
	}
	return false;
}

void b2ParticleSystem::ResizeParticleBuffers(int32 size)
{
	if (!AdjustCapacityToSize(m_capacity, size, b2_minParticleBufferCapacity)) return;

	ReallocateHandleBuffers(m_capacity);
	m_flagsBuffer.resize(m_capacity);

	m_lastBodyContactStepBuffer.resize(m_capacity);
	m_bodyContactCountBuffer.resize(m_capacity);
	m_consecutiveContactStepsBuffer.resize(m_capacity);

	m_positionBuffer.resize(m_capacity);
	m_velocityBuffer.resize(m_capacity);
	m_forceBuffer.resize(m_capacity);

	amp::resize(m_ampPositions, m_capacity, m_count);
	amp::resize(m_ampVelocities, m_capacity, m_count);
	amp::resize(m_ampForces, m_capacity, m_count);

	m_weightBuffer.resize(m_capacity);
	m_staticPressureBuf.resize(m_capacity);
	m_accumulationBuf.resize(m_capacity);
	m_accumulation3Buf.resize(m_capacity);
	m_depthBuffer.resize(m_capacity);
	m_colorBuffer.resize(m_capacity);
	m_partGroupIdxBuffer.resize(m_capacity);
	m_partMatIdxBuffer.resize(m_capacity);

	amp::resize(m_ampWeights, m_capacity);
	amp::resize(m_ampStaticPressures, m_capacity, m_count);
	amp::resize(m_ampAccumulations, m_capacity);
	amp::resize(m_ampAccumulationVec3s, m_capacity);
	amp::resize(m_ampMatIdxs, m_capacity, m_count);
	amp::resize(m_ampGroupIdxs, m_capacity, m_count);
	amp::resize(m_ampDepths, m_capacity);
	amp::resize(m_ampColors, m_capacity, m_count);

	m_heatBuffer.resize(m_capacity);
	m_healthBuffer.resize(m_capacity);

	amp::resize(m_ampHeats, m_capacity, m_count);
	amp::resize(m_ampHealths, m_capacity, m_count);
	amp::resize(m_ampFlags, m_capacity, m_count);

	m_proxyBuffer.resize(m_capacity);
	amp::resize(m_ampProxies, m_capacity);
	amp::resize(m_localContactCnts, m_capacity);
	amp::resize(m_localContacts, m_capacity);
	amp::resize(m_localBodyContactCnts, m_capacity);
	amp::resize(m_localBodyContacts, m_capacity);

	//m_findContactCountBuf.resize(m_capacity);
	//m_findContactIdxABuf.resize(m_capacity);
	//m_findContactIdxBBuf.resize(m_capacity);
	m_findContactRightTagBuf.resize(m_capacity);
	m_findContactBottomLeftTagBuf.resize(m_capacity);
	m_findContactBottomRightTagBuf.resize(m_capacity);

	m_expireTimeBuf.resize(m_capacity); 
	m_idxByExpireTimeBuf.resize(m_capacity);
}

void b2ParticleSystem::ResizeGroupBuffers(int32 size)
{
	if (!AdjustCapacityToSize(m_groupCapacity, size, b2_minGroupBufferCapacity)) return;
	m_groupBuffer.resize(m_groupCapacity);
	amp::resize(m_ampGroups, m_groupCapacity, m_groupCount);

	m_groupExtent = ampExtent(m_groupCount);
}

void b2ParticleSystem::ResizeContactBuffers(int32 size)
{
	if (!AdjustCapacityToSize(m_contactCapacity, size, b2_minParticleBufferCapacity)) return;
	m_partContactBuf.resize(m_contactCapacity);
	amp::resize(m_ampContacts, m_contactCapacity);
	amp::resize(m_ampPairs, m_contactCapacity);
}
void b2ParticleSystem::ResizeBodyContactBuffers(int32 size)
{
	if (!AdjustCapacityToSize(m_bodyContactCapacity, size, b2_minParticleBufferCapacity)) return;
	m_bodyContactBuf.resize(m_bodyContactCapacity);
}

void b2ParticleSystem::ResizePairBuffers(int32 size)
{
	if (!AdjustCapacityToSize(m_pairCapacity, size, b2_minParticleBufferCapacity)) return;
	m_pairBuffer.resize(m_pairCapacity);
	amp::resize(m_ampPairs, m_pairCapacity);
}
void b2ParticleSystem::ResizeTriadBuffers(int32 size)
{
	if (!AdjustCapacityToSize(m_pairCapacity, size + 1, b2_minParticleBufferCapacity)) return;
	m_triadBuffer.resize(m_pairCapacity);
	amp::resize(m_ampTriads, m_pairCapacity);
	m_triadCapacity = m_pairCapacity;
}

int32 b2ParticleSystem::CreateParticleMaterial(const b2ParticleMaterialDef & def)
{
	if (m_partMatCount >= m_partMatCapacity)
		ResizePartMatBuffers(m_partMatCount * 2);

	// Try finding duplicat first
	for (int idx = 0; idx < m_partMatCount; idx++)
		if (def.matFlags		 == m_partMatFlagsBuf[idx] &&
			def.partFlags		 == m_partMatPartFlagsBuf[idx] &&
			def.mass			 == m_partMatMassBuf[idx] &&
			def.stability		 == m_partMatStabilityBuf[idx] &&
			def.heatConductivity == m_partMatHeatConductivityBuf[idx])
			return idx;

	int32 idx = m_partMatCount;
	m_partMatCount++;
	m_partMatFlagsBuf[idx] = def.matFlags;
	m_partMatPartFlagsBuf[idx] = def.partFlags;
	m_partMatMassBuf[idx] = def.mass;
	m_partMatInvMassBuf[idx] = 1 / def.mass;
	m_partMatStabilityBuf[idx] = def.stability;
	m_partMatInvStabilityBuf[idx] = 1 / def.stability;
	m_partMatHeatConductivityBuf[idx] = def.heatConductivity;
	m_world.m_allBodyMaterialFlags |= def.matFlags;

	amp::copy(m_partMatFlagsBuf[idx], m_ampMatFlags, idx);
	amp::copy(m_partMatPartFlagsBuf[idx], m_ampMatPartFlags, idx);
	amp::copy(m_partMatMassBuf[idx], m_ampMatMasses, idx);
	amp::copy(m_partMatInvMassBuf[idx], m_ampMatInvMasses, idx);
	amp::copy(m_partMatStabilityBuf[idx], m_ampMatStabilities, idx);
	amp::copy(m_partMatInvStabilityBuf[idx], m_ampMatInvStabilities, idx);
	amp::copy(m_partMatHeatConductivityBuf[idx], m_ampMatHeatConductivities, idx);
	return idx;
}
void b2ParticleSystem::PartMatChangeMats(int32 matIdx, float32 colderThan, int32 changeToColdMat,
									     float32 hotterThan, int32 changeToHotMat, 
										 float32 ignitionPoint, int32 burnToMat)
{
	m_partMatColderThanBuf[matIdx]		= colderThan;
	m_partMatChangeToColdMatBuf[matIdx] = changeToColdMat;
	m_partMatHotterThanBuf[matIdx]		= hotterThan;
	m_partMatChangeToHotMatBuf[matIdx]	= changeToHotMat;
	m_partMatIgnitionPointBuf[matIdx]	= ignitionPoint;
	m_partMatBurnToMatBuf[matIdx]		= burnToMat;
	m_partMatIgnitionPointBuf[matIdx]	= ignitionPoint;

	amp::copy(m_partMatFlagsBuf[matIdx], m_ampMatFlags, matIdx);
	amp::copy(m_partMatPartFlagsBuf[matIdx], m_ampMatPartFlags, matIdx);
	amp::copy(m_partMatMassBuf[matIdx], m_ampMatMasses, matIdx);
	amp::copy(m_partMatInvMassBuf[matIdx], m_ampMatInvMasses, matIdx);
	amp::copy(m_partMatStabilityBuf[matIdx], m_ampMatStabilities, matIdx);
	amp::copy(m_partMatInvStabilityBuf[matIdx], m_ampMatInvStabilities, matIdx);
	amp::copy(m_partMatHeatConductivityBuf[matIdx], m_ampMatHeatConductivities, matIdx);
}

/// Retrieve a handle to the particle at the specified index.
const b2ParticleHandle* b2ParticleSystem::GetParticleHandleFromIndex(
	const int32 index)
{
	b2Assert(index >= 0 && index < GetParticleCount() &&
			 index != b2_invalidIndex);
	RequestBuffer(m_handleIndexBuffer, hasHandleIndexBuffer);
	b2ParticleHandle* handle = m_handleIndexBuffer[index];
	if (handle)
	{
		return handle;
	}
	// Create a handle.
	handle = m_handleAllocator.Allocate();
	b2Assert(handle);
	handle->SetIndex(index);
	m_handleIndexBuffer[index] = handle;
	return handle;
}

void b2ParticleSystem::DestroyParticle(
	int32 index, bool callDestructionListener)
{
	uint32 flags = b2_zombieParticle;
	if (callDestructionListener)
		flags |= b2_destructionListenerParticle;
	SetParticleFlags(index, m_flagsBuffer[index] | flags);
}

void b2ParticleSystem::DestroyAllParticles()
{
	m_count = 0;
	m_allParticleFlags = 0;
	ResizeParticleBuffers(0);
	
	m_groupCount = 0;
	m_freeGroupIdxs.clear();
	m_zombieRanges.clear();
	ResizeGroupBuffers(0);

	ResizeContactBuffers(0);
}

void b2ParticleSystem::DestroyOldestParticle(
	const int32 index, const bool callDestructionListener)
{
	const int32 particleCount = GetParticleCount();
	b2Assert(index >= 0 && index < particleCount);
	// Make sure particle lifetime tracking is enabled.
	b2Assert(m_indexByExpirationTimeBuffer.data);
	// Destroy the oldest particle (preferring to destroy finite
	// lifetime particles first) to free a slot in the buffer.
	const int32 oldestFiniteLifetimeParticle =
		m_idxByExpireTimeBuf[particleCount - (index + 1)];
	const int32 oldestInfiniteLifetimeParticle =
		m_idxByExpireTimeBuf[index];
	DestroyParticle(
		m_expireTimeBuf[oldestFiniteLifetimeParticle] > 0.0f ?
			oldestFiniteLifetimeParticle : oldestInfiniteLifetimeParticle,
		callDestructionListener);
}

void b2ParticleSystem::DestroyParticlesInGroup(const int32 groupIdx)
{
	DestroyParticlesInGroup(m_groupBuffer[groupIdx]);
}
void b2ParticleSystem::DestroyParticlesInGroup(const b2ParticleGroup& group)
{
	b2Assert(!m_world.IsLocked());
	if (m_world.IsLocked()) return;

	for (int32 i = group.m_firstIndex; i < group.m_lastIndex; i++) {
		DestroyParticle(i, false);
	}
}

int32 b2ParticleSystem::DestroyParticlesInFixture(
	const Fixture& fixture, const b2Transform& xf,
	bool callDestructionListener)
{
	b2Assert(!m_world.IsLocked());
	if (m_world.IsLocked())
	{
		return 0;
	}

	const b2Shape& shape = m_world.GetShape(fixture);
	class DestroyParticlesInShapeCallback : public b2QueryCallback
	{
	public:
		DestroyParticlesInShapeCallback(
			b2ParticleSystem* system, const b2Shape& shape,
			const b2Transform& xf, bool callDestructionListener)
		{
			m_system = system;
			m_shape = &shape;
			m_xf = xf;
			m_callDestructionListener = callDestructionListener;
			m_destroyed = 0;
		}

		bool ReportFixture(int32 fixtureIdx)
		{
			B2_NOT_USED(fixtureIdx);
			return false;
		}

		bool ReportParticle(const b2ParticleSystem* particleSystem, int32 index)
		{
			if (particleSystem != m_system)
				return false;

			b2Assert(index >=0 && index < m_system->m_count);
			if (m_shape->TestPoint(m_xf, m_system->m_positionBuffer[index]))
			{
				m_system->DestroyParticle(index, m_callDestructionListener);
				m_destroyed++;
			}
			return true;
		}

		int32 Destroyed() { return m_destroyed; }

	private:
		b2ParticleSystem* m_system;
		const b2Shape* m_shape;
		b2Transform m_xf;
		bool m_callDestructionListener;
		int32 m_destroyed;
	} callback(this, shape, xf, callDestructionListener);
	b2AABB aabb;
	shape.ComputeAABB(aabb, xf, 0);
	m_world.QueryAABB(&callback, aabb);
	return callback.Destroyed();
}

int32 b2ParticleSystem::CreateParticlesForGroup(const b2ParticleGroupDef& groupDef,
	const b2Transform& xf, const vector<b2Vec3>& poss)
{
	uint32 writeIdx = GetWriteIdx(poss.size());
	for (uint32 i = 0, wi = writeIdx; i < poss.size(); i++, wi++)
	{
		m_partGroupIdxBuffer[wi] = groupDef.idx;
		m_flagsBuffer[wi] = groupDef.flags;
		if (!m_lastBodyContactStepBuffer.empty())
			m_lastBodyContactStepBuffer[wi] = 0;
		if (!m_bodyContactCountBuffer.empty())
			m_bodyContactCountBuffer[wi] = 0;
		if (!m_consecutiveContactStepsBuffer.empty())
			m_consecutiveContactStepsBuffer[wi] = 0;
		const b2Vec3& p = poss[i];
		m_positionBuffer[wi] = b2Vec3(b2Mul(xf, p), p.z);
		m_velocityBuffer[wi] = b2Vec3(groupDef.linearVelocity +
			b2Cross(groupDef.angularVelocity, p - groupDef.position), 0);
		m_heatBuffer[wi] = groupDef.heat;
		m_healthBuffer[wi] = groupDef.health;
		m_forceBuffer[wi].SetZero();
		m_partMatIdxBuffer[wi] = groupDef.matIdx;
		if (!m_staticPressureBuf.empty())
			m_staticPressureBuf[wi] = 0;

		m_depthBuffer[wi] = 0;
		m_colorBuffer[wi] = groupDef.color;
	}
	m_allParticleFlags |= groupDef.flags;
	return writeIdx;
}
int32 b2ParticleSystem::CreateParticlesForGroup(const b2ParticleGroupDef& groupDef,
	const b2Transform& xf, const vector<b2Vec3>& poss, const vector<int32>& cols)
{
	int32 writeIdx = GetWriteIdx(poss.size());
	for (int32 i = writeIdx; i < poss.size(); i++)
	{
		m_flagsBuffer[i] = groupDef.flags;
		if (!m_lastBodyContactStepBuffer.empty())
			m_lastBodyContactStepBuffer[i] = 0;
		if (!m_bodyContactCountBuffer.empty())
			m_bodyContactCountBuffer[i] = 0;
		if (!m_consecutiveContactStepsBuffer.empty())
			m_consecutiveContactStepsBuffer[i] = 0;
		const b2Vec3& p = poss[i];
		m_positionBuffer[i] = b2Vec3(b2Mul(xf, p), p.z);
		m_velocityBuffer[i] = b2Vec3(groupDef.linearVelocity +
			b2Cross(groupDef.angularVelocity, p - groupDef.position), 0);
		m_heatBuffer[i] = groupDef.heat;
		m_healthBuffer[i] = groupDef.health;
		m_forceBuffer[i].SetZero();
		m_partMatIdxBuffer[i] = groupDef.matIdx;
		if (!m_staticPressureBuf.empty())
			m_staticPressureBuf[i] = 0;

		m_depthBuffer[i] = 0;
		m_colorBuffer[i] = groupDef.color;
	}
	m_allParticleFlags |= groupDef.flags;
	return writeIdx;
}

pair<int32, int32> b2ParticleSystem::CreateParticlesStrokeShapeForGroup(
	const b2Shape& shape, const b2ParticleGroupDef& groupDef, const b2Transform& xf)
{
	//float32 stride = groupDef.stride ? groupDef.stride : GetParticleStride();
	//float32 positionOnEdge = 0;
	//int32 childCount = shape->GetChildCount();
	//for (int32 childIndex = 0; childIndex < childCount; childIndex++)
	//{
	//	b2EdgeShape edge;
	//	if (shape->GetType() == Shape::e_edge)
	//		edge = *(b2EdgeShape*) shape;
	//	else
	//		((b2ChainShape*) shape)->GetChildEdge(&edge, childIndex);
	//	b2Vec2 d = edge.m_vertex2 - edge.m_vertex1;
	//	float32 edgeLength = d.Length();
	//	while (positionOnEdge < edgeLength)
	//	{
	//		b2Vec3 p(edge.m_vertex1 + positionOnEdge / edgeLength * d, 0);
	//		CreateParticleForGroup(groupDef, xf, p);
	//		positionOnEdge += stride;
	//	}
	//	positionOnEdge -= edgeLength;
	//}
	//uint32 firstIdx = CreateParticlesForGroup(cnt, groupDef, xf, positions);
	//return pair<uint32, uint32>(firstIdx, firstIdx + cnt);
	return pair<uint32, uint32>(0, 0);
}

pair<int32, int32> b2ParticleSystem::CreateParticlesFillShapeForGroup(
	const b2Shape& shape,
	const b2ParticleGroupDef& groupDef, const b2Transform& xf)
{
	float32 stride = groupDef.stride ? groupDef.stride : GetParticleStride();
	b2Transform identity;
	identity.SetIdentity();
	b2AABB aabb;
	b2Assert(shape.GetChildCount() == 1);
	shape.ComputeAABB(aabb, identity, 0);
	float32 startY = floorf(aabb.lowerBound.y / stride) * stride;
	float32 startX = floorf(aabb.lowerBound.x / stride) * stride;
	vector<b2Vec3> positions;
	positions.reserve((int32)(((aabb.upperBound.y - startY) * (aabb.upperBound.x - startX)) / (stride * stride)));
	for (float32 y = startY; y < aabb.upperBound.y; y += stride)
		for (float32 x = startX; x < aabb.upperBound.x; x += stride)
			if (const b2Vec3 p(x, y, groupDef.position.z); shape.TestPoint(identity, p))
				positions.push_back(p);
	
	uint32 firstIdx = CreateParticlesForGroup(groupDef, xf, positions);
	return pair<uint32, uint32>(firstIdx, firstIdx + positions.size());
}

pair<int32, int32> b2ParticleSystem::CreateParticlesWithShapeForGroup(
	const b2Shape::Type shapeType, const int32 shapeIdx,
	const b2ParticleGroupDef& groupDef, const b2Transform& xf)
{
	const b2Shape& shape = m_world.GetShape(shapeType, shapeIdx);
	switch (shape.m_type) {
	case b2Shape::e_edge:
	case b2Shape::e_chain:
		return CreateParticlesStrokeShapeForGroup(shape, groupDef, xf);
		break;
	case b2Shape::e_polygon:
	case b2Shape::e_circle:
		return CreateParticlesFillShapeForGroup(shape, groupDef, xf);
		break;
	default:
		b2Assert(false);
		break;
	}
}

int32 b2ParticleSystem::CreateGroup(
	b2ParticleGroupDef& groupDef)
{
	if (m_world.IsLocked()) return b2_invalidIndex;

	// get group Index
	if (!m_freeGroupIdxs.empty())
	{
		groupDef.groupIdx = groupDef.idx = m_freeGroupIdxs.back();
		m_freeGroupIdxs.pop_back();
	}
	else
	{
		ResizeGroupBuffers(m_groupCount + 1);
		groupDef.groupIdx = groupDef.idx = m_groupCount++;
	}

	b2Transform transform;
	transform.Set(groupDef.position, groupDef.angle);
	pair<int32, int32> firstAndLastIdx;
	if (groupDef.shapeIdx != b2_invalidIndex)
	{
		firstAndLastIdx = CreateParticlesWithShapeForGroup(groupDef.shapeType, groupDef.shapeIdx, groupDef, transform);
	}
	if (groupDef.particleCount)
	{
		const vector<b2Vec3> poss(groupDef.positionData, groupDef.positionData + groupDef.particleCount);
		const vector<int32> cols(groupDef.colorData, groupDef.colorData + groupDef.particleCount);
		firstAndLastIdx.first = CreateParticlesForGroup(groupDef, transform, poss, cols);
		firstAndLastIdx.second = firstAndLastIdx.first + groupDef.particleCount;
	}

	b2ParticleGroup& group = m_groupBuffer[groupDef.idx];
	group.m_firstIndex = firstAndLastIdx.first;
	group.m_lastIndex = firstAndLastIdx.second;
	group.m_strength = groupDef.strength;
	group.m_collisionGroup = groupDef.collisionGroup;
	group.m_matIdx = groupDef.matIdx;
	group.m_transform = transform;
	group.m_timestamp = groupDef.timestamp;
	SetGroupFlags(group, groupDef.groupFlags);

	if (m_accelerate)
	{
		CopyParticleRangeToGpu(group.m_firstIndex, group.m_lastIndex);
		amp::copy(group, m_ampGroups, groupDef.idx);
	}

	// Create pairs and triads between particles in the group->
	// ConnectionFilter filter;
	// UpdateContacts(true);
	// UpdatePairsAndTriads(firstIndex, lastIndex, filter);

	return groupDef.groupIdx;
}

void b2ParticleSystem::CopyParticleRangeToGpu(const uint32 first, const uint32 last)
{
	const int32 size = last - first;
	if (size <= 0) return;
	amp::copy(m_flagsBuffer, m_ampFlags, first, size);
	amp::copy(m_positionBuffer, m_ampPositions, first, size);
	amp::copy(m_velocityBuffer, m_ampVelocities, first, size);
	amp::copy(m_weightBuffer, m_ampWeights, first, size);
	amp::copy(m_heatBuffer, m_ampHeats, first, size);
	amp::copy(m_healthBuffer, m_ampHealths, first, size);
	amp::copy(m_forceBuffer, m_ampForces, first, size);
	amp::copy(m_partMatIdxBuffer, m_ampMatIdxs, first, size);
	if (!m_staticPressureBuf.empty())
		amp::copy(m_staticPressureBuf, m_ampStaticPressures, first, size);
	amp::copy(m_depthBuffer, m_ampDepths, first, size);
	amp::copy(m_colorBuffer, m_ampColors, first, size);
	amp::copy(m_proxyBuffer, m_ampProxies, first, size);
	amp::copy(m_partGroupIdxBuffer, m_ampGroupIdxs, first, size);
}

void b2ParticleSystem::JoinParticleGroups(int32 groupAIdx,
										  int32 groupBIdx)
{
	b2Assert(!m_world.IsLocked());
	if (m_world.IsLocked())
	{
		return;
	}
	b2Assert(groupAIdx != groupBIdx);

	b2ParticleGroup& groupA = m_groupBuffer[groupAIdx];
	b2ParticleGroup& groupB = m_groupBuffer[groupBIdx];

	RotateBuffer(groupB.m_firstIndex, groupB.m_lastIndex, m_count);
	b2Assert(groupB.m_lastIndex == m_count);
	RotateBuffer(groupA.m_firstIndex, groupA.m_lastIndex,
		groupB.m_firstIndex);
	b2Assert(groupA.m_lastIndex == groupB.m_firstIndex);

	// Create pairs and triads connecting groupA and groupB.
	class JoinParticleGroupsFilter : public ConnectionFilter
	{
		bool ShouldCreatePair(int32 a, int32 b) const
		{
			return
				(a < m_threshold && m_threshold <= b) ||
				(b < m_threshold && m_threshold <= a);
		}
		bool ShouldCreateTriad(int32 a, int32 b, int32 c) const
		{
			return
				(a < m_threshold || b < m_threshold || c < m_threshold) &&
				(m_threshold <= a || m_threshold <= b || m_threshold <= c);
		}
		int32 m_threshold;
	public:
		JoinParticleGroupsFilter(int32 threshold)
		{
			m_threshold = threshold;
		}
	} filter(groupB.m_firstIndex);
	UpdateContacts(true);
	UpdatePairsAndTriads(groupA.m_firstIndex, groupB.m_lastIndex, filter);

	for (int32 i = groupB.m_firstIndex; i < groupB.m_lastIndex; i++)
	{
		m_partGroupIdxBuffer[i] = groupAIdx;
	}
	uint32 groupFlags = groupA.m_groupFlags | groupB.m_groupFlags;
	SetGroupFlags(groupA, groupFlags);
	groupA.m_lastIndex = groupB.m_lastIndex;
	groupB.m_firstIndex = groupB.m_lastIndex;
	DestroyGroup(groupBIdx);
}

void b2ParticleSystem::SplitParticleGroup(b2ParticleGroup& group)
{
	UpdateContacts(true);
	int32 particleCount = group.GetParticleCount();
	// We create several linked lists. Each list represents a set of connected
	// particles.
	ParticleListNode* nodeBuffer =
		(ParticleListNode*) m_world.m_stackAllocator.Allocate(
									sizeof(ParticleListNode) * particleCount);
	InitializeParticleLists(group, nodeBuffer);
	MergeParticleListsInContact(group, nodeBuffer);
	ParticleListNode* survivingList =
									FindLongestParticleList(group, nodeBuffer);
	MergeZombieParticleListNodes(group, nodeBuffer, survivingList);
	CreateParticleGroupsFromParticleList(group, nodeBuffer, survivingList);
	UpdatePairsAndTriadsWithParticleList(group, nodeBuffer);
	m_world.m_stackAllocator.Free(nodeBuffer);
}

void b2ParticleSystem::InitializeParticleLists(
	const b2ParticleGroup& group, ParticleListNode* nodeBuffer)
{
	int32 bufferIndex = group.GetBufferIndex();
	int32 particleCount = group.GetParticleCount();
	for (int32 i = 0; i < particleCount; i++)
	{
		ParticleListNode* node = &nodeBuffer[i];
		node->list = node;
		node->next = NULL;
		node->count = 1;
		node->index = i + bufferIndex;
	}
}

void b2ParticleSystem::MergeParticleListsInContact(
	const b2ParticleGroup& group, ParticleListNode* nodeBuffer) const
{
	int32 bufferIndex = group.GetBufferIndex();
	for (int32 k = 0; k < m_contactCount; k++)
	{
		const b2ParticleContact& contact = m_partContactBuf[k];
		int32 a = contact.idxA;
		int32 b = contact.idxB;
		if (!group.ContainsParticle(a) || !group.ContainsParticle(b)) {
			continue;
		}
		ParticleListNode* listA = nodeBuffer[a - bufferIndex].list;
		ParticleListNode* listB = nodeBuffer[b - bufferIndex].list;
		if (listA == listB) {
			continue;
		}
		// To minimize the cost of insertion, make sure listA is longer than
		// listB.
		if (listA->count < listB->count)
		{
			b2Swap(listA, listB);
		}
		b2Assert(listA->count >= listB->count);
		MergeParticleLists(listA, listB);
	}
}

void b2ParticleSystem::MergeParticleLists(
	ParticleListNode* listA, ParticleListNode* listB)
{
	// Insert listB between index 0 and 1 of listA
	// Example:
	//     listA => a1 => a2 => a3 => NULL
	//     listB => b1 => b2 => NULL
	// to
	//     listA => listB => b1 => b2 => a1 => a2 => a3 => NULL
	b2Assert(listA != listB);
	for (ParticleListNode* b = listB;;)
	{
		b->list = listA;
		ParticleListNode* nextB = b->next;
		if (nextB)
		{
			b = nextB;
		}
		else
		{
			b->next = listA->next;
			break;
		}
	}
	listA->next = listB;
	listA->count += listB->count;
	listB->count = 0;
}

b2ParticleSystem::ParticleListNode* b2ParticleSystem::FindLongestParticleList(
	const b2ParticleGroup& group, ParticleListNode* nodeBuffer)
{
	int32 particleCount = group.GetParticleCount();
	ParticleListNode* result = nodeBuffer;
	for (int32 i = 0; i < particleCount; i++)
	{
		ParticleListNode* node = &nodeBuffer[i];
		if (result->count < node->count)
		{
			result = node;
		}
	}
	return result;
}

void b2ParticleSystem::MergeZombieParticleListNodes(
	const b2ParticleGroup& group, ParticleListNode* nodeBuffer,
	ParticleListNode* survivingList) const
{
	int32 particleCount = group.GetParticleCount();
	for (int32 i = 0; i < particleCount; i++)
	{
		ParticleListNode* node = &nodeBuffer[i];
		if (node != survivingList &&
			(m_flagsBuffer[node->index] & b2_zombieParticle))
		{
			MergeParticleListAndNode(survivingList, node);
		}
	}
}

void b2ParticleSystem::MergeParticleListAndNode(
	ParticleListNode* list, ParticleListNode* node)
{
	// Insert node between index 0 and 1 of list
	// Example:
	//     list => a1 => a2 => a3 => NULL
	//     node => NULL
	// to
	//     list => node => a1 => a2 => a3 => NULL
	b2Assert(node != list);
	b2Assert(node->list == node);
	b2Assert(node->count == 1);
	node->list = list;
	node->next = list->next;
	list->next = node;
	list->count++;
	node->count = 0;
}

void b2ParticleSystem::CreateParticleGroupsFromParticleList(
	const b2ParticleGroup& group, ParticleListNode* nodeBuffer,
	const ParticleListNode* survivingList)
{
	int32 particleCount = group.GetParticleCount();
	b2ParticleGroupDef def;
	def.groupFlags = group.GetGroupFlags();
	for (int32 i = 0; i < particleCount; i++)
	{
		ParticleListNode* list = &nodeBuffer[i];
		if (!list->count || list == survivingList)
		{
			continue;
		}
		b2Assert(list->list == list);
		int32 newGroupIdx = CreateGroup(def);
		for (ParticleListNode* node = list; node; node = node->next)
		{
			int32 oldIndex = node->index;
			uint32& flags = m_flagsBuffer[oldIndex];
			b2Assert(!(flags & b2_zombieParticle));
			int32 newIndex = CloneParticle(oldIndex, newGroupIdx);
			flags |= b2_zombieParticle;
			node->index = newIndex;
		}
	}
}

void b2ParticleSystem::UpdatePairsAndTriadsWithParticleList(
	const b2ParticleGroup& group, const ParticleListNode* nodeBuffer)
{
	int32 bufferIndex = group.GetBufferIndex();
	// Update indices in pairs and triads. If an index belongs to the group,
	// replace it with the corresponding value in nodeBuffer.
	// Note that nodeBuffer is allocated only for the group and the index should
	// be shifted by bufferIndex.
	for (int32 k = 0; k < m_pairCount; k++)
	{
		b2ParticlePair& pair = m_pairBuffer[k];
		int32 a = pair.indexA;
		int32 b = pair.indexB;
		if (group.ContainsParticle(a))
			pair.indexA = nodeBuffer[a - bufferIndex].index;		
		if (group.ContainsParticle(b))
			pair.indexB = nodeBuffer[b - bufferIndex].index;		
	}
	for (int32 k = 0; k < m_triadCount; k++)
	{
		b2ParticleTriad& triad = m_triadBuffer[k];
		int32 a = triad.indexA;
		int32 b = triad.indexB;
		int32 c = triad.indexC;
		if (group.ContainsParticle(a))		
			triad.indexA = nodeBuffer[a - bufferIndex].index;		
		if (group.ContainsParticle(b))		
			triad.indexB = nodeBuffer[b - bufferIndex].index;		
		if (group.ContainsParticle(c))		
			triad.indexC = nodeBuffer[c - bufferIndex].index;		
	}
}

int32 b2ParticleSystem::CloneParticle(int32 oldIndex, int32 groupIdx)
{
	b2ParticleDef def;
	def.flags	  = m_flagsBuffer[oldIndex];
	def.position  = m_positionBuffer[oldIndex];
	def.velocity  = m_velocityBuffer[oldIndex];
	def.heat	  = m_heatBuffer[oldIndex];
	def.health	  = m_healthBuffer[oldIndex];
	if (m_colorBuffer.data())
		def.color = m_colorBuffer[oldIndex];
	def.groupIdx = groupIdx;
	def.matIdx = m_partMatIdxBuffer[oldIndex];
	//int32 newIndex = CreateParticle(def);
	//if (!m_handleIndexBuffer.empty())
	//{
	//	b2ParticleHandle* handle = m_handleIndexBuffer[oldIndex];
	//	if (handle) handle->SetIndex(newIndex);
	//	m_handleIndexBuffer[newIndex] = handle;
	//	m_handleIndexBuffer[oldIndex] = NULL;
	//}
	//if (!m_lastBodyContactStepBuffer.empty())
	//{
	//	m_lastBodyContactStepBuffer[newIndex] =
	//		m_lastBodyContactStepBuffer[oldIndex];
	//}
	//if (!m_bodyContactCountBuffer.empty())
	//{
	//	m_bodyContactCountBuffer[newIndex] =
	//		m_bodyContactCountBuffer[oldIndex];
	//}
	//if (!m_consecutiveContactStepsBuffer.empty())
	//{
	//	m_consecutiveContactStepsBuffer[newIndex] =
	//		m_consecutiveContactStepsBuffer[oldIndex];
	//}
	//if (m_hasForce)
	//{
	//	m_forceBuffer[newIndex] = m_forceBuffer[oldIndex];
	//}
	//if (!m_staticPressureBuf.empty())
	//{
	//	m_staticPressureBuf[newIndex] = m_staticPressureBuf[oldIndex];
	//}
	//m_depthBuffer[newIndex] = m_depthBuffer[oldIndex];
	//if (!m_expireTimeBuf.empty())
	//{
	//	m_expireTimeBuf[newIndex] = m_expireTimeBuf[oldIndex];
	//}
	//return newIndex;
	return 0;
}

void b2ParticleSystem::UpdatePairsAndTriadsWithReactiveParticles()
{
	if (!(m_allParticleFlags & b2_reactiveParticle)) return;

	if (m_accelerate)
	{
		AmpUpdatePairsAndTriads(0, m_count);

		auto& flags = m_ampFlags;
		const uint32 remReactiveFlag = ~b2_reactiveParticle;
		AmpForEachParticle([=, &flags](const int32 i) restrict(amp)
		{
			flags[i] &= remReactiveFlag;
		});
		m_ampCopyFutTriads.set(amp::copyAsync(m_triadBuffer, m_ampTriads, m_triadCount));
	}
	else
	{
		class ReactiveFilter : public ConnectionFilter
		{
			bool IsNecessary(int32 index) const
			{
				return (m_flagsBuffer[index] & b2_reactiveParticle) != 0;
			}
			const uint32* m_flagsBuffer;
		public:
			ReactiveFilter(uint32* flagsBuffer)
			{
				m_flagsBuffer = flagsBuffer;
			}
		} filter(m_flagsBuffer.data());
		UpdatePairsAndTriads(0, m_count, filter);

		for (int32 i = 0; i < m_count; i++)
		{
			m_flagsBuffer[i] &= ~b2_reactiveParticle;
		}
	}
	m_allParticleFlags &= ~b2_reactiveParticle;
}

static bool ParticleCanBeConnected(uint32 flags, b2ParticleGroup& group, int32 groupIdx)
{
	return
		(flags & (b2_wallParticle | b2_springParticle | b2_elasticParticle)) ||
		(groupIdx != b2_invalidIndex && group.GetGroupFlags() & b2_rigidParticleGroup);
}

void b2ParticleSystem::UpdatePairsAndTriads(
	int32 firstIndex, int32 lastIndex, const ConnectionFilter& filter)
{
	// Create pairs or triads.
	// All particles in each pair/triad should satisfy the following:
	// * firstIndex <= index < lastIndex
	// * don't have b2_zombieParticle
	// * ParticleCanBeConnected returns true
	// * ShouldCreatePair/ShouldCreateTriad returns true
	// Any particles in each pair/triad should satisfy the following:
	// * filter.IsNeeded returns true
	// * have one of k_pairFlags/k_triadsFlags
	b2Assert(firstIndex <= lastIndex);
	uint32 particleFlags = 0;
	for (int32 i = firstIndex; i < lastIndex; i++)
	{
		particleFlags |= m_flagsBuffer[i];
	}
	if (particleFlags & k_pairFlags)
	{
		for (int32 k = 0; k < m_contactCount; k++)
		{
			const b2ParticleContact& contact = m_partContactBuf[k];
			int32 a = contact.idxA;
			int32 b = contact.idxB;
			uint32 af = m_flagsBuffer[a];
			uint32 bf = m_flagsBuffer[b];
			int32 groupAIdx = m_partGroupIdxBuffer[a];
			int32 groupBIdx = m_partGroupIdxBuffer[b];
			b2ParticleGroup& groupA = m_groupBuffer[groupAIdx];
			b2ParticleGroup& groupB = m_groupBuffer[groupBIdx];
			if (a >= firstIndex && a < lastIndex &&
				b >= firstIndex && b < lastIndex &&
				!((af | bf) & b2_zombieParticle) &&
				((af | bf) & k_pairFlags) &&
				(filter.IsNecessary(a) || filter.IsNecessary(b)) &&
				ParticleCanBeConnected(af, groupA, groupAIdx) &&
				ParticleCanBeConnected(bf, groupB, groupBIdx) &&
				filter.ShouldCreatePair(a, b))
			{
				ResizePairBuffers(m_pairCount);
				b2ParticlePair& pair = m_pairBuffer[m_pairCount];
				pair.indexA = a;
				pair.indexB = b;
				pair.flags = contact.flags;
				pair.strength = b2Min(
					groupAIdx != b2_invalidIndex ? groupA.m_strength : 1,
					groupBIdx != b2_invalidIndex ? groupB.m_strength : 1);
				pair.distance = b2Distance(m_positionBuffer[a], m_positionBuffer[b]);
			}
		}
		concurrency::parallel_sort(
			m_pairBuffer.begin(), m_pairBuffer.end(), ComparePairIndices);
		std::unique(m_pairBuffer.begin(), m_pairBuffer.end(), MatchPairIndices);
		m_pairCount = m_pairBuffer.size();
		m_pairTilableExtent = amp::getTilableExtent(m_pairCount);
	}
	if (particleFlags & k_triadFlags)
	{
		b2VoronoiDiagram diagram(
			&m_world.m_stackAllocator, lastIndex - firstIndex);
		for (int32 i = firstIndex; i < lastIndex; i++)
		{
			uint32 flags = m_flagsBuffer[i];
			int32 groupIdx = m_partGroupIdxBuffer[i];
			if (!(flags & b2_zombieParticle) &&
				ParticleCanBeConnected(flags, m_groupBuffer[groupIdx], groupIdx))
			{
				diagram.AddGenerator(b2Vec2(m_positionBuffer[i]), i, filter.IsNecessary(i));
			}
		}
		float32 stride = GetParticleStride();
		diagram.Generate(stride / 2, stride * 2);
		class UpdateTriadsCallback : public b2VoronoiDiagram::NodeCallback
		{
			void operator()(int32 a, int32 b, int32 c)
			{
				uint32 af = m_system->m_flagsBuffer[a];
				uint32 bf = m_system->m_flagsBuffer[b];
				uint32 cf = m_system->m_flagsBuffer[c];
				if (((af | bf | cf) & k_triadFlags) &&
					m_filter->ShouldCreateTriad(a, b, c))
				{
					const b2Vec3& pa = m_system->m_positionBuffer[a];
					const b2Vec3& pb = m_system->m_positionBuffer[b];
					const b2Vec3& pc = m_system->m_positionBuffer[c];
					b2Vec2 dab = pa - pb;
					b2Vec2 dbc = pb - pc;
					b2Vec2 dca = pc - pa;
					float32 maxDistanceSquared = b2_maxTriadDistanceSquared *
						m_system->m_squaredDiameter;
					if (b2Dot(dab, dab) > maxDistanceSquared ||
						b2Dot(dbc, dbc) > maxDistanceSquared ||
						b2Dot(dca, dca) > maxDistanceSquared)
						return;
					int32 groupAIdx = m_system->m_partGroupIdxBuffer[a];
					int32 groupBIdx = m_system->m_partGroupIdxBuffer[b];
					int32 groupCIdx = m_system->m_partGroupIdxBuffer[c];
					b2ParticleGroup& groupA = m_system->m_groupBuffer[groupAIdx];
					b2ParticleGroup& groupB = m_system->m_groupBuffer[groupBIdx];
					b2ParticleGroup& groupC = m_system->m_groupBuffer[groupCIdx];
					int32& triadCount = m_system->m_triadCount;
					m_system->ResizeTriadBuffers(triadCount);
					b2ParticleTriad& triad = m_system->m_triadBuffer[triadCount];
					triadCount++;
					triad.indexA = a;
					triad.indexB = b;
					triad.indexC = c;
					triad.flags = af | bf | cf;
					triad.strength = b2Min(b2Min(
						groupAIdx != b2_invalidIndex ? groupA.m_strength : 1,
						groupBIdx != b2_invalidIndex ? groupB.m_strength : 1),
						groupCIdx != b2_invalidIndex ? groupC.m_strength : 1);
					b2Vec2 midPoint = (float32)1 / 3 * (pa + pb + pc);
					triad.pa = midPoint - pa;
					triad.pb = midPoint - pb;
					triad.pc = midPoint - pc;
					triad.ka = -b2Dot(dca, dab);
					triad.kb = -b2Dot(dab, dbc);
					triad.kc = -b2Dot(dbc, dca);
					triad.s = b2Cross2D(pa, pb) + b2Cross2D(pb, pc) + b2Cross2D(pc, pa);
				}
			}
			b2ParticleSystem* m_system;
			const ConnectionFilter* m_filter;
		public:
			UpdateTriadsCallback(
				b2ParticleSystem* system, const ConnectionFilter* filter)
			{
				m_system = system;
				m_filter = filter;
			}
		} callback(this, &filter);
		diagram.GetNodes(callback);


		concurrency::parallel_sort(
			m_triadBuffer.begin(), m_triadBuffer.end(), CompareTriadIndices);
		std::unique(m_triadBuffer.begin(), m_triadBuffer.end(), MatchTriadIndices);
		m_triadCount = m_triadBuffer.size();
		m_triadTilableExtent = amp::getTilableExtent(m_triadCount);
	}
}
void b2ParticleSystem::AmpUpdatePairsAndTriads(
	int32 firstIndex, int32 lastIndex)
{
	class ReactiveFilter : public ConnectionFilter
	{
		bool IsNecessary(int32 index) const
		{
			return (m_flagsBuffer[index] & b2_reactiveParticle) != 0;
		}
		const uint32* m_flagsBuffer;
	public:
		ReactiveFilter(uint32* flagsBuffer)
		{
			m_flagsBuffer = flagsBuffer;
		}
	} filter(m_flagsBuffer.data());

	// Create pairs or triads.
	// All particles in each pair/triad should satisfy the following:
	// * firstIndex <= index < lastIndex
	// * don't have b2_zombieParticle
	// * ParticleCanBeConnected returns true
	// * ShouldCreatePair/ShouldCreateTriad returns true
	// Any particles in each pair/triad should satisfy the following:
	// * filter.IsNeeded returns true
	// * have one of k_pairFlags/k_triadsFlags
	b2Assert(firstIndex <= lastIndex);

	if (m_allParticleFlags & k_pairFlags)
	{
		const uint32 invalidIdx = b2_invalidIndex;
		const uint32 wallSpringElasticFlag = b2_wallParticle | b2_springParticle | b2_elasticParticle;
		const uint32 rigidFlag = b2_rigidParticleGroup;
		const uint32 groupInternalMask = b2_particleGroupInternalMask;
		auto particleCanBeConnected = [=](uint32 flags, const b2ParticleGroup& group, int32 groupIdx) restrict(amp) -> bool
		{
			return (flags & wallSpringElasticFlag) ||
				(groupIdx != invalidIdx && (group.m_groupFlags & ~groupInternalMask) & rigidFlag);
		};
		auto& contacts = m_ampContacts;
		auto& flags = m_ampFlags;
		auto& groupIdxs = m_ampGroupIdxs;
		auto& groups = m_ampGroups;
		auto& positions = m_ampPositions;
		auto& oldPairs = m_ampPairs;
		const uint32 reactiveFlag = b2_reactiveParticle;
		const uint32 zombieFlag = b2_zombieParticle;
		const uint32 pairFlag = k_pairFlags;
		ampArray<int32> cnts(contacts.extent, m_gpuAccelView);
		amp::fill(cnts, 0);
		amp::forEach(m_contactCount, [=, &contacts, &oldPairs, &cnts, &flags, &groupIdxs,
			&groups, &positions](const int32 i) restrict(amp)
		{
			const b2ParticleContact& contact = contacts[i];
			const int32 a = contact.idxA;
			const int32 b = contact.idxB;
			const uint32 f = contact.flags;
			const int32 groupAIdx = groupIdxs[a];
			const int32 groupBIdx = groupIdxs[b];
			const b2ParticleGroup& groupA = groups[groupAIdx];
			const b2ParticleGroup& groupB = groups[groupBIdx];
			if (!(f & zombieFlag) &&
				(f & pairFlag) &&
				(f & reactiveFlag) &&
				particleCanBeConnected(flags[a], groupA, groupAIdx) &&
				particleCanBeConnected(flags[b], groupB, groupBIdx))
			{
				cnts[i] = 1;
				b2ParticlePair& pair = oldPairs[i];
				pair.indexA = a;
				pair.indexB = b;
				pair.flags = contact.flags;
				pair.strength = b2Min(
					groupAIdx != b2_invalidIndex ? groupA.m_strength : 1,
					groupBIdx != b2_invalidIndex ? groupB.m_strength : 1);
				pair.distance = b2Distance(positions[a], positions[b]);
			}
		});
		ampArray<b2ParticlePair> newPairs(oldPairs.extent, m_gpuAccelView);
		m_pairCount = amp::reduce(cnts, m_contactCount,
			[=, &newPairs, &oldPairs](const int32 i, const int32 wi) restrict(amp)
		{
			newPairs[wi] = oldPairs[i];
		});
		oldPairs = newPairs;
	}
	if (m_allParticleFlags & k_triadFlags)
	{
		b2VoronoiDiagram diagram(
			&m_world.m_stackAllocator, lastIndex - firstIndex);
		for (int32 i = firstIndex; i < lastIndex; i++)
		{
			uint32 flags = m_flagsBuffer[i];
			int32 groupIdx = m_partGroupIdxBuffer[i];
			if (!(flags & b2_zombieParticle) &&
				ParticleCanBeConnected(flags, m_groupBuffer[groupIdx], groupIdx))
			{
				diagram.AddGenerator(b2Vec2(m_positionBuffer[i]), i, m_flagsBuffer[i] & b2_reactiveParticle);
			}
		}
		float32 stride = GetParticleStride();
		diagram.Generate(stride / 2, stride * 2);
		class UpdateTriadsCallback : public b2VoronoiDiagram::NodeCallback
		{
			void operator()(int32 a, int32 b, int32 c)
			{
				uint32 af = m_system->m_flagsBuffer[a];
				uint32 bf = m_system->m_flagsBuffer[b];
				uint32 cf = m_system->m_flagsBuffer[c];
				if (((af | bf | cf) & k_triadFlags) &&
					m_filter->ShouldCreateTriad(a, b, c))
				{
					const b2Vec3& pa = m_system->m_positionBuffer[a];
					const b2Vec3& pb = m_system->m_positionBuffer[b];
					const b2Vec3& pc = m_system->m_positionBuffer[c];
					b2Vec2 dab = pa - pb;
					b2Vec2 dbc = pb - pc;
					b2Vec2 dca = pc - pa;
					float32 maxDistanceSquared = b2_maxTriadDistanceSquared *
						m_system->m_squaredDiameter;
					if (b2Dot(dab, dab) > maxDistanceSquared ||
						b2Dot(dbc, dbc) > maxDistanceSquared ||
						b2Dot(dca, dca) > maxDistanceSquared)
						return;
					int32 groupAIdx = m_system->m_partGroupIdxBuffer[a];
					int32 groupBIdx = m_system->m_partGroupIdxBuffer[b];
					int32 groupCIdx = m_system->m_partGroupIdxBuffer[c];
					b2ParticleGroup & groupA = m_system->m_groupBuffer[groupAIdx];
					b2ParticleGroup & groupB = m_system->m_groupBuffer[groupBIdx];
					b2ParticleGroup & groupC = m_system->m_groupBuffer[groupCIdx];
					int32 & triadCount = m_system->m_triadCount;
					m_system->ResizeTriadBuffers(triadCount);
					b2ParticleTriad & triad = m_system->m_triadBuffer[triadCount];
					triadCount++;
					triad.indexA = a;
					triad.indexB = b;
					triad.indexC = c;
					triad.flags = af | bf | cf;
					triad.strength = b2Min(b2Min(
						groupAIdx != b2_invalidIndex ? groupA.m_strength : 1,
						groupBIdx != b2_invalidIndex ? groupB.m_strength : 1),
						groupCIdx != b2_invalidIndex ? groupC.m_strength : 1);
					b2Vec2 midPoint = (float32)1 / 3 * (pa + pb + pc);
					triad.pa = midPoint - pa;
					triad.pb = midPoint - pb;
					triad.pc = midPoint - pc;
					triad.ka = -b2Dot(dca, dab);
					triad.kb = -b2Dot(dab, dbc);
					triad.kc = -b2Dot(dbc, dca);
					triad.s = b2Cross2D(pa, pb) + b2Cross2D(pb, pc) + b2Cross2D(pc, pa);
				}
			}
			b2ParticleSystem * m_system;
			const ConnectionFilter * m_filter;
		public:
			UpdateTriadsCallback(
				b2ParticleSystem * system, const ConnectionFilter * filter)
			{
				m_system = system;
				m_filter = filter;
			}
		} callback(this, &filter);
		diagram.GetNodes(callback);


		concurrency::parallel_sort(
			m_triadBuffer.begin(), m_triadBuffer.end(), CompareTriadIndices);
		std::unique(m_triadBuffer.begin(), m_triadBuffer.end(), MatchTriadIndices);
		m_triadCount = m_triadBuffer.size();
	}
}

bool b2ParticleSystem::ComparePairIndices(
	const b2ParticlePair& a, const b2ParticlePair& b)
{
	int32 diffA = a.indexA - b.indexA;
	if (diffA != 0) return diffA < 0;
	return a.indexB < b.indexB;
}

bool b2ParticleSystem::MatchPairIndices(
	const b2ParticlePair& a, const b2ParticlePair& b)
{
	return a.indexA == b.indexA && a.indexB == b.indexB;
}

bool b2ParticleSystem::CompareTriadIndices(
	const b2ParticleTriad& a, const b2ParticleTriad& b)
{
	int32 diffA = a.indexA - b.indexA;
	if (diffA != 0) return diffA < 0;
	int32 diffB = a.indexB - b.indexB;
	if (diffB != 0) return diffB < 0;
	return a.indexC < b.indexC;
}

bool b2ParticleSystem::MatchTriadIndices(
	const b2ParticleTriad& a, const b2ParticleTriad& b)
{
	return a.indexA == b.indexA && a.indexB == b.indexB && a.indexC == b.indexC;
}

// Only called from SolveZombie() or JoinParticleGroups().
void b2ParticleSystem::DestroyGroup(int32 groupIdx, int32 timestamp, bool destroyParticles)
{
	b2ParticleGroup& group = m_groupBuffer[groupIdx];
	if (timestamp != b2_invalidIndex && timestamp && timestamp != group.m_timestamp)
		return;

	AddZombieRange(group.m_firstIndex, group.m_lastIndex);
	ResizeParticleBuffers(m_count);
	if (destroyParticles)
	{
		if (m_accelerate)
		{
			auto& flags = m_ampFlags;
			const uint32 zombieFlags = b2_zombieParticle;
			amp::forEach(group.m_firstIndex, group.m_lastIndex, [=, &flags](const int32 i) restrict(amp)
			{
				flags[i] = zombieFlags;
			});
		}
		else
		{
			for (int i = group.m_firstIndex; i < group.m_lastIndex; i++)
				m_flagsBuffer[i] = b2_zombieParticle;
		}
	}
	//if (m_world.m_destructionListener)
	//	m_world.m_destructionListener->SayGoodbye(group);

	//SetGroupFlags(group, 0);
	m_needsUpdateAllGroupFlags = true;
	// for (int32 i = group.m_firstIndex; i < group.m_lastIndex; i++)
	// 	m_partGroupIdxBuffer[i] = b2_invalidIndex;
	group.m_firstIndex = b2_invalidIndex;

	if (groupIdx + 1 == m_groupCount)
		m_groupCount--;
	else
		m_freeGroupIdxs.push_back(groupIdx);
}

void b2ParticleSystem::ComputeWeight()
{
	// calculates the sum of contact-weights for each particle
	// that means dimensionless density
	//memset(m_weightBuffer.data(), 0, sizeof(*(m_weightBuffer.data())) * m_count);
	//m_weightBuffer.resize(m_capacity);

	if (m_accelerate)
	{
		m_futureComputeWeight = std::async(launch::async, [=]()
		{
			auto& weights = m_ampWeights;
			amp::fill(weights, 0.f, m_count);
			AmpForEachBodyContact([=, &weights](const b2PartBodyContact& contact) restrict(amp)
			{
				int32 a = contact.partIdx;
				float32 w = contact.weight;
				amp::atomicAdd(weights[a], w);
			});
			AmpForEachContactShuffled([=, &weights](const b2ParticleContact& contact) restrict(amp)
			{
				const int32 a = contact.idxA;
				const int32 b = contact.idxB;
				const float32 w = contact.weight;
				amp::atomicAdd(weights[a], w);
				amp::atomicAdd(weights[b], w);
			});
			m_ampCopyFutWeights.set(amp::copyAsync(m_ampWeights, m_weightBuffer, m_count));
		});
	}
	else
	{
		std::fill(m_weightBuffer.begin(), m_weightBuffer.begin() + m_count, 0);
		for (int32 k = 0; k < m_bodyContactCount; k++)
		{
			const b2PartBodyContact& contact = m_bodyContactBuf[k];
			int32 a = contact.partIdx;
			float32 w = contact.weight;
			m_weightBuffer[a] += w;
		}
		for (int32 k = 0; k < m_contactCount; k++)
		{
			const b2ParticleContact& contact = m_partContactBuf[k];
			int32 a = contact.idxA;
			int32 b = contact.idxB;
			float32 w = contact.weight;
			m_weightBuffer[a] += w;
			m_weightBuffer[b] += w;
		}
	}
}
void b2ParticleSystem::WaitForComputeWeight()
{
	if (m_accelerate)
		m_futureComputeWeight.wait();
}
void b2ParticleSystem::WaitForUpdateBodyContacts()
{
	if (m_accelerate)
		m_futureUpdateBodyContacts.wait();
}

void b2ParticleSystem::ComputeDepth()
{
	if (!(m_allGroupFlags & b2_particleGroupNeedsUpdateDepth)) return;
	if (!m_contactCount) return;

	if (m_accelerate)
	{
		const float32 maxFloat = b2_maxFloat;
		const float32 particleDiameter = m_particleDiameter;
		ampArray<b2ParticleContact> contactGroups(m_contactCount, m_gpuAccelView);
		auto& groupIdxs = m_ampGroupIdxs;
		auto& contacts = m_ampContacts;
		auto& groups = m_ampGroups;
		const int32 contactTileCnt = amp::getTileCount(m_contactCount);
		ampArray<int32> contactCnts(contactTileCnt, m_gpuAccelView);
		amp::fill(contactCnts, 0);
		ampArray2D<b2ParticleContact> localContacts(contactTileCnt, TILE_SIZE, m_gpuAccelView);
		amp::forEachTiled(m_contactCount, [=, &contacts, &contactCnts, &localContacts,
			&groups, &groupIdxs](const int32 gi, const int32 ti, const int32 li) restrict(amp)
		{
			const b2ParticleContact& contact = contacts[gi];
			const int32 a = contact.idxA;
			const int32 b = contact.idxB;
			const int32 groupAIdx = groupIdxs[a];
			const int32 groupBIdx = groupIdxs[b];
		
			tile_static uint32 cnt;
			if (groupAIdx != b2_invalidIndex && groupAIdx == groupBIdx &&
				(groups[groupAIdx].m_groupFlags & b2_particleGroupNeedsUpdateDepth))
			{
				localContacts[ti][Concurrency::atomic_fetch_inc(&contactCnts[ti])] = contact;
			}
		});
		const uint32 contactGroupsCount = amp::reduce(contactCnts, contactTileCnt,
			[=, &contactCnts, &localContacts, &contactGroups](const int32 i, const int32 wi) restrict(amp)
		{
			for (uint32 j = 0; j < contactCnts[i]; j++)
				contactGroups[wi + j] = localContacts[i][j];
		});

		vector<uint32> groupIdxsToUpdate(m_groupCount);
		int32 groupsToUpdateCount = 0;

		auto& accumulations = m_ampAccumulations;
		for (int k = 0; k < m_groupCount; k++)
		{
			b2ParticleGroup& group = m_groupBuffer[k];
			if (group.m_firstIndex != b2_invalidIndex)
			{
				if (group.m_groupFlags & b2_particleGroupNeedsUpdateDepth)
				{
					groupIdxsToUpdate[groupsToUpdateCount++] = k;
					SetGroupFlags(group, group.m_groupFlags & ~b2_particleGroupNeedsUpdateDepth);
					amp::forEach(group.m_firstIndex, group.m_lastIndex, [=, &accumulations](const int32 i) restrict(amp)
					{
						accumulations[i] = 0;
					});
				}
			}
		}
		// Compute sum of weight of contacts except between different groups.
		amp::forEach(contactGroupsCount, [=, &contactGroups, &accumulations](const int32 i) restrict(amp)
		{
			const b2ParticleContact& contact = contactGroups[i];
			const int32 a = contact.idxA;
			const int32 b = contact.idxB;
			const float32 w = contact.weight;
			amp::atomicAdd(accumulations[a], w);
			amp::atomicAdd(accumulations[b], w);
		});
		b2Assert(m_hasDepth);
		auto& depths = m_ampDepths;
		for (int32 i = 0; i < groupsToUpdateCount; i++)
		{
			b2ParticleGroup& group = m_groupBuffer[groupIdxsToUpdate[i]];
			const uint32 startIdx = group.m_firstIndex;
			amp::forEach(group.m_firstIndex, group.m_lastIndex, [=, &depths, &accumulations](const int32 i) restrict(amp)
			{
				const float32 w = accumulations[i];
				depths[i] = w < 0.8f ? 0 : maxFloat;
			});
		}

		// The number of iterations is equal to particle number from the deepest
		// particle to the nearest surface particle, and in general it is smaller
		// than sqrt of total particle number.
		int32 iterationCount = (int32)b2Sqrt((float)m_count);

		ampArrayView<uint32> ampUpdated(iterationCount);
		amp::fill(ampUpdated, 0u);
		for (int32 t = 0; t < iterationCount; t++)
		{
			amp::forEach(contactGroupsCount, [=, &depths, &contactGroups](const int32 i) restrict(amp)
			{
				const b2ParticleContact& contact = contactGroups[i];
				const int32 a = contact.idxA;
				const int32 b = contact.idxB;
				const float32 r = 1 - contact.weight;
				float32& ap0 = depths[a];
				float32& bp0 = depths[b];
				const float32 ap1 = bp0 + r;
				const float32 bp1 = ap0 + r;
				if (ap0 > ap1)
				{
					Concurrency::atomic_exchange(&ap0, ap1);
					ampUpdated[t] = 1;
				}
				if (bp0 > bp1)
				{
					Concurrency::atomic_exchange(&bp0, bp1);
					ampUpdated[t] = 1;
				}
			});
			uint32 updated;
			amp::copy(ampUpdated, t, updated);
			if (!updated)
				break;
		}
		for (int32 i = 0; i < groupsToUpdateCount; i++)
		{
			const b2ParticleGroup& group = m_groupBuffer[groupIdxsToUpdate[i]];
			amp::forEach(group.m_firstIndex, group.m_lastIndex, [=, &depths](const int32 i) restrict(amp)
			{
				float32& p = depths[i];
				if (p < maxFloat)
					p *= particleDiameter;
				else
					p = 0;
			});
		}
		// m_ampCopyFutDepths.set(amp::copyAsync(m_ampDepths, m_depthBuffer, m_count));
	}
	else
	{
		vector<b2ParticleContact> contactGroups(m_contactCount);
		int32 contactGroupsCount = 0;
		for (int32 k = 0; k < m_contactCount; k++)
		{
			const b2ParticleContact& contact = m_partContactBuf[k];
			int32 a = contact.idxA;
			int32 b = contact.idxB;
			const int32 groupAIdx = m_partGroupIdxBuffer[a];
			const int32 groupBIdx = m_partGroupIdxBuffer[b];
			if (groupAIdx != b2_invalidIndex && groupAIdx == groupBIdx &&
				(m_groupBuffer[groupAIdx].m_groupFlags & b2_particleGroupNeedsUpdateDepth))
			{
				contactGroups[contactGroupsCount++] = contact;
			}
		}
		vector<uint32> groupIdxsToUpdate(m_groupCount);
		int32 groupsToUpdateCount = 0;

		for (int k = 0; k < m_groupCount; k++)
		{
			b2ParticleGroup& group = m_groupBuffer[k];
			if (group.m_firstIndex != b2_invalidIndex)
			{
				if (group.m_groupFlags & b2_particleGroupNeedsUpdateDepth)
				{
					groupIdxsToUpdate[groupsToUpdateCount++] = k;
					SetGroupFlags(group,
						group.m_groupFlags &
						~b2_particleGroupNeedsUpdateDepth);
					for (int32 i = group.m_firstIndex; i < group.m_lastIndex; i++)
					{
						m_accumulationBuf[i] = 0;
					}
				}
			}
		}
		// Compute sum of weight of contacts except between different groups.
		for (int32 k = 0; k < contactGroupsCount; k++)
		{
			const b2ParticleContact& contact = contactGroups[k];
			int32 a = contact.idxA;
			int32 b = contact.idxB;
			float32 w = contact.weight;
			m_accumulationBuf[a] += w;
			m_accumulationBuf[b] += w;
		}
		b2Assert(m_hasDepth);
		for (int32 i = 0; i < groupsToUpdateCount; i++)
		{
			b2ParticleGroup& group = m_groupBuffer[groupIdxsToUpdate[i]];
			for (int32 i = group.m_firstIndex; i < group.m_lastIndex; i++)
			{
				float32 w = m_accumulationBuf[i];
				m_depthBuffer[i] = w < 0.8f ? 0 : b2_maxFloat;
			}
		}
		// The number of iterations is equal to particle number from the deepest
		// particle to the nearest surface particle, and in general it is smaller
		// than sqrt of total particle number.
		int32 iterationCount = (int32)b2Sqrt((float)m_count);
		for (int32 t = 0; t < iterationCount; t++)
		{
			bool updated = false;
			for (int32 k = 0; k < contactGroupsCount; k++)
			{
				const b2ParticleContact& contact = contactGroups[k];
				int32 a = contact.idxA;
				int32 b = contact.idxB;
				float32 r = 1 - contact.weight;
				float32& ap0 = m_depthBuffer[a];
				float32& bp0 = m_depthBuffer[b];
				float32 ap1 = bp0 + r;
				float32 bp1 = ap0 + r;
				if (ap0 > ap1)
				{
					ap0 = ap1;
					updated = true;
				}
				if (bp0 > bp1)
				{
					bp0 = bp1;
					updated = true;
				}
			}
			if (!updated)
			{
				break;
			}
		}
		for (int32 i = 0; i < groupsToUpdateCount; i++)
		{
			b2ParticleGroup& group = m_groupBuffer[groupIdxsToUpdate[i]];
			for (int32 i = group.m_firstIndex; i < group.m_lastIndex; i++)
			{
				float32& p = m_depthBuffer[i];
				if (p < b2_maxFloat)
				{
					p *= m_particleDiameter;
				}
				else
				{
					p = 0;
				}
			}
		}
	}
}

void b2ParticleSystem::AddFlagInsideFixture(const uint32 flag, const int32 matIdx,
	const Fixture& fixture, const b2Transform& transform)
{
		if (m_accelerate)
		{
			auto& flags = m_ampFlags;
			auto& matIdxs = m_ampMatIdxs;
			//CopyShapeToGPU(fixture.m_shapeType, fixture.m_shapeIdx);
			switch (fixture.m_shapeType)
			{
			case b2Shape::e_circle:
				AmpForEachInsideCircle(m_world.m_circleShapeBuffer[fixture.m_shapeIdx],
					transform, [=, &flags, &matIdxs](int32 i) restrict(amp)
				{
					if (matIdxs[i] == matIdx)
						flags[i] |= flag;
				});
				break;
			}
		}
		else
		{
			b2AABB aabb;
			const b2Shape& shape = m_world.GetShape(fixture);
			int32 childCount = shape.GetChildCount();
			for (int32 childIndex = 0; childIndex < childCount; childIndex++)
			{
				shape.ComputeAABB(aabb, transform, childIndex);


				b2ParticleSystem::InsideBoundsEnumerator enumerator =
					GetInsideBoundsEnumerator(aabb);
				int32 i;
				while ((i = enumerator.GetNext()) >= 0)
				{
					if (m_partMatIdxBuffer[i] == matIdx && shape.TestPoint(transform, m_positionBuffer[i]))
					{
						AddParticleFlags(i, flag);
					}
				}
			}
		}
}

void b2ParticleSystem::CopyShapeToGPU(b2Shape::Type type, int32 idx)
{
	switch (type)
	{
	case b2Shape::e_chain:
		amp::copy((AmpChainShape&)m_world.m_chainShapeBuffer[idx], m_ampChainShapes, idx);
		return;
	case b2Shape::e_circle:
		amp::copy((AmpCircleShape&)m_world.m_circleShapeBuffer[idx], m_ampCircleShapes, idx);
		return;
	case b2Shape::e_edge:
		amp::copy((AmpEdgeShape&)m_world.m_edgeShapeBuffer[idx], m_ampEdgeShapes, idx);
		return;
	case b2Shape::e_polygon:
		amp::copy((AmpPolygonShape&)m_world.m_polygonShapeBuffer[idx], m_ampPolygonShapes, idx);
		return;
	}
}

b2ParticleSystem::InsideBoundsEnumerator
b2ParticleSystem::GetInsideBoundsEnumerator(const b2AABB& aabb) const
{
	uint32 lowerTag = computeTag(m_inverseDiameter * aabb.lowerBound.x - 1,
		m_inverseDiameter * aabb.lowerBound.y - 1);
	uint32 upperTag = computeTag(m_inverseDiameter * aabb.upperBound.x + 1,
		m_inverseDiameter * aabb.upperBound.y + 1);

	const Proxy* beginProxy = m_proxyBuffer.data();
	const Proxy* endProxy	= beginProxy + m_count;
	const Proxy* firstProxy = std::lower_bound(beginProxy, endProxy, lowerTag);
	const Proxy* lastProxy	= std::upper_bound(firstProxy, endProxy, upperTag);

	return InsideBoundsEnumerator(lowerTag, upperTag, firstProxy, lastProxy);
}

void b2ParticleSystem::BoundProxyToTagBound(const b2AABBFixtureProxy& aabb, b2TagBounds& tb)
{
	tb.lowerTag = computeTag(m_inverseDiameter * aabb.lowerBound.x - 1,
		m_inverseDiameter * aabb.lowerBound.y - 1);
	tb.upperTag = computeTag(m_inverseDiameter * aabb.upperBound.x + 1,
		m_inverseDiameter * aabb.upperBound.y + 1);
	tb.xLower = tb.lowerTag & xMask;
	tb.xUpper = tb.upperTag & xMask;
	tb.fixtureIdx = aabb.fixtureIdx;
	tb.childIdx = aabb.childIdx;
}

template<typename F>
void b2ParticleSystem::AmpForEachInsideBounds(const b2AABB& aabb, F& function)
{
	uint32 lowerTag = computeTag(m_inverseDiameter * aabb.lowerBound.x - 1,
		m_inverseDiameter * aabb.lowerBound.y - 1);
	uint32 upperTag = computeTag(m_inverseDiameter * aabb.upperBound.x + 1,
		m_inverseDiameter * aabb.upperBound.y + 1);
	const uint32 xLower = lowerTag & xMask;
	const uint32 xUpper = upperTag & xMask;

	auto& proxies = m_ampProxies;
	AmpForEachParticle([=, &proxies](const int32 i) restrict(amp)
	{
		const Proxy& proxy = proxies[i];
		if (proxy.tag < lowerTag || proxy.tag > upperTag) return;
		const uint32 xTag = proxy.tag & xMask;
		if (xTag < xLower || xTag > xUpper) return;
		function(proxy.idx);
	});
}
template<typename F>
void b2ParticleSystem::AmpForEachInsideBounds(const vector<b2AABBFixtureProxy>& aabbs, F& function)
{
	if (aabbs.empty()) return;
	int32 boundCnt = 0;
	vector<b2TagBounds> tagBounds(aabbs.size());
	for (const b2AABBFixtureProxy& aabb : aabbs)
		BoundProxyToTagBound(aabb, tagBounds[boundCnt++]);
	ampArrayView<const b2TagBounds> ampTagBounds(boundCnt, tagBounds);

	auto& proxies = m_ampProxies;
	AmpForEachParticle([=, &proxies](const int32 i) restrict(amp)
	{
		const Proxy& proxy = proxies[i];
		const uint32 xTag = proxy.tag & xMask;
		for (int32 i = 0; i < boundCnt; i++)
		{
			const b2TagBounds& tb = ampTagBounds[i];
			if (proxy.tag < tb.lowerTag || proxy.tag > tb.upperTag) continue;
			if (xTag < tb.xLower || xTag > tb.xUpper) continue;
			function(proxy.idx, tb.fixtureIdx, tb.childIdx);
		}
	});
}
template<typename F>
void b2ParticleSystem::AmpForEachInsideCircle(const b2CircleShape& circle, const b2Transform& transform, F& function)
{
	b2AABB aabb;
	circle.ComputeAABB(aabb, transform, 0);
	const uint32 lowerTag = computeTag(m_inverseDiameter * aabb.lowerBound.x - 1,
		m_inverseDiameter * aabb.lowerBound.y - 1);
	const uint32 upperTag = computeTag(m_inverseDiameter * aabb.upperBound.x + 1,
		m_inverseDiameter * aabb.upperBound.y + 1);
	const uint32 xLower = lowerTag & xMask;
	const uint32 xUpper = upperTag & xMask;

	ampArrayView<const AmpCircleShape> ampCircle(1, (AmpCircleShape*)&circle);
	auto& proxies = m_ampProxies;
	auto& positions = m_ampPositions;
	AmpForEachParticle([=, &proxies, &positions](const int32 i) restrict(amp)
	{
		const Proxy& proxy = proxies[i];
		if (proxy.tag < lowerTag || proxy.tag > upperTag) return;
		const uint32 xTag = proxy.tag & xMask;
		if (xTag < xLower || xTag > xUpper) return;
		if (ampCircle[0].TestPoint(transform, positions[proxy.idx]))
			function(proxy.idx);
	});
}

void b2ParticleSystem::FindContacts()
{
	ResizeContactBuffers(m_count * MAX_CONTACTS_PER_PARTICLE);
	m_contactCount = 0;

	for (int a = 0, c = 0; a < m_count; a++)
	{
		int32 aContactCount = 0;
		const int32 aIdx = m_proxyBuffer[a].idx;
		const uint32& aTag = m_proxyBuffer[a].tag;
		const uint32 rightTag = computeRelativeTag(aTag, 1, 0);
		for (int b = a + 1; b < m_count; b++)
		{
			if (rightTag < m_proxyBuffer[b].tag) break;
			if (AddContact(aIdx, m_proxyBuffer[b].idx, m_contactCount)
				&& ++aContactCount == MAX_CONTACTS_PER_PARTICLE) goto cnt;
		}

		const uint32 bottomLeftTag = computeRelativeTag(aTag, -1, 1);
		for (; c < m_count; c++)
			if (bottomLeftTag <= m_proxyBuffer[c].tag) break;

		const uint32 bottomRightTag = computeRelativeTag(aTag, 1, 1);
		for (int b = c; b < m_count; b++)
		{
			if (bottomRightTag < m_proxyBuffer[b].tag) break;
			if (AddContact(aIdx, m_proxyBuffer[b].idx, m_contactCount)
				&& ++aContactCount == MAX_CONTACTS_PER_PARTICLE) goto cnt;
		}
		cnt:;
	}
}
inline bool b2ParticleSystem::AddContact(int32 a, int32 b, int32& contactCount)
{
	const int32& colGroupA = m_groupBuffer[m_partGroupIdxBuffer[a]].m_collisionGroup;
	const int32& colGroupB = m_groupBuffer[m_partGroupIdxBuffer[b]].m_collisionGroup;
	if (colGroupA > 0 && colGroupA != colGroupB) return false;

	const b2Vec3 d = m_positionBuffer[b] - m_positionBuffer[a];
	const float32 distBtParticlesSq = d * d;
	if (distBtParticlesSq > m_squaredDiameter) return false;

	b2ParticleContact& contact = m_partContactBuf[contactCount];
	contactCount++;

	float32 invD = b2InvSqrt(distBtParticlesSq);
	contact.idxA = a;
	contact.idxB = b;
	contact.flags = m_flagsBuffer[a] | m_flagsBuffer[b];
	const int32 matAIdx = m_partMatIdxBuffer[a];
	const int32 matBIdx = m_partMatIdxBuffer[b];
	contact.matFlags = m_partMatFlagsBuf[matAIdx] | m_partMatFlagsBuf[matBIdx];
	contact.weight = 1 - distBtParticlesSq * invD * m_inverseDiameter;
	const float32 invM = 1 / (m_partMatMassBuf[matAIdx] + m_partMatMassBuf[matBIdx]);
	contact.mass = invM > 0 ? invM : 0;
	contact.normal = invD * b2Vec2(d);
	return true;
}

inline bool b2ParticleSystem::ShouldCollide(int32 i, const Fixture& f) const
{
	if (f.m_filter.groupIndex >= 0)
		return true;
	const int32 groupIdx = m_partGroupIdxBuffer[i];
	if (groupIdx == b2_invalidIndex)
		return false;
	const int32 partColGroup = m_groupBuffer[groupIdx].m_collisionGroup;
	if (!partColGroup || !(partColGroup < 0))
		return true;
	if (partColGroup != f.m_filter.groupIndex)
		return true;
	return false;
}


void b2ParticleSystem::AmpFindContacts(bool exceptZombie)
{
	ResizeContactBuffers(m_count * MAX_CONTACTS_PER_PARTICLE);

	const int32 cnt = m_count;

	auto& positions	= m_ampPositions;
	auto& flags = m_ampFlags;
	auto& matIdxs	= m_ampMatIdxs;
	auto& matFlags	= m_ampMatFlags;
	auto& matMasses	= m_ampMatMasses;
	const float32 invDiameter = m_inverseDiameter;
	const float32 sqrDiameter = m_squaredDiameter;
	const auto addContact = [=, &positions, &flags, &matIdxs, &matFlags,
		&matMasses](const uint32 & a, const uint32 & b, b2ParticleContact& contact) restrict(amp) -> bool {
		//const int32 colGroupA = groups[groupIdxs[a]].m_collisionGroup;
		//const int32 colGroupB = groups[groupIdxs[b]].m_collisionGroup;
		//if (colGroupA > 0 && colGroupA != colGroupB) return false;
		const uint32 contactFlags = flags[a] | flags[b];

		const b2Vec3 d = positions[b] - positions[a];
		const float32 distBtParticlesSq = d * d;
		if (distBtParticlesSq > sqrDiameter) return false;

		contact.idxA = a;
		contact.idxB = b;
		contact.flags = contactFlags;
		const int32 matAIdx = matIdxs[a];
		const int32 matBIdx = matIdxs[b];
		contact.matFlags = matFlags[matAIdx] | matFlags[matBIdx];
		const float32 invD = b2InvSqrt(distBtParticlesSq);
		contact.weight = 1 - distBtParticlesSq * invD * invDiameter;
		float32 invM = 1 / (matMasses[matAIdx] + matMasses[matBIdx]);
		contact.mass = invM > 0 ? invM : 0;
		contact.normal = invD * d;
		return true;
	};

	auto& proxies = m_ampProxies;
	auto& localContacts = m_localContacts;
	auto& localContactCnts = m_localContactCnts;
	amp::fill(localContactCnts, 0);

	const auto TagLowerBound = [=, &proxies](uint32 first, uint32 tag) restrict(amp) -> int32
	{
		int32 i, step;
		int32 count = cnt - first;

		while (count > 0) {
			i = first;
			step = count / 2;
			i += step;
			if (proxies[i].tag < tag) {
				first = ++i;
				count -= step + 1;
			}
			else
				count = step;
		}
		return first;
	};
	auto& contacts = m_ampContacts;
	AmpForEachParticle([=, &contacts, &localContacts, &localContactCnts, &proxies](const int32 i) restrict(amp)
	{
		int32& localContactCnt = localContactCnts[i];
		auto& contacts = localContacts[i];

		const Proxy aProxy = proxies[i];
		const int32 aIdx = aProxy.idx;
		const uint32 rightTag = computeRelativeTag(aProxy.tag, 1, 0);
		for (uint32 b = i + 1; b < cnt; b++)
		{
			if (rightTag < proxies[b].tag) break;
			if (addContact(aIdx, proxies[b].idx, contacts[localContactCnt])
				&& ++localContactCnt == MAX_CONTACTS_PER_PARTICLE) break;
		}
		// Optimisable ?
		const uint32 bottomLeftTag = computeRelativeTag(proxies[i].tag, -1, 1);
		uint32 c = TagLowerBound(i + 1, bottomLeftTag);
		const uint32 bottomRightTag = computeRelativeTag(aProxy.tag, 1, 1);
		for (uint32 b = c; b < cnt; b++)
		{
			if (bottomRightTag < proxies[b].tag) break;
			if (addContact(aIdx, proxies[b].idx, contacts[localContactCnt])
				&& ++localContactCnt == MAX_CONTACTS_PER_PARTICLE) break;
		}
	});
	m_contactCount = amp::reduce(localContactCnts, m_count, [=, &localContactCnts,
		&contacts, &localContacts](const int32 i, const int32 wi) restrict(amp)
	{
		for (int32 j = 0; j < localContactCnts[i]; j++)
			contacts[wi + j] = localContacts[i][j];
	});
}

static inline bool b2ParticleContactIsZombie(const b2ParticleContact& contact)
{
	return (contact.flags & b2_zombieParticle) == b2_zombieParticle;
}

// Get the world's contact filter if any particles with the
// b2_particleContactFilterParticle flag are present in the system.
inline b2ContactFilter* b2ParticleSystem::GetParticleContactFilter() const
{
	return (m_allParticleFlags & b2_particleContactFilterParticle) ?
		m_world.m_contactManager.m_contactFilter : NULL;
}

// Get the world's contact listener if any particles with the
// b2_particleContactListenerParticle flag are present in the system.
inline b2ContactListener* b2ParticleSystem::GetParticleContactListener() const
{
	return (m_allParticleFlags & b2_particleContactListenerParticle) ?
		m_world.m_contactManager.m_contactListener : NULL;
}



class b2ParticleContactRemovePredicate
{
public:
	b2ParticleContactRemovePredicate(
		b2ParticleSystem* system,
		b2ContactFilter* contactFilter) :
		m_system(system),
		m_contactFilter(contactFilter)
	{}

	bool operator()(const b2ParticleContact& contact)
	{
	    return (contact.flags & b2_particleContactFilterParticle)
	        && !m_contactFilter->ShouldCollide(m_system, contact.idxA,
	        								   contact.idxB);
	}

private:
	b2ParticleSystem* m_system;
	b2ContactFilter* m_contactFilter;
};

// Only changes 'contacts', but the contact filter has a non-const 'this'
// pointer, so this member function cannot be const.
void b2ParticleSystem::FilterContacts()
{
	// Optionally filter the contact.
	b2ContactFilter* const contactFilter = GetParticleContactFilter();
	if (contactFilter == NULL)
		return;

	//RemoveFromVectorIf(m_contacts, m_contactCount, b2ParticleContactRemovePredicate(this, contactFilter), false, true);

}

void b2ParticleSystem::NotifyContactListenerPreContact(
	b2ParticlePairSet* particlePairs) const
{
	/*b2ContactListener* const contactListener = GetParticleContactListener();
	if (contactListener == NULL)
		return;

	particlePairs->Initialize(m_contactIdxABuffer, m_contactIdxBBuffer,
							  m_contactCount,
						      m_flagsBuffer);*/
}

// Note: This function is not const because 'this' in BeginContact and
// EndContact callbacks must be non-const. However, this function itself
// does not change any internal data (though the callbacks might).
void b2ParticleSystem::NotifyContactListenerPostContact(
	b2ParticlePairSet& particlePairs)
{
	/*b2ContactListener* const contactListener = GetParticleContactListener();
	if (contactListener == NULL)
		return;

	// Loop through all new contacts, reporting any new ones, and
	// "invalidating" the ones that still exist.
	for (int i = 0; i < m_contactCount; i++)
	{
		ParticlePair pair;
		pair.first = m_contactIdxABuffer[i];
		pair.second = m_contactIdxBBuffer[i];
		const int32 itemIndex = particlePairs.Find(pair);
		if (itemIndex >= 0)
		{
			// Already touching, ignore this contact.
			particlePairs.Invalidate(itemIndex);
		}
		else
		{
			// Just started touching, inform the listener.
			//contactListener->BeginContact(this, &contact);
		}
	}

	// Report particles that are no longer touching.
	// That is, any pairs that were not invalidated above.
	const int32 pairCount = particlePairs.GetCount();
	const ParticlePair* const pairs = particlePairs.GetBuffer();
	const int8* const valid = particlePairs.GetValidBuffer();
	for (int32 i = 0; i < pairCount; ++i)
	{
		if (valid[i])
		{
			contactListener->EndContact(this, pairs[i].first,
										pairs[i].second);
		}
	}*/
}

void b2ParticleSystem::UpdateProxies()
{
	for (int i = 0; i < m_count; i++)
	{
		const b2Vec3& pos = m_positionBuffer[i];
		m_proxyBuffer[i] = Proxy(i, computeTag(m_inverseDiameter * pos.x, m_inverseDiameter * pos.y));
	}
}
void b2ParticleSystem::AmpUpdateProxies()
{
	const float32 invDiameter = m_inverseDiameter;
	auto& proxies = m_ampProxies;
	auto& positions = m_ampPositions;
	AmpForEachParticle([=, &proxies, &positions](const int32 i) restrict(amp)
	{
		const b2Vec3& pos = positions[i];
		proxies[i] = Proxy(i, computeTag(invDiameter * pos.x, invDiameter * pos.y));
	});
}

// Sort the proxy array by 'tag'. This orders the particles into rows that
// run left-to-right, top-to-bottom. The rows are spaced m_particleDiameter
// apart, such that a particle in one row can only collide with the rows
// immediately above and below it. This ordering makes collision computation
// tractable.
void b2ParticleSystem::SortProxies()
{
	std::sort(m_proxyBuffer.data(), m_proxyBuffer.data() + m_count);
}

void b2ParticleSystem::AmpSortProxies()
{
	amp::radixSort(m_ampProxies, m_count);
}


template<class T>
void b2ParticleSystem::reorder(vector<T>& v, const vector<int32>& order) 
{
	std::vector<bool> done(order.size());
	for (std::size_t i = 0; i < order.size(); ++i)
	{
		if (done[i])
		{
			continue;
		}
		done[i] = true;
		std::size_t prev_j = i;
		std::size_t j = order[i];
		while (i != j)
		{
			std::swap(v[prev_j], v[j]);
			done[j] = true;
			prev_j = j;
			j = order[j];
		}
	}
}
template<class T1, class T2>
void b2ParticleSystem::reorder(vector<T1>& v1, vector<T2>& v2, const vector<int32>& order)
{
	std::vector<bool> done(order.size());
	for (std::size_t i = 0; i < order.size(); ++i)
	{
		if (done[i])
		{
			continue;
		}
		done[i] = true;
		std::size_t prev_j = i;
		std::size_t j = order[i];
		while (i != j)
		{
			std::swap(v1[prev_j], v1[j]);
			std::swap(v2[prev_j], v2[j]);
			done[j] = true;
			prev_j = j;
			j = order[j];
		}
	}
}


void b2ParticleSystem::DetectStuckParticle(int32 particle)
{
	// Detect stuck particles
	//
	// The basic algorithm is to allow the user to specify an optional
	// threshold where we detect whenever a particle is contacting
	// more than one fixture for more than threshold consecutive
	// steps. This is considered to be "stuck", and these are put
	// in a list the user can query per step, if enabled, to deal with
	// such particles.

	if (m_stuckThreshold <= 0)
	{
		return;
	}

	// Get the state variables for this particle.
	int32& consecutiveCount = m_consecutiveContactStepsBuffer[particle];
	int32& lastStep = m_lastBodyContactStepBuffer[particle];
	int32& bodyCount = m_bodyContactCountBuffer[particle];

	// This is only called when there is a body contact for this particle.
	++bodyCount;

	// We want to only trigger detection once per step, the first time we
	// contact more than one fixture in a step for a given particle.
	if (bodyCount == 2)
	{
		++consecutiveCount;
		if (consecutiveCount > m_stuckThreshold)
		{
			m_stuckParticleBuffer[m_stuckParticleCount] = particle;
			m_stuckParticleCount++;
		}
	}
	lastStep = m_timestamp;
}

// Get the world's contact listener if any particles with the
// b2_fixtureContactListenerParticle flag are present in the system.
inline b2ContactListener* b2ParticleSystem::GetFixtureContactListener() const
{
	return (m_allParticleFlags & b2_fixtureContactListenerParticle) ?
		m_world.m_contactManager.m_contactListener : NULL;
}

// Get the world's contact filter if any particles with the
// b2_fixtureContactFilterParticle flag are present in the system.
inline b2ContactFilter* b2ParticleSystem::GetFixtureContactFilter() const
{
	return (m_allParticleFlags & b2_fixtureContactFilterParticle) ?
		m_world.m_contactManager.m_contactFilter : NULL;
}

/// Compute the axis-aligned bounding box for all particles contained
/// within this particle system.
/// @param aabb Returns the axis-aligned bounding box of the system.
void b2ParticleSystem::ComputeAABB(b2AABB* const aabb) const
{
	const int32 particleCount = GetParticleCount();
	b2Assert(aabb);
	aabb->lowerBound.x = +b2_maxFloat;
	aabb->lowerBound.y = +b2_maxFloat;
	aabb->upperBound.x = -b2_maxFloat;
	aabb->upperBound.y = -b2_maxFloat;

	for (int32 i = 0; i < particleCount; i++)
	{
		const b2Vec3& p = m_positionBuffer[i];
		aabb->lowerBound = b2Min(aabb->lowerBound, p);
		aabb->upperBound = b2Max(aabb->upperBound, p);
	}
	aabb->lowerBound.x -= m_particleDiameter;
	aabb->lowerBound.y -= m_particleDiameter;
	aabb->upperBound.x += m_particleDiameter;
	aabb->upperBound.y += m_particleDiameter;
}
void b2ParticleSystem::AmpComputeAABB(b2AABB& aabb, bool addVel) const
{
	const uint32 cnt = m_count;
	auto& flags = m_ampFlags;
	auto& positions = m_ampPositions;
	auto& velocities = m_ampVelocities;
	const uint32 zombieFlag = b2_zombieParticle;
	const float32 minFloat = -b2_maxFloat;
	const float32 maxFloat = +b2_maxFloat;
	int32 tileCnt;
	int32 halfCnt = amp::getTilable(cnt / 2, tileCnt);
	ampArrayView<b2AABB> tileAABBs(b2Max(1, tileCnt));
	amp::forEachTiledWithBarrier(halfCnt, [=, &flags,
		&positions, &velocities](ampTiledIdx<TILE_SIZE> tIdx) restrict(amp)
	{
		const uint32 gi = tIdx.global[0];
		const uint32 li = tIdx.local[0];
		const uint32 ti = tIdx.tile[0];
		tile_static b2AABB aabbs[TILE_SIZE];
		const uint32 a = gi;
		const uint32 b = gi + TILE_SIZE;
		const bool aZombie = flags[a] & zombieFlag;
		const bool bZombie = b >= cnt || flags[b] & zombieFlag;
		b2AABB& aabb = aabbs[li];
		if (aZombie && bZombie)
		{
			aabb.lowerBound.x = aabb.lowerBound.y = maxFloat;
			aabb.upperBound.x = aabb.upperBound.y = minFloat;
		}
		else if (aZombie)
		{
			b2Vec2 bPos = b2Vec2(positions[b]);
			if (addVel)
				bPos += velocities[b];
			aabb.lowerBound.x = aabb.upperBound.x = bPos.x;
			aabb.lowerBound.y = aabb.upperBound.y = bPos.y;
		}
		else if (bZombie)
		{
			b2Vec2 aPos = b2Vec2(positions[a]);
			if (addVel)
				aPos += velocities[a];
			aabb.lowerBound.x = aabb.upperBound.x = aPos.x;
			aabb.lowerBound.y = aabb.upperBound.y = aPos.y;
		}
		else
		{
			b2Vec2 aPos = b2Vec2(positions[a]);
			b2Vec2 bPos = b2Vec2(positions[b]);
			if (addVel)
			{
				aPos += velocities[a];
				bPos += velocities[b];
			}
			aabb.lowerBound = b2Min(aPos, bPos);
			aabb.upperBound = b2Max(aPos, bPos);
		}
		tIdx.barrier.wait_with_tile_static_memory_fence();
		for (uint32 stride = TILE_SIZE_HALF; stride > 0; stride /= 2)
		{
			if (li < stride)
			{
				const b2AABB& aabb2 = aabbs[li + stride];
				aabb.lowerBound = b2Min(aabb.lowerBound, aabb2.lowerBound);
				aabb.upperBound = b2Max(aabb.upperBound, aabb2.upperBound);
			}
			tIdx.barrier.wait_with_tile_static_memory_fence();
		}
		if (li == 0)
			tileAABBs[ti] = aabbs[0];
	});
	aabb.lowerBound.x = aabb.lowerBound.y = maxFloat;
	aabb.upperBound.x = aabb.upperBound.y = minFloat;
	for (int i = 0; i < tileCnt; i++)
	{
		aabb.lowerBound = b2Min(aabb.lowerBound, tileAABBs[i].lowerBound);
		aabb.upperBound = b2Max(aabb.upperBound, tileAABBs[i].upperBound);
	}
	aabb.lowerBound.x -= m_particleDiameter;
	aabb.lowerBound.y -= m_particleDiameter;
	aabb.upperBound.x += m_particleDiameter;
	aabb.upperBound.y += m_particleDiameter;
}

// Associate a memory allocator with this object.
FixedSetAllocator::FixedSetAllocator(
		b2StackAllocator* allocator) :
	m_buffer(NULL), m_valid(NULL), m_count(0), m_allocator(allocator)
{
	b2Assert(allocator);
}

// Allocate internal storage for this object.
int32 FixedSetAllocator::Allocate(
	const int32 itemSize, const int32 count)
{
	Clear();
	if (count)
	{
		m_buffer = m_allocator->Allocate(
			(itemSize + sizeof(*m_valid)) * count);
		b2Assert(m_buffer);
		m_valid = (int8*)m_buffer + (itemSize * count);
		memset(m_valid, 1, sizeof(*m_valid) * count);
		m_count = count;
	}
	return m_count;
}

// Deallocate the internal buffer if it's allocated.
void FixedSetAllocator::Clear()
{
	if (m_buffer)
	{
		m_allocator->Free(m_buffer);
		m_buffer = NULL;
        m_count = 0;
	}
}

// Search set for item returning the index of the item if it's found, -1
// otherwise.
template<typename T>
static int32 FindItemIndexInFixedSet(const TypedFixedSetAllocator<T>& set,
									 const T& item)
{
	if (set.GetCount())
	{
		const T* buffer = set.GetBuffer();
		const T* last = buffer + set.GetCount();
		const T* found = lower_bound( buffer, buffer + set.GetCount(),
											item, T::Compare);
		if( found != last )
		{
			return set.GetIndex( found );
		}
	}
	return -1;
}

// Initialize from a set of particle / body contacts for particles
// that have the b2_fixtureContactListenerParticle flag set.
void FixtureParticleSet::Initialize(
	const vector<b2PartBodyContact>& bodyContacts,
	const int32 numBodyContacts,
	const uint32 * const particleFlagsBuffer)
{
	Clear();
	if (Allocate(numBodyContacts))
	{
		FixtureParticle* set = GetBuffer();
		int32 insertedContacts = 0;
		for (int32 i = 0; i < numBodyContacts; ++i)
		{
			FixtureParticle* const fixtureParticle = &set[i];
			const b2PartBodyContact& bodyContact = bodyContacts[i];
			const int32 partIdx = bodyContact.partIdx;
			if (partIdx == b2_invalidIndex ||
				!(particleFlagsBuffer[partIdx] &
				  b2_fixtureContactListenerParticle))
			{
				continue;
			}
			fixtureParticle->first = bodyContact.fixtureIdx;
			fixtureParticle->second = partIdx;
			insertedContacts++;
		}
		SetCount(insertedContacts);
		//std::sort(set, set + insertedContacts, FixtureParticle::Compare);
		Concurrency::parallel_sort(set, set + insertedContacts, FixtureParticle::Compare);
	}
}

// Find the index of a particle / fixture pair in the set or -1 if it's not
// present.
int32 FixtureParticleSet::Find(
	const FixtureParticle& fixtureParticle) const
{
	return FindItemIndexInFixedSet(*this, fixtureParticle);
}

// Initialize from a set of particle contacts.
void b2ParticlePairSet::Initialize(
	const vector<int32> contactIdxAs, const vector<int32> contactIdxBs, int32 numContacts,
	const vector<uint32> particleFlagsBuffer)
{
	Clear();
	if (Allocate(numContacts))
	{
		ParticlePair* set = GetBuffer();
		int32 insertedContacts = 0;
		for (int32 i = 0; i < numContacts; ++i)
		{
			ParticlePair* const pair = &set[i];
			const int32 idxA = contactIdxAs[i],
						idxB = contactIdxBs[i];
			if (idxA == b2_invalidIndex ||
				idxB == b2_invalidIndex ||
				!((particleFlagsBuffer[idxA] |
				   particleFlagsBuffer[idxB]) &
				  b2_particleContactListenerParticle))
			{
				continue;
			}
			pair->first = idxA;
			pair->second = idxB;
			insertedContacts++;
		}
		SetCount(insertedContacts);
		//std::sort(set, set + insertedContacts, ParticlePair::Compare);
		Concurrency::parallel_sort(set, set + insertedContacts, ParticlePair::Compare);
	}
}

// Find the index of a particle pair in the set or -1 if it's not present.
int32 b2ParticlePairSet::Find(const ParticlePair& pair) const
{
	int32 index = FindItemIndexInFixedSet(*this, pair);
	if (index < 0)
	{
		ParticlePair swapped;
		swapped.first = pair.second;
		swapped.second = pair.first;
		index = FindItemIndexInFixedSet(*this, swapped);
	}
	return index;
}

/// Callback class to receive pairs of fixtures and particles which may be
/// overlapping. Used as an argument of b2World::QueryAABB.
class b2FixtureParticleQueryCallback : public b2QueryCallback
{
public:
	explicit b2FixtureParticleQueryCallback(b2World& world, b2ParticleSystem& system)
		: m_world(world), m_system(system) {}

private:
	// Skip reporting particles.
	bool ShouldQueryParticleSystem(const b2ParticleSystem* system)
	{
		B2_NOT_USED(system);
		return false;
	}

	// Receive a fixture and call ReportFixtureAndParticle() for each particle
	// inside aabb of the fixture.
	bool ReportFixture(int32 fixtureIdx)
	{
		Fixture& fixture = m_world.m_fixtureBuffer[fixtureIdx];
		if (fixture.m_isSensor) return true;

		const b2Shape& b2Shape = m_world.GetShape(fixture);
		int32 childCount = b2Shape.GetChildCount();
		for (int32 childIndex = 0; childIndex < childCount; childIndex++)
		{
			b2AABB aabb = m_world.GetAABB(fixture, childIndex);
			b2ParticleSystem::InsideBoundsEnumerator enumerator =
								m_system.GetInsideBoundsEnumerator(aabb);
			
			//m_system.AmpForEachInsideBounds(aabb, )

			// b2Vec2 bodyPos = fixture.GetBody().GetPosition();
			int32 index;

			while ((index = enumerator.GetNext()) >= 0)
			{
				ReportFixtureAndParticle(fixtureIdx, childIndex, index);
			}
		}
		return true;
	}

	// Receive a fixture and a particle which may be overlapping.
	virtual void ReportFixtureAndParticle(int32 fixtureIdx, int32 childIndex, int32 index) = 0;

protected:
	b2World& m_world;
	b2ParticleSystem& m_system;
};

void b2ParticleSystem::NotifyBodyContactListenerPreContact(
	FixtureParticleSet* fixtureSet) const
{
	b2ContactListener* const contactListener = GetFixtureContactListener();
	if (contactListener == NULL)
		return;

	fixtureSet->Initialize(m_bodyContactBuf,
						   m_bodyContactCount,
						   GetFlagsBuffer());
}

// If a contact listener is present and the contact is just starting
// report the contact.  If the contact is already in progress invalid
// the contact from m_fixtureSet.
void b2ParticleSystem::NotifyBodyContactListenerPostContact(
	FixtureParticleSet& fixtureSet)
{
	b2ContactListener* const contactListener = GetFixtureContactListener();
	if (contactListener == NULL)
		return;

	// Loop through all new contacts, reporting any new ones, and
	// "invalidating" the ones that still exist.
	for (int i = 0; i < m_bodyContactCount; i++)
	{
		const b2PartBodyContact& contact = m_bodyContactBuf[i];
		FixtureParticle fixtureParticleToFind;
		fixtureParticleToFind.first = contact.fixtureIdx;
		fixtureParticleToFind.second = contact.partIdx;
		const int32 index = fixtureSet.Find(fixtureParticleToFind);
		if (index >= 0)
		{
			// Already touching remove this from the set.
			fixtureSet.Invalidate(index);
		}
		else
		{
			// Just started touching, report it!
			//contactListener->BeginContact(this, &contact);
		}
	}

	// If the contact listener is enabled, report all fixtures that are no
	// longer in contact with particles.
	const FixtureParticle* const fixtureParticles = fixtureSet.GetBuffer();
	const int8* const fixtureParticlesValid = fixtureSet.GetValidBuffer();
	const int32 fixtureParticleCount = fixtureSet.GetCount();
	for (int32 i = 0; i < fixtureParticleCount; ++i)
	{
		if (fixtureParticlesValid[i])
		{
			const FixtureParticle* const fixtureParticle =
				&fixtureParticles[i];
			contactListener->EndContact(fixtureParticle->first, this,
										fixtureParticle->second);
		}
	}
}


void b2ParticleSystem::UpdateBodyContacts()
{
	// If the particle contact listener is enabled, generate a set of
	// fixture / particle contacts.
	FixtureParticleSet fixtureSet(&m_world.m_stackAllocator);
	NotifyBodyContactListenerPreContact(&fixtureSet);

	if (m_stuckThreshold > 0)
	{
		for (int32 i = 0; i < m_count; i++)
		{
			// Detect stuck particles, see comment in
			// b2ParticleSystem::DetectStuckParticle()
			m_bodyContactCountBuffer[i] = 0;
			if (m_timestamp > (m_lastBodyContactStepBuffer[i] + 1))
				m_consecutiveContactStepsBuffer[i] = 0;
		}
	}
	m_bodyContactCount = 0;
	m_stuckParticleCount = 0;

	class UpdateBodyContactsCallback : public b2FixtureParticleQueryCallback
	{
		void ReportFixtureAndParticle(int32 fixtureIdx, int32 childIndex, int32 a)
		{
			Fixture& fixture = m_world.m_fixtureBuffer[fixtureIdx];
			if (m_system.ShouldCollide(a, fixture))
			{
				b2Vec2 ap = b2Vec2(m_system.m_positionBuffer[a]);
				float32 d;
				b2Vec2 n;

				m_world.ComputeDistance(fixture, ap, d, n, childIndex);
				if (d < m_system.m_particleDiameter)
				{
					int32 bIdx = fixture.m_bodyIdx;
					Body& b = m_world.m_bodyBuffer[bIdx];
					b2Vec2 bp = b.GetWorldCenter();
					float32 bm = b.m_mass;
					float32 bI =
						b.GetInertia() - bm * b.GetLocalCenter().LengthSquared();
					float32 invBm = bm > 0 ? 1 / bm : 0;
					float32 invBI = bI > 0 ? 1 / bI : 0;
					float32 invAm =
						m_system.m_flagsBuffer[a] &
						b2_wallParticle ? 0 : m_system.m_partMatInvMassBuf[m_system.m_partMatIdxBuffer[a]];
					b2Vec2 rp = ap - bp;
					float32 rpn = b2Cross(rp, n);
					float32 invM = invAm + invBm + invBI * rpn * rpn;

					const uint32 bodyContactIdx = m_system.m_bodyContactCount++;
					m_system.ResizeBodyContactBuffers(m_system.m_bodyContactCount);
					b2PartBodyContact& contact = m_system.m_bodyContactBuf[bodyContactIdx];
					contact.partIdx = a;
					contact.bodyIdx = bIdx;
					contact.fixtureIdx = fixtureIdx;
					contact.weight = 1 - d * m_system.m_inverseDiameter;
					contact.normal = -n;
					contact.mass = invM > 0 ? 1 / invM : 0;
					m_system.DetectStuckParticle(a);
				}
			}
		}

		b2ContactFilter* m_contactFilter;

	public:
		UpdateBodyContactsCallback(b2World& world,
			b2ParticleSystem& system, b2ContactFilter* contactFilter):
			b2FixtureParticleQueryCallback(world, system)
		{
			m_contactFilter = contactFilter;
		}
	} callback(m_world, *this, GetFixtureContactFilter());

	b2AABB aabb;
	ComputeAABB(&aabb);
	m_world.QueryAABB(&callback, aabb);
	
	if (m_def.strictContactCheck)
		RemoveSpuriousBodyContacts();

	NotifyBodyContactListenerPostContact(fixtureSet);
}
void b2ParticleSystem::AmpUpdateBodyContacts()
{
	// If the particle contact listener is enabled, generate a set of
	// fixture / particle contacts.

	m_bodyContactCount = 0;
	
	vector<b2AABBFixtureProxy> fixtureBounds;

	b2AABB partsBounds;
	AmpComputeAABB(partsBounds);
	m_world.AmpQueryAABB(partsBounds, [=, &fixtureBounds](int32 fixtureIdx)
	{
		Fixture& fixture = m_world.m_fixtureBuffer[fixtureIdx];
		if (fixture.m_isSensor) return;

		const b2Shape& shape = m_world.GetShape(fixture);
		int32 childCount = shape.GetChildCount();
		for (int32 childIdx = 0; childIdx < childCount; childIdx++)
			fixtureBounds.push_back(b2AABBFixtureProxy(m_world.GetAABB(fixture, childIdx), fixtureIdx, childIdx));
	});
	if (fixtureBounds.empty()) return;

	auto& groupIdxs = m_ampGroupIdxs;
	auto& groups = m_ampGroups;
	const auto& shouldCollide = [=, &groupIdxs, &groups](int32 i, const Fixture& f) restrict(amp) -> bool
	{
		if (f.m_filter.groupIndex >= 0)
			return true;
		const int32 groupIdx = groupIdxs[i];
		if (groupIdx == b2_invalidIndex)
			return false;
		const int32 partColGroup = groups[groupIdx].m_collisionGroup;
		if (!partColGroup || !(partColGroup < 0))
			return true;
		if (partColGroup != f.m_filter.groupIndex)
			return true;
		return false;
	};

	auto& contactCnts = m_localBodyContactCnts;
	amp::fill(contactCnts, 0);
	auto& localContacts = m_localBodyContacts;
	int32 newFixtureCnt = fixtureBounds.size();
	if (m_bodyContactFixtureCnt < newFixtureCnt)
	{
		m_bodyContactFixtureCnt = newFixtureCnt;
		amp::resize2ndDim(localContacts, newFixtureCnt);
	}

	//WaitForCopyBox2DToGPU();

	auto& chainShapes = m_ampChainShapes;
	auto& circleShapes = m_ampCircleShapes;
	auto& edgeShapes = m_ampEdgeShapes;
	auto& polygonShapes = m_ampPolygonShapes;
	const auto& computeDistance = [=, &chainShapes, &circleShapes, &edgeShapes, &polygonShapes]
		(const Fixture& f, const b2Transform& xf, const b2Vec2& p, float32& d, b2Vec2& n, int32 childIndex) restrict(amp)
	{
		switch (f.m_shapeType)
		{
		case b2Shape::e_chain:
			chainShapes[f.m_shapeIdx].ComputeDistance(xf, p, d, n, childIndex);
			break;
		case b2Shape::e_circle:
			circleShapes[f.m_shapeIdx].ComputeDistance(xf, p, d, n);
			break;
		case b2Shape::e_edge:
			edgeShapes[f.m_shapeIdx].ComputeDistance(xf, p, d, n);
			break;
		case b2Shape::e_polygon:
			polygonShapes[f.m_shapeIdx].ComputeDistance(xf, p, d, n);
			break;
		}
	};

	const float32 partDiameter = m_particleDiameter;
	const float32 invDiameter = m_inverseDiameter;
	const uint32 wallFlag = b2_wallParticle;
	auto& bodies = m_ampBodies;
	auto& fixtures = m_ampFixtures;
	auto& positions = m_ampPositions;
	auto& flags = m_ampFlags;
	auto& matIdxs = m_ampMatIdxs;
	auto& matInvMasses = m_ampMatInvMasses;
	AmpForEachInsideBounds(fixtureBounds, [=, &bodies, &fixtures, &positions, &flags, &matIdxs, &matInvMasses,
		&contactCnts, &localContacts](int32 i, int32 fixtureIdx, int32 childIdx) restrict(amp)
	{
		Fixture& fixture = fixtures[fixtureIdx];
		if (!shouldCollide(i, fixture)) return;
		
		const b2Vec2 ap = b2Vec2(positions[i]);
		float32 d;
		b2Vec2 n;
		int32 bIdx = fixture.m_bodyIdx;
		Body& b = bodies[bIdx];
		computeDistance(fixture, b.m_xf, ap, d, n, childIdx);
		if (d >= partDiameter) return;
		
		b2Vec2 bp = b.GetWorldCenter();
		float32 bm = b.m_mass;
		float32 bI =
			b.GetInertia() - bm * b.GetLocalCenter().LengthSquared();
		float32 invBm = bm > 0 ? 1 / bm : 0;
		float32 invBI = bI > 0 ? 1 / bI : 0;
		float32 invAm = flags[i] & wallFlag ? 0 : matInvMasses[matIdxs[i]];
		b2Vec2 rp = ap - bp;
		float32 rpn = b2Cross(rp, n);
		float32 invM = invAm + invBm + invBI * rpn * rpn;
		
		b2PartBodyContact& contact = localContacts[i][contactCnts[i]++];
		contact.partIdx = i;
		contact.bodyIdx = bIdx;
		contact.fixtureIdx = fixtureIdx;
		contact.weight = 1 - d * invDiameter;
		contact.normal = -n;
		contact.mass = invM > 0 ? 1 / invM : 0;
		//DetectStuckParticle(i); //TODO
	});

	auto& bodyContacts = m_ampBodyContacts;
	m_bodyContactCount = amp::reduce(contactCnts, m_count,
		[=, &contactCnts , &bodyContacts, &localContacts](const int32 i, const int32 wi) restrict(amp)
	{
		for (int32 j = 0; j < contactCnts[i]; j++)
			bodyContacts[wi + j] = localContacts[i][j];
	});
	
	if (m_def.strictContactCheck)
		RemoveSpuriousBodyContacts();
	//amp::copy(m_bodyContactPartIdxs, m_ampBodyContactPartIdxs, m_bodyContactCount);
}

void b2ParticleSystem::RemoveSpuriousBodyContacts()
{
	// At this point we have a list of contact candidates based on AABB
	// overlap.The AABB query that  generated this returns all collidable
	// fixtures overlapping particle bounding boxes.  This breaks down around
	// vertices where two shapes intersect, such as a "ground" surface made
	// of multiple b2PolygonShapes; it potentially applies a lot of spurious
	// impulses from normals that should not actually contribute.  See the
	// Ramp example in Testbed.
	//
	// To correct for this, we apply this algorithm:
	//   * sort contacts by particle and subsort by weight (nearest to farthest)
	//   * for each contact per particle:
	//      - project a point at the contact distance along the inverse of the
	//        contact normal
	//      - if this intersects the fixture that generated the contact, apply
	//         it, otherwise discard as impossible
	//      - repeat for up to n nearest contacts, currently we get good results
	//        from n=3.
	//std::sort(m_bodyContactBuffer.Begin(), m_bodyContactBuffer.End(),
	//			b2ParticleSystem::BodyContactCompare);
	/*parallel_sort(m_bodyContactBuffer.begin(), m_bodyContactBuffer.begin() + m_bodyContactCount,	TODO
		b2ParticleSystem::BodyContactCompare);

	int32 discarded = 0;
	std::remove_if(m_bodyContactBuffer.begin(),
				   m_bodyContactBuffer.begin() + m_bodyContactCount,
				   b2ParticleBodyContactRemovePredicate(this, &discarded));

	m_bodyContactCount -= discarded;*/
}

/*bool b2ParticleSystem::BodyContactCompare(int32 lhsIdx,
										  int32 lhsIdx)
{
	
	if (m_bodyContactIdxBuffer[lhsIdx] == m_bodyContactIdxBuffer[lhsIdx])
	{
		// Subsort by weight, decreasing.
		return m_bodyContactWeightBuffer[lhsIdx] > m_bodyContactWeightBuffer[rhsIdx];
	}
	return m_bodyContactIdxBuffer[lhsIdx] < m_bodyContactIdxBuffer[lhsIdx];
}*/

//void b2ParticleSystem::AddBodyContactResults(ampArray<float32> dst, const ampArray<float32> bodyRes)
//{
//	if (!m_bodyContactCount) return;
//	ampArrayView<const float32> add = bodyRes.section(0, m_bodyContactCount);
//	auto& bodyContactPartIdxs = m_ampBodyContactPartIdxs;
//	amp::forEach(m_bodyContactCount, [=, &dst, &bodyContactPartIdxs](const int32 i) restrict(amp)
//	{
//		amp::atomicAdd(dst[bodyContactPartIdxs[i]], add[i]);
//	});
//}
//void b2ParticleSystem::AddBodyContactResults(ampArray<b2Vec3> dst, const ampArray<b2Vec3> bodyRes)
//{
//	if (!m_bodyContactCount) return;
//	ampArrayView<const b2Vec3> add = bodyRes.section(0, m_bodyContactCount);
//	auto& bodyContactPartIdxs = m_ampBodyContactPartIdxs;
//	amp::forEach(m_bodyContactCount, [=, &dst, &bodyContactPartIdxs](const int32 i) restrict(amp)
//	{
//		amp::atomicAdd(dst[bodyContactPartIdxs[i]], add[i]);
//	});
//}

static inline bool IsSignificantForce(b2Vec3 force)
{
	return force.x != 0 || force.y != 0 || force.z != 0;
}
static inline bool IsSignificantForce(b2Vec3 force) restrict(amp)
{
	return force.x != 0 || force.y != 0 || force.z != 0;
}
static inline bool IsSignificantForce(b2Vec2 force) restrict(amp)
{
	return force.x != 0 || force.y != 0;
}

template <typename T>
ampArray<T> AmpCreateArrFromPtr(T* ptr, uint32 elemCnt)
{
	const std::vector<T> vec(ptr, elemCnt);
	return ampArray<T>(vec.begin(), vec.end());
}

// SolveCollision, SolveRigid and SolveWall should be called after
// other force functions because they may require particles to have
// specific velocities.
void b2ParticleSystem::SolveCollision()
{
	// This function detects particles which are crossing boundary of bodies
	// and modifies velocities of them so that they will move just in front of
	// boundary. This function also applies the reaction force to
	// bodies as precisely as the numerical stability is kept.
	const b2TimeStep& step = m_subStep;

	if (m_accelerate)
	{
		vector<b2AABBFixtureProxy> fixtureBounds;

		b2AABB aabb;
		AmpComputeAABB(aabb, true);
		m_world.AmpQueryAABB(aabb, [=, &fixtureBounds](int32 fixtureIdx)
		{
			Fixture& fixture = m_world.m_fixtureBuffer[fixtureIdx];
			if (fixture.m_isSensor) return;

			const b2Shape& shape = m_world.GetShape(fixture);
			int32 childCount = shape.GetChildCount();

			for (int32 childIdx = 0; childIdx < childCount; childIdx++)
				fixtureBounds.push_back(b2AABBFixtureProxy(m_world.GetAABB(fixture, childIdx), fixtureIdx, childIdx));
		});

		auto& positions = m_ampPositions;
		auto& groupIdxs = m_ampGroupIdxs;
		auto& groups    = m_ampGroups;
		const auto& shouldCollide = [=, &groupIdxs, &groups](const Fixture& f, int32 i) restrict(amp) -> bool
		{
			if (f.m_filter.groupIndex >= 0)
				return true;
			const int32 groupIdx = groupIdxs[i];
			if (groupIdx == -1)
				return false;
			const int32 partColGroup = groups[groupIdx].m_collisionGroup;
			if (!partColGroup || !(partColGroup < 0))
				return true;
			if (partColGroup != f.m_filter.groupIndex)
				return true;
			return false;
		};

		auto& chainShapes = m_ampChainShapes;
		auto& circleShapes = m_ampCircleShapes;
		auto& edgeShapes = m_ampEdgeShapes;
		auto& polygonShapes = m_ampPolygonShapes;
		const auto& rayCast = [=, &chainShapes, &circleShapes, &edgeShapes, &polygonShapes](const Fixture& f,
			b2RayCastOutput& output, const b2RayCastInput& input, const b2Transform& xf, int32 childIdx) restrict(amp) -> bool
		{
			switch (f.m_shapeType)
			{
			case b2Shape::e_chain:
				return chainShapes[f.m_shapeIdx].RayCast(output, input, xf, childIdx);
			case b2Shape::e_circle:
				return circleShapes[f.m_shapeIdx].RayCast(output, input, xf);
			case b2Shape::e_edge:
				return edgeShapes[f.m_shapeIdx].RayCast(output, input, xf);
			case b2Shape::e_polygon:
				return polygonShapes[f.m_shapeIdx].RayCast(output, input, xf);
			}
			return false;
		};

		const uint32 wallFlag = b2_wallParticle;
		auto& flags = m_ampFlags;
		auto& forces = m_ampForces;
		const auto& particleApplyForce = [=, &flags, &forces](int32 index, const b2Vec2& force) restrict(amp)
		{
			if (IsSignificantForce(force) &&
				!(flags[index] & wallFlag))
			{
				amp::atomicAdd(forces[index], force);
			}
		};

		const int32 iteration = m_iteration;
		const float32 stepInvDt = m_step.inv_dt;
		const float32 stepDt = m_step.dt;
		const float32 linearSlop = b2_linearSlop;

		auto& bodies = m_ampBodies;
		auto& fixtures = m_ampFixtures;
		auto& velocities = m_ampVelocities;
		auto& matIdxs = m_ampMatIdxs;
		auto& matMasses = m_ampMatMasses;
		AmpForEachInsideBounds(fixtureBounds, [=, &bodies, &fixtures, &positions, &velocities,
			&matIdxs, &matMasses](int32 a, int32 fixtureIdx, int32 childIdx) restrict(amp)
		{
			const Fixture& fixture = fixtures[fixtureIdx];
			if (!fixture.TestZPos(positions[a].z) || !shouldCollide(fixture, a)) return;
			const b2Vec2& ap = positions[a];
			const Body& body = bodies[fixture.m_bodyIdx];
		
			b2Vec3& av = velocities[a];
			b2RayCastOutput output;
			b2RayCastInput input;
			if (iteration == 0)
			{
				// Put 'ap' in the local space of the previous frame
				b2Vec2 p1 = b2MulT(body.m_xf0, ap);
				if (fixture.m_shapeType == b2Shape::e_circle)
				{
					// Make relative to the center of the circle
					p1 -= body.GetLocalCenter();
					// Re-apply rotation about the center of the
					// circle
					p1 = b2Mul(body.m_xf0.q, p1);
					// Subtract rotation of the current frame
					p1 = b2MulT(body.m_xf.q, p1);
					// Return to local space
					p1 += body.GetLocalCenter();
				}
				// Return to global space and apply rotation of current frame
				input.p1 = b2Mul(body.m_xf, p1);
			}
			else
				input.p1 = ap;
			input.p2 = ap + stepDt * b2Vec2(av);
			input.maxFraction = 1;
			if (!rayCast(fixture, output, input, body.m_xf, childIdx)) return;
			const b2Vec2& n = output.normal;
			const b2Vec2 p =
				(1 - output.fraction) * input.p1 +
				output.fraction * input.p2 +
				b2_linearSlop * n;
			const b2Vec2 v = stepInvDt * (p - ap);
			av.x = v.x;
			av.y = v.y;
			const b2Vec2 f = stepInvDt *
				matMasses[matIdxs[a]] * (b2Vec2(av) - v);
			particleApplyForce(a, f);
		});
	}
	else
	{
		b2AABB aabb;
		aabb.lowerBound.x = +b2_maxFloat;
		aabb.lowerBound.y = +b2_maxFloat;
		aabb.upperBound.x = -b2_maxFloat;
		aabb.upperBound.y = -b2_maxFloat;
		for (int32 i = 0; i < m_count; i++)
		{
			b2Vec2 v = b2Vec2(m_velocityBuffer[i]);
			b2Vec2 p1 = b2Vec2(m_positionBuffer[i]);
			b2Vec2 p2 = p1 + step.dt * v;
			aabb.lowerBound = b2Min(aabb.lowerBound, b2Min(p1, p2));
			aabb.upperBound = b2Max(aabb.upperBound, b2Max(p1, p2));
		}
		class SolveCollisionCallback : public b2FixtureParticleQueryCallback
		{
			void ReportFixtureAndParticle(int32 fixtureIdx, int32 childIndex, int32 a)
			{
				Fixture& fixture = m_world.m_fixtureBuffer[fixtureIdx];
				if (m_system.ShouldCollide(a, fixture))
				{
					Body& body = m_world.m_bodyBuffer[fixture.m_bodyIdx];
					b2Vec2 ap = b2Vec2(m_system.m_positionBuffer[a]);
					b2Vec2 av = b2Vec2(m_system.m_velocityBuffer[a]);
					b2RayCastOutput output;
					b2RayCastInput input;
					if (m_system.m_iteration == 0)
					{
						// Put 'ap' in the local space of the previous frame
						b2Vec2 p1 = b2MulT(body.m_xf0, ap);
						;
						if (fixture.m_shapeType == b2Shape::e_circle)
						{
							// Make relative to the center of the circle
							p1 -= body.GetLocalCenter();
							// Re-apply rotation about the center of the
							// circle
							p1 = b2Mul(body.m_xf0.q, p1);
							// Subtract rotation of the current frame
							p1 = b2MulT(body.m_xf.q, p1);
							// Return to local space
							p1 += body.GetLocalCenter();
						}
						// Return to global space and apply rotation of current frame
						input.p1 = b2Mul(body.m_xf, p1);
					}
					else
					{
						input.p1 = ap;
					}
					input.p2 = ap + m_step.dt * av;
					input.maxFraction = 1;
					if (m_world.RayCast(fixture, output, input, childIndex))
					{
						b2Vec2 n = output.normal;
						b2Vec2 p =
							(1 - output.fraction) * input.p1 +
							output.fraction * input.p2 +
							b2_linearSlop * n;
						b2Vec2 v = m_step.inv_dt * (p - ap);
						m_system.m_velocityBuffer[a] = b2Vec3(v);
						b2Vec2 f = m_step.inv_dt *
							m_system.m_partMatMassBuf[m_system.m_partMatIdxBuffer[a]] * (av - v);
						m_system.ParticleApplyForce(a, f);
					}
				}
			}

			b2TimeStep m_step;
			b2ContactFilter* m_contactFilter;

		public:
			SolveCollisionCallback(b2World& world,
				b2ParticleSystem& system, const b2TimeStep& step, b2ContactFilter* contactFilter) :
				b2FixtureParticleQueryCallback(world, system)
			{
				m_step = step;
				m_contactFilter = contactFilter;
			}
		} callback(m_world, *this, step, GetFixtureContactFilter());
		m_world.QueryAABB(&callback, aabb);
	}
}

void b2ParticleSystem::SolveGroundCollision()
{
	if (m_accelerate)
	{
		const uint32 waterFlag = b2_waterParticle;
		const uint32 zombieFlag = b2_zombieParticle;
		const int32 txMax = m_world.m_ground->m_sizeX;
		const int32 tyMax = m_world.m_ground->m_sizeY;
		const float32 invStride = 1.0f / m_world.m_ground->m_stride;
		const float32 partRadius = GetRadius();
		auto& positions = m_ampPositions;
		auto& velocities = m_ampVelocities;
		auto& flags = m_ampFlags;
		auto& groundTiles = m_world.m_ground->m_ampTiles;
		auto& groundTilesTileHasChange = m_world.m_ground->m_ampTilesTileHasChange;
		auto& groundMats = m_world.m_ground->m_ampMaterials;
		AmpForEachParticle([=, &positions, &velocities, &flags,
			&groundTiles, &groundTilesTileHasChange, &groundMats](const int32 i) restrict(amp)
		{
			b2Vec3& p = positions[i];
			int32 tx = p.x * invStride;
			int32 ty = p.y * invStride;
			if (tx < 0 || tx >= txMax || ty < 0 || ty >= tyMax) return;

			Ground::Tile& gt = groundTiles[ty * txMax + tx];
			const float32 height = gt.height + partRadius;
			if (p.z > height) return;
			p.z = height;
			const Ground::Material& groundMat = groundMats[gt.matIdx];
			if (uint32& f = flags[i]; f & waterFlag)
			{
				if (!groundMat.isWaterRepellent() && gt.atomicSetWet())
				{
					f |= zombieFlag;
					groundTilesTileHasChange[ty / TILE_SIZE_SQRT][tx / TILE_SIZE_SQRT] = 1;
				}
				return;
			}
			// Solve Friction
			b2Vec3& v = velocities[i];
			if (groundMat.friction)
			{
				const float32 frictionFactor = 1 - groundMat.friction;
				v.x *= frictionFactor;
				v.y *= frictionFactor;
			}

			// Solve Bounciness
			if (v.z < 0)
				v.z *= -1 * groundMat.bounciness;
		});
	}
	else
	{

	}
	if (m_world.m_ground->m_allMaterialFlags & Ground::Material::Flags::waterRepellent &&
		m_allParticleFlags & b2_waterParticle)
		m_allParticleFlags |= b2_zombieParticle;
}

void b2ParticleSystem::SolveBarrier()
{
	// If a particle is passing between paired barrier particles,
	// its velocity will be decelerated to avoid passing.
	if (!(m_allParticleFlags & b2_barrierParticle)) return;

	const b2TimeStep& step = m_subStep;
	const uint32 filterFlags = b2_barrierParticle | b2_wallParticle;
	const uint32 barrierFilterFlag = b2_barrierParticle;
	const uint32 wallFilterFlag = b2_wallParticle;
	const uint32 zombieFlag = b2_zombieParticle;

	if (m_accelerate)
	{
		const int32 cnt = m_count;
		auto& flags = m_ampFlags;
		auto& velocities = m_ampVelocities;
		AmpForEachParticle([=, &flags, &velocities](const int32 i) restrict(amp)
		{
			if ((flags[i] & filterFlags) == filterFlags)
				velocities[i].SetZero();
		});
		const float32 tmax = b2_barrierCollisionTime * step.dt;
		auto& pairs = m_ampPairs;
		auto& positions = m_ampPositions;
		auto& groupIdxs = m_ampGroupIdxs;
		auto& matIdxs = m_ampMatIdxs;
		auto& groups = m_ampGroups;

		const int32 timeStamp = m_timestamp;
		auto&  matMasses = m_ampMatMasses;
		const auto UpdateStatistics = [=, &positions, &velocities, &matMasses, &flags](const b2ParticleGroup & group) restrict(amp)
		{
			if (group.m_timestamp == timeStamp) return;
			const float32 m = matMasses[group.m_matIdx];
			const int32& firstIdx = group.m_firstIndex;
			const int32& lastIdx = group.m_lastIndex;
			float32& mass = group.m_mass = 0;
			b2Vec2& center = group.m_center;
			b2Vec2& linVel = group.m_linearVelocity;
			center.SetZero();
			linVel.SetZero();
			for (int32 i = firstIdx; i < lastIdx; i++)
			{
				if (flags[i] & zombieFlag) continue;
				mass += m;
				center += m * (b2Vec2)positions[i];
				linVel += m * (b2Vec2)velocities[i];
			}
			if (mass > 0)
			{
				center *= 1 / mass;
				linVel *= 1 / mass;
			}
			float32& inertia = group.m_inertia = 0;
			float32& angVel = group.m_angularVelocity = 0;
			for (int32 i = firstIdx; i < lastIdx; i++)
			{
				if (flags[i] & zombieFlag) continue;
				b2Vec2 p = (b2Vec2)positions[i] - center;
				b2Vec2 v = (b2Vec2)velocities[i] - linVel;
				inertia += m * b2Dot(p, p);
				angVel += m * b2Cross(p, v);
			}
			if (inertia > 0)
			{
				angVel *= 1 / inertia;
			}
			group.m_timestamp = timeStamp;
		};

		const uint32 rigidGroupFlag = b2_rigidParticleGroup;
		const auto GetLinearVelocity = [=, &velocities](
			const b2ParticleGroup & group, int32 particleIndex,
			const b2Vec2 & point) restrict(amp) -> b2Vec2
		{
			if (group.m_groupFlags & rigidGroupFlag)
			{
				UpdateStatistics(group);
				return group.m_linearVelocity + b2Cross(group.m_angularVelocity, point - group.m_center);
			}
			else
				return velocities[particleIndex];
		};
		auto& proxies = m_ampProxies;
		const auto TagLowerBound = [=, &proxies](uint32 first, uint32 last, uint32 tag) restrict(amp) -> int32
		{
			int32 i, step;
			int32 count = last - first;

			while (count > 0) {
				i = first;
				step = count / 2;
				i += step;
				if (proxies[i].tag < tag) {
					first = ++i;
					count -= step + 1;
				}
				else
					count = step;
			}
			return first;
		};
		const auto TagUpperBound = [=, &proxies](uint32 first, uint32 last, uint32 tag) restrict(amp) -> int32
		{
			int32 i, step;
			int32 count = last - first;

			while (count > 0) {
				i = first;
				step = count / 2;
				i += step;
				if (!(proxies[i].tag < tag)) {
					first = ++i;
					count -= step + 1;
				}
				else
					count = step;
			}
			return first;
		};

		struct AmpInsideBoundsEnumerator
		{
			/// Construct an enumerator with bounds of tags and a range of proxies.
			AmpInsideBoundsEnumerator(
				uint32 lower, uint32 upper,
				int32 first, int32 last) restrict(amp)
			{
				m_xLower = lower & xMask;
				m_xUpper = upper & xMask;
				m_yLower = lower & yMask;
				m_yUpper = upper & yMask;
				m_first = first;
				m_last = last;
			};
			/// The lower and upper bound of x component in the tag.
			uint32 m_xLower, m_xUpper;
			/// The lower and upper bound of y component in the tag.
			uint32 m_yLower, m_yUpper;
			/// The range of proxies.
			int32 m_first;
			int32 m_last;
		};

		const float32 invDiameter = m_inverseDiameter;
		const auto GetInsideBoundsEnumerator = [=](const b2AABB & aabb)
			restrict(amp) -> AmpInsideBoundsEnumerator
		{
			uint32 lowerTag = computeTag(invDiameter * aabb.lowerBound.x - 1,
				invDiameter * aabb.lowerBound.y - 1);
			uint32 upperTag = computeTag(invDiameter * aabb.upperBound.x + 1,
				invDiameter * aabb.upperBound.y + 1);

			const int32 first = TagLowerBound(0, cnt, lowerTag);
			const int32 last  = TagUpperBound(first, cnt, upperTag);

			return AmpInsideBoundsEnumerator(lowerTag, upperTag, first, last);
		};

		const auto GetNext = [=, &proxies](AmpInsideBoundsEnumerator& ibe) restrict(amp) -> int32
		{
			while (ibe.m_first < ibe.m_last)
			{
				uint32 xTag = proxies[ibe.m_first].tag & xMask;
				if (xTag >= ibe.m_xLower && xTag <= ibe.m_xUpper)
				{
					return proxies[ibe.m_first++].idx;
				}
				ibe.m_first++;
			}
			return -1;
		};

		auto& forces = m_ampForces;
		AmpForEachPair([=, &pairs, &flags, &groups, &positions, &velocities,
			&groupIdxs, &matIdxs, &matMasses, &forces](const int32 i) restrict(amp)
		{
			const b2ParticlePair& pair = pairs[i];
			if (!(pair.flags & barrierFilterFlag)) return;
			const int32 a = pair.indexA;
			const int32 b = pair.indexB;
			const b2Vec2 pa = positions[a];
			const b2Vec2 pb = positions[b];
			b2AABB aabb;
			aabb.lowerBound = b2Min(pa, pb);
			aabb.upperBound = b2Max(pa, pb);
			const int32 aGroupIdx = groupIdxs[a];
			const int32 bGroupIdx = groupIdxs[b];
			b2ParticleGroup& aGroup = groups[aGroupIdx];
			b2ParticleGroup& bGroup = groups[bGroupIdx];
			const b2Vec2 va = GetLinearVelocity(aGroup, a, pa);
			const b2Vec2 vb = GetLinearVelocity(bGroup, b, pb);
			const b2Vec2 pba = pb - pa;
			const b2Vec2 vba = vb - va;
			AmpInsideBoundsEnumerator& enumerator = GetInsideBoundsEnumerator(aabb);
			int32 c;
			while ((c = GetNext(enumerator)) >= 0)
			{
				b2Vec2 pc = positions[c];
				int32 cGroupIdx = groupIdxs[c];
				if (aGroupIdx != cGroupIdx && bGroupIdx != cGroupIdx)
				{
					b2ParticleGroup& cGroup = groups[cGroupIdx];
					const b2Vec2 vc = GetLinearVelocity(cGroup, c, pc);
					// Solve the equation below:
					//   (1-s)*(pa+t*va)+s*(pb+t*vb) = pc+t*vc
					// which expresses that the particle c will pass a line
					// connecting the particles a and b at the time of t.
					// if s is between 0 and 1, c will pass between a and b.
					const b2Vec2 pca = pc - pa;
					const b2Vec2 vca = vc - va;
					const float32 e2 = b2Cross(vba, vca);
					const float32 e1 = b2Cross(pba, vca) - b2Cross(pca, vba);
					const float32 e0 = b2Cross(pba, pca);
					float32 s, t;
					b2Vec2 qba, qca;
					if (e2 == 0)
					{
						if (e1 == 0) continue;
						t = -e0 / e1;
						if (!(t >= 0 && t < tmax)) continue;
						qba = pba + t * vba;
						qca = pca + t * vca;
						s = b2Dot(qba, qca) / b2Dot(qba, qba);
						if (!(s >= 0 && s <= 1)) continue;
					}
					else
					{
						const float32 det = e1 * e1 - 4 * e0 * e2;
						if (det < 0) continue;
						const float32 sqrtDet = ampSqrt(det);
						float32 t1 = (-e1 - sqrtDet) / (2 * e2);
						float32 t2 = (-e1 + sqrtDet) / (2 * e2);
						if (t1 > t2) b2Swap(t1, t2);
						t = t1;
						qba = pba + t * vba;
						qca = pca + t * vca;
						s = b2Dot(qba, qca) / b2Dot(qba, qba);
						if (!(t >= 0 && t < tmax && s >= 0 && s <= 1))
						{
							t = t2;
							if (!(t >= 0 && t < tmax)) continue;
							qba = pba + t * vba;
							qca = pca + t * vca;
							s = b2Dot(qba, qca) / b2Dot(qba, qba);
							if (!(s >= 0 && s <= 1)) continue;
						}
					}
					// Apply a force to particle c so that it will have the
					// interpolated velocity at the collision point on line ab.
					const b2Vec2 dv = va + s * vba - vc;
					const b2Vec2 f = matMasses[matIdxs[c]] * dv;
					if (cGroup.m_groupFlags & rigidGroupFlag)
					{
						// If c belongs to a rigid group, the force will be
						// distributed in the group->
						const float32 mass = cGroup.m_mass;
						const float32 inertia = cGroup.m_inertia;
						if (mass > 0)
						{
							cGroup.m_linearVelocity += 1 / mass * f;
						}
						if (inertia > 0)
							cGroup.m_angularVelocity +=
							b2Cross(pc - cGroup.m_center, f) / inertia;
					}
					else
					{
						amp::atomicAdd(velocities[c], dv);
					}
					// Apply a reversed force to particle c after particle
					// movement so that momentum will be preserved.
					b2Vec2 force = -step.inv_dt * f;
					if (IsSignificantForce(force) && flags[c] & wallFilterFlag)
						amp::atomicAdd(forces[c], force);
				}
			}
		});
	}
	else
	{
		for (int32 i = 0; i < m_count; i++)
		{
			uint32 flags = m_flagsBuffer[i];
			static const uint32 k_barrierWallFlags =
				b2_barrierParticle | b2_wallParticle;
			if ((flags & k_barrierWallFlags) == k_barrierWallFlags)
			{
				m_velocityBuffer[i].SetZero();
			}
		}
		float32 tmax = b2_barrierCollisionTime * step.dt;
		for (int32 k = 0; k < m_pairCount; k++)
		{
			const b2ParticlePair& pair = m_pairBuffer[k];
			if (pair.flags & b2_barrierParticle)
			{
				int32 a = pair.indexA;
				int32 b = pair.indexB;
				b2Vec2 pa = b2Vec2(m_positionBuffer[a]);
				b2Vec2 pb = b2Vec2(m_positionBuffer[b]);
				b2AABB aabb;
				aabb.lowerBound = b2Min(pa, pb);
				aabb.upperBound = b2Max(pa, pb);
				int32 aGroupIdx = m_partGroupIdxBuffer[a];
				int32 bGroupIdx = m_partGroupIdxBuffer[b];
				b2ParticleGroup& aGroup = m_groupBuffer[aGroupIdx];
				b2ParticleGroup& bGroup = m_groupBuffer[bGroupIdx];
				b2Vec2 va = GetLinearVelocity(aGroup, a, pa);
				b2Vec2 vb = GetLinearVelocity(bGroup, b, pb);
				b2Vec2 pba = pb - pa;
				b2Vec2 vba = vb - va;
				InsideBoundsEnumerator enumerator = GetInsideBoundsEnumerator(aabb);
				int32 c;
				while ((c = enumerator.GetNext()) >= 0)
				{
					b2Vec2 pc = b2Vec2(m_positionBuffer[c]);
					int32 cGroupIdx = m_partGroupIdxBuffer[c];
					if (aGroupIdx != cGroupIdx && bGroupIdx != cGroupIdx)
					{
						b2ParticleGroup& cGroup = m_groupBuffer[cGroupIdx];
						b2Vec2 vc = GetLinearVelocity(cGroup, c, pc);
						// Solve the equation below:
						//   (1-s)*(pa+t*va)+s*(pb+t*vb) = pc+t*vc
						// which expresses that the particle c will pass a line
						// connecting the particles a and b at the time of t.
						// if s is between 0 and 1, c will pass between a and b.
						b2Vec2 pca = pc - pa;
						b2Vec2 vca = vc - va;
						float32 e2 = b2Cross(vba, vca);
						float32 e1 = b2Cross(pba, vca) - b2Cross(pca, vba);
						float32 e0 = b2Cross(pba, pca);
						float32 s, t;
						b2Vec2 qba, qca;
						if (e2 == 0)
						{
							if (e1 == 0) continue;
							t = -e0 / e1;
							if (!(t >= 0 && t < tmax)) continue;
							qba = pba + t * vba;
							qca = pca + t * vca;
							s = b2Dot(qba, qca) / b2Dot(qba, qba);
							if (!(s >= 0 && s <= 1)) continue;
						}
						else
						{
							float32 det = e1 * e1 - 4 * e0 * e2;
							if (det < 0) continue;
							float32 sqrtDet = b2Sqrt(det);
							float32 t1 = (-e1 - sqrtDet) / (2 * e2);
							float32 t2 = (-e1 + sqrtDet) / (2 * e2);
							if (t1 > t2) b2Swap(t1, t2);
							t = t1;
							qba = pba + t * vba;
							qca = pca + t * vca;
							s = b2Dot(qba, qca) / b2Dot(qba, qba);
							if (!(t >= 0 && t < tmax && s >= 0 && s <= 1))
							{
								t = t2;
								if (!(t >= 0 && t < tmax)) continue;
								qba = pba + t * vba;
								qca = pca + t * vca;
								s = b2Dot(qba, qca) / b2Dot(qba, qba);
								if (!(s >= 0 && s <= 1)) continue;
							}
						}
						// Apply a force to particle c so that it will have the
						// interpolated velocity at the collision point on line ab.
						b2Vec2 dv = va + s * vba - vc;
						b2Vec2 f = m_partMatMassBuf[m_partMatIdxBuffer[c]] * dv;
						if (IsRigidGroup(cGroup))
						{
							// If c belongs to a rigid group, the force will be
							// distributed in the group->
							float32 mass = GetMass(cGroup);
							float32 inertia = GetInertia(cGroup);
							if (mass > 0)
							{
								cGroup.m_linearVelocity += 1 / mass * f;
							}
							if (inertia > 0)
								cGroup.m_angularVelocity +=
								b2Cross(pc - GetCenter(cGroup), f) / inertia;
						}
						else
						{
							m_velocityBuffer[c] += dv;
						}
						// Apply a reversed force to particle c after particle
						// movement so that momentum will be preserved.
						b2Vec2 force = -step.inv_dt * f;
						ParticleApplyForce(c, force);
					}
				}
			}
		}
	}
}

bool b2ParticleSystem::ShouldSolve()
{
	if (!m_world.m_stepComplete || m_count == 0 || m_step.dt <= 0.0f || m_paused)
		return false;
	return true;
}

void b2ParticleSystem::SolveInit() 
{

	//if (!m_expireTimeBuf.empty())
	//	SolveLifetimes(m_step);
	m_iteration = 0;
	if (m_accelerate)
	{
		CopyBox2DToGPUAsync();
		if (m_needsUpdateAllParticleFlags)
			AmpUpdateAllParticleFlags();
	}
	else
	{
		if (m_allParticleFlags & b2_zombieParticle)
			SolveZombie();
		if (m_needsUpdateAllParticleFlags)
			UpdateAllParticleFlags();
	}
	if (m_needsUpdateAllGroupFlags)
		UpdateAllGroupFlags();

}

void b2ParticleSystem::InitStep()
{
	++m_timestamp;
	m_subStep = m_step;
	m_subStep.dt /= m_step.particleIterations;
	m_subStep.inv_dt *= m_step.particleIterations;
}

void b2ParticleSystem::UpdateContacts(bool exceptZombie)
{
	if (m_accelerate)
	{
		AmpUpdateProxies();
		AmpSortProxies();
		m_futureUpdateBodyContacts = std::async(launch::async, &b2ParticleSystem::AmpUpdateBodyContacts, this);
		AmpFindContacts(exceptZombie);
	}
	else
	{
		UpdateProxies();
		SortProxies();
		FindContacts();
		UpdateBodyContacts();
		if (exceptZombie)
			RemoveFromVectorIf(m_partContactBuf, m_contactCount, b2ParticleContactIsZombie, true);
	}
}


void b2ParticleSystem::SolveEnd() 
{
	if (m_accelerate && m_count)
	{
		if (m_allParticleFlags & b2_zombieParticle)
			AmpSolveZombie();
		m_ampCopyFutBodies.wait();
		m_ampCopyFutWeights.wait();
		m_ampCopyFutVelocities.wait();
		m_ampCopyFutHealths.wait();
		m_ampCopyFutHeats.wait();
		m_ampCopyFutMatIdxs.wait();
		m_ampCopyFutFlags.wait();
		m_ampCopyFutPositions.wait();
	}
}


float32 b2ParticleSystem::GetTimeDif(Time start, Time end)
{
	return std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / 1000000.0f;
}
float32 b2ParticleSystem::GetTimeDif(Time start)
{
	return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start).count() / 1000000.0f;
}

void b2ParticleSystem::UpdateAllParticleFlags()
{
	m_allParticleFlags = 0;
	for (int32 i = 0; i < m_count; i++)
	{
		m_allParticleFlags |= m_flagsBuffer[i];
	}
	m_needsUpdateAllParticleFlags = false;
}
void b2ParticleSystem::AmpUpdateAllParticleFlags()
{
	m_allParticleFlags = amp::reduceFlags(m_ampFlags, m_count);
	m_needsUpdateAllParticleFlags = false;
}

void b2ParticleSystem::UpdateAllGroupFlags()
{
	m_allGroupFlags = 0;
	for (int i = 0; i < m_groupCount; i++)
	{
		if (m_groupBuffer[i].m_firstIndex != b2_invalidIndex)
			m_allGroupFlags |= m_groupBuffer[i].m_groupFlags;
	}
	m_needsUpdateAllGroupFlags = false;
}

void b2ParticleSystem::LimitVelocity()
{
	const b2TimeStep& step = m_subStep;
	const float32 criticalVelocitySquared = GetCriticalVelocitySquared(step);
	if (m_accelerate)
	{
		auto& velocities = m_ampVelocities;
		AmpForEachParticle([=, &velocities](const int32 i) restrict(amp)
		{
			const b2Vec3 v = velocities[i];
			const float32 v2 = b2Dot(v, v);
			if (v2 <= criticalVelocitySquared) return;
			const float32 s = ampSqrt(criticalVelocitySquared / v2);
			velocities[i] *= s;
		});
	}
	else
	{
		for (int32 i = 0; i < m_count; i++)
		{
			b2Vec2 v = b2Vec2(m_velocityBuffer[i]);
			float32 v2 = b2Dot(v, v);
			if (v2 > criticalVelocitySquared)
			{
				float32 s = b2Sqrt(criticalVelocitySquared / v2);
				m_velocityBuffer[i] *= s;
			}
		}
	}
}

void b2ParticleSystem::SolveGravity()
{
	const b2Vec3 gravity = m_subStep.dt * m_def.gravityScale * m_world.GetGravity();
	const float32 atmosphericDensity = m_world.m_atmosphericDensity;
	//const uint32 controlledFlag = b2_controlledParticle;

	if (m_accelerate)
	{
		auto& flags = m_ampFlags;
		auto& velocities = m_ampVelocities;
		auto& matIdxs = m_ampMatIdxs;
		auto& matMasses = m_ampMatMasses;
		AmpForEachParticle([=, &flags, &matIdxs, &matMasses, &velocities](const int32 i) restrict(amp)
		{
			// if (flags[i] & controlledFlag) return;
			velocities[i] += (matMasses[matIdxs[i]] - atmosphericDensity) * gravity;
		});
	}
	else
	{
		for (int32 i = 0; i < m_count; i++)
		{
			//if (!(m_flagsBuffer[i] & controlledFlag))
				m_velocityBuffer[i] += (m_partMatMassBuf[m_partMatIdxBuffer[i]] - atmosphericDensity) * gravity;
		}
	}
}

void b2ParticleSystem::SolveStaticPressure()
{
	/// Compute pressure satisfying the modified Poisson equation:
	///     Sum_for_j((p_i - p_j) * w_ij) + relaxation * p_i =
	///     pressurePerWeight * (w_i - b2_minParticleWeight)
	/// by iterating the calculation:
	///     p_i = (Sum_for_j(p_j * w_ij) + pressurePerWeight *
	///           (w_i - b2_minParticleWeight)) / (w_i + relaxation)
	/// where
	///     p_i and p_j are static pressure of particle i and j
	///     w_ij is contact weight between particle i and j
	///     w_i is sum of contact weight of particle i
	if (!(m_allParticleFlags & b2_staticPressureParticle)) return;
	if (!m_contactCount) return;

	const b2TimeStep& step = m_subStep;
	const float32 criticalPressure = GetCriticalPressure(step);
	const float32 pressurePerWeight = m_def.staticPressureStrength * criticalPressure;
	const float32 maxPressure = b2_maxParticlePressure * criticalPressure;
	const float32 relaxation = m_def.staticPressureRelaxation;
	const float32 minWeight = b2_minParticleWeight;

	if (m_accelerate)
	{
		AmpRequestBuffer(m_ampStaticPressures, hasStaticPressureBuf);
		const uint32 filterFlag = b2_staticPressureParticle;
		auto& staticPressures = m_ampStaticPressures;
		auto& accumulations = m_ampAccumulations;
		auto& weights = m_ampWeights;
		auto& flags = m_ampFlags;
		for (int32 t = 0; t < m_def.staticPressureIterations; t++)
		{
			amp::fill(accumulations, 0.0f, m_count);
			AmpForEachContact([=, &accumulations,
				&staticPressures](const b2ParticleContact& contact) restrict(amp)
			{
				if (!(contact.flags & filterFlag)) return;
				int32 a = contact.idxA;
				int32 b = contact.idxB;
				float32 w = contact.weight;
				amp::atomicAdd(accumulations[a], w * staticPressures[b]);	// a <- b
				amp::atomicAdd(accumulations[b], w * staticPressures[a]);	// b <- a
			});
			AmpForEachParticle([=, &flags, &weights, &accumulations,
				&staticPressures](const int32 i) restrict(amp)
			{
				const float32 w = weights[i];
				if (flags[i] & b2_staticPressureParticle)
				{
					const float32 wh = accumulations[i];
					const float32 h =
						(wh + pressurePerWeight * (w - minWeight)) /
						(w + relaxation);
					staticPressures[i] = b2Clamp(h, 0.0f, maxPressure);
				}
				else
					staticPressures[i] = 0;
			});
		}
	}
	else
	{
		RequestBuffer(m_staticPressureBuf, hasStaticPressureBuf);
		for (int32 t = 0; t < m_def.staticPressureIterations; t++)
		{
			memset(m_accumulationBuf.data(), 0,
				sizeof(*m_accumulationBuf.data()) * m_count);
			for (int32 k = 0; k < m_contactCount; k++)
			{
				const b2ParticleContact& contact = m_partContactBuf[k];
				if (contact.flags & b2_staticPressureParticle)
				{
					int32 a = contact.idxA;
					int32 b = contact.idxB;
					float32 w = contact.weight;
					m_accumulationBuf[a] +=
						w * m_staticPressureBuf[b]; // a <- b
					m_accumulationBuf[b] +=
						w * m_staticPressureBuf[a]; // b <- a
				}
			}
			for (int32 i = 0; i < m_count; i++)
			{
				float32 w = m_weightBuffer[i];
				if (m_flagsBuffer[i] & b2_staticPressureParticle)
				{
					float32 wh = m_accumulationBuf[i];
					float32 h =
						(wh + pressurePerWeight * (w - b2_minParticleWeight)) /
						(w + relaxation);
					m_staticPressureBuf[i] = b2Clamp(h, 0.0f, maxPressure);
				}
				else
				{
					m_staticPressureBuf[i] = 0;
				}
			}
		}
	}

	
}

void b2ParticleSystem::SolvePressure()
{
	const b2TimeStep& step = m_subStep;
	const float32 minWeight = b2_minParticleWeight;

	// calculates pressure as a linear function of density
	const float32 criticalPressure = GetCriticalPressure(step);
	const float32 pressurePerWeight = m_def.pressureStrength * criticalPressure;
	const float32 maxPressure = b2_maxParticlePressure * criticalPressure;
	const float32 velocityPerPressure = step.dt / (m_def.density * m_particleDiameter);

	if (m_accelerate)
	{
		auto& weights = m_ampWeights;
		auto& accumulations = m_ampAccumulations;

		AmpForEachParticle([=, &weights, &accumulations](const int32 i) restrict(amp)
		{
			const float32 w = weights[i];
			const float32 h = pressurePerWeight * b2Max(0.0f, w - minWeight);
			accumulations[i] = b2Min(h, maxPressure);
		});
		// ignores particles which have their own repulsive force
		if (m_allParticleFlags & k_noPressureFlags)
		{
			AmpForEachParticle(k_noPressureFlags, [=, &accumulations](const int32 i) restrict(amp)
			{
				accumulations[i] = 0;
			});
		}
		// static pressure
		if (m_allParticleFlags & b2_staticPressureParticle)
		{
			auto& staticPressures = m_ampStaticPressures;
			AmpForEachParticle(b2_staticPressureParticle, [=, &accumulations, &staticPressures](const int32 i) restrict(amp)
			{
				accumulations[i] += staticPressures[i];
			});
		}

		// applies pressure between each particles in contact
		auto& velocities = m_ampVelocities;
		auto& matMasses = m_ampMatMasses;
		auto& matIdxs = m_ampMatIdxs;
		auto& bodies = m_ampBodies;
		auto& positions = m_ampPositions;
		auto& matInvMasses = m_ampMatInvMasses;
		AmpForEachBodyContact([=, &bodies, &positions, &accumulations,
			&velocities, &matIdxs, &matInvMasses](const b2PartBodyContact& contact) restrict(amp)
		{
			int32 a = contact.partIdx;
			Body& b = bodies(contact.bodyIdx);
			float32 w = contact.weight;
			float32 m = contact.mass;
			b2Vec2 n = contact.normal;
			b2Vec2 p = b2Vec2(positions[a]);
			float32 h = accumulations[a] + pressurePerWeight * w;
			b2Vec2 f = velocityPerPressure * w * m * h * n;
			float32 invMass = matInvMasses[matIdxs[a]];
			amp::atomicSub(velocities[a], invMass * f);
			b.ApplyLinearImpulse(f, p, true);
		});
		AmpForEachContactShuffled([=, &velocities, &accumulations, 
			&matMasses, &matIdxs](const b2ParticleContact& contact) restrict(amp)
		{
			const int32 a = contact.idxA;
			const int32 b = contact.idxB;
			const float32 w = contact.weight;
			const float32 m = contact.mass;
			const b2Vec3 n = contact.normal;
			const float32 h = accumulations[a] + accumulations[b];
			const b2Vec3 f = velocityPerPressure * w * m * h * n;
			amp::atomicSub(velocities[a], matMasses[matIdxs[b]] * f);
			amp::atomicAdd(velocities[b], matMasses[matIdxs[a]] * f);
		});
	}
	else
	{
		for (int32 i = 0; i < m_count; i++)
		{
			float32 w = m_weightBuffer[i];
			float32 h = pressurePerWeight * b2Max(0.0f, w - b2_minParticleWeight);
			m_accumulationBuf[i] = b2Min(h, maxPressure);
		}
		// ignores particles which have their own repulsive force
		if (m_allParticleFlags & k_noPressureFlags)
		{
			for (int32 i = 0; i < m_count; i++)
			{
				if (m_flagsBuffer[i] & k_noPressureFlags)
				{
					m_accumulationBuf[i] = 0;
				}
			}
		}
		// static pressure
		if (m_allParticleFlags & b2_staticPressureParticle)
		{
			b2Assert(m_staticPressureBuffer);
			for (int32 i = 0; i < m_count; i++)
			{
				if (m_flagsBuffer[i] & b2_staticPressureParticle)
				{
					m_accumulationBuf[i] += m_staticPressureBuf[i];
				}
			}
		}
		// applies pressure between each particles in contact
		for (int32 k = 0; k < m_bodyContactCount; k++)
		{
			const b2PartBodyContact& contact = m_bodyContactBuf[k];
			int32 a = contact.partIdx;
			Body& b = m_world.GetBody(contact.bodyIdx);
			float32 w = contact.weight;
			float32 m = contact.mass;
			b2Vec2 n = contact.normal;
			b2Vec2 p = b2Vec2(m_positionBuffer[a]);
			float32 h = m_accumulationBuf[a] + pressurePerWeight * w;
			b2Vec2 f = velocityPerPressure * w * m * h * n;
			float32 invMass = m_partMatInvMassBuf[m_partMatIdxBuffer[a]];
			m_velocityBuffer[a] -= invMass * f;
			b.ApplyLinearImpulse(f, p, true);
		}
		for (int32 k = 0; k < m_contactCount; k++)
		{
			const b2ParticleContact& contact = m_partContactBuf[k];
			int32 a = contact.idxA;
			int32 b = contact.idxB;
			float32 w = contact.weight;
			float32 m = contact.mass;
			b2Vec2 n = contact.normal;
			float32 h = m_accumulationBuf[a] + m_accumulationBuf[b];
			b2Vec2 f = velocityPerPressure * w * m * h * n;
			DistributeForce(a, b, f);
		}
	}
}

void b2ParticleSystem::SolveDamping()
{
	// reduces normal velocity of each contact
	const b2TimeStep& step = m_subStep;
	const float32 linearDamping = m_def.dampingStrength;
	const float32 quadraticDamping = 1 / GetCriticalVelocity(step);

	if (m_accelerate)
	{
		auto& velocities = m_ampVelocities;
		auto& matMasses = m_ampMatMasses;
		auto& matIdxs = m_ampMatIdxs;
		auto& bodies = m_ampBodies;
		auto& positions = m_ampPositions;
		auto& matInvMasses = m_ampMatInvMasses;
		AmpForEachBodyContact([=, &bodies, &positions, &velocities,
			&matIdxs, &matInvMasses](const b2PartBodyContact& contact) restrict(amp)
		{
			int32 a = contact.partIdx;
			Body& b = bodies[contact.bodyIdx];
			float32 w = contact.weight;
			float32 m = contact.mass;
			b2Vec2 n = contact.normal;
			b2Vec2 p = b2Vec2(positions[a]);
			b2Vec2 v = b.GetLinearVelocityFromWorldPoint(p) -
				b2Vec2(velocities[a]);
			float32 vn = b2Dot(v, n);
			if (vn < 0)
			{
				float32 damping =
					b2Max(linearDamping * w, b2Min(-quadraticDamping * vn, 0.5f));
				b2Vec2 f = damping * m * vn * n;
				float32 invMass = matInvMasses[matIdxs[a]];
				amp::atomicAdd(velocities[a], invMass * f);
				b.ApplyLinearImpulse(-f, p, true);
			}
		});
		AmpForEachContactShuffled([=, &velocities, &matMasses,
			&matIdxs](const b2ParticleContact& contact) restrict(amp)
		{
			const int32 a = contact.idxA;
			const int32 b = contact.idxB;
			const float32 w = contact.weight;
			const float32 m = contact.mass;
			const b2Vec3 n = contact.normal;
			const b2Vec3 v = velocities[b] - velocities[a];
			const float32 vn = b2Dot(v, n);
			if (vn >= 0) return;
			const float32 damping = b2Max(linearDamping * w, b2Min(-quadraticDamping * vn, 0.5f));
			const b2Vec3 f = damping * m * vn * n;
			amp::atomicAdd(velocities[a], matMasses[matIdxs[b]] * f);
			amp::atomicSub(velocities[b], matMasses[matIdxs[a]] * f);
		});

	}
	else
	{
		for (int32 k = 0; k < m_bodyContactCount; k++)
		{
			const b2PartBodyContact& contact = m_bodyContactBuf[k];
			int32 a = contact.partIdx;
			Body& b = m_world.GetBody(contact.bodyIdx);
			float32 w = contact.weight;
			float32 m = contact.mass;
			b2Vec2 n = contact.normal;
			b2Vec2 p = b2Vec2(m_positionBuffer[a]);
			b2Vec2 v = b.GetLinearVelocityFromWorldPoint(p) -
				b2Vec2(m_velocityBuffer[a]);
			float32 vn = b2Dot(v, n);
			if (vn < 0)
			{
				float32 damping =
					b2Max(linearDamping * w, b2Min(-quadraticDamping * vn, 0.5f));
				b2Vec2 f = damping * m * vn * n;
				float32 invMass = m_partMatInvMassBuf[m_partMatIdxBuffer[a]];
				m_velocityBuffer[a] += invMass * f;
				b.ApplyLinearImpulse(-f, p, true);
			}
		}
		for (int32 k = 0; k < m_contactCount; k++)
		{
			const b2ParticleContact& contact = m_partContactBuf[k];
			int32 a = contact.idxA;
			int32 b = contact.idxB;
			float32 w = contact.weight;
			float32 m = contact.mass;
			b2Vec2 n = contact.normal;
			b2Vec2 v = b2Vec2(m_velocityBuffer[b] - m_velocityBuffer[a]);
			float32 vn = b2Dot(v, n);
			if (vn < 0)
			{
				float32 damping =
					b2Max(linearDamping * w, b2Min(-quadraticDamping * vn, 0.5f));
				b2Vec2 f = damping * m * vn * n;
				DistributeForceDamp(a, b, f);
			}
		}
	}
}

void b2ParticleSystem::SolveSlowDown(const b2TimeStep& step)
{
	if (m_def.dampingStrength > 0)
	{
		float d = 1 - (step.dt * m_def.dampingStrength);
		float dd = d * d;
		for (int32 k = 0; k < m_count; k++)
		{
			// float slowDown = (m_heightLayerBuffer[k] > 0) ? d : dd;
			float slowDown = d;
			m_velocityBuffer[k] *= slowDown;
		}
	}
}

inline bool b2ParticleSystem::IsRigidGroup(const b2ParticleGroup& group) const
{
	return group.m_groupFlags & b2_rigidParticleGroup;
}

inline b2Vec2 b2ParticleSystem::GetLinearVelocity(
	const b2ParticleGroup& group, int32 particleIndex,
	const b2Vec2 &point)
{
	if (IsRigidGroup(group))
		return GetLinearVelocityFromWorldPoint(group, point);
	else
		return b2Vec2(m_velocityBuffer[particleIndex]);
}

inline void b2ParticleSystem::InitDampingParameter(
	float32& invMass, float32& invInertia, float32& tangentDistance,
	float32 mass, float32 inertia, const b2Vec2& center,
	const b2Vec2& point, const b2Vec2& normal) const
{
	invMass = mass > 0 ? 1 / mass : 0;
	invInertia = inertia > 0 ? 1 / inertia : 0;
	tangentDistance = b2Cross(point - center, normal);
}

inline void b2ParticleSystem::InitDampingParameterWithRigidGroupOrParticle(
	float32& invMass, float32& invInertia, float32& tangentDistance,
	bool isRigidGroup, const b2ParticleGroup& group, int32 particleIndex,
	const b2Vec2& point, const b2Vec2& normal)
{
	if (isRigidGroup)
	{
		InitDampingParameter(
			invMass, invInertia, tangentDistance,
			GetMass(group), GetInertia(group), GetCenter(group),
			point, normal);
	}
	else
	{
		uint32 flags = m_flagsBuffer[particleIndex];
		InitDampingParameter(
			invMass, invInertia, tangentDistance,
			//flags & b2_wallParticle ? 0 : GetParticleMass(), 0, point,
			flags & b2_wallParticle ? 0 : m_partMatMassBuf[m_partMatIdxBuffer[particleIndex]], 0, point,
			point, normal);
	}
}

inline float32 b2ParticleSystem::ComputeDampingImpulse(
	float32 invMassA, float32 invInertiaA, float32 tangentDistanceA,
	float32 invMassB, float32 invInertiaB, float32 tangentDistanceB,
	float32 normalVelocity) const
{
	float32 invMass =
		invMassA + invInertiaA * tangentDistanceA * tangentDistanceA +
		invMassB + invInertiaB * tangentDistanceB * tangentDistanceB;
	return invMass > 0 ? normalVelocity / invMass : 0;
}

inline void b2ParticleSystem::ApplyDamping(
	float32 invMass, float32 invInertia, float32 tangentDistance,
	bool isRigidGroup, b2ParticleGroup& group, int32 particleIndex,
	float32 impulse, const b2Vec2& normal)
{
	if (isRigidGroup)
	{
		group.m_linearVelocity += impulse * invMass * normal;
		group.m_angularVelocity += impulse * tangentDistance * invInertia;
	}
	else
	{
		b2Vec2 vel = impulse * invMass * normal;
		m_velocityBuffer[particleIndex] += vel;
	}
}

void b2ParticleSystem::SolveRigidDamping()
{
	// Apply impulse to rigid particle groups colliding with other objects
	// to reduce relative velocity at the colliding point.
	if (!(m_allGroupFlags & b2_rigidParticleGroup)) return;

	const float32 damping = m_def.dampingStrength;
	const uint32 filterFlag = b2_rigidParticleGroup;
	const uint32 wallFlag = b2_wallParticle;
	const int32 timeStamp = m_timestamp;

	if (m_accelerate)
	{
		auto& velocities = m_ampVelocities;
		auto& positions = m_ampPositions;
		auto& groupIdxs = m_ampGroupIdxs;
		auto& groups = m_ampGroups;
		auto& matMasses = m_ampMatMasses;
		
		const auto& AmpUpdateStatistics = [=, &positions, &velocities,
			&matMasses](const b2ParticleGroup& group) restrict(amp)
		{
			if (group.m_timestamp == timeStamp) return;
			const float32 m = matMasses[group.m_matIdx];
			const int32 & firstIdx = group.m_firstIndex;
			const int32 & lastIdx = group.m_lastIndex;
			float32 & mass = group.m_mass = 0;
			b2Vec2 & center = group.m_center;
			b2Vec2 & linVel = group.m_linearVelocity;
			center.SetZero();
			linVel.SetZero();
			for (int32 i = firstIdx; i < lastIdx; i++)
			{
				mass += m;
				center += m * (b2Vec2)positions[i];
				linVel += m * (b2Vec2)velocities[i];
			}
			if (mass > 0)
			{
				center *= 1 / mass;
				linVel *= 1 / mass;
			}
			float32& inertia = group.m_inertia = 0;
			float32& angVel = group.m_angularVelocity = 0;
			for (int32 i = firstIdx; i < lastIdx; i++)
			{
				b2Vec2 p = (b2Vec2)positions[i] - center;
				b2Vec2 v = (b2Vec2)velocities[i] - linVel;
				inertia += m * b2Dot(p, p);
				angVel += m * b2Cross(p, v);
			}
			if (inertia > 0)
			{
				angVel *= 1 / inertia;
			}
			group.m_timestamp = timeStamp;
		};

		const auto AmpGetLinearVelocity = [=](const b2ParticleGroup& group,
			const b2Vec2& point) restrict(amp) -> b2Vec2
		{
			AmpUpdateStatistics(group);
			return group.m_linearVelocity + b2Cross(group.m_angularVelocity, point - group.m_center);
		};

		const auto AmpInitDampingParameter = [=](
			float32& invMass, float32& invInertia, float32& tangentDistance,
			float32 mass, float32 inertia, const b2Vec2& center,
			const b2Vec2& point, const b2Vec2 & normal) restrict(amp)
		{
			invMass = mass > 0 ? 1 / mass : 0;
			invInertia = inertia > 0 ? 1 / inertia : 0;
			tangentDistance = b2Cross(point - center, normal);
		};

		auto& matIdxs = m_ampMatIdxs;
		auto& flags = m_ampFlags;
		const auto AmpInitDampingParameterWithRigidGroupOrParticle = [=, &matIdxs, &matMasses, &flags](
			float32& invMass, float32& invInertia, float32& tangentDistance,
			uint32 isRigid, const b2ParticleGroup& group, int32 particleIndex,
			const b2Vec2& point, const b2Vec2& normal) restrict(amp)
		{
			if (isRigid)
			{
				AmpInitDampingParameter(
					invMass, invInertia, tangentDistance,
					group.m_mass, group.m_inertia, group.m_center,
					point, normal);
			}
			else
			{
				AmpInitDampingParameter(
					invMass, invInertia, tangentDistance,
					flags[particleIndex] & wallFlag ? 0 : matMasses[matIdxs[particleIndex]],
					0, point, point, normal);
			}
		};
		const auto AmpComputeDampingImpulse = [=](
			float32 invMassA, float32 invInertiaA, float32 tangentDistanceA,
			float32 invMassB, float32 invInertiaB, float32 tangentDistanceB,
			float32 normalVelocity) restrict(amp) -> float32
		{
			const float32 invMass =
				invMassA + invInertiaA * tangentDistanceA * tangentDistanceA +
				invMassB + invInertiaB * tangentDistanceB * tangentDistanceB;
			return invMass > 0 ? normalVelocity / invMass : 0;
		};
		const auto ApplyDamping = [=, &velocities](
			float32 invMass, float32 invInertia, float32 tangentDistance,
			uint32 isRigid, b2ParticleGroup& group, int32 particleIndex,
			float32 impulse, const b2Vec2& normal) restrict(amp)
		{
			if (isRigid)
			{
				group.m_linearVelocity += impulse * invMass * normal;
				group.m_angularVelocity += impulse * tangentDistance * invInertia;
			}
			else
			{
				const b2Vec2 vel = impulse * invMass * normal;
				amp::atomicAdd(velocities[particleIndex], vel);
			}
		};
		auto& bodies = m_ampBodies;
		AmpForEachBodyContact([=, &groupIdxs, &groups, &bodies,
			&positions](const b2PartBodyContact& contact) restrict(amp)
		{
			int32 a = contact.partIdx;
			b2ParticleGroup& aGroup = groups[groupIdxs[a]];
			if (!(aGroup.m_groupFlags & filterFlag)) return;
			Body& b = bodies[contact.bodyIdx];
			b2Vec2 n = contact.normal;
			float32 w = contact.weight;
			b2Vec2 p = b2Vec2(positions[a]);
			b2Vec2 v = b.GetLinearVelocityFromWorldPoint(p) -
				AmpGetLinearVelocity(aGroup, p);
			float32 vn = b2Dot(v, n);
			if (vn >= 0) return;
			// The group's average velocity at particle position 'p' is pushing
			// the particle into the body.
			float32 invMassA, invInertiaA, tangentDistanceA;
			float32 invMassB, invInertiaB, tangentDistanceB;
			AmpInitDampingParameterWithRigidGroupOrParticle(
				invMassA, invInertiaA, tangentDistanceA, true,
				aGroup, a, p, n);
			AmpInitDampingParameter(
				invMassB, invInertiaB, tangentDistanceB,
				b.m_mass,
				// Calculate b.m_I from public functions of b2Body.
				b.GetInertia() -
				b.m_mass * b.GetLocalCenter().LengthSquared(),
				b.GetWorldCenter(),
				p, n);
			float32 f = damping * b2Min(w, 1.0f) * AmpComputeDampingImpulse(
				invMassA, invInertiaA, tangentDistanceA,
				invMassB, invInertiaB, tangentDistanceB,
				vn);
			ApplyDamping(
				invMassA, invInertiaA, tangentDistanceA,
				true, aGroup, a, f, n);
			b.ApplyLinearImpulse(-f * n, p, true);
		});
		AmpForEachContact([=, &groups, &groupIdxs,
			&positions, &velocities](const b2ParticleContact& contact) restrict(amp)
		{
			const int32 a = contact.idxA;
			const int32 b = contact.idxB;
			const b2Vec3 n = contact.normal;
			const float32 w = contact.weight;
			const int32 aGroupIdx = groupIdxs[a];
			const int32 bGroupIdx = groupIdxs[b];
			b2ParticleGroup& aGroup = groups[aGroupIdx];
			b2ParticleGroup& bGroup = groups[bGroupIdx];
			const uint32 aRigid = aGroup.m_groupFlags & filterFlag;
			const uint32 bRigid = aGroup.m_groupFlags & filterFlag;
			if (aGroupIdx == bGroupIdx || !(aRigid || bRigid)) return;
			const b2Vec2 p = 0.5f * positions[a] + positions[b];
			const b2Vec2 v = (bRigid ? AmpGetLinearVelocity(bGroup, p) : velocities[b]) -
							 (aRigid ? AmpGetLinearVelocity(aGroup, p) : velocities[a]);
			const float32 vn = b2Dot(v, n);
			if (vn >= 0) return;
			float32 invMassA, invInertiaA, tangentDistanceA;
			float32 invMassB, invInertiaB, tangentDistanceB;
			AmpInitDampingParameterWithRigidGroupOrParticle(
				invMassA, invInertiaA, tangentDistanceA,
				aRigid, aGroup, a, p, n);
			AmpInitDampingParameterWithRigidGroupOrParticle(
				invMassB, invInertiaB, tangentDistanceB,
				bRigid, bGroup, b, p, n);
			const float32 f = damping * w * AmpComputeDampingImpulse(
				invMassA, invInertiaA, tangentDistanceA,
				invMassB, invInertiaB, tangentDistanceB,
				vn);
			ApplyDamping(
				invMassA, invInertiaA, tangentDistanceA,
				aRigid, aGroup, a, f, n);
			ApplyDamping(
				invMassB, invInertiaB, tangentDistanceB,
				bRigid, bGroup, b, -f, n);
		});

	}
	else
	{
		for (int32 k = 0; k < m_bodyContactCount; k++)
		{
			const b2PartBodyContact& contact = m_bodyContactBuf[k];
			int32 a = contact.partIdx;
			int32 aGroupIdx = m_partGroupIdxBuffer[a];
			b2ParticleGroup& aGroup = m_groupBuffer[aGroupIdx];
			if (IsRigidGroup(aGroup))
			{
				Body& b = m_world.GetBody(contact.bodyIdx);
				b2Vec2 n = contact.normal;
				float32 w = contact.weight;
				b2Vec2 p = b2Vec2(m_positionBuffer[a]);
				b2Vec2 v = b.GetLinearVelocityFromWorldPoint(p) -
					GetLinearVelocityFromWorldPoint(aGroup, p);
				float32 vn = b2Dot(v, n);
				if (vn < 0)
					// The group's average velocity at particle position 'p' is pushing
					// the particle into the body.
				{
					float32 invMassA, invInertiaA, tangentDistanceA;
					float32 invMassB, invInertiaB, tangentDistanceB;
					InitDampingParameterWithRigidGroupOrParticle(
						invMassA, invInertiaA, tangentDistanceA,
						true, aGroup, a, p, n);
					InitDampingParameter(
						invMassB, invInertiaB, tangentDistanceB,
						b.m_mass,
						// Calculate b.m_I from public functions of b2Body.
						b.GetInertia() -
						b.m_mass * b.GetLocalCenter().LengthSquared(),
						b.GetWorldCenter(),
						p, n);
					float32 f = damping * b2Min(w, 1.0f) * ComputeDampingImpulse(
						invMassA, invInertiaA, tangentDistanceA,
						invMassB, invInertiaB, tangentDistanceB,
						vn);
					ApplyDamping(
						invMassA, invInertiaA, tangentDistanceA,
						true, aGroup, a, f, n);
					b.ApplyLinearImpulse(-f * n, p, true);
				}
			}
		}
		for (int32 k = 0; k < m_contactCount; k++)
		{
			const b2ParticleContact& contact = m_partContactBuf[k];
			int32 a = contact.idxA;
			int32 b = contact.idxB;
			b2Vec2 n = contact.normal;
			float32 w = contact.weight;
			int32 aGroupIdx = m_partGroupIdxBuffer[a];
			int32 bGroupIdx = m_partGroupIdxBuffer[b];
			b2ParticleGroup& aGroup = m_groupBuffer[aGroupIdx];
			b2ParticleGroup& bGroup = m_groupBuffer[bGroupIdx];
			bool aRigid = IsRigidGroup(aGroup);
			bool bRigid = IsRigidGroup(bGroup);
			if (aGroupIdx != bGroupIdx && (aRigid || bRigid))
			{
				b2Vec2 p =
					0.5f * b2Vec2(m_positionBuffer[a] + m_positionBuffer[b]);
				b2Vec2 v =
					GetLinearVelocity(bGroup, b, p) -
					GetLinearVelocity(aGroup, a, p);
				float32 vn = b2Dot(v, n);
				if (vn < 0)
				{
					float32 invMassA, invInertiaA, tangentDistanceA;
					float32 invMassB, invInertiaB, tangentDistanceB;
					InitDampingParameterWithRigidGroupOrParticle(
						invMassA, invInertiaA, tangentDistanceA,
						aRigid, aGroup, a,
						p, n);
					InitDampingParameterWithRigidGroupOrParticle(
						invMassB, invInertiaB, tangentDistanceB,
						bRigid, bGroup, b,
						p, n);
					float32 f = damping * w * ComputeDampingImpulse(
						invMassA, invInertiaA, tangentDistanceA,
						invMassB, invInertiaB, tangentDistanceB,
						vn);
					ApplyDamping(
						invMassA, invInertiaA, tangentDistanceA,
						aRigid, aGroup, a, f, n);
					ApplyDamping(
						invMassB, invInertiaB, tangentDistanceB,
						bRigid, bGroup, b, -f, n);
				}
			}
		}
	}
}

void b2ParticleSystem::SolveExtraDamping()
{
	// Applies additional damping force between bodies and particles which can
	// produce strong repulsive force. Applying damping force multiple times
	// is effective in suppressing vibration.

	if (!(m_allParticleFlags & k_extraDampingFlags)) return;

	if (m_accelerate)
	{
		const uint32 filterFlag = k_extraDampingFlags;
		auto& flags = m_ampFlags;
		auto& bodies = m_ampBodies;
		auto& positions = m_ampPositions;
		auto& velocities = m_ampVelocities;
		auto& matIdxs = m_ampMatIdxs;
		auto& matInvMasses = m_ampMatInvMasses;
		AmpForEachBodyContact([=, &flags, &bodies, &positions, &velocities,
			&matIdxs, &matInvMasses](const b2PartBodyContact& contact) restrict(amp)
		{
			int32 a = contact.partIdx;
			if (!(flags[a] & filterFlag)) return;
			Body& b = bodies[contact.bodyIdx];
			float32 m = contact.mass;
			b2Vec2 n = contact.normal;
			b2Vec2 p = b2Vec2(positions[a]);
			b2Vec2 v =
				b.GetLinearVelocityFromWorldPoint(p) -
				b2Vec2(velocities[a]);
			float32 vn = b2Dot(v, n);
			if (vn >= 0) return;
			b2Vec2 f = 0.5f * m * vn * n;
			//m_velocityBuffer.data[a] += GetParticleInvMass() * f;
			float32 invMass = matInvMasses[matIdxs[a]];
			amp::atomicAdd(velocities[a], invMass * f);
			b.ApplyLinearImpulse(-f, p, true);
		});
	}
	else
	{
		for (int32 k = 0; k < m_bodyContactCount; k++)
		{
			const b2PartBodyContact& contact = m_bodyContactBuf[k];
			int32 a = contact.partIdx;
			if (m_flagsBuffer[a] & k_extraDampingFlags)
			{
				Body& b = m_world.GetBody(contact.bodyIdx);
				float32 m = contact.mass;
				b2Vec2 n = contact.normal;
				b2Vec2 p = b2Vec2(m_positionBuffer[a]);
				b2Vec2 v =
					b.GetLinearVelocityFromWorldPoint(p) -
					b2Vec2(m_velocityBuffer[a]);
				float32 vn = b2Dot(v, n);
				if (vn < 0)
				{
					b2Vec2 f = 0.5f * m * vn * n;
					//m_velocityBuffer.data[a] += GetParticleInvMass() * f;
					float32 invMass = m_partMatInvMassBuf[m_partMatIdxBuffer[a]];
					m_velocityBuffer[a] += invMass * f;
					b.ApplyLinearImpulse(-f, p, true);
				}
			}
		}
	}
}

void b2ParticleSystem::SolveWall()
{
	if (!(m_allParticleFlags & b2_wallParticle)) return;

	if (m_accelerate)
	{
		auto& velocities = m_ampVelocities;
		AmpForEachParticle(b2_wallParticle, [=, &velocities](const int32 i) restrict(amp)
		{
			velocities[i].SetZero();
		});
	}
	else
	{
		for (int32 i = 0; i < m_count; i++)
		{
			if (m_flagsBuffer[i] & b2_wallParticle)
				m_velocityBuffer[i].SetZero();
		}
	}
}

void b2ParticleSystem::CopyVelocities()
{
	if (m_accelerate)
		m_ampCopyFutVelocities.set(amp::copyAsync(m_ampVelocities, m_velocityBuffer, m_count));
}

void b2ParticleSystem::SolveRigid()
{
	if (!(m_allGroupFlags & b2_rigidParticleGroup)) return;

	const b2TimeStep& step = m_subStep;

	if (m_accelerate)
	{
		auto& positions = m_ampPositions;
		auto& velocities = m_ampVelocities;
		for (int32 k = 0; k < m_groupCount; k++)
		{
			b2ParticleGroup& group = m_groupBuffer[k];
			if (group.m_firstIndex != b2_invalidIndex)
			{
				if (group.m_groupFlags & b2_rigidParticleGroup)
				{
					UpdateStatistics(group);
					b2Rot rotation(step.dt * group.m_angularVelocity);
					b2Vec2 center = group.m_center;
					b2Vec2 linVel = group.m_linearVelocity;
					b2Transform transform(center + step.dt * linVel -
						b2Mul(rotation, center), rotation);
					group.m_transform = b2Mul(transform, group.m_transform);
					b2Transform velocityTransform;
					velocityTransform.p.x = step.inv_dt * transform.p.x;
					velocityTransform.p.y = step.inv_dt * transform.p.y;
					velocityTransform.q.s = step.inv_dt * transform.q.s;
					velocityTransform.q.c = step.inv_dt * (transform.q.c - 1);

					amp::forEach(group.m_firstIndex, group.m_lastIndex,
						[=, &positions, &velocities](const int32 i) restrict(amp)
						{
							const b2Vec3 vel = b2Vec3(b2Mul(velocityTransform, positions[i]), 0);
							velocities[i] = vel;
						});
				}
			}
		}
	}
	else
	{
		for (int32 k = 0; k < m_groupCount; k++)
		{
			b2ParticleGroup& group = m_groupBuffer[k];
			if (group.m_firstIndex != b2_invalidIndex)
			{
				if (group.m_groupFlags & b2_rigidParticleGroup)
				{
					UpdateStatistics(group);
					b2Rot rotation(step.dt * group.m_angularVelocity);
					b2Vec2 center = group.m_center;
					b2Vec2 linVel = group.m_linearVelocity;
					b2Transform transform(center + step.dt * linVel -
						b2Mul(rotation, center), rotation);
					group.m_transform = b2Mul(transform, group.m_transform);
					b2Transform velocityTransform;
					velocityTransform.p.x = step.inv_dt * transform.p.x;
					velocityTransform.p.y = step.inv_dt * transform.p.y;
					velocityTransform.q.s = step.inv_dt * transform.q.s;
					velocityTransform.q.c = step.inv_dt * (transform.q.c - 1);
					for (int32 i = group.m_firstIndex; i < group.m_lastIndex; i++)
					{
						b2Vec2 vel = b2Mul(velocityTransform,
							b2Vec2(m_positionBuffer[i]));
						m_velocityBuffer[i] = vel;
					}
				}
			}
		}
	}
}

// SolveElastic and SolveSpring refer the current velocities for
// numerical stability, they should be called as late as possible.
void b2ParticleSystem::SolveElastic()
{
	if (!(m_allParticleFlags & b2_elasticParticle)) return;

	const b2TimeStep& step = m_subStep;
	const uint32 filterFlag = b2_elasticParticle;
	const float32 elasticStrength = step.inv_dt * m_def.elasticStrength;

	if (m_accelerate)
	{
		m_ampCopyFutTriads.wait();
		auto& triads = m_ampTriads;
		auto& positions = m_ampPositions;
		auto& velocities =  m_ampVelocities;
		AmpForEachTriad([=, &triads, &positions, &velocities](const int32 i) restrict(amp)
		{
			const b2ParticleTriad& triad = triads[i];
			if (!(triad.flags & filterFlag)) return;
			const int32 a = triad.indexA;
			const int32 b = triad.indexB;
			const int32 c = triad.indexC;
			const b2Vec2& oa = triad.pa;
			const b2Vec2& ob = triad.pb;
			const b2Vec2& oc = triad.pc;
			b2Vec3 pa = positions[a];
			b2Vec3 pb = positions[b];
			b2Vec3 pc = positions[c];
			const b2Vec3 va = velocities[a];
			const b2Vec3 vb = velocities[b];
			const b2Vec3 vc = velocities[c];
			pa += step.dt * va;
			pb += step.dt * vb;
			pc += step.dt * vc;
			b2Vec3 midPoint = (float32)1 / 3 * (pa + pb + pc);
			pa -= midPoint;
			pb -= midPoint;
			pc -= midPoint;
			b2Rot r;
			r.s = b2Cross(oa, pa) + b2Cross(ob, pb) + b2Cross(oc, pc);
			r.c = b2Dot(oa, pa) + b2Dot(ob, pb) + b2Dot(oc, pc);
			float32 r2 = r.s * r.s + r.c * r.c;
			float32 invR = b2InvSqrt(r2);
			r.s *= invR;
			r.c *= invR;
			float32 strength = elasticStrength * triad.strength;
			b2Vec2 vel = b2Vec2(b2Mul(r, oa) - pa);
			amp::atomicAdd(velocities[a], strength * vel);
			vel = b2Vec2(b2Mul(r, ob) - pb);
			amp::atomicAdd(velocities[b], strength * vel);
			vel = b2Vec2(b2Mul(r, oc) - pc);
			amp::atomicAdd(velocities[c], strength * vel);
		});
	}
	else
	{
		for (int32 k = 0; k < m_triadCount; k++)
		{
			const b2ParticleTriad& triad = m_triadBuffer[k];
			if (triad.flags & b2_elasticParticle)
			{
				int32 a = triad.indexA;
				int32 b = triad.indexB;
				int32 c = triad.indexC;
				const b2Vec2& oa = triad.pa;
				const b2Vec2& ob = triad.pb;
				const b2Vec2& oc = triad.pc;
				b2Vec2 pa = b2Vec2(m_positionBuffer[a]);
				b2Vec2 pb = b2Vec2(m_positionBuffer[b]);
				b2Vec2 pc = b2Vec2(m_positionBuffer[c]);
				b2Vec2 va = b2Vec2(m_velocityBuffer[a]);
				b2Vec2 vb = b2Vec2(m_velocityBuffer[b]);
				b2Vec2 vc = b2Vec2(m_velocityBuffer[c]);
				pa += step.dt * va;
				pb += step.dt * vb;
				pc += step.dt * vc;
				b2Vec2 midPoint = (float32)1 / 3 * (pa + pb + pc);
				pa -= midPoint;
				pb -= midPoint;
				pc -= midPoint;
				b2Rot r;
				r.s = b2Cross(oa, pa) + b2Cross(ob, pb) + b2Cross(oc, pc);
				r.c = b2Dot(oa, pa) + b2Dot(ob, pb) + b2Dot(oc, pc);
				float32 r2 = r.s * r.s + r.c * r.c;
				float32 invR = b2InvSqrt(r2);
				r.s *= invR;
				r.c *= invR;
				float32 strength = elasticStrength * triad.strength;
				b2Vec2 vel = (b2Mul(r, oa) - pa);
				m_velocityBuffer[a] += strength * vel;
				vel = (b2Mul(r, ob) - pb);
				m_velocityBuffer[b] += strength * vel;
				vel = (b2Mul(r, oc) - pc);
				m_velocityBuffer[c] += strength * vel;
			}
		}
	}
}

void b2ParticleSystem::SolveSpring()
{
	if (!(m_allParticleFlags & b2_springParticle)) return;

	const b2TimeStep& step = m_subStep;
	const uint32 filterFlag = b2_springParticle;
	const float32 springStrength = step.inv_dt * m_def.springStrength;

	if (m_accelerate)
	{
		auto& pairs = m_ampPairs;
		auto& positions = m_ampPositions;
		auto& velocities = m_ampVelocities;
		AmpForEachPair([=, &pairs, &positions, &velocities](const int32 i) restrict(amp)
		{
			const b2ParticlePair& pair = pairs[i];
			if (!(pair.flags & filterFlag)) return;
			const int32 a = pair.indexA;
			const int32 b = pair.indexB;
			b2Vec3 pa = positions[a];
			b2Vec3 pb = positions[b];
			const b2Vec3 vb = velocities[b];
			const b2Vec3 va = velocities[a];
			pa += step.dt * va;
			pb += step.dt * vb;
			const b2Vec3 d = pb - pa;
			const float32 r0 = pair.distance;
			const float32 r1 = d.Length();
			const float32 strength = springStrength * pair.strength;
			b2Vec2 f = strength * (r0 - r1) / r1 * d;
			amp::atomicAdd(velocities[a], f);
			amp::atomicSub(velocities[b], f);
		});
	}
	else
	{
		for (int32 k = 0; k < m_pairCount; k++)
		{
			const b2ParticlePair& pair = m_pairBuffer[k];
			if (pair.flags & b2_springParticle)
			{
				int32 a = pair.indexA;
				int32 b = pair.indexB;
				b2Vec2 pa = b2Vec2(m_positionBuffer[a]);
				b2Vec2 pb = b2Vec2(m_positionBuffer[b]);
				b2Vec2 va = b2Vec2(m_velocityBuffer[a]);
				b2Vec2 vb = b2Vec2(m_velocityBuffer[b]);
				pa += step.dt * va;
				pb += step.dt * vb;
				b2Vec2 d = pb - pa;
				float32 r0 = pair.distance;
				float32 r1 = d.Length();
				float32 strength = springStrength * pair.strength;
				b2Vec2 f = strength * (r0 - r1) / r1 * d;
				m_velocityBuffer[a] -= f;
				m_velocityBuffer[b] += f;
			}
		}
	}
}

void b2ParticleSystem::SolveTensile()
{
	if (!(m_allParticleFlags & b2_tensileParticle)) return;

	const b2TimeStep& step = m_subStep;
	const float32 criticalVelocity = GetCriticalVelocity(step);
	const float32 pressureStrength = m_def.surfaceTensionPressureStrength * criticalVelocity;
	const float32 normalStrength = m_def.surfaceTensionNormalStrength * criticalVelocity;
	const float32 maxVelocityVariation = b2_maxParticleForce * criticalVelocity;

	if (m_accelerate)
	{
		const uint32 filterFlag = b2_tensileParticle;
		auto& accumulations = m_ampAccumulationVec3s;
		amp::fill(accumulations, b2Vec3_zero);
		AmpForEachContact([=, &accumulations](const b2ParticleContact& contact) restrict(amp)
		{
			if (!(contact.flags & filterFlag)) return;
			const int32 a = contact.idxA;
			const int32 b = contact.idxB;
			const float32 w = contact.weight;
			const b2Vec3 n = contact.normal;
			const b2Vec3 weightedNormal = (1 - w) * w * n;
			amp::atomicSub(accumulations[a], weightedNormal);
			amp::atomicAdd(accumulations[b], weightedNormal);
		});

		auto& weights = m_ampWeights;
		auto& velocities = m_ampVelocities;
		auto& matIdxs = m_ampMatIdxs;
		auto& matMasses = m_ampMatMasses;
		AmpForEachContact([=, &weights, &velocities, &accumulations,
			&matIdxs, &matMasses](const b2ParticleContact& contact) restrict(amp)
		{
			if (!(contact.flags & filterFlag)) return;
			const int32 a = contact.idxA;
			const int32 b = contact.idxB;
			const float32 w = contact.weight;
			const float32 m = contact.mass;
			const b2Vec3 n = contact.normal;
			const float32 h = weights[a] + weights[b];
			const b2Vec3 s = accumulations[b] - accumulations[a];
			const float32 fn = b2Min(
				pressureStrength * (h - 2) + normalStrength * b2Dot(s, n),
				maxVelocityVariation) * w;
			b2Vec3 f = fn * n * m;
			f.z = 0;
			amp::atomicSub(velocities[a], matMasses[matIdxs[b]] * f);
			amp::atomicAdd(velocities[b], matMasses[matIdxs[a]] * f);
		});
	}
	else
	{
		for (int32 i = 0; i < m_count; i++)
			m_accumulation3Buf[i].SetZero();
		
		for (int32 k = 0; k < m_contactCount; k++)
		{
			const b2ParticleContact& contact = m_partContactBuf[k];
			if (contact.flags & b2_tensileParticle)
			{
				int32 a = contact.idxA;
				int32 b = contact.idxB;
				float32 w = contact.weight;
				b2Vec3 n = contact.normal;
				b2Vec3 weightedNormal = (1 - w) * w * n;
				m_accumulation3Buf[a] -= weightedNormal;
				m_accumulation3Buf[b] += weightedNormal;
			}
		}
		for (int32 k = 0; k < m_contactCount; k++)
		{
			const b2ParticleContact& contact = m_partContactBuf[k];
			if (contact.flags & b2_tensileParticle)
			{
				int32 a = contact.idxA;
				int32 b = contact.idxB;
				float32 w = contact.weight;
				float32 m = contact.mass;
				b2Vec3 n = contact.normal;
				float32 h = m_weightBuffer[a] + m_weightBuffer[b];
				b2Vec3 s = m_accumulation3Buf[b] - m_accumulation3Buf[a];
				float32 fn = b2Min(
					pressureStrength * (h - 2) + normalStrength * b2Dot(s, n),
					maxVelocityVariation) * w;
				b2Vec3 f = fn * n * m;
				f.z = 0;
				DistributeForce(a, b, f);
			}
		}
	}
}

void b2ParticleSystem::SolveViscous()
{
	if (!(m_allParticleFlags & b2_viscousParticle)) return;

	const float32 viscousStrength = m_def.viscousStrength;

	if (m_accelerate)
	{
		const uint32 filterFlag = b2_viscousParticle;

		auto& velocities = m_ampVelocities;
		auto& matIdxs = m_ampMatIdxs;
		auto& matMasses = m_ampMatMasses;
		auto& flags = m_ampFlags;
		auto& body = m_ampBodies;
		auto& positions = m_ampPositions;
		auto& matInvMasses = m_ampMatInvMasses;
		AmpForEachBodyContact([=, &flags, &body, &positions, &velocities,
			&matIdxs, &matInvMasses](const b2PartBodyContact& contact) restrict(amp)
		{
			int32 a = contact.partIdx;
			if (!(flags[a] & filterFlag)) return;
			Body& b = body[contact.bodyIdx];
			float32 w = contact.weight;
			float32 m = contact.mass;
			b2Vec2 p = b2Vec2(positions[a]);
			b2Vec2 v = b.GetLinearVelocityFromWorldPoint(p) -
				b2Vec2(velocities[a]);
			b2Vec2 f = viscousStrength * m * w * v;
			float32 invMass = matInvMasses[matIdxs[a]];
			amp::atomicAdd(velocities[a], invMass * f);
			b.ApplyLinearImpulse(-f, p, true);
		});
		AmpForEachContact([=, &velocities, &matIdxs, &matMasses](const b2ParticleContact& contact) restrict(amp)
		{
			if (!(contact.flags & filterFlag)) return;
			const int32 a = contact.idxA;
			const int32 b = contact.idxB;
			const float32 w = contact.weight;
			const float32 m = contact.mass;
			const b2Vec3 v = velocities[b] - velocities[a];
			const b2Vec3 f = viscousStrength * w * m * v;
			amp::atomicAdd(velocities[a], matMasses[matIdxs[b]] * f);
			amp::atomicSub(velocities[b], matMasses[matIdxs[a]] * f);
		});
	
	}
	else
	{
		for (int32 k = 0; k < m_bodyContactCount; k++)
		{
			const b2PartBodyContact& contact = m_bodyContactBuf[k];
			int32 a = contact.partIdx;
			if (m_flagsBuffer[a] & b2_viscousParticle)
			{
				Body& b = m_world.GetBody(contact.bodyIdx);
				float32 w = contact.weight;
				float32 m = contact.mass;
				b2Vec2 p = b2Vec2(m_positionBuffer[a]);
				b2Vec2 v = b.GetLinearVelocityFromWorldPoint(p) -
					b2Vec2(m_velocityBuffer[a]);
				b2Vec2 f = viscousStrength * m * w * v;
				float32 invMass = m_partMatInvMassBuf[m_partMatIdxBuffer[a]];
				m_velocityBuffer[a] += invMass * f;
				b.ApplyLinearImpulse(-f, p, true);
			}
		}
		for (int32 k = 0; k < m_contactCount; k++)
		{
			const b2ParticleContact& contact = m_partContactBuf[k];
			if (contact.flags & b2_viscousParticle)
			{
				int32 a = contact.idxA;
				int32 b = contact.idxB;
				float32 w = contact.weight;
				float32 m = contact.mass;
				b2Vec2 v = b2Vec2(m_velocityBuffer[b] - m_velocityBuffer[a]);
				b2Vec2 f = viscousStrength * w * m * v;
				DistributeForceDamp(a, b, f);
			}
		}
	}
}

void b2ParticleSystem::SolveRepulsive()
{
	if (!(m_allParticleFlags & b2_repulsiveParticle)) return;

	const b2TimeStep& step = m_subStep;
	float32 repulsiveStrength =
		m_def.repulsiveStrength * GetCriticalVelocity(step);

	if (m_accelerate)
	{
		const uint32 filterFlag = b2_repulsiveParticle;
		auto& velocities = m_ampVelocities;
		auto& groupIdxs = m_ampGroupIdxs;
		auto& matIdxs = m_ampMatIdxs;
		auto& matMasses = m_ampMatMasses;
		AmpForEachContact([=, &velocities, &groupIdxs, &matIdxs, &matMasses](const b2ParticleContact& contact) restrict(amp)
		{
			if (!(contact.flags & filterFlag)) return;
			const int32 a = contact.idxA;
			const int32 b = contact.idxB;
			if (groupIdxs[a] == groupIdxs[b]) return;
			const float32 w = contact.weight;
			const float32 m = contact.mass;
			const b2Vec3 n = contact.normal;
			const b2Vec3 f = repulsiveStrength * w * m * n;
			amp::atomicSub(velocities[a], matMasses[matIdxs[b]] * f);
			amp::atomicAdd(velocities[b], matMasses[matIdxs[a]] * f);
		});
	}
	else
	{
		for (int32 k = 0; k < m_contactCount; k++)
		{
			const b2ParticleContact& contact = m_partContactBuf[k];
			if (contact.flags & b2_repulsiveParticle)
			{
				int32 a = contact.idxA;
				int32 b = contact.idxB;
				if (m_partGroupIdxBuffer[a] != m_partGroupIdxBuffer[b])
				{
					float32 w = contact.weight;
					float32 m = contact.mass;
					b2Vec2 n = contact.normal;
					b2Vec2 f = repulsiveStrength * w * m * n;
					DistributeForce(a, b, f);
				}
			}
		}
	}
}

void b2ParticleSystem::SolvePowder()
{
	if (!(m_allParticleFlags & b2_powderParticle)) return;

	const b2TimeStep& step = m_subStep;
	float32 powderStrength = m_def.powderStrength * GetCriticalVelocity(step);
	const float32 minWeight = 1.0f - b2_particleStride;

	if (m_accelerate)
	{
		const uint32 filterFlag = b2_powderParticle;
		auto& velocities = m_ampVelocities;
		auto& matIdxs = m_ampMatIdxs;
		auto& matMasses = m_ampMatMasses;
		AmpForEachContact([=, &velocities, &matIdxs, &matMasses](const b2ParticleContact& contact) restrict(amp)
		{
			if (!(contact.flags & filterFlag)) return;
			const float32 w = contact.weight;
			if (w <= minWeight) return;
			const int32 a = contact.idxA;
			const int32 b = contact.idxB;
			const float32 m = contact.mass;
			const b2Vec3 n = contact.normal;
			const b2Vec3 f = powderStrength * (w - minWeight) * m * n;
			amp::atomicSub(velocities[a], matMasses[matIdxs[b]] * f);
			amp::atomicAdd(velocities[b], matMasses[matIdxs[a]] * f);
		});
	}
	else
	{
		for (int32 k = 0; k < m_contactCount; k++)
		{
			const b2ParticleContact& contact = m_partContactBuf[k];
			if (contact.flags & b2_powderParticle)
			{
				float32 w = contact.weight;
				if (w > minWeight)
				{
					int32 a = contact.idxA;
					int32 b = contact.idxB;
					float32 m = contact.mass;
					b2Vec2 n = contact.normal;
					b2Vec2 f = powderStrength * (w - minWeight) * m * n;
					DistributeForce(a, b, f);
				}
			}
		}
	}
}

void b2ParticleSystem::SolveSolid()
{
	// applies extra repulsive force from solid particle groups
	if (!(m_allGroupFlags & b2_solidParticleGroup)) return;

	b2Assert(m_hasDepth);
	const b2TimeStep& step = m_subStep;
	const float32 ejectionStrength = step.inv_dt * m_def.ejectionStrength;
	
	if (m_accelerate)
	{
		auto& velocities = m_ampVelocities;
		auto& groupIdxs = m_ampGroupIdxs;
		auto& matIdxs = m_ampMatIdxs;
		auto& matMasses = m_ampMatMasses;
		auto& depths = m_ampDepths;
		AmpForEachContact([=, &velocities, &groupIdxs, 
			&matIdxs, &matMasses, &depths](const b2ParticleContact& contact) restrict(amp)
		{
			const int32 a = contact.idxA;
			const int32 b = contact.idxB;
			if (groupIdxs[a] == groupIdxs[b]) return;
			const float32 w = contact.weight;
			const float32 m = contact.mass;
			const b2Vec3 n = contact.normal;
			const float32 h = depths[a] + depths[b];
			const b2Vec3 f = ejectionStrength * h * m * w * n;
			amp::atomicSub(velocities[a], matMasses[matIdxs[b]] * f);
			amp::atomicAdd(velocities[b], matMasses[matIdxs[a]] * f);
		});
	}
	else
	{
		for (int32 k = 0; k < m_contactCount; k++)
		{
			const b2ParticleContact& contact = m_partContactBuf[k];
			int32 a = contact.idxA;
			int32 b = contact.idxB;
			if (m_partGroupIdxBuffer[a] != m_partGroupIdxBuffer[b])
			{
				float32 w = contact.weight;
				float32 m = contact.mass;
				b2Vec2 n = contact.normal;
				float32 h = m_depthBuffer[a] + m_depthBuffer[b];
				b2Vec2 f = ejectionStrength * h * m * w * n;
				DistributeForce(a, b, f);
			}
		}
	}
}

void b2ParticleSystem::SolveForce()
{
	if (!m_hasForce) return;
	const b2TimeStep& step = m_subStep;

	if (m_accelerate)
	{
		auto& velocities = m_ampVelocities;
		auto& forces = m_ampForces;
		auto& matIdxs = m_ampMatIdxs;
		auto& matInvMasses = m_ampMatInvMasses;
		AmpForEachParticle([=, &velocities, &forces, &matIdxs,
			&matInvMasses](const int32 i) restrict(amp)
		{
			const float32 invMassStep = step.dt * matInvMasses[matIdxs[i]];
			velocities[i] += invMassStep * forces[i];
		});
	}
	else
	{
		for (int32 i = 0; i < m_count; i++)
		{
			float32 invMassStep = step.dt * m_partMatInvMassBuf[m_partMatIdxBuffer[i]];
			m_velocityBuffer[i] += invMassStep * m_forceBuffer[i];
		}
	}
	m_hasForce = false;
}

void b2ParticleSystem::SolveColorMixing()
{
	// mixes color between contacting particles
	b2Assert(m_colorBuffer.data());
	const int32 strength = (int32) (128 * m_def.colorMixingStrength);
	if (strength) {
		for (int32 k = 0; k < m_contactCount; k++)
		{
			const b2ParticleContact& contact = m_partContactBuf[k];
			int32 a = contact.idxA;
			int32 b = contact.idxB;
			if (m_flagsBuffer[a] & m_flagsBuffer[b] &
				b2_colorMixingParticle)
			{
				int32& colA = m_colorBuffer[a];
				int32& colB = m_colorBuffer[b];

				int8 ar = (colA & 0xFF000000) >> 24;
				int8 ag = (colA & 0x00FF0000) >> 16;
				int8 ab = (colA & 0x0000FF00) >>  8;
				int8 aa = (colA & 0x000000FF) >>  0;
				int8 br = (colB & 0xFF000000) >> 24;
				int8 bg = (colB & 0x00FF0000) >> 16;
				int8 bb = (colB & 0x0000FF00) >>  8;
				int8 ba = (colB & 0x000000FF) >>  0;
				const uint8 dr = (uint8)(strength * (br - ar));
				const uint8 dg = (uint8)(strength * (bg - ag));
				const uint8 db = (uint8)(strength * (bb - ab));
				const uint8 da = (uint8)(strength * (ba - aa));
				ar += dr;
				ag += dg;
				ab += db;
				aa += da;
				br -= dr;
				bg -= dg;
				bb -= db;
				ba -= da;
				colA = (ar << 24) | (ag << 16) | (ab << 8) | (aa);
				colB = (br << 24) | (bg << 16) | (bb << 8) | (ba);
			}
		}
	}
}


void b2ParticleSystem::SolveHeatConduct()
{
	if (!(m_world.m_allBodyMaterialFlags & b2_heatConductingMaterial)) return;

	const b2TimeStep& step = m_subStep;

	if (m_accelerate)
	{
		const uint32 matFilterFlag = b2_heatConductingMaterial;
		const uint32 extinguishingMatFlag = b2_extinguishingMaterial;
		auto& heats = m_ampHeats;
		auto& matIdxs = m_ampMatIdxs;
		auto& matFlags = m_ampMatFlags;
		auto& matHeatConductivities = m_ampMatHeatConductivities;
		auto& matMasses = m_ampMatMasses;
		auto& bodies = m_ampBodies;
		auto& bodyMats = m_world.m_ampBodyMaterials;
		AmpForEachBodyContact([=, &bodies, &matIdxs, &bodyMats,
			&matFlags, &matHeatConductivities, &matMasses, &heats](const b2PartBodyContact& contact) restrict(amp)
		{
			int32 a = contact.partIdx;
			Body& b = bodies[contact.bodyIdx];
			int32 matIdxP = matIdxs[a];
			b2BodyMaterial& matB = bodyMats[b.m_matIdx];
			if (!(matFlags[matIdxP] & matB.m_matFlags & b2_heatConductingMaterial)) return;
			float32 conductivityP = matHeatConductivities[matIdxP];
			float32 conductivityB = matB.m_heatConductivity;
			if (conductivityP <= 0 || conductivityB <= 0) return;
			float32& heatP = heats[a];
			float32& heatB = b.m_heat;
			if (b2Abs(heatB - heatP) < 1.0f) return;
			{
				float32 changeHeat = step.dt * 30.0f * (conductivityP * conductivityB)
					* (heatB - heatP);
				float32 massP = matMasses[matIdxP];
				float32 massB = b.m_mass;
				float32 invCombinedMass = 0.999f / (massP + massB);
				amp::atomicAdd(heatP, changeHeat * (0.001f + massB * invCombinedMass));
				amp::atomicSub(heatB, changeHeat * (0.001f + massP * invCombinedMass));

				/*float32 changeHeat = step.dt * (conductivityP * conductivityB)
					* (heatB - heatP);
				float32 massP = matP->m_mass;
				float32 massB = b.GetMass();
				float32 invCombinedMass = 0.5f / (massP + massB);
				m_heatBuffer[i] += changeHeat * (0.25f + massB * invCombinedMass);
				b.m_heat			 -= changeHeat * (0.25f + massP * invCombinedMass);*/
			}
		});
		AmpForEachContact([=, &heats, &matIdxs, &matFlags,
			&matHeatConductivities, &matMasses](const b2ParticleContact& contact) restrict(amp)
		{
			if (!(contact.matFlags & matFilterFlag)) return;
			const int32 a = contact.idxA;		//TODO include weight
			const int32 b = contact.idxB;
			const int32 matIdxA = matIdxs[a];
			const int32 matIdxB = matIdxs[b];
			const float32 conductivityA = matHeatConductivities[matIdxA];
			const float32 conductivityB = matHeatConductivities[matIdxB];
			if (!(conductivityA && conductivityB)) return;
			float32& heatA = heats[a];
			float32& heatB = heats[b];
			if ((heatA > heatB ? heatA - heatB : heatB - heatA) < 1.0f) return;
			const float32 changeHeat = step.dt * 30.0f * (conductivityA * conductivityB)
				* (heatB - heatA);
			const float32 massA = matMasses[matIdxA];
			const float32 massB = matMasses[matIdxB];
			const float32 invCombinedMass = 0.95f / (massA + massB);
			amp::atomicAdd(heatA, changeHeat * (0.05f + massB * invCombinedMass));
			amp::atomicSub(heatB, changeHeat * (0.05f + massA * invCombinedMass));
		});
	}
	else
	{
		for (int32 k = 0; k < m_contactCount; k++)
		{
			const b2ParticleContact& contact = m_partContactBuf[k];
			if (contact.matFlags & b2_heatConductingMaterial)
			{
				int32 a = contact.idxA;		//TODO include weight
				int32 b = contact.idxB;
				int32 matIdxA = m_partMatIdxBuffer[a];
				int32 matIdxB = m_partMatIdxBuffer[b];
				float32 conductivityA = m_partMatHeatConductivityBuf[matIdxA];
				float32 conductivityB = m_partMatHeatConductivityBuf[matIdxB];
				if (conductivityA > 0 && conductivityB > 0)
				{ 
					float32 heatA = m_heatBuffer[a];
					float32 heatB = m_heatBuffer[b];
					if (abs(heatA - heatB) > 1.0f)
					{
						float32 changeHeat = step.dt * 30.0f * (conductivityA * conductivityB)
							* (heatB - heatA);
						float32 massA = m_partMatMassBuf[matIdxA];
						float32 massB = m_partMatMassBuf[matIdxB];
						float32 invCombinedMass = 0.95f / (massA + massB);
						m_heatBuffer[a] += changeHeat * (0.05f + massB * invCombinedMass);
						m_heatBuffer[b] -= changeHeat * (0.05f + massA * invCombinedMass);
					}
				}
			}
		}

		// transfers heat to adjacent bodies
		for (int32 k = 0; k < m_bodyContactCount; k++)
		{
			const b2PartBodyContact& contact = m_bodyContactBuf[k];
			int32 a = contact.partIdx;
			Body& b = m_world.GetBody(contact.bodyIdx);
			int32 matIdxP = m_partMatIdxBuffer[a];
		
			b2BodyMaterial& matB = m_world.m_bodyMaterials[b.m_matIdx];
			if (m_partMatFlagsBuf[matIdxP] & matB.m_matFlags & b2_heatConductingMaterial)
			{
				float32 conductivityP = m_partMatHeatConductivityBuf[matIdxP];
				float32 conductivityB = matB.m_heatConductivity;
				if (conductivityP > 0 && conductivityB > 0)
				{
					float32 heatP = m_heatBuffer[a];
					float32 heatB = b.m_heat;
					if (abs(heatB - heatP) > 1.0f)
					{
						float32 changeHeat = step.dt * 30.0f * (conductivityP * conductivityB)
											 * (heatB - heatP);
						float32 massP = m_partMatMassBuf[matIdxP];
						float32 massB = b.m_mass;
						float32 invCombinedMass = 0.999f / (massP + massB);
						m_heatBuffer[a] += changeHeat * (0.001f + massB * invCombinedMass);
						b.m_heat		-= changeHeat * (0.001f + massP * invCombinedMass);

						/*float32 changeHeat = step.dt * (conductivityP * conductivityB)
							* (heatB - heatP);
						float32 massP = matP->m_mass;
						float32 massB = b.GetMass();
						float32 invCombinedMass = 0.5f / (massP + massB);
						m_heatBuffer[i] += changeHeat * (0.25f + massB * invCombinedMass);
						b.m_heat			 -= changeHeat * (0.25f + massP * invCombinedMass);*/
					}
				}
			}
		}
	}
}

void b2ParticleSystem::SolveLooseHeat()
{
	if (!(m_allParticleFlags & b2_heatLoosingParticle)) return;

	const b2TimeStep& step = m_subStep;

	if (m_accelerate)
	{
		auto& heats = m_ampHeats;
		auto& matIdxs  = m_ampMatIdxs;
		auto& matFlags = m_ampMatFlags;
		auto& matHeatConductivities = m_ampMatHeatConductivities;
		auto& matInvMasses			= m_ampMatInvMasses;
		const float32 roomTemp = m_world.m_roomTemperature;
		const float32 heatLossRatio = m_heatLossRatio;
		AmpForEachParticle(b2_heatLoosingParticle, [=, &heats, &matIdxs, &matFlags,
			&matHeatConductivities, &matInvMasses](const int32 i) restrict(amp)
		{
			const int32 matIdx = matIdxs[i];
			float32& heat = heats[i];
			const float32 loss = step.dt * matHeatConductivities[matIdx] * (heat - roomTemp);
			if (!loss) return;
			heat -= loss * (1 - __dp_math_powf(heatLossRatio,
					//(2.0f - (m_weightBuffer[k] < 0 ? 0 : m_weightBuffer[k] > 1 ? 1 : m_weightBuffer[k])) *
					0.0005f * matInvMasses[matIdx]));
		});
	}
	else
	{
		for (int32 k = 0; k < m_count; k++)
		{
			if (m_flagsBuffer[k] & b2_heatLoosingParticle)
			{
				int32 matIdx = m_partMatIdxBuffer[k];
				float32 loss = step.dt * m_partMatHeatConductivityBuf[matIdx] * (m_heatBuffer[k] - m_world.m_roomTemperature);
				if (abs(loss) > step.dt)
				{
					m_heatBuffer[k] -= loss * (1 - pow(m_heatLossRatio,
						//(2.0f - (m_weightBuffer[k] < 0 ? 0 : m_weightBuffer[k] > 1 ? 1 : m_weightBuffer[k])) *
						0.0005f * m_partMatInvMassBuf[matIdx]));
				}
			}
		}
	}
}

void b2ParticleSystem::CopyHeats()
{
	if (m_accelerate)
		m_ampCopyFutHeats.set(amp::copyAsync(m_ampHeats, m_heatBuffer, m_count));
}
void b2ParticleSystem::CopyFlags()
{
	if (m_accelerate)
		m_ampCopyFutFlags.set(amp::copyAsync(m_ampFlags, m_flagsBuffer, m_count));
}

void b2ParticleSystem::SolveFlame()
{
	if (!(m_allParticleFlags & b2_flameParticle)) return;

	const b2TimeStep& step = m_subStep;

	if (m_accelerate)
	{
		const uint32 zombieFlag = b2_zombieParticle;
		auto& flags = m_ampFlags;
		auto& matIdxs = m_ampMatIdxs;
		auto& heats = m_ampHeats;
		auto& healths = m_ampHealths;
		auto& matInvStabilities = m_ampMatInvStabilities;
		auto& matColderThans = m_ampMatColderThans;
		AmpForEachParticle(b2_flameParticle, [=, &flags, &matIdxs, &heats, &healths,
			&matInvStabilities, &matColderThans](const int32 i) restrict(amp)
		{
			float32& heat = heats[i];
			const int32 partMatIdx = matIdxs[i];
			const float32 loss = step.dt * 0.001f * heat * matInvStabilities[partMatIdx];
			healths[i] -= loss;
			if (heat < matColderThans[partMatIdx])
				flags[i] = zombieFlag;
			else
				heat += loss * 1000.0f;
		});
	}
	else
	{
		for (int32 k = 0; k < m_count; k++)
		{
			if (m_flagsBuffer[k] & b2_flameParticle)
			{
				float32& heat = m_heatBuffer[k];

				// loose health
				const int32 partMatIdx = m_partMatIdxBuffer[k];
				const float32 loss = step.dt * heat * m_partMatInvStabilityBuf[partMatIdx];
				if (loss > FLT_EPSILON)
					m_healthBuffer[k] -= loss;
				if (heat < m_partMatColderThanBuf[partMatIdx])
					DestroyParticle(k);
				else
				{
					// rise heat
					heat += loss * 1000.0f;
				}
			}
		}
	}
	m_allParticleFlags |= b2_zombieParticle;
}

void b2ParticleSystem::SolveIgnite()
{
	if (!(m_allParticleFlags & b2_flameParticle &&
		m_world.m_allBodyMaterialFlags & b2_flammableMaterial))
		return;

	const uint32 flameFlag = b2_flameParticle;
	const uint32 burningFlag = b2_burningParticle;
	const uint32 burningBodyFlag = b2_burningBody;
	const uint32 flameableMatFlag = b2_flammableMaterial;
	const uint32 inflammableBodyFlag = b2_inflammableBody;

	if (m_accelerate)
	{
		auto& heats = m_ampHeats;
		auto& flags = m_ampFlags;
		auto& matIdxs = m_ampMatIdxs;
		auto& matFlags = m_ampMatFlags;
		auto& matIgnitionPoints = m_ampMatIgnitionPoints;
		auto& bodies = m_ampBodies;
		auto& bodyMats = m_world.m_ampBodyMaterials;
		AmpForEachBodyContact([=, &bodies, &bodyMats, &flags](const b2PartBodyContact& contact) restrict(amp)
		{
			int32 a = contact.partIdx;
			Body& b = bodies[contact.bodyIdx];
			b2BodyMaterial& mat = bodyMats[b.m_matIdx];
			if (flags[a] & flameFlag
				&& b.m_flags & inflammableBodyFlag
				&& mat.m_matFlags & flameableMatFlag
				&& mat.m_ignitionPoint <= b.m_heat)
			{
				b.AddFlags(burningBodyFlag);
			}
		});
		AmpForEachContact([=, &heats, &flags, &matIdxs, &matFlags, 
			&matIgnitionPoints](const b2ParticleContact& contact) restrict(amp)
		{
			if (!(contact.flags & flameFlag && contact.matFlags & flameableMatFlag)) return;
	
			const int32 a = contact.idxA;
			const int32 b = contact.idxB;
			const int32 matIdxA = matIdxs[a];
			const int32 matIdxB = matIdxs[b];
			if ((flags[b] & flameFlag) & ~flags[a]
				&& matFlags[matIdxA] & flameableMatFlag
				&& matIgnitionPoints[matIdxA] <= heats[a])
			{
				flags[a] |= burningFlag;
			}
			else if (matFlags[matIdxB] & flameableMatFlag
				&& (flags[a] & flameFlag) & ~flags[b]
				&& matIgnitionPoints[matIdxB] <= heats[b])
			{
				flags[b] |= burningFlag;
			}
		});
	}
	else
	{
		for (int32 k = 0; k < m_bodyContactCount; k++)
		{
			const b2PartBodyContact& contact = m_bodyContactBuf[k];
			int32 a = contact.partIdx;
			Body& b = m_world.GetBody(contact.bodyIdx);
			b2BodyMaterial& mat = m_world.m_bodyMaterials[b.m_matIdx];
			if (m_flagsBuffer[a] & b2_flameParticle
				&& b.m_flags & b2_inflammableBody
				&& mat.m_matFlags & b2_flammableMaterial
				&& mat.m_ignitionPoint <= b.m_heat)
			{
				b.AddFlags((uint16)b2_burningBody);
			}
		}

		for (int32 k = 0; k < m_contactCount; k++)
		{
			const b2ParticleContact& contact = m_partContactBuf[k];
			if (contact.flags & b2_flameParticle && contact.matFlags & b2_flammableMaterial)
			{
				int32 a = contact.idxA;
				int32 b = contact.idxB;
				int32 matIdxA = m_partMatIdxBuffer[a];
				int32 matIdxB = m_partMatIdxBuffer[b];
				if (m_partMatFlagsBuf[matIdxA] & b2_flammableMaterial
					&& (m_flagsBuffer[b] & b2_flameParticle) & ~m_flagsBuffer[a]
					&& m_partMatIgnitionPointBuf[matIdxA] <= m_heatBuffer[a])
				{
					m_flagsBuffer[a] |= b2_burningParticle;
				}
				else if (m_partMatFlagsBuf[matIdxB] & b2_flammableMaterial
					&& (m_flagsBuffer[a] & b2_flameParticle) & ~m_flagsBuffer[b]
					&& m_partMatIgnitionPointBuf[matIdxB] <= m_heatBuffer[b])
				{
					m_flagsBuffer[b] |= b2_burningParticle;
				}
			}
		}
	}
}

void b2ParticleSystem::SolveExtinguish()
{
	if (m_allParticleFlags & b2_flameParticle &&
		m_world.m_allBodyMaterialFlags & b2_extinguishingMaterial) return;
	
	const uint32 burningFlag = b2_burningParticle;
	const uint32 burningBodyFlag = b2_burningBody;
	const uint32 wetBodyFlag = b2_wetBody;
	const uint32 extinguishingMatFlag = b2_extinguishingMaterial;

	if (m_accelerate)
	{
		auto& flags = m_ampFlags;
		auto& matIdxs = m_ampMatIdxs;
		auto& matFlags = m_ampMatFlags;
		auto& bodies = m_ampBodies;
		auto& matIgnitionPoints = m_ampMatIgnitionPoints;
		AmpForEachBodyContact([=, &bodies, &matIdxs, &matFlags,
			&matIgnitionPoints](const b2PartBodyContact& contact) restrict(amp)
		{
			int32 a = contact.partIdx;
			Body& b = bodies[contact.bodyIdx];
			int32 partMatIdx = matIdxs[a];
			if (matFlags[partMatIdx] & extinguishingMatFlag
				&& b.m_flags & burningBodyFlag
				&& b.m_heat < matIgnitionPoints[partMatIdx])
			{
				b.RemFlags(burningBodyFlag);
				b.AddFlags(wetBodyFlag);
			}
		});
		AmpForEachContact([=, &flags, &matIdxs, &matFlags](const b2ParticleContact& contact) restrict(amp)
		{
			if (!(contact.flags & burningFlag && contact.matFlags & extinguishingMatFlag)) return;
			const int32 a = contact.idxA;
			const int32 b = contact.idxB;
			if (matFlags[matIdxs[a]] & extinguishingMatFlag
				&& flags[b] & burningFlag)
				flags[b] &= ~burningFlag;

			else if (matFlags[matIdxs[b]] & extinguishingMatFlag
				&& flags[a] & burningFlag)
				flags[a] &= ~burningFlag;
		});
	}
	else
	{
		for (int32 k = 0; k < m_bodyContactCount; k++)
		{
			const b2PartBodyContact& contact = m_bodyContactBuf[k];
			int32 a = contact.partIdx;
			Body& b = m_world.GetBody(contact.bodyIdx);
			int32 partMatIdx = m_partMatIdxBuffer[a];
			if (m_partMatFlagsBuf[partMatIdx] & b2_extinguishingMaterial
				&& b.m_flags & b2_burningBody
				&& b.m_heat < m_partMatIgnitionPointBuf[partMatIdx])
			{
				b.RemFlags(b2_burningBody);
				b.AddFlags(b2_wetBody);
			}
		}

		for (int32 k = 0; k < m_contactCount; k++)
		{
			const b2ParticleContact& contact = m_partContactBuf[k];
			if (contact.flags & b2_burningParticle && contact.matFlags & b2_extinguishingMaterial)
			{
				int32 a = contact.idxA;
				int32 b = contact.idxB;
				if (m_partMatFlagsBuf[m_partMatIdxBuffer[a]] & b2_extinguishingMaterial
					&& m_flagsBuffer[b] & b2_burningParticle)
					m_flagsBuffer[b] &= ~b2_burningParticle;

				else if (m_partMatFlagsBuf[m_partMatIdxBuffer[b]] & b2_extinguishingMaterial
					&& m_flagsBuffer[a] & b2_burningParticle)
					m_flagsBuffer[a] &= ~b2_burningParticle;
			}
		}
	}
}

void b2ParticleSystem::CopyHealths()
{
	if (m_accelerate)
		m_ampCopyFutHealths.set(amp::copyAsync(m_ampHealths, m_healthBuffer, m_count));
}

void b2ParticleSystem::SolveWater()
{
	// make Objects wet
	for (int32 k = 0; k < m_bodyContactCount; k++)
	{
		const b2PartBodyContact& contact = m_bodyContactBuf[k];
		int32 a = contact.partIdx;
		Body& b = m_world.GetBody(contact.bodyIdx);
		b2BodyMaterial& mat = m_world.m_bodyMaterials[b.m_matIdx];
		if (m_flagsBuffer[a] & b2_extinguishingMaterial
			&& b.m_flags & b2_burningBody
			&& mat.m_matFlags & b2_flammableMaterial
			&& mat.m_extinguishingPoint > b.m_heat)
		{
			b.RemFlags((int16)b2_burningBody);
		}
	}
}
void b2ParticleSystem::SolveAir()
{
	if (!(m_allParticleFlags & b2_airParticle)) return;

	if (m_accelerate)
	{
		const uint32 zombieFlag = b2_zombieParticle;
		auto& flags = m_ampFlags;
		auto& velocities = m_ampVelocities;
		AmpForEachParticle(b2_airParticle, [=, &flags, &velocities](const int32 i) restrict(amp)
		{
			if (velocities[i].Length() < 0.1f)
				flags[i] = zombieFlag;
		});
	}
	else
	{
		for (int32 k = 0; k < m_count; k++)
		{
			// delete if stopped
			if (m_flagsBuffer[k] & b2_airParticle)
			{
				if (m_velocityBuffer[k].Length() < 0.1f)
					DestroyParticle(k);
			}

			// TODO blow out burning
		}
	}
}

void b2ParticleSystem::SolveChangeMat()
{
	if (m_accelerate)
	{
		const uint32 matColdFilterFlag = b2_changeWhenColdMaterial;
		const uint32 matHotFilterFlag = b2_changeWhenHotMaterial;
		const uint32 zombieFlag = b2_zombieParticle;
		auto& flags = m_ampFlags;
		auto& heats = m_ampHeats;
		auto& matIdxs = m_ampMatIdxs;
		auto& matColderThans = m_ampMatColderThans;
		auto& matHotterThans = m_ampMatHotterThans;
		auto& matChangeToColdMats = m_ampMatChangeToColdMats;
		auto& matChangeToHotMats = m_ampMatChangeToHotMats;
		auto& matPartFlags = m_ampMatPartFlags;
		auto& matFlags = m_ampMatFlags;
		AmpForEachParticle([=, &flags, &heats, &matIdxs, &matColderThans, &matHotterThans,
			&matChangeToColdMats, &matChangeToHotMats, &matPartFlags, &matFlags](const int32 i) restrict(amp)
		{
			int32& matIdx = matIdxs[i];
			const uint32 matFlag = matFlags[matIdx];
			const float32 heat = heats[i];
			if (matFlag & matColdFilterFlag && heat < matColderThans[matIdx])
			{
				const int32 newMatIdx = matChangeToColdMats[matIdx];
				if (newMatIdx != -1)
				{
					matIdx = newMatIdx;
					flags[i] = matPartFlags[newMatIdx];
				}
				else
					flags[i] = zombieFlag;
			}
			if (matFlag & matHotFilterFlag && heat > matHotterThans[matIdx])
			{
				const int32 newMatIdx = matChangeToHotMats[matIdx];
				if (newMatIdx != -1)
				{
					matIdx = newMatIdx;
					flags[i] = matPartFlags[newMatIdx];
				}
				else
					flags[i] = zombieFlag;
			}
		});
		m_ampCopyFutMatIdxs.set(amp::copyAsync(m_ampMatIdxs, m_partMatIdxBuffer, m_count));
	}
	else
	{
		for (int32 k = 0; k < m_count; k++)
		{
			int32 matIdx = m_partMatIdxBuffer[k];
			uint32 matFlags = m_partMatFlagsBuf[matIdx];
			if (matFlags & b2_changeWhenColdMaterial)
			{
				if (m_heatBuffer[k] < m_partMatColderThanBuf[matIdx])
				{
					int32 newMat = m_partMatChangeToColdMatBuf[matIdx];
					if (newMat != b2_invalidIndex)
					{
						m_partMatIdxBuffer[k] = newMat;
						SetParticleFlags(k, m_partMatPartFlagsBuf[newMat]);
					}
					else
						DestroyParticle(k);
				}
			}
			if (matFlags & b2_changeWhenHotMaterial)
			{
				if (m_heatBuffer[k] > m_partMatHotterThanBuf[matIdx])
				{
					int32 newMat = m_partMatChangeToHotMatBuf[matIdx];
					if (newMat != b2_invalidIndex)
					{
						m_partMatIdxBuffer[k] = newMat;
						SetParticleFlags(k, m_partMatPartFlagsBuf[newMat]);
					}
					else
						DestroyParticle(k);
				}
			}
		}
	}
}

void b2ParticleSystem::SolveFreeze()
{
	
}

void b2ParticleSystem::SolvePosition()
{
	const b2TimeStep& step = m_subStep;
	
	if (m_accelerate)
	{
		auto& positions = m_ampPositions;
		auto& velocities = m_ampVelocities;
		AmpForEachParticle([=, &positions, &velocities](const int32 i) restrict(amp)
		{
			positions[i] += step.dt * velocities[i];
		});
		m_ampCopyFutPositions.set(amp::copyAsync(m_ampPositions, m_positionBuffer, m_count));
	}
	else
	{
		for (int k = 0; k < m_count; k++)
			m_positionBuffer[k] += step.dt * m_velocityBuffer[k];
	}
}

void b2ParticleSystem::IncrementIteration()
{
	m_iteration++;
}

void b2ParticleSystem::SolveHealth()
{
	if (m_accelerate)
	{
		const uint32 zombieFlag = b2_zombieParticle;
		auto& flags = m_ampFlags;
		auto& healths = m_ampHealths;
		AmpForEachParticle([=, &flags, &healths](const int32 i) restrict(amp)
		{
			if (healths[i] < 0)
				flags[i] = zombieFlag;
		});
		//m_ampCopyFutFlags.set(amp::copyAsync(m_ampFlags, m_flagsBuffer, m_count));
	}
	else
	{
		for (int32 k = 0; k < m_count; k++)
		{
			if (m_healthBuffer[k] < 0)
				DestroyParticle(k);
				//m_flagsBuffer[k] |= b2_zombieParticle;
		}
	}
}

void b2ParticleSystem::CopyBodies()
{
	if (m_accelerate && m_bodyContactCount)
		m_ampCopyFutBodies.set(amp::copyAsync(m_ampBodies, m_world.m_bodyBuffer, m_world.m_bodyBuffer.size()));
}

template <class T1, class UnaryPredicate> 
static void b2ParticleSystem::RemoveFromVectorIf(vector<T1>& v1,
	int32& size, UnaryPredicate pred, bool adjustSize)
{
	int newI = 0;
	for (int i = 0; i < size; i++)
	{
		if (!pred(v1[i]))
		{
			v1[newI] = move(v1[i]);
			newI++;
		}
	}
	if (adjustSize)
	{
		size = newI;
	}
}
//void b2ParticleSystem::AmpRemoveZombieContacts()
//{
//	ampArrayView<const b2ParticleContact> contacts(m_ampContacts);
//	const uint32 tileCnt = m_contactCount / TILE_SIZE;
//	ampArrayView<uint32> invalidCnts(tileCnt);
//	ampExtent e(m_contactCount);
//	
//	Concurrency::parallel_for_each(ampExtent(m_contactCount).tile<TILE_SIZE>(), [=](ampTiledIdx<TILE_SIZE> tIdx) restrict(amp)
//	{
//		tile_static uint32 invalid[TILE_SIZE];
//		tile_static uint32 invalidCnt;
//		if (tIdx.local[0] == 0) invalidCnt = 0;
//		tIdx.barrier.wait_with_tile_static_memory_fence();
//
//		if ((contacts[tIdx.global].flags & b2_zombieParticle) == b2_zombieParticle)
//			invalid[Concurrency::atomic_fetch_add(&invalidCnt, 1)];
//
//		tIdx.barrier.wait_with_tile_static_memory_fence();
//		invalidCnts[tIdx.tile] = invalidCnt;
//	});
//	Concurrency::parallel_for_each(ampExtent(tileCnt).tile<TILE_SIZE>(), [=](ampTiledIdx<TILE_SIZE> tIdx) restrict(amp)
//	{
//
//	});
//	int newI = 0;
//	for (int i = 0; i < size; i++)
//	{
//		if (!pred(v1[i]))
//		{
//			v1[newI] = move(v1[i]);
//			newI++;
//		}
//	}
//	if (adjustSize)
//	{
//		size = newI;
//	}
//}
template <class T1, class T2, class UnaryPredicate>
static void b2ParticleSystem::RemoveFromVectorsIf(vector<T1>& v1, vector<T2>& v2,
	int32& size, UnaryPredicate pred, bool adjustSize)
{
	int newI = 0;
	for (int i = 0; i < size; i++)
	{
		if (!pred(v1[i]))
		{
			v1[newI] = move(v1[i]);
			v2[newI] = move(v2[i]);
			newI++;
		}
	}
	if (adjustSize)
	{
		size = newI;
	}
}
template <class T1, class T2, class T3, class T4, class T5, class T6, class T7, class UnaryPredicate>
static void b2ParticleSystem::RemoveFromVectorsIf(
	vector<T1>& v1, vector<T2>& v2, vector<T3>& v3, vector<T4>& v4, vector<T5>& v5, vector<T6>& v6, vector<T7>& v7,
	int32& size, UnaryPredicate pred, bool adjustSize)
{
	int newI = 0;
	for (int i = 0; i < size; i++)
	{
		if (!pred(v1[i]))
		{
			v1[newI] = move(v1[i]);
			v2[newI] = move(v2[i]);
			v3[newI] = move(v3[i]);
			v4[newI] = move(v4[i]);
			v5[newI] = move(v5[i]);
			v6[newI] = move(v6[i]);
			v7[newI] = move(v7[i]);
			newI++;
		}
	}
	if (adjustSize)
	{
		size = newI;
	}
}
template <class T1, class T2, class T3, class T4, class T5, class T6, class T7, class T8, class UnaryPredicate>
static void b2ParticleSystem::RemoveFromVectorsIf(
	vector<T1>& v1, vector<T2>& v2, vector<T3>& v3, vector<T4>& v4, vector<T5>& v5, vector<T6>& v6, vector<T7>& v7, vector<T8>& v8,
	int32& size, UnaryPredicate pred, bool adjustSize)
{
	int newI = 0;
	for (int i = 0; i < size; i++)
	{
		if (!pred(v1[i]))
		{
			v1[newI] = move(v1[i]);
			v2[newI] = move(v2[i]);
			v3[newI] = move(v3[i]);
			v4[newI] = move(v4[i]);
			v5[newI] = move(v5[i]);
			v6[newI] = move(v6[i]);
			v7[newI] = move(v7[i]);
			v8[newI] = move(v8[i]);
			newI++;
		}
	}
	if (adjustSize)
	{
		size = newI;
	}
}
template <class T1, class T2, class T3, class T4, class T5, class T6, class T7, class T8, class UnaryPredicate1, class UnaryPredicate2> 
static void b2ParticleSystem::RemoveFromVectorsIf(
	vector<T1>& v1, vector<T2>& v2, vector<T3>& v3, vector<T4>& v4, vector<T5>& v5, vector<T6>& v6, vector<T7>& v7, vector<T8>& v8,
	int32& size, UnaryPredicate1 pred1, UnaryPredicate2 pred2, bool adjustSize)
{
	int newI = 0;
	for (int i = 0; i < size; i++)
	{
		if (!(pred1(v1[i]) || pred2(v2[i])))
		{
			v1[newI] = move(v1[i]);
			v2[newI] = move(v2[i]);
			v3[newI] = move(v3[i]);
			v4[newI] = move(v4[i]);
			v5[newI] = move(v5[i]);
			v6[newI] = move(v6[i]);
			v7[newI] = move(v7[i]);
			v8[newI] = move(v8[i]);
			newI++;
		}
	}
	if (adjustSize)
	{
		size = newI;
	}
}

void b2ParticleSystem::SolveZombie()
{
	// removes particles with zombie flag
	int32 newCount = 0;
	vector<int32> newIndices(m_count);
	uint32 allParticleFlags = 0;
	for (int32 i = 0; i < m_count; i++)
	{
		int32 flags = m_flagsBuffer[i];
		if (flags & b2_zombieParticle)
		{
			b2DestructionListener * const destructionListener =
				m_world.m_destructionListener;
			if ((flags & b2_destructionListenerParticle) &&
				destructionListener)
			{
				destructionListener->SayGoodbye(this, i);
			}
			// Destroy particle handle.
			if (!m_handleIndexBuffer.empty())
			{
				b2ParticleHandle * const handle = m_handleIndexBuffer[i];
				if (handle)
				{
					handle->SetIndex(b2_invalidIndex);
					m_handleIndexBuffer[i] = NULL;
					m_handleAllocator.Free(handle);
				}
			}
			newIndices[i] = b2_invalidIndex;
		}
		else
		{
			newIndices[i] = newCount;
			if (i != newCount)
			{
				// Update handle to reference new particle index.
				if (!m_handleIndexBuffer.empty())
				{
					b2ParticleHandle * const handle =
						m_handleIndexBuffer[i];
					if (handle) handle->SetIndex(newCount);
					m_handleIndexBuffer[newCount] = handle;
				}
				m_flagsBuffer[newCount] = m_flagsBuffer[i];
				if (!m_lastBodyContactStepBuffer.empty())
					m_lastBodyContactStepBuffer[newCount] =
						m_lastBodyContactStepBuffer[i];
				
				if (!m_bodyContactCountBuffer.empty())
					m_bodyContactCountBuffer[newCount] =
						m_bodyContactCountBuffer[i];
				
				if (!m_consecutiveContactStepsBuffer.empty())
					m_consecutiveContactStepsBuffer[newCount] =
						m_consecutiveContactStepsBuffer[i];
				
				m_positionBuffer[newCount]  = m_positionBuffer[i];
				m_velocityBuffer[newCount]  = m_velocityBuffer[i];
				m_partGroupIdxBuffer[newCount]   = m_partGroupIdxBuffer[i];
				m_partMatIdxBuffer[newCount] = m_partMatIdxBuffer[i];
				if (m_hasForce)
					m_forceBuffer[newCount] = m_forceBuffer[i];
				
				if (!m_staticPressureBuf.empty())
					m_staticPressureBuf[newCount] =
						m_staticPressureBuf[i];
				
				if (m_hasDepth)
					m_depthBuffer[newCount] = m_depthBuffer[i];
				
				if (m_colorBuffer.data())
					m_colorBuffer[newCount] = m_colorBuffer[i];
				
				m_heatBuffer[newCount] = m_heatBuffer[i];
				m_healthBuffer[newCount] = m_healthBuffer[i];

				if (!m_expireTimeBuf.empty())
					m_expireTimeBuf[newCount] = m_expireTimeBuf[i];
				
			}
			newCount++;
			allParticleFlags |= flags;
		}
	}

	// predicate functions
	struct Test
	{
		static bool IsProxyInvalid(const Proxy& proxy)
		{
			return proxy.idx < 0;
		}
		static bool IsContactIdxInvalid(const b2ParticleContact& contact)
		{
			return contact.idxA < 0 || contact.idxB < 0;
		}
		static bool IsBodyContactInvalid(const b2PartBodyContact& contact)
		{
			return contact.partIdx < 0;
		}
		static bool IsPairInvalid(const b2ParticlePair& pair)
		{
			return pair.indexA < 0 || pair.indexB < 0;
		}
		static bool IsTriadInvalid(const b2ParticleTriad& triad)
		{
			return triad.indexA < 0 || triad.indexB < 0 || triad.indexC < 0;
		}
	};

	// update proxies
	for (int32 k = 0; k < m_count; k++)
	{
		Proxy& proxy = m_proxyBuffer.data()[k];
		proxy.idx = newIndices[proxy.idx];
	}
	RemoveFromVectorIf(m_proxyBuffer, m_count, Test::IsProxyInvalid, false);

	// update contacts
	for (int32 k = 0; k < m_contactCount; k++)
	{
		b2ParticleContact& contact = m_partContactBuf[k];
		contact.idxA = newIndices[contact.idxA];
		contact.idxB = newIndices[contact.idxB];
	}
	RemoveFromVectorIf(m_partContactBuf, m_contactCount, Test::IsContactIdxInvalid, true);

	// update particle-body contacts
	for (int32 k = 0; k < m_bodyContactCount; k++)
	{
		int32& partIdx = m_bodyContactBuf[k].partIdx;
		partIdx = newIndices[partIdx];
	}
	RemoveFromVectorIf(m_bodyContactBuf, m_bodyContactCount, Test::IsBodyContactInvalid, true);

	// update pairs
	for (int32 k = 0; k < m_pairCount; k++)
	{
		b2ParticlePair& pair = m_pairBuffer[k];
		pair.indexA = newIndices[pair.indexA];
		pair.indexB = newIndices[pair.indexB];
	}
	RemoveFromVectorIf(m_pairBuffer, m_pairCount, Test::IsPairInvalid, true);

	// update triads
	for (int32 k = 0; k < m_triadCount; k++)
	{
		b2ParticleTriad& triad = m_triadBuffer[k];
		triad.indexA = newIndices[triad.indexA];
		triad.indexB = newIndices[triad.indexB];
		triad.indexC = newIndices[triad.indexC];
	}
	RemoveFromVectorIf(m_triadBuffer, m_triadCount, Test::IsTriadInvalid, true);

	// Update lifetime indices.
	if (!m_idxByExpireTimeBuf.empty())
	{
		int32 writeOffset = 0;
		for (int32 readOffset = 0; readOffset < m_count; readOffset++)
		{
			const int32 newIndex = newIndices[
				m_idxByExpireTimeBuf[readOffset]];
			if (newIndex != b2_invalidIndex)
			{
				m_idxByExpireTimeBuf[writeOffset++] = newIndex;
			}
		}
	}

	// update groups
	for (int32 k = 0; k < m_groupCount; k++)
	{
		b2ParticleGroup& group = m_groupBuffer[k];
		if (group.m_firstIndex != b2_invalidIndex)
		{
			int32 firstIndex = newCount;
			int32 lastIndex = 0;
			bool modified = false;
			for (int32 i = group.m_firstIndex; i < group.m_lastIndex; i++)
			{
				int32 j = newIndices[i];
				if (j >= 0) {
					firstIndex = b2Min(firstIndex, j);
					lastIndex = b2Max(lastIndex, j + 1);
				}
				else {
					modified = true;
				}
			}
			if (firstIndex < lastIndex)
			{
				group.m_firstIndex = firstIndex;
				group.m_lastIndex = lastIndex;
				if (modified)
				{
					if (group.m_groupFlags & b2_solidParticleGroup)
					{
						SetGroupFlags(group,
							group.m_groupFlags |
							b2_particleGroupNeedsUpdateDepth);
					}
				}
			}
			else
			{
				group.m_firstIndex = 0;
				group.m_lastIndex = 0;
				if (!(group.m_groupFlags & b2_particleGroupCanBeEmpty))
				{
					SetGroupFlags(group,
						group.m_groupFlags | b2_particleGroupWillBeDestroyed);
				}
			}
		}
	}

	// update particle count
	m_count = newCount;
	m_needsUpdateAllParticleFlags = true;

	// destroy bodies with no particles
	for (int32 k = 0; k < m_groupCount; k++)
	{
		if (m_groupBuffer[k].m_firstIndex != b2_invalidIndex)
		{
			if (m_groupBuffer[k].m_groupFlags & b2_particleGroupWillBeDestroyed)
				DestroyGroup(k);
		}
	}
}

void b2ParticleSystem::AmpSolveZombie()
{
	if (!m_count) return;
	auto& flags = m_ampFlags;
	auto& groupIdxs = m_ampGroupIdxs;
	ampArrayView<uint32> groupHasAlive(m_groupCount);
	amp::fill(groupHasAlive, 0u);
	AmpForEachParticle([=, &groupIdxs](const int32 i) restrict(amp)
	{
		groupHasAlive[groupIdxs[i]] = 1;
	});

	// add new dead groups to zombieRanges
	bool groupWasDestroyed = false;
	for (int32 i = 0; i < m_groupCount; i++)
	{
		auto& group = m_groupBuffer[i];
		if (!groupHasAlive[i] && group.m_firstIndex != b2_invalidIndex)
		{
			DestroyGroup(i);
			groupWasDestroyed = true;
		}
	}
	if (groupWasDestroyed)
		amp::copy(m_groupBuffer, m_ampGroups, m_groupCount);
}

void b2ParticleSystem::AddZombieRange(int32 firstIdx, int32 lastIdx)
{
	// at the end of the Particle Buffers
	if (m_count == lastIdx)
	{
		m_count = firstIdx;
		if (!m_zombieRanges.empty() && m_count == m_zombieRanges.back().second)	// connected to previous
		{
			m_count = m_zombieRanges.back().first;
			m_zombieRanges.pop_back();
		}
		return;
	}
	// merge with prev or next range if possible
	for (auto curr = m_zombieRanges.begin(), prev = m_zombieRanges.begin(); curr != m_zombieRanges.end(); prev = curr, curr++)
	{
		if (firstIdx < curr->first)
		{
			if (prev->second == firstIdx)
			{
				if (curr->first == lastIdx)	// connects two existing ranges
				{
					prev->second = curr->second;
					m_zombieRanges.erase(curr);
				}
				else						// connected to previous range
					prev->second = lastIdx;
			}
			else if (curr->first == lastIdx)	// connected to next range
				curr->first = firstIdx;
			else								// connected to no range, but between existing ones
				m_zombieRanges.insert(curr, pair(firstIdx, lastIdx));
			return;
		}
	}
	// if at the end, join with previous if connected or create new
	if (!m_zombieRanges.empty() && m_zombieRanges.back().second == firstIdx)	
		m_zombieRanges.back().second = lastIdx;
	else
		m_zombieRanges.push_back(pair(firstIdx, lastIdx));
}

uint32 b2ParticleSystem::GetWriteIdx(int32 particleCnt)
{
	uint32 writeIdx;
	for (auto zombieRange = m_zombieRanges.begin(); zombieRange != m_zombieRanges.end(); zombieRange++)
	{
		int32 remainingSpace = zombieRange->second - zombieRange->first - particleCnt;
		if (remainingSpace >= 0)
		{
			writeIdx = zombieRange->first;
			if (remainingSpace == 0)
				m_zombieRanges.erase(zombieRange);
			else
				zombieRange->first += particleCnt;
			return writeIdx;
		}
	}
	writeIdx = m_count;
	ResizeParticleBuffers(m_count + particleCnt);
	m_count += particleCnt;
	return writeIdx;
}

/// Destroy all particles which have outlived their lifetimes set by
/// SetParticleLifetime().
void b2ParticleSystem::SolveLifetimes(const b2TimeStep& step)
{
	b2Assert(!m_expireTimeBuf.empty());
	b2Assert(!m_idxByExpireTimeBuf.empty());
	// Update the time elapsed.
	m_timeElapsed = LifetimeToExpirationTime(step.dt);
	// Get the floor (non-fractional component) of the elapsed time.
	const int32 quantizedTimeElapsed = GetQuantizedTimeElapsed();

	// Sort the lifetime buffer if it's required.
	if (m_expirationTimeBufferRequiresSorting)
	{
		const ExpirationTimeComparator expirationTimeComparator(
			m_expireTimeBuf.data());
		//std::sort(expirationTimeIndices,
		//		  expirationTimeIndices + particleCount,
		//		  expirationTimeComparator);
		Concurrency::parallel_sort(m_idxByExpireTimeBuf.begin(),
			m_idxByExpireTimeBuf.begin() + m_count,
					  expirationTimeComparator);
		m_expirationTimeBufferRequiresSorting = false;
	}

	// Destroy particles which have expired.
	for (int32 i = m_count - 1; i >= 0; --i)
	{
		const int32 particleIndex = m_idxByExpireTimeBuf[i];
		const int32 expirationTime = m_expireTimeBuf[particleIndex];
		// If no particles need to be destroyed, skip this.
		if (quantizedTimeElapsed < expirationTime || expirationTime <= 0)
		{
			break;
		}
		// Destroy this particle.
		DestroyParticle(particleIndex);
	}
}

void b2ParticleSystem::RotateBuffer(int32 start, int32 mid, int32 end)
{
	// move the particles assigned to the given group toward the end of array
	if (start == mid || mid == end)
	{
		return;
	}
	b2Assert(mid >= start && mid <= end);
	struct NewIndices
	{
		int32 operator[](int32 i) const
		{
			if (i < start)
			{
				return i;
			}
			else if (i < mid)
			{
				return i + end - mid;
			}
			else if (i < end)
			{
				return i + start - mid;
			}
			else
			{
				return i;
			}
		}
		int32 start, mid, end;
	} newIndices;
	newIndices.start = start;
	newIndices.mid = mid;
	newIndices.end = end;

	rotate(m_flagsBuffer.begin() + start, m_flagsBuffer.begin() + mid,
				m_flagsBuffer.begin() + end);
	if (!m_lastBodyContactStepBuffer.empty())
		rotate(m_lastBodyContactStepBuffer.begin() + start,
					m_lastBodyContactStepBuffer.begin() + mid,
					m_lastBodyContactStepBuffer.begin() + end);
	
	if (!m_bodyContactCountBuffer.empty())
		rotate(m_bodyContactCountBuffer.begin() + start,
					m_bodyContactCountBuffer.begin() + mid,
					m_bodyContactCountBuffer.begin() + end);
	
	if (!m_consecutiveContactStepsBuffer.empty())
		rotate(m_consecutiveContactStepsBuffer.begin() + start,
					m_consecutiveContactStepsBuffer.begin() + mid,
					m_consecutiveContactStepsBuffer.begin() + end);
	
	rotate(m_positionBuffer.data() + start, m_positionBuffer.data() + mid,
		m_positionBuffer.data() + end);
	rotate(m_velocityBuffer.data() + start, m_velocityBuffer.data() + mid,
		m_velocityBuffer.data() + end);
	rotate(m_partGroupIdxBuffer.begin() + start, m_partGroupIdxBuffer.begin() + mid,
		m_partGroupIdxBuffer.begin() + end);
	rotate(m_partMatIdxBuffer.begin() + start, m_partMatIdxBuffer.begin() + mid,
		m_partMatIdxBuffer.begin() + end);
	if (m_hasForce)
		rotate(m_forceBuffer.data() + start, m_forceBuffer.data() + mid,
			m_forceBuffer.data() + end);
	
	if (!m_staticPressureBuf.empty())
		rotate(m_staticPressureBuf.begin() + start,
			m_staticPressureBuf.begin() + mid,
			m_staticPressureBuf.begin() + end);
	
	if (m_hasDepth)
		rotate(m_depthBuffer.begin() + start, m_depthBuffer.begin() + mid,
					m_depthBuffer.begin() + end);
	
	if (m_colorBuffer.data())
		rotate(m_colorBuffer.begin() + start,
					m_colorBuffer.begin() + mid, m_colorBuffer.begin() + end);
	
	rotate(m_heatBuffer.begin() + start,
		m_heatBuffer.begin() + mid, m_heatBuffer.begin() + end);
	
	rotate(m_healthBuffer.begin() + start,
		m_healthBuffer.begin() + mid, m_healthBuffer.begin() + end);
	
	// Update handle indices.
	if (!m_handleIndexBuffer.empty())
	{
		rotate(m_handleIndexBuffer.begin() + start,
					m_handleIndexBuffer.begin() + mid,
					m_handleIndexBuffer.begin() + end);
		for (int32 i = start; i < end; ++i)
		{
			b2ParticleHandle * const handle = m_handleIndexBuffer[i];
			if (handle) handle->SetIndex(newIndices[handle->GetIndex()]);
		}
	}

	if (!m_expireTimeBuf.empty())
	{
		rotate(m_expireTimeBuf.begin() + start,
			m_expireTimeBuf.begin() + mid,
			m_expireTimeBuf.begin() + end);
		// Update expiration time buffer indices.
		const int32 particleCount = GetParticleCount();
		for (int32 i = 0; i < particleCount; ++i)
		{
			m_idxByExpireTimeBuf[i] = newIndices[m_idxByExpireTimeBuf[i]];
		}
	}

	// update proxies
	for (int32 k = 0; k < m_count; k++)
	{
		Proxy& proxy = m_proxyBuffer.data()[k];
		proxy.idx = newIndices[proxy.idx];
	}

	// update contacts
	for (int32 k = 0; k < m_contactCount; k++)
	{
		b2ParticleContact& contact = m_partContactBuf[k];
		contact.idxA = newIndices[contact.idxA];
		contact.idxB = newIndices[contact.idxB];
	}

	// update particle-body contacts
	for (int32 k = 0; k < m_bodyContactCount; k++)
	{
		int32& partIdx = m_bodyContactBuf[k].partIdx;
		partIdx = newIndices[partIdx];
	}

	// update pairs
	for (int32 k = 0; k < m_pairCount; k++)
	{
		b2ParticlePair& pair = m_pairBuffer[k];
		pair.indexA = newIndices[pair.indexA];
		pair.indexB = newIndices[pair.indexB];
	}

	// update triads
	for (int32 k = 0; k < m_triadCount; k++)
	{
		b2ParticleTriad& triad = m_triadBuffer[k];
		triad.indexA = newIndices[triad.indexA];
		triad.indexB = newIndices[triad.indexB];
		triad.indexC = newIndices[triad.indexC];
	}

	// update groups
	for (int32 k = 0; k < m_groupCount; k++)
	{
		b2ParticleGroup& group = m_groupBuffer[k];
		if (group.m_firstIndex != b2_invalidIndex)
		{
			group.m_firstIndex = newIndices[group.m_firstIndex];
			group.m_lastIndex = newIndices[group.m_lastIndex - 1] + 1;
		}
	}
}

/// Set the lifetime (in seconds) of a particle relative to the current
/// time.
void b2ParticleSystem::SetParticleLifetime(const int32 index,
										   const float32 lifetime)
{
	b2Assert(ValidateParticleIndex(index));
	const bool initializeExpirationTimes =
		m_idxByExpireTimeBuf.empty();

	// Initialize the inverse mapping buffer.
	if (initializeExpirationTimes)
	{
		const int32 particleCount = GetParticleCount();
		for (int32 i = 0; i < particleCount; ++i)
		{
			m_idxByExpireTimeBuf[i] = i;
		}
	}
	const int32 quantizedLifetime = (int32)(lifetime /
											m_def.lifetimeGranularity);
	// Use a negative lifetime so that it's possible to track which
	// of the infinite lifetime particles are older.
	const int32 newExpirationTime = quantizedLifetime > 0 ?
		GetQuantizedTimeElapsed() + quantizedLifetime : quantizedLifetime;
	if (newExpirationTime != m_expireTimeBuf[index])
	{
		m_expireTimeBuf[index] = newExpirationTime;
		m_expirationTimeBufferRequiresSorting = true;
	}
}


/// Convert a lifetime value in returned by GetExpirationTimeBuffer()
/// to a value in seconds relative to the current simulation time.
float32 b2ParticleSystem::ExpirationTimeToLifetime(
	const int32 expirationTime) const
{
	return (float32)(expirationTime > 0 ?
					 	expirationTime - GetQuantizedTimeElapsed() :
					 	expirationTime) * m_def.lifetimeGranularity;
}

/// Get the lifetime (in seconds) of a particle relative to the current
/// time.
float32 b2ParticleSystem::GetParticleLifetime(const int32 index)
{
	b2Assert(ValidateParticleIndex(index));
	return ExpirationTimeToLifetime(m_expireTimeBuf[index]);
}

/// Get the array of particle indices ordered by lifetime.
/// GetExpirationTimeBuffer(
///    GetIndexByExpirationTimeBuffer()[index])
/// is equivalent to GetParticleLifetime(index).
/// GetParticleCount() items are in the returned array.
vector<int32> b2ParticleSystem::GetIndexByExpirationTimeBuffer()
{
	// If particles are present, initialize / reinitialize the lifetime buffer.
	if (GetParticleCount())
	{
		SetParticleLifetime(0, GetParticleLifetime(0));
	}
	else
	{
		m_idxByExpireTimeBuf.resize(m_capacity);
	}
	return m_idxByExpireTimeBuf;
}

void b2ParticleSystem::SetDestructionByAge(const bool enable)
{
	if (enable)
	{
		m_expireTimeBuf.resize(m_capacity);
	}
	m_def.destroyByAge = enable;
}

/// Get the time elapsed in b2ParticleSystemDef::lifetimeGranularity.
int32 b2ParticleSystem::GetQuantizedTimeElapsed() const
{
	return (int32)(m_timeElapsed >> 32);
}

/// Convert a lifetime in seconds to an expiration time.
int64 b2ParticleSystem::LifetimeToExpirationTime(const float32 lifetime) const
{
	return m_timeElapsed + (int64)((lifetime / m_def.lifetimeGranularity) *
								   (float32)(1LL << 32));
}

template <typename T> void b2ParticleSystem::SetUserOverridableBuffer(
	UserOverridableBuffer<T>* buffer, T* newData, int32 newCapacity)
{
	b2Assert((newData && newCapacity) || (!newData && !newCapacity));
	if (!buffer->userSuppliedCapacity && buffer->data())
	{
		m_world.m_blockAllocator.Free(
			buffer->data(), sizeof(T) * m_capacity);
	}
	buffer->data = newData;
	buffer->userSuppliedCapacity = newCapacity;
}

void b2ParticleSystem::SetIndex(int ind)
{
	int bloop = ind + 5;
	MyIndex = 0;
}

void b2ParticleSystem::SetFlagsBuffer(uint32* buffer, int32 capacity)
{
	m_flagsBuffer.assign(buffer, buffer + capacity);
	//SetUserOverridableBuffer(&m_flagsBuffer, buffer, capacity);
}

void b2ParticleSystem::SetPositionBuffer(b2Vec3* buffer, int32 capacity)
{
	m_positionBuffer.assign(buffer, buffer + capacity);
	//Concurrency::copy(buffer, buffer + capacity, m_positionBuffer);
}

void b2ParticleSystem::SetVelocityBuffer(b2Vec3* buffer, int32 capacity)
{
	m_velocityBuffer.assign(buffer, buffer + capacity);
	//Concurrency::copy(buffer, buffer + capacity, m_velocityBuffer);
}

void b2ParticleSystem::SetColorBuffer(int32* buffer, int32 capacity)
{
	m_colorBuffer.assign(buffer, buffer + capacity);
}

void b2ParticleSystem::SetParticleFlags(int32 index, uint32 newFlags)
{
	uint32& oldFlags = m_flagsBuffer[index];
	if (oldFlags & ~newFlags)
	{
		// If any flags might be removed
		m_needsUpdateAllParticleFlags = true;
	}
	if (~m_allParticleFlags & newFlags)
	{
		// If any flags were added
		if (newFlags & b2_tensileParticle)
		{
			RequestBuffer(m_accumulation3Buf, hasAccumulation2Buf);

		}
		if (newFlags & b2_colorMixingParticle)
		{
			RequestBuffer(m_colorBuffer, m_hasColorBuf);
		}
		m_allParticleFlags |= newFlags;
	}
	oldFlags = newFlags;
}

void b2ParticleSystem::AddParticleFlags(int32 index, uint32 newFlags)
{
	if (~m_allParticleFlags & newFlags)
	{
		// If any flags were added
		m_needsUpdateAllParticleFlags = true;
		if (newFlags & b2_tensileParticle)
		{
			RequestBuffer(m_accumulation3Buf, hasAccumulation2Buf);
		}
		if (newFlags & b2_colorMixingParticle)
		{
			RequestBuffer(m_colorBuffer, m_hasColorBuf);
		}
		m_allParticleFlags |= newFlags;
	}
	m_flagsBuffer[index] |= newFlags;
}
void b2ParticleSystem::RemovePartFlagsFromAll(uint32 flags)
{
	if (m_allParticleFlags & flags)
	{
		uint32 invFlags = ~flags;
		m_allParticleFlags &= invFlags;
		for (int32 k = 0; k < m_count; k++)
			m_flagsBuffer[k] &= invFlags;
	}
}

void b2ParticleSystem::SetGroupFlags(
	b2ParticleGroup& group, uint32 newFlags)
{
	b2Assert((newFlags & b2_particleGroupInternalMask) == 0);
	uint32& oldFlags = group.m_groupFlags;
	newFlags |= oldFlags & b2_particleGroupInternalMask;

	if ((oldFlags ^ newFlags) & b2_solidParticleGroup)
	{
		// If the b2_solidParticleGroup flag changed schedule depth update.
		newFlags |= b2_particleGroupNeedsUpdateDepth;
	}
	if (oldFlags & ~newFlags)
	{
		// If any flags might be removed
		m_needsUpdateAllGroupFlags = true;
	}
	if (~m_allGroupFlags & newFlags)
	{
		// If any flags were added
		if (newFlags & b2_solidParticleGroup)
		{
			RequestBuffer(m_depthBuffer, m_hasDepth);
		}
		m_allGroupFlags |= newFlags;
	}
	oldFlags = newFlags;
}
void b2ParticleSystem::UpdateStatistics(const b2ParticleGroup& group) const
{
	if (group.m_timestamp != m_timestamp)
	{
		const float32 m = m_partMatMassBuf[group.m_matIdx];
		const int32& firstIdx = group.m_firstIndex;
		const int32& lastIdx = group.m_lastIndex;
		float32& mass = group.m_mass = 0;
		b2Vec2& center = group.m_center;
		b2Vec2& linVel = group.m_linearVelocity;
		center.SetZero();
		linVel.SetZero();
		for (int32 i = firstIdx; i < lastIdx; i++)
		{
			mass += m;
			center += m * (b2Vec2)m_positionBuffer[i];
			linVel += m * (b2Vec2)m_velocityBuffer[i];
		}
		if (mass > 0)
		{
			center *= 1 / mass;
			linVel *= 1 / mass;
		}
		float32& inertia = group.m_inertia = 0;
		float32& angVel = group.m_angularVelocity = 0;
		for (int32 i = firstIdx; i < lastIdx; i++)
		{
			b2Vec2 p = (b2Vec2)m_positionBuffer[i] - center;
			b2Vec2 v = (b2Vec2)m_velocityBuffer[i] - linVel;
			inertia += m * b2Dot(p, p);
			angVel += m * b2Cross(p, v);
		}
		if (inertia > 0)
		{
			angVel *= 1 / inertia;
		}
		group.m_timestamp = m_timestamp;
	}
}

inline bool b2ParticleSystem::ForceCanBeApplied(uint32 flags) const
{
	return !(flags & b2_wallParticle);
}

inline void b2ParticleSystem::PrepareForceBuffer()
{
	if (!m_hasForce)
	{
		if (m_accelerate)
			amp::fill(m_ampForces, b2Vec3_zero, m_count);
		else
			memset(m_forceBuffer.data(), 0, sizeof(*(m_forceBuffer.data())) * m_count);
		m_hasForce = true;
	}
}

void b2ParticleSystem::ApplyForce(const b2ParticleGroup& group, const b2Vec3& force)
{
	ApplyForce(group.m_firstIndex, group.m_lastIndex, force);
}

void b2ParticleSystem::ApplyForce(int32 firstIndex, int32 lastIndex, const b2Vec3& force)
{
	// Ensure we're not trying to apply force to particles that can't move,
	// such as wall particles.
#if B2_ASSERT_ENABLED
	uint32 flags = 0;
	for (int32 i = firstIndex; i < lastIndex; i++)
	{
		flags |= m_flagsBuffer[i];
	}
	b2Assert(ForceCanBeApplied(flags));
#endif

	// Early out if force does nothing (optimization).
	const uint32 cnt = lastIndex - firstIndex;
	b2Vec3 distributedForce = force / (float32)(cnt);
	if (IsSignificantForce(distributedForce))
	{
		PrepareForceBuffer();

		// Distribute the force over all the particles.
		if (m_accelerate)
		{
			auto& forces = m_ampForces;
			amp::forEach(firstIndex, lastIndex, [=, &forces](const int32 i) restrict(amp)
			{
				forces[i] += distributedForce;
			});
		}
		else
		{
			for (int32 i = firstIndex; i < lastIndex; i++)
			{
				m_forceBuffer[i] += distributedForce;
			}
		}
	}
}
void b2ParticleSystem::ApplyForceInDirIfHasFlag(const b2Vec3& pos, float32 strength, uint32 flag)
{
	PrepareForceBuffer();

	if (m_accelerate)
	{
		auto& flags = m_ampFlags;
		auto& positions = m_ampPositions;
		auto& forces = m_ampForces;
		const uint32 cnt = m_count;
		AmpForEachParticle(flag, [=, &flags, &positions, &forces](const int32 i) restrict(amp)
		{
			b2Vec3 f = (pos - positions[i]);
			f.Normalize();
			forces[i] += f * strength;
		});
	}
	else
	{
		for (int32 k = 0; k < m_count; k++)
		{
			if (m_flagsBuffer[k] & flag)
			{
				b2Vec3 f = (pos - m_positionBuffer[k]);
				f.Normalize();
				m_forceBuffer[k] += f * strength;
			}
		}
	}
}

void b2ParticleSystem::ParticleApplyForce(int32 index, const b2Vec3& force)
{
	if (IsSignificantForce(force) &&
		ForceCanBeApplied(m_flagsBuffer[index]))
	{
		m_forceBuffer[index] += force;
	}
}

void b2ParticleSystem::ApplyLinearImpulse(const b2ParticleGroup& group, const b2Vec2& impulse)
{
	ApplyLinearImpulse(group.m_firstIndex, group.m_lastIndex, impulse);
}

void b2ParticleSystem::ApplyLinearImpulse(int32 firstIndex, int32 lastIndex,
										  const b2Vec2& impulse)
{
	const float32 numParticles = (float32)(lastIndex - firstIndex);
	//const float32 totalMass = numParticles * GetParticleMass();
	//const b2Vec2 velocityDelta = impulse / totalMass;	vd = im / np * gpm
	b2Vec2 velDeltaWithoutMass = impulse / numParticles;
	for (int32 i = firstIndex; i < lastIndex; i++)
	{
		//m_velocityBuffer[i] += velocityDelta;
		b2Vec2 vel = velDeltaWithoutMass * m_partMatInvMassBuf[m_partMatIdxBuffer[i]];
		m_velocityBuffer[i] += vel;
	}
}

void b2ParticleSystem::QueryAABB(b2QueryCallback* callback,
								 const b2AABB& aabb) const
{
	if (m_proxyBuffer.empty())
	{
		return;
	}
	const Proxy* beginProxy = m_proxyBuffer.data();
	const Proxy* endProxy = beginProxy + m_count;
	const Proxy* firstProxy = std::lower_bound(
		beginProxy, endProxy,
		computeTag(
			m_inverseDiameter * aabb.lowerBound.x,
			m_inverseDiameter * aabb.lowerBound.y));
	const Proxy* lastProxy = std::upper_bound(
		firstProxy, endProxy,
		computeTag(
			m_inverseDiameter * aabb.upperBound.x,
			m_inverseDiameter * aabb.upperBound.y));
	for (const Proxy* proxy = firstProxy; proxy < lastProxy; ++proxy)
	{
		int32 i = proxy->idx;
		const b2Vec3& p = m_positionBuffer[i];
		if (aabb.lowerBound.x < p.x && p.x < aabb.upperBound.x &&
			aabb.lowerBound.y < p.y && p.y < aabb.upperBound.y)
		{
			if (!callback->ReportParticle(this, i))
			{
				break;
			}
		}
	}
}

void b2ParticleSystem::QueryShapeAABB(b2QueryCallback* callback,
									  const b2Shape& shape,
									  const b2Transform& xf) const
{
	b2AABB aabb;
	shape.ComputeAABB(aabb, xf, 0);
	QueryAABB(callback, aabb);
}

void b2ParticleSystem::RayCast(b2RayCastCallback& callback,
							   const b2Vec2& point1,
							   const b2Vec2& point2) const
{
	if (m_proxyBuffer.empty())
	{
		return;
	}
	b2AABB aabb;
	aabb.lowerBound = b2Min(point1, point2);
	aabb.upperBound = b2Max(point1, point2);
	float32 fraction = 1;
	// solving the following equation:
	// ((1-t)*point1+t*point2-position)^2=diameter^2
	// where t is a potential fraction
	b2Vec2 v = point2 - point1;
	float32 v2 = b2Dot(v, v);
	InsideBoundsEnumerator enumerator = GetInsideBoundsEnumerator(aabb);
	int32 i;
	while ((i = enumerator.GetNext()) >= 0)
	{
		b2Vec2 p = point1 - m_positionBuffer[i];
		float32 pv = b2Dot(p, v);
		float32 p2 = b2Dot(p, p);
		float32 determinant = pv * pv - v2 * (p2 - m_squaredDiameter);
		if (determinant >= 0)
		{
			float32 sqrtDeterminant = b2Sqrt(determinant);
			// find a solution between 0 and fraction
			float32 t = (-pv - sqrtDeterminant) / v2;
			if (t > fraction)
			{
				continue;
			}
			if (t < 0)
			{
				t = (-pv + sqrtDeterminant) / v2;
				if (t < 0 || t > fraction)
				{
					continue;
				}
			}
			b2Vec2 n = p + t * v;
			n.Normalize();
			float32 f = callback.ReportParticle(this, i, point1 + t * v, n, t);
			fraction = b2Min(fraction, f);
			if (fraction <= 0)
			{
				break;
			}
		}
	}
}

float32 b2ParticleSystem::ComputeCollisionEnergy() const
{
	float32 sum_v2 = 0;
	for (int32 k = 0; k < m_contactCount; k++)
	{
		const b2ParticleContact& contact = m_partContactBuf[k];
		int32 a = contact.idxA;
		int32 b = contact.idxB;
		b2Vec2 n = contact.normal;
		b2Vec2 v = b2Vec2(m_velocityBuffer[b] - m_velocityBuffer[a]);
		float32 vn = b2Dot(v, n);
		if (vn < 0)
		{
			sum_v2 += vn * vn;
		}
	}
	return 0.5f * GetParticleMass() * sum_v2;
}

void b2ParticleSystem::SetStuckThreshold(int32 steps)
{
	m_stuckThreshold = steps;

	if (steps > 0)
	{
		RequestBuffer(m_lastBodyContactStepBuffer, hasLastBodyContactStepBuffer);
		RequestBuffer(m_bodyContactCountBuffer, hasBodyContactCountBuffer);
		RequestBuffer(m_consecutiveContactStepsBuffer, hasConsecutiveContactStepsBuffer);
	}
}

#if LIQUIDFUN_EXTERNAL_LANGUAGE_API

b2ParticleSystem::b2ExceptionType b2ParticleSystem::IsBufCopyValid(
	int startIndex, int numParticles, int copySize, int bufSize) const
{
	const int maxNumParticles = GetParticleCount();

	// are we actually copying?
	if (copySize == 0)
	{
		return b2_noExceptions;
	}

	// is the index out of bounds?
	if (startIndex < 0 ||
		startIndex >= maxNumParticles ||
		numParticles < 0 ||
		numParticles + startIndex > maxNumParticles)
	{
		return b2_particleIndexOutOfBounds;
	}

	// are we copying within the boundaries?
	if (copySize > bufSize)
	{
		return b2_bufferTooSmall;
	}

	return b2_noExceptions;
}

#endif // LIQUIDFUN_EXTERNAL_LANGUAGE_API

void b2ParticleSystem::CopyAmpPositions(ID3D11Buffer* dstPtr)
{
	D3D11_BUFFER_DESC desc;
	dstPtr->GetDesc(&desc);
	auto& a = Concurrency::direct3d::make_array<b2Vec3>(ampExtent(m_count), m_gpuAccelView, dstPtr);
	Concurrency::copy(m_ampPositions.section(0, m_count), a);
	return;

	// D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS

	//Microsoft::WRL::ComPtr<ID3D11Device> baseDevice;
	//Microsoft::WRL::ComPtr<ID3D12Device> device12;
	//3D11CreateDevice(
	//	nullptr,
	//	D3D_DRIVER_TYPE_WARP,
	//	0,
	//	D3D11_CREATE_DEVICE_VIDEO_SUPPORT | D3D11_CREATE_DEVICE_BGRA_SUPPORT,
	//	nullptr,
	//	0,
	//	D3D11_SDK_VERSION,
	//	baseDevice.GetAddressOf(),
	//	nullptr,
	//	nullptr
	//;
	ID3D11Buffer* dx11PosBuf = reinterpret_cast<ID3D11Buffer*>(Concurrency::direct3d::get_buffer(m_ampPositions));
	ID3D11Device* baseDevice = reinterpret_cast<ID3D11Device*>(Concurrency::direct3d::get_device(m_gpuAccelView));
	//Microsoft::WRL::ComPtr<ID3D11Device1> device11;
	//baseDevice.As(&device11);

	//D3D12CreateDevice(
	//	nullptr,
	//	D3D_FEATURE_LEVEL_11_0,
	//	IID_PPV_ARGS(&device12)
	//);


	//HANDLE sharedHandle;
	//device12->CreateSharedHandle(
	//	dstPtr,
	//	nullptr,
	//	GENERIC_ALL,
	//	nullptr,
	//	&sharedHandle
	//);
	//
	//ID3D11Resource* sharedRecource;
	//device11->OpenSharedResource1(
	//	sharedHandle, 
	//	__uuidof(ID3D11Resource),
	//	(void**)&sharedRecource
	//);
	ID3D11DeviceContext* context = NULL;
	baseDevice->GetImmediateContext(&context);
	// device11->GetImmediateContext(&context);
	// context->CopyResource(sharedRecource, dx11PosBuf);
	context->CopyResource(dstPtr, dx11PosBuf);
	return;
	context->Release();
}

template<typename S, typename A>
inline ampCopyFuture CopyShapeBufferToGpu(std::vector<S>& shapeBuffer, ampArray<A>& array)
{
	b2Assert(sizeof(S) == sizeof(A));
	return amp::copyAsync(reinterpret_cast<A*>(shapeBuffer.data()), array, shapeBuffer.size());
}

void b2ParticleSystem::CopyBox2DToGPUAsync()
{
	m_ampCopyFutBodies.set(amp::copyAsync(m_world.m_bodyBuffer, m_ampBodies));
	m_ampCopyFutFixtures.set(amp::copyAsync(m_world.m_fixtureBuffer, m_ampFixtures));
	m_ampCopyFutChainShapes.set(CopyShapeBufferToGpu(m_world.m_chainShapeBuffer, m_ampChainShapes));
	m_ampCopyFutCircleShapes.set(CopyShapeBufferToGpu(m_world.m_circleShapeBuffer, m_ampCircleShapes));
	m_ampCopyFutEdgeShapes.set(CopyShapeBufferToGpu(m_world.m_edgeShapeBuffer, m_ampEdgeShapes));
	m_ampCopyFutPolygonShapes.set(CopyShapeBufferToGpu(m_world.m_polygonShapeBuffer, m_ampPolygonShapes));
}

void b2ParticleSystem::WaitForCopyBox2DToGPU() 
{
	m_ampCopyFutBodies.wait();
	m_ampCopyFutFixtures.wait();
	m_ampCopyFutChainShapes.wait();
	m_ampCopyFutCircleShapes.wait();
	m_ampCopyFutEdgeShapes.wait();
	m_ampCopyFutPolygonShapes.wait();
	//int32 offsetb2Type = offsetof(b2PolygonShape, b2PolygonShape::m_type);
	//int32 offsetampType = offsetof(AmpPolygonShape, AmpPolygonShape::m_type);
	//int32 offsetb2Count = offsetof(b2PolygonShape, b2PolygonShape::m_count);
	//int32 offsetampCount = offsetof(AmpPolygonShape, AmpPolygonShape::m_count);
	//int32 sizeb2 = sizeof(b2PolygonShape);
	//int32 sizeamp = sizeof(AmpPolygonShape);
	//ampArrayView<AmpPolygonShape> polys = m_ampPolygonShapes.section(0, 2);
	//AmpPolygonShape poly0 = polys[1];
}
