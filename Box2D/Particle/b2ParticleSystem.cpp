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
#include <Box2D/Particle/b2Particlegroup.h>
#include <Box2D/Particle/b2VoronoiDiagram.h>
#include <Box2D/Particle/b2ParticleAssembly.h>
#include <Box2D/Common/b2BlockAllocator.h>
#include <Box2D/Common/b2GlobalVariables.h>
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Dynamics/b2WorldCallbacks.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Collision/Shapes/b2Shape.h>
#include <Box2D/Collision/Shapes/b2EdgeShape.h>
#include <Box2D/Collision/Shapes/b2ChainShape.h>
#include <algorithm>
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

static const uint32 afArrayDims = 1;
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
	b2ParticleBodyContactRemovePredicate(b2ParticleSystem* system,
										 int32* discarded)
		: m_system(system), m_lastIndex(-1), m_currentContacts(0),
		  m_discarded(discarded) {}

	bool operator()(const b2PartBodyContact& contact)
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
	}
private:
	// Max number of contacts processed per particle, from nearest to farthest.
	// This must be at least 2 for correctness with concave shapes; 3 was
	// experimentally arrived at as looking reasonable.
	static const int32 k_maxContactsPerPoint = 3;
	const b2ParticleSystem* m_system;
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
typedef LightweightPair<b2Fixture*,int32> FixtureParticle;

// Associates a fixture with a particle index.
typedef LightweightPair<int32,int32> ParticlePair;

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
	void Initialize(const vector<b2PartBodyContact>& bodyContactIdxs,
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

static inline uint32 computeRelativeTag(uint32 tag, int32 x, int32 y)
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

b2ParticleSystem::b2ParticleSystem(const b2ParticleSystemDef* def,
								   b2World* world, 
								   vector<b2Body*>& bodyBuffer,
								   vector<b2Fixture*>& fixtureBuffer) :
	m_handleAllocator(b2_minParticleBufferCapacity),
	m_bodyBuffer(bodyBuffer),
	m_fixtureBuffer(fixtureBuffer)

{

	//AllocConsole();
	//FILE* fp;
	//freopen_s(&fp, "CONOUT$", "w", stdout);

	b2Assert(def);
	m_paused = false;
	m_timestamp = 0;
	m_allParticleFlags = 0;
	m_needsUpdateAllParticleFlags = false;
	m_allGroupFlags = 0;
	m_needsUpdateAllGroupFlags = false;
	m_hasForce = false;
	m_hasDepth = false;
	m_iterationIndex = 0;

	SetStrictContactCheck(def->strictContactCheck);
	SetDensity(def->density);
	SetGravityScale(def->gravityScale);
	SetRadius(def->radius);
	SetMaxParticleCount(def->maxCount);

	m_count = 0;
	m_particleBufferSize = 0;
	m_groupCount = 0;
	m_groupBufferSize = 0;
	m_partMatCount = 0;
	m_partMatBufferSize = 0;
	m_contactCount = 0;
	m_contactBufferSize = 0;
	m_bodyContactCount = 0;
	m_bodyContactBufferSize = 0;
	m_pairCount = 0;
	m_pairBufferSize = 0;
	m_triadCount = 0;
	m_triadBufferSize = 0;

	hasColorBuf						 = false;
	hasHandleIndexBuffer			 = false;
	hasStaticPressureBuf			 = false;
	hasAccumulation2Buf				 = false;
	hasLastBodyContactStepBuffer	 = false;
	hasBodyContactCountBuffer		 = false;
	hasConsecutiveContactStepsBuffer = false;

	b2Assert(def->lifetimeGranularity > 0.0f);
	m_def = *def;

	m_world = world;

	m_stuckThreshold = 0;

	m_timeElapsed = 0;
	m_expirationTimeBufferRequiresSorting = false;

	SetDestructionByAge(m_def.destroyByAge);
}

b2ParticleSystem::~b2ParticleSystem()
{
	for (int i = 0; i < m_groupCount; i++)
		DestroyParticleGroup(i);
}

template <typename T> void b2ParticleSystem::FreeBuffer(T** b, int capacity)
{
	if (*b == NULL)
		return;

	m_world->m_blockAllocator.Free(*b, sizeof(**b) * capacity);
	*b = NULL;
}

// Free buffer, if it was allocated with b2World's block allocator
template <typename T> void b2ParticleSystem::FreeUserOverridableBuffer(
	UserOverridableBuffer<T>* b)
{
	if (b->userSuppliedCapacity == 0)
	{
		FreeBuffer(&b->data, m_particleBufferSize);
	}
}

// Reallocate a buffer
template <typename T> T* b2ParticleSystem::ReallocateBuffer(
	T* oldBuffer, int32 oldCapacity, int32 newCapacity)
{
	b2Assert(newCapacity > oldCapacity);
	T* newBuffer = (T*) m_world->m_blockAllocator.Allocate(
		sizeof(T) * newCapacity);
	if (oldBuffer)
	{
		memcpy(newBuffer, oldBuffer, sizeof(T) * oldCapacity);
		m_world->m_blockAllocator.Free(oldBuffer, sizeof(T) * oldCapacity);
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
	b2Assert(newCapacity > m_particleBufferSize);
	// Reallocate a new handle / index map buffer, copying old handle pointers
	// is fine since they're kept around.
	m_handleIndexBuffer.resize(newCapacity);
	// Set the size of the next handle allocation.
	m_handleAllocator.SetItemsPerSlab(newCapacity -
									  m_particleBufferSize);
}

template <typename T> 
void b2ParticleSystem::RequestBuffer(vector<T>& buf, bool& hasBuffer)
{
	if (!hasBuffer)
	{
		buf.resize(m_particleBufferSize);
		hasBuffer = true;
	}
}

int32* b2ParticleSystem::GetColorBuffer()
{
	return m_colorBuffer.data();
}

int32* b2ParticleSystem::GetUserDataBuffer()
{
	//m_userDataBuffer.data = RequestBuffer(m_userDataBuffer.data);
	return m_userDataBuffer.data();
}

static int32 LimitCapacity(int32 capacity, int32 maxCount)
{
	return maxCount && capacity > maxCount ? maxCount : capacity;
}

void b2ParticleSystem::ResizePartMatBuffers(int32 size)
{
	if (!size) size = b2_minPartMatBufferCapacity;
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
	m_partMatBufferSize = size;
}

void b2ParticleSystem::ResizeParticleBuffers(int32 size)
{
	if (!size) size = b2_minParticleBufferCapacity;
	ReallocateHandleBuffers(size);
	m_flagsBuffer.resize(size);
	m_heightLayerBuffer.resize(size);

	// Conditionally defer these as they are optional if the feature is
	// not enabled.
	const bool stuck = m_stuckThreshold > 0;
	m_lastBodyContactStepBuffer.resize(size);
	m_bodyContactCountBuffer.resize(size);
	m_consecutiveContactStepsBuffer.resize(size);
	m_positionBuffer.resize(size);
	m_velocityBuffer.resize(size);
	m_forceBuffer.resize(size);
	m_weightBuffer.resize(size);
	m_staticPressureBuf.resize(size);
	m_accumulationBuf.resize(size);
	m_accumulation2Buf.resize(size);
	m_depthBuffer.resize(size);
	m_colorBuffer.resize(size);
	m_partGroupIdxBuffer.resize(size);
	m_partMatIdxBuffer.resize(size);

	m_userDataBuffer.resize(size);
	m_heatBuffer.resize(size);
	m_healthBuffer.resize(size);

	m_proxyBuffer.resize(size);

	//m_findContactCountBuf.resize(size);
	//m_findContactIdxABuf.resize(size);
	//m_findContactIdxBBuf.resize(size);
	m_findContactRightTagBuf.resize(size);
	m_findContactBottomLeftTagBuf.resize(size);
	m_findContactBottomRightTagBuf.resize(size);

	m_expireTimeBuf.resize(size); 
	m_idxByExpireTimeBuf.resize(size);

	m_particleBufferSize = size;
}

void b2ParticleSystem::ResizeGroupBuffers(int32 size)
{
	if (!size) size = b2_minGroupBufferCapacity;
	m_groupBuffer.resize(size);
	m_groupBufferSize = size;
}

void b2ParticleSystem::ResizeContactBuffers(int32 size)
{
	if (!size) size = b2_minParticleBufferCapacity;
	m_partContactBuf.resize(size);
	m_contactBufferSize = size;
}
void b2ParticleSystem::ResizeBodyContactBuffers(int32 size)
{
	if (!size) size = b2_minParticleBufferCapacity;
	m_bodyContactBuf.resize(size);
	m_bodyContactBufferSize = size;
}

void b2ParticleSystem::ResizePairBuffers(int32 size)
{
	if (!size) size = b2_minParticleBufferCapacity;
	m_pairBuffer.resize(size);
	m_pairBufferSize = size;
}
void b2ParticleSystem::ResizeTriadBuffers(int32 size)
{
	if (!size) size = b2_minParticleBufferCapacity;
	m_triadBuffer.resize(size);
	m_triadBufferSize = size;
}

int32 b2ParticleSystem::CreateParticleMaterial(const b2ParticleMaterialDef & def)
{
	if (m_partMatCount >= m_partMatBufferSize)
		ResizePartMatBuffers(m_partMatCount * 2);
	int32 idx = m_partMatCount;
	m_partMatCount++;
	m_partMatFlagsBuf[idx] = def.matFlags;
	m_partMatPartFlagsBuf[idx] = def.partFlags;
	m_partMatMassBuf[idx] = def.mass;
	m_partMatInvMassBuf[idx] = 1 / def.mass;
	m_partMatStabilityBuf[idx] = def.stability;
	m_partMatInvStabilityBuf[idx] = 1 / def.stability;
	m_partMatHeatConductivityBuf[idx] = def.heatConductivity;
	m_world->m_allMaterialFlags |= def.matFlags;
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
}

int32 b2ParticleSystem::CreateParticle(const b2ParticleDef& def)
{
	b2Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked())
		return 0;

	if (m_count >= m_particleBufferSize)
		ResizeParticleBuffers(2 * m_count);
	
	if (m_count >= m_particleBufferSize)
	{
		// If the oldest particle should be destroyed...
		if (m_def.destroyByAge)
		{
			DestroyOldestParticle(0, false);
			// Need to destroy this particle *now* so that it's possible to
			// create a new particle.
			SolveZombie();
		}
		else
		{
			return b2_invalidIndex;
		}
	}
	int32 index = m_count++;

	m_flagsBuffer[index] = 0;
	if (!m_lastBodyContactStepBuffer.empty())
		m_lastBodyContactStepBuffer[index] = 0;
	if (!m_bodyContactCountBuffer.empty())
		m_bodyContactCountBuffer[index] = 0;
	if (!m_consecutiveContactStepsBuffer.empty())
		m_consecutiveContactStepsBuffer[index] = 0;
	m_positionBuffer[index] = def.position;
	m_heightLayerBuffer[index] = GetLayerFromZ(def.position.z);
	m_velocityBuffer[index] = def.velocity;
	m_weightBuffer[index] = 0;
	m_heatBuffer[index] = def.heat;
	m_healthBuffer[index] = def.health;
	m_forceBuffer[index] = b2Vec3_zero;
	m_partMatIdxBuffer[index] = def.matIdx;
	if (!m_staticPressureBuf.empty())
		m_staticPressureBuf[index] = 0;
	
	m_depthBuffer[index] = 0;
	m_colorBuffer[index] = def.color;
	m_userDataBuffer[index] = def.userData;
	if (!m_handleIndexBuffer.empty())
		m_handleIndexBuffer[index] = NULL;
	
	m_proxyBuffer[index].idx = index;
	m_proxyBuffer[index].tag = 0;

	// If particle lifetimes are enabled or the lifetime is set in the particle
	// definition, initialize the lifetime.
	const bool finiteLifetime = def.lifetime > 0;
	if (!m_expireTimeBuf.empty() || finiteLifetime)
	{
		SetParticleLifetime(index, finiteLifetime ? def.lifetime :
								ExpirationTimeToLifetime(
									-GetQuantizedTimeElapsed()));
		// Add a reference to the newly added particle to the end of the
		// queue.
		m_idxByExpireTimeBuf[index] = index;
	}

	if (def.groupIdx != b2_invalidIndex)
	{
		b2ParticleGroup* group = m_groupBuffer[def.groupIdx];
		if (group->m_firstIndex < group->m_lastIndex)
		{
			// Move particles in the group just before the new particle.
			RotateBuffer(group->m_firstIndex, group->m_lastIndex, index);
			b2Assert(group->m_lastIndex == index);
			// Update the index range of the group to contain the new particle.
			group->m_lastIndex = index + 1;
		}
		else
		{
			// If the group is empty, reset the index range to contain only the
			// new particle.
			group->m_firstIndex = index;
			group->m_lastIndex = index + 1;
		}
	}
	SetParticleFlags(index, def.flags);
	return index;
}

inline const int32 b2ParticleSystem::GetLayerFromZ(float32 z)
{
	return z * m_invPointsPerLayer;
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
	{
		flags |= b2_destructionListenerParticle;
	}
	SetParticleFlags(index, m_flagsBuffer[index] | flags);
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

int32 b2ParticleSystem::DestroyParticlesInShape(
	const b2Shape& shape, const b2Transform& xf,
	bool callDestructionListener)
{
	b2Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked())
	{
		return 0;
	}

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
	m_world->QueryAABB(&callback, aabb);
	return callback.Destroyed();
}

int32 b2ParticleSystem::CreateParticleForGroup(
	const b2ParticleGroupDef& groupDef, const b2Transform& xf, const b2Vec3& p)
{
	b2ParticleDef particleDef;
	particleDef.flags = groupDef.flags;
	b2Vec2 pos = b2Mul(xf, p);
	particleDef.position = b2Vec3(pos, groupDef.shape->m_zPos);
	b2Vec2 vel =
		groupDef.linearVelocity +
		b2Cross(groupDef.angularVelocity,
			pos - groupDef.position);
	particleDef.velocity = b2Vec3(vel, 0);
	particleDef.color = groupDef.color;
	particleDef.lifetime = groupDef.lifetime;
	particleDef.userData = groupDef.userData;
	particleDef.health = groupDef.health;
	particleDef.heat = groupDef.heat;
	particleDef.matIdx = groupDef.matIdx;
	return CreateParticle(particleDef);
}

int32 b2ParticleSystem::CreateParticleForGroup(
	const b2ParticleGroupDef& groupDef, const b2Transform& xf, const b2Vec3& p, int32 c)
{
	b2ParticleDef particleDef;
	particleDef.flags = groupDef.flags;
	b2Vec2 pos = b2Mul(xf, p);
	particleDef.position = b2Vec3(pos, p.z);
	b2Vec2 vel =
		groupDef.linearVelocity +
		b2Cross(groupDef.angularVelocity,
			pos - groupDef.position);
	particleDef.velocity = b2Vec3(vel, 0);
	particleDef.color = groupDef.color;
	particleDef.lifetime = groupDef.lifetime;
	particleDef.userData = groupDef.userData;
	particleDef.health = groupDef.health;
	particleDef.heat = groupDef.heat;
	particleDef.matIdx = groupDef.matIdx;
	return CreateParticle(particleDef);
}

void b2ParticleSystem::CreateParticlesStrokeShapeForGroup(
	const b2Shape *shape,
	const b2ParticleGroupDef& groupDef, const b2Transform& xf)
{
	float32 stride = groupDef.stride;
	if (stride == 0)
	{
		stride = GetParticleStride();
	}
	float32 positionOnEdge = 0;
	int32 childCount = shape->GetChildCount();
	for (int32 childIndex = 0; childIndex < childCount; childIndex++)
	{
		b2EdgeShape edge;
		if (shape->GetType() == b2Shape::e_edge)
		{
			edge = *(b2EdgeShape*) shape;
		}
		else
		{
			b2Assert(shape->GetType() == b2Shape::e_chain);
			((b2ChainShape*) shape)->GetChildEdge(&edge, childIndex);
		}
		b2Vec2 d = edge.m_vertex2 - edge.m_vertex1;
		float32 edgeLength = d.Length();
		while (positionOnEdge < edgeLength)
		{
			b2Vec3 p(edge.m_vertex1 + positionOnEdge / edgeLength * d, 0);
			CreateParticleForGroup(groupDef, xf, p);
			positionOnEdge += stride;
		}
		positionOnEdge -= edgeLength;
	}
}

void b2ParticleSystem::CreateParticlesFillShapeForGroup(
	const b2Shape *shape,
	const b2ParticleGroupDef& groupDef, const b2Transform& xf)
{
	float32 stride = groupDef.stride;
	if (stride == 0)
	{
		stride = GetParticleStride();
	}
	b2Transform identity;
	identity.SetIdentity();
	b2AABB aabb;
	b2Assert(shape->GetChildCount() == 1);
	shape->ComputeAABB(aabb, identity, 0);
	for (float32 y = floorf(aabb.lowerBound.y / stride) * stride;
		y < aabb.upperBound.y; y += stride)
	{
		for (float32 x = floorf(aabb.lowerBound.x / stride) * stride;
			x < aabb.upperBound.x; x += stride)
		{
			b2Vec3 p(x, y, 0);
			if (shape->TestPoint(identity, p))
			{
				CreateParticleForGroup(groupDef, xf, p);
			}
		}
	}
}

void b2ParticleSystem::CreateParticlesWithShapeForGroup(
	const b2Shape* shape,
	const b2ParticleGroupDef& groupDef, const b2Transform& xf)
{
	switch (shape->GetType()) {
	case b2Shape::e_edge:
	case b2Shape::e_chain:
		CreateParticlesStrokeShapeForGroup(shape, groupDef, xf);
		break;
	case b2Shape::e_polygon:
	case b2Shape::e_circle:
		CreateParticlesFillShapeForGroup(shape, groupDef, xf);
		break;
	default:
		b2Assert(false);
		break;
	}
}

void b2ParticleSystem::CreateParticlesWithShapesForGroup(
	const b2Shape* const* shapes, int32 shapeCount,
	const b2ParticleGroupDef& groupDef, const b2Transform& xf)
{
	class CompositeShape : public b2Shape
	{
	public:
		CompositeShape(const b2Shape* const* shapes, int32 shapeCount)
		{
			m_shapes = shapes;
			m_shapeCount = shapeCount;
		}
		b2Shape* Clone(b2BlockAllocator* allocator) const
		{
			b2Assert(false);
			B2_NOT_USED(allocator);
			return NULL;
		}
		int32 GetChildCount() const
		{
			return 1;
		}
		bool TestPoint(const b2Transform& xf, const b2Vec3& p) const
		{
			for (int32 i = 0; i < m_shapeCount; i++)
			{
				if (m_shapes[i]->TestPoint(xf, p))
				{
					return true;
				}
			}
			return false;
		}
		void ComputeDistance(const b2Transform& xf, const b2Vec2& p,
					float32* distance, b2Vec2* normal, int32 childIndex) const
		{
			b2Assert(false);
			B2_NOT_USED(xf);
			B2_NOT_USED(p);
			B2_NOT_USED(distance);
			B2_NOT_USED(normal);
			B2_NOT_USED(childIndex);
		}
		bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
						const b2Transform& transform, int32 childIndex) const
		{
			b2Assert(false);
			B2_NOT_USED(output);
			B2_NOT_USED(input);
			B2_NOT_USED(transform);
			B2_NOT_USED(childIndex);
			return false;
		}
		void ComputeAABB(
				b2AABB& aabb, const b2Transform& xf, int32 childIndex) const
		{
			B2_NOT_USED(childIndex);
			aabb.lowerBound.x = +FLT_MAX;
			aabb.lowerBound.y = +FLT_MAX;
			aabb.upperBound.x = -FLT_MAX;
			aabb.upperBound.y = -FLT_MAX;
			b2Assert(childIndex == 0);
			for (int32 i = 0; i < m_shapeCount; i++)
			{
				int32 childCount = m_shapes[i]->GetChildCount();
				for (int32 j = 0; j < childCount; j++)
				{
					b2AABB subaabb;
					m_shapes[i]->ComputeAABB(subaabb, xf, j);
					aabb.Combine(subaabb);
				}
			}
		}
		void ComputeMass(b2MassData* massData, float32 density) const
		{
			b2Assert(false);
			B2_NOT_USED(massData);
			B2_NOT_USED(density);
		}
	private:
		const b2Shape* const* m_shapes;
		int32 m_shapeCount;
	} compositeShape(shapes, shapeCount);
	CreateParticlesFillShapeForGroup(&compositeShape, groupDef, xf);
}

int32 b2ParticleSystem::CreateParticleGroup(
	const b2ParticleGroupDef& groupDef)
{
	//clock_t clockCreateGroup = clock();
	b2Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked())
	{
		return 0;
	}

	b2Transform transform;
	transform.Set(groupDef.position, groupDef.angle);
	int32 firstIndex = m_count;
	if (groupDef.shape)
	{
		b2Assert(groupDef.shape->m_zPos);
		CreateParticlesWithShapeForGroup(groupDef.shape, groupDef, transform);
	}
	if (groupDef.shapes)
	{
		CreateParticlesWithShapesForGroup(
					groupDef.shapes, groupDef.shapeCount, groupDef, transform);
	}
	if (groupDef.particleCount)
	{
		b2Assert(groupDef.positionData);
		b2Assert(groupDef.colorData);
		for (int32 i = 0; i < groupDef.particleCount; i++)
		{
			int32 c = groupDef.colorData[i];
			CreateParticleForGroup(groupDef, transform, b2Vec3(groupDef.positionDataX[i], groupDef.positionDataY[i], groupDef.positionDataZ[i]), c);
		}
	}

	int32 lastIndex = m_count;

	int32 groupIdx = b2_invalidIndex;
	if (m_emptyGroupSlots)
	{
		for (int32 i = 0; i < m_groupCount; i++)
		{
			if (!m_groupBuffer[i])
			{
				groupIdx = i;
				m_emptyGroupSlots--;
				break;
			}
		}
	}
	if (groupIdx == b2_invalidIndex)
	{
		groupIdx = m_groupCount++; 
		if (m_groupCount >= m_groupBufferSize)
			ResizeGroupBuffers(m_groupBufferSize * 2);
	}
	void* mem = m_world->m_blockAllocator.Allocate(sizeof(b2ParticleGroup));
	b2ParticleGroup* group = m_groupBuffer[groupIdx] = new (mem) b2ParticleGroup();
	group->m_system = this;
	group->m_firstIndex = firstIndex;
	group->m_lastIndex = lastIndex;
	group->m_strength = groupDef.strength;
	group->m_userData = groupDef.userData;
	group->m_collisionGroup = groupDef.collisionGroup;
	group->m_matIdx = groupDef.matIdx;
	group->m_transform = transform;

	for (int32 i = firstIndex; i < lastIndex; i++)
	{
		m_partGroupIdxBuffer[i] = groupIdx;
	}
	SetGroupFlags(group, groupDef.groupFlags);


	// Create pairs and triads between particles in the group->
	ConnectionFilter filter;
	UpdateContacts(true);
	UpdatePairsAndTriads(firstIndex, lastIndex, filter);

	return groupIdx;

	if (groupDef.groupIdx)
	{
		JoinParticleGroups(groupDef.groupIdx, groupIdx);
		groupIdx = groupDef.groupIdx;
	}

	return groupIdx;
}

void b2ParticleSystem::JoinParticleGroups(int32 groupAIdx,
										  int32 groupBIdx)
{
	b2Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked())
	{
		return;
	}
	b2Assert(groupAIdx != groupBIdx);

	b2ParticleGroup* groupA = m_groupBuffer[groupAIdx];
	b2ParticleGroup* groupB = m_groupBuffer[groupBIdx];

	RotateBuffer(groupB->m_firstIndex, groupB->m_lastIndex, m_count);
	b2Assert(groupB->m_lastIndex == m_count);
	RotateBuffer(groupA->m_firstIndex, groupA->m_lastIndex,
		groupB->m_firstIndex);
	b2Assert(groupA->m_lastIndex == groupB->m_firstIndex);

	// Create pairs and triads connecting groupA and groupB->
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
	} filter(groupB->m_firstIndex);
	UpdateContacts(true);
	UpdatePairsAndTriads(groupA->m_firstIndex, groupB->m_lastIndex, filter);

	for (int32 i = groupB->m_firstIndex; i < groupB->m_lastIndex; i++)
	{
		m_partGroupIdxBuffer[i] = groupAIdx;
	}
	uint32 groupFlags = groupA->m_groupFlags | groupB->m_groupFlags;
	SetGroupFlags(groupA, groupFlags);
	groupA->m_lastIndex = groupB->m_lastIndex;
	groupB->m_firstIndex = groupB->m_lastIndex;
	DestroyParticleGroup(groupBIdx);
}

void b2ParticleSystem::SplitParticleGroup(b2ParticleGroup* group)
{
	UpdateContacts(true);
	int32 particleCount = group->GetParticleCount();
	// We create several linked lists. Each list represents a set of connected
	// particles.
	ParticleListNode* nodeBuffer =
		(ParticleListNode*) m_world->m_stackAllocator.Allocate(
									sizeof(ParticleListNode) * particleCount);
	InitializeParticleLists(group, nodeBuffer);
	MergeParticleListsInContact(group, nodeBuffer);
	ParticleListNode* survivingList =
									FindLongestParticleList(group, nodeBuffer);
	MergeZombieParticleListNodes(group, nodeBuffer, survivingList);
	CreateParticleGroupsFromParticleList(group, nodeBuffer, survivingList);
	UpdatePairsAndTriadsWithParticleList(group, nodeBuffer);
	m_world->m_stackAllocator.Free(nodeBuffer);
}

void b2ParticleSystem::InitializeParticleLists(
	const b2ParticleGroup* group, ParticleListNode* nodeBuffer)
{
	int32 bufferIndex = group->GetBufferIndex();
	int32 particleCount = group->GetParticleCount();
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
	const b2ParticleGroup* group, ParticleListNode* nodeBuffer) const
{
	int32 bufferIndex = group->GetBufferIndex();
	for (int32 k = 0; k < m_contactCount; k++)
	{
		const b2ParticleContact& contact = m_partContactBuf[k];
		int32 a = contact.idxA;
		int32 b = contact.idxB;
		if (!group->ContainsParticle(a) || !group->ContainsParticle(b)) {
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
	const b2ParticleGroup* group, ParticleListNode* nodeBuffer)
{
	int32 particleCount = group->GetParticleCount();
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
	const b2ParticleGroup* group, ParticleListNode* nodeBuffer,
	ParticleListNode* survivingList) const
{
	int32 particleCount = group->GetParticleCount();
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
	const b2ParticleGroup* group, ParticleListNode* nodeBuffer,
	const ParticleListNode* survivingList)
{
	int32 particleCount = group->GetParticleCount();
	b2ParticleGroupDef def;
	def.groupFlags = group->GetGroupFlags();
	def.userData = group->GetUserData();
	for (int32 i = 0; i < particleCount; i++)
	{
		ParticleListNode* list = &nodeBuffer[i];
		if (!list->count || list == survivingList)
		{
			continue;
		}
		b2Assert(list->list == list);
		int32 newGroupIdx = CreateParticleGroup(def);
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
	const b2ParticleGroup* group, const ParticleListNode* nodeBuffer)
{
	int32 bufferIndex = group->GetBufferIndex();
	// Update indices in pairs and triads. If an index belongs to the group,
	// replace it with the corresponding value in nodeBuffer.
	// Note that nodeBuffer is allocated only for the group and the index should
	// be shifted by bufferIndex.
	for (int32 k = 0; k < m_pairCount; k++)
	{
		b2ParticlePair& pair = m_pairBuffer[k];
		int32 a = pair.indexA;
		int32 b = pair.indexB;
		if (group->ContainsParticle(a))
			pair.indexA = nodeBuffer[a - bufferIndex].index;		
		if (group->ContainsParticle(b))
			pair.indexB = nodeBuffer[b - bufferIndex].index;		
	}
	for (int32 k = 0; k < m_triadCount; k++)
	{
		b2ParticleTriad& triad = m_triadBuffer[k];
		int32 a = triad.indexA;
		int32 b = triad.indexB;
		int32 c = triad.indexC;
		if (group->ContainsParticle(a))		
			triad.indexA = nodeBuffer[a - bufferIndex].index;		
		if (group->ContainsParticle(b))		
			triad.indexB = nodeBuffer[b - bufferIndex].index;		
		if (group->ContainsParticle(c))		
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
	{
		def.color = m_colorBuffer[oldIndex];
	}
	if (m_userDataBuffer.data())
	{
		def.userData = m_userDataBuffer[oldIndex];
	}
	def.groupIdx = groupIdx;
	def.matIdx = m_partMatIdxBuffer[oldIndex];
	int32 newIndex = CreateParticle(def);
	if (!m_handleIndexBuffer.empty())
	{
		b2ParticleHandle* handle = m_handleIndexBuffer[oldIndex];
		if (handle) handle->SetIndex(newIndex);
		m_handleIndexBuffer[newIndex] = handle;
		m_handleIndexBuffer[oldIndex] = NULL;
	}
	if (!m_lastBodyContactStepBuffer.empty())
	{
		m_lastBodyContactStepBuffer[newIndex] =
			m_lastBodyContactStepBuffer[oldIndex];
	}
	if (!m_bodyContactCountBuffer.empty())
	{
		m_bodyContactCountBuffer[newIndex] =
			m_bodyContactCountBuffer[oldIndex];
	}
	if (!m_consecutiveContactStepsBuffer.empty())
	{
		m_consecutiveContactStepsBuffer[newIndex] =
			m_consecutiveContactStepsBuffer[oldIndex];
	}
	if (m_hasForce)
	{
		m_forceBuffer[newIndex] = m_forceBuffer[oldIndex];
	}
	if (!m_staticPressureBuf.empty())
	{
		m_staticPressureBuf[newIndex] = m_staticPressureBuf[oldIndex];
	}
	m_depthBuffer[newIndex] = m_depthBuffer[oldIndex];
	if (!m_expireTimeBuf.empty())
	{
		m_expireTimeBuf[newIndex] = m_expireTimeBuf[oldIndex];
	}
	return newIndex;
}

void b2ParticleSystem::UpdatePairsAndTriadsWithReactiveParticles()
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
	m_allParticleFlags &= ~b2_reactiveParticle;
}

static bool ParticleCanBeConnected(
	uint32 flags, b2ParticleGroup* group, int32 groupIdx)
{
	return
		(flags & (b2_wallParticle | b2_springParticle | b2_elasticParticle)) ||
		(groupIdx != b2_invalidIndex && group->GetGroupFlags() & b2_rigidParticleGroup);
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
			b2ParticleGroup* groupA = m_groupBuffer[groupAIdx];
			b2ParticleGroup* groupB = m_groupBuffer[groupBIdx];
			if (a >= firstIndex && a < lastIndex &&
				b >= firstIndex && b < lastIndex &&
				!((af | bf) & b2_zombieParticle) &&
				((af | bf) & k_pairFlags) &&
				(filter.IsNecessary(a) || filter.IsNecessary(b)) &&
				ParticleCanBeConnected(af, groupA, groupAIdx) &&
				ParticleCanBeConnected(bf, groupB, groupBIdx) &&
				filter.ShouldCreatePair(a, b))
			{
				if (m_pairCount >= m_pairBufferSize)
					ResizePairBuffers(m_pairCount * 2);
				b2ParticlePair& pair = m_pairBuffer[m_pairCount];
				pair.indexA = a;
				pair.indexB = b;
				pair.flags = contact.flags;
				pair.strength = b2Min(
					groupAIdx != b2_invalidIndex ? groupA->m_strength : 1,
					groupBIdx != b2_invalidIndex ? groupB->m_strength : 1);
				pair.distance = b2Distance(m_positionBuffer[a], m_positionBuffer[b]);
			}
		}
		concurrency::parallel_sort(
			m_pairBuffer.begin(), m_pairBuffer.end(), ComparePairIndices);
		unique(m_pairBuffer.begin(), m_pairBuffer.end(), MatchPairIndices);
		m_pairCount = m_pairBuffer.size();
	}
	if (particleFlags & k_triadFlags)
	{
		b2VoronoiDiagram diagram(
			&m_world->m_stackAllocator, lastIndex - firstIndex);
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
					b2ParticleGroup* groupA = m_system->m_groupBuffer[groupAIdx];
					b2ParticleGroup* groupB = m_system->m_groupBuffer[groupBIdx];
					b2ParticleGroup* groupC = m_system->m_groupBuffer[groupCIdx];
					int32& triadCount = m_system->m_triadCount;
					if (triadCount >= m_system->m_triadBufferSize)
						m_system->ResizeTriadBuffers(triadCount * 2);
					b2ParticleTriad& triad = m_system->m_triadBuffer[triadCount];
					triadCount++;
					triad.indexA = a;
					triad.indexB = b;
					triad.indexC = c;
					triad.flags = af | bf | cf;
					triad.strength = b2Min(b2Min(
						groupAIdx != b2_invalidIndex ? groupA->m_strength : 1,
						groupBIdx != b2_invalidIndex ? groupB->m_strength : 1),
						groupCIdx != b2_invalidIndex ? groupC->m_strength : 1);
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
		unique(m_triadBuffer.begin(), m_triadBuffer.end(), MatchTriadIndices);
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
void b2ParticleSystem::DestroyParticleGroup(int32 groupIdx)
{
	b2Assert(m_groupCount > 0);
	b2Assert(groupIdx > 0);

	b2ParticleGroup* group = m_groupBuffer[groupIdx];

	if (m_world->m_destructionListener)
	{
		m_world->m_destructionListener->SayGoodbye(group);
	}

	SetGroupFlags(group, 0);
	for (int32 i = group->m_firstIndex; i < group->m_lastIndex; i++)
	{
		m_partGroupIdxBuffer[i] = b2_invalidIndex;
	}
	m_world->m_blockAllocator.Free(group, sizeof(b2ParticleGroup));

	for (int i = m_groupCount - 1; i >= 0; i--)
	{
		if (!m_groupBuffer[i])
			break;
		m_groupCount--;
	}
}

void b2ParticleSystem::ComputeWeight()
{
	// calculates the sum of contact-weights for each particle
	// that means dimensionless density
	//memset(m_weightBuffer.data(), 0, sizeof(*(m_weightBuffer.data())) * m_count);
	//m_weightBuffer.resize(m_particleBufferSize);
	std::fill(m_weightBuffer.begin(), m_weightBuffer.begin() + m_count, 0);
	for (int32 k = 0; k < m_bodyContactCount; k++)
	{
		const b2PartBodyContact& contact = m_bodyContactBuf[k];
		int32 a = contact.idx;
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

void b2ParticleSystem::ComputeDepth()
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
			(m_groupBuffer[groupAIdx]->m_groupFlags & b2_particleGroupNeedsUpdateDepth))
		{
			contactGroups[contactGroupsCount++] = contact;
		}
	}
	vector<b2ParticleGroup*> groupsToUpdate(m_groupCount);
	int32 groupsToUpdateCount = 0;

	for (int k = 0; k < m_groupCount; k++)
	{
		b2ParticleGroup* group = m_groupBuffer[k];
		if (group)
		{
			if (group->m_groupFlags & b2_particleGroupNeedsUpdateDepth)
			{
				groupsToUpdate[groupsToUpdateCount++] = group;
				SetGroupFlags(group,
					group->m_groupFlags &
					~b2_particleGroupNeedsUpdateDepth);
				for (int32 i = group->m_firstIndex; i < group->m_lastIndex; i++)
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
		b2ParticleGroup* group = groupsToUpdate[i];
		for (int32 i = group->m_firstIndex; i < group->m_lastIndex; i++)
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
		b2ParticleGroup* group = groupsToUpdate[i];
		for (int32 i = group->m_firstIndex; i < group->m_lastIndex; i++)
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

b2ParticleSystem::InsideBoundsEnumerator
b2ParticleSystem::GetInsideBoundsEnumerator(const b2AABB& aabb) const
{
	uint32 lowerTag = computeTag(m_inverseDiameter * aabb.lowerBound.x - 1,
		m_inverseDiameter * aabb.lowerBound.y - 1);
	uint32 upperTag = computeTag(m_inverseDiameter * aabb.upperBound.x + 1,
		m_inverseDiameter * aabb.upperBound.y + 1);

	const Proxy* beginProxy = m_proxyBuffer.data();
	const Proxy* endProxy = beginProxy + m_count;
	const Proxy* firstProxy = std::lower_bound(beginProxy, endProxy, lowerTag);
	const Proxy* lastProxy = std::upper_bound(firstProxy, endProxy, upperTag);

	return InsideBoundsEnumerator(lowerTag, upperTag, firstProxy, lastProxy);
}

void b2ParticleSystem::FindContacts()
{
	m_contactCount = 0;

	for (int32 i = 0; i < m_count; i++)
	{
		const uint32& tag = m_proxyBuffer[i].tag;
		m_findContactRightTagBuf[i]		  = computeRelativeTag(tag,  1, 0);
		m_findContactBottomLeftTagBuf[i]  = computeRelativeTag(tag, -1, 1);
		m_findContactBottomRightTagBuf[i] = computeRelativeTag(tag,  1, 1);
	}

	//Time t = Clock::now();
	const int end = m_count;
	for (int a = 0, c = 0; a < end; a++)
	{
		const int32 aIdx = m_proxyBuffer[a].idx;
		//const uint32 aTag = m_proxyTagBuffer[a];
		const uint32 rightTag = m_findContactRightTagBuf[a];
		int32 aContactCount = 0;
		for (int b = a + 1; b < end; b++)
		{
			if (rightTag < m_proxyBuffer[b].tag) break;
			int32 bIdx = m_proxyBuffer[b].idx;
			if (!ShouldCollide(aIdx, bIdx)) break;
			AddContact(aIdx, bIdx, m_contactCount);
			if (++aContactCount >= MAX_CONTACTS_PER_PARTICLE) goto cnt;
		}
		const uint32 bottomLeftTag = m_findContactBottomLeftTagBuf[a];
		for (; c < end; c++)
		{
			if (bottomLeftTag <= m_proxyBuffer[c].tag) break;
		}
		const uint32 bottomRightTag = m_findContactBottomRightTagBuf[a];
		for (int b = c; b < end; b++)
		{
			if (bottomRightTag < m_proxyBuffer[b].tag) break;
			int32 bIdx = m_proxyBuffer[b].idx;
			if (!ShouldCollide(aIdx, bIdx)) break;
			AddContact(aIdx, bIdx, m_contactCount);
			//m_findContactIdxABuf[a][aContactCount] = aIdx;
			//m_findContactIdxBBuf[a][aContactCount] = bIdx;
			if (++aContactCount >= MAX_CONTACTS_PER_PARTICLE) goto cnt;
		}
		cnt:;
	}
	//cout << "FillBuffer " << GetTimeDif(t) << "ms\n";

	//t = Clock::now();
	//for (int i = 0; i < m_count; i++)
	//	for (int j = 0; j < m_findContactCountBuf[i]; j++)
	//		AddContact(m_findContactIdxABuf[i][j], m_findContactIdxBBuf[i][j], m_contactCount);
	//cout << "AddContacts " << GetTimeDif(t) << "ms\n";

	//std::snprintf(debugString, 64, "contactCount: %d", m_contactCount);
}
inline void b2ParticleSystem::AddContact(int32 a, int32 b, int32& contactCount)
{
	b2Vec3 d = m_positionBuffer[b] - m_positionBuffer[a];
	float32 distBtParticlesSq = (d.x * d.x) + (d.y * d.y);			//dot product
	if (distBtParticlesSq < m_squaredDiameter)
	{
		if (m_contactBufferSize <= contactCount)
			ResizeContactBuffers(contactCount * 2);
		b2ParticleContact& contact = m_partContactBuf[contactCount];
		contactCount++;

		float32 invD = b2InvSqrt(distBtParticlesSq);
		contact.idxA = a;
		contact.idxB = b;
		contact.flags = m_flagsBuffer[a] | m_flagsBuffer[b];
		int32 matAIdx = m_partMatIdxBuffer[a];
		int32 matBIdx = m_partMatIdxBuffer[b];
		contact.matFlags = m_partMatFlagsBuf[matAIdx] | m_partMatFlagsBuf[matBIdx];
		contact.weight = 1 - distBtParticlesSq * invD * m_inverseDiameter;
		float32 invM = 1 / (m_partMatMassBuf[matAIdx] + m_partMatMassBuf[matBIdx]);
		contact.mass = invM > 0 ? invM : 0;
		contact.normal = invD * b2Vec2(d);
	
	}
}
inline bool b2ParticleSystem::ShouldCollide(int32 a, int32 b) const
{
	if (std::abs(m_heightLayerBuffer[a] - m_heightLayerBuffer[b]) > 1) return false;
	int32 colGroupA = m_groupBuffer[m_partGroupIdxBuffer[a]]->m_collisionGroup;
	int32 colGroupB = m_groupBuffer[m_partGroupIdxBuffer[b]]->m_collisionGroup;

	//if (colGroupA < 0)
	//	return colGroupA != colGroupB;
	if (colGroupA > 0)
		return colGroupA == colGroupB;
	return true;
}
inline bool b2ParticleSystem::ShouldCollide(int32 i, b2Fixture* f) const
{
	if (!f->TestZPos(m_positionBuffer[i].z))
		return false;
	int32 partColGroup = m_groupBuffer[m_partGroupIdxBuffer[i]]->m_collisionGroup;
	if (partColGroup >= 0)								return true;
	if (f->GetFilterData().groupIndex >= 0)				return true;
	if (partColGroup != f->GetFilterData().groupIndex)	return true;
	return false;
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
		m_world->m_contactManager.m_contactFilter : NULL;
}

// Get the world's contact listener if any particles with the
// b2_particleContactListenerParticle flag are present in the system.
inline b2ContactListener* b2ParticleSystem::GetParticleContactListener() const
{
	return (m_allParticleFlags & b2_particleContactListenerParticle) ?
		m_world->m_contactManager.m_contactListener : NULL;
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
		Proxy& proxy = m_proxyBuffer[i];
		b2Vec3& pos = m_positionBuffer[proxy.idx];
		proxy.tag = computeTag(m_inverseDiameter * pos.x, m_inverseDiameter * pos.y);
	}
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
		m_world->m_contactManager.m_contactListener : NULL;
}

// Get the world's contact filter if any particles with the
// b2_fixtureContactFilterParticle flag are present in the system.
inline b2ContactFilter* b2ParticleSystem::GetFixtureContactFilter() const
{
	return (m_allParticleFlags & b2_fixtureContactFilterParticle) ?
		m_world->m_contactManager.m_contactFilter : NULL;
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
			if (bodyContact.idx == b2_invalidIndex ||
				!(particleFlagsBuffer[bodyContact.idx] &
				  b2_fixtureContactListenerParticle))
			{
				continue;
			}
			fixtureParticle->first = bodyContact.fixture;
			fixtureParticle->second = bodyContact.idx;
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
	explicit b2FixtureParticleQueryCallback(b2ParticleSystem* system)
	{
		m_system = system;
	}

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
		b2Fixture* fixture = m_system->m_fixtureBuffer[fixtureIdx];
		if (fixture->IsSensor())
		{
			return true;
		}
		const b2Shape* shape = fixture->GetShape();
		int32 childCount = shape->GetChildCount();
		for (int32 childIndex = 0; childIndex < childCount; childIndex++)
		{
			b2AABB aabb = fixture->GetAABB(childIndex);
			b2ParticleSystem::InsideBoundsEnumerator enumerator =
								m_system->GetInsideBoundsEnumerator(aabb);
			b2Vec2 bodyPos = fixture->GetBody()->GetPosition();
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
	b2ParticleSystem* m_system;
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
		fixtureParticleToFind.first = contact.fixture;
		fixtureParticleToFind.second = contact.idx;
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
	FixtureParticleSet fixtureSet(&m_world->m_stackAllocator);
	NotifyBodyContactListenerPreContact(&fixtureSet);

	if (m_stuckThreshold > 0)
	{
		for (int32 i = 0; i < m_count; i++)
		{
			// Detect stuck particles, see comment in
			// b2ParticleSystem::DetectStuckParticle()
			m_bodyContactCountBuffer[i] = 0;
			if (m_timestamp > (m_lastBodyContactStepBuffer[i] + 1))
			{
				m_consecutiveContactStepsBuffer[i] = 0;
			}
		}
	}
	m_bodyContactCount = 0;
	m_stuckParticleCount = 0;

	class UpdateBodyContactsCallback : public b2FixtureParticleQueryCallback
	{
		void ReportFixtureAndParticle(int32 fixtureIdx, int32 childIndex, int32 a)
		{
			b2Fixture* fixture = m_system->m_fixtureBuffer[fixtureIdx];
			if (m_system->ShouldCollide(a, fixture))
			{
				b2Vec2 ap = b2Vec2(m_system->m_positionBuffer[a]);
				float32 d;
				b2Vec2 n;

				fixture->ComputeDistance(ap, &d, &n, childIndex);
				if (d < m_system->m_particleDiameter)
				{
					int32 bIdx = fixture->GetBodyIdx();
					b2Body* b = m_system->m_bodyBuffer[bIdx];
					b2Vec2 bp = b->GetWorldCenter();
					float32 bm = b->GetMass();
					float32 bI =
						b->GetInertia() - bm * b->GetLocalCenter().LengthSquared();
					float32 invBm = bm > 0 ? 1 / bm : 0;
					float32 invBI = bI > 0 ? 1 / bI : 0;
					float32 invAm =
						m_system->m_flagsBuffer[a] &
						b2_wallParticle ? 0 : m_system->m_partMatInvMassBuf[m_system->m_partMatIdxBuffer[a]];
					b2Vec2 rp = ap - bp;
					float32 rpn = b2Cross(rp, n);
					float32 invM = invAm + invBm + invBI * rpn * rpn;

					int32& bodyContactCount = m_system->m_bodyContactCount;
					if (bodyContactCount+1 >= m_system->m_bodyContactBufferSize)
						m_system->ResizeBodyContactBuffers(bodyContactCount * 2);
					b2PartBodyContact& contact = m_system->m_bodyContactBuf[bodyContactCount];
					bodyContactCount++;
					contact.idx = a;
					contact.body = b;
					contact.fixture = fixture;
					contact.weight = 1 - d * m_system->m_inverseDiameter;
					contact.normal = -n;
					contact.mass = invM > 0 ? 1 / invM : 0;
					m_system->DetectStuckParticle(a);
				}
			}
		}

		b2ContactFilter* m_contactFilter;

	public:
		UpdateBodyContactsCallback(
			b2ParticleSystem* system, b2ContactFilter* contactFilter):
			b2FixtureParticleQueryCallback(system)
		{
			m_contactFilter = contactFilter;
		}
	} callback(this, GetFixtureContactFilter());

	b2AABB aabb;
	ComputeAABB(&aabb);
	m_world->QueryAABB(&callback, aabb);
	
	if (m_def.strictContactCheck)
	{
		RemoveSpuriousBodyContacts();
	}

	NotifyBodyContactListenerPostContact(fixtureSet);
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


void b2ParticleSystem::SolveCollision(const b2TimeStep& step)
{
	// This function detects particles which are crossing boundary of bodies
	// and modifies velocities of them so that they will move just in front of
	// boundary. This function function also applies the reaction force to
	// bodies as precisely as the numerical stability is kept.
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
		// Call the contact filter if it's set, to determine whether to
		// filter this contact.  Returns true if contact calculations should
		// be performed, false otherwise.
		/*inline bool ShouldCollide(b2Fixture * const fixture,
								  int32 particleIndex)
		{
			if (m_contactFilter) {
				const uint32* const flags = m_system->GetFlagsBuffer();
				if (flags[particleIndex] & b2_fixtureContactFilterParticle) {
					return m_contactFilter->ShouldCollide(fixture, m_system,
														  particleIndex);
				}
			}
			return true;
		}*/

		void ReportFixtureAndParticle(int32 fixtureIdx, int32 childIndex, int32 a)
		{
			b2Fixture* fixture = m_system->m_fixtureBuffer[fixtureIdx];
			if (m_system->ShouldCollide(a, fixture)) {
				b2Body* body = fixture->GetBody();
				b2Vec2 ap = b2Vec2(m_system->m_positionBuffer[a]);
				b2Vec2 av = b2Vec2(m_system->m_velocityBuffer[a]);
				b2RayCastOutput output;
				b2RayCastInput input;
				if (m_system->m_iterationIndex == 0)
				{
					// Put 'ap' in the local space of the previous frame
					b2Vec2 p1 = b2MulT(body->m_xf0, ap);
					if (fixture->GetShape()->GetType() == b2Shape::e_circle)
					{
						// Make relative to the center of the circle
						p1 -= body->GetLocalCenter();
						// Re-apply rotation about the center of the
						// circle
						p1 = b2Mul(body->m_xf0.q, p1);
						// Subtract rotation of the current frame
						p1 = b2MulT(body->m_xf.q, p1);
						// Return to local space
						p1 += body->GetLocalCenter();
					}
					// Return to global space and apply rotation of current frame
					input.p1 = b2Mul(body->m_xf, p1);
				}
				else
				{
					input.p1 = ap;
				}
				input.p2 = ap + m_step.dt * av;
				input.maxFraction = 1;
				if (fixture->RayCast(&output, input, childIndex))
				{
					b2Vec2 n = output.normal;
					b2Vec2 p =
						(1 - output.fraction) * input.p1 +
						output.fraction * input.p2 +
						b2_linearSlop * n;
					b2Vec2 v = m_step.inv_dt * (p - ap);
					m_system->m_velocityBuffer[a] = b2Vec3(v);
					b2Vec2 f = m_step.inv_dt *
						m_system->m_partMatMassBuf[m_system->m_partMatIdxBuffer[a]] * (av - v);
					m_system->ParticleApplyForce(a, f);
				}
			}
		}

		b2TimeStep m_step;
		b2ContactFilter* m_contactFilter;

	public:
		SolveCollisionCallback(
			b2ParticleSystem* system, const b2TimeStep& step, b2ContactFilter* contactFilter) :
			b2FixtureParticleQueryCallback(system)
		{
			m_step = step;
			m_contactFilter = contactFilter;
		}
	} callback(this, step, GetFixtureContactFilter());
	m_world->QueryAABB(&callback, aabb);
}

void b2ParticleSystem::SolveBarrier(const b2TimeStep& step)
{
	// If a particle is passing between paired barrier particles,
	// its velocity will be decelerated to avoid passing.
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
			b2ParticleGroup* aGroup = m_groupBuffer[aGroupIdx];
			b2ParticleGroup* bGroup = m_groupBuffer[bGroupIdx];
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
					b2ParticleGroup* cGroup = m_groupBuffer[cGroupIdx];
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
						t = - e0 / e1;
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
						float32 t1 = (- e1 - sqrtDet) / (2 * e2);
						float32 t2 = (- e1 + sqrtDet) / (2 * e2);
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
						float32 mass = cGroup->GetMass();
						float32 inertia = cGroup->GetInertia();
						if (mass > 0)
						{
							cGroup->m_linearVelocity += 1 / mass * f;
						}
						if (inertia > 0)
							cGroup->m_angularVelocity +=
								b2Cross(pc - cGroup->GetCenter(), f) / inertia;
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



void b2ParticleSystem::SolveInit() 
{
	if (!m_world->m_stepComplete || m_count == 0 || m_step.dt <= 0.0f)
		return;

	//if (!m_expireTimeBuf.empty())
	//	SolveLifetimes(m_step);
	if (m_allParticleFlags & b2_zombieParticle)
		SolveZombie();
	if (m_needsUpdateAllParticleFlags)
		UpdateAllParticleFlags();
	if (m_needsUpdateAllGroupFlags)
		UpdateAllGroupFlags();
}

void b2ParticleSystem::UpdateContacts(bool exceptZombie)
{
	if (!m_world->m_stepComplete || m_count == 0 || m_step.dt <= 0.0f)
		return;
	if (m_paused)
		return;


	Time sortStart = Clock::now();
	// Update Proxy Tags and Sort by Tags
	UpdateProxies();
	SortProxies();
	float32 sortTime = GetTimeDif(sortStart);

	//find Body COntacts in seperate Thread
	//Time findStart = Clock::now();
	findBodyContactsThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)StartBodyContactThread, this, 0, NULL);
	UpdateBodyContacts();

	// While findBodyContactsThread is running
	//clock_t clockPartContacts = clock();
	FindContacts();
	//cout << "FindContacts " << (clock() - clockPartContacts) * 1000 / CLOCKS_PER_SEC << "ms\n";

	WaitForSingleObject(findBodyContactsThread, INFINITE);
	//float32 findTime = GetTimeDif(findStart);


	if (exceptZombie)
	{
		RemoveFromVectorIf(m_partContactBuf, m_contactCount, b2ParticleContactIsZombie, true);
	}
	//snprintf(debugString, 64, "Sort %fms  find %fms", sortTime, findTime);
}

void b2ParticleSystem::SolveIteration(int32 iteration)
{
	m_iterationIndex = iteration;
	++m_timestamp;
	subStep = m_step;
	subStep.dt /= m_step.particleIterations;
	subStep.inv_dt *= m_step.particleIterations;

	//clock_t clockSolve = clock();

	ComputeWeight(
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	);

	if (m_allGroupFlags & b2_particleGroupNeedsUpdateDepth)
		ComputeDepth();
	if (m_allParticleFlags & b2_reactiveParticle)
		UpdatePairsAndTriadsWithReactiveParticles();
	if (m_hasForce)
		SolveForce(subStep);
	if (m_allParticleFlags & b2_viscousParticle)
		SolveViscous();
	if (m_allParticleFlags & b2_repulsiveParticle)
		SolveRepulsive(subStep);

	if (m_allParticleFlags & b2_powderParticle)
		SolvePowder(subStep);

	if (m_allParticleFlags & b2_tensileParticle)
		SolveTensile(subStep);


	if (m_allGroupFlags & b2_solidParticleGroup)
		SolveSolid(subStep);
	if (m_allParticleFlags & b2_colorMixingParticle)
		SolveColorMixing();

	SolveGravity(subStep);

	if (m_allParticleFlags & b2_staticPressureParticle)
		SolveStaticPressure(subStep);

	SolvePressure(subStep);
	SolveDamping(subStep);
	SolveSlowDown(subStep);
	if (m_allParticleFlags & k_extraDampingFlags)
		SolveExtraDamping();

	// SolveElastic and SolveSpring refer the current velocities for
	// numerical stability, they should be called as late as possible.
	if (m_allParticleFlags & b2_elasticParticle)
		SolveElastic(subStep);
	if (m_allParticleFlags & b2_springParticle)
		SolveSpring(subStep);

	//LimitVelocity(subStep);
	if (m_allGroupFlags & b2_rigidParticleGroup)
		SolveRigidDamping();
	if (m_allParticleFlags & b2_barrierParticle)
		SolveBarrier(subStep);


	// SolveCollision, SolveRigid and SolveWall should be called after
	// other force functions because they may require particles to have
	// specific velocities.
	SolveCollision(subStep);
	if (m_allGroupFlags & b2_rigidParticleGroup)
		SolveRigid(subStep);
	if (m_allParticleFlags & b2_wallParticle)
		SolveWall();



	// FIRE
	if (m_allParticleFlags & b2_flameParticle)
	{
		SolveFlame(subStep);
		if (m_world->m_allMaterialFlags & b2_flammableMaterial)
			SolveIgnite();
		if (m_world->m_allMaterialFlags & b2_extinguishingMaterial)
			SolveExtinguish();
	}
	if (m_allParticleFlags & b2_burningParticle)
		SolveBurning(subStep);

	// WATER
	if (m_allParticleFlags & b2_waterParticle)
		SolveWater();
	SolveFreeze();


	// AIR
	if (m_allParticleFlags & b2_airParticle)
		SolveAir();

	// HEAT MANAGEMENT
	if (m_world->m_allMaterialFlags & b2_heatConductingMaterial)
		SolveHeatConduct(subStep);
	if (m_allParticleFlags & b2_heatLoosingParticle)
		SolveLooseHeat(subStep);

	if (m_world->m_allMaterialFlags & (b2_changeWhenColdMaterial | b2_changeWhenHotMaterial))
		SolveChangeMat();

	//Destroy Dead Particles
	SolveDestroyDead();




	// The particle positions can be updated only at the end of substep.
	if (m_allParticleFlags & b2_fallingParticle)
		SolveFalling(subStep);
	if (m_allParticleFlags & b2_risingParticle)
		SolveRising(subStep);
	for (int32 i = 0; i < m_count; i++)
	{
		m_positionBuffer[i] += subStep.dt * m_velocityBuffer[i];
	}
	//cout << "Solve " << (clock() - clockSolve) * 1000 / CLOCKS_PER_SEC << "ms\n";
}

void b2ParticleSystem::SolveEnd() 
{

}


float32 b2ParticleSystem::GetTimeDif(Time start, Time end)
{
	return std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / 1000000.0f;
}
float32 b2ParticleSystem::GetTimeDif(Time start)
{
	return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start).count() / 1000000.0f;
}

void b2ParticleSystem::SolveFalling(const b2TimeStep& step)
{
	for (int32 k = 0; k < m_count; k++)
	{
		if (m_flagsBuffer[k] & b2_fallingParticle & ~b2_controlledParticle)
		{
			float32 z = m_positionBuffer[k].z - step.dt;
			int layer = GetLayerFromZ(z);
			if (layer <= m_lowestLayer)
			{
				layer = m_lowestLayer;
				z = layer * m_pointsPerLayer;
				m_flagsBuffer[k] &= ~b2_fallingParticle;
			}
			m_heightLayerBuffer[k] = layer;
			m_positionBuffer[k].z = z;
		}
	}
}

void b2ParticleSystem::SolveRising(const b2TimeStep& step)
{
	for (int32 k = 0; k < m_count; k++)
	{
		if (m_flagsBuffer[k] & b2_risingParticle & ~b2_controlledParticle)
		{
			float32 z = m_positionBuffer[k].z + step.dt;
			int layer = GetLayerFromZ(z);
			if (layer >= m_highestLayer)
			{
				layer = m_highestLayer;
				z = layer * m_pointsPerLayer;
				m_flagsBuffer[k] &= ~b2_risingParticle;
			}
			m_heightLayerBuffer[k] = layer;
			m_positionBuffer[k].z = z;
		}
	}
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

void b2ParticleSystem::UpdateAllGroupFlags()
{
	m_allGroupFlags = 0;
	for (int i = 0; i < m_groupCount; i++)
	{
		if (m_groupBuffer[i])
			m_allGroupFlags |= m_groupBuffer[i]->m_groupFlags;
	}
	m_needsUpdateAllGroupFlags = false;
}

void b2ParticleSystem::LimitVelocity(const b2TimeStep& step)
{
	float32 criticalVelocitySquared = GetCriticalVelocitySquared(step);
	for (int32 i = 0; i < m_count; i++)
	{
		b2Vec2 v = b2Vec2(m_velocityBuffer[i]);
		float32 v2 = b2Dot(v, v);
		if (v2 > criticalVelocitySquared)
		{
			int32 s = b2Sqrt(criticalVelocitySquared / v2);
			m_velocityBuffer[i] *= s;

		}
	}
}

void b2ParticleSystem::SolveGravity(const b2TimeStep& step)
{
	b2Vec2 gravity = step.dt * m_def.gravityScale * m_world->GetGravity();
	for (int32 i = 0; i < m_count; i++)
	{
		m_velocityBuffer[i] += gravity;
	}
}

void b2ParticleSystem::SolveStaticPressure(const b2TimeStep& step)
{
	RequestBuffer(m_staticPressureBuf, hasStaticPressureBuf);
	float32 criticalPressure = GetCriticalPressure(step);
	float32 pressurePerWeight = m_def.staticPressureStrength * criticalPressure;
	float32 maxPressure = b2_maxParticlePressure * criticalPressure;
	float32 relaxation = m_def.staticPressureRelaxation;
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

void b2ParticleSystem::SolvePressure(const b2TimeStep& step)
{
	// calculates pressure as a linear function of density
	float32 criticalPressure = GetCriticalPressure(step);
	float32 pressurePerWeight = m_def.pressureStrength * criticalPressure;
	float32 maxPressure = b2_maxParticlePressure * criticalPressure;
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
	float32 velocityPerPressure = step.dt / (m_def.density * m_particleDiameter);
	for (int32 k = 0; k < m_bodyContactCount; k++)
	{
		const b2PartBodyContact& contact = m_bodyContactBuf[k];
		int32 a = contact.idx;
		b2Body* b = contact.body;
		float32 w = contact.weight;
		float32 m = contact.mass;
		b2Vec2 n = contact.normal;
		b2Vec2 p = b2Vec2(m_positionBuffer[a]);
		float32 h = m_accumulationBuf[a] + pressurePerWeight * w;
		b2Vec2 f = velocityPerPressure * w * m * h * n;
		float32 invMass = m_partMatInvMassBuf[m_partMatIdxBuffer[a]];
		m_velocityBuffer[a] -= invMass * f;
		b->ApplyLinearImpulse(f, p, true);
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

void b2ParticleSystem::SolveDamping(const b2TimeStep& step)
{
	// reduces normal velocity of each contact
	float32 linearDamping = m_def.dampingStrength;
	float32 quadraticDamping = 1 / GetCriticalVelocity(step);
	for (int32 k = 0; k < m_bodyContactCount; k++)
	{
		const b2PartBodyContact& contact = m_bodyContactBuf[k];
		int32 a = contact.idx;
		b2Body* b = contact.body;
		float32 w = contact.weight;
		float32 m = contact.mass;
		b2Vec2 n = contact.normal;
		b2Vec2 p = b2Vec2(m_positionBuffer[a]);
		b2Vec2 v = b->GetLinearVelocityFromWorldPoint(p) -
				   b2Vec2(m_velocityBuffer[a]);
		float32 vn = b2Dot(v, n);
		if (vn < 0)
		{
			float32 damping =
				b2Max(linearDamping * w, b2Min(- quadraticDamping * vn, 0.5f));
			b2Vec2 f = damping * m * vn * n;
			float32 invMass = m_partMatInvMassBuf[m_partMatIdxBuffer[a]];
			m_velocityBuffer[a] += invMass * f;
			b->ApplyLinearImpulse(-f, p, true);
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
				b2Max(linearDamping * w, b2Min(- quadraticDamping * vn, 0.5f));
			b2Vec2 f = damping * m * vn * n;
			DistributeForceDamp(a, b, f);
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
			float slowDown = (m_heightLayerBuffer[k] > 0) ? d : dd;
			m_velocityBuffer[k] *= slowDown;
		}
	}
}

inline bool b2ParticleSystem::IsRigidGroup(b2ParticleGroup* group) const
{
	return group->m_groupFlags & b2_rigidParticleGroup;
}

inline b2Vec2 b2ParticleSystem::GetLinearVelocity(
	b2ParticleGroup* group, int32 particleIndex,
	const b2Vec2 &point)
{
	if (IsRigidGroup(group))
		return group->GetLinearVelocityFromWorldPoint(point);
	else
		return b2Vec2(m_velocityBuffer[particleIndex]);
}

inline void b2ParticleSystem::InitDampingParameter(
	float32* invMass, float32* invInertia, float32* tangentDistance,
	float32 mass, float32 inertia, const b2Vec2& center,
	const b2Vec2& point, const b2Vec2& normal) const
{
	*invMass = mass > 0 ? 1 / mass : 0;
	*invInertia = inertia > 0 ? 1 / inertia : 0;
	*tangentDistance = b2Cross(point - center, normal);
}

inline void b2ParticleSystem::InitDampingParameterWithRigidGroupOrParticle(
	float32* invMass, float32* invInertia, float32* tangentDistance,
	bool isRigidGroup, b2ParticleGroup* group, int32 particleIndex,
	const b2Vec2& point, const b2Vec2& normal)
{
	if (isRigidGroup)
	{
		InitDampingParameter(
			invMass, invInertia, tangentDistance,
			group->GetMass(), group->GetInertia(), group->GetCenter(),
			point, normal);
	}
	else
	{
		uint32 flags = m_flagsBuffer[particleIndex];
		InitDampingParameter(
			invMass, invInertia, tangentDistance,
			//flags & b2_wallParticle ? 0 : GetParticleMass(), 0, point,
			flags & b2_wallParticle ? 0 : m_partMatMassBuf[m_partMatIdxBuffer[particleIndex]], 0, point, //CHANGED
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
	bool isRigidGroup, b2ParticleGroup* group, int32 particleIndex,
	float32 impulse, const b2Vec2& normal)
{
	if (isRigidGroup)
	{
		group->m_linearVelocity += impulse * invMass * normal;
		group->m_angularVelocity += impulse * tangentDistance * invInertia;
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
	float32 damping = m_def.dampingStrength;
	for (int32 k = 0; k < m_bodyContactCount; k++)
	{
		const b2PartBodyContact& contact = m_bodyContactBuf[k];
		int32 a = contact.idx;
		int32 aGroupIdx = m_partGroupIdxBuffer[a];
		b2ParticleGroup* aGroup = m_groupBuffer[aGroupIdx];
		if (IsRigidGroup(aGroup))
		{
			b2Body* b = contact.body;
			b2Vec2 n = contact.normal;
			float32 w = contact.weight;
			b2Vec2 p = b2Vec2(m_positionBuffer[a]);
			b2Vec2 v = b->GetLinearVelocityFromWorldPoint(p) -
				aGroup->GetLinearVelocityFromWorldPoint(p);
			float32 vn = b2Dot(v, n);
			if (vn < 0)
				// The group's average velocity at particle position 'p' is pushing
				// the particle into the body.
			{
				float32 invMassA, invInertiaA, tangentDistanceA;
				float32 invMassB, invInertiaB, tangentDistanceB;
				InitDampingParameterWithRigidGroupOrParticle(
					&invMassA, &invInertiaA, &tangentDistanceA,
					true, aGroup, a, p, n);
				InitDampingParameter(
					&invMassB, &invInertiaB, &tangentDistanceB,
					b->GetMass(),
					// Calculate b->m_I from public functions of b2Body.
					b->GetInertia() -
					b->GetMass() * b->GetLocalCenter().LengthSquared(),
					b->GetWorldCenter(),
					p, n);
				float32 f = damping * b2Min(w, 1.0f) * ComputeDampingImpulse(
					invMassA, invInertiaA, tangentDistanceA,
					invMassB, invInertiaB, tangentDistanceB,
					vn);
				ApplyDamping(
					invMassA, invInertiaA, tangentDistanceA,
					true, aGroup, a, f, n);
				b->ApplyLinearImpulse(-f * n, p, true);
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
		b2ParticleGroup* aGroup = m_groupBuffer[aGroupIdx];
		b2ParticleGroup* bGroup = m_groupBuffer[bGroupIdx];
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
					&invMassA, &invInertiaA, &tangentDistanceA,
					aRigid, aGroup, a,
					p, n);
				InitDampingParameterWithRigidGroupOrParticle(
					&invMassB, &invInertiaB, &tangentDistanceB,
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

void b2ParticleSystem::SolveExtraDamping()
{
	// Applies additional damping force between bodies and particles which can
	// produce strong repulsive force. Applying damping force multiple times
	// is effective in suppressing vibration.
	for (int32 k = 0; k < m_bodyContactCount; k++)
	{
		const b2PartBodyContact& contact = m_bodyContactBuf[k];
		int32 a = contact.idx;
		if (m_flagsBuffer[a] & k_extraDampingFlags)
		{
			b2Body* b = contact.body;
			float32 m = contact.mass;
			b2Vec2 n = contact.normal;
			b2Vec2 p = b2Vec2(m_positionBuffer[a]);
			b2Vec2 v =
				b->GetLinearVelocityFromWorldPoint(p) -
				b2Vec2(m_velocityBuffer[a]);
			float32 vn = b2Dot(v, n);
			if (vn < 0)
			{
				b2Vec2 f = 0.5f * m * vn * n;
				//m_velocityBuffer.data[a] += GetParticleInvMass() * f;
				float32 invMass = m_partMatInvMassBuf[m_partMatIdxBuffer[a]];
				m_velocityBuffer[a] += invMass * f;
				b->ApplyLinearImpulse(-f, p, true);
			}
		}
	}
}

void b2ParticleSystem::SolveWall()
{
	for (int32 i = 0; i < m_count; i++)
	{
		if (m_flagsBuffer[i] & b2_wallParticle)
		{
			m_velocityBuffer[i].SetZero();
		}
	}
}

void b2ParticleSystem::SolveRigid(const b2TimeStep& step)
{
	for (int32 k = 0; k < m_groupCount; k++)
	{
		b2ParticleGroup* group = m_groupBuffer[k];
		if (group)
		{
			if (group->m_groupFlags & b2_rigidParticleGroup)
			{
				group->UpdateStatistics();
				b2Rot rotation(step.dt * group->m_angularVelocity);
				b2Vec2 center = group->m_center;
				b2Vec2 linVel = group->m_linearVelocity;
				b2Transform transform(center + step.dt * linVel -
					b2Mul(rotation, center), rotation);
				group->m_transform = b2Mul(transform, group->m_transform);
				b2Transform velocityTransform;
				velocityTransform.p.x = step.inv_dt * transform.p.x;
				velocityTransform.p.y = step.inv_dt * transform.p.y;
				velocityTransform.q.s = step.inv_dt * transform.q.s;
				velocityTransform.q.c = step.inv_dt * (transform.q.c - 1);
				for (int32 i = group->m_firstIndex; i < group->m_lastIndex; i++)
				{
					b2Vec2 vel = b2Mul(velocityTransform,
						b2Vec2(m_positionBuffer[i]));
					m_velocityBuffer[i] = vel;
				}
			}
		}
	}
}

void b2ParticleSystem::SolveElastic(const b2TimeStep& step)
{
	float32 elasticStrength = step.inv_dt * m_def.elasticStrength;
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

void b2ParticleSystem::SolveSpring(const b2TimeStep& step)
{
	float32 springStrength = step.inv_dt * m_def.springStrength;
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

void b2ParticleSystem::SolveTensile(const b2TimeStep& step)
{
	b2Assert(!m_accumulation2Buffer.empty());
	for (int32 i = 0; i < m_count; i++)
	{
		m_accumulation2Buf[i] = b2Vec2_zero;
	}
	for (int32 k = 0; k < m_contactCount; k++)
	{
		const b2ParticleContact& contact = m_partContactBuf[k];
		if (contact.flags & b2_tensileParticle)
		{
			int32 a = contact.idxA;
			int32 b = contact.idxB;
			float32 w = contact.weight;
			b2Vec2 n = contact.normal;
			b2Vec2 weightedNormal = (1 - w) * w * n;
			m_accumulation2Buf[a] -= weightedNormal;
			m_accumulation2Buf[b] += weightedNormal;
		}
	}
	float32 criticalVelocity = GetCriticalVelocity(step);
	float32 pressureStrength = m_def.surfaceTensionPressureStrength
		* criticalVelocity;
	float32 normalStrength = m_def.surfaceTensionNormalStrength
		* criticalVelocity;
	float32 maxVelocityVariation = b2_maxParticleForce * criticalVelocity;
	for (int32 k = 0; k < m_contactCount; k++)
	{
		const b2ParticleContact& contact = m_partContactBuf[k];
		if (contact.flags & b2_tensileParticle)
		{
			int32 a = contact.idxA;
			int32 b = contact.idxB;
			float32 w = contact.weight;
			float32 m = contact.mass;
			b2Vec2 n = contact.normal;
			float32 h = m_weightBuffer[a] + m_weightBuffer[b];
			b2Vec2 s = m_accumulation2Buf[b] - m_accumulation2Buf[a];
			float32 fn = b2Min(
				pressureStrength * (h - 2) + normalStrength * b2Dot(s, n),
				maxVelocityVariation) * w;
			b2Vec2 f = fn * n * m;
			DistributeForce(a, b, f);
		}
	}
}

void b2ParticleSystem::SolveViscous()
{
	float32 viscousStrength = m_def.viscousStrength;
	for (int32 k = 0; k < m_bodyContactCount; k++)
	{
		const b2PartBodyContact& contact = m_bodyContactBuf[k];
		int32 a = contact.idx;
		if (m_flagsBuffer[a] & b2_viscousParticle)
		{
			b2Body* b = contact.body;
			float32 w = contact.weight;
			float32 m = contact.mass;
			b2Vec2 p = b2Vec2(m_positionBuffer[a]);
			b2Vec2 v = b->GetLinearVelocityFromWorldPoint(p) -
				b2Vec2(m_velocityBuffer[a]);
			b2Vec2 f = viscousStrength * m * w * v;
			float32 invMass = m_partMatInvMassBuf[m_partMatIdxBuffer[a]];
			m_velocityBuffer[a] += invMass * f;
			b->ApplyLinearImpulse(-f, p, true);
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

void b2ParticleSystem::SolveRepulsive(const b2TimeStep& step)
{
	float32 repulsiveStrength =
		m_def.repulsiveStrength * GetCriticalVelocity(step);
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

void b2ParticleSystem::SolvePowder(const b2TimeStep& step)
{
	float32 powderStrength = m_def.powderStrength * GetCriticalVelocity(step);
	float32 minWeight = 1.0f - b2_particleStride;
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

void b2ParticleSystem::SolveSolid(const b2TimeStep& step)
{
	// applies extra repulsive force from solid particle groups
	b2Assert(m_hasDepth);
	float32 ejectionStrength = step.inv_dt * m_def.ejectionStrength;
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

void b2ParticleSystem::SolveForce(const b2TimeStep& step)
{
	for (int32 i = 0; i < m_count; i++)
	{
		float32 invMassStep = step.dt * m_partMatInvMassBuf[m_partMatIdxBuffer[i]];
		m_velocityBuffer[i] += invMassStep * m_forceBuffer[i];
	}
	if (m_allParticleFlags & b2_controlledParticle)
	{
		for (int32 k = 0; k < m_count; k++)
		{
			if (m_flagsBuffer[k] & b2_controlledParticle)
			{
				float32 z = m_positionBuffer[k].z += m_forceBuffer[k].z * step.dt;
				m_heightLayerBuffer[k] = GetLayerFromZ(z);
			}
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


void b2ParticleSystem::SolveHeatConduct(const b2TimeStep& step)
{
	// transfers heat to adjacent particles
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
		int32 a = contact.idx;
		b2Body* b = contact.body;
		int32 matIdxP = m_partMatIdxBuffer[a];
		b2BodyMaterial* matB = b->m_material;
		if (m_partMatFlagsBuf[matIdxP] & matB->m_matFlags & b2_heatConductingMaterial)
		{
			float32 conductivityP = m_partMatHeatConductivityBuf[matIdxP];
			float32 conductivityB = matB->m_heatConductivity;
			if (conductivityP > 0 && conductivityB > 0)
			{
				float32 heatP = m_heatBuffer[a];
				float32 heatB = b->m_heat;
				if (abs(heatB - heatP) > 1.0f)
				{
					float32 changeHeat = step.dt * 30.0f * (conductivityP * conductivityB)
										 * (heatB - heatP);
					float32 massP = m_partMatMassBuf[matIdxP];
					float32 massB = b->GetMass();
					float32 invCombinedMass = 0.999f / (massP + massB);
					m_heatBuffer[a] += changeHeat * (0.001f + massB * invCombinedMass);
					b->m_heat		-= changeHeat * (0.001f + massP * invCombinedMass);

					/*float32 changeHeat = step.dt * (conductivityP * conductivityB)
						* (heatB - heatP);
					float32 massP = matP->m_mass;
					float32 massB = b->GetMass();
					float32 invCombinedMass = 0.5f / (massP + massB);
					m_heatBuffer[i] += changeHeat * (0.25f + massB * invCombinedMass);
					b->m_heat			 -= changeHeat * (0.25f + massP * invCombinedMass);*/
				}
			}
		}
	}
}

void b2ParticleSystem::SolveLooseHeat(const b2TimeStep& step)
{
	for (int32 k = 0; k < m_count; k++)
	{
		if (m_flagsBuffer[k] & b2_heatLoosingParticle)
		{
			int32 matIdx = m_partMatIdxBuffer[k];
			float32 loss = step.dt * m_partMatHeatConductivityBuf[matIdx] * (m_heatBuffer[k] - m_roomTemp);
			if (abs(loss) > step.dt)
			{
				m_heatBuffer[k] -= loss * (1 - pow(m_heatLossRatio,
					//(2.0f - (m_weightBuffer[k] < 0 ? 0 : m_weightBuffer[k] > 1 ? 1 : m_weightBuffer[k])) *
					0.0005f * m_partMatInvMassBuf[matIdx]));
			}
		}
	}
}

void b2ParticleSystem::SolveBurning(const b2TimeStep& step)
{
	for (int32 k = 0; k < m_count; k++)
	{
		if (m_flagsBuffer[k] & b2_burningParticle)
		{
			// loose health
			float loss = step.dt * m_heatBuffer[k] * 0.001f * m_partMatInvStabilityBuf[m_partMatIdxBuffer[k]];
			if (loss > FLT_EPSILON)
			{
				m_healthBuffer[k] -= loss;
			}
		}
	}
}

void b2ParticleSystem::SolveFlame(const b2TimeStep& step)
{
	for (int32 k = 0; k < m_count; k++)
	{
		if (m_flagsBuffer[k] & b2_flameParticle)
		{
			// rise heat
			float& heat = m_heatBuffer[k];
			heat += step.dt * 1000.0f;

			// loose health
			int32 partMatIdx = m_partMatIdxBuffer[k];
			float loss = step.dt * heat * 0.001f * m_partMatInvStabilityBuf[partMatIdx];
			if (loss > FLT_EPSILON)
				m_healthBuffer[k] -= loss;
			if (heat < m_partMatColderThanBuf[partMatIdx])
				DestroyParticle(k);
			//cout << "heat:" << m_heatBuffer[k] << " loss:" << loss << " health:" << m_healthBuffer[k] << "\n";
		}
	}
}

void b2ParticleSystem::SolveIgnite()
{
	for (int32 k = 0; k < m_bodyContactCount; k++)
	{
		const b2PartBodyContact& contact = m_bodyContactBuf[k];
		int32 a = contact.idx;
		b2Body* b = contact.body;
		if (m_flagsBuffer[a] & b2_flameParticle
			&& b->GetFlags() & b2_inflammableBody
			&& b->m_material->m_matFlags & b2_flammableMaterial
			&& b->m_material->m_ignitionPoint <= b->m_heat)
		{
			b->AddFlags((uint16)b2_burningBody);
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

void b2ParticleSystem::SolveExtinguish()
{
	for (int32 k = 0; k < m_bodyContactCount; k++)
	{
		const b2PartBodyContact& contact = m_bodyContactBuf[k];
		int32 a = contact.idx;
		b2Body* b = contact.body;
		int32 partMatIdx = m_partMatIdxBuffer[a];
		if (m_partMatFlagsBuf[partMatIdx] & b2_extinguishingMaterial
			&& b->GetFlags() & b2_burningBody
			&& b->m_heat < m_partMatIgnitionPointBuf[partMatIdx])
		{
			b->RemFlags(b2_burningBody);
			b->AddFlags(b2_wetBody);
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

void b2ParticleSystem::SolveWater()
{
	// make Objects wet
	for (int32 k = 0; k < m_bodyContactCount; k++)
	{
		const b2PartBodyContact& contact = m_bodyContactBuf[k];
		int32 a = contact.idx;
		b2Body* b = contact.body;
		if (m_flagsBuffer[a] & b2_extinguishingMaterial
			&& b->GetFlags() & b2_burningBody
			&& b->m_material->m_matFlags & b2_flammableMaterial
			&& b->m_material->m_extinguishingPoint > b->m_heat)
		{
			b->RemFlags((int16)b2_burningBody);
		}
	}
}
void b2ParticleSystem::SolveAir()
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

void b2ParticleSystem::SolveChangeMat()
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

void b2ParticleSystem::SolveFreeze()
{
	
}


void b2ParticleSystem::SolveDestroyDead()
{
	for (int32 k = 0; k < m_count; k++)
	{
		if (m_healthBuffer[k] < 0)
			DestroyParticle(k);
			//m_flagsBuffer[k] |= b2_zombieParticle;
	}
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
				m_world->m_destructionListener;
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
				m_heightLayerBuffer[newCount] = m_heightLayerBuffer[i];
				if (!m_lastBodyContactStepBuffer.empty())
				{
					m_lastBodyContactStepBuffer[newCount] =
						m_lastBodyContactStepBuffer[i];
				}
				if (!m_bodyContactCountBuffer.empty())
				{
					m_bodyContactCountBuffer[newCount] =
						m_bodyContactCountBuffer[i];
				}
				if (!m_consecutiveContactStepsBuffer.empty())
				{
					m_consecutiveContactStepsBuffer[newCount] =
						m_consecutiveContactStepsBuffer[i];
				}
				m_positionBuffer[newCount]  = m_positionBuffer[i];
				m_velocityBuffer[newCount]  = m_velocityBuffer[i];
				m_partGroupIdxBuffer[newCount]   = m_partGroupIdxBuffer[i];
				m_partMatIdxBuffer[newCount] = m_partMatIdxBuffer[i];
				if (m_hasForce)
				{
					m_forceBuffer[newCount] = m_forceBuffer[i];
				}
				if (!m_staticPressureBuf.empty())
				{
					m_staticPressureBuf[newCount] =
						m_staticPressureBuf[i];
				}
				if (m_hasDepth)
				{
					m_depthBuffer[newCount] = m_depthBuffer[i];
				}
				if (m_colorBuffer.data())
				{
					m_colorBuffer[newCount] = m_colorBuffer[i];
				}
				m_heatBuffer[newCount] = m_heatBuffer[i];
				m_healthBuffer[newCount] = m_healthBuffer[i];

				if (m_userDataBuffer.data())
				{
					m_userDataBuffer[newCount] = m_userDataBuffer[i];
				}
				if (!m_expireTimeBuf.empty())
				{
					m_expireTimeBuf[newCount] = m_expireTimeBuf[i];
				}
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
			return contact.idx < 0;
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
		b2PartBodyContact& contact = m_bodyContactBuf[k];
		contact.idx = newIndices[contact.idx];
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
		b2ParticleGroup* group = m_groupBuffer[k];
		if (group)
		{
			int32 firstIndex = newCount;
			int32 lastIndex = 0;
			bool modified = false;
			for (int32 i = group->m_firstIndex; i < group->m_lastIndex; i++)
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
				group->m_firstIndex = firstIndex;
				group->m_lastIndex = lastIndex;
				if (modified)
				{
					if (group->m_groupFlags & b2_solidParticleGroup)
					{
						SetGroupFlags(group,
							group->m_groupFlags |
							b2_particleGroupNeedsUpdateDepth);
					}
				}
			}
			else
			{
				group->m_firstIndex = 0;
				group->m_lastIndex = 0;
				if (!(group->m_groupFlags & b2_particleGroupCanBeEmpty))
				{
					SetGroupFlags(group,
						group->m_groupFlags | b2_particleGroupWillBeDestroyed);
				}
			}
		}
	}

	// update particle count
	m_count = newCount;
	m_allParticleFlags = allParticleFlags;
	m_needsUpdateAllParticleFlags = false;

	// destroy bodies with no particles
	for (int32 k = 0; k < m_groupCount; k++)
	{
		if (m_groupBuffer[k])
		{
			if (m_groupBuffer[k]->m_groupFlags & b2_particleGroupWillBeDestroyed)
				DestroyParticleGroup(k);
		}
	}
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
	rotate(m_heightLayerBuffer.begin() + start, m_heightLayerBuffer.begin() + mid,
			m_heightLayerBuffer.begin() + end);
	if (!m_lastBodyContactStepBuffer.empty())
	{
		rotate(m_lastBodyContactStepBuffer.begin() + start,
					m_lastBodyContactStepBuffer.begin() + mid,
					m_lastBodyContactStepBuffer.begin() + end);
	}
	if (!m_bodyContactCountBuffer.empty())
	{
		rotate(m_bodyContactCountBuffer.begin() + start,
					m_bodyContactCountBuffer.begin() + mid,
					m_bodyContactCountBuffer.begin() + end);
	}
	if (!m_consecutiveContactStepsBuffer.empty())
	{
		rotate(m_consecutiveContactStepsBuffer.begin() + start,
					m_consecutiveContactStepsBuffer.begin() + mid,
					m_consecutiveContactStepsBuffer.begin() + end);
	}
	rotate(m_positionBuffer.begin() + start, m_positionBuffer.begin() + mid,
		m_positionBuffer.begin() + end);
	rotate(m_velocityBuffer.begin() + start, m_velocityBuffer.begin() + mid,
		m_velocityBuffer.begin() + end);
	rotate(m_partGroupIdxBuffer.begin() + start, m_partGroupIdxBuffer.begin() + mid,
		m_partGroupIdxBuffer.begin() + end);
	rotate(m_partMatIdxBuffer.begin() + start, m_partMatIdxBuffer.begin() + mid,
		m_partMatIdxBuffer.begin() + end);
	if (m_hasForce)
	{
		rotate(m_forceBuffer.begin() + start, m_forceBuffer.begin() + mid,
			m_forceBuffer.begin() + end);
	}
	if (!m_staticPressureBuf.empty())
	{
		rotate(m_staticPressureBuf.begin() + start,
			m_staticPressureBuf.begin() + mid,
			m_staticPressureBuf.begin() + end);
	}
	if (m_hasDepth)
	{
		rotate(m_depthBuffer.begin() + start, m_depthBuffer.begin() + mid,
					m_depthBuffer.begin() + end);
	}
	if (m_colorBuffer.data())
	{
		rotate(m_colorBuffer.begin() + start,
					m_colorBuffer.begin() + mid, m_colorBuffer.begin() + end);
	}
	if (m_userDataBuffer.data())
	{
		rotate(m_userDataBuffer.begin() + start,
					m_userDataBuffer.begin() + mid, m_userDataBuffer.begin() + end);
	}
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
		b2PartBodyContact& contact = m_bodyContactBuf[k];
		contact.idx = newIndices[contact.idx];
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
		b2ParticleGroup* group = m_groupBuffer[k];
		if (group)
		{
			group->m_firstIndex = newIndices[group->m_firstIndex];
			group->m_lastIndex = newIndices[group->m_lastIndex - 1] + 1;
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
		m_idxByExpireTimeBuf.resize(m_particleBufferSize);
	}
	return m_idxByExpireTimeBuf;
}

void b2ParticleSystem::SetDestructionByAge(const bool enable)
{
	if (enable)
	{
		m_expireTimeBuf.resize(m_particleBufferSize);
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
		m_world->m_blockAllocator.Free(
			buffer->data(), sizeof(T) * m_particleBufferSize);
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
}

void b2ParticleSystem::SetVelocityBuffer(b2Vec3* buffer, int32 capacity)
{
	m_velocityBuffer.assign(buffer, buffer + capacity);
}

void b2ParticleSystem::SetColorBuffer(int32* buffer,
											  int32 capacity)
{
	m_colorBuffer.assign(buffer, buffer + capacity);
	//SetUserOverridableBuffer(&m_colorBuffer, buffer, capacity);
}

void b2ParticleSystem::SetUserDataBuffer(int32* buffer, int32 capacity)
{
	m_userDataBuffer.assign(buffer, buffer + capacity);
	//SetUserOverridableBuffer(&m_userDataBuffer, buffer, capacity);
}

void b2ParticleSystem::SetParticleFlags(int32 index, uint32 newFlags)
{
	uint32* oldFlags = &m_flagsBuffer[index];
	if (*oldFlags & ~newFlags)
	{
		// If any flags might be removed
		m_needsUpdateAllParticleFlags = true;
	}
	if (~m_allParticleFlags & newFlags)
	{
		// If any flags were added
		if (newFlags & b2_tensileParticle)
		{
			RequestBuffer(m_accumulation2Buf, hasAccumulation2Buf);

		}
		if (newFlags & b2_colorMixingParticle)
		{
			RequestBuffer(m_colorBuffer, hasColorBuf);
		}
		m_allParticleFlags |= newFlags;
	}
	*oldFlags = newFlags;
}

void b2ParticleSystem::AddParticleFlags(int32 index, uint32 newFlags)
{
	if (~m_allParticleFlags & newFlags)
	{
		// If any flags were added
		m_needsUpdateAllParticleFlags = true;
		if (newFlags & b2_tensileParticle)
		{
			RequestBuffer(m_accumulation2Buf, hasAccumulation2Buf);
		}
		if (newFlags & b2_colorMixingParticle)
		{
			RequestBuffer(m_colorBuffer, hasColorBuf);
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
	b2ParticleGroup* group, uint32 newFlags)
{
	uint32& oldFlags = group->m_groupFlags;
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

static inline bool IsSignificantForce(b2Vec3 force)
{
	return force.x != 0 || force.y != 0 || force.z != 0;
}

inline bool b2ParticleSystem::ForceCanBeApplied(uint32 flags) const
{
	return !(flags & b2_wallParticle);
}

inline void b2ParticleSystem::PrepareForceBuffer()
{
	if (!m_hasForce)
	{
		memset(m_forceBuffer.data(), 0, sizeof(*(m_forceBuffer.data())) * m_count);
		m_hasForce = true;
	}
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
	b2Vec3 distributedForce = force / (float32)(lastIndex - firstIndex);
	if (IsSignificantForce(distributedForce))
	{
		PrepareForceBuffer();

		// Distribute the force over all the particles.
		for (int32 i = firstIndex; i < lastIndex; i++)
		{
			m_forceBuffer[i] += distributedForce;
		}
	}
}
void b2ParticleSystem::ApplyForceInDirIfHasFlag(const b2Vec3& pos, float32 strength, uint32 flag)
{
	PrepareForceBuffer();

	for (int32 k = 0; k < m_count; k++)
	{
		if (m_flagsBuffer[k] & flag)
		{
			b2Vec3 f = (pos - m_positionBuffer[k]);
			f /= f.Length();	// Normalize
			m_forceBuffer[k] += f * strength;
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

void b2ParticleSystem::RayCast(b2RayCastCallback* callback,
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
			float32 f = callback->ReportParticle(this, i, point1 + t * v, n, t);
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
