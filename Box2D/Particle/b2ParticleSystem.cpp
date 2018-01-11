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
#include <Box2D/Particle/b2ParticleGroup.h>
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

	bool operator()(int32 contactIdx)
	{
		// This implements the selection criteria described in
		// RemoveSpuriousBodyContacts().
		// This functor is iterating through a list of Body contacts per
		// Particle, ordered from near to far.  For up to the maximum number of
		// contacts we allow per point per step, we verify that the contact
		// normal of the Body that genenerated the contact makes physical sense
		// by projecting a point back along that normal and seeing if it
		// intersects the fixture generating the contact.
		
		int32 particleIdx = m_system->m_bodyContactIdxBuffer[contactIdx];

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
		b2Vec2 n = b2Vec2(m_system->m_bodyContactNormalXBuffer[contactIdx],
						  m_system->m_bodyContactNormalYBuffer[contactIdx]);
		// weight is 1-(inv(diameter) * distance)
		n *= m_system->m_particleDiameter * (1 - m_system->m_bodyContactWeightBuffer[contactIdx]);
		b2Vec2 pos = b2Vec2(m_system->m_positionXBuffer[particleIdx], m_system->m_positionYBuffer[particleIdx]) + n;

		// pos is now a point projected back along the contact normal to the
		// contact distance. If the surface makes sense for a contact, pos will
		// now lie on or in the fixture generating
		b2Fixture* fixture = m_system->m_fixtureBuffer[m_system->m_bodyContactFixtureIdxBuffer[contactIdx]];
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
typedef LightweightPair<int32,int32> FixtureParticle;

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
	void Initialize(const vector<int32> bodyContactIdxs, const vector<int32> bodyContactFixturesIdxs,
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
static inline af::array afComputeTag(const af::array& x, const af::array& y)
{
	return ((y + yOffset).as(af::dtype::u32) << yShift) + (xScale * x + xOffset).as(af::dtype::u32);
}

static inline uint32 computeRelativeTag(uint32 tag, int32 x, int32 y)
{
	return tag + (y << yShift) + (x << xShift);
}
static inline af::array afComputeRelativeTag(const af::array& tag, int32 x, int32 y)
{
	return tag.as(af::dtype::u32) + (y << yShift) + (x << xShift);
}
/*/
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
			return (m_first++)->index;
		}
		m_first++;
	}
	return b2_invalidParticleIndex;
}
/*/
b2ParticleSystem::InsideBoundsEnumerator::InsideBoundsEnumerator(
	const b2ParticleSystem* partSys, uint32 lower, uint32 upper, int32 firstPos, int32 lastPos)
{
	m_partSys = partSys;
	m_xLower = lower & xMask;
	m_xUpper = upper & xMask;
	m_yLower = lower & yMask;
	m_yUpper = upper & yMask;
	m_firstPos = firstPos;
	m_lastPos = lastPos;
	b2Assert(m_firstPos <= m_lastPos);
}

int32 b2ParticleSystem::InsideBoundsEnumerator::GetNext()
{
	const vector<uint32>& proxyTags = m_partSys->m_proxyTagBuffer;
	while (m_firstPos < m_lastPos)
	{
		uint32 xTag = proxyTags[m_firstPos] & xMask;
#if B2_ASSERT_ENABLED
		uint32 yTag = *m_firstTag & yMask;
		b2Assert(yTag >= m_yLower);
		b2Assert(yTag <= m_yUpper);
#endif
		if (xTag >= m_xLower && xTag <= m_xUpper)
		{
			m_firstPos++;
			return m_partSys->m_proxyIdxBuffer[m_firstPos];
		}
		m_firstPos++;
	}
	return b2_invalidParticleIndex;
}


//*/
b2ParticleSystem::b2ParticleSystem(const b2ParticleSystemDef* def,
								   b2World* world, 
								   vector<b2Body*>& bodyBuffer,
								   vector<b2Fixture*>& fixtureBuffer) :
	m_handleAllocator(b2_minParticleBufferCapacity),
	m_bodyBuffer(bodyBuffer),
	m_fixtureBuffer(fixtureBuffer)

{

	AllocConsole();
	FILE* fp;
	freopen_s(&fp, "CONOUT$", "w", stdout);

	b2Assert(def);
	m_paused = false;
	m_timestamp = 0;
	m_allParticleFlags = 0;
	m_needsUpdateAllParticleFlags = false;
	m_allGroupFlags = 0;
	m_needsUpdateAllGroupFlags = false;
	m_hasForce = false;
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
	//m_forceBuffer = NULL;
	//m_weightBuffer = NULL;
	//m_staticPressureBuffer = NULL;
	//m_accumulationBuffer = NULL;
	//m_accumulation2Buffer = NULL;
	//m_depthBuffer = NULL;
	//m_groupBuffer = NULL;
	//m_materialBuffer = NULL;

	afPartBufs = {
	afPosXBuf						= af::array(0, af::dtype::f32),
	afPosYBuf						= af::array(0, af::dtype::f32),
	afPosZBuf						= af::array(0, af::dtype::f32),
	afFlagBuf						= af::array(0, af::dtype::u32),
	afColLayBuf						= af::array(0, af::dtype::u32),	
	afVelXBuf						= af::array(0, af::dtype::f32),
	afVelYBuf						= af::array(0, af::dtype::f32),
	afForceXBuf						= af::array(0, af::dtype::f32),
	afForceYBuf						= af::array(0, af::dtype::f32),
	afWeightBuf						= af::array(0, af::dtype::f32),
	afHeatBuf						= af::array(0, af::dtype::f32),
	afHealthBuf						= af::array(0, af::dtype::f32),
	afColorBuf						= af::array(0, af::dtype::s32),
	afUserDataBuf					= af::array(0, af::dtype::s32),
	afGroupIdxBuf					= af::array(0, af::dtype::s32),
	afPartMatIdxBuf					= af::array(0, af::dtype::f32)};

	afPartExtraBufs = {
	afAccumulationBuf				= af::array(0, af::dtype::f32),
	afAccumulation2XBuf				= af::array(0, af::dtype::f32),
	afAccumulation2YBuf				= af::array(0, af::dtype::f32),
	afStaticPressureBuf				= af::array(0, af::dtype::f32),
	afDepthBuf						= af::array(0, af::dtype::f32),
																  
	afLastBodyContactStepBuf		= af::array(0, af::dtype::s32),
	afBodyContactCountBuf			= af::array(0, af::dtype::s32),
	afConsecutiveContactStepsBuf	= af::array(0, af::dtype::s32),
	afStuckParticleBuf				= af::array(0, af::dtype::s32)};

	afProxyBufs = {
	afProxyIdxBuf					= af::array(0, af::dtype::s32),
	afProxyTagBuf					= af::array(0, af::dtype::u32)};

	afGroupBufs = {
	afGroupFirstIdxBuf				= af::array(0, af::dtype::s32),
	afGroupLastIdxBuf				= af::array(0, af::dtype::s32),
	afGroupFlagsBuf					= af::array(0, af::dtype::u32),
	afGroupColGroupBuf				= af::array(0, af::dtype::u32),
	afGroupStrengthBuf				= af::array(0, af::dtype::f32),
	afGroupMatIdxBuf				= af::array(0, af::dtype::s32),
	afGroupTimestampBuf				= af::array(0, af::dtype::f32),
	afGroupMassBuf					= af::array(0, af::dtype::f32),
	afGroupInertiaBuf				= af::array(0, af::dtype::f32),
	afGroupCenterXBuf				= af::array(0, af::dtype::f32),
	afGroupCenterYBuf				= af::array(0, af::dtype::f32),
	afGroupLinVelXBuf				= af::array(0, af::dtype::f32),
	afGroupLinVelYBuf				= af::array(0, af::dtype::f32),
	afGroupAngVelBuf				= af::array(0, af::dtype::f32),
	afGroupTransformBuf				= af::array(0, af::dtype::f32),
	afGroupUserDataBuf				= af::array(0, af::dtype::s32)};

	afContactBufs = {
	afContactIdxABuf				= af::array(0, af::dtype::s32),
	afContactIdxBBuf				= af::array(0, af::dtype::s32),
	afContactWeightBuf				= af::array(0, af::dtype::f32),
	afContactMassBuf				= af::array(0, af::dtype::f32),
	afContactNormalXBuf				= af::array(0, af::dtype::f32),
	afContactNormalYBuf				= af::array(0, af::dtype::f32),
	afContactFlagsBuf				= af::array(0, af::dtype::u32),
	afContactMatFlagsBuf			= af::array(0, af::dtype::u32)};

	afBodyBufs = {
	afBodyContactIdxBuf				= af::array(0, af::dtype::s32),
	afBodyContactBodyIdxBuf			= af::array(0, af::dtype::s32),
	afBodyContactFixtIdxBuf			= af::array(0, af::dtype::s32),
	afBodyContactWeightBuf			= af::array(0, af::dtype::f32),
	afBodyContactNormalXBuf			= af::array(0, af::dtype::f32),
	afBodyContactNormalYBuf			= af::array(0, af::dtype::f32),
	afBodyContactMassBuf			= af::array(0, af::dtype::f32)};

	afPartMatBufs = {
	afPartMatFlagsBuf				= af::array(0, af::dtype::u32),
	afPartMatMassBuf				= af::array(0, af::dtype::f32),
	afPartMatInvMassBuf				= af::array(0, af::dtype::f32),
	afPartMatStabilityBuf			= af::array(0, af::dtype::f32),
	afPartMatInvStabilityBuf		= af::array(0, af::dtype::f32),
	afPartMatExtgshPntBuf			= af::array(0, af::dtype::f32),
	afPartMatMeltingPntBuf			= af::array(0, af::dtype::f32),
	afPartMatBoilingPntBuf			= af::array(0, af::dtype::f32),
	afPartMatIgnitPntBuf			= af::array(0, af::dtype::f32),
	afPartMatHeatCondBuf			= af::array(0, af::dtype::f32)};

	afPairBufs = {
	afPairIdxABuf					= af::array(0, af::dtype::s32),
	afPairIdxBBuf					= af::array(0, af::dtype::s32),
	afPairFlagsBuf					= af::array(0, af::dtype::u32),
	afPairStrengthBuf				= af::array(0, af::dtype::u32),
	afPairDistanceBuf				= af::array(0, af::dtype::u32)};

	afTriadBufs = {
	afTriadIdxABuf					= af::array(0, af::dtype::s32),
	afTriadIdxBBuf					= af::array(0, af::dtype::s32),
	afTriadIdxCBuf					= af::array(0, af::dtype::s32),
	afTriadFlagsBuf					= af::array(0, af::dtype::u32),
	afTriadStrengthBuf				= af::array(0, af::dtype::f32),
	afTriadPAXBuf					= af::array(0, af::dtype::f32),
	afTriadPAYBuf					= af::array(0, af::dtype::f32),
	afTriadPBXBuf					= af::array(0, af::dtype::f32),
	afTriadPBYBuf					= af::array(0, af::dtype::f32),
	afTriadPCXBuf					= af::array(0, af::dtype::f32),
	afTriadPCYBuf					= af::array(0, af::dtype::f32),
	afTriadKABuf					= af::array(0, af::dtype::f32),
	afTriadKBBuf					= af::array(0, af::dtype::f32),
	afTriadKCBuf					= af::array(0, af::dtype::f32),
	afTriadSBuf						= af::array(0, af::dtype::f32)};
	
	afHasColorBuf					= false;
	afHasUserDataBuf				= false;
	afHasHandleIdxBuf				= false;
	afHasStaticPresBuf				= false;
	afHasExpireTimeBuf				= false;
	afHasIdxByExpireTimeBuf			= false;
	afHasAccumulation2XBuf			= false;
	afHasAccumulation2YBuf			= false;
	afHasDepthBuf					= false;
	afHasLastBodyContactBuf			= false;
	afHasBodyContactCountBuf		= false;
	afHasConsecutiveContactStepsBuf	= false;


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
void b2ParticleSystem::AFRequestBuffer(af::array& buf, bool& hasBuf)
{
	if (!hasBuf)
	{
		buf = af::constant(0, m_count);
		hasBuf = true;
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
	m_partMatMassBuf.resize(size);
	m_partMatInvMassBuf.resize(size);
	m_partMatStabilityBuf.resize(size);
	m_partMatInvStabilityBuf.resize(size);
	m_partMatExtinguishingPointBuf.resize(size);
	m_partMatMeltingPointBuf.resize(size);
	m_partMatBoilingPointBuf.resize(size);
	m_partMatIgnitionPointBuf.resize(size);
	m_partMatHeatConductivityBuf.resize(size);
	m_partMatBufferSize = size;
}

void b2ParticleSystem::ResizeParticleBuffers(int32 size)
{
	if (!size) size = b2_minParticleBufferCapacity;
	ReallocateHandleBuffers(size);
	m_flagsBuffer.resize(size);
	m_collisionLayerBuffer.resize(size);

	// Conditionally defer these as they are optional if the feature is
	// not enabled.
	const bool stuck = m_stuckThreshold > 0;
	m_lastBodyContactStepBuffer.resize(size);
	m_bodyContactCountBuffer.resize(size);
	m_consecutiveContactStepsBuffer.resize(size);
	m_positionXBuffer.resize(size);
	m_positionYBuffer.resize(size);
	m_positionZBuffer.resize(size);
	m_velocityXBuffer.resize(size);
	m_velocityYBuffer.resize(size);
	m_forceXBuffer.resize(size);
	m_forceYBuffer.resize(size);
	m_weightBuffer.resize(size);
	m_staticPressureBuf.resize(size);
	m_accumulationBuf.resize(size);
	m_accumulation2Buf.resize(size);
	m_depthBuffer.resize(size);
	m_colorBuffer.resize(size);
	m_groupIdxBuffer.resize(size);

	m_partMatIdxBuffer.resize(size);

	m_userDataBuffer.resize(size);
	m_heatBuffer.resize(size);
	m_healthBuffer.resize(size);

	m_proxyIdxBuffer.resize(size);
	m_proxyTagBuffer.resize(size);

	m_findContactCountBuf.resize(size);
	m_findContactIdxABuf.resize(size);
	m_findContactIdxBBuf.resize(size);
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
	m_groupFirstIdxBuf.resize(size);
	m_groupLastIdxBuf.resize(size);
	m_groupFlagsBuf.resize(size);
	m_groupColGroupBuf.resize(size);
	m_groupStrengthBuf.resize(size);
	m_groupMatIdxBuf.resize(size);
	m_groupTimestampBuf.resize(size);
	m_groupMassBuf.resize(size);
	m_groupInertiaBuf.resize(size);
	m_groupCenterXBuf.resize(size);
	m_groupCenterYBuf.resize(size);
	m_groupLinVelXBuf.resize(size);
	m_groupLinVelYBuf.resize(size);
	m_groupAngVelBuf.resize(size);
	m_groupTransformBuf.resize(size);
	m_groupUserDataBuf.resize(size);
	m_groupBufferSize = size;
}

void b2ParticleSystem::ResizeContactBuffers(int32 size)
{
	if (!size) size = b2_minParticleBufferCapacity;
	m_contactIdxABuffer.resize(size);
	m_contactIdxBBuffer.resize(size);
	m_contactWeightBuffer.resize(size);
	m_contactMassBuffer.resize(size);
	m_contactNormalXBuffer.resize(size);
	m_contactNormalYBuffer.resize(size);
	m_contactFlagsBuffer.resize(size);
	m_contactMatFlagsBuffer.resize(size);
	m_contactBufferSize = size;
}
void b2ParticleSystem::ResizeBodyContactBuffers(int32 size)
{
	if (!size) size = b2_minParticleBufferCapacity;
	m_bodyContactIdxBuffer.resize(size);
	m_bodyContactBodyIdxBuffer.resize(size);
	m_bodyContactFixtureIdxBuffer.resize(size);
	m_bodyContactWeightBuffer.resize(size);
	m_bodyContactNormalXBuffer.resize(size);
	m_bodyContactNormalYBuffer.resize(size);
	m_bodyContactMassBuffer.resize(size);
	m_bodyContactBufferSize = size;
}

void b2ParticleSystem::ResizePairBuffers(int32 size)
{
	if (!size) size = b2_minParticleBufferCapacity;
	m_pairIdxABuf.resize(size);
	m_pairIdxBBuf.resize(size);
	m_pairFlagsBuf.resize(size);
	m_pairStrengthBuf.resize(size);
	m_pairDistanceBuf.resize(size);
	m_pairBufferSize = size;
}
void b2ParticleSystem::ResizeTriadBuffers(int32 size)
{
	if (!size) size = b2_minParticleBufferCapacity;
	m_triadIdxABuf.resize(size);
	m_triadIdxBBuf.resize(size);
	m_triadIdxCBuf.resize(size);
	m_triadFlagsBuf.resize(size);
	m_triadStrengthBuf.resize(size);
	m_triadPAXBuf.resize(size);
	m_triadPAYBuf.resize(size);
	m_triadPBXBuf.resize(size);
	m_triadPBYBuf.resize(size);
	m_triadPCXBuf.resize(size);
	m_triadPCYBuf.resize(size);
	m_triadKABuf.resize(size);
	m_triadKBBuf.resize(size);
	m_triadKCBuf.resize(size);
	m_triadSBuf.resize(size);
	m_triadBufferSize = size;
}

int32 b2ParticleSystem::CreateParticleMaterial(const b2ParticleMaterialDef & def)
{
	if (m_partMatCount >= m_partMatBufferSize)
		ResizePartMatBuffers(m_partMatCount * 2);
	int32 idx = m_partMatCount;
	m_partMatCount++;
	m_partMatFlagsBuf[idx] = def.matFlags;
	m_partMatMassBuf[idx] = def.mass;
	m_partMatInvMassBuf[idx] = 1 / def.mass;
	m_partMatStabilityBuf[idx] = def.stability;
	m_partMatInvStabilityBuf[idx] = 1 / def.stability;
	m_partMatExtinguishingPointBuf[idx] = def.extinguishingPoint;
	m_partMatMeltingPointBuf[idx] = def.meltingPoint;
	m_partMatBoilingPointBuf[idx] = def.boilingPoint;
	m_partMatIgnitionPointBuf[idx] = def.ignitionPoint;
	m_partMatHeatConductivityBuf[idx] = def.heatConductivity;
	CopyMatsToGPU();
	return idx;
}

int32 b2ParticleSystem::CreateParticle(const b2ParticleDef& def)
{
	b2Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked())
	{
		return 0;
	}

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
			return b2_invalidParticleIndex;
		}
	}
	int32 index = m_count++;

	m_flagsBuffer[index] = 0;
	if (!m_lastBodyContactStepBuffer.empty())
	{
		m_lastBodyContactStepBuffer[index] = 0;
	}
	if (!m_bodyContactCountBuffer.empty())
	{
		m_bodyContactCountBuffer[index] = 0;
	}
	if (!m_consecutiveContactStepsBuffer.empty())
	{
		m_consecutiveContactStepsBuffer[index] = 0;
	}
	m_positionXBuffer[index] = def.positionX;
	m_positionYBuffer[index] = def.positionY;
	m_positionZBuffer[index] = def.positionZ;
	m_collisionLayerBuffer[index] = GetLayerMaskFromZ(def.positionZ);
	m_velocityXBuffer[index] = def.velocityX;
	m_velocityYBuffer[index] = def.velocityY;
	m_weightBuffer[index] = 0;
	if (index >= m_heatBuffer.size())
	m_heatBuffer[index] = def.heat;
	m_healthBuffer[index] = def.health;
	m_forceXBuffer[index] = 0;
	m_forceYBuffer[index] = 0;
	m_partMatIdxBuffer[index] = def.matIdx;
	if (!m_staticPressureBuf.empty())
	{
		m_staticPressureBuf[index] = 0;
	}
	m_depthBuffer[index] = 0;
	m_colorBuffer[index] = def.color;
	m_userDataBuffer[index] = def.userData;
	if (!m_handleIndexBuffer.empty())
	{
		m_handleIndexBuffer[index] = NULL;
	}
	m_proxyIdxBuffer[index] = index;
	m_proxyTagBuffer[index] = 0;

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

	int32 groupIdx = def.groupIdx;
	m_groupIdxBuffer[index] = groupIdx;
	if (groupIdx >= 0)
	{
		int32& firstIdx = m_groupFirstIdxBuf[groupIdx];
		int32& lastIdx = m_groupLastIdxBuf[groupIdx];
		if (firstIdx < lastIdx)
		{
			// Move particles in the group just before the new particle.
			RotateBuffer(firstIdx, lastIdx, index);
			b2Assert(lastIdx == index);
			// Update the index range of the group to contain the new particle.
			lastIdx = index + 1;
		}
		else
		{
			// If the group is empty, reset the index range to contain only the
			// new particle.
			firstIdx = index;
			lastIdx = index + 1;
		}
	}
	SetParticleFlags(index, def.flags);
	return index;
}
int32 b2ParticleSystem::AFCreateParticle(const afParticleDef& def)
{
	b2Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked())
		return 0;

	const int32 addCount = def.positionX.elements();

	if (addCount <= 0)
		return 0;

	int32 newCount = m_count + addCount;
	const af::array& newIdxs = (af::array)af::seq(addCount) + m_count;
	const af::array& newEmpty = af::constant(0, addCount);

	if (!afLastBodyContactStepBuf.isempty())
		AFJoin(afLastBodyContactStepBuf, newEmpty, af::dtype::s32);

	if (!afBodyContactCountBuf.isempty())
		AFJoin(afBodyContactCountBuf, newEmpty, af::dtype::u32);

	if (!afConsecutiveContactStepsBuf.isempty())
		AFJoin(afConsecutiveContactStepsBuf, newEmpty, af::dtype::u32);

	AFJoin(afPosXBuf,		def.positionX);
	AFJoin(afPosYBuf,		def.positionY);
	AFJoin(afPosZBuf,		def.positionZ);
	AFJoin(afFlagBuf,		newEmpty, af::dtype::u32);
	AFJoin(afColLayBuf,		AFGetLayerMaskFromZ(def.positionZ), af::dtype::u32);
	AFJoin(afVelXBuf,		def.velocityX);
	AFJoin(afVelYBuf,		def.velocityY);
	AFJoin(afWeightBuf,		newEmpty);
	AFJoin(afHeatBuf,		af::constant(def.heat, addCount));
	AFJoin(afHealthBuf,		af::constant(def.health, addCount));
	AFJoin(afColorBuf,		def.color, af::dtype::s32);
	AFJoin(afUserDataBuf,	af::constant(def.userData, addCount), af::dtype::s32);
	AFJoin(afForceXBuf,		newEmpty);
	AFJoin(afForceYBuf,		newEmpty);
	AFJoin(afPartMatIdxBuf, af::constant(def.matIdx, addCount), af::dtype::s32);
	if (!afStaticPressureBuf.isempty())
		AFJoin(afStaticPressureBuf, newEmpty);
	AFJoin(afDepthBuf,		newEmpty);
	if (afHasHandleIdxBuf)
		AFJoin(afHandleIdxBuf, af::constant(b2_invalidParticleIndex, addCount), af::dtype::s32);
	AFJoin(afProxyIdxBuf, newIdxs, af::dtype::s32);
	AFJoin(afProxyTagBuf, newEmpty, af::dtype::u32);

	// If particle lifetimes are enabled or the lifetime is set in the particle
	// definition, initialize the lifetime.
	/*const bool finiteLifetime = def.lifetime > 0;	//TODO
	if (!m_expireTimeBuf.empty() || finiteLifetime)
	{
		SetParticleLifetime(index, finiteLifetime ? def.lifetime :
			ExpirationTimeToLifetime(
				-GetQuantizedTimeElapsed()));
		// Add a reference to the newly added particle to the end of the
		// queue.
		m_idxByExpireTimeBuf[index] = index;
	}*/

	int32 groupIdx = def.groupIdx;
	AFJoin(afGroupIdxBuf, af::constant(groupIdx, addCount), af::dtype::s32);
	if (groupIdx != b2_invalidGroupIndex)
	{
		int32& firstIdx = m_groupFirstIdxBuf[groupIdx];
		int32& lastIdx = m_groupLastIdxBuf[groupIdx];
		if (firstIdx < lastIdx)
		{
			// Move particles in the group just before the new particle.
			AFRotateBuffer(firstIdx, lastIdx, m_count);
			b2Assert(lastIdx == index);
			// Update the index range of the group to contain the new particles.
			lastIdx = newCount;
		}
		else
		{
			// If the group is empty, reset the index range to contain only the
			// new particles.
			firstIdx = m_count;
			lastIdx = newCount;
		}
	}
	AFSetParticleFlags(newIdxs, def.flags);
	m_count = newCount;


	return newCount;
}


const uint32 b2ParticleSystem::GetLayerMaskFromZ(float32 z)
{
	int32 layer = z * m_invPointsPerLayer + m_layerGraphB;
	return (1 << layer);
}
const af::array b2ParticleSystem::AFGetLayerMaskFromZ(const af::array& z)
{
	af::array layer = (z * m_invPointsPerLayer + m_layerGraphB).as(af::dtype::u32);
	return 1 << layer;
}

/// Retrieve a handle to the particle at the specified index.
const b2ParticleHandle* b2ParticleSystem::GetParticleHandleFromIndex(
	const int32 index)
{
	b2Assert(index >= 0 && index < GetParticleCount() &&
			 index != b2_invalidParticleIndex);
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
void b2ParticleSystem::AFDestroyParticles(af::array idxs, bool callDestructionListener)
{
	uint32 flags = b2_zombieParticle;
	if (callDestructionListener)
	{
		flags |= b2_destructionListenerParticle;
	}
	afFlagBuf(idxs) = afFlagBuf(idxs) | flags;
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
			if (m_shape->TestPoint(m_xf,
				b2Vec2(m_system->m_positionXBuffer[index], m_system->m_positionYBuffer[index])))
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
	shape.ComputeAABB(&aabb, xf, 0);
	m_world->QueryAABB(&callback, aabb);
	return callback.Destroyed();
}

int32 b2ParticleSystem::CreateParticleForGroup(
	const b2ParticleGroupDef& groupDef, const b2Transform& xf, const b2Vec2& p)
{
	b2ParticleDef particleDef;
	particleDef.flags = groupDef.flags;
	b2Vec2 pos = b2Mul(xf, p);
	particleDef.positionX = pos.x;
	particleDef.positionY = pos.y;
	particleDef.positionZ = m_lowestPoint + m_pointsPerLayer * (float32)groupDef.layer;
	b2Vec2 vel =
		groupDef.linearVelocity +
		b2Cross(groupDef.angularVelocity,
			pos - groupDef.position);
	particleDef.velocityX = vel.x;
	particleDef.velocityY = vel.y;

	particleDef.color = groupDef.color;
	particleDef.lifetime = groupDef.lifetime;
	particleDef.userData = groupDef.userData;
	particleDef.health = groupDef.health;
	particleDef.heat = groupDef.heat;
	particleDef.matIdx = groupDef.matIdx;
	return CreateParticle(particleDef);
}
int32 b2ParticleSystem::AFCreateParticleForGroup(
	const b2ParticleGroupDef& groupDef, const b2Transform& xf, const af::array& px, const af::array& py)
{
	//TODO join with create PG
	afParticleDef particleDef;
	particleDef.flags     = groupDef.flags;
	particleDef.positionX = b2MulX(xf, px, py);
	particleDef.positionY = b2MulY(xf, px, py);
	float32 z = m_lowestPoint + m_pointsPerLayer * (float32)groupDef.layer;
	particleDef.positionZ = af::constant(z, px.elements(), af::dtype::f32);
	particleDef.velocityX = groupDef.linearVelocity.x +
								b2CrossX(groupDef.angularVelocity,
									particleDef.positionX - groupDef.position.x);
	particleDef.velocityY = groupDef.linearVelocity.y +
								b2CrossY(groupDef.angularVelocity,
									particleDef.positionY - groupDef.position.y);
	particleDef.color	  = af::constant(groupDef.color, px.elements(), af::dtype::s32);
	particleDef.lifetime  = groupDef.lifetime;
	particleDef.userData  = groupDef.userData;
	particleDef.health	  = groupDef.health;
	particleDef.heat	  = groupDef.heat;
	particleDef.matIdx    = groupDef.matIdx;
	return AFCreateParticle(particleDef);
}

int32 b2ParticleSystem::AFCreateParticleForGroup(const b2ParticleGroupDef& groupDef,
												 const b2Transform& xf,
												 const af::array& posX, 
												 const af::array& posY, 
												 const af::array& posZ,
												 const af::array& color) {
	afParticleDef particleDef;
	particleDef.flags = groupDef.flags;
	particleDef.positionX = b2MulX(xf, posX, posY);
	particleDef.positionY = b2MulY(xf, posX, posY);
	particleDef.positionZ = posZ;
	particleDef.velocityX = groupDef.linearVelocity.x +
		b2CrossX(groupDef.angularVelocity,
			particleDef.positionX - groupDef.position.x);
	particleDef.velocityY = groupDef.linearVelocity.y +
		b2CrossY(groupDef.angularVelocity,
			particleDef.positionY - groupDef.position.y);
	particleDef.color = groupDef.colorData;
	particleDef.lifetime = groupDef.lifetime;
	particleDef.userData = groupDef.userData;
	particleDef.health = groupDef.health;
	particleDef.heat = groupDef.heat;
	particleDef.matIdx = groupDef.matIdx;
	return AFCreateParticle(particleDef);
}
int32 b2ParticleSystem::CreateParticleForGroup(
	const b2ParticleGroupDef& groupDef, const b2Transform& xf, const b2Vec2& p, const float32 z, int32 c)
{
	b2ParticleDef particleDef;
	particleDef.flags = groupDef.flags;
	b2Vec2 pos = b2Mul(xf, p);
	particleDef.positionX = pos.x;
	particleDef.positionY = pos.y;
	particleDef.positionZ = z;
	b2Vec2 vel =
		groupDef.linearVelocity +
		b2Cross(groupDef.angularVelocity,
			pos - groupDef.position);
	particleDef.velocityX = vel.x;
	particleDef.velocityY = vel.y;
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
			b2Vec2 p = edge.m_vertex1 + positionOnEdge / edgeLength * d;
			CreateParticleForGroup(groupDef, xf, p);
			positionOnEdge += stride;
		}
		positionOnEdge -= edgeLength;
	}
}
void b2ParticleSystem::AFCreateParticlesStrokeShapeForGroup(
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
			edge = *(b2EdgeShape*)shape;
		}
		else
		{
			b2Assert(shape->GetType() == b2Shape::e_chain);
			((b2ChainShape*)shape)->GetChildEdge(&edge, childIndex);
		}
		b2Vec2 d = edge.m_vertex2 - edge.m_vertex1;
		float32 edgeLength = d.Length();
		af::array positionsOnEdge = af::seq(positionOnEdge, edgeLength, stride);
		af::array px = edge.m_vertex1.x + positionsOnEdge / edgeLength * d.x;
		af::array py = edge.m_vertex1.y + positionsOnEdge / edgeLength * d.y;
		AFCreateParticleForGroup(groupDef, xf, px, py);
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
	shape->ComputeAABB(&aabb, identity, 0);
	for (float32 y = floorf(aabb.lowerBound.y / stride) * stride;
		y < aabb.upperBound.y; y += stride)
	{
		for (float32 x = floorf(aabb.lowerBound.x / stride) * stride;
			x < aabb.upperBound.x; x += stride)
		{
			b2Vec2 p(x, y);
			if (shape->TestPoint(identity, p))
			{
				CreateParticleForGroup(groupDef, xf, p);
			}
		}
	}
}
void b2ParticleSystem::AFCreateParticlesFillShapeForGroup(
	const b2Shape *shape,
	const b2ParticleGroupDef& groupDef, const b2Transform& xf)
{
	float32 stride = groupDef.stride;
	if (stride == 0)
		stride = GetParticleStride();
	
	b2Transform identity;
	identity.SetIdentity();
	b2AABB aabb;
	b2Assert(shape->GetChildCount() == 1);
	shape->ComputeAABB(&aabb, identity, 0);

	int32 yStart = floorf(aabb.lowerBound.y / stride) * stride;
	int32 yEnd   = aabb.upperBound.y;
	int32 xStart = floorf(aabb.lowerBound.x / stride) * stride;
	int32 xEnd   = aabb.upperBound.x;
	af::array y = af::seq(yStart, yEnd, stride);
	af::array x = af::seq(xStart, xEnd, stride);
	int32 ySize = (yEnd - yStart) / stride;
	int32 xSize = (xEnd - xStart) / stride;
	if (ySize <= 0 || xSize <= 0)
		return;
	x = af::tile(x, ySize);
	y = af::tile(y, 1, xSize);
	y = af::transpose(y);
	y = af::flat(y);

	af::array condIdxs = af::where(shape->AFTestPoints(identity, x, y));
	if (!condIdxs.isempty())
		AFCreateParticleForGroup(groupDef, xf, x(condIdxs), y(condIdxs));
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
void b2ParticleSystem::AFCreateParticlesWithShapeForGroup(
	const b2Shape* shape,
	const b2ParticleGroupDef& groupDef, const b2Transform& xf)
{
	switch (shape->GetType()) {
	case b2Shape::e_edge:
	case b2Shape::e_chain:
		AFCreateParticlesStrokeShapeForGroup(shape, groupDef, xf);
		break;
	case b2Shape::e_polygon:
	case b2Shape::e_circle:
		AFCreateParticlesFillShapeForGroup(shape, groupDef, xf);
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
		bool TestPoint(const b2Transform& xf, const b2Vec2& p) const
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
		af::array AFTestPoints(const b2Transform& xf, const af::array& px, const af::array& py) const
		{
			af::array ret = af::constant(0, px.elements(), af::dtype::b8);
			for (int32 i = 0; i < m_shapeCount; ++i)
			{
				ret(af::where(m_shapes[i]->AFTestPoints(xf, px, py))) = 1;
			}
			return ret;
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
		void AFComputeDistance(const b2Transform& xf, const af::array& px,
			const af::array& py, af::array& distance, af::array& normalX, af::array& normalY,
			int32 childIndex) const
		{
			b2Assert(false);
			B2_NOT_USED(xf);
			B2_NOT_USED(px);
			B2_NOT_USED(py);
			B2_NOT_USED(distance);
			B2_NOT_USED(normalX);
			B2_NOT_USED(normalY);
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
		af::array AFRayCast(afRayCastOutput* output, const afRayCastInput& input,
			const b2Transform& transform, int32 childIndex) const
		{
			b2Assert(false);
			B2_NOT_USED(output);
			B2_NOT_USED(input);
			B2_NOT_USED(transform);
			B2_NOT_USED(childIndex);
			return af::array(false, input.p1x.elements(), af::dtype::b8);
		}
		void ComputeAABB(
				b2AABB* aabb, const b2Transform& xf, int32 childIndex) const
		{
			B2_NOT_USED(childIndex);
			aabb->lowerBound.x = +FLT_MAX;
			aabb->lowerBound.y = +FLT_MAX;
			aabb->upperBound.x = -FLT_MAX;
			aabb->upperBound.y = -FLT_MAX;
			b2Assert(childIndex == 0);
			for (int32 i = 0; i < m_shapeCount; i++)
			{
				int32 childCount = m_shapes[i]->GetChildCount();
				for (int32 j = 0; j < childCount; j++)
				{
					b2AABB subaabb;
					m_shapes[i]->ComputeAABB(&subaabb, xf, j);
					aabb->Combine(subaabb);
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
void b2ParticleSystem::AFCreateParticlesWithShapesForGroup(
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
		bool TestPoint(const b2Transform& xf, const b2Vec2& p) const
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
		af::array AFTestPoints(const b2Transform& xf, const af::array& px, const af::array& py) const
		{
			af::array ret = af::constant(0, px.elements(), af::dtype::b8);
			for (int32 i = 0; i < m_shapeCount; ++i)
			{
				ret(af::where(m_shapes[i]->AFTestPoints(xf, px, py))) = true;
			}
			return ret;
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
		void AFComputeDistance(const b2Transform& xf, const af::array& px,
			const af::array& py, af::array& distance, af::array& normalX, af::array& normalY,
			int32 childIndex) const
		{
			b2Assert(false);
			B2_NOT_USED(xf);
			B2_NOT_USED(px);
			B2_NOT_USED(py);
			B2_NOT_USED(distance);
			B2_NOT_USED(normalX);
			B2_NOT_USED(normalY);
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
		af::array AFRayCast(afRayCastOutput* output, const afRayCastInput& input,
			const b2Transform& transform, int32 childIndex) const
		{
			b2Assert(false);
			B2_NOT_USED(output);
			B2_NOT_USED(input);
			B2_NOT_USED(transform);
			B2_NOT_USED(childIndex);
			return af::array(false, input.p1x.elements(), af::dtype::b8);
		}
		void ComputeAABB(
			b2AABB* aabb, const b2Transform& xf, int32 childIndex) const
		{
			B2_NOT_USED(childIndex);
			aabb->lowerBound.x = +FLT_MAX;
			aabb->lowerBound.y = +FLT_MAX;
			aabb->upperBound.x = -FLT_MAX;
			aabb->upperBound.y = -FLT_MAX;
			b2Assert(childIndex == 0);
			for (int32 i = 0; i < m_shapeCount; i++)
			{
				int32 childCount = m_shapes[i]->GetChildCount();
				for (int32 j = 0; j < childCount; j++)
				{
					b2AABB subaabb;
					m_shapes[i]->ComputeAABB(&subaabb, xf, j);
					aabb->Combine(subaabb);
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
	AFCreateParticlesFillShapeForGroup(&compositeShape, groupDef, xf);
}

int32 b2ParticleSystem::CreateParticleGroup(
	const b2ParticleGroupDef& groupDef)
{
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
			CreateParticleForGroup(groupDef, transform, b2Vec2(groupDef.positionDataX[i], groupDef.positionDataY[i]), groupDef.positionDataZ[i], c);
		}
	}
	if (m_groupCount >= m_groupBufferSize)
		ResizeGroupBuffers(m_groupBufferSize * 2);

	int32 lastIndex = m_count;

	int32 groupIdx;
	if (!m_freeGroupIdxBuffer.empty())
	{
		groupIdx = m_freeGroupIdxBuffer.back();
		m_freeGroupIdxBuffer.pop_back();
	}
	else
	{
		groupIdx = m_groupCount;
		m_groupCount++;
	}
	m_groupFirstIdxBuf[groupIdx] = firstIndex;
	m_groupLastIdxBuf[groupIdx] = lastIndex;
	m_groupStrengthBuf[groupIdx] = groupDef.strength;
	m_groupUserDataBuf[groupIdx] = groupDef.userData;
	m_groupColGroupBuf[groupIdx] = groupDef.collisionGroup;
	m_groupMatIdxBuf[groupIdx] = groupDef.matIdx;
	m_groupTransformBuf[groupIdx] = transform;

	for (int32 i = firstIndex; i < lastIndex; i++)
	{
		m_groupIdxBuffer[i] = groupIdx;
	}
	SetGroupFlags(groupIdx, groupDef.groupFlags);


	// Create pairs and triads between particles in the group.
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
int32 b2ParticleSystem::AFCreateParticleGroup(
	const b2ParticleGroupDef& groupDef)
{
	b2Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked())
		return 0;

	b2Transform transform;
	transform.Set(groupDef.position, groupDef.angle);
	int32 firstIndex = m_count;
	if (groupDef.shape)
	{
		AFCreateParticlesWithShapeForGroup(groupDef.shape, groupDef, transform);
	}
	if (groupDef.shapes)
	{
		AFCreateParticlesWithShapesForGroup(
			groupDef.shapes, groupDef.shapeCount, groupDef, transform);
	}
	if (groupDef.particleCount)
	{
		b2Assert(groupDef.positionDataX);
		b2Assert(groupDef.colorData);
		af::array px = af::array(groupDef.particleCount, afArrayDims, groupDef.positionDataX);
		af::array py = af::array(groupDef.particleCount, afArrayDims, groupDef.positionDataY);
		af::array pz = af::array(groupDef.particleCount, afArrayDims, groupDef.positionDataZ);
		af::array c  = af::array(groupDef.particleCount, afArrayDims, groupDef.colorData);
		AFCreateParticleForGroup(groupDef, transform, px, py, pz, c);
	}
	if (m_groupCount >= m_groupBufferSize)
		ResizeGroupBuffers(m_groupBufferSize * 2);

	int32 lastIndex = m_count;

	int32 groupIdx;
	if (!m_freeGroupIdxBuffer.empty())
	{
		groupIdx = m_freeGroupIdxBuffer.back();
		m_freeGroupIdxBuffer.pop_back();
	}
	else
	{
		groupIdx = m_groupCount;
		m_groupCount++;
	}
	m_groupFirstIdxBuf[groupIdx] = firstIndex;
	m_groupLastIdxBuf[groupIdx] = lastIndex;
	m_groupStrengthBuf[groupIdx] = groupDef.strength;
	m_groupUserDataBuf[groupIdx] = groupDef.userData;
	m_groupColGroupBuf[groupIdx] = groupDef.collisionGroup;
	m_groupMatIdxBuf[groupIdx] = groupDef.matIdx;
	m_groupTransformBuf[groupIdx] = transform;

	// Set Particle Group
	int32 partCount = lastIndex - firstIndex;
	if (partCount)
	{
		afGroupIdxBuf((af::array)(af::seq(partCount)) + firstIndex) = groupIdx;
	}

	SetGroupFlags(groupIdx, groupDef.groupFlags);

	// Create pairs and triads between particles in the group.
	AFConnectionFilter filter;
	
	CopyGroupsToGPU();
	//AFUpdateContacts(true);
	//AFUpdatePairsAndTriads(firstIndex, lastIndex, filter);

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

	b2Assert(groupA != groupB);
	RotateBuffer(m_groupFirstIdxBuf[groupBIdx], m_groupLastIdxBuf[groupBIdx], m_count);
	b2Assert(m_groupLastIdxBuf[groupBIdx] == m_count);
	RotateBuffer(m_groupFirstIdxBuf[groupAIdx], m_groupLastIdxBuf[groupAIdx],
				 m_groupFirstIdxBuf[groupBIdx]);
	b2Assert(m_groupLastIdxBuf[groupAIdx] == m_groupFirstIdxBuf[groupBIdx]);

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
	} filter(m_groupFirstIdxBuf[groupBIdx]);
	UpdateContacts(true);
	UpdatePairsAndTriads(m_groupFirstIdxBuf[groupAIdx], m_groupLastIdxBuf[groupBIdx], filter);

	for (int32 i = m_groupFirstIdxBuf[groupBIdx]; i < m_groupLastIdxBuf[groupBIdx]; i++)
	{
		m_groupIdxBuffer[i] = groupAIdx;
	}
	uint32 groupFlags = m_groupFlagsBuf[groupAIdx] | m_groupFlagsBuf[groupBIdx];
	SetGroupFlags(groupAIdx, groupFlags);
	m_groupLastIdxBuf[groupAIdx] = m_groupLastIdxBuf[groupBIdx];
	m_groupFirstIdxBuf[groupBIdx] = m_groupLastIdxBuf[groupBIdx];
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

void b2ParticleSystem::UpdateGroupStatistics(int32 groupIdx)
{
	if (m_groupTimestampBuf[groupIdx] != m_timestamp)
	{
		int32 firstIdx = m_groupFirstIdxBuf[groupIdx];
		int32 lastIdx = m_groupLastIdxBuf[groupIdx];

		float32 m = m_partMatMassBuf[m_groupMatIdxBuf[groupIdx]];
		float32& mass = m_groupMassBuf[groupIdx];
		float32& centerX = m_groupCenterXBuf[groupIdx];
		float32& centerY = m_groupCenterYBuf[groupIdx];
		float32& linVelX = m_groupLinVelXBuf[groupIdx];
		float32& linVelY = m_groupLinVelYBuf[groupIdx];
		mass = 0;
		centerX = 0;
		centerY	= 0;
		linVelX	= 0;
		linVelY	= 0;
		for (int32 i = firstIdx; i < lastIdx; i++)
		{
			mass += m;
			centerX += m * m_positionXBuffer[i];
			centerY += m * m_positionYBuffer[i];
			linVelX += m * m_velocityXBuffer[i];
			linVelY += m * m_velocityXBuffer[i];
		}
		if (mass > 0)
		{
			centerX *= 1 / mass;
			centerY *= 1 / mass;
			linVelX *= 1 / mass;
			linVelY *= 1 / mass;
		}
		float32& inertia = m_groupInertiaBuf[groupIdx];
		float32& angVel = m_groupAngVelBuf[groupIdx];
		inertia = 0;
		angVel = 0;
		for (int32 i = firstIdx; i < lastIdx; i++)
		{
			float32 px = m_positionXBuffer[i] - centerX,
					py = m_positionYBuffer[i] - centerY,
					vx = m_velocityXBuffer[i] - linVelX,
					vy = m_velocityYBuffer[i] - linVelY;
			inertia += m * b2Dot(px, py, px, py);
			angVel += m * b2Cross(px, py, vx, vy);
		}
		if (inertia > 0)
			angVel *= 1 / inertia;

		m_groupTimestampBuf[groupIdx] = m_timestamp;
	}
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
		int32 a = m_contactIdxABuffer[k];
		int32 b = m_contactIdxBBuffer[k];
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
		int32 a = m_pairIdxABuf[k];
		int32 b = m_pairIdxBBuf[k];
		if (group->ContainsParticle(a))
		{
			m_pairIdxABuf[k] = nodeBuffer[a - bufferIndex].index;
		}
		if (group->ContainsParticle(b))
		{
			m_pairIdxBBuf[k] = nodeBuffer[b - bufferIndex].index;
		}
	}
	for (int32 k = 0; k < m_triadCount; k++)
	{
		int32 a = m_triadIdxABuf[k];
		int32 b = m_triadIdxBBuf[k];
		int32 c = m_triadIdxCBuf[k];
		if (group->ContainsParticle(a))
		{
			m_triadIdxABuf[k] = nodeBuffer[a - bufferIndex].index;
		}
		if (group->ContainsParticle(b))
		{
			m_triadIdxBBuf[k] = nodeBuffer[b - bufferIndex].index;
		}
		if (group->ContainsParticle(c))
		{
			m_triadIdxCBuf[k] = nodeBuffer[c - bufferIndex].index;
		}
	}
}

int32 b2ParticleSystem::CloneParticle(int32 oldIndex, int32 groupIdx)
{
	b2ParticleDef def;
	def.flags	  = m_flagsBuffer[oldIndex];
	def.positionX = m_positionXBuffer[oldIndex];
	def.positionY = m_positionYBuffer[oldIndex];
	def.positionZ = m_positionZBuffer[oldIndex];
	def.velocityX = m_velocityXBuffer[oldIndex];
	def.velocityY = m_velocityYBuffer[oldIndex];
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
		m_forceXBuffer[newIndex] = m_forceXBuffer[oldIndex];
		m_forceYBuffer[newIndex] = m_forceYBuffer[oldIndex];
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
void b2ParticleSystem::AFUpdatePairsAndTriadsWithReactiveParticles()
{
	class AFReactiveFilter : public AFConnectionFilter
	{
		af::array IsNecessary(af::array idxs) const
		{
			return (flagsBuf(idxs) & b2_reactiveParticle) != 0;
		}
		af::array flagsBuf;
	public:
		AFReactiveFilter(af::array flags)
		{
			flagsBuf = flags;
		}
	} filter(afFlagBuf);
	AFUpdatePairsAndTriads(0, m_count, filter);

	afFlagBuf = afFlagBuf & ~b2_reactiveParticle;
	m_allParticleFlags &= ~b2_reactiveParticle;
}

static bool ParticleCanBeConnected(
	uint32 flags, int32 groupIdx, int32 groupFlags)
{
	return
		(flags & (b2_wallParticle | b2_springParticle | b2_elasticParticle)) ||
		(groupIdx && groupFlags & b2_rigidParticleGroup);
}
static af::array AFParticleCanBeConnected(
	af::array flags, af::array groupIdx, af::array groupFlags)
{
	return
		(flags & (b2_wallParticle | b2_springParticle | b2_elasticParticle)) ||
		(groupIdx && groupFlags & b2_rigidParticleGroup);
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
			int32 a = m_contactIdxABuffer[k];
			int32 b = m_contactIdxBBuffer[k];
			uint32 af = m_flagsBuffer[a];
			uint32 bf = m_flagsBuffer[b];
			int32 groupAIdx = m_groupIdxBuffer[a];
			int32 groupBIdx = m_groupIdxBuffer[b];
			if (a >= firstIndex && a < lastIndex &&
				b >= firstIndex && b < lastIndex &&
				!((af | bf) & b2_zombieParticle) &&
				((af | bf) & k_pairFlags) &&
				(filter.IsNecessary(a) || filter.IsNecessary(b)) &&
				ParticleCanBeConnected(af, groupAIdx, m_groupFlagsBuf[groupAIdx]) &&
				ParticleCanBeConnected(bf, groupBIdx, m_groupFlagsBuf[groupBIdx]) &&
				filter.ShouldCreatePair(a, b))
			{
				if (m_pairCount >= m_pairBufferSize)
					ResizePairBuffers(m_pairCount * 2);
				int32 i = m_pairCount;
				m_pairCount++;
				m_pairIdxABuf[i] = a;
				m_pairIdxBBuf[i] = b;
				m_pairFlagsBuf[i] = m_contactFlagsBuffer[k];
				m_pairStrengthBuf[i] = b2Min(
					groupAIdx != b2_invalidGroupIndex ? m_groupStrengthBuf[groupAIdx] : 1,
					groupBIdx != b2_invalidGroupIndex ? m_groupStrengthBuf[groupBIdx] : 1);
				m_pairDistanceBuf[i] = b2Distance(b2Vec2(m_positionXBuffer[a], m_positionYBuffer[a]),
												  b2Vec2(m_positionXBuffer[b], m_positionYBuffer[b]));
			}
		}

		// Sort
		vector<int32> idxs(m_pairCount);
		std::iota(idxs.begin(), idxs.end(), 0);

		auto ComparePairIndices = [this](int32 a, int32 b) -> bool
		{
			int32 diffA = m_pairIdxABuf[a] - m_pairIdxABuf[b];
			if (diffA != 0) return diffA < 0;
			return m_pairIdxBBuf[a] < m_pairIdxBBuf[b];
		};

		concurrency::parallel_sort(idxs.begin(), idxs.end(), ComparePairIndices);

		reorder(m_pairIdxABuf, idxs);
		reorder(m_pairIdxBBuf, idxs);
		reorder(m_pairFlagsBuf, idxs);
		reorder(m_pairStrengthBuf, idxs);
		reorder(m_pairDistanceBuf, idxs);

		auto MatchPairIndices = [this](int32 a, int32 b) -> bool
		{
			return m_pairIdxABuf[a] == m_pairIdxABuf[b] && m_pairIdxBBuf[a] == m_pairIdxBBuf[b];
		};

		idxs = GetUniqueIdxs(m_pairCount, MatchPairIndices);
		reorder(m_pairIdxABuf, idxs);
		reorder(m_pairIdxBBuf, idxs);
		reorder(m_pairFlagsBuf, idxs);
		reorder(m_pairStrengthBuf, idxs);
		reorder(m_pairDistanceBuf, idxs);

		m_pairCount = idxs.size();
	}
	if (particleFlags & k_triadFlags)
	{
		b2VoronoiDiagram diagram(
			&m_world->m_stackAllocator, lastIndex - firstIndex);
		for (int32 i = firstIndex; i < lastIndex; i++)
		{
			uint32 flags = m_flagsBuffer[i];
			int32 groupIdx = m_groupIdxBuffer[i];
			if (!(flags & b2_zombieParticle) &&
				ParticleCanBeConnected(flags, groupIdx, m_groupFlagsBuf[groupIdx]))
			{
				diagram.AddGenerator(
					b2Vec2(m_positionXBuffer[i], m_positionYBuffer[i]), i, filter.IsNecessary(i));
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
					const float32 pax = m_system->m_positionXBuffer[a];
					const float32 pay = m_system->m_positionYBuffer[a];
					const float32 pbx = m_system->m_positionXBuffer[b];
					const float32 pby = m_system->m_positionYBuffer[b];
					const float32 pcx = m_system->m_positionXBuffer[c];
					const float32 pcy = m_system->m_positionYBuffer[c];
					float32 dabx = pax - pbx;
					float32 daby = pay - pby;
					float32 dbcx = pbx - pcx;
					float32 dbcy = pby - pcy;
					float32 dcax = pcx - pax;
					float32 dcay = pcy - pay;
					float32 maxDistanceSquared = b2_maxTriadDistanceSquared *
												 m_system->m_squaredDiameter;
					if (b2Dot(dabx, daby, dabx, daby) > maxDistanceSquared ||
						b2Dot(dbcx, dbcy, dbcx, dbcy) > maxDistanceSquared ||
						b2Dot(dcax, dcay, dcax, dcay) > maxDistanceSquared)
					{
						return;
					}
					int32 groupAIdx = m_system->m_groupIdxBuffer[a];
					int32 groupBIdx = m_system->m_groupIdxBuffer[b];
					int32 groupCIdx = m_system->m_groupIdxBuffer[c];
					int32& triadCount = m_system->m_triadCount;
					if (triadCount >= m_system->m_triadBufferSize)
						m_system->ResizeTriadBuffers(triadCount * 2);
					int i = triadCount;
					triadCount++;
					m_system->m_triadIdxABuf[i] = a;
					m_system->m_triadIdxBBuf[i] = b;
					m_system->m_triadIdxCBuf[i] = c;
					m_system->m_triadFlagsBuf[i] = af | bf | cf;
					const std::vector<float32>& strengthBuf = m_system->m_groupStrengthBuf;
					m_system->m_triadStrengthBuf[i] = b2Min(b2Min(
						groupAIdx ? strengthBuf[groupAIdx] : 1,
						groupBIdx ? strengthBuf[groupBIdx] : 1),
						groupCIdx ? strengthBuf[groupCIdx] : 1);
					float32 midPointX = (float32)1 / 3 * (pax + pbx + pcx);
					float32 midPointY = (float32)1 / 3 * (pay + pby + pcy);
					m_system->m_triadPAXBuf[i] = pax - midPointX;
					m_system->m_triadPAYBuf[i] = pay - midPointY;
					m_system->m_triadPBXBuf[i] = pbx - midPointX;
					m_system->m_triadPBYBuf[i] = pby - midPointY;
					m_system->m_triadPCXBuf[i] = pcx - midPointX;
					m_system->m_triadPCYBuf[i] = pcy - midPointY;
					m_system->m_triadKABuf[i] = -b2Dot(dcay, dcay, dabx, daby);
					m_system->m_triadKBBuf[i] = -b2Dot(daby, daby, dbcx, dbcy);
					m_system->m_triadKCBuf[i] = -b2Dot(dbcy, dbcy, dcax, dcay);
					m_system->m_triadSBuf[i] = b2Cross(pax, pay, pbx, pby) + b2Cross(pbx, pby, pcx, pcy) + b2Cross(pcx, pcy, pax, pay);
					triadCount++;
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


		auto CompareTriadIndices = [this](int32 a, int32 b) -> bool
		{
			int32 diffA = m_triadIdxABuf[a] - m_triadIdxABuf[b];
			if (diffA != 0) return diffA < 0;
			int32 diffB = m_triadIdxBBuf[a] - m_triadIdxBBuf[b];
			if (diffB != 0) return diffB < 0;
			return m_triadIdxCBuf[a] < m_triadIdxCBuf[b];
		};
		auto MatchTriadIndices = [this](int32 a, int32 b) -> bool
		{
			return m_triadIdxABuf[a] == m_triadIdxABuf[b]
				&& m_triadIdxBBuf[a] == m_triadIdxBBuf[b]
				&& m_triadIdxCBuf[a] == m_triadIdxCBuf[b];
		};



		vector<int32> idxs = GetStableSortedIdxs(m_triadCount, CompareTriadIndices);
		reorder(m_triadIdxABuf, idxs);
		reorder(m_triadIdxBBuf, idxs);
		reorder(m_triadIdxCBuf, idxs);
		reorder(m_triadFlagsBuf, idxs);
		reorder(m_triadStrengthBuf, idxs);
		reorder(m_triadPAXBuf, idxs);
		reorder(m_triadPAYBuf, idxs);
		reorder(m_triadPBXBuf, idxs);
		reorder(m_triadPBYBuf, idxs);
		reorder(m_triadPCXBuf, idxs);
		reorder(m_triadPCYBuf, idxs);
		reorder(m_triadKABuf, idxs);
		reorder(m_triadKBBuf, idxs);
		reorder(m_triadKCBuf, idxs);
		reorder(m_triadSBuf, idxs);

		idxs = GetUniqueIdxs(m_triadCount, MatchTriadIndices);
		reorder(m_triadIdxABuf, idxs);
		reorder(m_triadIdxBBuf, idxs);
		reorder(m_triadIdxCBuf, idxs);
		reorder(m_triadFlagsBuf, idxs);
		reorder(m_triadStrengthBuf, idxs);
		reorder(m_triadPAXBuf, idxs);
		reorder(m_triadPAYBuf, idxs);
		reorder(m_triadPBXBuf, idxs);
		reorder(m_triadPBYBuf, idxs);
		reorder(m_triadPCXBuf, idxs);
		reorder(m_triadPCYBuf, idxs);
		reorder(m_triadKABuf, idxs);
		reorder(m_triadKBBuf, idxs);
		reorder(m_triadKCBuf, idxs);
		reorder(m_triadSBuf, idxs);
		m_triadCount = idxs.size();
	}
}
void b2ParticleSystem::AFUpdatePairsAndTriads(
int32 firstIndex, int32 lastIndex, const AFConnectionFilter& filter)
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
	bool hasPairs = !af::where(afFlagBuf & k_pairFlags).isempty();
	bool hasTriads = !af::where(afFlagBuf & k_triadFlags).isempty();

	if (hasPairs)
	{
		const af::array& a = afContactIdxABuf;
		const af::array& b = afContactIdxBBuf;
		const af::array& af = afFlagBuf(a);
		const af::array& bf = afFlagBuf(b);
		const af::array& groupAIdx = afGroupIdxBuf(a);
		const af::array& groupBIdx = afGroupIdxBuf(b);

		const af::array& condIdxs = af::where(a >= firstIndex && a < lastIndex &&
											  b >= firstIndex && b < lastIndex &&
											  !((af | bf) & b2_zombieParticle) &&
											  ((af | bf) & k_pairFlags) &&
											  (filter.IsNecessary(a) || filter.IsNecessary(b)) &&
											  AFParticleCanBeConnected(af, groupAIdx, afGroupFlagsBuf(groupAIdx)) &&
											  AFParticleCanBeConnected(bf, groupBIdx, afGroupFlagsBuf(groupBIdx)) &&
											  filter.ShouldCreatePair(a, b));
		if (!condIdxs.isempty())
		{
			const af::array& newPairIdxABuf = a(condIdxs);
			const af::array& newPairIdxBBuf = b(condIdxs);
			const af::array& newPairFlagsBuf = afContactFlagsBuf(condIdxs);
			const af::array& strengthA = afGroupStrengthBuf(groupAIdx);
			const af::array& strengthB = afGroupStrengthBuf(groupBIdx);
			const af::array& newPairStrengthBuf = af::select(strengthA < strengthB, strengthA, strengthB);
			const af::array& newPairDistanceBuf = afDistance(afPosXBuf(a), afPosYBuf(a), afPosXBuf(b), afPosYBuf(b));

			//Append to existing
			AFJoin(afPairIdxABuf, newPairIdxABuf, af::dtype::s32);
			AFJoin(afPairIdxBBuf, newPairIdxBBuf, af::dtype::s32);
			AFJoin(afPairFlagsBuf, newPairFlagsBuf, af::dtype::u32);
			AFJoin(afPairStrengthBuf, newPairStrengthBuf);
			AFJoin(afPairDistanceBuf, newPairDistanceBuf);

			/*//Sort Pairs
			af::array idxs = AFGetSortedPairIdxs();
			afPairIdxABuf = afPairIdxABuf(idxs);
			afPairIdxBBuf = afPairIdxBBuf(idxs);
			afPairFlagsBuf = afPairFlagsBuf(idxs);
			afPairStrengthBuf = afPairStrengthBuf(idxs);
			afPairDistanceBuf = afPairDistanceBuf(idxs);*/

			// TODO Fix Unique
			//Filter Unique Pairs
			/*af::array idxs = AFGetUniquePairIdxs();
			afPairIdxABuf = afPairIdxABuf(idxs);
			afPairIdxBBuf = afPairIdxBBuf(idxs);
			afPairFlagsBuf = afPairFlagsBuf(idxs);
			afPairStrengthBuf = afPairStrengthBuf(idxs);
			afPairDistanceBuf = afPairDistanceBuf(idxs);*/

			//m_pairCount = idxs.elements();
		}
	}

	if (hasTriads)
	{
		AFVoronoiDiagram diagram(lastIndex - firstIndex);

		const af::array& idxs = af::seq(firstIndex, lastIndex);
		const af::array& flags = afFlagBuf(idxs);
		const af::array& groupIdxs = afGroupIdxBuf(idxs);
		const af::array& condIdxs = af::where(!(flags & b2_zombieParticle) &&
			AFParticleCanBeConnected(flags, groupIdxs, afGroupFlagsBuf(groupIdxs)));
		if (!condIdxs.isempty())
			diagram.AddGenerators(afPosXBuf(condIdxs), afPosYBuf(condIdxs), condIdxs, filter.IsNecessary(condIdxs));
		
		float32 stride = GetParticleStride();
		diagram.Generate(stride / 2, stride * 2);

		class AFUpdateTriadsCallback : public AFVoronoiDiagram::NodeCallback
		{
			void operator()(af::array a, af::array b, af::array c)
			{
				af::array af = m_system->afFlagBuf(a);
				af::array bf = m_system->afFlagBuf(b);
				af::array cf = m_system->afFlagBuf(c);

				af::array condIdxs = af::where(((af | bf | cf) & k_triadFlags) &&
					m_filter->AFShouldCreateTriad(a, b, c));
				if (!condIdxs.isempty())
				{
					a = a(condIdxs);
					b = b(condIdxs);
					c = c(condIdxs);

					af::array pax = m_system->afPosXBuf(a);
					af::array pay = m_system->afPosYBuf(a);
					af::array pbx = m_system->afPosXBuf(b);
					af::array pby = m_system->afPosYBuf(b);
					af::array pcx = m_system->afPosXBuf(c);
					af::array pcy = m_system->afPosYBuf(c);
					af::array dabx = pax - pbx;
					af::array daby = pay - pby;
					af::array dbcx = pbx - pcx;
					af::array dbcy = pby - pcy;
					af::array dcax = pcx - pax;
					af::array dcay = pcy - pay;

					float32 maxDistanceSquared = b2_maxTriadDistanceSquared *
						m_system->m_squaredDiameter;

					af::array condIdxs = af::where(!(b2Dot(dabx, daby, dabx, daby) > maxDistanceSquared ||
						b2Dot(dbcx, dbcy, dbcx, dbcy) > maxDistanceSquared ||
						b2Dot(dcax, dcay, dcax, dcay) > maxDistanceSquared));
					if (!condIdxs.isempty())
					{
						a = a(condIdxs);
						b = b(condIdxs);
						c = c(condIdxs);
						af = af(condIdxs);
						bf = bf(condIdxs);
						cf = cf(condIdxs);
						pax	= pax(condIdxs);
						pay	= pay(condIdxs);
						pbx	= pbx(condIdxs);
						pby	= pby(condIdxs);
						pcx	= pcx(condIdxs);
						pcy	= pcy(condIdxs);
						dabx = dabx(condIdxs);
						daby = daby(condIdxs);
						dbcx = dbcx(condIdxs);
						dbcy = dbcy(condIdxs);
						dcax = dcax(condIdxs);
						dcay = dcay(condIdxs);

						const af::array& groupAIdx = m_system->afGroupIdxBuf(a);
						const af::array& groupBIdx = m_system->afGroupIdxBuf(b);
						const af::array& groupCIdx = m_system->afGroupIdxBuf(c);

						const af::array& strengthBuf = m_system->afGroupStrengthBuf;
						af::array s = af::min(af::min(
							af::select(groupAIdx != -1, strengthBuf(groupAIdx), 1),
							af::select(groupBIdx != -1, strengthBuf(groupBIdx), 1)),
							af::select(groupCIdx != -1, strengthBuf(groupCIdx), 1));

						const af::array& midPointX = 1.0f / 3.0f * (pax + pbx + pcx);
						const af::array& midPointY = 1.0f / 3.0f * (pay + pby + pcy);

						m_system->AFJoin(m_system->afTriadIdxABuf, a, af::dtype::s32);
						m_system->AFJoin(m_system->afTriadIdxBBuf, b, af::dtype::s32);
						m_system->AFJoin(m_system->afTriadIdxCBuf, c, af::dtype::s32);
						m_system->AFJoin(m_system->afTriadFlagsBuf, af | bf | cf, af::dtype::u32);
						m_system->AFJoin(m_system->afTriadStrengthBuf, s);
						m_system->AFJoin(m_system->afTriadPAXBuf, pax - midPointX);
						m_system->AFJoin(m_system->afTriadPAYBuf, pay - midPointY);
						m_system->AFJoin(m_system->afTriadPBXBuf, pbx - midPointX);
						m_system->AFJoin(m_system->afTriadPBYBuf, pby - midPointY);
						m_system->AFJoin(m_system->afTriadPCXBuf, pcx - midPointX);
						m_system->AFJoin(m_system->afTriadPCYBuf, pcy - midPointY);
						m_system->AFJoin(m_system->afTriadKABuf, -b2Dot(dcax, dcay, dabx, daby));
						m_system->AFJoin(m_system->afTriadKBBuf, -b2Dot(dabx, daby, dbcx, dbcy));
						m_system->AFJoin(m_system->afTriadKCBuf, -b2Dot(dbcx, dbcy, dcax, dcay));
						m_system->AFJoin(m_system->afTriadSBuf, b2Cross(pax, pay, pbx, pby) + b2Cross(pbx, pby, pcx, pcy) + b2Cross(pcx, pcy, pax, pay));
					}
				}
			}
			b2ParticleSystem* m_system;
			const AFConnectionFilter* m_filter;
		public:
			AFUpdateTriadsCallback(b2ParticleSystem* system, const AFConnectionFilter* filter)
			{
				m_system = system;
				m_filter = filter;
			}
		} callback(this, &filter);
		diagram.GetNodes(callback);


		// TODO Fix Unique
		/*
		af::array uniqueIdxs = AFGetUniqueTriadIdxs();
		afTriadStrengthBuf = afTriadStrengthBuf(uniqueIdxs);
		afTriadPAXBuf = afTriadPAXBuf(uniqueIdxs);
		afTriadPAYBuf = afTriadPAYBuf(uniqueIdxs);
		afTriadPBXBuf = afTriadPBXBuf(uniqueIdxs);
		afTriadPBYBuf = afTriadPBYBuf(uniqueIdxs);
		afTriadPCXBuf = afTriadPCXBuf(uniqueIdxs);
		afTriadPCYBuf = afTriadPCYBuf(uniqueIdxs);
		afTriadKABuf = afTriadKABuf(uniqueIdxs);
		afTriadKBBuf = afTriadKBBuf(uniqueIdxs);
		afTriadKCBuf = afTriadKCBuf(uniqueIdxs);
		afTriadSBuf = afTriadSBuf(uniqueIdxs);
		
		m_triadCount = uniqueIdxs.elements()
		*/
	}
}

// Only called from SolveZombie() or JoinParticleGroups().
void b2ParticleSystem::DestroyParticleGroup(int32 groupIdx)
{
	b2Assert(m_groupCount > 0);
	b2Assert(groupIdx > 0);

	if (m_world->m_destructionListener)
	{
		m_world->m_destructionListener->SayGoodbye(groupIdx);
	}

	SetGroupFlags(groupIdx, 0);
	for (int32 i = m_groupFirstIdxBuf[groupIdx]; i < m_groupLastIdxBuf[groupIdx]; i++)
	{
		m_groupIdxBuffer[i] = b2_invalidGroupIndex;
	}

	m_freeGroupIdxBuffer.push_back(groupIdx);
	//--m_groupCount;
}

void b2ParticleSystem::ComputeWeight()
{
	// calculates the sum of contact-weights for each particle
	// that means dimensionless density
	//memset(m_weightBuffer.data(), 0, sizeof(*(m_weightBuffer.data())) * m_count);
	//m_weightBuffer.resize(m_particleBufferSize);
	std::fill(m_weightBuffer.begin(), m_weightBuffer.end(), 0);
	for (int32 k = 0; k < m_bodyContactCount; k++)
	{
		int32 a = m_bodyContactIdxBuffer[k];
		float32 w = m_bodyContactWeightBuffer[k];
		m_weightBuffer[a] += w;
	}
	for (int32 k = 0; k < m_contactCount; k++)
	{
		int32 a = m_contactIdxABuffer[k];
		int32 b = m_contactIdxBBuffer[k];
		float32 w = m_contactWeightBuffer[k];
		m_weightBuffer[a] += w;
		m_weightBuffer[b] += w;
	}
}
void b2ParticleSystem::AFComputeWeight()
{
	afWeightBuf = af::constant(0.0f, m_count);

	if (m_bodyContactCount)
		afWeightBuf(afBodyContactIdxBuf) += afBodyContactWeightBuf;

	if (m_contactCount)
	{
		afWeightBuf(afContactIdxABuf) += afContactWeightBuf;
		afWeightBuf(afContactIdxBBuf) += afContactWeightBuf;
	}
}

void b2ParticleSystem::FilterRealGroups()
{
	m_realGroupIdxBuffer.resize(m_groupCount);
	std::iota(m_realGroupIdxBuffer.begin(), m_realGroupIdxBuffer.end(), 0);

	std::sort(m_freeGroupIdxBuffer.begin(), m_freeGroupIdxBuffer.end());

	auto ib = std::begin(m_freeGroupIdxBuffer);
	auto iter = std::remove_if(
		std::begin(m_realGroupIdxBuffer), std::end(m_realGroupIdxBuffer),
		[&ib, this](int x) -> bool {
		while (ib != std::end(m_freeGroupIdxBuffer) && *ib < x) ++ib;
		return (ib != std::end(m_freeGroupIdxBuffer) && *ib == x);
	});
}
void b2ParticleSystem::AFFilterRealGroups()
{
	if (!m_freeGroupIdxBuffer.empty())
	{
		af::array afFreeGroupIdxBuf(m_freeGroupIdxBuffer.size(), afArrayDims, m_freeGroupIdxBuffer.data());
		afRealGroupIdxBuf = af::seq(0, m_groupCount);
		af::array empty = af::constant(0, m_groupCount);
		empty(afFreeGroupIdxBuf) = 1;
		const af::array& notEmptyIdxs = af::where(empty);
		afRealGroupIdxBuf = afRealGroupIdxBuf(notEmptyIdxs);
	}
}

void b2ParticleSystem::ComputeDepth()
{
	vector<int32> contactGroups(m_contactCount);
	int32 contactGroupsCount = 0;
	for (int32 k = 0; k < m_contactCount; k++)
	{
		int32 a = m_contactIdxABuffer[k];
		int32 b = m_contactIdxBBuffer[k];
		const int32 groupAIdx = m_groupIdxBuffer[a];
		const int32 groupBIdx = m_groupIdxBuffer[b];
		if (groupAIdx != b2_invalidGroupIndex && groupAIdx == groupBIdx &&
			(m_groupFlagsBuf[a] & b2_particleGroupNeedsUpdateDepth))
		{
			contactGroups[contactGroupsCount++] = k;
		}
	}
	vector<int32> groupsToUpdate(m_groupCount);
	int32 groupsToUpdateCount = 0;

	for each (const int32 groupIdx in m_realGroupIdxBuffer)
	{
		if (m_groupFlagsBuf[groupIdx] & b2_particleGroupNeedsUpdateDepth)
		{
			groupsToUpdate[groupsToUpdateCount++] = groupIdx;
			SetGroupFlags(groupIdx,
				m_groupFlagsBuf[groupIdx] &
				~b2_particleGroupNeedsUpdateDepth);
			for (int32 i = m_groupFirstIdxBuf[groupIdx]; i < m_groupLastIdxBuf[groupIdx]; i++)
			{
				m_accumulationBuf[i] = 0;
			}
		}
	}
	// Compute sum of weight of contacts except between different groups.
	for (int32 k = 0; k < contactGroupsCount; k++)
	{
		const int32 contactPos = contactGroups[k];
		int32 a = m_contactIdxABuffer[contactPos];
		int32 b = m_contactIdxBBuffer[contactPos];
		float32 w = m_contactWeightBuffer[contactPos];
		m_accumulationBuf[a] += w;
		m_accumulationBuf[b] += w;
	}
	b2Assert(m_depthBuffer);
	for (int32 i = 0; i < groupsToUpdateCount; i++)
	{
		const int32 groupIdx = groupsToUpdate[i];
		for (int32 i = m_groupFirstIdxBuf[groupIdx]; i < m_groupLastIdxBuf[groupIdx]; i++)
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
			const int32 contactPos = contactGroups[k];
			int32 a = m_contactIdxABuffer[contactPos];
			int32 b = m_contactIdxBBuffer[contactPos];
			float32 r = 1 - m_contactWeightBuffer[contactPos];
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
		const int32 groupIdx = groupsToUpdate[i];
		for (int32 i = m_groupFirstIdxBuf[groupIdx]; i < m_groupLastIdxBuf[groupIdx]; i++)
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
void b2ParticleSystem::AFComputeDepth()
{
	AFRequestBuffer(afDepthBuf, afHasDepthBuf);
	const af::array& a = afContactIdxABuf;
	const af::array& b = afContactIdxBBuf;
	const af::array groupAIdx = afGroupIdxBuf(a);
	const af::array groupBIdx = afGroupIdxBuf(b);
	af::array afContactGroupIdxs = af::where(groupAIdx != b2_invalidGroupIndex
										 	 && groupBIdx == groupBIdx
											 && afGroupFlagsBuf(groupAIdx) & b2_particleGroupNeedsUpdateDepth);
	

	//int contactGroupsCount = afContactGroups.elements();

	af::array afGroupsToUpdateIdxs = af::where(afGroupFlagsBuf(afRealGroupIdxBuf) & b2_particleGroupNeedsUpdateDepth);
	int32 groupsToUpdateCount = afGroupsToUpdateIdxs.elements();
	if (!afGroupsToUpdateIdxs.isempty())
	{
		AFSetGroupFlags(afGroupsToUpdateIdxs, afGroupFlagsBuf(afGroupsToUpdateIdxs) &
			~b2_particleGroupNeedsUpdateDepth);

		gfor(af::seq i, groupsToUpdateCount)
		{
			af::array groupIdx = afGroupsToUpdateIdxs(i);
			af::array partIdxs = af::seq(afGroupFirstIdxBuf(groupIdx).scalar<int32>(),
										 afGroupLastIdxBuf(groupIdx).scalar<int32>());
			afAccumulationBuf(partIdxs) = 0;
		}
	}

	// Compute sum of weight of contacts except between different groups.
	if (!afContactGroupIdxs.isempty())
	{
		af::array a = afContactIdxABuf(afContactGroupIdxs);
		af::array b = afContactIdxBBuf(afContactGroupIdxs);
		af::array w = afContactWeightBuf(afContactGroupIdxs);
		afAccumulationBuf(a) += w;
		afAccumulationBuf(b) += w;
	}

	gfor(af::seq i, groupsToUpdateCount)
	{
		af::array groupIdx = afGroupsToUpdateIdxs(i);
		af::array partIdxs = af::seq(afGroupFirstIdxBuf(groupIdx).scalar<int32>(),
			afGroupLastIdxBuf(groupIdx).scalar<int32>());
		af::array d = afDepthBuf(partIdxs);
		afDepthBuf(partIdxs) = af::select(afAccumulationBuf(partIdxs) < 0.8f, 
										  af::constant(0, groupsToUpdateCount), b2_maxFloat);
	}

	// The number of iterations is equal to particle number from the deepest
	// particle to the nearest surface particle, and in general it is smaller
	// than sqrt of total particle number.
	int32 iterationCount = (int32)b2Sqrt((float)m_count);
	for (int32 t = 0; t < iterationCount; t++)
	{
		bool updated = false;
		
		af::array a = afContactIdxABuf(afContactGroupIdxs);
		af::array b = afContactIdxBBuf(afContactGroupIdxs);
		af::array r = 1.0f - afContactWeightBuf(afContactGroupIdxs);
		af::array ap0 = afDepthBuf(a);
		af::array bp0 = afDepthBuf(b);
		af::array ap1 = bp0 + r;
		af::array bp1 = ap0 + r;
		af::array aCondIdxs = af::where(ap0 > ap1);
		if (!aCondIdxs.isempty())
		{
			ap0(aCondIdxs) = ap1(aCondIdxs);
			updated = true;
		}
		af::array bCondIdxs = af::where(bp0 > bp1);
		{
			bp0(bCondIdxs) = bp1(bCondIdxs);
			updated = true;
		}
		if (!updated)
			break;
	}
	gfor(af::seq i, groupsToUpdateCount)
	{
		af::array groupIdx = afGroupsToUpdateIdxs(i);
		af::array partIdxs = af::seq(afGroupFirstIdxBuf(groupIdx).scalar<int32>(),
			afGroupLastIdxBuf(groupIdx).scalar<int32>());
		af::array d = afDepthBuf(partIdxs);
		afDepthBuf(partIdxs) = af::select(afDepthBuf(partIdxs) < b2_maxFloat,
										  afDepthBuf(partIdxs) * m_particleDiameter, 0);
	}
}

b2ParticleSystem::InsideBoundsEnumerator
b2ParticleSystem::GetInsideBoundsEnumerator(const b2AABB& aabb) const
{
	uint32 lowerTag = computeTag(m_inverseDiameter * aabb.lowerBound.x - 1,
		m_inverseDiameter * aabb.lowerBound.y - 1);
	uint32 upperTag = computeTag(m_inverseDiameter * aabb.upperBound.x + 1,
		m_inverseDiameter * aabb.upperBound.y + 1);

	auto beginIt = m_proxyTagBuffer.begin();
	auto endIt   = m_proxyTagBuffer.begin() + m_count;
	auto firstIt = lower_bound(beginIt, endIt, lowerTag);
	auto lastIt  = upper_bound(firstIt, endIt, upperTag);
	const int firstPos = firstIt - beginIt;
	const int lastPos  = lastIt  - beginIt;

	return InsideBoundsEnumerator(this, lowerTag, upperTag, firstPos, lastPos);
}
af::array b2ParticleSystem::AFGetInsideBoundsEnumerator(const b2AABB& aabb) const
{
	uint32 lowerTag = computeTag(m_inverseDiameter * aabb.lowerBound.x - 1,
		m_inverseDiameter * aabb.lowerBound.y - 1);
	uint32 upperTag = computeTag(m_inverseDiameter * aabb.upperBound.x + 1,
		m_inverseDiameter * aabb.upperBound.y + 1);

	const af::array& idxs = af::where(afProxyTagBuf >= lowerTag && afProxyTagBuf <= upperTag);

	uint32 m_xLower = lowerTag & xMask;
	uint32 m_xUpper = upperTag & xMask;
	uint32 m_yLower = lowerTag & yMask;
	uint32 m_yUpper = upperTag & yMask;

	//Get Next
	const af::array& xTags = afProxyTagBuf(idxs) & xMask;
	const af::array& idxsInBounds = af::where(xTags >= m_xLower && xTags <= m_xUpper);

	return afProxyIdxBuf(idxsInBounds);
}

void b2ParticleSystem::FindContacts()
{
	for (int32 i = 0; i < m_count; i++)
	{
		const uint32& tag = m_proxyTagBuffer[i];
		m_findContactRightTagBuf[i]		  = computeRelativeTag(tag,  1, 0);
		m_findContactBottomLeftTagBuf[i]  = computeRelativeTag(tag, -1, 1);
		m_findContactBottomRightTagBuf[i] = computeRelativeTag(tag,  1, 1);
	}
	const int end = m_count;

	//clock_t clockFillBuffer = clock();
	for (int a = 0, c = 0; a < end; a++)
	{
		const int32  aIdx = m_proxyIdxBuffer[a];
		const uint32 aTag = m_proxyTagBuffer[a];
		const uint32 rightTag = m_findContactRightTagBuf[a];
		int32 aContactCount = 0;
		for (int b = a + 1; b < end; b++)
		{
			if (rightTag < m_proxyTagBuffer[b]) break;
			int32 bIdx = m_proxyIdxBuffer[b];
			if (!(m_collisionLayerBuffer[aIdx] & m_collisionLayerBuffer[bIdx]) &&
				ShouldCollide(aIdx, bIdx)) break;
			m_findContactIdxABuf[a][aContactCount] = aIdx;
			m_findContactIdxBBuf[a][aContactCount] = bIdx;
			if (++aContactCount >= MAX_CONTACTS_PER_PARTICLE) goto cnt;
		}
		const uint32 bottomLeftTag = m_findContactBottomLeftTagBuf[a];
		for (; c < end; c++)
		{
			if (bottomLeftTag <= m_proxyTagBuffer[c]) break;
		}
		const uint32 bottomRightTag = m_findContactBottomRightTagBuf[a];
		for (int b = c; b < end; b++)
		{
			if (bottomRightTag < m_proxyTagBuffer[b]) break;
			int32 bIdx = m_proxyIdxBuffer[b];
			if (!(m_collisionLayerBuffer[aIdx] & m_collisionLayerBuffer[bIdx]) &&
				ShouldCollide(aIdx, bIdx)) break;
			m_findContactIdxABuf[a][aContactCount] = aIdx;
			m_findContactIdxBBuf[a][aContactCount] = bIdx;
			if (++aContactCount >= MAX_CONTACTS_PER_PARTICLE) goto cnt;
		}
		cnt:;
		m_findContactCountBuf[a] = aContactCount;
	}
	//cout << "FillBuffer " << (clock() - clockFillBuffer) * 1000 / CLOCKS_PER_SEC << "ms\n";

	clock_t clockAddContact = clock();
	m_contactCount = 0;
	for (int i = 0; i < m_count; i++)
		for (int j = 0; j < m_findContactCountBuf[i]; j++)
			AddContact(m_findContactIdxABuf[i][j], m_findContactIdxBBuf[i][j], m_contactCount);
	//cout << "AddContacts " << (clock() - clockAddContact) * 1000 / CLOCKS_PER_SEC << "ms\n";

	std::snprintf(debugString, 64, "contactCount: %d", m_contactCount);
}
void b2ParticleSystem::AFFindContacts()
{

	int maxContactCount = m_count * MAX_CONTACTS_PER_PARTICLE;
	vector<int32> contactIdxAs(maxContactCount, b2_invalidParticleIndex);
	vector<int32> contactIdxBs(maxContactCount, b2_invalidParticleIndex);

	const uint32* rightTagBuf = afComputeRelativeTag(afProxyTagBuf, 1, 0).host<uint32>();
	const uint32* bottomLeftTagBuf = afComputeRelativeTag(afProxyTagBuf, -1, 1).host<uint32>();
	const uint32* bottomRightTagBuf = afComputeRelativeTag(afProxyTagBuf, 1, 1).host<uint32>();

	const int end = m_count;
	for (int a = 0, c = 0; a < end; a++)
	{
		const int32  aIdx = m_proxyIdxBuffer[a];
		const uint32 aTag = m_proxyTagBuffer[a];
		const uint32 rightTag = rightTagBuf[a];
		int32 writeOffset = 0;
		const uint32 writeStart = a * MAX_CONTACTS_PER_PARTICLE;
		for (int b = a + 1; b < end; b++)
		{
			if (rightTag < m_proxyTagBuffer[b]) break;
			contactIdxAs[writeStart + writeOffset] = aIdx;
			contactIdxBs[writeStart + writeOffset] = m_proxyIdxBuffer[b];
			if (++writeOffset >= MAX_CONTACTS_PER_PARTICLE) goto cnt;
		}
		const uint32 bottomLeftTag = bottomLeftTagBuf[a];
		for (; c < end; c++)
		{
			if (bottomLeftTag <= m_proxyTagBuffer[c]) break;
		}
		const uint32 bottomRightTag = bottomRightTagBuf[a];
		for (int b = c; b < end; b++)
		{
			if (bottomRightTag < m_proxyTagBuffer[b]) break;
			contactIdxAs[writeStart + writeOffset] = aIdx;
			contactIdxBs[writeStart + writeOffset] = m_proxyIdxBuffer[b];
			if (++writeOffset >= MAX_CONTACTS_PER_PARTICLE) goto cnt;
		}
		cnt:;
	}

	//Copy to GPU
	af::array afContactIdxsA, afContactIdxsB;
	CPPtoAF(maxContactCount, contactIdxAs, afContactIdxsA);
	CPPtoAF(maxContactCount, contactIdxBs, afContactIdxsB);

	//Remove Empty Contacts
	const af::array& k = af::where(afContactIdxsA != b2_invalidParticleIndex &&
								   afContactIdxsB != b2_invalidParticleIndex);
	if (!k.isempty())
	{
		afContactIdxsA = afContactIdxsA(k);
		afContactIdxsB = afContactIdxsB(k);

		//Remove Contacts that shouldn't collide
		const af::array& k2 = af::where(AFShouldCollide(afContactIdxsA, afContactIdxsB));
		if (!k2.isempty())
		{
			AFAddContacts(afContactIdxsA(k2), afContactIdxsB(k2));
		}
	}
}
inline void b2ParticleSystem::AddContact(int32 a, int32 b, int32& contactCount)
{
	float32 dx = m_positionXBuffer[b] - m_positionXBuffer[a];
	float32 dy = m_positionYBuffer[b] - m_positionYBuffer[a];
	float32 distBtParticlesSq = (dx * dx) + (dy * dy);			//dot product
	if (distBtParticlesSq < m_squaredDiameter)
	{
		if (m_contactBufferSize <= contactCount)
			ResizeContactBuffers(contactCount * 2);
		float32 invD = b2InvSqrt(distBtParticlesSq);
		m_contactIdxABuffer[contactCount] = a;
		m_contactIdxBBuffer[contactCount] = b;
		m_contactFlagsBuffer[contactCount] = m_flagsBuffer[a] | m_flagsBuffer[b];
		int32 matAIdx = m_partMatIdxBuffer[a];
		int32 matBIdx = m_partMatIdxBuffer[b];
		m_contactMatFlagsBuffer[contactCount] = m_partMatFlagsBuf[matAIdx] | m_partMatFlagsBuf[matBIdx];
		m_contactWeightBuffer[contactCount] = 1 - distBtParticlesSq * invD * m_inverseDiameter;
		float32 invM = 1 / (m_partMatMassBuf[matAIdx] + m_partMatMassBuf[matBIdx]);
		m_contactMassBuffer[contactCount] = invM > 0 ? invM : 0;
		m_contactNormalXBuffer[contactCount] = invD * dx;
		m_contactNormalYBuffer[contactCount] = invD * dy;
		contactCount++;
	
	}
}
inline void b2ParticleSystem::AFAddContacts(const af::array& a, const af::array& b)
{
	af::array dx = afPosXBuf(b) - afPosXBuf(a);
	af::array dy = afPosYBuf(b) - afPosYBuf(a);
	af::array distBtParticlesSq = (dx * dx) + (dy * dy);	//dot product

	const af::array& k = af::where(distBtParticlesSq < m_squaredDiameter);
	m_contactCount = k.elements();
	if (k.isempty()) return;
	
	dx = dx(k);
	dy = dy(k);
	distBtParticlesSq = distBtParticlesSq(k);

	afContactIdxABuf = a(k);
	afContactIdxBBuf = b(k);
	afContactFlagsBuf = afFlagBuf(afContactIdxABuf) | afFlagBuf(afContactIdxBBuf);
	const af::array& matIdxsA = afPartMatIdxBuf(afContactIdxABuf);
	const af::array& matIdxsB = afPartMatIdxBuf(afContactIdxBBuf);
	afContactMatFlagsBuf = afPartMatFlagsBuf(matIdxsA) | afPartMatFlagsBuf(matIdxsB);
	const af::array& invD = afInvSqrt(distBtParticlesSq);
	afContactWeightBuf = 1 - distBtParticlesSq * invD * m_inverseDiameter;
	const af::array& invMass = 1 / (afPartMatMassBuf(matIdxsA) + afPartMatMassBuf(matIdxsB));
	afContactMassBuf = af::select(invMass > 0, invMass, 0);
	afContactNormalXBuf = invD * dx;
	afContactNormalYBuf = invD * dy;
}
inline bool b2ParticleSystem::ShouldCollide(int32 a, int32 b) const
{
	const int32& groupAIdx = m_groupIdxBuffer[a];
	const int32& groupBIdx = m_groupIdxBuffer[b];
	const int32& colGroupA = m_groupColGroupBuf[groupAIdx];
	const int32& colGroupB = m_groupColGroupBuf[groupBIdx];
	return ((colGroupA >= 0 || colGroupB >= 0) ||
			(groupAIdx == groupBIdx) ||
			(colGroupA != colGroupB));
}
inline af::array b2ParticleSystem::AFShouldCollide(const af::array& a, const af::array& b) const
{
	const af::array& groupColGroupA = afGroupColGroupBuf(a);
	const af::array& groupColGroupB = afGroupColGroupBuf(b);
	return (afColLayBuf(a) & afColLayBuf(b)
		&& ((groupColGroupA >= 0 || groupColGroupB >= 0)
			|| afGroupIdxBuf(a) == afGroupIdxBuf(b)
			|| groupColGroupA != groupColGroupB));
}
inline bool b2ParticleSystem::ShouldCollide(int32 i, b2Fixture* f) const
{
	if (m_collisionLayerBuffer[i] & f->GetCollisionLayers())
		return false;
	int32 partColGroup = m_groupColGroupBuf[m_groupIdxBuffer[i]];
	if (partColGroup >= 0)								return true;
	if (f->GetFilterData().groupIndex >= 0)				return true;
	if (partColGroup != f->GetFilterData().groupIndex)	return true;
	return false;
}
inline af::array b2ParticleSystem::AFShouldCollide(const af::array& i, b2Fixture* f) const
{
	const af::array& groupColGroup = afGroupColGroupBuf(i);
	uint32 fixColGroup = f->GetFilterData().groupIndex;
	return (afColLayBuf(i) & f->GetCollisionLayers()
		&& ((groupColGroup >= 0 || fixColGroup >= 0)
			|| afGroupIdxBuf(i) == fixColGroup
			|| groupColGroup != fixColGroup));
}

inline af::array b2ParticleSystem::afInvSqrt(af::array x)
{
	af::array xhalf = 0.5f * x.as(af::dtype::f32);
	af::array i = x.as(af::dtype::s32);				// store floating-point bits in integer
	i = 0x5f3759df - (i >> 1);						// initial guess for Newton's method
	x = i.as(af::dtype::f32);						// convert new bits into float
	x *= 1.5f - xhalf * x * x;						// One round of Newton's method
	return x;

	/*af::array xhalf = 0.5f * x.as(af::dtype::f32);
	x = 0x5f3759df - (x.as(af::dtype::s32) >> 1);
	x *= 1.5f - xhalf * x * x;
	return x;*/
}



static inline bool b2ParticleContactIsZombie(const uint32& flags)
{
	return (flags & b2_zombieParticle) == b2_zombieParticle;
}
static inline af::array afParticleContactIsZombie(const af::array& flags)
{
	return (flags & b2_zombieParticle) == b2_zombieParticle;
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
	    return (contact.GetFlags() & b2_particleContactFilterParticle)
	        && !m_contactFilter->ShouldCollide(m_system, contact.GetIndexA(),
	        								   contact.GetIndexB());
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

void b2ParticleSystem::UpdateContacts(bool exceptZombie)
{
	if (afProxyIdxBuf.isempty()) return;

	// Update Proxy Tags
	afProxyTagBuf = afComputeTag(m_inverseDiameter * afPosXBuf(afProxyIdxBuf),
		m_inverseDiameter * afPosYBuf(afProxyIdxBuf));


	// Sort Proxies by Tag
	//clock_t clockSort = clock();
	AFSort(afProxyTagBuf, afProxyIdxBuf, true);
	CopyProxiesToCPU();
	//cout << "SortProxies " << (clock() - clockSort) * 1000 / CLOCKS_PER_SEC << "ms\n";
	//SortProxies();

	//find Body COntacts in seperate Thread
	//clock_t clockfindBodyContacts = clock();
	findBodyContactsThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)StartBodyContactThread, this, 0, NULL);

	// While findBodyContactsThread is running
	//clock_t clockPartContacts = clock();
	FindContacts();
	//CopyContactsToGPU();
	//cout << "FindContacts " << (clock() - clockPartContacts) * 1000 / CLOCKS_PER_SEC << "ms\n";

	WaitForSingleObject(findBodyContactsThread, INFINITE);
	//cout << "FindBodyContacts " << (clock() - clockfindBodyContacts) * 1000 / CLOCKS_PER_SEC << "ms\n";

	if (exceptZombie)
	{
		RemoveFromVectorsIf(m_contactFlagsBuffer, m_contactIdxABuffer, m_contactIdxBBuffer,
			m_contactWeightBuffer, m_contactMassBuffer, 
			m_contactNormalXBuffer,	m_contactNormalYBuffer, 
			m_contactMatFlagsBuffer,
			m_contactCount, b2ParticleContactIsZombie, true);
	}
	
}
void b2ParticleSystem::AFUpdateContacts(bool exceptZombie)
{
	if (afProxyIdxBuf.isempty()) return;

	// Update Proxy Tags
	afProxyTagBuf = afComputeTag(m_inverseDiameter * afPosXBuf(afProxyIdxBuf),
								 m_inverseDiameter * afPosYBuf(afProxyIdxBuf));


	// Sort Proxies by Tag
	//clock_t clockSort = clock();
	AFSort(afProxyTagBuf, afProxyIdxBuf, true);
	CopyProxiesToCPU();
	//cout << "SortProxies " << (clock() - clockSort) * 1000 / CLOCKS_PER_SEC << "ms\n";
	//SortProxies();

	//find Body COntacts in seperate Thread
	//clock_t clockfindBodyContacts = clock();
	findBodyContactsThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)StartBodyContactThread, this, 0, NULL);

	// While findBodyContactsThread is running
	//clock_t clockPartContacts = clock();
	FindContacts();
	CopyContactsToGPU();
	//cout << "FindContacts " << (clock() - clockPartContacts) * 1000 / CLOCKS_PER_SEC << "ms\n";
	
	

	WaitForSingleObject(findBodyContactsThread, INFINITE);
	//cout << "FindBodyContacts " << (clock() - clockfindBodyContacts) * 1000 / CLOCKS_PER_SEC << "ms\n";

	if (exceptZombie)
	{
		af::array zombies = afParticleContactIsZombie(afContactFlagsBuf);
		af::array zombieIdxs = af::where(zombies);
		af::array aliveIdxs = af::where(!zombies);
		if (!zombieIdxs.isempty())
		{
			AFReorder(aliveIdxs, vector<af::array>{afContactIdxABuf,
				afContactIdxBBuf, afContactWeightBuf, afContactMassBuf,
				afContactNormalXBuf, afContactNormalYBuf,
				afContactFlagsBuf, afContactMatFlagsBuf});
			m_contactCount		 = aliveIdxs.elements();
		}
	}
	std::snprintf(debugString, 64, "contactCount: %d", m_contactCount);
}

void b2ParticleSystem::UpdateProxies()
{
	for (int i = 0; i < m_count; i++)
	{
		const int32 idx = m_proxyIdxBuffer[i];
		m_proxyTagBuffer[i] = computeTag(m_inverseDiameter * m_positionXBuffer[idx],
										 m_inverseDiameter * m_positionYBuffer[idx]);
	}
}

// Sort the proxy array by 'tag'. This orders the particles into rows that
// run left-to-right, top-to-bottom. The rows are spaced m_particleDiameter
// apart, such that a particle in one row can only collide with the rows
// immediately above and below it. This ordering makes collision computation
// tractable.
void b2ParticleSystem::SortProxies()
{
	const auto compareProxies = [this](int32 a, int32 b) -> bool
	{
		return m_proxyTagBuffer[a] < m_proxyTagBuffer[b];
	};
	const vector<int32>& sortedIdxs = GetSortedIdxs(m_count, compareProxies);
	reorder(m_proxyIdxBuffer, m_proxyTagBuffer, sortedIdxs);
}
void b2ParticleSystem::AFSort(af::array& compareArray, af::array& array, bool ascending)
{
	af::array sortedIdxs;
	af::sort(compareArray, sortedIdxs, compareArray, 0, ascending);
	array = array(sortedIdxs);
}
void b2ParticleSystem::AFSort(af::array& compareArray, vector<af::array>& arrays)
{
	af::array sortedIdxs;
	af::sort(compareArray, sortedIdxs, compareArray);
	for each (af::array a in arrays)
		a = a(sortedIdxs);
}
void b2ParticleSystem::AFReorder(af::array& idxs, vector<af::array>& arrays)
{
	for each (af::array a in arrays)
		a = a(idxs);
}
inline void b2ParticleSystem::AFJoin(af::array& origArray, const af::array& newArray, af::dtype type)
{
	origArray = af::join(0, origArray.as(type), newArray.as(type));
}

template <class UnaryPredicate>
vector<int32> b2ParticleSystem::GetValidIdxs(int32 vSize, const UnaryPredicate isValidFunc)
{
	vector<int32> idx(vSize);
	std::iota(idx.begin(), idx.end(), 0);
	int newI = 0;
	for (int i = 0; i < vSize; i++)
	{
		if (isValidFunc(i))
		{
			idx[newI] = i;
			newI++;
		}
	}
	return idx;
}
template <class UnaryPredicate>
vector<int32> b2ParticleSystem::GetSortedIdxs(int32 vSize, const UnaryPredicate& compFunc)
{
	vector<int32> idx(vSize);
	std::iota(idx.begin(), idx.end(), 0);
	concurrency::parallel_sort(idx.begin(), idx.end(), compFunc);
	return idx;
}
af::array b2ParticleSystem::AFGetSortedPairIdxs() const
{
	af::array keys = af::select(afPairIdxABuf != 0, -afPairIdxABuf, -afPairIdxBBuf);
	af::array idxs = af::seq(0, m_pairCount);
	af::sort(keys, idxs, keys, idxs);
	return idxs;
}
template <class UnaryPredicate>
vector<int32> b2ParticleSystem::GetStableSortedIdxs(int32 vSize, UnaryPredicate compFunc)
{
	vector<int32> idx(vSize);
	iota(idx.begin(), idx.end(), 0);
	stable_sort(idx.begin(), idx.end(), compFunc);
	return idx;
}
template <class UnaryPredicate>
vector<int32> b2ParticleSystem::GetUniqueIdxs(int32 vSize, UnaryPredicate compFunc)
{
	vector<int32> idx(vSize);
	iota(idx.begin(), idx.end(), 0);
	unique(idx.begin(), idx.end(), compFunc);
	return idx;
}

af::array b2ParticleSystem::AFGetUniquePairIdxs()
{
	//TODO Fix
	int32 p = m_pairCount + 1;
	af::array hashs = afPairIdxABuf * p + afPairIdxBBuf;
	af::array vals, idxs, locs;
	//af::sea(vals, idxs, locs, hashs);
	return idxs;
}
af::array b2ParticleSystem::AFGetUniqueTriadIdxs()
{
	int32 p = m_triadCount + 1;
	af::array hashs = (afTriadIdxABuf * p + afTriadIdxBBuf) * p + afTriadIdxCBuf;

	af::array vals, idxs, locs;
	//af::setUnique(vals, idxs, locs, hashs);
	af::setUnique(hashs);
	return idxs;
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
void b2ParticleSystem::AFDetectStuckParticle(const af::array& particles)
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
		return;
	
	// This is only called when there is a body contact for this particle.
	const af::array bodyCount = afBodyContactCountBuf(particles) += 1;

	// We want to only trigger detection once per step, the first time we
	// contact more than one fixture in a step for a given particle.
	af::array condIdxs = af::where(bodyCount == 2);
	if (!condIdxs.isempty())
	{
		af::array idxs = particles(condIdxs);
		const af::array consecutiveCount = afConsecutiveContactStepsBuf(idxs) += 1;
		af::array cond2Idxs = af::where(consecutiveCount > m_stuckThreshold);
		if (!cond2Idxs.isempty())
		{
			AFJoin(afStuckParticleBuf, idxs(cond2Idxs));
		}
	}
	afLastBodyContactStepBuf(particles) = m_timestamp;
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
		b2Vec2 p = b2Vec2(m_positionXBuffer[i], m_positionYBuffer[i]);
		aabb->lowerBound = b2Min(aabb->lowerBound, p);
		aabb->upperBound = b2Max(aabb->upperBound, p);
	}
	aabb->lowerBound.x -= m_particleDiameter;
	aabb->lowerBound.y -= m_particleDiameter;
	aabb->upperBound.x += m_particleDiameter;
	aabb->upperBound.y += m_particleDiameter;
}
void b2ParticleSystem::AFComputeAABB(b2AABB* const aabb) const
{
	b2Assert(aabb);
	aabb->lowerBound.x = af::min<float32>(afPosXBuf);
	aabb->lowerBound.y = af::min<float32>(afPosYBuf);
	aabb->upperBound.x = af::max<float32>(afPosXBuf);
	aabb->upperBound.y = af::max<float32>(afPosYBuf);
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
	const vector<int32> bodyContactIdxs, const vector<int32> bodyContactFixtureIdxs,
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
			const int32& idx = bodyContactIdxs[i];
			if (idx == b2_invalidParticleIndex ||
				!(particleFlagsBuffer[idx] &
				  b2_fixtureContactListenerParticle))
			{
				continue;
			}
			fixtureParticle->first = bodyContactFixtureIdxs[i];
			fixtureParticle->second = idx;
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
			if (idxA == b2_invalidParticleIndex ||
				idxB == b2_invalidParticleIndex ||
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
			//cout << "ReportFixture at " << fixtureIdx << "\n";
			b2AABB aabb = fixture->GetAABB(childIndex);
			b2ParticleSystem::InsideBoundsEnumerator enumerator =
								m_system->GetInsideBoundsEnumerator(aabb);

			b2Fixture* fixture = m_system->m_fixtureBuffer[fixtureIdx];
			b2Vec2 bodyPos = fixture->GetBody()->GetPosition();
			int32 index;

			while ((index = enumerator.GetNext()) >= 0)
			{
				//cout << "bodyPos: " << bodyPos.x << "/" << bodyPos.y << "\n";
				//cout << "aabb low: " << aabb.lowerBound.x << "/" << aabb.lowerBound.y << "\n";
				//cout << "aabb upp: " << aabb.upperBound.x << "/" << aabb.upperBound.y << "\n";
				
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
/// Callback class to receive pairs of fixtures and particles which may be
/// overlapping. Used as an argument of b2World::QueryAABB.
class afFixtureParticleQueryCallback : public afQueryCallback
{
public:
	explicit afFixtureParticleQueryCallback(b2ParticleSystem* system)
	{
		m_system = system;
	}

private:
	// Skip reporting particles.
	bool AFShouldQueryParticleSystem(const b2ParticleSystem* system)
	{
		B2_NOT_USED(system);
		return false;
	}

	// Receive a fixture and call ReportFixtureAndParticle() for each particle
	// inside aabb of the fixture.
	bool AFReportFixture(int32 fixtureIdx)
	{
		b2Fixture* fixture = m_system->m_fixtureBuffer[fixtureIdx];
		if (fixture->IsSensor())
			return true;
		
		const b2Shape* shape = fixture->GetShape();
		int32 childCount = shape->GetChildCount();
		for (int32 childIndex = 0; childIndex < childCount; childIndex++)
		{
			b2AABB aabb = fixture->GetAABB(childIndex);
			af::array idxs = m_system->AFGetInsideBoundsEnumerator(aabb);
			if (!idxs.isempty())
				AFReportFixtureAndParticle(fixtureIdx, childIndex, idxs);
		}
		return true;
	}

	// Receive a fixture and a particle which may be overlapping.
	virtual void AFReportFixtureAndParticle(
		int32 fixtureIdx, int32 childIdx, const af::array& partIdxs) = 0;

protected:
	b2ParticleSystem * m_system;
};

void b2ParticleSystem::NotifyBodyContactListenerPreContact(
	FixtureParticleSet* fixtureSet) const
{
	b2ContactListener* const contactListener = GetFixtureContactListener();
	if (contactListener == NULL)
		return;

	fixtureSet->Initialize(m_bodyContactIdxBuffer, m_bodyContactFixtureIdxBuffer,
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
		FixtureParticle fixtureParticleToFind;
		fixtureParticleToFind.first = m_bodyContactFixtureIdxBuffer[i];
		fixtureParticleToFind.second = m_bodyContactIdxBuffer[i];
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
				b2Vec2 ap = b2Vec2(m_system->m_positionXBuffer[a], m_system->m_positionYBuffer[a]);
				float32 d;
				b2Vec2 n;
				float32 partPosX = m_system->m_positionXBuffer[a];
				float32 partPosY = m_system->m_positionYBuffer[a];

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

					int32 bodyContactCount = m_system->m_bodyContactCount;
					m_system->m_bodyContactCount++;
					if (bodyContactCount >= m_system->m_bodyContactBufferSize)
						m_system->ResizeBodyContactBuffers(bodyContactCount * 2);
					m_system->m_bodyContactIdxBuffer[bodyContactCount] = a;
					m_system->m_bodyContactBodyIdxBuffer[bodyContactCount] = bIdx;
					m_system->m_bodyContactFixtureIdxBuffer[bodyContactCount] = fixtureIdx;
					m_system->m_bodyContactWeightBuffer[bodyContactCount] = 1 - d * m_system->m_inverseDiameter;
					m_system->m_bodyContactNormalXBuffer[bodyContactCount] = -n.x;
					m_system->m_bodyContactNormalYBuffer[bodyContactCount] = -n.y;
					m_system->m_bodyContactMassBuffer[bodyContactCount] = invM > 0 ? 1 / invM : 0;
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

	CopyBodyContactsToGPU();
}

void b2ParticleSystem::AFUpdateBodyContacts()
{
	// If the particle contact listener is enabled, generate a set of
	// fixture / particle contacts.

	//TODO
	FixtureParticleSet fixtureSet(&m_world->m_stackAllocator);
	NotifyBodyContactListenerPreContact(&fixtureSet);

	if (m_stuckThreshold > 0)
	{
		afBodyContactCountBuf = af::constant(0, m_count);
		afConsecutiveContactStepsBuf(af::where(
			m_timestamp > (afLastBodyContactStepBuf + 1))) = 0;
	}
	afStuckParticleBuf = af::array();
	m_bodyContactCount = 0;

	class UpdateBodyContactsCallback : public afFixtureParticleQueryCallback
	{
		void AFReportFixtureAndParticle(
			int32 fixtureIdx, int32 childIndex, const af::array& a)
		{
			af::array apx = m_system->afPosXBuf(a);
			af::array apy = m_system->afPosYBuf(a);
			af::array d;
			af::array nx;
			af::array ny;
			b2Fixture* fixture = m_system->m_fixtureBuffer[fixtureIdx];
			fixture->AFComputeDistance(apx, apy, d, nx, ny, childIndex);
			if (!d.isempty())
			{
				af::array condIdxs = af::where(d < m_system->m_particleDiameter && m_system->AFShouldCollide(a, fixture));
				if (!condIdxs.isempty())
				{
					af::array idxs = a(condIdxs);
					apx = apx(condIdxs);
					apy = apy(condIdxs);
					d = d(condIdxs);
					nx = nx(condIdxs);
					ny = ny(condIdxs);
					b2Body* b = fixture->GetBody();
					int32 bIdx = fixture->GetBodyIdx();
					float32 bpx = b->GetWorldCenter().x;
					float32 bpy = b->GetWorldCenter().y;
					float32 bm = b->GetMass();
					float32 bI =
						b->GetInertia() - bm * b->GetLocalCenter().LengthSquared();
					float32 invBm = bm > 0 ? 1 / bm : 0;
					float32 invBI = bI > 0 ? 1 / bI : 0;
					const af::array& matIdxs = m_system->afPartMatIdxBuf(idxs);
					const af::array& invAm = af::select(m_system->afFlagBuf(idxs) & b2_wallParticle,
						0, m_system->afPartMatInvMassBuf(matIdxs));
					const af::array& rpx = apx - bpx;
					const af::array& rpy = apy - bpy;
					const af::array& rpn = b2Cross(rpx, rpy, nx, ny);
					const af::array& invM = invAm + invBm + invBI * rpn * rpn;

					int32& i = m_system->m_bodyContactCount;
					int32 addCount = idxs.elements();
					m_system->AFJoin(m_system->afBodyContactIdxBuf, idxs);
					m_system->AFJoin(m_system->afBodyContactBodyIdxBuf, af::constant(bIdx, addCount));
					m_system->AFJoin(m_system->afBodyContactFixtIdxBuf, af::constant(fixtureIdx, addCount));
					m_system->AFJoin(m_system->afBodyContactWeightBuf, 1 - d * m_system->m_inverseDiameter);
					m_system->AFJoin(m_system->afBodyContactNormalXBuf, -nx);
					m_system->AFJoin(m_system->afBodyContactNormalYBuf, -ny);
					m_system->AFJoin(m_system->afBodyContactMassBuf, af::select(invM > 0, 1 / invM, 0));
					m_system->m_bodyContactCount += addCount;

					m_system->AFDetectStuckParticle(idxs);
				}
			}
		}

		b2ContactFilter* m_contactFilter;

	public:
		UpdateBodyContactsCallback(
			b2ParticleSystem* system, b2ContactFilter* contactFilter) :
			afFixtureParticleQueryCallback(system)
		{
			m_contactFilter = contactFilter;
		}
	} callback(this, GetFixtureContactFilter());

	b2AABB aabb;							// TODO is this Needed?
	AFComputeAABB(&aabb);
	m_world->AFQueryAABB(&callback, aabb);
	
	if (m_def.strictContactCheck)
	{
		RemoveSpuriousBodyContacts();
	}

	//TODO
	//NotifyBodyContactListenerPostContact(fixtureSet);
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


af::array b2ParticleSystem::AFBodyGetLinVelXFromWorldPoint(const af::array& bodyIdxs, const af::array& y)
{
	return afBodyVelXBuf(bodyIdxs) + b2CrossX(afBodyAngVelBuf(bodyIdxs), y - afBodySweepCY(bodyIdxs));	// y is right
}
af::array b2ParticleSystem::AFBodyGetLinVelYFromWorldPoint(const af::array& bodyIdxs, const af::array& x)
{
	return afBodyVelYBuf(bodyIdxs) + b2CrossY(afBodyAngVelBuf(bodyIdxs), x - afBodySweepCX(bodyIdxs));	// x is right
}


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
		b2Vec2 v = b2Vec2(m_velocityXBuffer[i], m_velocityYBuffer[i]);
		b2Vec2 p1 = b2Vec2(m_positionXBuffer[i], m_positionYBuffer[i]);
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
				b2Vec2 ap = b2Vec2(m_system->m_positionXBuffer[a], m_system->m_positionYBuffer[a]);
				b2Vec2 av = b2Vec2(m_system->m_velocityXBuffer[a], m_system->m_velocityYBuffer[a]);
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
					m_system->m_velocityXBuffer[a] = v.x;
					m_system->m_velocityYBuffer[a] = v.y;
					b2Vec2 f = m_step.inv_dt *
						m_system->m_partMatMassBuf[m_system->m_partMatIdxBuffer[a]] * (av - v);
					m_system->ParticleApplyForce(a, f.x, f.y);
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
void b2ParticleSystem::AFSolveCollision(const b2TimeStep& step)
{
	// This function detects particles which are crossing boundary of bodies
	// and modifies velocities of them so that they will move just in front of
	// boundary. This function function also applies the reaction force to
	// bodies as precisely as the numerical stability is kept.

	const af::array& p1x = afPosXBuf;
	const af::array& p1y = afPosYBuf;
	const af::array& p2x = afPosXBuf + step.dt * afVelXBuf;
	const af::array& p2y = afPosYBuf + step.dt * afVelYBuf;

	b2AABB aabb;
	aabb.lowerBound.x = af::min<float32>(af::min(p1x, p2x));
	aabb.lowerBound.y = af::max<float32>(af::max(p1x, p2x));
	aabb.upperBound.x = af::min<float32>(af::min(p1y, p2y));
	aabb.upperBound.y = af::max<float32>(af::max(p1y, p2y));

	class AFSolveCollisionCallback : public afFixtureParticleQueryCallback
	{
		void AFReportFixtureAndParticle(
			int32 fixtureIdx, int32 childIndex, const af::array& a)
		{
			b2Fixture* fixture = m_system->m_fixtureBuffer[fixtureIdx];
			af::array k = m_system->AFShouldCollide(a, fixture);
			if (!k.isempty()) 
			{
				af::array idxs = a(k);
				b2Body* body = fixture->GetBody();
				const af::array& apx = m_system->afPosXBuf(idxs);
				const af::array& apy = m_system->afPosYBuf(idxs);
				const af::array& avx = m_system->afVelXBuf(idxs);
				const af::array& avy = m_system->afVelYBuf(idxs);
				afRayCastOutput output;
				afRayCastInput input;
				if (m_system->m_iterationIndex == 0)
				{
					// Put 'ap' in the local space of the previous frame
					af::array p1x = b2MulTX(body->m_xf0, apx, apy);
					af::array p1y = b2MulTY(body->m_xf0, apx, apy);
					if (fixture->GetShape()->GetType() == b2Shape::e_circle)
					{
						// Make relative to the center of the circle
						p1x -= body->GetLocalCenter().x;
						p1y -= body->GetLocalCenter().y;
						// Re-apply rotation about the center of the
						// circle
						p1x = b2MulX(body->m_xf0.q, p1x, p1y);
						p1y = b2MulY(body->m_xf0.q, p1x, p1y);
						// Subtract rotation of the current frame
						p1x = b2MulTX(body->m_xf.q, p1x, p1y);
						p1y = b2MulTY(body->m_xf.q, p1x, p1y);
						// Return to local space
						p1x += body->GetLocalCenter().x;
						p1y += body->GetLocalCenter().y;
					}
					// Return to global space and apply rotation of current frame
					input.p1x = b2MulX(body->m_xf, p1x, p1y);
					input.p1y = b2MulY(body->m_xf, p1x, p1y);
				}
				else
				{
					input.p1x = apx;
					input.p1y = apy;
				}
				input.p2x = apx + m_step.dt * avx;
				input.p2y = apy + m_step.dt * avy;
				input.maxFraction = 1;
				af::array k2 = af::where(fixture->AFRayCast(&output, input, childIndex));
				if (!k2.isempty())
				{
					idxs = idxs(k2);
					output.fraction = output.fraction(k2);
					output.normalX = output.normalX(k2);
					output.normalY = output.normalY(k2);
					const af::array& p1x = input.p1x(k2);
					const af::array& p1y = input.p1y(k2);
					const af::array& p2x = input.p2x(k2);
					const af::array& p2y = input.p2y(k2);
					const af::array& nx = output.normalX;
					const af::array& ny = output.normalY;
					const af::array& px = (1 - output.fraction) * p1x +
						output.fraction * p2x +
						b2_linearSlop * nx;
					const af::array& py = (1 - output.fraction) * p1y +
						output.fraction * p2y +
						b2_linearSlop * ny;
					const af::array& vx = m_step.inv_dt * (px - apx);
					const af::array& vy = m_step.inv_dt * (py - apy);
					m_system->afVelXBuf(idxs) = vx;
					m_system->afVelYBuf(idxs) = vy;
					const af::array& matIdxs = m_system->afPartMatIdxBuf(idxs);
					const af::array& mass = m_system->afPartMatMassBuf(matIdxs);
					const af::array& fx = m_step.inv_dt * mass * (avx - vx);
					const af::array& fy = m_step.inv_dt * mass * (avy - vy);
					m_system->AFParticleApplyForce(idxs, fx, fy);
				}
			}
		}

		b2TimeStep m_step;
		b2ContactFilter* m_contactFilter;

	public:
		AFSolveCollisionCallback(
			b2ParticleSystem* system, const b2TimeStep& step, b2ContactFilter* contactFilter) :
			afFixtureParticleQueryCallback(system)
		{
			m_step = step;
			m_contactFilter = contactFilter;
		}
	} callback(this, step, GetFixtureContactFilter());
	m_world->AFQueryAABB(&callback, aabb);
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
			m_velocityXBuffer[i] = 0.0f;
			m_velocityYBuffer[i] = 0.0f;
		}
	}
	float32 tmax = b2_barrierCollisionTime * step.dt;
	for (int32 k = 0; k < m_pairCount; k++)
	{
		if (m_pairFlagsBuf[k] & b2_barrierParticle)
		{
			int32 a = m_pairIdxABuf[k];
			int32 b = m_pairIdxBBuf[k];
			b2Vec2 pa = b2Vec2(m_positionXBuffer[a], m_positionYBuffer[a]);
			b2Vec2 pb = b2Vec2(m_positionXBuffer[b], m_positionYBuffer[b]);
			b2AABB aabb;
			aabb.lowerBound = b2Min(pa, pb);
			aabb.upperBound = b2Max(pa, pb);
			int32 aGroupIdx = m_groupIdxBuffer[a];
			int32 bGroupIdx = m_groupIdxBuffer[b];
			b2Vec2 va = GetLinearVelocity(aGroupIdx, a, pa);
			b2Vec2 vb = GetLinearVelocity(bGroupIdx, b, pb);
			b2Vec2 pba = pb - pa;
			b2Vec2 vba = vb - va;
			InsideBoundsEnumerator enumerator = GetInsideBoundsEnumerator(aabb);
			int32 c;
			while ((c = enumerator.GetNext()) >= 0)
			{
				b2Vec2 pc = b2Vec2(m_positionXBuffer[c], m_positionYBuffer[c]);
				int32 cGroupIdx = m_groupIdxBuffer[c];
				if (aGroupIdx != cGroupIdx && bGroupIdx != cGroupIdx)
				{
					b2Vec2 vc = GetLinearVelocity(cGroupIdx, c, pc);
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
					if (IsRigidGroup(cGroupIdx))
					{
						// If c belongs to a rigid group, the force will be
						// distributed in the group.
						float32 mass = GetGroupMass(cGroupIdx);
						float32 inertia = GetGroupInertia(cGroupIdx);
						if (mass > 0)
						{
							m_groupLinVelXBuf[cGroupIdx] += 1 / mass * f.x;
							m_groupLinVelXBuf[cGroupIdx] += 1 / mass * f.y;
						}
						if (inertia > 0)
							m_groupAngVelBuf[cGroupIdx] +=
								b2Cross(pc - GetGroupCenter(cGroupIdx), f) / inertia;
					}
					else
					{
						m_velocityXBuffer[c] += dv.x;
						m_velocityYBuffer[c] += dv.y;
					}
					// Apply a reversed force to particle c after particle
					// movement so that momentum will be preserved.
					b2Vec2 force = -step.inv_dt * f;
					ParticleApplyForce(c, force.x, force.y);
				}
			}
		}
	}
}



void b2ParticleSystem::AFSolveInit() 
{
	if (!m_world->m_stepComplete || m_count == 0 || m_step.dt <= 0.0f)
		return;

	//CopyBodiesToGPU();
	CopyParticlesToCPU();

	//if (!m_expireTimeBuf.empty())
	//	SolveLifetimes(m_step);
	if (m_allParticleFlags & b2_zombieParticle)
		//AFSolveZombie();
		SolveZombie();
	if (m_needsUpdateAllParticleFlags)
		//AFUpdateAllParticleFlags();
		UpdateAllParticleFlags();
	if (m_needsUpdateAllGroupFlags)
		UpdateAllGroupFlags();
	//AFFilterRealGroups();
	FilterRealGroups();

}

void b2ParticleSystem::AFSolveIterationPart1(int32 iteration)
{
	if (!m_world->m_stepComplete || m_count == 0 || m_step.dt <= 0.0f)
		return;
	if (m_paused)
		return;

	m_iterationIndex = iteration;
	++m_timestamp;
	subStep = m_step;
	subStep.dt /= m_step.particleIterations;
	subStep.inv_dt *= m_step.particleIterations; 

	AFUpdateContacts(false);		// <-- THIS ... 50% of calculation time

}

void b2ParticleSystem::AFSolveIterationPart2()
{
	if (!m_world->m_stepComplete || m_count == 0 || m_step.dt <= 0.0f)
		return;
	if (m_paused)
		return;

}

void b2ParticleSystem::AFSolveIterationPart3()
{
	if (!m_world->m_stepComplete || m_count == 0 || m_step.dt <= 0.0f)
		return;
	if (m_paused)
		return;

	//clock_t clockSolve = clock();

	//AFComputeWeight();
	ComputeWeight();

	//CopyParticlesToCPU();

	if (m_allGroupFlags & b2_particleGroupNeedsUpdateDepth)
		//AFComputeDepth();
		ComputeDepth();
	if (m_allParticleFlags & b2_reactiveParticle)
		//AFUpdatePairsAndTriadsWithReactiveParticles();
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
	//if (m_allParticleFlags & b2_colorMixingParticle)
	//	SolveColorMixing();


	// HEAT MANAGEMENT
	//if (m_world->m_allMaterialFlags & b2_heatConductingMaterial)
	//SolveHeatConduct(subStep);
	//if (m_allParticleFlags & b2_heatLoosingParticle)
	//SolveLooseHeat(subStep);

	// FIRE
	/*if (m_allParticleFlags & b2_flameParticle)
	{
	SolveFlame(subStep);
	if (m_world->m_allMaterialFlags & b2_flammableMaterial)
	SolveIgnite();
	if (m_world->m_allMaterialFlags & b2_extinguishingMaterial)
	SolveExtinguish();
	}
	if (m_allParticleFlags & b2_burningParticle)
	SolveBurning(subStep);*/

	// WATER
	/*if (m_allParticleFlags & b2_waterParticle)
	SolveWater();
	if (m_world->m_allMaterialFlags & b2_boilingMaterial)
	SolveEvaporate();
	SolveFreeze();*/

	//Destroy Dead Particles
	//AFDestroyParticles(af::where(afHealthBuf < 0));
	SolveDestroyDead();

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

	LimitVelocity(subStep);
	//if (m_allGroupFlags & b2_rigidParticleGroup)
	//	SolveRigidDamping();
	//if (m_allParticleFlags & b2_barrierParticle)
	//	SolveBarrier(subStep);


	// SolveCollision, SolveRigid and SolveWall should be called after
	// other force functions because they may require particles to have
	// specific velocities.
	SolveCollision(subStep);

	CopyParticlesToGPU();

	if (m_allGroupFlags & b2_rigidParticleGroup)
		AFSolveRigid(subStep);
	if (m_allParticleFlags & b2_wallParticle)
		AFSolveWall();


	if (m_allParticleFlags & b2_fallingParticle)
		AFSolveFalling(subStep);
	if (m_allParticleFlags & b2_risingParticle)
		AFSolveRising(subStep);

	// The particle positions can be updated only at the end of substep.
	afPosXBuf += subStep.dt * afVelXBuf;
	afPosYBuf += subStep.dt * afVelYBuf;
	//for (int32 i = 0; i < m_count; i++)
	//{
	//	m_positionXBuffer[i] += subStep.dt * m_velocityXBuffer[i];
	//	m_positionYBuffer[i] += subStep.dt * m_velocityYBuffer[i];
	//}
	//cout << "Solve " << (clock() - clockSolve) * 1000 / CLOCKS_PER_SEC << "ms\n";
}

void b2ParticleSystem::AFSolveIterationPart4()
{

}

void b2ParticleSystem::AFSolveIterationPart5()
{
}

void b2ParticleSystem::AFSolveEnd() 
{
	//CopyParticlesToCPU();
	//CopyGroupsToCPU();
}




void b2ParticleSystem::SolveInit()
{
	if (!m_world->m_stepComplete || m_count == 0 || m_step.dt <= 0.0f)
		return;

	if (!m_expireTimeBuf.empty())
		SolveLifetimes(m_step);
	if (m_allParticleFlags & b2_zombieParticle)
		SolveZombie();
	if (m_needsUpdateAllParticleFlags)
		UpdateAllParticleFlags();
	FilterRealGroups();
	if (m_needsUpdateAllGroupFlags)
		UpdateAllGroupFlags();
}

void b2ParticleSystem::SolveIteration(int32 iteration)
{
	if (!m_world->m_stepComplete || m_count == 0 || m_step.dt <= 0.0f)
		return;
	if (m_paused)
		return;

	m_iterationIndex = iteration;
	++m_timestamp;
	subStep = m_step;
	subStep.dt /= m_step.particleIterations;
	subStep.inv_dt *= m_step.particleIterations;
	UpdateContacts(false);		// <-- THIS ... 50% of calculation time
	UpdateBodyContacts();

	ComputeWeight();
	if (!m_world->m_stepComplete || m_count == 0 || m_step.dt <= 0.0f)
		return;
	if (m_paused)
		return;

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

	// HEAT MANAGEMENT
	if (m_world->m_allMaterialFlags & b2_heatConductingMaterial)
		SolveHeatConduct(subStep);
	if (m_allParticleFlags & b2_heatLoosingParticle)
		SolveLooseHeat(subStep);

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
	if (m_world->m_allMaterialFlags & b2_boilingMaterial)
		SolveEvaporate();
	SolveFreeze();

	SolveDestroyDead();

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
	LimitVelocity(subStep);
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

	if (m_allParticleFlags & b2_fallingParticle)
		SolveFalling(subStep);
	if (m_allParticleFlags & b2_risingParticle)
		SolveRising(subStep);

	// The particle positions can be updated only at the end of substep.
	for (int32 i = 0; i < m_count; i++)
	{
		m_positionXBuffer[i] += subStep.dt * m_velocityXBuffer[i];
		m_positionYBuffer[i] += subStep.dt * m_velocityYBuffer[i];
	}
}

void b2ParticleSystem::SolveEnd()
{

}


void b2ParticleSystem::CopyParticlesToGPU()
{
	CPPtoAF(m_count, m_positionXBuffer,			afPosXBuf			);
	CPPtoAF(m_count, m_positionYBuffer,			afPosYBuf			);
	CPPtoAF(m_count, m_positionZBuffer,			afPosZBuf			);
	CPPtoAF(m_count, m_flagsBuffer,				afFlagBuf			);
	CPPtoAF(m_count, m_collisionLayerBuffer,	afColLayBuf			);
	CPPtoAF(m_count, m_velocityXBuffer,			afVelXBuf			);
	CPPtoAF(m_count, m_velocityYBuffer,			afVelYBuf			);
	CPPtoAF(m_count, m_forceXBuffer,			afForceXBuf			);
	CPPtoAF(m_count, m_forceYBuffer,			afForceYBuf			);
	CPPtoAF(m_count, m_weightBuffer,			afWeightBuf			);
	CPPtoAF(m_count, m_heatBuffer,				afHeatBuf			);
	CPPtoAF(m_count, m_healthBuffer,			afHealthBuf			);
	CPPtoAF(m_count, m_colorBuffer,				afColorBuf			);
	CPPtoAF(m_count, m_userDataBuffer,			afUserDataBuf		);
	CPPtoAF(m_count, m_groupIdxBuffer,			afGroupIdxBuf		);
	CPPtoAF(m_count, m_partMatIdxBuffer,		afPartMatIdxBuf		);

	CPPtoAF(m_count, m_expireTimeBuf,			afExpireTimeBuf		);
	CPPtoAF(m_count, m_idxByExpireTimeBuf,		afIdxByExpireTimeBuf);
}
void b2ParticleSystem::CopyParticlesToCPU()
{
	if (m_count >= m_particleBufferSize)
		ResizeParticleBuffers(m_count * 2);
	AFtoCPP(m_count, afFlagBuf,		  m_flagsBuffer			);
	AFtoCPP(m_count, afColLayBuf,	  m_collisionLayerBuffer);	
	AFtoCPP(m_count, afPosXBuf,		  m_positionXBuffer		);
	AFtoCPP(m_count, afPosYBuf,		  m_positionYBuffer		);
	AFtoCPP(m_count, afPosZBuf,		  m_positionZBuffer		);	
	AFtoCPP(m_count, afVelXBuf,		  m_velocityXBuffer		);
	AFtoCPP(m_count, afVelYBuf,		  m_velocityYBuffer		);
	AFtoCPP(m_count, afWeightBuf,	  m_weightBuffer		);
	AFtoCPP(m_count, afHeatBuf,		  m_heatBuffer			);
	AFtoCPP(m_count, afHealthBuf,	  m_healthBuffer		);
	AFtoCPP(m_count, afColorBuf,	  m_colorBuffer			);
	AFtoCPP(m_count, afUserDataBuf,	  m_userDataBuffer		);
	AFtoCPP(m_count, afGroupIdxBuf,   m_groupIdxBuffer		);
	AFtoCPP(m_count, afPartMatIdxBuf, m_partMatIdxBuffer	);

	//if (afHasExpireTimeBuf)
	//	CopyFromAFArrayToVector(m_count, afExpireTimeBuf,	   m_expireTimeBuf);
	//CopyFromAFArrayToVector(m_count, afIdxByExpireTimeBuf, m_idxByExpireTimeBuf);
}

void b2ParticleSystem::CopyProxiesToGPU()
{
	CPPtoAF(m_count, m_proxyIdxBuffer, afProxyIdxBuf);
	CPPtoAF(m_count, m_proxyTagBuffer, afProxyTagBuf);
}
void b2ParticleSystem::CopyProxiesToCPU()
{
	AFtoCPP(m_count, afProxyIdxBuf, m_proxyIdxBuffer);
	AFtoCPP(m_count, afProxyTagBuf, m_proxyTagBuffer);
}

void b2ParticleSystem::CopyGroupsToGPU()
{
	CPPtoAF(m_groupCount, m_groupFirstIdxBuf,		afGroupFirstIdxBuf	);
	CPPtoAF(m_groupCount, m_groupLastIdxBuf,		afGroupLastIdxBuf	);
	CPPtoAF(m_groupCount, m_groupFlagsBuf,			afGroupFlagsBuf		);
	CPPtoAF(m_groupCount, m_groupColGroupBuf,		afGroupColGroupBuf	);
	CPPtoAF(m_groupCount, m_groupStrengthBuf,		afGroupStrengthBuf	);
	CPPtoAF(m_groupCount, m_groupMatIdxBuf,			afGroupMatIdxBuf	);
	CPPtoAF(m_groupCount, m_groupTimestampBuf,      afGroupTimestampBuf	);
	CPPtoAF(m_groupCount, m_groupMassBuf,			afGroupMassBuf		);
	CPPtoAF(m_groupCount, m_groupInertiaBuf,		afGroupInertiaBuf	);
	CPPtoAF(m_groupCount, m_groupCenterXBuf,		afGroupCenterXBuf	);
	CPPtoAF(m_groupCount, m_groupCenterYBuf,		afGroupCenterYBuf	);
	CPPtoAF(m_groupCount, m_groupLinVelXBuf,		afGroupLinVelXBuf	);
	CPPtoAF(m_groupCount, m_groupLinVelYBuf,		afGroupLinVelYBuf	);
	CPPtoAF(m_groupCount, m_groupAngVelBuf,			afGroupAngVelBuf	);
  //CPPtoAF(m_groupCount, m_groupTransformBuf,      afGroupTransformBuf	);
	CPPtoAF(m_groupCount, m_groupUserDataBuf,		afGroupUserDataBuf	);
}
void b2ParticleSystem::CopyGroupsToCPU()
{
	AFtoCPP(m_groupCount, afGroupFirstIdxBuf,	m_groupFirstIdxBuf	);
	AFtoCPP(m_groupCount, afGroupLastIdxBuf,	m_groupLastIdxBuf	);
	AFtoCPP(m_groupCount, afGroupFlagsBuf,		m_groupFlagsBuf		);
	AFtoCPP(m_groupCount, afGroupColGroupBuf,	m_groupColGroupBuf	);
	AFtoCPP(m_groupCount, afGroupStrengthBuf,	m_groupStrengthBuf	);
	AFtoCPP(m_groupCount, afGroupMatIdxBuf,		m_groupMatIdxBuf	);
	AFtoCPP(m_groupCount, afGroupTimestampBuf,	m_groupTimestampBuf	);
	AFtoCPP(m_groupCount, afGroupMassBuf,		m_groupMassBuf		);
	AFtoCPP(m_groupCount, afGroupInertiaBuf,	m_groupInertiaBuf	);
	AFtoCPP(m_groupCount, afGroupCenterXBuf,	m_groupCenterXBuf	);
	AFtoCPP(m_groupCount, afGroupCenterYBuf,	m_groupCenterYBuf	);
	AFtoCPP(m_groupCount, afGroupLinVelXBuf,	m_groupLinVelXBuf	);
	AFtoCPP(m_groupCount, afGroupLinVelYBuf,	m_groupLinVelYBuf	);
	AFtoCPP(m_groupCount, afGroupAngVelBuf,		m_groupAngVelBuf	);
  //AFtoCPP(m_groupCount, afGroupTransformBuf,	m_groupTransformBuf );
	AFtoCPP(m_groupCount, afGroupUserDataBuf,	m_groupUserDataBuf	);
}

void b2ParticleSystem::CopyMatsToGPU()
{
	CPPtoAF(m_partMatCount, m_partMatFlagsBuf,				afPartMatFlagsBuf			);
	CPPtoAF(m_partMatCount, m_partMatMassBuf,				afPartMatMassBuf			);
	CPPtoAF(m_partMatCount, m_partMatInvMassBuf,			afPartMatInvMassBuf			);
	CPPtoAF(m_partMatCount, m_partMatStabilityBuf,			afPartMatStabilityBuf		);
	CPPtoAF(m_partMatCount, m_partMatInvStabilityBuf,		afPartMatInvStabilityBuf	);
	CPPtoAF(m_partMatCount, m_partMatExtinguishingPointBuf,	afPartMatExtgshPntBuf		);
	CPPtoAF(m_partMatCount, m_partMatMeltingPointBuf,		afPartMatMeltingPntBuf		);
	CPPtoAF(m_partMatCount, m_partMatBoilingPointBuf,		afPartMatBoilingPntBuf		);
	CPPtoAF(m_partMatCount, m_partMatIgnitionPointBuf,		afPartMatIgnitPntBuf		);
	CPPtoAF(m_partMatCount, m_partMatHeatConductivityBuf,	afPartMatHeatCondBuf		);
}
void b2ParticleSystem::CopyMatsToCPU()
{
	AFtoCPP(m_partMatCount, afPartMatFlagsBuf,			m_partMatFlagsBuf				);
	AFtoCPP(m_partMatCount, afPartMatMassBuf,			m_partMatMassBuf				);
	AFtoCPP(m_partMatCount, afPartMatInvMassBuf,		m_partMatInvMassBuf				);
	AFtoCPP(m_partMatCount, afPartMatStabilityBuf,		m_partMatStabilityBuf			);
	AFtoCPP(m_partMatCount, afPartMatInvStabilityBuf,	m_partMatInvStabilityBuf		);
	AFtoCPP(m_partMatCount, afPartMatExtgshPntBuf,		m_partMatExtinguishingPointBuf	);
	AFtoCPP(m_partMatCount, afPartMatMeltingPntBuf,		m_partMatMeltingPointBuf		);
	AFtoCPP(m_partMatCount, afPartMatBoilingPntBuf,		m_partMatBoilingPointBuf		);
	AFtoCPP(m_partMatCount, afPartMatIgnitPntBuf,		m_partMatIgnitionPointBuf		);
	AFtoCPP(m_partMatCount, afPartMatHeatCondBuf,		m_partMatHeatConductivityBuf	);
}

void b2ParticleSystem::CopyContactsToGPU()
{
	CPPtoAF(m_contactCount,  m_contactIdxABuffer,		afContactIdxABuf	 );
	CPPtoAF(m_contactCount,  m_contactIdxBBuffer,		afContactIdxBBuf	 );
	CPPtoAF(m_contactCount,  m_contactWeightBuffer,		afContactWeightBuf	 );
	CPPtoAF(m_contactCount,  m_contactMassBuffer,		afContactMassBuf	 );
	CPPtoAF(m_contactCount,  m_contactNormalXBuffer,	afContactNormalXBuf	 );
	CPPtoAF(m_contactCount,  m_contactNormalYBuffer,	afContactNormalYBuf	 );
	CPPtoAF(m_contactCount,  m_contactFlagsBuffer,		afContactFlagsBuf	 );
	CPPtoAF(m_contactCount,  m_contactMatFlagsBuffer,	afContactMatFlagsBuf );

	//vector<af::array> afContactIdxASplits = AFSeperateToUniqueIdxBufs(afContactIdxABuf);
	//vector<af::array> afContactIdxBSplits = AFSeperateToUniqueIdxBufs(afContactIdxBBuf);
}
void b2ParticleSystem::CopyContactsToCPU()
{
	AFtoCPP(m_contactCount, afContactIdxABuf,		m_contactIdxABuffer		);
	AFtoCPP(m_contactCount, afContactIdxBBuf,		m_contactIdxBBuffer		);
	AFtoCPP(m_contactCount, afContactWeightBuf,		m_contactWeightBuffer	);
	AFtoCPP(m_contactCount, afContactMassBuf,		m_contactMassBuffer		);
	AFtoCPP(m_contactCount, afContactNormalXBuf,	m_contactNormalXBuffer	);
	AFtoCPP(m_contactCount, afContactNormalYBuf,	m_contactNormalYBuffer	);
	AFtoCPP(m_contactCount, afContactFlagsBuf,		m_contactFlagsBuffer	);
	AFtoCPP(m_contactCount, afContactMatFlagsBuf,	m_contactMatFlagsBuffer	);
}

void b2ParticleSystem::CopyBodyContactsToGPU()
{
	if (m_bodyContactCount == 0) return;
	CPPtoAF(m_bodyContactCount, m_bodyContactIdxBuffer,			afBodyContactIdxBuf		  );
    CPPtoAF(m_bodyContactCount, m_bodyContactBodyIdxBuffer,		afBodyContactBodyIdxBuf	  );
	CPPtoAF(m_bodyContactCount, m_bodyContactFixtureIdxBuffer,	afBodyContactFixtIdxBuf   );
	CPPtoAF(m_bodyContactCount, m_bodyContactWeightBuffer,		afBodyContactWeightBuf	  );
	CPPtoAF(m_bodyContactCount, m_bodyContactNormalXBuffer,		afBodyContactNormalXBuf	  );
	CPPtoAF(m_bodyContactCount, m_bodyContactNormalYBuffer,		afBodyContactNormalYBuf	  );
	CPPtoAF(m_bodyContactCount, m_bodyContactMassBuffer,		afBodyContactMassBuf	  );
}
void b2ParticleSystem::CopyBodyContactsToCPU()
{
	AFtoCPP(m_bodyContactCount, afBodyContactIdxBuf,		m_bodyContactIdxBuffer			);
    AFtoCPP(m_bodyContactCount, afBodyContactBodyIdxBuf,	m_bodyContactBodyIdxBuffer		);
    AFtoCPP(m_bodyContactCount, afBodyContactFixtIdxBuf,	m_bodyContactFixtureIdxBuffer	);
	AFtoCPP(m_bodyContactCount, afBodyContactWeightBuf,		m_bodyContactWeightBuffer		);
	AFtoCPP(m_bodyContactCount, afBodyContactNormalXBuf,	m_bodyContactNormalXBuffer		);
	AFtoCPP(m_bodyContactCount, afBodyContactNormalYBuf,	m_bodyContactNormalYBuffer		);
	AFtoCPP(m_bodyContactCount, afBodyContactMassBuf,		m_bodyContactMassBuffer			);
}

void b2ParticleSystem::CopyBodiesToGPU()
{
	m_bodyCount = m_bodyBuffer.size();

	vector<float32> velX		(m_bodyCount);
	vector<float32> velY		(m_bodyCount);
	vector<float32> angVel		(m_bodyCount);
	vector<float32> bodySweepCX	(m_bodyCount);
	vector<float32> bodySweepCY	(m_bodyCount);

	for (int i = 0; i < m_bodyCount; i++)
	{
		const b2Body* b = m_bodyBuffer[i];
		if (b != NULL)
		{
			velX[i]			= b->m_linearVelocity.x;
			velY[i]			= b->m_linearVelocity.y;
			angVel[i]		= b->m_angularVelocity;
			bodySweepCX[i]	= b->m_sweep.c.x;
			bodySweepCY[i]	= b->m_sweep.c.y;
		}
	}
	CPPtoAF(m_bodyCount, velX,			afBodyVelXBuf	);
	CPPtoAF(m_bodyCount, velY,			afBodyVelYBuf	);
	CPPtoAF(m_bodyCount, angVel,		afBodyAngVelBuf	);
	CPPtoAF(m_bodyCount, bodySweepCX,	afBodySweepCX	);
	CPPtoAF(m_bodyCount, bodySweepCY,	afBodySweepCY	);
}

template <class T> 
inline void b2ParticleSystem::AFtoCPP(const int32& count, const af::array& arr, vector<T>& vec)
{
	T* buf = arr.host<T>();
	vec = vector<T>(buf, buf + count);
	//copy(arr, arr + count, vec.begin());
}
template <class T>
inline void b2ParticleSystem::CPPtoAF(const int32& count, const vector<T>& vec, af::array& arr)
{
	arr = af::array(count, vec.data(), af::source::afHost);
}
template <class T>
inline void b2ParticleSystem::CPPtoAF(const int32& count, const T* vec, af::array& arr)
{
	arr = af::array(count, vec, af::source::afHost);
}

vector<af::array> b2ParticleSystem::AFSeperateToUniqueIdxBufs(const af::array& a)
{
	int count = a.elements();
	af::array aSorted;
	af::array origIdxs = af::seq(count);
	//af::sort(aSorted, origIdxs, a);

	vector<af::array> v;
	for (int i = 0; i < 10; i++)
	{
		const af::array& unique = aSorted != af::shift(aSorted, 1);
		const af::array& uniqueOrigIdxs = origIdxs(af::where(unique));
		if (!uniqueOrigIdxs.isempty())
			v.push_back(uniqueOrigIdxs);
		else 
			break;
		const af::array& restIdxs = af::where(!unique);
		if (restIdxs.isempty()) break;
		v.push_back(origIdxs(restIdxs));
		count = restIdxs.elements();
		aSorted = aSorted(restIdxs);
		origIdxs = origIdxs(restIdxs);
	}
	return v;
}

void b2ParticleSystem::SolveFalling(const b2TimeStep& step)
{
	for (int32 k = 0; k < m_count; k++)
	{
		if (m_flagsBuffer[k] & b2_fallingParticle)
		{
			float32 z = m_positionZBuffer[k] - step.dt;
			if (z <= 0)
			{
				z = 0;
				m_flagsBuffer[k] &= ~b2_fallingParticle;
			}
			m_positionZBuffer[k] = z;
			m_collisionLayerBuffer[k] = 1 << (int)(z * m_invPointsPerLayer + m_layerGraphB);
		}
	}
}
void b2ParticleSystem::AFSolveFalling(const b2TimeStep& step)
{
	const af::array& k = af::where(afFlagBuf & b2_fallingParticle);
	if (!k.isempty())
	{
		af::array z = afPosZBuf(k) - step.dt;
		const af::array& k2 = af::where(z <= 0);
		if (!k2.isempty())
		{
			z(k2) = 0;
			const af::array& newK = k(k2);
			afFlagBuf(newK) = afFlagBuf(newK) & ~b2_fallingParticle;
		}
		afPosZBuf(k) = z;
		afColLayBuf(k) = 1 << (z * m_invPointsPerLayer + m_layerGraphB).as(af::dtype::s32);
	}
}

void b2ParticleSystem::SolveRising(const b2TimeStep& step)
{
	for (int32 k = 0; k < m_count; k++)
	{
		if (m_flagsBuffer[k] & b2_risingParticle)
		{
			float32 z = m_collisionLayerBuffer[k] + step.dt;
			m_positionZBuffer[k] = z;
			m_collisionLayerBuffer[k] = 1 << (int)(z * m_invPointsPerLayer + m_layerGraphB);
		}
	}
}
void b2ParticleSystem::AFSolveRising(const b2TimeStep& step)
{
	const af::array& k = af::where(afFlagBuf & b2_risingParticle);
	if (!k.isempty())
	{
		const af::array& z = afColLayBuf(k) + step.dt;
		afPosZBuf(k) = z;
		afColLayBuf(k) = 1 << (z * m_invPointsPerLayer + m_layerGraphB).as(af::dtype::s32);
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
void b2ParticleSystem::AFUpdateAllParticleFlags()
{
	//TODO ...
	m_allParticleFlags = 0;

	for (int32 i = 0; i < 32; i++)
		m_allParticleFlags |= !af::where(afFlagBuf & (1 << i)).isempty() << i;

	m_needsUpdateAllParticleFlags = false;
}

void b2ParticleSystem::UpdateAllGroupFlags()
{
	m_allGroupFlags = 0;
	for each (const int32 groupIdx in m_realGroupIdxBuffer)
	{
		m_allGroupFlags |= m_groupFlagsBuf[groupIdx];
	}
	m_needsUpdateAllGroupFlags = false;
}

void b2ParticleSystem::LimitVelocity(const b2TimeStep& step)
{
	float32 criticalVelocitySquared = GetCriticalVelocitySquared(step);
	for (int32 i = 0; i < m_count; i++)
	{
		b2Vec2 v = b2Vec2(m_velocityXBuffer[i], m_velocityYBuffer[i]);
		float32 v2 = b2Dot(v, v);
		if (v2 > criticalVelocitySquared)
		{
			int32 s = b2Sqrt(criticalVelocitySquared / v2);
			m_velocityXBuffer[i] *= s;
			m_velocityYBuffer[i] *= s;

		}
	}
}
void b2ParticleSystem::AFLimitVelocity(const b2TimeStep& step)
{
	float32 criticalVelocitySquared = GetCriticalVelocitySquared(step);
	const af::array& v2 = b2Dot(afVelXBuf, afVelYBuf, afVelXBuf, afVelYBuf);
	const af::array& k = af::where(v2 > criticalVelocitySquared);
	if (!k.isempty())
	{
		const af::array& s = af::sqrt(criticalVelocitySquared / v2(k));
		afVelXBuf(k) *= s;
		afVelYBuf(k) *= s;
	}
}

void b2ParticleSystem::SolveGravity(const b2TimeStep& step)
{
	b2Vec2 gravity = step.dt * m_def.gravityScale * m_world->GetGravity();
	int32 gravityX = gravity.x;
	int32 gravityY = gravity.y;
	for (int32 i = 0; i < m_count; i++)
	{
		m_velocityXBuffer[i] += gravityX;
		m_velocityYBuffer[i] += gravityY;
	}
}
void b2ParticleSystem::AFSolveGravity(const b2TimeStep& step)
{
	float32 gravX = step.dt * m_def.gravityScale * m_world->m_gravity.x;
	float32 gravY = step.dt * m_def.gravityScale * m_world->m_gravity.x;
	afVelXBuf += gravX;
	afVelYBuf += gravY;
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
			if (m_contactFlagsBuffer[k] & b2_staticPressureParticle)
			{
				int32 a = m_contactIdxABuffer[k];
				int32 b = m_contactIdxBBuffer[k];
				float32 w = m_contactWeightBuffer[k];
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
void b2ParticleSystem::AFSolveStaticPressure(const b2TimeStep& step)
{
	AFRequestBuffer(afStaticPressureBuf, afHasStaticPresBuf);
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
		afAccumulationBuf = af::constant(0, m_count);

		const af::array& k = af::where(afContactFlagsBuf & b2_staticPressureParticle);
		if (!k.isempty())
		{
			const af::array& a = afContactIdxABuf(k);
			const af::array& b = afContactIdxBBuf(k);
			const af::array& w = afContactWeightBuf(k);
			afAccumulationBuf(a) += w * afStaticPressureBuf(b);
			afAccumulationBuf(b) += w * afStaticPressureBuf(a);
			
		}

		afStaticPressureBuf = af::constant(0, m_count);
		const af::array& k2 = af::where(afFlagBuf & b2_staticPressureParticle);
		if (!k2.isempty())
		{
			const af::array& w = afWeightBuf(k2);
			const af::array& wh = afAccumulationBuf(k2);
			const af::array& h = (wh + pressurePerWeight * (w - b2_minParticleWeight)) /
								 (w + relaxation);
			afStaticPressureBuf(k2) = af::max(0.0f, af::min(h, maxPressure));
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
		int32 a = m_bodyContactIdxBuffer[k];
		int32 bIdx = m_bodyContactBodyIdxBuffer[k];
		float32 w = m_bodyContactWeightBuffer[k];
		float32 m = m_bodyContactMassBuffer[k];
		b2Vec2 n = b2Vec2(m_bodyContactNormalXBuffer[k], m_bodyContactNormalYBuffer[k]);
		b2Vec2 p = b2Vec2(m_positionXBuffer[a], m_positionYBuffer[a]);
		float32 h = m_accumulationBuf[a] + pressurePerWeight * w;
		b2Vec2 f = velocityPerPressure * w * m * h * n;
		float32 invMass = m_partMatInvMassBuf[m_partMatIdxBuffer[a]];
		m_velocityXBuffer[a] -= invMass * f.x;
		m_velocityYBuffer[a] -= invMass * f.y;
		m_bodyBuffer[bIdx]->ApplyLinearImpulse(f, p, true);
	}
	for (int32 k = 0; k < m_contactCount; k++)
	{
		int32 a = m_contactIdxABuffer[k];
		int32 b = m_contactIdxBBuffer[k];
		float32 w = m_contactWeightBuffer[k];
		float32 m = m_contactMassBuffer[k];
		b2Vec2 n = b2Vec2(m_contactNormalXBuffer[k], m_contactNormalYBuffer[k]);
		float32 h = m_accumulationBuf[a] + m_accumulationBuf[b];
		b2Vec2 f = velocityPerPressure * w * m * h * n;
		float32 massA = m_partMatMassBuf[m_partMatIdxBuffer[a]];
		float32 massB = m_partMatMassBuf[m_partMatIdxBuffer[b]];
		m_velocityXBuffer[a] -= massB * f.x;
		m_velocityYBuffer[a] -= massB * f.y;
		m_velocityXBuffer[b] += massA * f.x;
		m_velocityYBuffer[b] += massA * f.y;
	}
}
void b2ParticleSystem::AFSolvePressure(const b2TimeStep& step)
{
	// calculates pressure as a linear function of density
	float32 criticalPressure = GetCriticalPressure(step);
	float32 pressurePerWeight = m_def.pressureStrength * criticalPressure;
	float32 maxPressure = b2_maxParticlePressure * criticalPressure;

	const af::array& h = pressurePerWeight * af::max(0.0f, afWeightBuf - b2_minParticleWeight);
	afAccumulationBuf = af::min(h, maxPressure);

	// ignores particles which have their own repulsive force
	if (m_allParticleFlags & k_noPressureFlags)
		afAccumulationBuf(af::where(afFlagBuf & k_noPressureFlags)) = 0;

	// static pressure
	if (m_allParticleFlags & b2_staticPressureParticle)
	{
		b2Assert(!afStaticPressureBuf.isempty());
		const af::array& k = af::where(afFlagBuf & b2_staticPressureParticle);
		if (!k.isempty())
			afAccumulationBuf(k) += afStaticPressureBuf(k);
	}
	// applies pressure between each particles in contact
	float32 velocityPerPressure = step.dt / (m_def.density * m_particleDiameter);

	// Bodies
	/*do
	{
		const af::array& a = afBodyContactIdxBuf;
		const af::array& bIdx = afBodyContactBodyIdxBuf;
		const af::array& w = afBodyContactWeightBuf;
		const af::array& m = afBodyContactMassBuf;
		const af::array& nx = afBodyContactNormalXBuf;
		const af::array& ny = afBodyContactNormalYBuf;
		const af::array& px = afPosXBuf(a);
		const af::array& py = afPosYBuf(a);
		const af::array& h = afAccumulationBuf(a) + pressurePerWeight * w;
		const af::array& fx = velocityPerPressure * w * m * h * nx;
		const af::array& fy = velocityPerPressure * w * m * h * ny;
		const af::array& invMass = afPartMatInvMassBuf((af::array)afPartMatIdxBuf(a));
		afVelXBuf(a) -= invMass * fx;
		afVelYBuf(a) -= invMass * fy;
		//b->ApplyLinearImpulse(f, p, true);	//TODO difficult
	} while (false);*/

	// Particles
	const af::array& ca = afContactIdxABuf;
	const af::array& cb = afContactIdxBBuf;
	const af::array& cw = afContactWeightBuf;
	const af::array& cm = afContactMassBuf;
	const af::array& nx = afContactNormalXBuf;
	const af::array& ny = afContactNormalYBuf;
	const af::array& ch = afAccumulationBuf(ca) + afAccumulationBuf(cb);
	const af::array& fx = velocityPerPressure * cw * cm * ch * nx;
	const af::array& fy = velocityPerPressure * cw * cm * ch * ny;
	const af::array& matAIdxs = afPartMatIdxBuf(ca);
	const af::array& matBIdxs = afPartMatIdxBuf(cb);
	const af::array& massA = afPartMatMassBuf(matAIdxs);
	const af::array& massB = afPartMatMassBuf(matBIdxs);
	//cout << "SolvePressure\n";
	//cout << "Changing Vel of " << (massB * fx).elements() << "\n";
	//cout << "Example: " << afVelXBuf(ca).scalar<float32>() << " -= "  << (massB * fx).scalar<float32>() << "\n";
	afVelXBuf(ca) -= massB * fx;
	afVelYBuf(ca) -= massB * fy;
	afVelXBuf(cb) += massA * fx;
	afVelYBuf(cb) += massA * fy;
}

void b2ParticleSystem::SolveDamping(const b2TimeStep& step)
{
	// reduces normal velocity of each contact
	float32 linearDamping = m_def.dampingStrength;
	float32 quadraticDamping = 1 / GetCriticalVelocity(step);
	for (int32 k = 0; k < m_bodyContactCount; k++)
	{
		int32 a = m_bodyContactIdxBuffer[k];
		int32 bIdx = m_bodyContactBodyIdxBuffer[k];
		float32 w = m_bodyContactWeightBuffer[k];
		float32 m = m_bodyContactMassBuffer[k];
		b2Vec2 n = b2Vec2(m_bodyContactNormalXBuffer[k], m_bodyContactNormalYBuffer[k]);
		b2Vec2 p = b2Vec2(m_positionXBuffer[a], m_positionYBuffer[a]);
		b2Vec2 v = m_bodyBuffer[bIdx]->GetLinearVelocityFromWorldPoint(p) -
				   b2Vec2(m_velocityXBuffer[a], m_velocityYBuffer[a]);
		float32 vn = b2Dot(v, n);
		if (vn < 0)
		{
			float32 damping =
				b2Max(linearDamping * w, b2Min(- quadraticDamping * vn, 0.5f));
			b2Vec2 f = damping * m * vn * n;
			float32 invMass = m_partMatInvMassBuf[m_partMatIdxBuffer[a]];
			m_velocityXBuffer[a] += invMass * f.x;
			m_velocityYBuffer[a] += invMass * f.y;
			m_bodyBuffer[bIdx]->ApplyLinearImpulse(-f, p, true);
		}
	}
	for (int32 k = 0; k < m_contactCount; k++)
	{
		int32 a = m_contactIdxABuffer[k];
		int32 b = m_contactIdxBBuffer[k];
		float32 w = m_contactWeightBuffer[k];
		float32 m = m_contactMassBuffer[k];
		b2Vec2 n = b2Vec2(m_contactNormalXBuffer[k], m_contactNormalYBuffer[k]);
		b2Vec2 v = b2Vec2(m_velocityXBuffer[b] - m_velocityXBuffer[a], m_velocityYBuffer[b] - m_velocityYBuffer[a]);
		float32 vn = b2Dot(v, n);
		if (vn < 0)
		{
			float32 damping =
				b2Max(linearDamping * w, b2Min(- quadraticDamping * vn, 0.5f));
			b2Vec2 f = damping * m * vn * n;
			float32 massA = m_partMatMassBuf[m_partMatIdxBuffer[a]];
			float32 massB = m_partMatMassBuf[m_partMatIdxBuffer[b]];
			m_velocityXBuffer[a] += massB * f.x;
			m_velocityYBuffer[a] += massB * f.y;
			m_velocityXBuffer[b] -= massA * f.x;
			m_velocityYBuffer[b] -= massA * f.y;
		}
	}
}
void b2ParticleSystem::AFSolveDamping(const b2TimeStep& step)
{
	// reduces normal velocity of each contact
	float32 linearDamping = m_def.dampingStrength;
	float32 quadraticDamping = 1 / GetCriticalVelocity(step);

	/*do
	{
		af::array a = afBodyContactIdxBuf;
		const af::array& bIdx = afBodyContactBodyIdxBuf;
		af::array w = afBodyContactWeightBuf;
		af::array m = afBodyContactMassBuf;
		af::array nx = afBodyContactNormalXBuf;
		af::array ny = afBodyContactNormalYBuf;
		const af::array& px = afPosXBuf(a);
		const af::array& py = afPosYBuf(a);
		const af::array& vx = AFBodyGetLinVelXFromWorldPoint(bIdx, py) - afVelXBuf(a);
		const af::array& vy = AFBodyGetLinVelYFromWorldPoint(bIdx, px) - afVelYBuf(a);
		af::array vn = b2Dot(vx, vy, nx, ny);
		const af::array& k = af::where(vn < 0);
		if (!k.isempty())
		{
			a = a(k);
			w = w(k);
			m = m(k);
			nx = nx(k);
			ny = ny(k);
			vn = vn(k);
			const af::array& damping =
				af::max(linearDamping * w, af::min(-quadraticDamping * vn, 0.5f));
			const af::array& fx = damping * m * vn * nx;
			const af::array& fy = damping * m * vn * ny;
			const af::array& invMass = afPartMatInvMassBuf((af::array)afPartMatIdxBuf(a));
			afVelXBuf(a) += invMass * fx;
			afVelYBuf(a) += invMass * fy;
			//m_bodyBuffer[bIdx]->ApplyLinearImpulse(-f, p, true);	//TODO Difficult
		}
	} while (false);*/
	
	af::array a = afContactIdxABuf;
	af::array b = afContactIdxBBuf;
	af::array w = afContactWeightBuf;
	af::array m = afContactMassBuf;
	af::array nx = afContactNormalXBuf;
	af::array ny = afContactNormalYBuf;
	const af::array& vx = afVelXBuf(b) - afVelXBuf(a);
	const af::array& vy = afVelYBuf(b) - afVelYBuf(a);
	af::array vn = b2Dot(vx, vy, nx, ny);
	const af::array& k = af::where(vn < 0);
	if (!k.isempty())
	{
		a = a(k);
		b = b(k);
		w = w(k);
		m = m(k);
		nx = nx(k);
		ny = ny(k);
		vn = vn(k);
		const af::array& damping =
			af::max(linearDamping * w, af::min(-quadraticDamping * vn, 0.5f));
		const af::array& fx = damping * m * vn * nx;
		const af::array& fy = damping * m * vn * ny;
		const af::array& massA = afPartMatMassBuf((af::array)afPartMatIdxBuf(a));
		const af::array& massB = afPartMatMassBuf((af::array)afPartMatIdxBuf(b));
		afVelXBuf(a) += massB * fx;
		afVelYBuf(a) += massB * fy;
		afVelXBuf(b) -= massA * fx;
		afVelYBuf(b) -= massA * fy;
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
			float slowDown = (m_collisionLayerBuffer[k] & (b2_Layer1ground | b2_Layer0underGround)) ? dd : d;
			m_velocityXBuffer[k] *= slowDown;
			m_velocityYBuffer[k] *= slowDown;
		}
	}
}
void b2ParticleSystem::AFSolveSlowDown(const b2TimeStep& step)
{
	if (m_def.dampingStrength > 0)
	{
		float32 d = 1 - (step.dt * m_def.dampingStrength);
		afVelXBuf *= d;
		afVelYBuf *= d;
		af::array k = af::where(afColLayBuf & (b2_Layer1ground | b2_Layer0underGround));
		if (!k.isempty())
		{
			afVelXBuf(k) *= d;
			afVelYBuf(k) *= d;
		}
	}
}

inline bool b2ParticleSystem::IsRigidGroup(int32 groupIdx) const
{
	return groupIdx && (m_groupFlagsBuf[groupIdx] & b2_rigidParticleGroup);
}

inline b2Vec2 b2ParticleSystem::GetLinearVelocity(
	int32 groupIdx, int32 particleIndex,
	const b2Vec2 &point)
{
	if (IsRigidGroup(groupIdx))
	{
		return GetGroupLinearVelocityFromWorldPoint(groupIdx, point);
	}
	else
	{
		return b2Vec2(m_velocityXBuffer[particleIndex], m_velocityYBuffer[particleIndex]);
	}
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
	bool isRigidGroup, int32 groupIdx, int32 particleIndex,
	const b2Vec2& point, const b2Vec2& normal)
{
	if (isRigidGroup)
	{
		InitDampingParameter(
			invMass, invInertia, tangentDistance,
			GetGroupMass(groupIdx), GetGroupInertia(groupIdx), GetGroupCenter(groupIdx),
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
	bool isRigidGroup, int32 groupIdx, int32 particleIndex,
	float32 impulse, float32 normalX, float32 normalY)
{
	if (isRigidGroup)
	{
		m_groupLinVelXBuf[groupIdx] += impulse * invMass * normalX;
		m_groupLinVelYBuf[groupIdx] += impulse * invMass * normalY;
		m_groupAngVelBuf[groupIdx] += impulse * tangentDistance * invInertia;
	}
	else
	{
		m_velocityXBuffer[particleIndex] += impulse * invMass * normalX;
		m_velocityYBuffer[particleIndex] += impulse * invMass * normalY;
	}
}

void b2ParticleSystem::SolveRigidDamping()
{
	// Apply impulse to rigid particle groups colliding with other objects
	// to reduce relative velocity at the colliding point.
	float32 damping = m_def.dampingStrength;
	for (int32 k = 0; k < m_bodyContactCount; k++)
	{
		int32 a = m_bodyContactIdxBuffer[k];
		int32 aGroupIdx = m_groupIdxBuffer[a];
		if (IsRigidGroup(aGroupIdx))
		{
			int32 bIdx = m_bodyContactBodyIdxBuffer[k];
			b2Body* b = m_bodyBuffer[bIdx];
			b2Vec2 n = b2Vec2(m_bodyContactNormalXBuffer[k], m_bodyContactNormalYBuffer[k]);
			float32 w = m_bodyContactWeightBuffer[k];
			b2Vec2 p = b2Vec2(m_positionXBuffer[a], m_positionYBuffer[a]);
			b2Vec2 v = b->GetLinearVelocityFromWorldPoint(p) -
				GetGroupLinearVelocityFromWorldPoint(aGroupIdx, p);
			float32 vn = b2Dot(v, n);
			if (vn < 0)
				// The group's average velocity at particle position 'p' is pushing
				// the particle into the body.
			{
				float32 invMassA, invInertiaA, tangentDistanceA;
				float32 invMassB, invInertiaB, tangentDistanceB;
				InitDampingParameterWithRigidGroupOrParticle(
					&invMassA, &invInertiaA, &tangentDistanceA,
					true, aGroupIdx, a, p, n);
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
					true, aGroupIdx, a, f, n.x, n.y);
				b->ApplyLinearImpulse(-f * n, p, true);
			}
		}
	}
	for (int32 k = 0; k < m_contactCount; k++)
	{
		int32 a = m_contactIdxABuffer[k];
		int32 b = m_contactIdxBBuffer[k];
		b2Vec2 n = b2Vec2(m_contactNormalXBuffer[k], m_contactNormalYBuffer[k]);
		float32 w = m_contactWeightBuffer[k];
		int32 aGroupIdx = m_groupIdxBuffer[a];
		int32 bGroupIdx = m_groupIdxBuffer[b];
		bool aRigid = IsRigidGroup(aGroupIdx);
		bool bRigid = IsRigidGroup(bGroupIdx);
		if (aGroupIdx != bGroupIdx && (aRigid || bRigid))
		{
			b2Vec2 p =
				0.5f * b2Vec2(m_positionXBuffer[a] + m_positionXBuffer[b], m_positionYBuffer[a] + m_positionYBuffer[b]);
			b2Vec2 v =
				GetLinearVelocity(bGroupIdx, b, p) -
				GetLinearVelocity(aGroupIdx, a, p);
			float32 vn = b2Dot(v, n);
			if (vn < 0)
			{
				float32 invMassA, invInertiaA, tangentDistanceA;
				float32 invMassB, invInertiaB, tangentDistanceB;
				InitDampingParameterWithRigidGroupOrParticle(
					&invMassA, &invInertiaA, &tangentDistanceA,
					aRigid, aGroupIdx, a,
					p, n);
				InitDampingParameterWithRigidGroupOrParticle(
					&invMassB, &invInertiaB, &tangentDistanceB,
					bRigid, bGroupIdx, b,
					p, n);
				float32 f = damping * w * ComputeDampingImpulse(
					invMassA, invInertiaA, tangentDistanceA,
					invMassB, invInertiaB, tangentDistanceB,
					vn);
				ApplyDamping(
					invMassA, invInertiaA, tangentDistanceA,
					aRigid, aGroupIdx, a, f, n.x, n.y);
				ApplyDamping(
					invMassB, invInertiaB, tangentDistanceB,
					bRigid, bGroupIdx, b, -f, n.x, n.y);
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
		int32 a = m_bodyContactIdxBuffer[k];
		if (m_flagsBuffer[a] & k_extraDampingFlags)
		{
			int32 bIdx = m_bodyContactBodyIdxBuffer[k];
			float32 m = m_bodyContactMassBuffer[k];
			b2Vec2 n = b2Vec2(m_bodyContactNormalXBuffer[k], m_bodyContactNormalYBuffer[k]);
			b2Vec2 p = b2Vec2(m_positionXBuffer[a], m_positionYBuffer[a]);
			b2Vec2 v =
				m_bodyBuffer[bIdx]->GetLinearVelocityFromWorldPoint(p) -
				b2Vec2(m_velocityXBuffer[a], m_velocityYBuffer[a]);
			float32 vn = b2Dot(v, n);
			if (vn < 0)
			{
				b2Vec2 f = 0.5f * m * vn * n;
				//m_velocityBuffer.data[a] += GetParticleInvMass() * f;
				float32 invMass = m_partMatInvMassBuf[m_partMatIdxBuffer[a]];
				m_velocityXBuffer[a] += invMass * f.x;
				m_velocityYBuffer[a] += invMass * f.y;
				m_bodyBuffer[bIdx]->ApplyLinearImpulse(-f, p, true);
			}
		}
	}
}
void b2ParticleSystem::AFSolveExtraDamping()
{
	// Applies additional damping force between bodies and particles which can
	// produce strong repulsive force. Applying damping force multiple times
	// is effective in suppressing vibration.
	const af::array& k = af::where(afFlagBuf(afBodyContactIdxBuf) & k_extraDampingFlags);
	if (!k.isempty())
	{
		af::array a = afBodyContactIdxBuf(k);
		af::array bIdx = afBodyContactBodyIdxBuf(k);
		af::array m = afBodyContactMassBuf(k);
		af::array nx = afBodyContactNormalXBuf(k);
		af::array ny = afBodyContactNormalYBuf(k);
		const af::array& px = afPosXBuf(a);
		const af::array& py = afPosYBuf(a);
		const af::array& vx = AFBodyGetLinVelXFromWorldPoint(bIdx, py) - afVelXBuf(a);
		const af::array& vy = AFBodyGetLinVelYFromWorldPoint(bIdx, px) - afVelYBuf(a);
		af::array vn = b2Dot(vx, vy, nx, ny);
		const af::array& k2 = af::where(vn < 0);
		if (!k2.isempty())
		{
			a = a(k2);
			bIdx = bIdx(k2);
			m = m(k2);
			nx = nx(k2);
			ny = ny(k2);
			vn = vn(k2);
			const af::array& fx = 0.5f * m * vn * nx;
			const af::array& fy = 0.5f * m * vn * ny;
			const af::array& invMass = afPartMatInvMassBuf((af::array)afPartMatIdxBuf(a));
			afVelXBuf(a) += invMass * fx;
			afVelYBuf(a) += invMass * fy;
			//m_bodyBuffer[bIdx]->ApplyLinearImpulse(-f, p, true);	// TODO difficult :/
		}
	}
}

void b2ParticleSystem::SolveWall()
{
	for (int32 i = 0; i < m_count; i++)
	{
		if (m_flagsBuffer[i] & b2_wallParticle)
		{
			m_velocityXBuffer[i] = 0.0f;
			m_velocityYBuffer[i] = 0.0f;
		}
	}
}
void b2ParticleSystem::AFSolveWall()
{
	const af::array& k = af::where(afFlagBuf & b2_wallParticle);
	if (!k.isempty())
	{
		afVelXBuf(k) = 0.0f;
		afVelYBuf(k) = 0.0f;
	}
}

void b2ParticleSystem::SolveRigid(const b2TimeStep& step)
{
	for each (const int32 groupIdx in m_realGroupIdxBuffer)
	{
		if (m_groupFlagsBuf[groupIdx] & b2_rigidParticleGroup)
		{
			UpdateGroupStatistics(groupIdx);
			b2Rot rotation(step.dt * m_groupAngVelBuf[groupIdx]);
			b2Vec2 center = b2Vec2(m_groupCenterXBuf[groupIdx], m_groupCenterYBuf[groupIdx]);
			b2Vec2 linVel = b2Vec2(m_groupLinVelXBuf[groupIdx], m_groupLinVelYBuf[groupIdx]);
			b2Transform transform(center + step.dt * linVel -
				b2Mul(rotation, center), rotation);
			m_groupTransformBuf[groupIdx] = b2Mul(transform, m_groupTransformBuf[groupIdx]);
			b2Transform velocityTransform;
			velocityTransform.p.x = step.inv_dt * transform.p.x;
			velocityTransform.p.y = step.inv_dt * transform.p.y;
			velocityTransform.q.s = step.inv_dt * transform.q.s;
			velocityTransform.q.c = step.inv_dt * (transform.q.c - 1);
			for (int32 i = m_groupFirstIdxBuf[groupIdx]; i < m_groupLastIdxBuf[groupIdx]; i++)
			{
				b2Vec2 vel = b2Mul(velocityTransform,
					b2Vec2(m_positionXBuffer[i], m_positionYBuffer[i]));
				m_velocityXBuffer[i] = vel.x;
				m_velocityYBuffer[i] = vel.y;
			}
		}
	}
}
void b2ParticleSystem::AFSolveRigid(const b2TimeStep& step)
{
	for each (const int32 groupIdx in m_realGroupIdxBuffer)
	{
		if (m_groupFlagsBuf[groupIdx] & b2_rigidParticleGroup)
		{
			UpdateGroupStatistics(groupIdx);
			b2Rot rotation(step.dt * m_groupAngVelBuf[groupIdx]);
			b2Vec2 center = b2Vec2(m_groupCenterXBuf[groupIdx], m_groupCenterYBuf[groupIdx]);
			b2Vec2 linVel = b2Vec2(m_groupLinVelXBuf[groupIdx], m_groupLinVelYBuf[groupIdx]);
			b2Transform transform(center + step.dt * linVel -
				b2Mul(rotation, center), rotation);
			m_groupTransformBuf[groupIdx] = b2Mul(transform, m_groupTransformBuf[groupIdx]);
			b2Transform velocityTransform;
			velocityTransform.p.x = step.inv_dt * transform.p.x;
			velocityTransform.p.y = step.inv_dt * transform.p.y;
			velocityTransform.q.s = step.inv_dt * transform.q.s;
			velocityTransform.q.c = step.inv_dt * (transform.q.c - 1);

			af::array idxs = af::seq(m_groupFirstIdxBuf[groupIdx], m_groupLastIdxBuf[groupIdx]);
			afVelXBuf(idxs) = b2MulX(velocityTransform, afPosXBuf, afPosYBuf)(idxs);
			afVelYBuf(idxs) = b2MulY(velocityTransform, afPosXBuf, afPosYBuf)(idxs);
		}
	}
}

void b2ParticleSystem::SolveElastic(const b2TimeStep& step)
{
	float32 elasticStrength = step.inv_dt * m_def.elasticStrength;
	for (int32 k = 0; k < m_triadCount; k++)
	{
		if (m_triadFlagsBuf[k] & b2_elasticParticle)
		{
			int32 a = m_triadIdxABuf[k];
			int32 b = m_triadIdxBBuf[k];
			int32 c = m_triadIdxCBuf[k];
			const float32 oax = m_triadPAXBuf[k];
			const float32 oay = m_triadPAYBuf[k];
			const float32 obx = m_triadPBXBuf[k];
			const float32 oby = m_triadPBYBuf[k];
			const float32 ocx = m_triadPCXBuf[k];
			const float32 ocy = m_triadPCYBuf[k];
			float32 pax = m_positionXBuffer[a];
			float32 pay = m_positionYBuffer[a];
			float32 pbx = m_positionXBuffer[b];
			float32 pby = m_positionYBuffer[b];
			float32 pcx = m_positionXBuffer[c];
			float32 pcy = m_positionYBuffer[c];
			float32 vax = m_velocityXBuffer[a];
			float32 vay = m_velocityYBuffer[a];
			float32 vbx = m_velocityXBuffer[b];
			float32 vby = m_velocityYBuffer[b];
			float32 vcx = m_velocityXBuffer[c];
			float32 vcy = m_velocityYBuffer[c];
			pax += step.dt * vax;
			pay += step.dt * vay;
			pbx += step.dt * vbx;
			pby += step.dt * vby;
			pcx += step.dt * vcx;
			pcy += step.dt * vcy;
			float32 midPointX = (float32)1 / 3 * (pax + pbx + pcx);
			float32 midPointY = (float32)1 / 3 * (pay + pby + pcy);
			pax -= midPointX;
			pay -= midPointY;
			pbx -= midPointX;
			pby -= midPointY;
			pcx -= midPointX;
			pcy -= midPointY;
			b2Rot r;
			r.s = b2Cross(oax, oay, pax, pay) + b2Cross(obx, oby, pbx, pby) + b2Cross(ocx, ocy, pcx, pcy);
			r.c = b2Dot(oax, oay, pax, pay) + b2Dot(obx, oby, pbx, pby) + b2Dot(ocx, ocy, pcx, pcy);
			float32 r2 = r.s * r.s + r.c * r.c;
			float32 invR = b2InvSqrt(r2);
			r.s *= invR;
			r.c *= invR;
			float32 strength = elasticStrength * m_triadStrengthBuf[k];
			float32 velX = (b2MulX(r, oax, oay) - pax);
			float32 velY = (b2MulY(r, oax, oay) - pay);
			m_velocityXBuffer[a] += strength * velX;
			m_velocityYBuffer[a] += strength * velY;
			velX = (b2MulX(r, obx, oby) - pbx);
			velY = (b2MulY(r, obx, oby) - pby);
			m_velocityXBuffer[b] += strength * velX;
			m_velocityYBuffer[b] += strength * velY;
			velX = (b2MulX(r, ocx, ocy) - pcx);
			velY = (b2MulY(r, ocx, ocy) - pcy);
			m_velocityXBuffer[c] += strength * velX;
			m_velocityYBuffer[c] += strength * velY;
		}
	}
}
void b2ParticleSystem::AFSolveElastic(const b2TimeStep& step)
{
	float32 elasticStrength = step.inv_dt * m_def.elasticStrength;
	
	const af::array condIdxs = af::where(afTriadFlagsBuf & b2_elasticParticle);
	if (!condIdxs.isempty())
	{
		const af::array& a = afTriadIdxABuf(condIdxs);
		const af::array& b = afTriadIdxBBuf(condIdxs);
		const af::array& c = afTriadIdxCBuf(condIdxs);
		const af::array& oax = afTriadPAXBuf(condIdxs);
		const af::array& oay = afTriadPAYBuf(condIdxs);
		const af::array& obx = afTriadPBXBuf(condIdxs);
		const af::array& oby = afTriadPBYBuf(condIdxs);
		const af::array& ocx = afTriadPCXBuf(condIdxs);
		const af::array& ocy = afTriadPCYBuf(condIdxs);
		af::array pax = afPosXBuf(a);
		af::array pay = afPosYBuf(a);
		af::array pbx = afPosXBuf(b);
		af::array pby = afPosYBuf(b);
		af::array pcx = afPosXBuf(c);
		af::array pcy = afPosYBuf(c);
		const af::array& vax = afVelXBuf(a);
		const af::array& vay = afVelYBuf(a);
		const af::array& vbx = afVelXBuf(b);
		const af::array& vby = afVelYBuf(b);
		const af::array& vcx = afVelXBuf(c);
		const af::array& vcy = afVelYBuf(c);
		pax += step.dt * vax;
		pay += step.dt * vay;
		pbx += step.dt * vbx;
		pby += step.dt * vby;
		pcx += step.dt * vcx;
		pcy += step.dt * vcy;
		const af::array& midPointX = 1.0f / 3 * (pax + pbx + pcx);
		const af::array& midPointY = 1.0f / 3 * (pay + pby + pcy);
		pax -= midPointX;
		pay -= midPointY;
		pbx -= midPointX;
		pby -= midPointY;
		pcx -= midPointX;
		pcy -= midPointY;
		af::array rotS = b2Cross(oax, oay, pax, pay) + b2Cross(obx, oby, pbx, pby) + b2Cross(ocx, ocy, pcx, pcy);
		af::array rotC = b2Dot(oax, oay, pax, pay) + b2Dot(obx, oby, pbx, pby) + b2Dot(ocx, ocy, pcx, pcy);
		const af::array& r2 = rotS * rotS + rotC * rotC;
		const af::array& invR = afInvSqrt(r2);
		rotS *= invR;
		rotC *= invR;
		const af::array& strength = elasticStrength * afTriadStrengthBuf(condIdxs);
		af::array velX = (b2MulX(rotS, rotC, oax, oay) - pax);
		af::array velY = (b2MulY(rotS, rotC, oax, oay) - pay);
		afVelXBuf(a) += strength * velX;
		afVelYBuf(a) += strength * velY;
		velX = (b2MulX(rotS, rotC, obx, oby) - pbx);
		velY = (b2MulY(rotS, rotC, obx, oby) - pby);
		afVelXBuf(b) += strength * velX;
		afVelYBuf(b) += strength * velY;
		velX = (b2MulX(rotS, rotC, ocx, ocy) - pcx);
		velY = (b2MulY(rotS, rotC, ocx, ocy) - pcy);
		afVelXBuf(c) += strength * velX;
		afVelYBuf(c) += strength * velY;
	}
}

void b2ParticleSystem::SolveSpring(const b2TimeStep& step)
{
	float32 springStrength = step.inv_dt * m_def.springStrength;
	for (int32 k = 0; k < m_pairCount; k++)
	{
		if (m_pairFlagsBuf[k] & b2_springParticle)
		{
			int32 a = m_pairIdxABuf[k];
			int32 b = m_pairIdxBBuf[k];
			b2Vec2 pa = b2Vec2(m_positionXBuffer[a], m_positionYBuffer[a]);
			b2Vec2 pb = b2Vec2(m_positionXBuffer[b], m_positionYBuffer[b]);
			b2Vec2 va = b2Vec2(m_velocityXBuffer[a], m_velocityYBuffer[a]);
			b2Vec2 vb = b2Vec2(m_velocityXBuffer[b], m_velocityYBuffer[b]);
			pa += step.dt * va;
			pb += step.dt * vb;
			b2Vec2 d = pb - pa;
			float32 r0 = m_pairDistanceBuf[k];
			float32 r1 = d.Length();
			float32 strength = springStrength * m_pairStrengthBuf[k];
			b2Vec2 f = strength * (r0 - r1) / r1 * d;
			m_velocityXBuffer[a] -= f.x;
			m_velocityYBuffer[a] -= f.y;
			m_velocityXBuffer[b] += f.x;
			m_velocityYBuffer[b] += f.y;
		}
	}
}
void b2ParticleSystem::AFSolveSpring(const b2TimeStep& step)
{
	float32 springStrength = step.inv_dt * m_def.springStrength;
	const af::array& k = af::where(afPairFlagsBuf & b2_springParticle);
	if (!k.isempty())
	{
		const af::array& a = afPairIdxABuf(k);
		const af::array& b = afPairIdxBBuf(k);
		const af::array& dx = (afPosXBuf(b) + step.dt * afVelXBuf(b))
							- (afPosXBuf(a) + step.dt * afVelXBuf(a));
		const af::array& dy = (afPosYBuf(b) + step.dt * afVelYBuf(b))
							- (afPosYBuf(a) + step.dt * afVelYBuf(a));
		const af::array& r0 = afPairDistanceBuf(k);
		const af::array& r1 = AFLength(dx, dy);
		const af::array& strength = springStrength * afPairStrengthBuf(k);
		const af::array& fx = strength * (r0 - r1) / r1 * dx;
		const af::array& fy = strength * (r0 - r1) / r1 * dy;
		afVelXBuf(a) -= fx;
		afVelYBuf(a) -= fy;
		afVelXBuf(b) += fx;
		afVelYBuf(b) += fy;
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
		if (m_contactFlagsBuffer[k] & b2_tensileParticle)
		{
			int32 a = m_contactIdxABuffer[k];
			int32 b = m_contactIdxBBuffer[k];
			float32 w = m_contactWeightBuffer[k];
			b2Vec2 n = b2Vec2(m_contactNormalXBuffer[k], m_contactNormalYBuffer[k]);
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
		if (m_contactFlagsBuffer[k] & b2_tensileParticle)
		{
			int32 a = m_contactIdxABuffer[k];
			int32 b = m_contactIdxBBuffer[k];
			float32 w = m_contactWeightBuffer[k];
			float32 m = m_contactMassBuffer[k];
			b2Vec2 n = b2Vec2(m_contactNormalXBuffer[k], m_contactNormalYBuffer[k]);
			float32 h = m_weightBuffer[a] + m_weightBuffer[b];
			b2Vec2 s = m_accumulation2Buf[b] - m_accumulation2Buf[a];
			float32 fn = b2Min(
				pressureStrength * (h - 2) + normalStrength * b2Dot(s, n),
				maxVelocityVariation) * w;
			b2Vec2 f = fn * n * m;
			float32 massA = m_partMatMassBuf[m_partMatIdxBuffer[a]];
			float32 massB = m_partMatMassBuf[m_partMatIdxBuffer[b]];
			m_velocityXBuffer[a] -= massB * f.x;	//CHANGED
			m_velocityYBuffer[a] -= massB * f.y;
			m_velocityXBuffer[b] += massA * f.x;
			m_velocityYBuffer[b] += massA * f.y;
		}
	}
}
void b2ParticleSystem::AFSolveTensile(const b2TimeStep& step)
{
	b2Assert(afHasAccumulation2XBuf);
	afAccumulation2XBuf = af::constant(0.0f, m_count);
	afAccumulation2YBuf = af::constant(0.0f, m_count);
	
	const af::array& k = af::where(afContactFlagsBuf & b2_tensileParticle);
	if (k.isempty())
		return;
	return;

	const af::array& a = afContactIdxABuf(k);
	const af::array& b = afContactIdxBBuf(k);
	const vector<af::array>& aSplit = AFSeperateToUniqueIdxBufs(a);
	const vector<af::array>& bSplit = AFSeperateToUniqueIdxBufs(b);

	const af::array& w = afContactWeightBuf(k);
	const af::array& nx = afContactNormalXBuf(k);
	const af::array& ny = afContactNormalYBuf(k);
	const af::array& weightedNormalX = (1 - w) * w * nx;
	const af::array& weightedNormalY = (1 - w) * w * ny;

	int32* nafA = a.host<int32>();
	int32* nafB = b.host<int32>();
	float32* nafWeightedNormalX = weightedNormalX.host<float32>();
	float32* nafWeightedNormalY = weightedNormalY.host<float32>();
	float32* nafAccumulation2XBuf = afAccumulation2XBuf.host<float32>();
	float32* nafAccumulation2YBuf = afAccumulation2YBuf.host<float32>();
	int32 count = a.elements();
	for (int32 i = 0; i < count; i++)	//on CPU because overlapping idxs
	{
		nafAccumulation2XBuf[nafA[i]] -= nafWeightedNormalX[i];
		nafAccumulation2YBuf[nafA[i]] -= nafWeightedNormalY[i];
		nafAccumulation2XBuf[nafB[i]] += nafWeightedNormalX[i];
		nafAccumulation2YBuf[nafB[i]] += nafWeightedNormalY[i];
	}
	CPPtoAF(m_count, nafAccumulation2XBuf, afAccumulation2XBuf);
	CPPtoAF(m_count, nafAccumulation2YBuf, afAccumulation2YBuf);

	float32 criticalVelocity = GetCriticalVelocity(step);
	float32 pressureStrength = m_def.surfaceTensionPressureStrength
		* criticalVelocity;
	float32 normalStrength = m_def.surfaceTensionNormalStrength
		* criticalVelocity;
	float32 maxVelocityVariation = b2_maxParticleForce * criticalVelocity;

	const af::array& m = afContactMassBuf(k);
	const af::array& h = afWeightBuf(a) + afWeightBuf(b);
	const af::array& sx = afAccumulation2XBuf(b) - afAccumulation2XBuf(a);
	const af::array& sy = afAccumulation2YBuf(b) - afAccumulation2YBuf(a);
	const af::array& fn = af::min(
		pressureStrength * (h - 2) + normalStrength * b2Dot(sx, sy, nx, ny),
		maxVelocityVariation) * w;
	const af::array& fx = fn * nx * m;
	const af::array& fy = fn * ny * m;
	const af::array& massA = afPartMatMassBuf((af::array)afPartMatIdxBuf(a));
	const af::array& massB = afPartMatMassBuf((af::array)afPartMatIdxBuf(b));


	float32* nafFx	    = fx.host<float32>();
	float32* nafFy	    = fy.host<float32>();
	float32* nafMassA   = massA.host<float32>();
	float32* nafMassB   = massB.host<float32>();
	float32* nafVelXBuf = afVelXBuf.host<float32>();
	float32* nafVelYBuf = afVelYBuf.host<float32>();
	for (int32 i = 0; i < count; i++)
	{
		nafVelXBuf[nafA[i]] -= nafMassB[i] * nafFx[i];
		nafVelYBuf[nafA[i]] -= nafMassB[i] * nafFy[i];
		nafVelXBuf[nafB[i]] += nafMassA[i] * nafFx[i];
		nafVelYBuf[nafB[i]] += nafMassA[i] * nafFy[i];
	}
	CPPtoAF(m_count, nafVelXBuf, afVelXBuf);
	CPPtoAF(m_count, nafVelYBuf, afVelYBuf);
}

void b2ParticleSystem::SolveViscous()
{
	float32 viscousStrength = m_def.viscousStrength;
	for (int32 k = 0; k < m_bodyContactCount; k++)
	{
		int32 a = m_bodyContactIdxBuffer[k];
		if (m_flagsBuffer[a] & b2_viscousParticle)
		{
			int32 bIdx = m_bodyContactBodyIdxBuffer[k];
			float32 w = m_bodyContactWeightBuffer[k];
			float32 m = m_bodyContactMassBuffer[k];
			b2Vec2 p = b2Vec2(m_positionXBuffer[a], m_positionYBuffer[a]);
			b2Vec2 v = m_bodyBuffer[bIdx]->GetLinearVelocityFromWorldPoint(p) -
				b2Vec2(m_velocityXBuffer[a], m_velocityYBuffer[a]);
			b2Vec2 f = viscousStrength * m * w * v;
			float32 invMass = m_partMatInvMassBuf[m_partMatIdxBuffer[a]];
			m_velocityXBuffer[a] += invMass * f.x;
			m_velocityYBuffer[a] += invMass * f.y;
			m_bodyBuffer[bIdx]->ApplyLinearImpulse(-f, p, true);
		}
	}
	for (int32 k = 0; k < m_contactCount; k++)
	{
		if (m_contactFlagsBuffer[k] & b2_viscousParticle)
		{
			int32 a = m_contactIdxABuffer[k];
			int32 b = m_contactIdxBBuffer[k];
			float32 w = m_contactWeightBuffer[k];
			float32 m = m_contactMassBuffer[k];
			b2Vec2 v = b2Vec2(m_velocityXBuffer[b] - m_velocityXBuffer[a], m_velocityYBuffer[b] - m_velocityYBuffer[a]);
			b2Vec2 f = viscousStrength * w * m * v;
			float32 massA = m_partMatMassBuf[m_partMatIdxBuffer[a]];
			float32 massB = m_partMatMassBuf[m_partMatIdxBuffer[b]];
			m_velocityXBuffer[a] += massB * f.x;
			m_velocityYBuffer[a] += massB * f.y;
			m_velocityXBuffer[b] -= massA * f.x;
			m_velocityYBuffer[b] -= massA * f.y;
		}
	}
}
void b2ParticleSystem::AFSolveViscous()
{

	float32 viscousStrength = m_def.viscousStrength;

	//TODO after Body Contacts
	/*for (int32 k = 0; k < m_bodyContactCount; k++)
	{
		int32 a = m_bodyContactIdxBuffer[k];
		if (m_flagsBuffer[a] & b2_viscousParticle)
		{
			b2Body* b = m_bodyContactBodyBuffer[k];
			float32 w = m_bodyContactWeightBuffer[k];
			float32 m = m_bodyContactMassBuffer[k];
			b2Vec2 p = b2Vec2(m_positionXBuffer[a], m_positionYBuffer[a]);
			b2Vec2 v = b->GetLinearVelocityFromWorldPoint(p) -
				b2Vec2(m_velocityXBuffer[a], m_velocityYBuffer[a]);
			b2Vec2 f = viscousStrength * m * w * v;
			float32 invMass = m_partMatInvMassBuf[m_partMatIdxBuffer[a]];
			m_velocityXBuffer[a] += invMass * f.x;
			m_velocityYBuffer[a] += invMass * f.y;
			b->ApplyLinearImpulse(-f, p, true);
		}
	}*/

	af::array condIdxs = af::where(afContactFlagsBuf & b2_viscousParticle);
	if (!condIdxs.isempty())
	{
		const af::array& a = afContactIdxABuf(condIdxs);
		const af::array& b = afContactIdxBBuf(condIdxs);
		const af::array& w = afContactWeightBuf(condIdxs);
		const af::array& m = afContactMassBuf(condIdxs);
		const af::array& vx = afVelXBuf(b) - afVelXBuf(a);
		const af::array& vy = afVelYBuf(b) - afVelYBuf(a);
		const af::array& fx = viscousStrength * w * m * vx;
		const af::array& fy = viscousStrength * w * m * vy;
		const af::array& matAIdx = afPartMatIdxBuf(a);
		const af::array& matBIdx = afPartMatIdxBuf(b);
		const af::array& massA = afPartMatMassBuf(matAIdx);
		const af::array& massB = afPartMatMassBuf(matBIdx);
		//cout << "SolveViscious\n";
		//cout << "Changing Vel of " << (massB * fx).elements() << "\n";
		afVelXBuf(a) += massB * fx;
		afVelYBuf(a) += massB * fy;
		afVelXBuf(b) += massA * fx;
		afVelYBuf(b) += massA * fy;
	}
}

void b2ParticleSystem::SolveRepulsive(const b2TimeStep& step)
{
	float32 repulsiveStrength =
		m_def.repulsiveStrength * GetCriticalVelocity(step);
	for (int32 k = 0; k < m_contactCount; k++)
	{
		if (m_contactFlagsBuffer[k] & b2_repulsiveParticle)
		{
			int32 a = m_contactIdxABuffer[k];
			int32 b = m_contactIdxBBuffer[k];
			if (m_groupIdxBuffer[a] != m_groupIdxBuffer[b])
			{
				float32 w = m_contactWeightBuffer[k];
				float32 m = m_contactMassBuffer[k];
				b2Vec2 n = b2Vec2(m_contactNormalXBuffer[k], m_contactNormalYBuffer[k]);
				b2Vec2 f = repulsiveStrength * w * m * n;
				float32 massA = m_partMatMassBuf[m_partMatIdxBuffer[a]];
				float32 massB = m_partMatMassBuf[m_partMatIdxBuffer[b]];
				m_velocityXBuffer[a] -= massB * f.x;
				m_velocityYBuffer[a] -= massB * f.y;
				m_velocityXBuffer[b] += massA * f.x;
				m_velocityYBuffer[b] += massA * f.y;
			}
		}
	}
}
void b2ParticleSystem::AFSolveRepulsive(const b2TimeStep& step)
{
	float32 repulsiveStrength =
		m_def.repulsiveStrength * GetCriticalVelocity(step);

	af::array k = af::where(afContactFlagsBuf & b2_repulsiveParticle);
	if (!k.isempty())
	{
		af::array a = afContactIdxABuf(k);
		af::array b = afContactIdxBBuf(k);
		const af::array& k2 = af::where(afGroupIdxBuf(a) != afGroupIdxBuf(b));
		{
			k = k(k2);
			a = a(k2);
			b = b(k2);
			const af::array& w = afContactWeightBuf(k);
			const af::array& m = afContactMassBuf(k);
			const af::array& nx = afContactNormalXBuf(k);
			const af::array& ny = afContactNormalYBuf(k);
			const af::array& fx = repulsiveStrength * w * m * nx;
			const af::array& fy = repulsiveStrength * w * m * ny;
			const af::array& matIdxA = afPartMatIdxBuf(a);
			const af::array& matIdxB = afPartMatIdxBuf(b);
			const af::array& massA = afPartMatMassBuf(matIdxA);
			const af::array& massB = afPartMatMassBuf(matIdxB);
			afVelXBuf(a) -= massB * fx;
			afVelYBuf(a) -= massB * fy;
			afVelXBuf(b) += massA * fx;
			afVelYBuf(b) += massA * fy;
		}
	}
}

void b2ParticleSystem::SolvePowder(const b2TimeStep& step)
{
	float32 powderStrength = m_def.powderStrength * GetCriticalVelocity(step);
	float32 minWeight = 1.0f - b2_particleStride;
	for (int32 k = 0; k < m_contactCount; k++)
	{
		if (m_contactFlagsBuffer[k] & b2_powderParticle)
		{
			float32 w = m_contactWeightBuffer[k];
			if (w > minWeight)
			{
				int32 a = m_contactIdxABuffer[k];
				int32 b = m_contactIdxBBuffer[k];
				float32 m = m_contactMassBuffer[k];
				b2Vec2 n = b2Vec2(m_contactNormalXBuffer[k], m_contactNormalYBuffer[k]);
				b2Vec2 f = powderStrength * (w - minWeight) * m * n;
				float32 massA = m_partMatMassBuf[m_partMatIdxBuffer[a]];
				float32 massB = m_partMatMassBuf[m_partMatIdxBuffer[b]];
				m_velocityXBuffer[a] -= massB * f.x;
				m_velocityYBuffer[a] -= massB * f.y;
				m_velocityXBuffer[b] += massA * f.x;
				m_velocityYBuffer[b] += massA * f.y;
			}
		}
	}
}
void b2ParticleSystem::AFSolvePowder(const b2TimeStep& step)
{
	float32 powderStrength = m_def.powderStrength * GetCriticalVelocity(step);
	float32 minWeight = 1.0f - b2_particleStride;
	af::array k = af::where(afContactFlagsBuf & b2_powderParticle);
	if (!k.isempty())
	{
		af::array w = afContactWeightBuf(k);
		const af::array& k2 = af::where(w > minWeight);
		if (!k2.isempty())
		{
			k = k(k2);
			w = w(k2);
			const af::array& a = afContactIdxABuf(k);
			const af::array& b = afContactIdxBBuf(k);
			const af::array& m = afContactMassBuf(k);
			const af::array& nx = afContactNormalXBuf(k);
			const af::array& ny = afContactNormalYBuf(k);
			const af::array& fx = powderStrength * (w - minWeight) * m * nx;
			const af::array& fy = powderStrength * (w - minWeight) * m * ny;
			const af::array& matIdxA = afPartMatIdxBuf(a);
			const af::array& matIdxB = afPartMatIdxBuf(b);
			const af::array& massA = afPartMatMassBuf(matIdxA);
			const af::array& massB = afPartMatMassBuf(matIdxB);
			afVelXBuf(a) -= massB * fx;
			afVelYBuf(a) -= massB * fy;
			afVelXBuf(b) += massA * fx;
			afVelYBuf(b) += massA * fy;
		}
	}
}

void b2ParticleSystem::SolveSolid(const b2TimeStep& step)
{
	// applies extra repulsive force from solid particle groups
	b2Assert(m_depthBuffer);
	float32 ejectionStrength = step.inv_dt * m_def.ejectionStrength;
	for (int32 k = 0; k < m_contactCount; k++)
	{
		int32 a = m_contactIdxABuffer[k];
		int32 b = m_contactIdxBBuffer[k];
		if (m_groupIdxBuffer[a] != m_groupIdxBuffer[b])
		{
			float32 w = m_contactWeightBuffer[k];
			float32 m = m_contactMassBuffer[k];
			b2Vec2 n = b2Vec2(m_contactNormalXBuffer[k], m_contactNormalYBuffer[k]);
			float32 h = m_depthBuffer[a] + m_depthBuffer[b];
			b2Vec2 f = ejectionStrength * h * m * w * n;
			float32 massA = m_partMatMassBuf[m_partMatIdxBuffer[a]];
			float32 massB = m_partMatMassBuf[m_partMatIdxBuffer[b]];
			m_velocityXBuffer[a] -= massB * f.x;
			m_velocityYBuffer[a] -= massB * f.y;
			m_velocityXBuffer[b] += massA * f.x;
			m_velocityYBuffer[b] += massA * f.y;
		}
	}
}
void b2ParticleSystem::AFSolveSolid(const b2TimeStep& step)
{
	// applies extra repulsive force from solid particle groups
	b2Assert(!afDepthBuf.isempty());
	float32 ejectionStrength = step.inv_dt * m_def.ejectionStrength;
	const af::array& k = af::where(afContactIdxABuf != afContactIdxBBuf);
	if (!k.isempty())
	{
		const af::array& a = afContactIdxABuf(k);
		const af::array& b = afContactIdxBBuf(k);
		const af::array& w = afContactWeightBuf(k);
		const af::array& m = afContactMassBuf(k);
		const af::array& nx = afContactNormalXBuf(k);
		const af::array& ny = afContactNormalYBuf(k);
		const af::array& h = afDepthBuf(a) + afDepthBuf(b);
		const af::array& fx = ejectionStrength * h * m * w * nx;
		const af::array& fy = ejectionStrength * h * m * w * ny;
		const af::array& massA = afPartMatMassBuf((af::array)afPartMatIdxBuf(a));
		const af::array& massB = afPartMatMassBuf((af::array)afPartMatIdxBuf(b));
		afVelXBuf(a) -= massB * fx;
		afVelYBuf(a) -= massB * fy;
		afVelXBuf(b) += massA * fx;
		afVelYBuf(b) += massA * fy;
	}
}

void b2ParticleSystem::SolveForce(const b2TimeStep& step)
{
	for (int32 i = 0; i < m_count; i++)
	{
		m_velocityXBuffer[i] += step.dt * m_partMatInvMassBuf[m_partMatIdxBuffer[i]] * m_forceXBuffer[i];
		m_velocityYBuffer[i] += step.dt * m_partMatInvMassBuf[m_partMatIdxBuffer[i]] * m_forceYBuffer[i];
	}
	m_hasForce = false;
}
void b2ParticleSystem::AFSolveForce(const b2TimeStep& step)
{
	//cout << "SolveForce\n";
	//cout << "ForceBufSize: " << afForceXBuf.elements() << "\n";
	//cout << "VelBufSize:   " << afVelXBuf.elements() << "\n";
	afVelXBuf += step.dt * afPartMatInvMassBuf(afPartMatIdxBuf) * afForceXBuf;
	afVelYBuf += step.dt * afPartMatInvMassBuf(afPartMatIdxBuf) * afForceYBuf;
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
			int32 a = m_contactIdxABuffer[k];
			int32 b = m_contactIdxBBuffer[k];
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
		if (m_contactMatFlagsBuffer[k] & b2_heatConductingMaterial)
		{
			int32 a = m_contactIdxABuffer[k];
			int32 b = m_contactIdxBBuffer[k];
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
		int32 i = m_bodyContactIdxBuffer[k];
		int32 bIdx = m_bodyContactBodyIdxBuffer[k];
		b2Body* b = m_bodyBuffer[bIdx];
		int32 matIdxP = m_partMatIdxBuffer[i];
		b2BodyMaterial* matB = b->m_material;
		if (m_partMatFlagsBuf[matIdxP] & matB->m_matFlags & b2_heatConductingMaterial)
		{
			float32 conductivityP = m_partMatHeatConductivityBuf[matIdxP];
			float32 conductivityB = matB->m_heatConductivity;
			if (conductivityP > 0 && conductivityB > 0)
			{
				float32 heatP = m_heatBuffer[i];
				float32 heatB = b->m_heat;
				if (abs(heatB - heatP) > 1.0f)
				{
					float32 changeHeat = step.dt * 30.0f * (conductivityP * conductivityB)
										 * (heatB - heatP);
					float32 massP = m_partMatMassBuf[matIdxP];
					float32 massB = b->GetMass();
					float32 invCombinedMass = 0.999f / (massP + massB);
					m_heatBuffer[i] += changeHeat * (0.001f + massB * invCombinedMass);
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
			float loss = step.dt * m_partMatHeatConductivityBuf[m_partMatIdxBuffer[k]] * (m_heatBuffer[k] - m_roomTemp);
			if (abs(loss) > step.dt)
			{
				m_heatBuffer[k] -= loss * (1 - pow(m_heatLossRatio,
					//(2.0f - (m_weightBuffer[k] < 0 ? 0 : m_weightBuffer[k] > 1 ? 1 : m_weightBuffer[k])) *
					0.0005f * m_partMatInvMassBuf[m_partMatIdxBuffer[k]])); //CHANGED
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
			float loss = step.dt * m_heatBuffer[k] * 0.001 * m_partMatInvStabilityBuf[m_partMatIdxBuffer[k]];
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
			// loose health
			int32 partMatIdx = m_partMatIdxBuffer[k];
			float loss = step.dt * m_heatBuffer[k] * 0.001 * m_partMatInvStabilityBuf[partMatIdx];
			if (loss > FLT_EPSILON)
				m_healthBuffer[k] -= loss;
			if (m_heatBuffer[k] < m_partMatExtinguishingPointBuf[partMatIdx])
				DestroyParticle(k);
		}
	}
}

void b2ParticleSystem::SolveIgnite()
{
	// Particle Ignition
	for (int32 k = 0; k < m_contactCount; k++)
	{
		if (m_contactFlagsBuffer[k] & b2_flameParticle && m_contactMatFlagsBuffer[k] & b2_flammableMaterial)
		{
			int32 a = m_contactIdxABuffer[k];
			int32 b = m_contactIdxBBuffer[k];
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
	// Body ignitions
	for (int32 k = 0; k < m_bodyContactCount; k++)
	{
		int32 i = m_bodyContactIdxBuffer[k];
		int32 bIdx = m_bodyContactBodyIdxBuffer[k];
		b2Body* b = m_bodyBuffer[bIdx];
		if (m_flagsBuffer[i] & b2_flameParticle
			&& b->GetFlags() & b2_inflammableBody
			&& b->m_material->m_matFlags & b2_flammableMaterial
			&& b->m_material->m_ignitionPoint <= b->m_heat)
		{
			b->AddFlags((int)b2_burningBody);
		}
	}
}

void b2ParticleSystem::SolveExtinguish()
{
	// Particle Extinguishing
	for (int32 k = 0; k < m_contactCount; k++)
	{
		if (m_contactFlagsBuffer[k] & b2_burningParticle && m_contactMatFlagsBuffer[k] & b2_extinguishingMaterial)
		{
			int32 a = m_contactIdxABuffer[k];
			int32 b = m_contactIdxBBuffer[k];
			if (m_partMatFlagsBuf[m_partMatIdxBuffer[a]] & b2_extinguishingMaterial
				&& m_flagsBuffer[b] & b2_burningParticle)
				m_flagsBuffer[b] &= ~b2_burningParticle;

			else if (m_partMatFlagsBuf[m_partMatIdxBuffer[b]] & b2_extinguishingMaterial
				&& m_flagsBuffer[a] & b2_burningParticle)
				m_flagsBuffer[a] &= ~b2_burningParticle;
		}
	}

	// Body Extinguishing
	for (int32 k = 0; k < m_bodyContactCount; k++)
	{
		int32 i = m_bodyContactIdxBuffer[k];
		int32 bIdx = m_bodyContactBodyIdxBuffer[k];
		b2Body* b = m_bodyBuffer[bIdx];
		int32 partMatIdx = m_partMatIdxBuffer[i];
		if (m_partMatFlagsBuf[partMatIdx] & b2_extinguishingMaterial
			&& b->GetFlags() & b2_burningBody
			&& b->m_heat < m_partMatBoilingPointBuf[partMatIdx])
		{
			b->RemFlags(b2_burningBody);
			b->AddFlags(b2_wetBody);
		}
	}
}

void b2ParticleSystem::SolveWater()
{
	// make Objects wet
	for (int32 k = 0; k < m_bodyContactCount; k++)
	{
		int32 i = m_bodyContactIdxBuffer[k];
		int32 bIdx = m_bodyContactBodyIdxBuffer[k];
		b2Body* b = m_bodyBuffer[bIdx];
		if (m_flagsBuffer[i] & b2_extinguishingMaterial
			&& b->GetFlags() & b2_burningBody
			&& b->m_material->m_matFlags & b2_flammableMaterial
			&& b->m_material->m_extinguishingPoint > b->m_heat)
		{
			b->RemFlags((int)b2_burningBody);
		}
	}
}

void b2ParticleSystem::SolveEvaporate()
{
	for (int32 k = 0; k < m_count; k++)
	{
		if (m_partMatFlagsBuf[m_partMatIdxBuffer[k]] & b2_boilingMaterial)
		{
			if (m_heatBuffer[k] > m_partMatBoilingPointBuf[m_partMatIdxBuffer[k]])
				//TODO
				DestroyParticle(k);
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
					handle->SetIndex(b2_invalidParticleIndex);
					m_handleIndexBuffer[i] = NULL;
					m_handleAllocator.Free(handle);
				}
			}
			newIndices[i] = b2_invalidParticleIndex;
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
				m_collisionLayerBuffer[newCount] = m_collisionLayerBuffer[i];
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
				m_positionXBuffer[newCount]  = m_positionXBuffer[i];
				m_positionYBuffer[newCount]  = m_positionYBuffer[i];
				m_positionZBuffer[newCount]  = m_positionZBuffer[i];
				m_velocityXBuffer[newCount]  = m_velocityXBuffer[i];
				m_velocityYBuffer[newCount]  = m_velocityYBuffer[i];
				m_groupIdxBuffer[newCount]   = m_groupIdxBuffer[i];
				m_partMatIdxBuffer[newCount] = m_partMatIdxBuffer[i];
				if (m_hasForce)
				{
					m_forceXBuffer[newCount] = m_forceXBuffer[i];
					m_forceYBuffer[newCount] = m_forceYBuffer[i];
				}
				if (!m_staticPressureBuf.empty())
				{
					m_staticPressureBuf[newCount] =
						m_staticPressureBuf[i];
				}
				if (m_depthBuffer.data())
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
			return proxy.index < 0;
		}
		static bool IsProxyIdxInvalid(const int32& proxyIdx)
		{
			return proxyIdx < 0;
		}
		static bool IsContactIdxInvalid(const int32& idx)
		{
			return idx < 0;
		}
		static bool IsBodyContactInvalid(const int32& idx)
		{
			return idx < 0;
		}
	};

	// update proxies
	for (int32 k = 0; k < m_proxyIdxBuffer.size(); k++)
	{
		int32& idx = m_proxyIdxBuffer[k];
		idx = newIndices[idx];
	}
	RemoveFromVectorsIf(m_proxyIdxBuffer, m_proxyTagBuffer, m_count, Test::IsProxyIdxInvalid, false);

	// update contacts
	for (int32 k = 0; k < m_contactCount; k++)
	{
		m_contactIdxABuffer[k] = newIndices[m_contactIdxABuffer[k]];
		m_contactIdxBBuffer[k] = newIndices[m_contactIdxBBuffer[k]];
	}
	RemoveFromVectorsIf(m_contactIdxABuffer, m_contactIdxBBuffer, 
		m_contactWeightBuffer, m_contactMassBuffer, 
		m_contactNormalXBuffer, m_contactNormalYBuffer, 
		m_contactFlagsBuffer, m_contactMatFlagsBuffer, 
		m_contactCount, Test::IsContactIdxInvalid, Test::IsContactIdxInvalid, true);

	// update particle-body contacts
	for (int32 k = 0; k < m_bodyContactCount; k++)
	{
		int32& idx = m_bodyContactIdxBuffer[k];
		idx = newIndices[idx];
	}
	RemoveFromVectorsIf(m_bodyContactIdxBuffer, m_bodyContactBodyIdxBuffer,
		m_bodyContactFixtureIdxBuffer, m_bodyContactWeightBuffer, 
		m_bodyContactNormalXBuffer, m_bodyContactNormalYBuffer, 
		m_bodyContactMassBuffer, m_bodyContactCount, Test::IsBodyContactInvalid, true);

	// update pairs
	for (int32 k = 0; k < m_pairCount; k++)
	{
		m_pairIdxABuf[k] = newIndices[m_pairIdxABuf[k]];
		m_pairIdxABuf[k] = newIndices[m_pairIdxABuf[k]];
	}

	auto IsPairValid = [this](int32 i) -> bool
	{
		return m_pairIdxABuf[i] >= 0 && m_pairIdxBBuf[i] >= 0;
	};
	vector<int32> idxs = GetValidIdxs(m_pairCount, IsPairValid);
	reorder(m_pairIdxABuf, idxs);
	reorder(m_pairIdxBBuf, idxs);
	reorder(m_pairFlagsBuf, idxs);
	reorder(m_pairStrengthBuf, idxs);
	reorder(m_pairDistanceBuf, idxs);
	m_pairCount = idxs.size();

	// update triads
	for (int32 k = 0; k < m_triadCount; k++)
	{
		m_triadIdxABuf[k] = newIndices[m_triadIdxABuf[k]];
		m_triadIdxBBuf[k] = newIndices[m_triadIdxBBuf[k]];
		m_triadIdxCBuf[k] = newIndices[m_triadIdxCBuf[k]];

	}

	auto IsTriadValid = [this](int32 i) -> bool
	{
		return m_triadIdxABuf[i] >= 0 && m_triadIdxBBuf[i] >= 0 && m_triadIdxCBuf[i] >= 0;
	};
	idxs = GetValidIdxs(m_triadCount, IsTriadValid);
	reorder(m_triadIdxABuf, idxs);
	reorder(m_triadIdxBBuf, idxs);
	reorder(m_triadIdxCBuf, idxs);
	reorder(m_triadFlagsBuf, idxs);
	reorder(m_triadStrengthBuf, idxs);
	reorder(m_triadPAXBuf, idxs);
	reorder(m_triadPAYBuf, idxs);
	reorder(m_triadPBXBuf, idxs);
	reorder(m_triadPBYBuf, idxs);
	reorder(m_triadPCXBuf, idxs);
	reorder(m_triadPCYBuf, idxs);
	reorder(m_triadKABuf, idxs);
	reorder(m_triadKBBuf, idxs);
	reorder(m_triadKCBuf, idxs);
	reorder(m_triadSBuf, idxs);
	m_triadCount = idxs.size();

	// Update lifetime indices.
	if (!m_idxByExpireTimeBuf.empty())
	{
		int32 writeOffset = 0;
		for (int32 readOffset = 0; readOffset < m_count; readOffset++)
		{
			const int32 newIndex = newIndices[
				m_idxByExpireTimeBuf[readOffset]];
			if (newIndex != b2_invalidParticleIndex)
			{
				m_idxByExpireTimeBuf[writeOffset++] = newIndex;
			}
		}
	}

	// update groups
	for each (const int32 groupIdx in m_realGroupIdxBuffer)
	{
		int32 firstIndex = newCount;
		int32 lastIndex = 0;
		bool modified = false;
		for (int32 i = m_groupFirstIdxBuf[groupIdx]; i < m_groupLastIdxBuf[groupIdx]; i++)
		{
			int32 j = newIndices[i];
			if (j >= 0) {
				firstIndex = b2Min(firstIndex, j);
				lastIndex = b2Max(lastIndex, j + 1);
			} else {
				modified = true;
			}
		}
		if (firstIndex < lastIndex)
		{
			m_groupFirstIdxBuf[groupIdx] = firstIndex;
			m_groupLastIdxBuf[groupIdx] = lastIndex;
			if (modified)
			{
				if (m_groupFlagsBuf[groupIdx] & b2_solidParticleGroup)
				{
					SetGroupFlags(groupIdx,
								  m_groupFlagsBuf[groupIdx] |
								  b2_particleGroupNeedsUpdateDepth);
				}
			}
		}
		else
		{
			m_groupFirstIdxBuf[groupIdx] = 0;
			m_groupLastIdxBuf[groupIdx] = 0;
			if (!(m_groupFlagsBuf[groupIdx] & b2_particleGroupCanBeEmpty))
			{
				SetGroupFlags(groupIdx,
					m_groupFlagsBuf[groupIdx] | b2_particleGroupWillBeDestroyed);
			}
		}
	}

	// update particle count
	m_count = newCount;
	m_allParticleFlags = allParticleFlags;
	m_needsUpdateAllParticleFlags = false;

	// destroy bodies with no particles
	for each (int32 groupIdx in m_realGroupIdxBuffer)
	{
		if (m_groupFlagsBuf[groupIdx] & b2_particleGroupWillBeDestroyed)
			DestroyParticleGroup(groupIdx);
	}
}
void b2ParticleSystem::AFSolveZombie()
{
	// removes particles with zombie flag
	uint32 allParticleFlags = 0;
	const af::array& zombies = afFlagBuf & b2_zombieParticle;
	const af::array& zombieIdxs = af::where(zombies);

	if (zombieIdxs.isempty())
		return;

	const af::array& newIdxs = af::where(!zombies);
	int32 newCount = newIdxs.elements();
	af::array reorderedIdxs(m_count, af::dtype::s32);
	reorderedIdxs(newIdxs) = af::seq(newCount);
	reorderedIdxs(zombieIdxs) = b2_invalidParticleIndex;


	AFDestructionListener * const destructionListener =
		m_world->afDestructionListener;
	if (destructionListener)
	{
		const af::array& condIdxs = af::where(afFlagBuf(zombieIdxs) & b2_destructionListenerParticle);
		if (!condIdxs.isempty())
			destructionListener->SayGoodbye(this, condIdxs);
	}
	// Destroy particle handle.
	if (!afHandleIdxBuf.isempty())
	{
		afHandleIdxBuf(zombieIdxs) = b2_invalidParticleIndex;
	}

	// Update handle to reference new particle index.
	if (!afHandleIdxBuf.isempty())
		afHandleIdxBuf = afHandleIdxBuf(newIdxs);
	afFlagBuf = afFlagBuf(newIdxs);
	afColLayBuf = afColLayBuf(newIdxs);
	if (!afLastBodyContactStepBuf.isempty())
		afLastBodyContactStepBuf = afLastBodyContactStepBuf(newIdxs);
	if (!afBodyContactCountBuf.isempty())
		afBodyContactCountBuf = afBodyContactCountBuf(newIdxs);
	if (!afConsecutiveContactStepsBuf.isempty())
		afConsecutiveContactStepsBuf = afConsecutiveContactStepsBuf(newIdxs);
	afPosXBuf = afPosXBuf(newIdxs);
	afPosYBuf = afPosYBuf(newIdxs);
	afPosZBuf = afPosZBuf(newIdxs);
	afVelXBuf = afVelXBuf(newIdxs);
	afVelYBuf = afVelYBuf(newIdxs);
	afGroupIdxBuf = afGroupIdxBuf(newIdxs);
	afPartMatIdxBuf = afPartMatIdxBuf(newIdxs);
	if (m_hasForce)
		afForceXBuf = afForceXBuf(newIdxs),
		afForceYBuf = afForceYBuf(newIdxs);
	if (!afStaticPressureBuf.isempty())
		afStaticPressureBuf = afStaticPressureBuf(newIdxs);
	if (!afDepthBuf.isempty())
		afDepthBuf = afDepthBuf(newIdxs);
	afHeatBuf = afHeatBuf(newIdxs);
	afHealthBuf = afHealthBuf(newIdxs);
	afColorBuf = afColorBuf(newIdxs);
	if (!afUserDataBuf.isempty())
		afUserDataBuf = afUserDataBuf(newIdxs);
	if (!afExpireTimeBuf.isempty())
		afExpireTimeBuf = afExpireTimeBuf(newIdxs);
	//allParticleFlags |= flags;	//TODO FIX
	

	// update proxies
	af::array condIdxs = newIdxs(af::where(afProxyIdxBuf(newIdxs) >= 0));
	afProxyIdxBuf = afProxyIdxBuf(condIdxs);
	afProxyTagBuf = afProxyTagBuf(condIdxs);

	// update contacts
	condIdxs = newIdxs(af::where(afContactIdxABuf(newIdxs) >= 0 && afContactIdxBBuf(newIdxs) >= 0));
	afContactIdxABuf	 = afContactIdxABuf(condIdxs);
	afContactIdxBBuf	 = afContactIdxBBuf(condIdxs);
	afContactWeightBuf   = afContactWeightBuf(condIdxs);
	afContactMassBuf	 = afContactMassBuf(condIdxs);
	afContactNormalXBuf  = afContactNormalXBuf(condIdxs);
	afContactNormalYBuf	 = afContactNormalYBuf(condIdxs);
	afContactFlagsBuf	 = afContactFlagsBuf(condIdxs);
	afContactMatFlagsBuf = afContactMatFlagsBuf(condIdxs);
	m_contactCount		 = condIdxs.elements();
	
	// update particle-body contacts
	condIdxs = newIdxs(af::where(afBodyContactIdxBuf(newIdxs) >= 0));
	afBodyContactIdxBuf		= afBodyContactIdxBuf(condIdxs);
	afBodyContactBodyIdxBuf	= afBodyContactBodyIdxBuf(condIdxs);
	afBodyContactFixtIdxBuf = afBodyContactFixtIdxBuf(condIdxs);
	afBodyContactWeightBuf  = afBodyContactWeightBuf(condIdxs);
	afBodyContactNormalXBuf = afBodyContactNormalXBuf(condIdxs);
	afBodyContactNormalYBuf = afBodyContactNormalYBuf(condIdxs);
	afBodyContactMassBuf	= afBodyContactMassBuf(condIdxs);
	m_bodyContactCount		= condIdxs.elements();

	// update pairs
	condIdxs = newIdxs(af::where(afPairIdxABuf(newIdxs) >= 0 && afPairIdxBBuf(newIdxs) >= 0));
	afPairIdxABuf	  = afPairIdxABuf(condIdxs);
	afPairIdxBBuf	  = afPairIdxBBuf(condIdxs);
	afPairFlagsBuf	  = afPairFlagsBuf(condIdxs);
	afPairStrengthBuf = afPairStrengthBuf(condIdxs);
	afPairDistanceBuf = afPairDistanceBuf(condIdxs);
	m_pairCount		  = condIdxs.elements();

	// update triads
	condIdxs = newIdxs(af::where(afTriadIdxABuf(newIdxs) >= 0 && afTriadIdxBBuf(newIdxs) >= 0 && afTriadIdxCBuf(newIdxs) >= 0));
	afTriadIdxABuf	   = afTriadIdxABuf(condIdxs);
	afTriadIdxBBuf	   = afTriadIdxBBuf(condIdxs);
	afTriadIdxCBuf	   = afTriadIdxCBuf(condIdxs);
	afTriadFlagsBuf	   = afTriadFlagsBuf(condIdxs);
	afTriadStrengthBuf = afTriadStrengthBuf(condIdxs);
	afTriadPAXBuf	   = afTriadPAXBuf(condIdxs);
	afTriadPAYBuf	   = afTriadPAYBuf(condIdxs);
	afTriadPBXBuf	   = afTriadPBXBuf(condIdxs);
	afTriadPBYBuf	   = afTriadPBYBuf(condIdxs);
	afTriadPCXBuf	   = afTriadPCXBuf(condIdxs);
	afTriadPCYBuf	   = afTriadPCYBuf(condIdxs);
	afTriadKABuf	   = afTriadKABuf(condIdxs);
	afTriadKBBuf	   = afTriadKBBuf(condIdxs);
	afTriadKCBuf	   = afTriadKCBuf(condIdxs);
	afTriadSBuf		   = afTriadSBuf(condIdxs);
	m_triadCount	   = condIdxs.elements();

	// Update lifetime indices.
	if (!afIdxByExpireTimeBuf.isempty())
		afIdxByExpireTimeBuf = afIdxByExpireTimeBuf(newIdxs);

	// update groups
	for each (const int32 groupIdx in m_realGroupIdxBuffer)
	{
		int32 firstIndex = newCount;
		int32 lastIndex = 0;
		bool modified = false;
		const af::array& groupPartIdxs = af::seq(m_groupFirstIdxBuf[groupIdx], m_groupLastIdxBuf[groupIdx] - 1);
		const af::array& newgroupPartIdxs = reorderedIdxs(groupPartIdxs);

		const af::array& valid = newgroupPartIdxs >= 0;
		const af::array& validIdxs = af::where(valid);
		const af::array& inValidIdxs = af::where(!valid);
		if (!validIdxs.isempty())
		{
			const af::array& validNewGroupPartIdxs = newgroupPartIdxs(validIdxs);
			firstIndex = af::min<int32>(validNewGroupPartIdxs);
			lastIndex = af::max<int32>(validNewGroupPartIdxs);
		}
		if (!inValidIdxs.isempty())
			modified = true;

		if (firstIndex < lastIndex)
		{
			m_groupFirstIdxBuf[groupIdx] = firstIndex;
			m_groupLastIdxBuf[groupIdx] = lastIndex;
			if (modified)
			{
				if (m_groupFlagsBuf[groupIdx] & b2_solidParticleGroup)
				{
					SetGroupFlags(groupIdx,
						m_groupFlagsBuf[groupIdx] |
						b2_particleGroupNeedsUpdateDepth);
				}
			}
		}
		else
		{
			m_groupFirstIdxBuf[groupIdx] = 0;
			m_groupLastIdxBuf[groupIdx] = 0;
			if (!(m_groupFlagsBuf[groupIdx] & b2_particleGroupCanBeEmpty))
			{
				SetGroupFlags(groupIdx,
					m_groupFlagsBuf[groupIdx] | b2_particleGroupWillBeDestroyed);
			}
		}
	}

	// update particle count
	m_count = newCount;
	m_allParticleFlags = allParticleFlags;
	m_needsUpdateAllParticleFlags = false;

	// destroy bodies with no particles
	for each (int32 groupIdx in m_realGroupIdxBuffer)
	{
		if (m_groupFlagsBuf[groupIdx] & b2_particleGroupWillBeDestroyed)
			DestroyParticleGroup(groupIdx);
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
	rotate(m_collisionLayerBuffer.begin() + start, m_collisionLayerBuffer.begin() + mid,
				m_collisionLayerBuffer.begin() + end);
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
	rotate(m_positionXBuffer.begin() + start, m_positionXBuffer.begin() + mid,
		m_positionXBuffer.begin() + end);
	rotate(m_positionYBuffer.begin() + start, m_positionYBuffer.begin() + mid,
		m_positionYBuffer.begin() + end);
	rotate(m_positionZBuffer.begin() + start, m_positionZBuffer.begin() + mid,
		m_positionZBuffer.begin() + end);
	rotate(m_velocityXBuffer.begin() + start, m_velocityXBuffer.begin() + mid,
		m_velocityXBuffer.begin() + end);
	rotate(m_velocityYBuffer.begin() + start, m_velocityYBuffer.begin() + mid,
		m_velocityYBuffer.begin() + end);
	rotate(m_groupIdxBuffer.begin() + start, m_groupIdxBuffer.begin() + mid,
		m_groupIdxBuffer.begin() + end);
	rotate(m_partMatIdxBuffer.begin() + start, m_partMatIdxBuffer.begin() + mid,
		m_partMatIdxBuffer.begin() + end);
	if (m_hasForce)
	{
		rotate(m_forceXBuffer.begin() + start, m_forceXBuffer.begin() + mid,
			m_forceXBuffer.begin() + end);
		rotate(m_forceYBuffer.begin() + start, m_forceYBuffer.begin() + mid,
			m_forceYBuffer.begin() + end);
	}
	if (!m_staticPressureBuf.empty())
	{
		rotate(m_staticPressureBuf.begin() + start,
			m_staticPressureBuf.begin() + mid,
			m_staticPressureBuf.begin() + end);
	}
	if (m_depthBuffer.data())
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
		int32 &idx = m_proxyIdxBuffer[k];
		idx = newIndices[idx];
	}

	// update contacts
	for (int32 k = 0; k < m_contactCount; k++)
	{
		m_contactIdxABuffer[k] = newIndices[m_contactIdxABuffer[k]];
		m_contactIdxBBuffer[k] = newIndices[m_contactIdxBBuffer[k]];
	}

	// update particle-body contacts
	for (int32 k = 0; k < m_bodyContactCount; k++)
	{
		int32& idx = m_bodyContactIdxBuffer[k];
		idx = newIndices[idx];
	}

	// update pairs
	for (int32 k = 0; k < m_pairCount; k++)
	{
		m_pairIdxABuf[k] = newIndices[m_pairIdxABuf[k]];
		m_pairIdxABuf[k] = newIndices[m_pairIdxABuf[k]];
	}

	// update triads
	for (int32 k = 0; k < m_triadCount; k++)
	{
		m_triadIdxABuf[k] = newIndices[m_triadIdxABuf[k]];
		m_triadIdxBBuf[k] = newIndices[m_triadIdxBBuf[k]];
		m_triadIdxCBuf[k] = newIndices[m_triadIdxCBuf[k]];
	}

	// update groups
	for each (const int32 groupIdx in m_realGroupIdxBuffer)
	{
		m_groupFirstIdxBuf[groupIdx] = newIndices[m_groupFirstIdxBuf[groupIdx]];
		m_groupLastIdxBuf[groupIdx] = newIndices[m_groupLastIdxBuf[groupIdx] - 1] + 1;
	}
}
void b2ParticleSystem::AFRotateBuffer(int32 start, int32 mid, int32 end)
{

	// move the particles assigned to the given group toward the end of array
	if (start == mid || mid == end)
		return;
	b2Assert(mid >= start && mid <= end);
	
	
	// Switch blocks between start->mid and mid->end
	const af::array& origIdxs = af::seq(start, end);
	const af::array& rotIdxs = af::join(afArrayDims, af::seq(mid, end), af::seq(start, mid));
	
	afFlagBuf(origIdxs)   = afFlagBuf(rotIdxs);
	afColLayBuf(origIdxs) = afColLayBuf(rotIdxs);

	if (afHasLastBodyContactBuf)
		afLastBodyContactStepBuf(origIdxs) = afLastBodyContactStepBuf(rotIdxs);

	if (afHasBodyContactCountBuf)
		afBodyContactCountBuf(origIdxs) = afBodyContactCountBuf(rotIdxs);
	
	if (afHasConsecutiveContactStepsBuf)
		afConsecutiveContactStepsBuf(origIdxs) = afConsecutiveContactStepsBuf(rotIdxs);
	
	afPosXBuf(origIdxs)		  = afPosXBuf(rotIdxs);
	afPosYBuf(origIdxs)		  = afPosYBuf(rotIdxs);
	afPosZBuf(origIdxs)		  = afPosZBuf(rotIdxs);
	afVelXBuf(origIdxs)		  = afVelXBuf(rotIdxs);
	afVelYBuf(origIdxs)		  = afVelYBuf(rotIdxs);
	afGroupIdxBuf(origIdxs)	  = afGroupIdxBuf(rotIdxs);
	afPartMatIdxBuf(origIdxs) = afPartMatIdxBuf(rotIdxs);

	if (m_hasForce)
	{
		afForceXBuf(origIdxs) = afForceXBuf(rotIdxs);
		afForceYBuf(origIdxs) = afForceYBuf(rotIdxs);
	}
	if (afHasStaticPresBuf)
		afStaticPressureBuf(origIdxs) = afStaticPressureBuf(rotIdxs);
	
	if (afHasDepthBuf)
		afDepthBuf(origIdxs) = afDepthBuf(rotIdxs);
	
	if (afHasColorBuf)
		afColorBuf(origIdxs) = afColorBuf(rotIdxs);
	
	if (afHasUserDataBuf)
		afUserDataBuf(origIdxs) = afUserDataBuf(rotIdxs);

	afHeatBuf(origIdxs)   = afHeatBuf(rotIdxs);
	afHealthBuf(origIdxs) = afHealthBuf(rotIdxs);

	// Update handle indices.
	if (afHasHandleIdxBuf)
	{
		afHandleIdxBuf(origIdxs) = afHandleIdxBuf(rotIdxs);
		const af::array& handleIdxs = afHandleIdxBuf(origIdxs);
		afHandleIdxBuf(origIdxs) = rotIdxs(handleIdxs);
	}

	if (afHasExpireTimeBuf)
	{
		afExpireTimeBuf(origIdxs) = afExpireTimeBuf(rotIdxs);
		// Update expiration time buffer indices.
		afIdxByExpireTimeBuf(origIdxs) = afIdxByExpireTimeBuf(rotIdxs);
	}

	// update proxies
	afProxyIdxBuf(origIdxs) = afProxyIdxBuf(rotIdxs);

	// update contacts
	afContactIdxABuf(origIdxs) = afContactIdxABuf(rotIdxs);
	afContactIdxBBuf(origIdxs) = afContactIdxBBuf(rotIdxs);

	// update particle-body contacts
	afBodyContactIdxBuf(origIdxs) = afBodyContactIdxBuf(rotIdxs);

	// update pairs
	afPairIdxABuf(origIdxs) = afPairIdxABuf(rotIdxs);
	afPairIdxBBuf(origIdxs) = afPairIdxBBuf(rotIdxs);

	// update triads
	afTriadIdxABuf(origIdxs) = afTriadIdxABuf(rotIdxs);
	afTriadIdxBBuf(origIdxs) = afTriadIdxBBuf(rotIdxs);
	afTriadIdxCBuf(origIdxs) = afTriadIdxCBuf(rotIdxs);

	// update groups
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
	for each (const int32 groupIdx in m_realGroupIdxBuffer)
	{
		m_groupFirstIdxBuf[groupIdx] = newIndices[m_groupFirstIdxBuf[groupIdx]];
		m_groupLastIdxBuf[groupIdx] = newIndices[m_groupLastIdxBuf[groupIdx] - 1] + 1;
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

void b2ParticleSystem::SetPositionBuffer(float* bufferX, float* bufferY,
	int32 capacity)
{
	m_positionXBuffer.assign(bufferX, bufferX + capacity);
	m_positionYBuffer.assign(bufferY, bufferY + capacity);
	//SetUserOverridableBuffer(&m_positionXBuffer, bufferX, capacity);
	//SetUserOverridableBuffer(&m_positionYBuffer, bufferY, capacity);
}

void b2ParticleSystem::SetVelocityBuffer(float* bufferX, float* bufferY,
	int32 capacity)
{
	m_velocityXBuffer.assign(bufferX, bufferX + capacity);
	m_velocityYBuffer.assign(bufferY, bufferY + capacity);
	//SetUserOverridableBuffer(&m_velocityXBuffer, bufferX, capacity);
	//SetUserOverridableBuffer(&m_velocityYBuffer, bufferY, capacity);
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
			RequestBuffer(m_colorBuffer, afHasColorBuf);
		}
		m_allParticleFlags |= newFlags;
	}
	*oldFlags = newFlags;
}
void b2ParticleSystem::AFSetParticleFlags(const af::array& idxs, uint32 newFlags)
{
	const af::array& oldFlags = afFlagBuf(idxs);


	// If any flags might be removed
	const af::array& condIdxs = af::where(oldFlags & ~newFlags);
	if (!condIdxs.isempty())
		m_needsUpdateAllParticleFlags = true;
		
	// If any flags were added
	if (~m_allParticleFlags & newFlags)
	{
		if (newFlags & b2_tensileParticle)
		{
			 AFRequestBuffer(afAccumulation2XBuf, afHasAccumulation2XBuf);
			 AFRequestBuffer(afAccumulation2YBuf, afHasAccumulation2YBuf);
		}
		if (newFlags & b2_colorMixingParticle)
		{
			AFRequestBuffer(afColorBuf, afHasColorBuf);
		}
		m_allParticleFlags |= newFlags;
	}
	afFlagBuf(idxs) = oldFlags | newFlags;
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
			RequestBuffer(m_colorBuffer, afHasColorBuf);
		}
		m_allParticleFlags |= newFlags;
	}
	m_flagsBuffer[index] |= newFlags;
}

void b2ParticleSystem::SetGroupFlags(
	int32 groupIdx, uint32 newFlags)
{
	uint32& oldFlags = m_groupFlagsBuf[groupIdx];
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
			//m_depthBuffer = RequestBuffer(m_depthBuffer);
			m_depthBuffer.resize(m_count);
		}
		m_allGroupFlags |= newFlags;
	}
	oldFlags = newFlags;
}
void b2ParticleSystem::AFSetGroupFlags(const af::array& afGroupIdxs, af::array& afNewFlags)
{
	//TODO

	af::array afOldFlags = afGroupFlagsBuf(afGroupIdxs);
	afNewFlags = afNewFlags | (afOldFlags & b2_particleGroupInternalMask != 0);

	// If the b2_solidParticleGroup flag changed schedule depth update.
	af::array afOldXorNew = afOldFlags ^ afNewFlags;
	afNewFlags(afOldXorNew != 0) = afNewFlags(afOldXorNew != 0) | b2_particleGroupNeedsUpdateDepth;

	// If any flags might be removed
	af::array afOldWithoutNew = afOldFlags & (afNewFlags ^ MAXUINT32);
	if (af::anyTrue<bool>(afOldWithoutNew))
		m_needsUpdateAllGroupFlags = true;

	af::array afAktualNewFlags = ~m_allGroupFlags & afNewFlags;

	//TODO after bitor sum
	/*
	if (~m_allGroupFlags & newFlags)
	{
		// If any flags were added
		if (newFlags & b2_solidParticleGroup)
		{
			//m_depthBuffer = RequestBuffer(m_depthBuffer);
			m_depthBuffer.resize(m_count);
		}
		m_allGroupFlags |= newFlags;
	}
	oldFlags = newFlags;*/
}


static inline bool IsSignificantForce(float32 forceX, float32 forceY)
{
	return forceX != 0 || forceY != 0;
}
static inline af::array AFIsSignificantForce(const af::array& forceX, const af::array& forceY)
{
	return forceX != 0 || forceY != 0;
}

inline bool b2ParticleSystem::ForceCanBeApplied(uint32 flags) const
{
	return !(flags & b2_wallParticle);
}
inline af::array b2ParticleSystem::AFForceCanBeApplied(const af::array& flags) const
{
	return !(flags & b2_wallParticle);
}

inline void b2ParticleSystem::PrepareForceBuffer()
{
	if (!m_hasForce)
	{
		m_forceXBuffer.resize(m_particleBufferSize);
		m_forceYBuffer.resize(m_particleBufferSize);
		std::fill(m_forceXBuffer.begin(), m_forceXBuffer.end(), 0.0f);
		std::fill(m_forceYBuffer.begin(), m_forceYBuffer.end(), 0.0f);
		m_hasForce = true;
	}
}
inline void b2ParticleSystem::AFPrepareForceBuffer()
{
	if (!m_hasForce)
	{
		afForceXBuf = af::constant(0, m_count);
		afForceYBuf = af::constant(0, m_count);
		m_hasForce = true;
	}
}

void b2ParticleSystem::ApplyForce(int32 firstIndex, int32 lastIndex,
									float32 forceX, float32 forceY)
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
	float32 distributedForceX = forceX / (float32)(lastIndex - firstIndex);
	float32 distributedForceY = forceY / (float32)(lastIndex - firstIndex);
	if (IsSignificantForce(distributedForceX, distributedForceY))
	{
		PrepareForceBuffer();

		// Distribute the force over all the particles.
		for (int32 i = firstIndex; i < lastIndex; i++)
		{
			m_forceXBuffer[i] += distributedForceX;
			m_forceYBuffer[i] += distributedForceY;
		}
	}
}
void b2ParticleSystem::AFApplyForce(int32 firstIndex, int32 lastIndex,
	float32 forceX, float32 forceY)
{
	// Early out if force does nothing (optimization).
	int32 count = lastIndex - firstIndex;
	float32 distributedForceX = forceX / (float32)(count);
	float32 distributedForceY = forceY / (float32)(count);

	if (!m_hasForce)
	{
		afForceXBuf = af::constant(0.0f, m_count);
		afForceYBuf = af::constant(0.0f, m_count);
		m_hasForce = true;
	}

	if (IsSignificantForce(distributedForceX, distributedForceY))
	{
		// Distribute the force over all the particles.
		af::array idxs = af::seq(firstIndex, lastIndex);
		afForceXBuf(idxs) += distributedForceX;
		afForceYBuf(idxs) += distributedForceY;
	}
}

void b2ParticleSystem::ParticleApplyForce(int32 index, float32 forceX, float32 forceY)
{
	if (IsSignificantForce(forceX, forceY) &&
		ForceCanBeApplied(m_flagsBuffer[index]))
	{
		PrepareForceBuffer();
		m_forceXBuffer[index] += forceX;
		m_forceYBuffer[index] += forceY;
	}
}
void b2ParticleSystem::AFParticleApplyForce(const af::array& idxs, const af::array& forceX, const af::array& forceY)
{
	const af::array& condIdxs = af::where(AFIsSignificantForce(forceX, forceY) 
								   && AFForceCanBeApplied(afFlagBuf(idxs)));
	if (!condIdxs.isempty())
	{
		const af::array& sysIdxs = idxs(condIdxs);
		AFPrepareForceBuffer();
		afForceXBuf(sysIdxs) += forceX(condIdxs);
		afForceYBuf(sysIdxs) += forceY(condIdxs);
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
		m_velocityXBuffer[i] += vel.x;
		m_velocityYBuffer[i] += vel.y;
	}
}

void b2ParticleSystem::QueryAABB(b2QueryCallback* callback,
								 const b2AABB& aabb) const
{
	if (m_proxyIdxBuffer.empty())
	{
		return;
	}
	auto beginIt = m_proxyTagBuffer.begin();
	auto endIt = m_proxyTagBuffer.begin() + m_count;

	auto firstIt = lower_bound(beginIt, endIt, computeTag(
		m_inverseDiameter * aabb.lowerBound.x,
		m_inverseDiameter * aabb.lowerBound.y));
	auto lastIt = upper_bound(firstIt, endIt, computeTag(
		m_inverseDiameter * aabb.upperBound.x,
		m_inverseDiameter * aabb.upperBound.y));

	const int lastProxyPos = lastIt - beginIt;
	for (int proxyPos = firstIt - beginIt; proxyPos < lastProxyPos; ++proxyPos)
	{
		int32 i = m_proxyIdxBuffer[proxyPos];
		const b2Vec2 p = b2Vec2(m_positionXBuffer[i], m_positionYBuffer[i]);
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
void b2ParticleSystem::AFQueryAABB(afQueryCallback* callback,
	const b2AABB& aabb) const
{
	if (afProxyIdxBuf.isempty())
		return;

	uint32 lowerTag = computeTag(m_inverseDiameter * aabb.lowerBound.x,
								 m_inverseDiameter * aabb.lowerBound.y);
	uint32 upperTag = computeTag(m_inverseDiameter * aabb.upperBound.x,
								 m_inverseDiameter * aabb.upperBound.y);

	const af::array& tagsBetweenIdxs = af::where(afProxyTagBuf >= lowerTag &&
												 afProxyTagBuf <= upperTag);

	const af::array& i = afProxyIdxBuf(tagsBetweenIdxs);
	const af::array& px = afPosXBuf(i);
	const af::array& py = afPosYBuf(i);

	af::array k = af::where(aabb.lowerBound.x < px && px < aabb.upperBound.x &&
							aabb.lowerBound.y < py && py < aabb.upperBound.y);
	if (!k.isempty())
	{
		k = af::where(callback->AFReportParticle(this, k));
		if (!k.isempty())
		{
			//here there could be something done with particles
			return;
		}
	}
}

void b2ParticleSystem::QueryShapeAABB(b2QueryCallback* callback,
									  const b2Shape& shape,
									  const b2Transform& xf) const
{
	b2AABB aabb;
	shape.ComputeAABB(&aabb, xf, 0);
	QueryAABB(callback, aabb);
}

void b2ParticleSystem::RayCast(b2RayCastCallback* callback,
							   const b2Vec2& point1,
							   const b2Vec2& point2) const
{
	if (m_proxyIdxBuffer.empty())
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
		b2Vec2 p = point1 - b2Vec2(m_positionXBuffer[i], m_positionYBuffer[i]);
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
		int32 a = m_contactIdxABuffer[k];
		int32 b = m_contactIdxBBuffer[k];
		b2Vec2 n = b2Vec2(m_contactNormalXBuffer[k], m_contactNormalYBuffer[k]);
		b2Vec2 v = b2Vec2(m_velocityXBuffer[b] - m_velocityXBuffer[a], m_velocityYBuffer[b] - m_velocityYBuffer[a]);
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
void b2ParticleSystem::AFSetStuckThreshold(int32 steps)
{
	m_stuckThreshold = steps;

	if (steps > 0)
	{
		AFRequestBuffer(afLastBodyContactStepBuf, afHasLastBodyContactBuf);
		AFRequestBuffer(afBodyContactCountBuf, afHasBodyContactCountBuf);
		AFRequestBuffer(afConsecutiveContactStepsBuf, afHasConsecutiveContactStepsBuf);
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
