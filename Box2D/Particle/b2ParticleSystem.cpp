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
	b2ParticleBodyContactRemovePredicate(b2World& world, ParticleSystem& system,
										 int32* discarded)
		: m_world(world), m_system(system), m_lastIndex(-1), m_currentContacts(0),
		  m_discarded(discarded) {}

	/*bool operator()(const Particle::BodyContact& contact)
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
		Vec2 n = contact.normal;
		// weight is 1-(inv(diameter) * distance)
		n *= m_system->m_particleDiameter * (1 - contact.weight);
		Vec2 pos = n + (m_system->m_positionBuffer[particleIdx]);
	
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
				Vec2 normal;
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
	const ParticleSystem& m_system;
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
	void Initialize(const vector<Particle::BodyContact>& bodyContacts,
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

ParticleSystem::InsideBoundsEnumerator::InsideBoundsEnumerator(
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

int32 ParticleSystem::InsideBoundsEnumerator::GetNext()
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
	return INVALID_IDX;
}

ParticleSystem::ParticleSystem(b2World& world, b2TimeStep& step, vector<Body>& bodyBuffer, vector<Fixture>& fixtureBuffer) :
	m_handleAllocator(MIN_PART_CAPACITY),
	m_ampGroups(b2_minGroupBufferCapacity, amp::accelView()),
	m_ampGroupHasAlive(b2_minGroupBufferCapacity, amp::accelView()),

	// Box2D
	m_ampBodies(TILE_SIZE, amp::accelView()),
	m_ampBodyParticles(TILE_SIZE, amp::accelView()),
	m_ampFixtures(TILE_SIZE, amp::accelView()),
	m_ampChainShapes(TILE_SIZE, amp::accelView()),
	m_ampCircleShapes(TILE_SIZE, amp::accelView()),
	m_ampEdgeShapes(TILE_SIZE, amp::accelView()),
	m_ampPolygonShapes(TILE_SIZE, amp::accelView()),

	// Particles
	m_buffers(MIN_PART_CAPACITY),
	m_ampParts(amp::accelView()),
	m_ampContacts(amp::accelView(), m_ampParts),
	m_ampBodyContacts(amp::accelView(), m_ampParts),
	m_ampGroundContacts(amp::accelView(), m_ampParts),

	// Materials
	m_mats(amp::accelView()),

	// pairs and triads
	m_ampPairs(TILE_SIZE, amp::accelView()),
	m_ampTriads(TILE_SIZE, amp::accelView()),
	m_world(world),
	m_step(step)
{
	m_paused = false;
	m_timestamp = 0;
	m_allFlags = 0;
	m_needsUpdateAllParticleFlags = true;
	m_allGroupFlags = 0;
	m_needsUpdateAllGroupFlags = false;
	m_hasForce = false;
	m_hasDepth = false;
	m_iteration = 0;

	SetDensity(1.0f);
	SetRadius(1.0f);
	SetMaxParticleCount(0);

	m_freeGroupIdxs.reserve(256);

	m_groupCount = 0;
	m_groupCapacity = 0;
	m_pairCount = 0;
	m_pairCapacity = 0;
	m_triadCount = 0;
	m_triadCapacity = 0;

	m_hasColorBuf					 = false;
	hasHandleIndexBuffer			 = false;
	hasStaticPressureBuf			 = false;
	hasAccumulation2Buf				 = false;

	b2Assert(1.0f / 60.0f > 0.0f);

	m_timeElapsed = 0;
	m_expirationTimeBufferRequiresSorting = false;
}

ParticleSystem::~ParticleSystem()
{
	for (int i = 0; i < m_groupCount; i++)
		DestroyGroup(i, 0, true);
	amp::uninitialize();
}

template <typename T> void ParticleSystem::FreeBuffer(T** b, int capacity)
{
	if (*b == NULL)
		return;

	m_world.m_blockAllocator.Free(*b, sizeof(**b) * capacity);
	*b = NULL;
}

// Free buffer, if it was allocated with b2World's block allocator
template <typename T> void ParticleSystem::FreeUserOverridableBuffer(
	UserOverridableBuffer<T>* b)
{
	if (b->userSuppliedCapacity == 0)
		FreeBuffer(&b->data, m_capacity);
}

// Reallocate a buffer
template <typename T> T* ParticleSystem::ReallocateBuffer(
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
template <typename T> T* ParticleSystem::ReallocateBuffer(
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
template <typename T> T* ParticleSystem::ReallocateBuffer(
	UserOverridableBuffer<T>* buffer, int32 oldCapacity, int32 newCapacity,
	bool deferred)
{
	b2Assert(newCapacity > oldCapacity);
	return ReallocateBuffer(buffer->data, buffer->userSuppliedCapacity,
							oldCapacity, newCapacity, deferred);
}

template <typename T>
void ParticleSystem::RequestBuffer(ampArray<T>& a, bool& hasBuffer)
{
	if (!hasBuffer)
	{
		amp::resize(a, m_ampParts.m_capacity);
		hasBuffer = true;
	}
}

template<typename F>
void ParticleSystem::ForEachGroup(const F& function) const
{
	for (int32 i = 0; i < m_groupCount; i++)
	{
		ParticleGroup& group = m_groupBuffer[i];
		if (group.m_firstIndex != INVALID_IDX)
			function(group);
	}
}


template<typename F>
inline void ParticleSystem::ForEachPair(const F& function) const
{
	amp::forEach(m_pairCount, [=](const int32 i) restrict(amp)
	{
		function(i);
	});
}
template<typename F>
inline void ParticleSystem::ForEachTriad(const F& function) const
{
	amp::forEach(m_triadCount, [=](const int32 i) restrict(amp)
	{
		function(i);
	});
}

void ParticleSystem::ResizeParticleBuffers(int32 size)
{
	if (!m_ampParts.Resize(size)) return;
	m_buffers.Resize(m_ampParts.m_capacity);
	m_ampContacts.Resize(m_ampParts.m_capacity * MAX_CONTACTS_PER_PARTICLE);
	m_ampBodyContacts.Resize(m_ampParts.m_capacity * MAX_BODY_CONTACTS_PER_PARTICLE);
	m_ampGroundContacts.Resize(m_ampParts.m_capacity);
	if (m_resizeCallback) m_resizeCallback(m_ampParts.m_capacity);
}

void ParticleSystem::ResizeGroupBuffers(int32 size)
{
	if (!AdjustCapacityToSize(m_groupCapacity, size, b2_minGroupBufferCapacity)) return;
	m_groupBuffer.resize(m_groupCapacity);
	m_groupHasAlive.resize(m_groupCapacity);
	amp::resize(m_ampGroups, m_groupCapacity, m_groupCount);
	amp::resize(m_ampGroupHasAlive, m_groupCapacity);

	m_groupExtent = ampExtent(m_groupCount);
}

void ParticleSystem::DestroyAllParticles()
{
	m_allFlags = 0;
	ResizeParticleBuffers(0);
	m_ampParts.SetD11Buffers(nullptr);

	m_groupCount = 0;
	m_freeGroupIdxs.clear();
	m_zombieRanges.clear();
	ResizeGroupBuffers(0);
}

pair<int32, int32> ParticleSystem::CreateParticlesWithPositions(const ParticleGroup::Def& groupDef)
{
	if (!groupDef.particleCount) return pair<uint32, uint32>(INVALID_IDX, INVALID_IDX);
	uint32 writeIdx = GetWriteIdx(groupDef.particleCount);
	const auto& mat = m_mats.m_vector[groupDef.matIdx];
	const uint32 flags = groupDef.flags | mat.m_flags;
	const bool hasColors = !groupDef.colors.empty();
	for (uint32 i = 0, wi = writeIdx; i < groupDef.particleCount; i++, wi++)
	{
		m_buffers.groupIdx[wi] = groupDef.idx;
		m_buffers.flags[wi] = flags;
		const Vec3 p = groupDef.positions[i];
		m_buffers.position[wi] = b2Mul3D(groupDef.transform, p);
		m_buffers.velocity[wi] = groupDef.linearVelocity + Vec3(
			b2Cross(groupDef.angularVelocity, (Vec2)p - groupDef.transform.p), 0);
		m_buffers.heat[wi] = groupDef.heat;
		m_buffers.health[wi] = groupDef.health;
		m_buffers.force[wi].SetZero();
		m_buffers.matIdx[wi] = groupDef.matIdx;
		m_buffers.mass[wi] = mat.m_mass;
		m_buffers.invMass[wi] = mat.m_invMass;
		m_buffers.staticPressure[wi] = 0;

		m_buffers.depth[wi] = 0;
		m_buffers.color[wi] = hasColors ? groupDef.colors[i] : groupDef.color;
	}
	m_allFlags |= flags;
	return pair<uint32, uint32>(writeIdx, writeIdx + groupDef.particleCount);
}

pair<int32, int32> ParticleSystem::CreateParticlesStrokeShapeForGroup(
	const b2Shape& shape, const ParticleGroup::Def& groupDef, const b2Transform& xf)
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
	//	Vec2 d = edge.m_vertex2 - edge.m_vertex1;
	//	float32 edgeLength = d.Length();
	//	while (positionOnEdge < edgeLength)
	//	{
	//		Vec3 p(edge.m_vertex1 + positionOnEdge / edgeLength * d, 0);
	//		CreateParticleForGroup(groupDef, xf, p);
	//		positionOnEdge += stride;
	//	}
	//	positionOnEdge -= edgeLength;
	//}
	//uint32 firstIdx = CreateParticlesForGroup(cnt, groupDef, xf, positions);
	//return pair<uint32, uint32>(firstIdx, firstIdx + cnt);
	return pair<uint32, uint32>(0, 0);
}

pair<int32, int32> ParticleSystem::CreateParticlesFillShapeForGroup(
	const b2Shape& shape, ParticleGroup::Def& groupDef)
{
	float32 stride = groupDef.stride ? groupDef.stride : GetParticleStride();
	b2Transform identity;
	identity.SetIdentity();
	b2AABB aabb;
	b2Assert(shape.GetChildCount() == 1);
	shape.ComputeAABB(aabb, identity, 0);
	float32 startY = floorf(aabb.lowerBound.y / stride) * stride;
	float32 startX = floorf(aabb.lowerBound.x / stride) * stride;
	float32 z = 0; // groupDef.transform.z;
	groupDef.positions.reserve((int32)(((aabb.upperBound.y - startY) * (aabb.upperBound.x - startX)) / (stride * stride)));
	for (float32 y = startY; y < aabb.upperBound.y; y += stride)
		for (float32 x = startX; x < aabb.upperBound.x; x += stride)
			if (const Vec3 p(x, y, z); shape.TestPoint(identity, p))
				groupDef.positions.push_back(p);
	groupDef.particleCount = groupDef.positions.size();
	return CreateParticlesWithPositions(groupDef);
}

pair<int32, int32> ParticleSystem::CreateParticlesWithShapeForGroup(
	ParticleGroup::Def& gd)
{
	const b2Shape& shape = m_world.GetShape(gd.shapeType, gd.shapeIdx);
	switch (shape.m_type) {
		case b2Shape::e_edge:
		case b2Shape::e_chain:
			return CreateParticlesStrokeShapeForGroup(shape, gd, gd.transform);
		case b2Shape::e_polygon:
		case b2Shape::e_circle:
			return CreateParticlesFillShapeForGroup(shape, gd);
		default:
			b2Assert(false);
	}
	return pair<int32, int32>(INVALID_IDX, INVALID_IDX);
}

int32 ParticleSystem::CreateGroup(ParticleGroup::Def& groupDef)
{
	if (m_world.IsLocked()) return INVALID_IDX;

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

	pair<int32, int32> firstAndLastIdx;
	if (groupDef.shapeIdx != INVALID_IDX)
		firstAndLastIdx = CreateParticlesWithShapeForGroup(groupDef);
	else if (groupDef.particleCount)
		firstAndLastIdx = CreateParticlesWithPositions(groupDef);
	

	const Particle::Mat& mat = m_mats.m_vector[groupDef.matIdx];
	ParticleGroup& group = m_groupBuffer[groupDef.idx];
	group.m_firstIndex = firstAndLastIdx.first;
	group.m_lastIndex = firstAndLastIdx.second;
	group.m_strength = mat.m_strength;
	group.m_collisionGroup = groupDef.collisionGroup;
	group.m_matIdx = groupDef.matIdx;
	group.m_transform = groupDef.transform;
	group.m_timestamp = groupDef.timestamp;
	SetGroupFlags(group, groupDef.groupFlags);

	Particle::CopyBufferRangeToAmpArrays(m_buffers, m_ampParts, group.m_firstIndex, group.m_lastIndex);
	amp::copy(group, m_ampGroups, groupDef.idx);

	// Create pairs and triads between particles in the group->
	// ConnectionFilter filter;
	// UpdateContacts(true);
	// UpdatePairsAndTriads(firstIndex, lastIndex, filter);

	return groupDef.groupIdx;
}

void ParticleSystem::InitializeParticleLists(
	const ParticleGroup& group, ParticleListNode* nodeBuffer)
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

ParticleSystem::ParticleListNode* ParticleSystem::FindLongestParticleList(
	const ParticleGroup& group, ParticleListNode* nodeBuffer)
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

void ParticleSystem::MergeZombieParticleListNodes(
	const ParticleGroup& group, ParticleListNode* nodeBuffer,
	ParticleListNode* survivingList) const
{
	int32 particleCount = group.GetParticleCount();
	for (int32 i = 0; i < particleCount; i++)
	{
		ParticleListNode* node = &nodeBuffer[i];
		if (node != survivingList &&
			(m_buffers.flags[node->index] & Particle::Flag::Zombie))
		{
			MergeParticleListAndNode(survivingList, node);
		}
	}
}

void ParticleSystem::MergeParticleListAndNode(
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

void ParticleSystem::CreateParticleGroupsFromParticleList(
	const ParticleGroup& group, ParticleListNode* nodeBuffer,
	const ParticleListNode* survivingList)
{
	int32 particleCount = group.GetParticleCount();
	ParticleGroup::Def def;
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
			uint32& flags = m_buffers.flags[oldIndex];
			b2Assert(!(flags & Particle::Flag::Zombie));
			int32 newIndex = CloneParticle(oldIndex, newGroupIdx);
			flags = Particle::Flag::Zombie;
			node->index = newIndex;
		}
	}
}

void ParticleSystem::UpdatePairsAndTriadsWithParticleList(
	const ParticleGroup& group, const ParticleListNode* nodeBuffer)
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

int32 ParticleSystem::CloneParticle(int32 oldIndex, int32 groupIdx)
{
	Particle::Def def;
	def.flags	  = m_buffers.flags[oldIndex];
	def.position  = m_buffers.position[oldIndex];
	def.velocity  = m_buffers.velocity[oldIndex];
	def.heat	  = m_buffers.heat[oldIndex];
	def.health	  = m_buffers.health[oldIndex];
	def.color     = m_buffers.color[oldIndex];
	def.groupIdx  = groupIdx;
	def.matIdx    = m_buffers.matIdx[oldIndex];
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
	//if (!m_buffers.bodyContactCnt.empty())
	//{
	//	m_buffers.bodyContactCnt[newIndex] =
	//		m_buffers.bodyContactCnt[oldIndex];
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
	//if (!m_buffers.staticPressure.empty())
	//{
	//	m_buffers.staticPressure[newIndex] = m_buffers.staticPressure[oldIndex];
	//}
	//m_buffers.depth[newIndex] = m_buffers.depth[oldIndex];
	//if (!m_expireTimeBuf.empty())
	//{
	//	m_expireTimeBuf[newIndex] = m_expireTimeBuf[oldIndex];
	//}
	//return newIndex;
	return 0;
}

void ParticleSystem::UpdatePairsAndTriadsWithReactiveParticles()
{
	if (!(m_allFlags & Particle::Flag::Reactive)) return;

	AmpUpdatePairsAndTriads(0, m_ampParts.m_count);

	auto flags = m_ampParts.m_flags.GetView();
	const uint32 remReactiveFlag = ~Particle::Flag::Reactive;
	m_ampParts.ForEach([=](const int32 i) restrict(amp)
	{
		flags[i] &= remReactiveFlag;
	});
	m_ampCopyFutTriads.set(amp::copyAsync(m_triadBuffer, m_ampTriads, m_triadCount));
}

static bool ParticleCanBeConnected(uint32 flags, const ParticleGroup& group, int32 groupIdx)
{
	return
		(flags & Particle::Mat::k_wallOrSpringOrElasticFlags) ||
		(groupIdx != INVALID_IDX && group.HasFlag(ParticleGroup::Flag::Rigid));
}

void ParticleSystem::AmpUpdatePairsAndTriads(
	int32 firstIndex, int32 lastIndex)
{
	class ReactiveFilter : public ConnectionFilter
	{
		bool IsNecessary(int32 index) const
		{
			return (m_flagsBuffer[index] & Particle::Flag::Reactive) != 0;
		}
		const uint32* m_flagsBuffer;
	public:
		ReactiveFilter(uint32* flagsBuffer)
		{
			m_flagsBuffer = flagsBuffer;
		}
	} filter(m_buffers.flags.data());

	// Create pairs or triads.
	// All particles in each pair/triad should satisfy the following:
	// * firstIndex <= index < lastIndex
	// * don't have Particle::Flag::b2_zombieParticle
	// * ParticleCanBeConnected returns true
	// * ShouldCreatePair/ShouldCreateTriad returns true
	// Any particles in each pair/triad should satisfy the following:
	// * filter.IsNeeded returns true
	// * have one of k_pairFlags/k_triadsFlags
	b2Assert(firstIndex <= lastIndex);

	if (m_allFlags & Particle::Mat::k_pairFlags)
	{
		auto particleCanBeConnected = [=](uint32 flags, const ParticleGroup& group, int32 groupIdx) restrict(amp) -> bool
		{
			return flags & Particle::Mat::k_wallOrSpringOrElasticFlags ||
				(groupIdx != INVALID_IDX && group.HasFlag(ParticleGroup::Flag::Rigid));
		};
		auto flags = m_ampParts.m_flags.GetConstView();
		auto groupIdxs = m_ampParts.m_groupIdx.GetConstView();
		auto groups = GetConstGroups();
		auto positions = m_ampParts.m_position.GetConstView();
		auto& oldPairs = m_ampPairs;
		auto contactIdxs = m_ampContacts.m_idx.GetConstView();
		auto contacts = m_ampContacts.m_perTile.GetConstView();
		const int32 contactsPerTile = m_ampContacts.m_maxPerTile;
		ampArray<int32> cnts(m_ampContacts.m_count, amp::accelView());
		amp::fill(cnts, 0);
		amp::forEach(m_ampContacts.m_count, [=, &oldPairs, &cnts](const int32 i) restrict(amp)
		{
			const int32 contactIdx = contactIdxs[i];
			const Particle::Contact& contact = contacts[contactIdx / contactsPerTile][contactIdx % contactsPerTile];
			const int32 a = contact.idxA;
			const int32 b = contact.idxB;
			const uint32 f = contact.flags;
			const int32 groupAIdx = groupIdxs[a];
			const int32 groupBIdx = groupIdxs[b];
			const ParticleGroup& groupA = groups[groupAIdx];
			const ParticleGroup& groupB = groups[groupBIdx];
			if (!(f & Particle::Flag::Zombie) &&
				(f & Particle::Mat::k_pairFlags) &&
				(f & Particle::Flag::Reactive) &&
				particleCanBeConnected(flags[a], groupA, groupAIdx) &&
				particleCanBeConnected(flags[b], groupB, groupBIdx))
			{
				cnts[i] = 1;
				b2ParticlePair& pair = oldPairs[i];
				pair.indexA = a;
				pair.indexB = b;
				pair.flags = contact.flags;
				pair.strength = b2Min(
					groupAIdx != INVALID_IDX ? groupA.m_strength : 1,
					groupBIdx != INVALID_IDX ? groupB.m_strength : 1);
				pair.distance = b2Distance(positions[a], positions[b]);
			}
		});
		ampArray<b2ParticlePair> newPairs(oldPairs.extent, amp::accelView());
		m_pairCount = amp::scan(ampArrayView<const int32>(cnts), m_ampContacts.m_count,
			[=, &newPairs, &oldPairs](const int32 i, const int32 wi) restrict(amp)
		{
			newPairs[wi] = oldPairs[i];
		});
		oldPairs = newPairs;
	}
	//if (m_allFlags & Particle::Mat::k_triadFlags)
	//{
	//	b2VoronoiDiagram diagram(
	//		&m_world.m_stackAllocator, lastIndex - firstIndex);
	//	for (int32 i = firstIndex; i < lastIndex; i++)
	//	{
	//		uint32 flags = m_buffers.flags[i];
	//		int32 groupIdx = m_buffers.groupIdx[i];
	//		if (!(flags & Particle::Flag::Zombie) &&
	//			ParticleCanBeConnected(m_buffers.flags[i], m_groupBuffer[groupIdx], groupIdx))
	//		{
	//			diagram.AddGenerator(Vec2(m_buffers.position[i]), i, m_buffers.flags[i] & Particle::Flag::Reactive);
	//		}
	//	}
	//	float32 stride = GetParticleStride();
	//	diagram.Generate(stride / 2, stride * 2);
	//	class UpdateTriadsCallback : public b2VoronoiDiagram::NodeCallback
	//	{
	//		void operator()(int32 a, int32 b, int32 c)
	//		{
	//			uint32 af = m_system->m_buffers.flags[a];
	//			uint32 bf = m_system->m_buffers.flags[b];
	//			uint32 cf = m_system->m_buffers.flags[c];
	//			if (((af | bf | cf) & Particle::Mat::k_triadFlags) &&
	//				m_filter->ShouldCreateTriad(a, b, c))
	//			{
	//				const Vec3& pa = m_system->m_buffers.position[a];
	//				const Vec3& pb = m_system->m_buffers.position[b];
	//				const Vec3& pc = m_system->m_buffers.position[c];
	//				Vec2 dab = pa - pb;
	//				Vec2 dbc = pb - pc;
	//				Vec2 dca = pc - pa;
	//				float32 maxDistanceSquared = b2_maxTriadDistanceSquared *
	//					m_system->m_squaredDiameter;
	//				if (b2Dot(dab, dab) > maxDistanceSquared ||
	//					b2Dot(dbc, dbc) > maxDistanceSquared ||
	//					b2Dot(dca, dca) > maxDistanceSquared)
	//					return;
	//				int32 groupAIdx = m_system->m_buffers.groupIdx[a];
	//				int32 groupBIdx = m_system->m_buffers.groupIdx[b];
	//				int32 groupCIdx = m_system->m_buffers.groupIdx[c];
	//				ParticleGroup & groupA = m_system->m_groupBuffer[groupAIdx];
	//				ParticleGroup & groupB = m_system->m_groupBuffer[groupBIdx];
	//				ParticleGroup & groupC = m_system->m_groupBuffer[groupCIdx];
	//				int32 & triadCount = m_system->m_triadCount;
	//				m_system->ResizeTriadBuffers(triadCount);
	//				b2ParticleTriad & triad = m_system->m_triadBuffer[triadCount];
	//				triadCount++;
	//				triad.indexA = a;
	//				triad.indexB = b;
	//				triad.indexC = c;
	//				triad.flags = af | bf | cf;
	//				triad.strength = b2Min(b2Min(
	//					groupAIdx != INVALID_IDX ? groupA.m_strength : 1,
	//					groupBIdx != INVALID_IDX ? groupB.m_strength : 1),
	//					groupCIdx != INVALID_IDX ? groupC.m_strength : 1);
	//				Vec2 midPoint = (float32)1 / 3 * (pa + pb + pc);
	//				triad.pa = midPoint - pa;
	//				triad.pb = midPoint - pb;
	//				triad.pc = midPoint - pc;
	//				triad.ka = -b2Dot(dca, dab);
	//				triad.kb = -b2Dot(dab, dbc);
	//				triad.kc = -b2Dot(dbc, dca);
	//				triad.s = b2Cross2D(pa, pb) + b2Cross2D(pb, pc) + b2Cross2D(pc, pa);
	//			}
	//		}
	//		ParticleSystem * m_system;
	//		const ConnectionFilter * m_filter;
	//	public:
	//		UpdateTriadsCallback(
	//			ParticleSystem * system, const ConnectionFilter * filter)
	//		{
	//			m_system = system;
	//			m_filter = filter;
	//		}
	//	} callback(this, &filter);
	//	diagram.GetNodes(callback);


	//	concurrency::parallel_sort(
	//		m_triadBuffer.begin(), m_triadBuffer.end(), CompareTriadIndices);
	//	std::unique(m_triadBuffer.begin(), m_triadBuffer.end(), MatchTriadIndices);
	//	m_triadCount = m_triadBuffer.size();
	//}
}

bool ParticleSystem::ComparePairIndices(
	const b2ParticlePair& a, const b2ParticlePair& b)
{
	int32 diffA = a.indexA - b.indexA;
	if (diffA != 0) return diffA < 0;
	return a.indexB < b.indexB;
}

bool ParticleSystem::MatchPairIndices(
	const b2ParticlePair& a, const b2ParticlePair& b)
{
	return a.indexA == b.indexA && a.indexB == b.indexB;
}

bool ParticleSystem::CompareTriadIndices(
	const b2ParticleTriad& a, const b2ParticleTriad& b)
{
	int32 diffA = a.indexA - b.indexA;
	if (diffA != 0) return diffA < 0;
	int32 diffB = a.indexB - b.indexB;
	if (diffB != 0) return diffB < 0;
	return a.indexC < b.indexC;
}

bool ParticleSystem::MatchTriadIndices(
	const b2ParticleTriad& a, const b2ParticleTriad& b)
{
	return a.indexA == b.indexA && a.indexB == b.indexB && a.indexC == b.indexC;
}

// Only called from SolveZombie() or JoinParticleGroups().
void ParticleSystem::DestroyGroup(int32 groupIdx, int32 timestamp, bool destroyParticles)
{
	ParticleGroup& group = m_groupBuffer[groupIdx];
	if ((timestamp != INVALID_IDX && timestamp != group.m_timestamp) ||
		group.m_firstIndex == INVALID_IDX)
		return;

	AddZombieRange(group.m_firstIndex, group.m_lastIndex);
	if (destroyParticles)
	{
		auto flags = m_ampParts.m_flags.GetView();
		amp::forEach(group.m_firstIndex, group.m_lastIndex, [=](const int32 i) restrict(amp)
		{
			flags[i] = Particle::Flag::Zombie;
		});
	}
	//if (m_world.m_destructionListener)
	//	m_world.m_destructionListener->SayGoodbye(group);

	//SetGroupFlags(group, 0);
	m_needsUpdateAllGroupFlags = true;
	// for (int32 i = group.m_firstIndex; i < group.m_lastIndex; i++)
	// 	m_partGroupIdxBuffer[i] = b2_invalidIndex;
	group.m_firstIndex = INVALID_IDX;

	if (groupIdx + 1 == m_groupCount)
		m_groupCount--;
	else
		m_freeGroupIdxs.push_back(groupIdx);
}

void ParticleSystem::ComputeWeight()
{
	// calculates the sum of contact-weights for each particle
	// that means dimensionless density
	//memset(m_buffers.weight.data(), 0, sizeof(*(m_buffers.weight.data())) * m_count);
	//m_buffers.weight.resize(m_capacity);

	m_futureComputeWeight.RunDeferred([=]()
	{
		auto weights = m_ampParts.m_weight.GetView();
		amp::fill(weights, 0.f, m_ampParts.m_count);
		m_ampBodyContacts.ForEach([=](const int32 i, const Particle::BodyContact& contact) restrict(amp)
		{
			amp::atomicAdd(weights[i], contact.weight);
		});
		m_ampGroundContacts.ForEach([=](const int32 a, const Particle::GroundContact& contact) restrict(amp)
		{
			weights[a] += contact.weight;
		});
		m_ampContacts.ShuffledForEach([=](const Particle::Contact& contact) restrict(amp)
		{
			const int32 a = contact.idxA;
			const int32 b = contact.idxB;
			const float32 w = contact.weight;
			amp::atomicAdd(weights[a], w);
			amp::atomicAdd(weights[b], w);
		});
	});
}
void ParticleSystem::WaitForComputeWeight()
{
	m_futureComputeWeight.wait();
	if (IsLastIteration())
		m_ampParts.m_weight.CopyToD11Async();
}
void ParticleSystem::WaitForUpdateBodyContacts()
{
	m_futureUpdateBodyContacts.wait();
	m_futureUpdateGroundContacts.wait();
}

void ParticleSystem::ComputeDepth()
{
	if (!(m_allGroupFlags & ParticleGroup::Flag::NeedsUpdateDepth)) return;
	if (m_ampContacts.Empty()) return;

	const float32 maxFloat = b2_maxFloat;
	const float32 particleDiameter = m_particleDiameter;
	ampArray<Particle::Contact> contactGroups(m_ampContacts.m_count, amp::accelView());
	auto groupIdxs = m_ampParts.m_groupIdx.GetConstView();
	auto contactIdxs = m_ampContacts.m_idx.GetConstView();
	auto contacts = m_ampContacts.m_perTile.GetConstView();
	const uint32 contactsPerTile = m_ampContacts.m_maxPerTile;
	auto groups = GetConstGroups();
	const int32 contactTileCnt = amp::getTileCount(m_ampContacts.m_count);
	ampArray<int32> contactCnts(contactTileCnt, amp::accelView());
	amp::fill(contactCnts, 0);
	ampArray2D<Particle::Contact> localContacts(contactTileCnt, TILE_SIZE, amp::accelView());
	amp::forEachTiled(m_ampContacts.m_count,
		[=, &contactCnts, &localContacts](const int32 gi, const int32 ti, const int32 li) restrict(amp)
	{
		const uint32 contactIdx = contactIdxs[gi];
		const Particle::Contact& contact = contacts[contactIdx / contactsPerTile][contactIdx % contactsPerTile];
		const int32 a = contact.idxA;
		const int32 b = contact.idxB;
		const int32 groupAIdx = groupIdxs[a];
		const int32 groupBIdx = groupIdxs[b];
		
		tile_static uint32 cnt;
		if (groupAIdx != INVALID_IDX && groupAIdx == groupBIdx &&
			groups[groupAIdx].HasFlag(ParticleGroup::Flag::NeedsUpdateDepth))
		{
			localContacts[ti][Concurrency::atomic_fetch_inc(&contactCnts[ti])] = contact;
		}
	});
	const uint32 contactGroupsCount = amp::scan(ampArrayView<const int32>(contactCnts), contactTileCnt,
		[=, &contactCnts, &localContacts, &contactGroups](const int32 i, const int32 wi) restrict(amp)
	{
		for (uint32 j = 0; j < contactCnts[i]; j++)
			contactGroups[wi + j] = localContacts[i][j];
	});

	vector<uint32> groupIdxsToUpdate(m_groupCount);
	int32 groupsToUpdateCount = 0;

	auto accumulations = m_ampParts.m_accumulation.GetView();
	for (int k = 0; k < m_groupCount; k++)
	{
		ParticleGroup& group = m_groupBuffer[k];
		if (group.m_firstIndex != INVALID_IDX)
		{
			if (group.HasFlag(ParticleGroup::Flag::NeedsUpdateDepth))
			{
				groupIdxsToUpdate[groupsToUpdateCount++] = k;
				SetGroupFlags(group, group.m_groupFlags & ~ParticleGroup::Flag::NeedsUpdateDepth);
				amp::forEach(group.m_firstIndex, group.m_lastIndex, [=](const int32 i) restrict(amp)
				{
					accumulations[i] = 0;
				});
			}
		}
	}
	// Compute sum of weight of contacts except between different groups.
	amp::forEach(contactGroupsCount, [=, &contactGroups](const int32 i) restrict(amp)
	{
		const Particle::Contact& contact = contactGroups[i];
		const int32 a = contact.idxA;
		const int32 b = contact.idxB;
		const float32 w = contact.weight;
		amp::atomicAdd(accumulations[a], w);
		amp::atomicAdd(accumulations[b], w);
	});
	b2Assert(m_hasDepth);
	auto depths = m_ampParts.m_depth.GetView();
	for (int32 i = 0; i < groupsToUpdateCount; i++)
	{
		ParticleGroup& group = m_groupBuffer[groupIdxsToUpdate[i]];
		const uint32 startIdx = group.m_firstIndex;
		amp::forEach(group.m_firstIndex, group.m_lastIndex, [=](const int32 i) restrict(amp)
		{
			const float32 w = accumulations[i];
			depths[i] = w < 0.8f ? 0 : maxFloat;
		});
	}

	// The number of iterations is equal to particle number from the deepest
	// particle to the nearest surface particle, and in general it is smaller
	// than sqrt of total particle number.
	int32 iterationCount = (int32)b2Sqrt((float32)m_ampParts.m_count);

	ampArrayView<uint32> ampUpdated(iterationCount);
	amp::fill(ampUpdated, 0u);
	for (int32 t = 0; t < iterationCount; t++)
	{
		amp::forEach(contactGroupsCount, [=, &contactGroups](const int32 i) restrict(amp)
		{
			const Particle::Contact& contact = contactGroups[i];
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
		const ParticleGroup& group = m_groupBuffer[groupIdxsToUpdate[i]];
		amp::forEach(group.m_firstIndex, group.m_lastIndex, [=](const int32 i) restrict(amp)
		{
			float32& p = depths[i];
			if (p < maxFloat)
				p *= particleDiameter;
			else
				p = 0;
		});
	}
	// m_ampCopyFutDepths.set(amp::copyAsync(m_ampArrays.depth, m_buffers.depth, m_count));
}

void ParticleSystem::AddFlagInsideFixture(const Particle::Flag flag, const int32 matIdx,
	const Fixture& fixture)
{
	b2Transform transform;
	transform.SetIdentity();
	auto flags = m_ampParts.m_flags.GetView();
	auto matIdxs = m_ampParts.m_matIdx.GetConstView();
	switch (fixture.m_shapeType)
	{
	case b2Shape::e_circle:
		ForEachInsideCircle(m_world.m_circleShapeBuffer[fixture.m_shapeIdx], transform, [=](int32 i) restrict(amp)
		{
			if (matIdxs[i] == matIdx)
				flags[i] |= flag;
		});
		break;
	}
	m_allFlags |= flag;
}

void ParticleSystem::CopyShapeToGPU(b2Shape::Type type, int32 idx)
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

inline void ParticleSystem::BoundProxyToTagBound(const b2AABBFixtureProxy& aabb, b2TagBounds& tb)
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
void ParticleSystem::ForEachInsideBounds(const b2AABB& aabb, const F& function)
{
	uint32 lowerTag = computeTag(m_inverseDiameter * aabb.lowerBound.x - 1,
		m_inverseDiameter * aabb.lowerBound.y - 1);
	uint32 upperTag = computeTag(m_inverseDiameter * aabb.upperBound.x + 1,
		m_inverseDiameter * aabb.upperBound.y + 1);
	const uint32 xLower = lowerTag & xMask;
	const uint32 xUpper = upperTag & xMask;

	auto flags = m_ampParts.m_flags.GetConstView();
	auto proxies = m_ampParts.m_proxy.GetConstView();
	m_ampParts.ForEachProxy([=](const Proxy& proxy) restrict(amp)
	{
		if (proxy.tag > upperTag || lowerTag > proxy.tag) return;
		const uint32 xTag = proxy.tag & xMask;
		if (xTag < xLower || xTag > xUpper) return;
		function(proxy.idx);
	});
}
template<typename F>
void ParticleSystem::ForEachInsideBounds(const vector<b2AABBFixtureProxy>& aabbs, const F& function)
{
	if (aabbs.empty()) return;
	int32 boundCnt = 0;
	vector<b2TagBounds> tagBounds(aabbs.size());
	for (const b2AABBFixtureProxy& aabb : aabbs)
		BoundProxyToTagBound(aabb, tagBounds[boundCnt++]);
	ampArrayView<const b2TagBounds> ampTagBounds(boundCnt, tagBounds);

	auto flags = m_ampParts.m_flags.GetConstView();
	auto proxies = m_ampParts.m_proxy.GetConstView();
	m_ampParts.ForEachProxy([=](const Proxy& proxy) restrict(amp)
	{
		const uint32 xTag = proxy.tag & xMask;
		for (int32 i = 0; i < boundCnt; i++)
		{
			const b2TagBounds& tb = ampTagBounds[i];
			if (proxy.tag < tb.lowerTag || tb.upperTag < proxy.tag) continue;
			if (xTag < tb.xLower || tb.xUpper < xTag) continue;
			function(proxy.idx, tb.fixtureIdx, tb.childIdx);
		}
	});
}
template<typename F>
void ParticleSystem::ForEachInsideCircle(const b2CircleShape& circle,
	const b2Transform& transform, const F& function)
{
	ampArrayView<const AmpCircleShape> ampCircle(1, (AmpCircleShape*)&circle);
	b2AABB aabb;
	circle.ComputeAABB(aabb, transform, 0);
	auto positions = m_ampParts.m_position.GetConstView();
	ForEachInsideBounds(aabb, [=](const int32 i) restrict(amp)
	{
		if (ampCircle[0].TestPoint(transform, positions[i]))
			function(i);
	});
}

bool ShouldCollisionGroupsCollide(int32 collGroupA, int32 collGroupB) restrict(amp)
{
	if (collGroupA == 0) return true;
	return collGroupA != -collGroupB;
	// if (collGroupA < 0 || collGroupB < 0) return collGroupA != collGroupB;
	// return collGroupA == collGroupB;
}

void ParticleSystem::FindContacts(bool exceptZombie)
{
	auto groups = ampArrayView<const ParticleGroup>(m_ampGroups);
	auto groupIdxs = m_ampParts.m_groupIdx.GetConstView();
	const auto shouldCollide = [=](int32 a, int32 b) restrict(amp) -> bool
	{
		return ShouldCollisionGroupsCollide(groups[groupIdxs[a]].m_collisionGroup,
			groups[groupIdxs[b]].m_collisionGroup);
	};

	auto positions = m_ampParts.m_position.GetConstView();
	auto velocities = m_ampParts.m_velocity.GetConstView();
	auto flags = m_ampParts.m_flags.GetConstView();
	auto invMasses = m_ampParts.m_invMass.GetConstView();
	const float32 invDiameter = m_inverseDiameter;
	const float32 diameter = m_particleDiameter;
	const auto addContact = [=](const int32 a, const int32 b, Particle::Contact& contact) restrict(amp) -> bool
	{
		const uint32 flagsB = flags[b];
		if (exceptZombie && flagsB & Particle::Flag::Zombie) return false;

		const Vec3 d = positions[b] - positions[a];
		const float32 dist = d.Length();
		if (dist > diameter) return false;

		if (!shouldCollide(a, b)) return false;
		//if (d.Length() < b2_epsilon)
		//	positions[b] += velocities[b].Normalized() * b2_linearSlop;
		const uint32 flagsA = flags[a];
		const float32 invM = (flagsA & Particle::Mat::Flag::Wall ? 0 : invMasses[a]) +
			(flagsB & Particle::Mat::Flag::Wall ? 0 : invMasses[b]);
		contact = Particle::Contact(
			a, b, 
			1 - dist * invDiameter,
			invM > 0 ? 1 / invM : 0,
			d / dist,
			flagsA | flagsB
		);
		return true;
	};

	auto proxies = m_ampParts.m_proxy.GetConstView();
	const int32 cnt = m_ampParts.m_count;
	const auto LowerBoundTag = [=](int32 first, const uint32 tag) restrict(amp) -> int32
	{
		int32 i, step, count = cnt - first;
		while (count > 0)
		{
			step = count / 2;
			i = first + step;
			if (proxies[i].tag < tag)
			{
				first = ++i;
				count -= step + 1;
			}
			else
				count = step;
		}
		return first;
	};

	const int32 iterationCnt = m_ampParts.m_iterations;
	const uint32 partsPerTile = TILE_SIZE * m_ampParts.m_iterations;
	const auto GetLocalOrGlobalProxy = [=](const uint32 i, const uint32 t, const Proxy* lProxies) restrict(amp) -> Proxy
	{
		return (i / partsPerTile == t) ? lProxies[i % partsPerTile] : proxies[i];
	};

	const auto FindNextContacts = [=](uint32& b, const uint32 t, const Proxy* lProxies, const uint32 tag, const int32 aIdx,
		uint32& contactCnt, Particle::Contact* contacts) restrict(amp)
	{
		for (; b < cnt; b++)
		{
			const Proxy bProxy = GetLocalOrGlobalProxy(b, t, lProxies);
			if (tag < bProxy.tag) return;
			if (addContact(aIdx, bProxy.idx, contacts[contactCnt])
				&& ++contactCnt == MAX_CONTACTS_PER_PARTICLE) return;
		}
	};

	const auto FindContactsOfProxy = [=](const Proxy& aProxy, uint32 b, uint32& c,
		const uint32 t, const Proxy* lProxies, Particle::Contact* contacts) restrict(amp) -> uint32
	{
		if (exceptZombie && flags[aProxy.idx] & Particle::Flag::Zombie) return 0;

		uint32 contactCnt = 0;
		FindNextContacts(b, t, lProxies, computeRelativeTag(aProxy.tag, 1, 0), aProxy.idx, contactCnt, contacts);
		if (contactCnt == MAX_CONTACTS_PER_PARTICLE) return contactCnt;

		if (!c) c = LowerBoundTag(b, computeRelativeTag(aProxy.tag, -1, 1));
		FindNextContacts(c, t, lProxies, computeRelativeTag(aProxy.tag, 1, 1), aProxy.idx, contactCnt, contacts);
		return contactCnt;
	};
	

	auto contacts = m_ampContacts.m_perTile.GetView();
	const int32 contactsPerTile = m_ampContacts.m_maxPerTile;
	auto contactCnts = m_ampContacts.m_cnt.GetView();
	amp::forEachTiledWithBarrier(b2Min(cnt, CONTACT_THREADS), [=](const ampTiledIdx<TILE_SIZE> tIdx) restrict(amp)
	{
		const uint32 t = tIdx.tile[0];
		const uint32 l = tIdx.local[0] * iterationCnt;
		const uint32 g = tIdx.global[0] * iterationCnt;

		// cache global Proxies in local memory
		Proxy lProxies[MAX_ITERATIONS_PER_PARTICLE];
		tile_static Proxy tProxies[TILE_SIZE * MAX_ITERATIONS_PER_PARTICLE];	// only works with (iterationCnt <= 8) so less then ‭1.048.576‬ Particles
		for (uint32 i = 0; i < iterationCnt; i++)
			if (const uint32 j = g + i; j < cnt)
				lProxies[i] = tProxies[l + i] = proxies[j];

		tIdx.barrier.wait_with_tile_static_memory_fence();

		uint32 contactCnt = 0;
		Particle::Contact lContacts[MAX_CONTACTS_PER_PARTICLE * MAX_ITERATIONS_PER_PARTICLE];
		for (uint32 i = 0, c = 0; i < iterationCnt; i++)
			if (const uint32 j = g + i; j < cnt)
				contactCnt += FindContactsOfProxy(lProxies[i], j + 1, c, t, tProxies, (Particle::Contact*)lContacts + contactCnt);

		uint32 sum;
		const uint32 scan = amp::_prefix_sum_detail::tilePrefixSum(contactCnt, tIdx, sum);
		if (l == 0)
			contactCnts[t] = sum;

		// write contacts to global memory
		ampArrayView<Particle::Contact>& tContacts = contacts[t];
		for (int32 i = 0; i < contactCnt; i++)
			tContacts[scan + i] = lContacts[i];
	});
	//amp::accelView().wait();
}
float32 ParticleSystem::FindContactsTest()
{
	amp::accelView().wait();
	Timer t = Timer();
	FindContacts(true);
	amp::accelView().wait();
	return t.Stop();
}

void ParticleSystem::ReduceContacts()
{
	const int32 contactsPerTile = m_ampContacts.m_maxPerTile;
	auto idxs = m_ampContacts.m_idx.GetView();
	auto cnts = m_ampContacts.m_cnt.GetConstView();
	
	m_ampContacts.m_count = amp::scan(cnts, amp::getTileCount(b2Min(m_ampParts.m_count, CONTACT_THREADS)),
		[=](const int32 i, const int32 wi) restrict(amp)
	{
		for (int32 j = 0; j < cnts[i]; j++)
			idxs[wi + j] = i * contactsPerTile + j;
	});
	//amp::accelView().wait();
}

void ParticleSystem::SortProxies()
{
	// Sort the proxy array by 'tag'. This orders the particles into rows that
	// run left-to-right, top-to-bottom. The rows are spaced m_particleDiameter
	// apart, such that a particle in one row can only collide with the rows
	// immediately above and below it. This ordering makes collision computation
	// tractable.

	const float32 invDiameter = m_inverseDiameter;
	auto proxies = m_ampParts.m_proxy.GetView();
	auto positions = m_ampParts.m_position.GetConstView();
	m_ampParts.ForEachWithZombies([=](const int32 i) restrict(amp)
	{
		const Vec3& pos = positions[i];
		proxies[i].Set(i, computeTag(invDiameter * pos.x, invDiameter * pos.y));
	},
	[=](const int32 i) restrict(amp)
	{
		proxies[i].Set(INVALID_IDX, 0);
	});
	amp::radixSort(proxies, m_ampParts.m_count);

	//ampArrayView<int32> sortError(1);
	//amp::fill(sortError, 0);
	//const int32 count = m_count;
	//amp::forEach(count, [=, &proxies](const int32 i) restrict(amp)
	//{
	//	const int32 i2 = i + 1;
	//	if (i2 >= count) return;
	//	if (proxies[i].tag > proxies[i2].tag)
	//		sortError[0] = 1;
	//});
	//amp::accelView().wait();
}

template<class T>
void ParticleSystem::reorder(vector<T>& v, const vector<int32>& order) 
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
void ParticleSystem::reorder(vector<T1>& v1, vector<T2>& v2, const vector<int32>& order)
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

void ParticleSystem::ComputeAABB(b2AABB& aabb, bool addVel) const
{
	// TODO calc y bounds with Proxy (wait for proxy sort)
	const uint32 cnt = m_ampParts.m_count;
	auto flags = m_ampParts.m_flags.GetConstView();
	auto positions = m_ampParts.m_position.GetConstView();
	auto velocities = m_ampParts.m_velocity.GetConstView();
	int32 tileCnt;
	int32 halfCnt = amp::getTilable(cnt / 2, tileCnt);
	ampArrayView<b2AABB> tileAABBs(b2Max(1, tileCnt));
	amp::forEachTiledWithBarrier(halfCnt, [=](ampTiledIdx<TILE_SIZE> tIdx) restrict(amp)
	{
		const int32 gi = tIdx.global[0];
		const int32 li = tIdx.local[0];
		const int32 ti = tIdx.tile[0];
		tile_static b2AABB aabbs[TILE_SIZE];
		const int32 a = gi * 2;
		const int32 b = a + 1;
		const bool aZombie = (a >= cnt) || (flags[a] & Particle::Flag::Zombie);
		const bool bZombie = (b >= cnt) || (flags[b] & Particle::Flag::Zombie);
		b2AABB& aabb = aabbs[li];
		if (aZombie && bZombie)
		{
			aabb.lowerBound.x = aabb.lowerBound.y = b2_maxFloat;
			aabb.upperBound.x = aabb.upperBound.y = -b2_maxFloat;
		}
		else if (aZombie)
		{
			Vec2 bPos = Vec2(positions[b]);
			if (addVel)
				bPos += velocities[b];
			aabb.lowerBound.x = aabb.upperBound.x = bPos.x;
			aabb.lowerBound.y = aabb.upperBound.y = bPos.y;
		}
		else if (bZombie)
		{
			Vec2 aPos = Vec2(positions[a]);
			if (addVel)
				aPos += velocities[a];
			aabb.lowerBound.x = aabb.upperBound.x = aPos.x;
			aabb.lowerBound.y = aabb.upperBound.y = aPos.y;
		}
		else
		{
			Vec2 aPos = Vec2(positions[a]);
			Vec2 bPos = Vec2(positions[b]);
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
	aabb.lowerBound.x = aabb.lowerBound.y = b2_maxFloat;
	aabb.upperBound.x = aabb.upperBound.y = -b2_maxFloat;
	for (int32 i = 0; i < tileCnt; i++)
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
	const vector<Particle::BodyContact>& bodyContacts,
	const int32 numBodyContacts,
	const uint32 * const particleFlagsBuffer)
{
	//Clear();
	//if (Allocate(numBodyContacts))
	//{
	//	FixtureParticle* set = GetBuffer();
	//	int32 insertedContacts = 0;
	//	for (int32 i = 0; i < numBodyContacts; ++i)
	//	{
	//		FixtureParticle* const fixtureParticle = &set[i];
	//		const Particle::BodyContact& bodyContact = bodyContacts[i];
	//		const int32 partIdx = bodyContact.partIdx;
	//		if (partIdx == INVALID_IDX)
	//			continue;
	//		fixtureParticle->first = bodyContact.fixtureIdx;
	//		fixtureParticle->second = partIdx;
	//		insertedContacts++;
	//	}
	//	SetCount(insertedContacts);
	//	//std::sort(set, set + insertedContacts, FixtureParticle::Compare);
	//	Concurrency::parallel_sort(set, set + insertedContacts, FixtureParticle::Compare);
	//}
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
	const vector<uint32> flags)
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
			if (idxA == INVALID_IDX ||
				idxB == INVALID_IDX ||
				!((flags[idxA] |
				   flags[idxB])))
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

void ParticleSystem::UpdateBodyContacts()
{
	// If the particle contact listener is enabled, generate a set of
	// fixture / particle contacts.
	m_futureUpdateBodyContacts.RunAsync([=]()
	{
		m_ampBodyContacts.Clear();
	
		vector<b2AABBFixtureProxy> fixtureBounds;

		b2AABB partsBounds;
		ComputeAABB(partsBounds);
		m_world.AmpQueryAABB(partsBounds, [=, &fixtureBounds](const Fixture& f)
		{
			if (f.m_isSensor || f.m_idx == INVALID_IDX) return;

			const int32 childCount = m_world.GetShape(f).GetChildCount();
			for (int32 childIdx = 0; childIdx < childCount; childIdx++)
				fixtureBounds.push_back(b2AABBFixtureProxy(m_world.GetAABB(f, childIdx), f.m_idx, childIdx));
		});
		if (fixtureBounds.empty()) return;

		auto groupIdxs = m_ampParts.m_groupIdx.GetConstView();
		auto groups = GetConstGroups();
		const auto shouldCollide = [=](int32 i, const Fixture& f) restrict(amp) -> bool
		{
			return ShouldCollisionGroupsCollide(f.m_filter.collisionGroup, groups[groupIdxs[i]].m_collisionGroup);
		};

		
		auto bodyContactCnts = m_ampBodyContacts.m_cnt.GetView();
		amp::fill(bodyContactCnts, 0, m_ampParts.m_count);

		WaitForCopyBox2DToGPU();

		auto chainShapes = GetConstChainShapes();
		auto circleShapes = GetConstCircleShapes();
		auto edgeShapes = GetConstEdgeShapes();
		auto polygonShapes = GetConstPolygonShapes();
		const float32 partDiameter = m_particleDiameter;
		const float32 doublePartDiameter = 2 * m_particleDiameter;
		const float32 invDiameter = m_inverseDiameter;
		const auto computeDistance = [=] (const Fixture& f, const b2Transform& xf, const Vec3& p,
			float32& d, Vec3& n, int32 childIndex) restrict(amp) -> bool
		{
			//if (f.m_shapeType == b2Shape::e_chain)
			//{
			//	const auto& s = chainShapes[f.m_shapeIdx];
			//	if (!s.TestZ(xf, p.z)) return false;
			//	s.ComputeDistance(xf, p, d, n, childIndex);
			//}
			if (f.m_shapeType == b2Shape::e_circle)
				return circleShapes[f.m_shapeIdx].FindCollision(xf, p, d, n, partDiameter);
			
			//else if (f.m_shapeType == b2Shape::e_edge)
			//{
			//	const auto& s = edgeShapes[f.m_shapeIdx];
			//	if (!s.TestZ(xf, p.z)) return false;
			//	s.ComputeDistance(xf, p, d, n);

			//}
			else if (f.m_shapeType == b2Shape::e_polygon)
				return polygonShapes[f.m_shapeIdx].FindCollision(xf, p, d, n, partDiameter);
			
			return true;
		};

		auto bodies = GetConstBodies();
		auto fixtures = GetConstFixtures();
		auto positions = m_ampParts.m_position.GetConstView();
		auto flags = m_ampParts.m_flags.GetConstView();
		auto invMasses = m_ampParts.m_invMass.GetConstView();
		auto bodyContacts = m_ampBodyContacts.m_array.GetView();
		ForEachInsideBounds(fixtureBounds, [=](int32 i, int32 fixtureIdx, int32 childIdx) restrict(amp)
		{
			const Fixture& fixture = fixtures[fixtureIdx];
			if (!shouldCollide(i, fixture)) return;

			int32& bodyContactIdx = bodyContactCnts[i];
			if (bodyContactIdx + 1 >= MAX_BODY_CONTACTS_PER_PARTICLE) return;
		
			float32 d;
			Vec3 n;
			const int32 bIdx = fixture.m_bodyIdx;
			const Body& b = bodies[bIdx];
			const Vec3 ap = positions[i];
			const bool touch = computeDistance(fixture, b.m_xf, ap, d, n, childIdx);

			if (d > doublePartDiameter) return;

			Particle::BodyContact& contact = bodyContacts[i].contacts[bodyContactIdx++];
			contact.bodyIdx = bIdx;
			contact.fixtureIdx = fixtureIdx;
			contact.childIdx = childIdx;
			contact.flags = 0;

			if (!touch) return;
			contact.AddFlag(Particle::BodyContact::Flag::Touching);

			const Vec2 bp = b.GetWorldCenter();
			const float32 bm = b.m_mass;
			const float32 bI =
				b.GetInertia() - bm * b.GetLocalCenter().LengthSquared();
			const float32 invBm = b.m_invMass;
			const float32 invBI = bI > 0 ? 1 / bI : 0;
			const float32 invAm = flags[i] & Particle::Mat::Flag::Wall ? 0 : invMasses[i];
			const Vec2 rp = ap - bp;
			const float32 rpn = b2Cross(rp, n);
			const float32 invM = invAm + invBm + invBI * rpn * rpn;
			
			contact.weight = b2Max(1 - d * invDiameter, 1.0f);
			contact.normal = -n;
			contact.mass = invM > 0 ? 1 / invM : 0;
			//DetectStuckParticle(i); //TODO
		});

		//vector<int32> cpuContactCnts;
		//cpuContactCnts.resize(m_count);
		//amp::copy(contactCnts, cpuContactCnts, m_count);
		
		auto bodyContactIdxs = m_ampBodyContacts.m_idx.GetView();
		auto constBodyContactCnts = m_ampBodyContacts.m_cnt.GetConstView();
		m_ampBodyContacts.m_count = amp::scan(constBodyContactCnts, m_ampParts.m_count,
			[=](const int32 i, const int32 wi) restrict(amp)
		{
			for (int32 j = 0; j < constBodyContactCnts[i]; j++)
				bodyContactIdxs[wi + j].Set(i, j);
		});
	
		if (m_def.strictContactCheck)
			RemoveSpuriousBodyContacts();
	});
}

void ParticleSystem::UpdateGroundContacts()
{
	m_futureUpdateGroundContacts.RunDeferred([=]()
	{
		const float32 invStride = 1.0f / m_world.m_ground->m_stride;
		const float32 partRadius = m_particleRadius;
		const float32 invDiameter = m_inverseDiameter;
		const Vec3& vec3Up = Vec3_up;
		const int32 txMax = m_world.m_ground->m_tileCntX;
		const int32 tyMax = m_world.m_ground->m_tileCntY;
		const int32 cxMax = m_world.m_ground->m_chunkCntX;
		auto groundContacts = m_ampGroundContacts.m_array.GetView();
		auto positions = m_ampParts.m_position.GetConstView();
		auto masses = m_ampParts.m_mass.GetConstView();
		auto groundTiles = m_world.m_ground->GetConstTiles();
		m_ampParts.ForEach([=](int32 i) restrict(amp)
		{
			const Vec3& p = positions[i];
			int32 tx = p.x * invStride;
			int32 ty = p.y * invStride;

			Particle::GroundContact& contact = groundContacts[i];
			if (tx < 0 || tx >= txMax || ty < 0 || ty >= tyMax)
			{
				contact.setInvalid();
				return;
			}
			const int32 tileIdx = ty * txMax + tx;
			const Ground::Tile& groundTile = groundTiles[tileIdx];
			const float32 d = p.z - (groundTile.height + b2_linearSlop);
			//if (r >= partRadius)
			if (d >= partRadius)
			{
				contact.setInvalid();
				return;
			}
			contact.groundTileIdx = tileIdx;
			contact.groundChunkIdx = (ty / TILE_SIZE_SQRT) * cxMax + (tx / TILE_SIZE_SQRT);
			contact.groundMatIdx = groundTile.matIdx;
			contact.weight = 1 - d * invDiameter;
			contact.normal = vec3Up;
			contact.mass = masses[i];
		});
	});
	
}

const ampArrayView<Body::Mat> ParticleSystem::GetBodyMats()
{
	return ampArrayView<Body::Mat>(m_world.m_ampBodyMaterials);
}
const ampArrayView<const Body::Mat> ParticleSystem::GetConstBodyMats()
{
	return ampArrayView<const Body::Mat>(m_world.m_ampBodyMaterials);
}


void ParticleSystem::RemoveSpuriousBodyContacts()
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
//void b2ParticleSystem::AddBodyContactResults(ampArray<Vec3> dst, const ampArray<Vec3> bodyRes)
//{
//	if (!m_bodyContactCount) return;
//	ampArrayView<const Vec3> add = bodyRes.section(0, m_bodyContactCount);
//	auto& bodyContactPartIdxs = m_ampBodyContactPartIdxs;
//	amp::forEach(m_bodyContactCount, [=, &dst, &bodyContactPartIdxs](const int32 i) restrict(amp)
//	{
//		amp::atomicAdd(dst[bodyContactPartIdxs[i]], add[i]);
//	});
//}

static inline bool IsSignificantForce(Vec3 force)
{
	return force.x != 0 || force.y != 0 || force.z != 0;
}
static inline bool IsSignificantForce(Vec3 force) restrict(amp)
{
	return force.x != 0 || force.y != 0 || force.z != 0;
}
static inline bool IsSignificantForce(Vec2 force) restrict(amp)
{
	return force.x != 0 || force.y != 0;
}

// SolveCollision, SolveRigid and SolveWall should be called after
// other force functions because they may require particles to have
// specific velocities.
void ParticleSystem::SolveCollision()
{
	// This function detects particles which are crossing boundary of bodies
	// and modifies velocities of them so that they will move just in front of
	// boundary. This function also applies the reaction force to
	// bodies as precisely as the numerical stability is kept.
	const b2TimeStep& step = m_subStep;

	//vector<b2AABBFixtureProxy> fixtureBounds;

	//b2AABB aabb;
	//AmpComputeAABB(aabb, true);
	//m_world.AmpQueryAABB(aabb, [=, &fixtureBounds](int32 fixtureIdx)
	//{
	//	const Fixture& f = m_world.m_fixtureBuffer[fixtureIdx];
	//	if (f.m_isSensor) return;

	//	const int32 childCount = m_world.GetShape(f).GetChildCount();
	//	for (int32 childIdx = 0; childIdx < childCount; childIdx++)
	//		fixtureBounds.push_back(b2AABBFixtureProxy(m_world.GetAABB(f, childIdx), fixtureIdx, childIdx));
	//});

	auto positions = m_ampParts.m_position.GetView();
	auto groupIdxs = m_ampParts.m_groupIdx.GetConstView();
	auto groups = GetConstGroups();
	const auto shouldCollide = [=](const Fixture& f, int32 i) restrict(amp) -> bool
	{
		return ShouldCollisionGroupsCollide(f.m_filter.collisionGroup, groups[groupIdxs[i]].m_collisionGroup);
	};

	auto chainShapes = GetConstChainShapes();
	auto circleShapes = GetConstCircleShapes();
	auto edgeShapes = GetConstEdgeShapes();
	auto polygonShapes = GetConstPolygonShapes();
	const auto rayCast = [=](const Fixture& f, RayCastOutput& output, const RayCastInput& input,
		const float32 z, const b2Transform& xf, int32 childIdx) restrict(amp) -> bool
	{
		switch (f.m_shapeType)
		{
		//case b2Shape::e_chain:
		//{
		//	const auto& s = chainShapes[f.m_shapeIdx];
		//	//if (!s.TestZ(xf, z)) return false;
		//	return s.RayCast(output, input, xf, childIdx);
		//}
		case b2Shape::e_circle:
		{
			const auto& s = circleShapes[f.m_shapeIdx];
			//if (!s.TestZ(xf, z)) return false;
			return s.RayCast(output, input, xf);
		}
		//case b2Shape::e_edge:
		//{
		//	const auto& s = edgeShapes[f.m_shapeIdx];
		//	//if (!s.TestZ(xf, z)) return false;
		//	return s.RayCast(output, input, xf);
		//}
		case b2Shape::e_polygon:
		{
			const auto& s = polygonShapes[f.m_shapeIdx];
			//if (!s.TestZ(xf, z)) return false;
			return s.RayCast(output, input, xf);
		}
		}
		return false;
	};

	auto flags = m_ampParts.m_flags.GetConstView();
	auto forces = m_ampParts.m_force.GetView();
	const auto particleAtomicApplyForce = [=](int32 index, const Vec2& force) restrict(amp)
	{
		if (IsSignificantForce(force) && !(flags[index] & Particle::Mat::Flag::Wall))
			amp::atomicAdd(forces[index], force);
	};
	const auto particleApplyForce = [=](int32 index, const Vec2& force) restrict(amp)
	{
		if (IsSignificantForce(force) && !(flags[index] & Particle::Mat::Flag::Wall))
			forces[index] += force;
	};

	const int32 iteration = m_iteration;
	const float32 stepInvDt = m_step.inv_dt;
	const float32 stepDt = m_step.dt;

	auto bodies = GetConstBodies();
	auto fixtures = GetConstFixtures();
	auto velocities = m_ampParts.m_velocity.GetView();
	auto masses = m_ampParts.m_mass.GetConstView();
	//AmpForEachInsideBounds(fixtureBounds, [=](int32 a, int32 fixtureIdx, int32 childIdx) restrict(amp)
	//{
	//	const Fixture& fixture = fixtures[fixtureIdx];
	//	if (!shouldCollide(fixture, a)) return;
	//	const Vec3& ap = positions[a];
	//	const Body& body = bodies[fixture.m_bodyIdx];
	//
	//	const Vec3 av = velocities[a];
	//	b2RayCastOutput output;
	//	b2RayCastInput input;
	//	if (iteration == 0)
	//	{
	//		// Put 'ap' in the local space of the previous frame
	//		Vec2 p1 = b2MulT(body.m_xf0, ap);
	//		if (fixture.m_shapeType == b2Shape::e_circle)
	//		{
	//			// Make relative to the center of the circle
	//			p1 -= body.GetLocalCenter();
	//			// Re-apply rotation about the center of the
	//			// circle
	//			p1 = b2Mul(body.m_xf0.q, p1);
	//			// Subtract rotation of the current frame
	//			p1 = b2MulT(body.m_xf.q, p1);
	//			// Return to local space
	//			p1 += body.GetLocalCenter();
	//		}
	//		// Return to global space and apply rotation of current frame
	//		input.p1 = b2Mul(body.m_xf, p1);
	//	}
	//	else
	//		input.p1 = ap;
	//	input.p2 = ap + stepDt * Vec2(av);
	//	input.maxFraction = 1;
	//	if (!rayCast(fixture, output, input, ap.z, body.m_xf, childIdx)) return;
	//	const Vec2& n = output.normal;
	//	const Vec2 p = (1 - output.fraction) * input.p1 +
	//		output.fraction * input.p2 + b2_linearSlop * n;
	//	const Vec2 v = stepInvDt * (p - ap);
	//	velocities[a] = Vec3(v, av.z);
	//	const Vec2 f = stepInvDt * masses[a] * (Vec2(av) - v);
	//	particleAtomicApplyForce(a, f);
	//});

	
	m_ampBodyContacts.ForEachPotential([=](const int32 i, const Particle::BodyContact& contact) restrict(amp)
	{
		const Vec3 ap = positions[i];
		const Body& body = bodies[contact.bodyIdx];
		const Fixture& fixture = fixtures[contact.fixtureIdx];

		const Vec3 av = velocities[i];
		RayCastOutput output;
		RayCastInput input;
		if (iteration == 0)
		{
			// Put 'ap' in the local space of the previous frame
			Vec2 p1 = b2MulT(body.m_xf0, ap);
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
			input.p1 = Vec3(b2Mul(body.m_xf, p1), ap.z);
		}
		else
			input.p1 = ap;
		input.p2 = ap + stepDt * av;
		input.maxFraction = 1;
		if (!rayCast(fixture, output, input, ap.z, body.m_xf, contact.childIdx)) return;
		const Vec3& n = output.normal;
		const Vec3 p = (1 - output.fraction) * input.p1 +
			output.fraction * input.p2 + b2_linearSlop * n;
		const Vec3 v = stepInvDt * (p - ap);
		velocities[i] = v;
		const Vec3 f = stepInvDt * masses[i] * (av - v);
		particleAtomicApplyForce(i, f);
	});

	const float32 heightOffset = b2_linearSlop; // m_particleRadius;
	auto groundTiles = m_world.m_ground->GetConstTiles();
	m_ampGroundContacts.ForEach([=](const int32 a, const Particle::GroundContact& contact) restrict(amp)
	{
		const Vec3 p1 = positions[a];
		Vec3& v = velocities[a];
		if (v.z >= 0) return;
		const Vec3 p2 = p1 + stepDt * v;
		const float32 h = groundTiles[contact.groundTileIdx].height + heightOffset;
		if (p2.z > h) return;

		const Vec3 av = v;
		v.z = stepInvDt * (h - p1.z);
		const Vec3 f = stepInvDt * masses[a] * (av - v);
		particleApplyForce(a, f);
	});
}

void ParticleSystem::SolveBarrier()
{
	// If a particle is passing between paired barrier particles,
	// its velocity will be decelerated to avoid passing.
	if (!(m_allFlags & Particle::Mat::Flag::Barrier)) return;

	const b2TimeStep& step = m_subStep;

	auto flags = m_ampParts.m_flags.GetConstView();
	auto velocities = m_ampParts.m_velocity.GetView();
	m_ampParts.ForEach([=](const int32 i) restrict(amp)
	{
		if ((flags[i] & Particle::Mat::k_barrierWallFlags) == Particle::Mat::k_barrierWallFlags)
			velocities[i].SetZero();
	});
	const float32 tmax = b2_barrierCollisionTime * step.dt;
	auto pairs = GetConstPairs();
	auto positions = m_ampParts.m_position.GetConstView();

	const int32 timeStamp = m_timestamp;
	auto masses = m_ampParts.m_mass.GetConstView();
	const auto UpdateStatistics = [=](const int32 partIdx, const ParticleGroup& group) restrict(amp)
	{
		if (group.m_timestamp == timeStamp) return;
		const float32 m = masses[partIdx];
		const int32& firstIdx = group.m_firstIndex;
		const int32& lastIdx = group.m_lastIndex;
		float32& mass = group.m_mass = 0;
		Vec2& center = group.m_center;
		Vec2& linVel = group.m_linearVelocity;
		center.SetZero();
		linVel.SetZero();
		for (int32 i = firstIdx; i < lastIdx; i++)
		{
			if (flags[i] & Particle::Flag::Zombie) continue;
			mass += m;
			center += m * (Vec2)positions[i];
			linVel += m * (Vec2)velocities[i];
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
			if (flags[i] & Particle::Flag::Zombie) continue;
			Vec2 p = (Vec2)positions[i] - center;
			Vec2 v = (Vec2)velocities[i] - linVel;
			inertia += m * b2Dot(p, p);
			angVel += m * b2Cross(p, v);
		}
		if (inertia > 0)
		{
			angVel *= 1 / inertia;
		}
		group.m_timestamp = timeStamp;
	};

	const auto GetLinearVelocity = [=](const ParticleGroup & group, int32 partIdx,
		const Vec2 & point) restrict(amp) -> Vec2
	{
		if (group.HasFlag(ParticleGroup::Flag::Rigid))
		{
			UpdateStatistics(partIdx, group);
			return group.m_linearVelocity + b2Cross(group.m_angularVelocity, point - group.m_center);
		}
		else
			return velocities[partIdx];
	};
	auto proxies = m_ampParts.m_proxy.GetConstView();
	const auto TagLowerBound = [=](uint32 first, uint32 last, uint32 tag) restrict(amp) -> int32
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
	const auto TagUpperBound = [=](uint32 first, uint32 last, uint32 tag) restrict(amp) -> int32
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

	const int32 cnt = m_ampParts.m_count;
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

	const auto GetNext = [=](AmpInsideBoundsEnumerator& ibe) restrict(amp) -> int32
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

	auto forces = m_ampParts.m_force.GetView();
	auto groupIdxs = m_ampParts.m_groupIdx.GetConstView();
	auto groups = GetGroups();
	ForEachPair([=](const int32 i) restrict(amp)
	{
		const b2ParticlePair& pair = pairs[i];
		if (!(pair.flags & Particle::Mat::Flag::Barrier)) return;
		const int32 a = pair.indexA;
		const int32 b = pair.indexB;
		const Vec2 pa = positions[a];
		const Vec2 pb = positions[b];
		b2AABB aabb;
		aabb.lowerBound = b2Min(pa, pb);
		aabb.upperBound = b2Max(pa, pb);
		const int32 aGroupIdx = groupIdxs[a];
		const int32 bGroupIdx = groupIdxs[b];
		ParticleGroup& aGroup = groups[aGroupIdx];
		ParticleGroup& bGroup = groups[bGroupIdx];
		const Vec2 va = GetLinearVelocity(aGroup, a, pa);
		const Vec2 vb = GetLinearVelocity(bGroup, b, pb);
		const Vec2 pba = pb - pa;
		const Vec2 vba = vb - va;
		AmpInsideBoundsEnumerator enumerator = GetInsideBoundsEnumerator(aabb);
		int32 c;
		while ((c = GetNext(enumerator)) >= 0)
		{
			Vec2 pc = positions[c];
			int32 cGroupIdx = groupIdxs[c];
			if (aGroupIdx != cGroupIdx && bGroupIdx != cGroupIdx)
			{
				ParticleGroup& cGroup = groups[cGroupIdx];
				const Vec2 vc = GetLinearVelocity(cGroup, c, pc);
				// Solve the equation below:
				//   (1-s)*(pa+t*va)+s*(pb+t*vb) = pc+t*vc
				// which expresses that the particle c will pass a line
				// connecting the particles a and b at the time of t.
				// if s is between 0 and 1, c will pass between a and b.
				const Vec2 pca = pc - pa;
				const Vec2 vca = vc - va;
				const float32 e2 = b2Cross(vba, vca);
				const float32 e1 = b2Cross(pba, vca) - b2Cross(pca, vba);
				const float32 e0 = b2Cross(pba, pca);
				float32 s, t;
				Vec2 qba, qca;
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
				const Vec2 dv = va + s * vba - vc;
				const Vec2 f = masses[c] * dv;
				if (cGroup.HasFlag(ParticleGroup::Flag::Rigid))
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
				Vec2 force = -step.inv_dt * f;
				if (IsSignificantForce(force) && flags[c] & Particle::Mat::Flag::Wall)
					amp::atomicAdd(forces[c], force);
			}
		}
	});
}

bool ParticleSystem::ShouldSolve()
{
	if (!m_world.m_stepComplete || m_ampParts.Empty() || m_step.dt <= 0.0f || m_paused)
		return false;
	return true;
}

void ParticleSystem::SolveInit(int32 timestamp) 
{
	//if (!m_expireTimeBuf.empty())
	//	SolveLifetimes(m_step);
	m_iteration = 0;
	m_timestamp = timestamp;

	CopyBox2DToGPUAsync();
	SolveZombie();
	m_ampParts.m_color.CopyToD11Async();
	if (m_needsUpdateAllParticleFlags)
		UpdateAllParticleFlags();
	
	if (m_needsUpdateAllGroupFlags)
		UpdateAllGroupFlags();

}

void ParticleSystem::InitStep()
{
	++m_timestamp;
	m_subStep = m_step;
	m_subStep.dt /= m_step.particleIterations;
	m_subStep.inv_dt *= m_step.particleIterations;

	//amp::accelView().wait();
}

void ParticleSystem::UpdateContacts(bool exceptZombie)
{
	UpdateBodyContacts();
	UpdateGroundContacts();
	FindContacts(exceptZombie);
	//amp::accelView().wait();
}


void ParticleSystem::SolveEnd()
{
	if (!m_ampParts.Empty())
	{
		m_ampCopyFutGroups.wait();
		m_ampCopyFutBodies.wait();
		if (m_debugContacts) m_ampCopyFutContacts.wait();
		m_ampParts.WaitForCopies();
		//amp::accelView().wait();
	}
}

void ParticleSystem::UpdateAllParticleFlags()
{
	m_allFlags = amp::reduceFlags(m_ampParts.m_flags.arr, m_ampParts.m_count);
	m_needsUpdateAllParticleFlags = false;
}

void ParticleSystem::UpdateAllGroupFlags()
{
	m_allGroupFlags = 0;
	for (int i = 0; i < m_groupCount; i++)
	{
		if (m_groupBuffer[i].m_firstIndex != INVALID_IDX)
			m_allGroupFlags |= m_groupBuffer[i].m_groupFlags;
	}
	m_needsUpdateAllGroupFlags = false;
}

void ParticleSystem::LimitVelocity()
{
	const b2TimeStep& step = m_subStep;
	const float32 criticalVelocitySquared = GetCriticalVelocitySquared(step);

	auto velocities = m_ampParts.m_velocity.GetView();
	m_ampParts.ForEach([=](const int32 i) restrict(amp)
	{
		const Vec3 v = velocities[i];
		const float32 v2 = b2Dot(v, v);
		if (v2 <= criticalVelocitySquared) return;
		const float32 s = ampSqrt(criticalVelocitySquared / v2);
		velocities[i] *= s;
	});
}

void ParticleSystem::SolveGravity()
{
	const Vec3 gravity = m_atmosphereParticleInvMass * m_subStep.dt * m_def.gravityScale * m_world.m_gravity;
	const float32 riseFactor = m_world.m_riseFactor;
	const float32 atmosphericMass = m_atmosphereParticleMass;

	auto velocities = m_ampParts.m_velocity.GetView();
	auto masses = m_ampParts.m_mass.GetConstView();
	auto flags = m_ampParts.m_flags.GetConstView();
	m_ampParts.ForEach([=](const int32 i) restrict(amp)
	{
		if (flags[i] & Particle::Flag::Controlled) return;
		const float32 relativeMass = masses[i] - atmosphericMass;
		velocities[i] += (relativeMass > 0 ? relativeMass : relativeMass * riseFactor) * gravity;
	});
}

void ParticleSystem::SolveWind()
{
	const Vec3 wind = m_world.m_wind;
	const float32 factor = m_subStep.dt * m_atmosphereParticleMass;
	const float32 rand = Random(0, b2_2pi);

	auto velocities = m_ampParts.m_velocity.GetView();
	auto invMasses = m_ampParts.m_invMass.GetConstView();
	m_ampParts.ForEach([=](const int32 i) restrict(amp)
	{
		velocities[i] += invMasses[i] * factor * (wind + amp::toNormal(rand * i));
	});
}

void ParticleSystem::SolveAirResistance()
{
	const float32 airResistance = m_def.airResistanceFactor * m_atmosphereParticleMass * m_subStep.dt;

	auto velocities = m_ampParts.m_velocity.GetView();
	auto invMasses = m_ampParts.m_invMass.GetConstView();
	m_ampParts.ForEach([=](const int32 i) restrict(amp)
	{
		velocities[i] *= 1 - (airResistance * invMasses[i]);
	});
}

void ParticleSystem::SolveStaticPressure()
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
	if (!(m_allFlags & Particle::Mat::Flag::StaticPressure)) return;
	if (m_ampContacts.Empty()) return;

	const b2TimeStep& step = m_subStep;
	const float32 criticalPressure = GetCriticalPressure(step);
	const float32 pressurePerWeight = m_def.staticPressureStrength * criticalPressure;
	const float32 maxPressure = b2_maxParticlePressure * criticalPressure;
	const float32 relaxation = m_def.staticPressureRelaxation;
	const float32 minWeight = b2_minParticleWeight;

	RequestBuffer(m_ampParts.m_staticPressure.arr, hasStaticPressureBuf);
	auto staticPressures = m_ampParts.m_staticPressure.GetView();
	auto accumulations = m_ampParts.m_accumulation.GetView();
	auto weights = m_ampParts.m_weight.GetConstView();
	auto flags = m_ampParts.m_flags.GetConstView();
	for (int32 t = 0; t < m_def.staticPressureIterations; t++)
	{
		amp::fill(accumulations, 0.0f, m_ampParts.m_count);
		m_ampContacts.ForEach(Particle::Mat::Flag::StaticPressure, 
			[=](const Particle::Contact& contact) restrict(amp)
		{
			const int32 a = contact.idxA;
			const int32 b = contact.idxB;
			const float32 w = contact.weight;
			amp::atomicAdd(accumulations[a], w * staticPressures[b]);	// a <- b
			amp::atomicAdd(accumulations[b], w * staticPressures[a]);	// b <- a
		});
		m_ampParts.ForEach([=](const int32 i) restrict(amp)
		{
			const float32 w = weights[i];
			if (flags[i] & Particle::Mat::Flag::StaticPressure)
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

void ParticleSystem::SolvePressure()
{
	// calculates pressure as a linear function of density
	const float32 criticalPressure = GetCriticalPressure(m_subStep);
	const float32 pressurePerWeight = m_def.pressureStrength * criticalPressure;
	const float32 maxPressure = b2_maxParticlePressure * criticalPressure;
	const float32 velocityPerPressure = m_subStep.dt / (m_def.density * m_particleDiameter);

	auto weights = m_ampParts.m_weight.GetConstView();
	auto accumulations = m_ampParts.m_accumulation.GetView();
	m_ampParts.ForEach([=](const int32 i) restrict(amp)
	{
		const float32 h = pressurePerWeight * b2Max(0.0f, weights[i] - b2_minParticleWeight);
		accumulations[i] = b2Min(h, maxPressure);
	});
	// ignores particles which have their own repulsive force
	m_ampParts.ForEach(Particle::Mat::k_noPressureFlags, [=](const int32 i) restrict(amp)
	{
		accumulations[i] = 0;
	});
	// static pressure
	auto staticPressures = m_ampParts.m_staticPressure.GetConstView();
	m_ampParts.ForEach(Particle::Mat::Flag::StaticPressure, [=](const int32 i) restrict(amp)
	{
		accumulations[i] += staticPressures[i];
	});

	// applies pressure between each particles in contact<
	auto velocities = m_ampParts.m_velocity.GetView();
	auto invMasses = m_ampParts.m_invMass.GetConstView();
	auto bodies = GetBodies();
	auto fixtures = GetConstFixtures();
	auto positions = m_ampParts.m_position.GetConstView();
	m_ampBodyContacts.ForEach([=](const int32 i, const Particle::BodyContact& contact) restrict(amp)
	{
		Body& b = bodies[contact.bodyIdx];
		const float32 w = contact.weight;
		const float32 m = contact.mass;
		const Vec3 n = contact.normal;
		const float32 h = accumulations[i] + pressurePerWeight * w;
		const Vec3 f = fixtures[contact.fixtureIdx].m_restitution *
						velocityPerPressure * w * m * h * n;
		amp::atomicSub(velocities[i], invMasses[i] * f);
		b.ApplyLinearImpulse(f, positions[i], true);
	});
	auto groundMats = m_world.m_ground->GetConstMats();
	m_ampGroundContacts.ForEach([=](int32 i, const Particle::GroundContact& contact) restrict(amp)
	{
		const float32 w = contact.weight;
		const Vec3 n = contact.normal;
		const float32 h = accumulations[i] + pressurePerWeight * w;
		const Vec3 f = groundMats[contact.groundMatIdx].bounciness * w * h * n;
		velocities[i] += f;
	});
	m_ampContacts.ShuffledForEach([=](const Particle::Contact& contact) restrict(amp)
	{
		const int32 a = contact.idxA;
		const int32 b = contact.idxB;
		const float32 w = contact.weight;
		const float32 m = contact.mass;
		const Vec3 n = contact.normal;
		const float32 h = accumulations[a] + accumulations[b];
		const Vec3 f = velocityPerPressure * w * m * h * n;
		amp::atomicSub(velocities[a], invMasses[a] * f);
		amp::atomicAdd(velocities[b], invMasses[b] * f);
	});
}

void ParticleSystem::SolveDamping()
{
	// reduces normal velocity of each contact
	const b2TimeStep& step = m_subStep;
	const float32 linearDamping = m_def.dampingStrength;
	const float32 quadraticDamping = 1 / GetCriticalVelocity(step);
	const float32 nonSolidFriction = m_def.nonSolidFriction;

	auto velocities = m_ampParts.m_velocity.GetView();
	auto invMasses = m_ampParts.m_invMass.GetConstView();
	auto bodies = GetBodies();
	auto positions = m_ampParts.m_position.GetConstView();
	m_ampBodyContacts.ForEach([=](const int32 i, const Particle::BodyContact& contact) restrict(amp)
	{
		Body& b = bodies[contact.bodyIdx];
		const float32 w = contact.weight;
		const float32 m = contact.mass;
		const Vec3 n = contact.normal;
		const Vec2 p = Vec2(positions[i]);
		const Vec2 v = b.GetLinearVelocityFromWorldPoint(p) -
			Vec2(velocities[i]);
		const float32 vn = b2Dot(v, n);
		if (vn >= 0) return;
		const float32 damping =
			b2Max(linearDamping * w, b2Min(-quadraticDamping * vn, 0.5f));
		const Vec3 f = damping * m * vn * n;
		amp::atomicAdd(velocities[i], invMasses[i] * f);
		b.ApplyLinearImpulse(-f, p, true);
	});
	auto flags = m_ampParts.m_flags.GetConstView();
	auto groundMats = m_world.m_ground->GetConstMats();
	m_ampGroundContacts.ForEach([=](int32 a, const Particle::GroundContact& contact) restrict(amp)
	{
		const float32 w = contact.weight;
		const Vec3 n = contact.normal;
		Vec3& v = velocities[a];
		const float32 vn = b2Dot(v, n);
		if (vn >= 0) return;
		const float32 damping =
			b2Max(linearDamping * w, b2Min(-quadraticDamping * vn, 0.5f));
		v += damping * vn * n;
		float32 groundFriction = groundMats[contact.groundMatIdx].friction;
		if (flags[a] & Particle::Mat::k_nonSolidFlags)
			groundFriction *= nonSolidFriction;
		const float32 frictionFactor = 1 - groundFriction;
		v.x *= frictionFactor;
		v.y *= frictionFactor;
	});
	m_ampContacts.ShuffledForEach([=](const Particle::Contact& contact) restrict(amp)
	{
		const int32 a = contact.idxA;
		const int32 b = contact.idxB;
		const float32 w = contact.weight;
		const float32 m = contact.mass;
		const Vec3 n = contact.normal;
		const Vec3 v = velocities[b] - velocities[a];
		const float32 vn = b2Dot(v, n);
		if (vn >= 0) return;
		const float32 damping = b2Max(linearDamping * w, b2Min(-quadraticDamping * vn, 0.5f));
		const Vec3 f = damping * m * vn * n;
		amp::atomicAdd(velocities[a], invMasses[a] * f);
		amp::atomicSub(velocities[b], invMasses[b] * f);
	});
}

inline bool ParticleSystem::IsRigidGroup(const ParticleGroup& group) const
{
	return group.HasFlag(ParticleGroup::Flag::Rigid);
}

inline Vec2 ParticleSystem::GetLinearVelocity(
	const ParticleGroup& group, int32 particleIndex,
	const Vec2 &point)
{
	if (IsRigidGroup(group))
		return GetLinearVelocityFromWorldPoint(group, point);
	else
		return Vec2(m_buffers.velocity[particleIndex]);
}

inline void ParticleSystem::InitDampingParameter(
	float32& invMass, float32& invInertia, float32& tangentDistance,
	float32 mass, float32 inertia, const Vec2& center,
	const Vec2& point, const Vec2& normal) const
{
	invMass = mass > 0 ? 1 / mass : 0;
	invInertia = inertia > 0 ? 1 / inertia : 0;
	tangentDistance = b2Cross(point - center, normal);
}

inline void ParticleSystem::InitDampingParameterWithRigidGroupOrParticle(
	float32& invMass, float32& invInertia, float32& tangentDistance,
	bool isRigidGroup, const ParticleGroup& group, int32 particleIndex,
	const Vec2& point, const Vec2& normal)
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
		uint32 flags = m_buffers.flags[particleIndex];
		InitDampingParameter(
			invMass, invInertia, tangentDistance,
			//flags & Particle::Mat::Flag::b2_wallParticle ? 0 : GetParticleMass(), 0, point,
			flags & Particle::Mat::Flag::Wall ? 0 : m_buffers.mass[particleIndex], 0, point,
			point, normal);
	}
}

inline float32 ParticleSystem::ComputeDampingImpulse(
	float32 invMassA, float32 invInertiaA, float32 tangentDistanceA,
	float32 invMassB, float32 invInertiaB, float32 tangentDistanceB,
	float32 normalVelocity) const
{
	float32 invMass =
		invMassA + invInertiaA * tangentDistanceA * tangentDistanceA +
		invMassB + invInertiaB * tangentDistanceB * tangentDistanceB;
	return invMass > 0 ? normalVelocity / invMass : 0;
}

inline void ParticleSystem::ApplyDamping(
	float32 invMass, float32 invInertia, float32 tangentDistance,
	bool isRigidGroup, ParticleGroup& group, int32 particleIndex,
	float32 impulse, const Vec2& normal)
{
	if (isRigidGroup)
	{
		group.m_linearVelocity += impulse * invMass * normal;
		group.m_angularVelocity += impulse * tangentDistance * invInertia;
	}
	else
	{
		Vec2 vel = impulse * invMass * normal;
		m_buffers.velocity[particleIndex] += vel;
	}
}

void ParticleSystem::SolveRigidDamping()
{
	// Apply impulse to rigid particle groups colliding with other objects
	// to reduce relative velocity at the colliding point.
	if (!(m_allGroupFlags & ParticleGroup::Flag::Rigid)) return;

	const float32 damping = m_def.dampingStrength;
	const int32 timeStamp = m_timestamp;

	auto velocities = m_ampParts.m_velocity.GetView();
	auto positions = m_ampParts.m_position.GetConstView();
	auto groupIdxs = m_ampParts.m_groupIdx.GetConstView();
	auto groups = GetGroups();
	auto masses = m_ampParts.m_mass.GetConstView();
	
	const auto AmpUpdateStatistics = [=](const int32 partIdx, const ParticleGroup& group) restrict(amp)
	{
		if (group.m_timestamp == timeStamp) return;
		const float32 m = masses[partIdx];
		const int32 & firstIdx = group.m_firstIndex;
		const int32 & lastIdx = group.m_lastIndex;
		float32 & mass = group.m_mass = 0;
		Vec2 & center = group.m_center;
		Vec2 & linVel = group.m_linearVelocity;
		center.SetZero();
		linVel.SetZero();
		for (int32 i = firstIdx; i < lastIdx; i++)
		{
			mass += m;
			center += m * (Vec2)positions[i];
			linVel += m * (Vec2)velocities[i];
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
			Vec2 p = (Vec2)positions[i] - center;
			Vec2 v = (Vec2)velocities[i] - linVel;
			inertia += m * b2Dot(p, p);
			angVel += m * b2Cross(p, v);
		}
		if (inertia > 0)
		{
			angVel *= 1 / inertia;
		}
		group.m_timestamp = timeStamp;
	};

	const auto AmpGetLinearVelocity = [=](const int32 partIdx,
		const ParticleGroup& group, const Vec2& point) restrict(amp) -> Vec2
	{
		AmpUpdateStatistics(partIdx, group);
		return group.m_linearVelocity + b2Cross(group.m_angularVelocity, point - group.m_center);
	};

	const auto AmpInitDampingParameter = [=](
		float32& invMass, float32& invInertia, float32& tangentDistance,
		float32 mass, float32 inertia, const Vec2& center,
		const Vec2& point, const Vec2 & normal) restrict(amp)
	{
		invMass = mass > 0 ? 1 / mass : 0;
		invInertia = inertia > 0 ? 1 / inertia : 0;
		tangentDistance = b2Cross(point - center, normal);
	};

	auto matIdxs = m_ampParts.m_matIdx.GetConstView();
	auto flags = m_ampParts.m_flags.GetConstView();
	const auto AmpInitDampingParameterWithRigidGroupOrParticle = [=](
		float32& invMass, float32& invInertia, float32& tangentDistance,
		uint32 isRigid, const ParticleGroup& group, int32 particleIndex,
		const Vec2& point, const Vec2& normal) restrict(amp)
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
				flags[particleIndex] & Particle::Mat::Flag::Wall ? 0 : masses[particleIndex],
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
	const auto ApplyDamping = [=](
		float32 invMass, float32 invInertia, float32 tangentDistance,
		uint32 isRigid, ParticleGroup& group, int32 particleIndex,
		float32 impulse, const Vec2& normal) restrict(amp)
	{
		if (isRigid)
		{
			group.m_linearVelocity += impulse * invMass * normal;
			group.m_angularVelocity += impulse * tangentDistance * invInertia;
		}
		else
		{
			const Vec2 vel = impulse * invMass * normal;
			amp::atomicAdd(velocities[particleIndex], vel);
		}
	};
	auto bodies = GetBodies();
	m_ampBodyContacts.ForEach([=](const int32 i, const Particle::BodyContact& contact) restrict(amp)
	{
		ParticleGroup& aGroup = groups[groupIdxs[i]];
		if (!aGroup.HasFlag(ParticleGroup::Flag::Rigid)) return;
		Body& b = bodies[contact.bodyIdx];
		Vec3 n = contact.normal;
		float32 w = contact.weight;
		Vec2 p = Vec2(positions[i]);
		Vec2 v = b.GetLinearVelocityFromWorldPoint(p) -
			AmpGetLinearVelocity(i, aGroup, p);
		float32 vn = b2Dot(v, n);
		if (vn >= 0) return;
		// The group's average velocity at particle position 'p' is pushing
		// the particle into the body.
		float32 invMassA, invInertiaA, tangentDistanceA;
		float32 invMassB, invInertiaB, tangentDistanceB;
		AmpInitDampingParameterWithRigidGroupOrParticle(
			invMassA, invInertiaA, tangentDistanceA, true,
			aGroup, i, p, n);
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
			true, aGroup, i, f, n);
		b.ApplyLinearImpulse(-f * n, p, true);
	});
	m_ampContacts.ForEach([=](const Particle::Contact& contact) restrict(amp)
	{
		const int32 a = contact.idxA;
		const int32 b = contact.idxB;
		const Vec3 n = contact.normal;
		const float32 w = contact.weight;
		const int32 aGroupIdx = groupIdxs[a];
		const int32 bGroupIdx = groupIdxs[b];
		ParticleGroup& aGroup = groups[aGroupIdx];
		ParticleGroup& bGroup = groups[bGroupIdx];
		const bool aRigid = aGroup.HasFlag(ParticleGroup::Flag::Rigid);
		const bool bRigid = bGroup.HasFlag(ParticleGroup::Flag::Rigid);
		if (aGroupIdx == bGroupIdx || !(aRigid || bRigid)) return;
		const Vec2 p = 0.5f * positions[a] + positions[b];
		const Vec2 v = (bRigid ? AmpGetLinearVelocity(b, bGroup, p) : velocities[b]) -
							(aRigid ? AmpGetLinearVelocity(a, aGroup, p) : velocities[a]);
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

void ParticleSystem::SolveExtraDamping()
{
	// Applies additional damping force between bodies and particles which can
	// produce strong repulsive force. Applying damping force multiple times
	// is effective in suppressing vibration.

	if (!(m_allFlags & Particle::Mat::k_extraDampingFlags)) return;

	auto bodies = GetBodies();
	auto positions = m_ampParts.m_position.GetConstView();
	auto velocities = m_ampParts.m_velocity.GetView();
	auto invMasses = m_ampParts.m_invMass.GetConstView();
	m_ampBodyContacts.ForEach(Particle::Mat::k_extraDampingFlags,
		[=](const int32 i, const Particle::BodyContact& contact) restrict(amp)
	{
		Body& b = bodies[contact.bodyIdx];
		const float32 m = contact.mass;
		const Vec3 n = contact.normal;
		const Vec2 p = Vec2(positions[i]);
		const Vec2 v =
			b.GetLinearVelocityFromWorldPoint(p) -
			Vec2(velocities[i]);
		const float32 vn = b2Dot(v, n);
		if (vn >= 0) return;
		const Vec3 f = 0.5f * m * vn * n;
		amp::atomicAdd(velocities[i], invMasses[i] * f);
		b.ApplyLinearImpulse(-f, p, true);
	});
	m_ampGroundContacts.ForEach(Particle::Mat::k_extraDampingFlags,
		[=](const int32 i, const Particle::GroundContact& contact) restrict(amp)
	{
		const Vec3 n = contact.normal;
		Vec3& v = velocities[i];
		const float32 vn = b2Dot(v, n);
		if (vn >= 0) return;
		const Vec3 f = 0.5f * vn * n;
		v += f;
	});
}

void ParticleSystem::SolveWall()
{
	if (!FlagExists(Particle::Mat::Flag::Wall)) return;

	auto velocities = m_ampParts.m_velocity.GetView();
	m_ampParts.ForEach(Particle::Mat::Flag::Wall, [=](const int32 i) restrict(amp)
	{
		velocities[i].SetZero();
	});
}

void ParticleSystem::CopyVelocities()
{
	if (IsLastIteration())
		m_ampParts.m_velocity.CopyToD11Async();
}

void ParticleSystem::SolveRigid()
{
	if (!GroupFlagExists(ParticleGroup::Flag::Rigid)) return;

	const b2TimeStep& step = m_subStep;

	auto positions = m_ampParts.m_position.GetConstView();
	auto velocities = m_ampParts.m_velocity.GetView();
	for (int32 k = 0; k < m_groupCount; k++)
	{
		ParticleGroup& group = m_groupBuffer[k];
		if (group.m_firstIndex != INVALID_IDX)
		{
			if (group.HasFlag(ParticleGroup::Flag::Rigid))
			{
				UpdateStatistics(group);
				const b2Rot rotation(step.dt * group.m_angularVelocity);
				const Vec2 center = group.m_center;
				const Vec2 linVel = group.m_linearVelocity;
				const b2Transform transform(center + step.dt * linVel -
					b2Mul(rotation, center), rotation);
				group.m_transform = b2Mul(transform, group.m_transform);
				b2Transform velocityTransform;
				velocityTransform.p.x = step.inv_dt * transform.p.x;
				velocityTransform.p.y = step.inv_dt * transform.p.y;
				velocityTransform.q.s = step.inv_dt * transform.q.s;
				velocityTransform.q.c = step.inv_dt * (transform.q.c - 1);

				amp::forEach(group.m_firstIndex, group.m_lastIndex,
					[=](const int32 i) restrict(amp)
				{
					const Vec3 vel = Vec3(b2Mul(velocityTransform, positions[i]), 0);
					velocities[i] = vel;
				});
			}
		}
	}
}

// SolveElastic and SolveSpring refer the current velocities for
// numerical stability, they should be called as late as possible.
void ParticleSystem::SolveElastic()
{
	if (!(m_allFlags & Particle::Mat::Flag::Elastic)) return;

	const b2TimeStep& step = m_subStep;
	const float32 elasticStrength = step.inv_dt * m_def.elasticStrength;

	m_ampCopyFutTriads.wait();
	auto triads = GetConstTriads();
	auto positions = m_ampParts.m_position.GetConstView();
	auto velocities =  m_ampParts.m_velocity.GetView();
	ForEachTriad([=](const int32 i) restrict(amp)
	{
		const b2ParticleTriad& triad = triads[i];
		if (!(triad.flags & Particle::Mat::Flag::Elastic)) return;
		const int32 a = triad.indexA;
		const int32 b = triad.indexB;
		const int32 c = triad.indexC;
		const Vec2& oa = triad.pa;
		const Vec2& ob = triad.pb;
		const Vec2& oc = triad.pc;
		Vec2 pa = positions[a];
		Vec2 pb = positions[b];
		Vec2 pc = positions[c];
		const Vec2 va = velocities[a];
		const Vec2 vb = velocities[b];
		const Vec2 vc = velocities[c];
		pa += step.dt * va;
		pb += step.dt * vb;
		pc += step.dt * vc;
		Vec2 midPoint = 1.0f / 3 * (pa + pb + pc);
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
		Vec2 vel = b2Mul(r, oa) - pa;
		amp::atomicAdd(velocities[a], strength * vel);
		vel = b2Mul(r, ob) - pb;
		amp::atomicAdd(velocities[b], strength * vel);
		vel = b2Mul(r, oc) - pc;
		amp::atomicAdd(velocities[c], strength * vel);
	});
}

void ParticleSystem::SolveSpring()
{
	if (!(m_allFlags & Particle::Mat::Flag::Spring)) return;

	const b2TimeStep& step = m_subStep;
	const float32 springStrength = step.inv_dt * m_def.springStrength;

	auto pairs = GetConstPairs();
	auto positions = m_ampParts.m_position.GetConstView();
	auto velocities = m_ampParts.m_velocity.GetView();
	ForEachPair([=](const int32 i) restrict(amp)
	{
		const b2ParticlePair& pair = pairs[i];
		if (!(pair.flags & Particle::Mat::Flag::Spring)) return;
		const int32 a = pair.indexA;
		const int32 b = pair.indexB;
		Vec3 pa = positions[a];
		Vec3 pb = positions[b];
		const Vec3 vb = velocities[b];
		const Vec3 va = velocities[a];
		pa += step.dt * va;
		pb += step.dt * vb;
		const Vec3 d = pb - pa;
		const float32 r0 = pair.distance;
		const float32 r1 = d.Length();
		const float32 strength = springStrength * pair.strength;
		Vec2 f = strength * (r0 - r1) / r1 * d;
		amp::atomicAdd(velocities[a], f);
		amp::atomicSub(velocities[b], f);
	});
}

void ParticleSystem::SolveTensile()
{
	if (!(m_allFlags & Particle::Mat::Flag::Tensile)) return;

	const b2TimeStep& step = m_subStep;
	const float32 criticalVelocity = GetCriticalVelocity(step);
	const float32 pressureStrength = m_def.surfaceTensionPressureStrength * criticalVelocity;
	const float32 normalStrength = m_def.surfaceTensionNormalStrength * criticalVelocity;
	const float32 maxVelocityVariation = b2_maxParticleForce * criticalVelocity;

	auto accumulations = m_ampParts.m_accumulationVec3.GetView();
	amp::fill(accumulations, Vec3_zero);
	m_ampContacts.ForEach(Particle::Mat::Flag::Tensile, [=](const Particle::Contact& contact) restrict(amp)
	{
		const int32 a = contact.idxA;
		const int32 b = contact.idxB;
		const float32 w = contact.weight;
		const Vec3 n = contact.normal;
		const Vec3 weightedNormal = (1 - w) * w * n;
		amp::atomicSub(accumulations[a], weightedNormal);
		amp::atomicAdd(accumulations[b], weightedNormal);
	});

	auto weights = m_ampParts.m_weight.GetConstView();
	auto velocities = m_ampParts.m_velocity.GetView();
	auto invMasses = m_ampParts.m_invMass.GetConstView();
	m_ampContacts.ForEach(Particle::Mat::Flag::Tensile, [=](const Particle::Contact& contact) restrict(amp)
	{
		const int32 a = contact.idxA;
		const int32 b = contact.idxB;
		const float32 w = contact.weight;
		const float32 m = contact.mass;
		const Vec3 n = contact.normal;
		const float32 h = weights[a] + weights[b];
		const Vec3 s = accumulations[b] - accumulations[a];
		const float32 fn = b2Min(
			pressureStrength * (h - 2) + normalStrength * b2Dot(s, n),
			maxVelocityVariation) * w;
		Vec3 f = fn * n * m;
		f.z = 0;
		amp::atomicSub(velocities[a], invMasses[a] * f);
		amp::atomicAdd(velocities[b], invMasses[b] * f);
	});
}

void ParticleSystem::SolveViscous()
{
	if (!(m_allFlags & Particle::Mat::Flag::Viscous)) return;

	const float32 viscousStrength = m_def.viscousStrength;

	auto velocities = m_ampParts.m_velocity.GetView();
	auto invMasses = m_ampParts.m_invMass.GetConstView();
	auto bodies = GetBodies();
	auto positions = m_ampParts.m_position.GetConstView();
	m_ampBodyContacts.ForEach(Particle::Mat::Flag::Viscous,
		[=](const int32 i, const Particle::BodyContact& contact) restrict(amp)
	{
		Body& b = bodies[contact.bodyIdx];
		const float32 w = contact.weight;
		const float32 m = contact.mass;
		const Vec3 p = positions[i];
		const Vec3 v(b.GetLinearVelocityFromWorldPoint(p) -
			Vec2(velocities[i]), 0);
		const Vec3 f = viscousStrength * m * w * v;
		amp::atomicAdd(velocities[i], invMasses[i] * f);
		b.ApplyLinearImpulse(-f, p, true);
	});
	m_ampGroundContacts.ForEach(Particle::Mat::Flag::Viscous, 
		[=](const int32 i, const Particle::GroundContact& contact) restrict(amp)
	{
		const float32 w = contact.weight;
		const float32 m = contact.mass;
		Vec3& v = velocities[i];
		Vec3 f = viscousStrength * w * v;
		v += f;
	});
	m_ampContacts.ForEach(Particle::Mat::Flag::Viscous, [=](const Particle::Contact& contact) restrict(amp)
	{
		const int32 a = contact.idxA;
		const int32 b = contact.idxB;
		const float32 w = contact.weight;
		const float32 m = contact.mass;
		const Vec3 v = velocities[b] - velocities[a];
		const Vec3 f = viscousStrength * w * m * v;
		amp::atomicAdd(velocities[a], invMasses[a] * f);
		amp::atomicSub(velocities[b], invMasses[b] * f);
	});
}

void ParticleSystem::SolveRepulsive()
{
	if (!(m_allFlags & Particle::Mat::Flag::Repulsive)) return;

	const b2TimeStep& step = m_subStep;
	float32 repulsiveStrength =
		m_def.repulsiveStrength * GetCriticalVelocity(step);

	auto velocities = m_ampParts.m_velocity.GetView();
	auto groupIdxs = m_ampParts.m_groupIdx.GetConstView();
	auto invMasses = m_ampParts.m_invMass.GetConstView();
	m_ampContacts.ForEach(Particle::Mat::Flag::Repulsive, [=](const Particle::Contact& contact) restrict(amp)
	{
		const int32 a = contact.idxA;
		const int32 b = contact.idxB;
		if (groupIdxs[a] == groupIdxs[b]) return;
		const float32 w = contact.weight;
		const float32 m = contact.mass;
		const Vec3 n = contact.normal;
		const Vec3 f = repulsiveStrength * w * m * n;
		amp::atomicSub(velocities[a], invMasses[a] * f);
		amp::atomicAdd(velocities[b], invMasses[b] * f);
	});
}

void ParticleSystem::SolvePowder()
{
	if (!(m_allFlags & Particle::Mat::Flag::Powder)) return;

	const b2TimeStep& step = m_subStep;
	float32 powderStrength = m_def.powderStrength * GetCriticalVelocity(step);
	const float32 minWeight = 1.0f - b2_particleStride;

	auto velocities = m_ampParts.m_velocity.GetView();
	auto invMasses = m_ampParts.m_invMass.GetConstView();
	m_ampContacts.ForEach(Particle::Mat::Flag::Powder, [=](const Particle::Contact& contact) restrict(amp)
	{
		const float32 w = contact.weight;
		if (w <= minWeight) return;
		const int32 a = contact.idxA;
		const int32 b = contact.idxB;
		const float32 m = contact.mass;
		const Vec3 n = contact.normal;
		const Vec3 f = powderStrength * (w - minWeight) * m * n;
		amp::atomicSub(velocities[a], invMasses[a] * f);
		amp::atomicAdd(velocities[b], invMasses[b] * f);
	});
}

void ParticleSystem::SolveSolid()
{
	// applies extra repulsive force from solid particle groups
	if (!(m_allGroupFlags & ParticleGroup::Flag::Solid)) return;

	b2Assert(m_hasDepth);
	const b2TimeStep& step = m_subStep;
	const float32 ejectionStrength = step.inv_dt * m_def.ejectionStrength;
	
	auto velocities = m_ampParts.m_velocity.GetView();
	auto groupIdxs = m_ampParts.m_groupIdx.GetConstView();
	auto invMasses = m_ampParts.m_invMass.GetConstView();
	auto depths = m_ampParts.m_depth.GetConstView();
	m_ampContacts.ForEach([=](const Particle::Contact& contact) restrict(amp)
	{
		const int32 a = contact.idxA;
		const int32 b = contact.idxB;
		if (groupIdxs[a] == groupIdxs[b]) return;
		const float32 w = contact.weight;
		const float32 m = contact.mass;
		const Vec3 n = contact.normal;
		const float32 h = depths[a] + depths[b];
		const Vec3 f = ejectionStrength * h * m * w * n;
		amp::atomicSub(velocities[a], invMasses[a] * f);
		amp::atomicAdd(velocities[b], invMasses[b] * f);
	});
}

void ParticleSystem::SolveForce()
{
	if (!m_hasForce) return;
	const b2TimeStep& step = m_subStep;

	auto velocities = m_ampParts.m_velocity.GetView();
	auto forces = m_ampParts.m_force.GetConstView();
	auto invMasses = m_ampParts.m_invMass.GetConstView();
	m_ampParts.ForEach([=](const int32 i) restrict(amp)
	{
		velocities[i] += step.dt * invMasses[i] * forces[i];
	});
	m_hasForce = false;
}

void ParticleSystem::SolveColorMixing()
{
	// mixes color between contacting particles
	//b2Assert(m_buffers.color.data());
	//const int32 strength = (int32) (128 * m_def.colorMixingStrength);
	//if (strength) {
	//	for (int32 k = 0; k < m_contactCount; k++)
	//	{
	//		const Particle::Contact& contact = m_contacts[k];
	//		int32 a = contact.idxA;
	//		int32 b = contact.idxB;
	//		if (m_buffers.flags[a] & m_buffers.flags[b] &
	//			Particle::Mat::Flag::ColorMixing)
	//		{
	//			uint32& colA = m_buffers.color[a];
	//			uint32& colB = m_buffers.color[b];

	//			uint8 ar = (colA & 0xFF000000) >> 24;
	//			uint8 ag = (colA & 0x00FF0000) >> 16;
	//			uint8 ab = (colA & 0x0000FF00) >>  8;
	//			uint8 aa = (colA & 0x000000FF) >>  0;
	//			uint8 br = (colB & 0xFF000000) >> 24;
	//			uint8 bg = (colB & 0x00FF0000) >> 16;
	//			uint8 bb = (colB & 0x0000FF00) >>  8;
	//			uint8 ba = (colB & 0x000000FF) >>  0;
	//			const uint8 dr = (uint8)(strength * (br - ar));
	//			const uint8 dg = (uint8)(strength * (bg - ag));
	//			const uint8 db = (uint8)(strength * (bb - ab));
	//			const uint8 da = (uint8)(strength * (ba - aa));
	//			ar += dr;
	//			ag += dg;
	//			ab += db;
	//			aa += da;
	//			br -= dr;
	//			bg -= dg;
	//			bb -= db;
	//			ba -= da;
	//			colA = (ar << 24) | (ag << 16) | (ab << 8) | (aa);
	//			colB = (br << 24) | (bg << 16) | (bb << 8) | (ba);
	//		}
	//	}
	//}
}

void DistributeHeatAtomicB(float32& aHeat, float32& bHeat, float32 factor,
	const float32& aMass, const float32& bMass) restrict(amp)
{
	factor /= aMass + bMass;
	if (b2Abs(factor) < b2_epsilon) return;
	float32 expectedB, newB, newA;
	do
	{
		const float32 d = (aHeat - bHeat) * factor;
		if (b2Abs(factor) < b2_epsilon) return;
		expectedB = bHeat;
		newA = aHeat - d * bMass;
		newB = bHeat + d * aMass;
	} while (!Concurrency::atomic_compare_exchange((uint32*)&bHeat,
		(uint32*)&expectedB, reinterpret_cast<uint32&>(newB)));
	aHeat = newA;
}

void DistributeHeat(float32& aHeat, float32& bHeat, const float32& factor,
	const float32& aMass, const float32& bMass) restrict(amp)
{
	const float32 d = (aHeat - bHeat) * factor / (aMass + bMass);
	if (b2Abs(d) < b2_epsilon) return;
	aHeat -= d * bMass;
	bHeat += d * aMass;
}

void ParticleSystem::SolveHeatConduct()
{
	const b2TimeStep& step = m_subStep;

	auto heats = m_ampParts.m_heat.GetView();
	auto matIdxs = m_ampParts.m_matIdx.GetConstView();
	auto mats = m_mats.m_array.GetConstView();
	auto bodies = GetBodies();
	auto bodyMats = GetConstBodyMats();
	m_ampBodyContacts.ForEach(Particle::Mat::Flag::HeatConducting,
		[=](const int32 i, const Particle::BodyContact& contact) restrict(amp)
	{
		Body& b = bodies[contact.bodyIdx];
		const Body::Mat& bMat = bodyMats[b.m_matIdx];
		if (!bMat.HasFlag(Body::Mat::Flag::HeatConducting)) return;
		const Particle::Mat& aMat = mats[matIdxs[i]];
		const float32 factor = step.dt * aMat.m_heatConductivity * bMat.m_heatConductivity
			* ampSqrt(contact.weight);
		DistributeHeatAtomicB(heats[i], b.m_surfaceHeat, factor, aMat.m_mass, b.m_surfaceMass);
	});
	m_ampContacts.ShuffledForEach(Particle::Mat::Flag::HeatConducting, [=](const Particle::Contact& contact) restrict(amp)
	{
		const int32 a = contact.idxA;
		const int32 b = contact.idxB;
		const Particle::Mat& aMat = mats[matIdxs[a]];
		const Particle::Mat& bMat = mats[matIdxs[b]];
		if (!(aMat.m_heatConductivity && bMat.m_heatConductivity)) return;
		const float32 factor = step.dt * aMat.m_heatConductivity * bMat.m_heatConductivity
			* ampSqrt(contact.weight);
		DistributeHeat(heats[a], heats[b], factor, aMat.m_mass, bMat.m_mass);
	});
}

void ParticleSystem::SolveLooseHeat()
{
	if (!(m_allFlags & Particle::Mat::Flag::HeatLoosing)) return;

	const b2TimeStep& step = m_subStep;

	auto heats = m_ampParts.m_heat.GetView();
	auto matIdxs = m_ampParts.m_matIdx.GetConstView();
	auto mats = m_mats.m_array.GetConstView();
	const float32 roomTemp = m_world.m_roomTemperature;
	const float32 heatLossRatio = m_def.heatLossRatio;
	m_ampParts.ForEach(Particle::Mat::Flag::HeatLoosing, [=](const int32 i) restrict(amp)
	{
		const Particle::Mat& mat = mats[matIdxs[i]];
		float32& heat = heats[i];
		const float32 loss = step.dt * mat.m_heatConductivity * (heat - roomTemp);
		if (!loss) return;
		heat -= loss * (1 - __dp_math_powf(heatLossRatio,
				//(2.0f - (m_buffers.weight[k] < 0 ? 0 : m_buffers.weight[k] > 1 ? 1 : m_buffers.weight[k])) *
				0.0005f * mat.m_invMass));
	});
}

void ParticleSystem::CopyHeats()
{
	if (IsLastIteration())
		m_ampParts.m_heat.CopyToD11Async();
}
void ParticleSystem::CopyFlags()
{
	if (IsLastIteration())
		m_ampParts.m_flags.CopyToD11Async();
}

void ParticleSystem::SolveFlame()
{
	if (!(m_allFlags & Particle::Mat::Flag::Flame)) return;

	const b2TimeStep& step = m_subStep;
	const float32 lifetime = step.dt * m_def.flameHealthLossPerSec;
	const float32 heatGain = step.dt * m_def.flameHeatGainPerSec;

	auto flags = m_ampParts.m_flags.GetConstView();
	auto matIdxs = m_ampParts.m_matIdx.GetConstView();
	auto mats = m_mats.m_array.GetConstView();
	auto heats = m_ampParts.m_heat.GetView();
	auto healths = m_ampParts.m_health.GetView();
	auto weights = m_ampParts.m_weight.GetConstView();
	m_ampParts.ForEach(Particle::Mat::Flag::Flame, [=](const int32 i) restrict(amp)
	{
		float32& heat = heats[i];
		const Particle::Mat& mat = mats[matIdxs[i]];
		const float32 w = b2Max(2 - weights[i], 0.5f);
		heat += heatGain * mat.m_stability * w;
		healths[i] -= lifetime * mat.m_invStability * w;
	});
}

void ParticleSystem::SolveIgnite()
{
	if (!(m_allFlags & Particle::Mat::Flag::Flame &&
		m_world.m_allBodyMaterialFlags & Body::Mat::Flag::Inflammable))
		return;
	
	auto bodies = GetBodies();
	auto bodyMats = GetConstBodyMats();
	m_ampBodyContacts.ForEach(Particle::Mat::Flag::Flame,
		[=](const int32 i, const Particle::BodyContact& contact) restrict(amp)
	{
		Body& b = bodies[contact.bodyIdx];
		const Body::Mat& bMat = bodyMats[b.m_matIdx];
		if (bMat.m_matFlags & Body::Mat::Flag::Inflammable &&
			bMat.m_ignitionThreshold <= b.m_surfaceHeat)
		{
			b.AddFlag(Body::Flag::Burning);
		}
	});
	auto heats = m_ampParts.m_heat.GetConstView();
	auto matIdxs = m_ampParts.m_matIdx.GetConstView();
	auto mats = m_mats.m_array.GetConstView();
	auto flags = m_ampParts.m_flags.GetView();
	m_ampContacts.ForEach(Particle::Mat::Flag::Flame | Particle::Mat::Flag::Inflammable,
		[=](const Particle::Contact& contact) restrict(amp)
	{	
		const int32 a = contact.idxA;
		const int32 b = contact.idxB;
		const Particle::Mat& aMat = mats[matIdxs[a]];
		const Particle::Mat& bMat = mats[matIdxs[b]];
		const bool aIsFlame = aMat.HasFlag(Particle::Mat::Flag::Flame);
		const bool bIsFlame = bMat.HasFlag(Particle::Mat::Flag::Flame);	
		if (aIsFlame && bIsFlame) return;
		if (aIsFlame)
		{
			if (bMat.HasFlag(Particle::Mat::Flag::Inflammable) && heats[b] >= bMat.m_ignitionThreshold)
				flags[b] |= Particle::Flag::Burning;
		}
		else
		{
			if (aMat.HasFlag(Particle::Mat::Flag::Inflammable) && heats[a] >= aMat.m_ignitionThreshold)
				flags[a] |= Particle::Flag::Burning;
		}
	});
}

void ParticleSystem::SolveExtinguish()
{
	auto flags = m_ampParts.m_flags.GetView();
	auto heats = m_ampParts.m_heat.GetConstView();
	auto matIdxs = m_ampParts.m_matIdx.GetConstView();
	auto mats = m_mats.m_array.GetConstView();
	auto bodies = GetBodies();
	auto bodyMats = GetConstBodyMats();
	m_ampBodyContacts.ForEach(Particle::Mat::Flag::Extinguishing,
		[=](const int32 i, const Particle::BodyContact& contact) restrict(amp)
	{
		Body& b = bodies[contact.bodyIdx];
		if (!b.HasFlag(Body::Flag::Burning)) return;

		if (b.m_surfaceHeat < bodyMats[b.m_matIdx].m_ignitionThreshold)
			b.RemFlag(Body::Flag::Burning);
	});
	m_ampContacts.ForEach(Particle::Flag::Burning | Particle::Mat::Flag::Extinguishing,
		[=](const Particle::Contact& contact) restrict(amp)
	{
		const int32 a = contact.idxA;
		const int32 b = contact.idxB;
		const Particle::Mat& aMat = mats[matIdxs[a]];
		const Particle::Mat& bMat = mats[matIdxs[b]];
		if (uint32& bFlags = flags[b]; aMat.HasFlag(Particle::Mat::Flag::Extinguishing) &&
			bFlags & Particle::Flag::Burning &&
			heats[b] < bMat.m_ignitionThreshold)
			bFlags &= ~Particle::Flag::Burning;

		else if (uint32& aFlags = flags[a]; aMat.HasFlag(Particle::Mat::Flag::Extinguishing) &&
			aFlags & Particle::Flag::Burning &&
			heats[a] < aMat.m_ignitionThreshold)
			aFlags &= ~Particle::Flag::Burning;
	});
}

void ParticleSystem::CopyHealths()
{
	if (IsLastIteration())
		m_ampParts.m_health.CopyToD11Async();
}

void ParticleSystem::SolveSource()
{
	auto flags = m_ampParts.m_flags.GetView();
	auto matIdxs = m_ampParts.m_matIdx.GetConstView();
	auto groundMats = m_world.m_ground->GetConstMats();
	m_ampGroundContacts.ForEach([=](int32 a, const Particle::GroundContact& contact) restrict(amp)
	{
		if (groundMats[contact.groundMatIdx].partMatIdx == matIdxs[a])
			flags[a] = Particle::Flag::Zombie;
	});
}

void ParticleSystem::SolveFluid()
{
	// make Objects wet
	//for (int32 k = 0; k < m_bodyContactCount; k++)
	//{
	//	const Particle::BodyContact& contact = m_bodyContactBuf[k];
	//	int32 a = contact.partIdx;
	//	Body& b = m_world.GetBody(contact.bodyIdx);
	//	Body::Material& mat = m_world.m_bodyMaterials[b.m_matIdx];
	//	if (m_buffers.flagsBuffer[a] & b2_extinguishingMaterial
	//		&& b.m_flags & b2_burningBody
	//		&& mat.m_matFlags & b2_flammableMaterial
	//		&& mat.m_extinguishingPoint > b.m_heat)
	//	{
	//		b.RemFlags((int16)b2_burningBody);
	//	}
	//}
	auto flags = m_ampParts.m_flags.GetView();
	auto groundTiles = m_world.m_ground->GetTiles();
	auto groundMats = m_world.m_ground->GetConstMats();
	auto groundChunkHasChange = m_world.m_ground->GetChunkHasChange();
	auto matIdxs = m_ampParts.m_matIdx.GetConstView();
	m_ampGroundContacts.ForEach(Particle::Mat::Flag::Fluid,
		[=](const int32 i, const Particle::GroundContact& contact) restrict(amp)
	{
		if (flags[i] & Particle::Flag::Controlled) return;
		Ground::Tile& tile = groundTiles[contact.groundTileIdx];
		const Ground::Mat& mat = groundMats[tile.matIdx];
		if (mat.isWaterRepellent()) return;

		const int32 cnt = amp::atomicInc(tile.particleCnt, mat.particleCapacity);
		if (!cnt) return;
		flags[i] = Particle::Flag::Zombie;
		tile.particleMatIdxs[cnt - 1] = matIdxs[i];
		groundChunkHasChange[contact.groundChunkIdx] = 1;
	});

	auto mats = m_mats.m_array.GetConstView();
	auto bodies = GetBodies();
	auto bodyMats = GetConstBodyMats();
	auto heats = m_ampParts.m_heat.GetView();
	auto masses = m_ampParts.m_mass.GetConstView();
	m_ampBodyContacts.ForEach(Particle::Mat::Flag::Fluid,
		[=](const int32 i, const Particle::BodyContact& contact) restrict(amp)
	{
		Body& b = bodies[contact.bodyIdx];
		if (b.HasFlag(Body::Flag::Wet) ||
			bodyMats[b.m_matIdx].HasFlag(Body::Mat::Flag::WaterRepellent))
			return;
		if (!b.atomicAddFlag(Body::Flag::Wet)) return;
		DistributeHeat(heats[i], b.m_surfaceHeat, 1, masses[i], b.m_surfaceMass);
		b.AddFlag(Body::Flag::Wet);
		flags[i] = Particle::Flag::Zombie;
	});

	if (m_allFlags & Particle::Mat::Flag::Fluid)
		m_allFlags |= Particle::Flag::Zombie;
}
void ParticleSystem::SolveKillNotMoving()
{
	const Vec3& wind = m_world.m_wind;
	const float32 minSpeed = m_def.minAirSpeed;
	
	auto flags = m_ampParts.m_flags.GetView();
	auto velocities = m_ampParts.m_velocity.GetConstView();
	auto weights = m_ampParts.m_weight.GetConstView();
	m_ampParts.ForEach(Particle::Mat::Flag::KillIfNotMoving, [=](const int32 i) restrict(amp)
	{
		const Vec3& v = velocities[i];
		if (v.Length() < minSpeed || (v - wind).Length() < minSpeed || weights[i] < 0.2)
			flags[i] = Particle::Flag::Zombie;
	});
	
	if (m_allFlags & Particle::Mat::Flag::KillIfNotMoving)
		m_allFlags |= Particle::Flag::Zombie;
}

void ParticleSystem::SolveChangeMat()
{
	auto flags = m_ampParts.m_flags.GetView();
	auto masses = m_ampParts.m_mass.GetView();
	auto invMasses = m_ampParts.m_invMass.GetView();
	const auto UpdateParticle = [=](int32 idx, const Particle::Mat& newMat) restrict(amp)
	{
		uint32& f = flags[idx];
		f = ((f & Particle::k_mask) & ~Particle::Flag::Controlled) | newMat.m_flags;
		if (!(newMat.m_flags & Particle::Mat::Flag::Inflammable))
			f &= ~Particle::Flag::Burning;
		masses[idx] = newMat.m_mass;
		invMasses[idx] = newMat.m_invMass;
	};
	auto heats = m_ampParts.m_heat.GetConstView();
	auto matIdxs = m_ampParts.m_matIdx.GetView();
	auto mats = m_mats.m_array.GetConstView();

	m_ampParts.ForEach(Particle::Mat::Flag::ChangeWhenCold, [=](const int32 i) restrict(amp)
	{
		int32& matIdx = matIdxs[i];
		const Particle::Mat& mat = mats[matIdx];
		if (heats[i] >= mat.m_coldThreshold) return;
			
		matIdx = mat.m_changeToColdMatIdx;
		if (matIdx != INVALID_IDX)
			UpdateParticle(i, mats[matIdx]);
		else
			flags[i] = Particle::Flag::Zombie;
	});

	m_ampParts.ForEach(Particle::Mat::Flag::ChangeWhenHot, [=](const int32 i) restrict(amp)
	{
		int32& matIdx = matIdxs[i];
		const Particle::Mat& mat = mats[matIdx];
		if (heats[i] <= mat.m_hotThreshold) return;

		matIdx = mat.m_changeToHotMatIdx;
		if (matIdx != INVALID_IDX)
			UpdateParticle(i, mats[matIdx]);
		else
			flags[i] = Particle::Flag::Zombie;
	});

	auto healths = m_ampParts.m_health.GetView();
	m_ampParts.ForEach(Particle::Flag::Burning, [=](const int32 i) restrict(amp)
	{
		if (healths[i] > 0) return;
		int32& matIdx = matIdxs[i];
		const Particle::Mat& mat = mats[matIdx];

		matIdx = mat.m_changeToBurnedMatIdx;
		if (matIdx != INVALID_IDX)
		{
			healths[i] = 1;
			UpdateParticle(i, mats[matIdx]);
		}
		else
			flags[i] = Particle::Flag::Zombie;
	});

	if (IsLastIteration())
		m_ampParts.m_matIdx.CopyToD11Async();

	if (m_allFlags & Particle::Mat::k_changeFlags)
		m_allFlags |= Particle::Flag::Zombie;
}

void ParticleSystem::SolveFreeze()
{
	
}

void ParticleSystem::SolvePosition()
{
	const b2TimeStep& step = m_subStep;
	
	auto positions = m_ampParts.m_position.GetView();
	auto velocities = m_ampParts.m_velocity.GetConstView();
	m_ampParts.ForEach([=](const int32 i) restrict(amp)
	{
		positions[i] += step.dt * velocities[i];
	});
	if (IsLastIteration())
		m_ampParts.m_position.CopyToD11Async();
}
void ParticleSystem::SolveOutOfBounds()
{
	if (!m_world.m_deleteOutside) return;
	const Vec3& lowerBound = m_world.m_lowerBorder;
	const Vec3& upperBound = m_world.m_upperBorder;

	auto positions = m_ampParts.m_position.GetConstView();
	auto flags = m_ampParts.m_flags.GetView();
	m_ampParts.ForEach([=](const int32 i) restrict(amp)
	{
		const Vec3& p = positions[i];
		if (!(p > lowerBound) || !(p < upperBound))
			flags[i] = Particle::Flag::Zombie;
	});
}

void ParticleSystem::IncrementIteration()
{
	m_iteration++;
}

void ParticleSystem::SolveHealth()
{
	auto flags = m_ampParts.m_flags.GetView();
	auto masses = m_ampParts.m_mass.GetView();
	auto invMasses = m_ampParts.m_invMass.GetView();
	const auto UpdateParticle = [=](int32 idx, const Particle::Mat& newMat) restrict(amp)
	{
		flags[idx] = (flags[idx] & Particle::k_mask) | newMat.m_flags;
		masses[idx] = newMat.m_mass;
		invMasses[idx] = newMat.m_invMass;
	};

	auto healths = m_ampParts.m_health.GetView();
	auto mats = m_mats.m_array.GetConstView();
	auto matIdxs = m_ampParts.m_matIdx.GetConstView();
	m_ampParts.ForEach([=](const int32 i) restrict(amp)
	{
		float32& h = healths[i];
		if (h > 0) return;
		if (const int32 deadMatIdx = mats[matIdxs[i]].m_changeToDeadMatIdx; deadMatIdx != INVALID_IDX)
		{
			UpdateParticle(i, mats[deadMatIdx]);
			h = 1;
		}
		else
			flags[i] = Particle::Flag::Zombie;
	});
	m_allFlags |= Particle::Flag::Zombie;
}

void ParticleSystem::CopyBodies()
{
	if (!m_ampBodyContacts.Empty())
		m_ampCopyFutBodies.set(amp::copyAsync(m_ampBodies, m_world.m_bodyBuffer, m_world.m_bodyBuffer.size()));
}

template <class T1, class UnaryPredicate> 
static void ParticleSystem::RemoveFromVectorIf(vector<T1>& v1,
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
//	ampArrayView<const Particle::Contact> contacts(m_ampContacts);
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
//		if ((contacts[tIdx.global].flags & Particle::Flag::b2_zombieParticle) == Particle::Flag::b2_zombieParticle)
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
static void ParticleSystem::RemoveFromVectorsIf(vector<T1>& v1, vector<T2>& v2,
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
static void ParticleSystem::RemoveFromVectorsIf(
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
static void ParticleSystem::RemoveFromVectorsIf(
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
static void ParticleSystem::RemoveFromVectorsIf(
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

void ParticleSystem::SolveZombie()
{
	if (m_ampParts.Empty()) return;
	if (!(m_allFlags & Particle::Flag::Zombie)) return;
	
	auto groupIdxs = m_ampParts.m_groupIdx.GetConstView();
	auto groupHasAlive = m_ampGroupHasAlive.section(0, m_groupCount);
	amp::fill(groupHasAlive, 0u, m_groupCount);
	m_ampParts.ForEach([=](const int32 i) restrict(amp)
	{
		groupHasAlive[groupIdxs[i]] = 1;
	});
	// add new dead groups to zombieRanges
	bool groupWasDestroyed = false;
	for (int32 i = 0; i < m_groupCount; i++)
	{
		auto& group = m_groupBuffer[i];
		if (!groupHasAlive[i] && group.m_firstIndex != INVALID_IDX)
		{
			DestroyGroup(i);
			groupWasDestroyed = true;
		}
	}
	if (groupWasDestroyed)
	{
		ResizeParticleBuffers(m_ampParts.m_count);
		m_ampCopyFutGroups.set(amp::copyAsync(m_groupBuffer, m_ampGroups, m_groupCount));
		m_needsUpdateAllParticleFlags = true;
	}
}

void ParticleSystem::AddZombieRange(int32 firstIdx, int32 lastIdx)
{
	// at the end of the Particle Buffers
	if (m_ampParts.m_count == lastIdx)
	{
		int32 newCnt = firstIdx;
		if (!m_zombieRanges.empty() && newCnt == m_zombieRanges.back().second)	// connected to previous
		{
			newCnt = m_zombieRanges.back().first;
			m_zombieRanges.pop_back();
		}
		ResizeParticleBuffers(newCnt);
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

uint32 ParticleSystem::GetWriteIdx(int32 particleCnt)
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
	writeIdx = m_ampParts.m_count;
	ResizeParticleBuffers(m_ampParts.m_count + particleCnt);
	m_ampParts.m_count += particleCnt;
	return writeIdx;
}

/// Get the time elapsed in b2ParticleSystemDef::lifetimeGranularity.
int32 ParticleSystem::GetQuantizedTimeElapsed() const
{
	return (int32)(m_timeElapsed >> 32);
}

/// Convert a lifetime in seconds to an expiration time.
int64 ParticleSystem::LifetimeToExpirationTime(const float32 lifetime) const
{
	return m_timeElapsed + (int64)((lifetime / m_def.lifetimeGranularity) *
								   (float32)(1LL << 32));
}

template <typename T> void ParticleSystem::SetUserOverridableBuffer(
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

void ParticleSystem::SetFlagsBuffer(uint32* buffer, int32 capacity)
{
	m_buffers.flags.assign(buffer, buffer + capacity);
	//SetUserOverridableBuffer(&m_buffers.flagsBuffer, buffer, capacity);
}

void ParticleSystem::SetPositionBuffer(Vec3* buffer, int32 capacity)
{
	m_buffers.position.assign(buffer, buffer + capacity);
	//Concurrency::copy(buffer, buffer + capacity, m_positionBuffer);
}

void ParticleSystem::SetVelocityBuffer(Vec3* buffer, int32 capacity)
{
	m_buffers.velocity.assign(buffer, buffer + capacity);
	//Concurrency::copy(buffer, buffer + capacity, m_velocityBuffer);
}

void ParticleSystem::SetColorBuffer(int32* buffer, int32 capacity)
{
	m_buffers.color.assign(buffer, buffer + capacity);
}

void ParticleSystem::RemovePartFlagFromAll(const uint32 flag)
{
	const uint32 invFlag = ~flag;
	auto flags = m_ampParts.m_flags.GetView();
	m_ampParts.ForEach(flag, [=](const int32 i) restrict(amp)
	{
		flags[i] &= invFlag;
	});
	m_allFlags &= invFlag;
}

void ParticleSystem::SetGroupFlags(
	ParticleGroup& group, uint32 newFlags)
{
	b2Assert((newFlags & b2_particleGroupInternalMask) == 0);
	uint32& oldFlags = group.m_groupFlags;
	newFlags |= oldFlags & ParticleGroup::Flag::InternalMask;

	if ((oldFlags ^ newFlags) & ParticleGroup::Flag::Solid)
	{
		// If the b2_solidParticleGroup flag changed schedule depth update.
		newFlags |= ParticleGroup::Flag::NeedsUpdateDepth;
	}
	if (oldFlags & ~newFlags)
	{
		// If any flags might be removed
		m_needsUpdateAllGroupFlags = true;
	}
	if (~m_allGroupFlags & newFlags)
	{
		// If any flags were added
		//if (newFlags & ParticleGroup::Flag::Solid)
		//	RequestBuffer(m_buffers.depth, m_hasDepth);
		
		m_allGroupFlags |= newFlags;
	}
	oldFlags = newFlags;
}
void ParticleSystem::UpdateStatistics(const ParticleGroup& group) const
{
	if (group.m_timestamp != m_timestamp)
	{
		const float32 m = m_mats.m_vector[group.m_matIdx].m_mass;
		const int32& firstIdx = group.m_firstIndex;
		const int32& lastIdx = group.m_lastIndex;
		float32& mass = group.m_mass = 0;
		Vec2& center = group.m_center;
		Vec2& linVel = group.m_linearVelocity;
		center.SetZero();
		linVel.SetZero();
		for (int32 i = firstIdx; i < lastIdx; i++)
		{
			mass += m;
			center += m * (Vec2)m_buffers.position[i];
			linVel += m * (Vec2)m_buffers.velocity[i];
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
			Vec2 p = (Vec2)m_buffers.position[i] - center;
			Vec2 v = (Vec2)m_buffers.velocity[i] - linVel;
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

inline bool ParticleSystem::ForceCanBeApplied(uint32 flags) const
{
	return !(flags & Particle::Mat::Flag::Wall);
}

inline void ParticleSystem::PrepareForceBuffer()
{
	if (!m_hasForce)
	{
		amp::fill(m_ampParts.m_force.arr, Vec3_zero, m_ampParts.m_count);
		m_hasForce = true;
	}
}

void ParticleSystem::ApplyForce(const ParticleGroup& group, const Vec3& force)
{
	ApplyForce(group.m_firstIndex, group.m_lastIndex, force);
}

void ParticleSystem::ApplyForce(int32 firstIndex, int32 lastIndex, const Vec3& force)
{
	// Ensure we're not trying to apply force to particles that can't move,
	// such as wall particles.
#if B2_ASSERT_ENABLED
	uint32 flags = 0;
	for (int32 i = firstIndex; i < lastIndex; i++)
	{
		flags |= m_buffers.flagsBuffer[i];
	}
	b2Assert(ForceCanBeApplied(flags));
#endif

	// Early out if force does nothing (optimization).
	const uint32 cnt = lastIndex - firstIndex;
	Vec3 distributedForce = force / (float32)(cnt);
	if (IsSignificantForce(distributedForce))
	{
		PrepareForceBuffer();

		// Distribute the force over all the particles.
		auto forces = m_ampParts.m_force.GetView();
		amp::forEach(firstIndex, lastIndex, [=](const int32 i) restrict(amp)
		{
			forces[i] += distributedForce;
		});
	}
}
void ParticleSystem::PullIntoCircle(const Vec3& pos, const float32 radius,
	float32 strength, uint32 flag, bool ignoreMass, float32 step)
{
	const float32 str = strength * step;
	const float32 invRadius = 0.5f / radius;
	PrepareForceBuffer();
	
	auto positions = m_ampParts.m_position.GetView();
	auto forces = ignoreMass ? m_ampParts.m_velocity.GetView() : m_ampParts.m_force.GetView();
	m_ampParts.ForEach(flag, [=](const int32 i) restrict(amp)
	{
		Vec3& p = positions[i];
		Vec3& f = forces[i];

		const Vec3 diff = pos - p;
		const Vec2 diff2D = diff;
		const float32 strength = b2Min(ampPow(diff2D.Length() * invRadius, 2), 1.0f) * str;
		f += Vec3(diff2D.Normalized() * strength, diff.z > 0 ? str : -str);

		if (IsBetween(pos.z, p.z, p.z + f.z, b2_linearSlop))
		{
			p.z = pos.z;
			f.z = 0;
		}
	});
}
void ParticleSystem::Swirl(const Vec3& center, float32 swirlStrength, float32 pullStrength,
	float32 upDraft, uint32 flag, float32 controlRadius, float32 step)
{
	const float32 swirlStr = swirlStrength * step;
	const float32 pullStr = pullStrength * step;
	PrepareForceBuffer();
	auto positions = m_ampParts.m_position.GetConstView();
	auto forces = m_ampParts.m_force.GetView();
	m_ampParts.ForEach(flag, [=](const int32 i) restrict(amp)
	{
		Vec3 toCenter = center - positions[i];
		float32 toCenterLength = toCenter.Length();
		if (toCenterLength > controlRadius) return;
		toCenter /= toCenterLength;
		const Vec3 rot = b2Cross(Vec3(0, 0, 1), toCenter);
		forces[i] += rot * swirlStr + toCenter * pullStr + Vec3(0, 0, 1) * upDraft;
	});
}

void ParticleSystem::ParticleApplyForce(int32 index, const Vec3& force)
{
	if (IsSignificantForce(force) &&
		ForceCanBeApplied(m_buffers.flags[index]))
	{
		m_buffers.force[index] += force;
	}
}

void ParticleSystem::ApplyLinearImpulse(const ParticleGroup& group, const Vec2& impulse)
{
	ApplyLinearImpulse(group.m_firstIndex, group.m_lastIndex, impulse);
}

void ParticleSystem::ApplyLinearImpulse(int32 firstIndex, int32 lastIndex,
										  const Vec2& impulse)
{
	const float32 numParticles = (float32)(lastIndex - firstIndex);
	//const float32 totalMass = numParticles * GetParticleMass();
	//const Vec2 velocityDelta = impulse / totalMass;	vd = im / np * gpm
	Vec2 velDeltaWithoutMass = impulse / numParticles;
	for (int32 i = firstIndex; i < lastIndex; i++)
	{
		//m_velocityBuffer[i] += velocityDelta;
		Vec2 vel = velDeltaWithoutMass * m_buffers.invMass[i];
		m_buffers.velocity[i] += vel;
	}
}


//void ParticleSystem::RayCast(b2RayCastCallback& callback,
//							   const Vec2& point1,
//							   const Vec2& point2) const
//{
//	if (m_buffers.proxy.empty())
//	{
//		return;
//	}
//	b2AABB aabb;
//	aabb.lowerBound = b2Min(point1, point2);
//	aabb.upperBound = b2Max(point1, point2);
//	float32 fraction = 1;
//	// solving the following equation:
//	// ((1-t)*point1+t*point2-position)^2=diameter^2
//	// where t is a potential fraction
//	Vec2 v = point2 - point1;
//	float32 v2 = b2Dot(v, v);
//	InsideBoundsEnumerator enumerator = GetInsideBoundsEnumerator(aabb);
//	int32 i;
//	while ((i = enumerator.GetNext()) >= 0)
//	{
//		Vec2 p = point1 - m_buffers.position[i];
//		float32 pv = b2Dot(p, v);
//		float32 p2 = b2Dot(p, p);
//		float32 determinant = pv * pv - v2 * (p2 - m_squaredDiameter);
//		if (determinant >= 0)
//		{
//			float32 sqrtDeterminant = b2Sqrt(determinant);
//			// find a solution between 0 and fraction
//			float32 t = (-pv - sqrtDeterminant) / v2;
//			if (t > fraction)
//			{
//				continue;
//			}
//			if (t < 0)
//			{
//				t = (-pv + sqrtDeterminant) / v2;
//				if (t < 0 || t > fraction)
//				{
//					continue;
//				}
//			}
//			Vec2 n = p + t * v;
//			n.Normalize();
//			float32 f = callback.ReportParticle(this, i, point1 + t * v, n, t);
//			fraction = b2Min(fraction, f);
//			if (fraction <= 0)
//			{
//				break;
//			}
//		}
//	}
//}

template<typename S, typename A>
inline ampCopyFuture CopyShapeBufferToGpu(std::vector<S>& shapeBuffer, ampArray<A>& array)
{
	b2Assert(sizeof(S) == sizeof(A));
	return amp::copyAsync(reinterpret_cast<A*>(shapeBuffer.data()), array, shapeBuffer.size());
}

void ParticleSystem::CopyBox2DToGPUAsync()
{
	m_ampCopyFutBodies.set(amp::copyAsync(m_world.m_bodyBuffer, m_ampBodies));
	m_ampCopyFutFixtures.set(amp::copyAsync(m_world.m_fixtureBuffer, m_ampFixtures));
	m_ampCopyFutChainShapes.set(CopyShapeBufferToGpu(m_world.m_chainShapeBuffer, m_ampChainShapes));
	m_ampCopyFutCircleShapes.set(CopyShapeBufferToGpu(m_world.m_circleShapeBuffer, m_ampCircleShapes));
	m_ampCopyFutEdgeShapes.set(CopyShapeBufferToGpu(m_world.m_edgeShapeBuffer, m_ampEdgeShapes));
	m_ampCopyFutPolygonShapes.set(CopyShapeBufferToGpu(m_world.m_polygonShapeBuffer, m_ampPolygonShapes));
}

void ParticleSystem::WaitForCopyBox2DToGPU() 
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

	//static int32 sizea = sizeof(b2PolygonShape);
	//static int32 sizeb = sizeof(AmpPolygonShape);
	//static int32 offsetA = offsetof(b2PolygonShape, b2PolygonShape::m_height);
	//static int32 offsetB = offsetof(AmpPolygonShape, AmpPolygonShape::m_height);
	//if (sizea != sizeb) return;
	//if (offsetA != offsetB) return;
	//
	//static auto& real = m_world.m_polygonShapeBuffer;
	//vector<AmpCircleShape> shapes;
	//shapes.resize(m_ampCircleShapes.extent[0]);
	//amp::copy(m_ampCircleShapes, shapes);
	//return;
}

void ParticleSystem::SetRadius(float32 radius)
{
	m_particleRadius = radius;
	m_inverseRadius = 1 / radius;
	m_particleDiameter = 2 * radius;
	m_squaredDiameter = m_particleDiameter * m_particleDiameter;
	m_cubicDiameter = m_particleDiameter * m_particleDiameter * m_particleDiameter;
	m_inverseDiameter = 1 / m_particleDiameter;
	m_particleVolume = 1; // (4.0 / 3.0)* b2_pi* pow(radius, 3);
	m_atmosphereParticleMass = m_world.m_atmosphericDensity * m_particleVolume;
	m_atmosphereParticleInvMass = 1 / m_atmosphereParticleMass;
}