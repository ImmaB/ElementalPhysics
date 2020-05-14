
#include <Box2D/Particle/b2ParticleContact.h>


inline bool Particle::Contact::operator==(const Particle::Contact& rhs) const
{
	return idxA == rhs.idxA
		&& idxB == rhs.idxB
		&& flags == rhs.flags
		&& weight == rhs.weight
		&& normal == rhs.normal;
}

// The reciprocal sqrt function differs between SIMD and non-SIMD, but they
// should create approximately equal results.
inline bool Particle::Contact::ApproximatelyEqual(const Particle::Contact& rhs) const
{
	static const float MAX_WEIGHT_DIFF = 0.01f; // Weight 0 ~ 1, so about 1%
	static const float MAX_NORMAL_DIFF = 0.01f; // Normal length = 1, so 1%
	return idxA == rhs.idxA
		&& idxB == rhs.idxB
		&& flags == rhs.flags
		&& b2Abs(weight - rhs.weight) < MAX_WEIGHT_DIFF
		&& (normal - rhs.normal).Length() < MAX_NORMAL_DIFF;
}




Particle::ContactArrays::ContactArrays(const ampAccelView& accView, const Particle::AmpArrays& particleArrays) :
	m_particleArrays(particleArrays),
	m_cnt(accView, CONTACT_THREADS / TILE_SIZE),
	m_idx(accView, MIN_PART_CAPACITY * MAX_CONTACTS_PER_PARTICLE),
	m_perTile(accView, CONTACT_THREADS / TILE_SIZE, 1),
	m_count(0), m_capacity(0), m_maxPerTile(1)
{}

bool Particle::ContactArrays::Resize(int32 size)
{
	if (!AdjustCapacityToSize(m_capacity, size, MIN_PART_CAPACITY)) return false;

	m_idx.Resize(m_capacity);
	m_maxPerTile = TILE_SIZE * m_particleArrays.m_iterations * MAX_CONTACTS_PER_PARTICLE;
	m_perTile.Resize2ndDim(m_maxPerTile);

	return true;
}


Particle::BodyContactArrays::BodyContactArrays(const ampAccelView& accView,
	const Particle::AmpArrays& particleArrays) :
	m_particleArrays(particleArrays),
	m_array(accView),
	m_cnt(accView),
	m_idx(accView, MIN_PART_CAPACITY * MAX_BODY_CONTACTS_PER_PARTICLE),
	m_count(0), m_capacity(0)
{}

void Particle::BodyContactArrays::Resize(int32 size)
{
	if (!AdjustCapacityToSize(m_capacity, size, MIN_PART_CAPACITY)) return;

	m_idx.Resize(m_capacity);
	m_array.Resize(m_particleArrays.m_capacity);
	m_cnt.Resize(m_particleArrays.m_capacity);
}


Particle::GroundContactArrays::GroundContactArrays(const ampAccelView& accView,
	const Particle::AmpArrays& particleArrays) :
	m_particleArrays(particleArrays),
	m_array(accView),
	m_count(0), m_capacity(0)
{}

void Particle::GroundContactArrays::Resize(int32 size)
{
	if (!AdjustCapacityToSize(m_capacity, size, MIN_PART_CAPACITY)) return;

	m_array.Resize(m_capacity);
}
