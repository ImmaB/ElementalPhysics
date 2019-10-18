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
#include <Box2D/Particle/b2Particle.h>
#include <Box2D/Common/b2Draw.h>

#define B2PARTICLECOLOR_BITS_PER_COMPONENT (sizeof(uint8) << 3)
// Maximum value of a b2ParticleColor component.
#define B2PARTICLECOLOR_MAX_VALUE \
	((1U << B2PARTICLECOLOR_BITS_PER_COMPONENT) - 1)

/// Number of bits used to store each b2ParticleColor component.
/*const uint8 b2ParticleColor::k_bitsPerComponent =
	B2PARTICLECOLOR_BITS_PER_COMPONENT;
const float32 b2ParticleColor::k_maxValue = (float)B2PARTICLECOLOR_MAX_VALUE;
const float32 b2ParticleColor::k_inverseMaxValue =
	1.0f / (float)B2PARTICLECOLOR_MAX_VALUE;

b2ParticleColor b2ParticleColor_zero(0, 0, 0, 0);

b2ParticleColor::b2ParticleColor(const b2Color& color)
{
	Set(color);
}

b2Color b2ParticleColor::GetColor() const
{
	return b2Color(k_inverseMaxValue * r,
				   k_inverseMaxValue * g,
				   k_inverseMaxValue * b);
}

void b2ParticleColor::Set(const b2Color& color)
{
	Set((uint8)(k_maxValue * color.r),
		(uint8)(k_maxValue * color.g),
		(uint8)(k_maxValue * color.b),
		B2PARTICLECOLOR_MAX_VALUE);
}*/

int32 b2CalculateParticleIterations(
	float32 gravity, float32 radius, float32 timeStep)
{
	// In some situations you may want more particle iterations than this,
	// but to avoid excessive cycle cost, don't recommend more than this.
	const int32 B2_MAX_RECOMMENDED_PARTICLE_ITERATIONS = 8;
	const float32 B2_RADIUS_THRESHOLD = 0.01f;
	int32 iterations =
		(int32) ceilf(b2Sqrt(gravity / (B2_RADIUS_THRESHOLD * radius)) * timeStep);
	return b2Clamp(iterations, 1, B2_MAX_RECOMMENDED_PARTICLE_ITERATIONS);
}


Particle::Buffers::Buffers(int32 cap) :
	flags(cap),
	position(cap), velocity(cap), force(cap),
	accumulationVec3(cap),
	weight(cap), heat(cap), health(cap),
	mass(cap), invMass(cap),
	staticPressure(cap), accumulation(cap), depth(cap),
	matIdx(cap), groupIdx(cap), color(cap), bodyContactCnt(cap),
	proxy(cap)
{}

void Particle::Buffers::Resize(int32 capacity)
{
	flags.resize(capacity);
	position.resize(capacity);
	velocity.resize(capacity);
	force.resize(capacity);
	accumulationVec3.resize(capacity);
	weight.resize(capacity);
	heat.resize(capacity);
	health.resize(capacity);
	mass.resize(capacity);
	invMass.resize(capacity);
	staticPressure.resize(capacity);
	accumulation.resize(capacity);
	depth.resize(capacity);
	matIdx.resize(capacity);
	groupIdx.resize(capacity);
	color.resize(capacity);
	bodyContactCnt.resize(capacity);
	proxy.resize(capacity);
}


Particle::AmpArrays::AmpArrays(const ampAccelView& accView) :
	flags(accView),
	position(accView), velocity(accView),
	weight(accView), heat(accView), health(accView),
	matIdx(accView), color(accView),
	force(accView),
	accumulationVec3(accView),
	mass(accView), invMass(accView),
	staticPressure(accView), accumulation(accView),
	depth(accView),
	groupIdx(accView),
	contactCnt(accView), bodyContactCnt(accView),
	proxy(accView),
	contactIdx(accView, MIN_PART_CAPACITY * MAX_CONTACTS_PER_PARTICLE),
	contact(accView),
	bodyContactIdx(accView, MIN_PART_CAPACITY * MAX_BODY_CONTACTS_PER_PARTICLE),
	bodyContact(accView),
	groundContact(accView)
{}

void Particle::AmpArrays::Resize(int32 capacity, int32 copyCnt)
{
	amp::resize(flags.arr, capacity, copyCnt);
	amp::resize(position.arr, capacity, copyCnt);
	amp::resize(velocity.arr, capacity, copyCnt);
	amp::resize(force.arr, capacity, copyCnt);
	amp::resize(accumulationVec3.arr, capacity);

	amp::resize(weight.arr, capacity);
	amp::resize(heat.arr, capacity, copyCnt);
	amp::resize(health.arr, capacity, copyCnt);
	amp::resize(mass.arr, capacity, copyCnt);
	amp::resize(invMass.arr, capacity, copyCnt);
	amp::resize(staticPressure.arr, capacity, copyCnt);
	amp::resize(accumulation.arr, capacity);
	amp::resize(depth.arr, capacity);

	amp::resize(matIdx.arr, capacity, copyCnt);
	amp::resize(groupIdx.arr, capacity, copyCnt);
	amp::resize(color.arr, capacity, copyCnt);

	amp::resize(contactCnt.arr, capacity);
	amp::resize(bodyContactCnt.arr, capacity);
	amp::resize(proxy.arr, capacity);

	amp::resize(contactIdx.arr, capacity * MAX_CONTACTS_PER_PARTICLE);
	amp::resize(contact.arr, capacity);
	amp::resize(bodyContactIdx.arr, capacity * MAX_BODY_CONTACTS_PER_PARTICLE);
	amp::resize(bodyContact.arr, capacity);
	amp::resize(groundContact.arr, capacity);
}

void Particle::AmpArrays::SetD11Buffers(ID3D11Buffer** ppBufs)
{
	if (!ppBufs) return;

	flags.SetD11Arr(ppBufs[0]);
	position.SetD11Arr(ppBufs[1]);
	velocity.SetD11Arr(ppBufs[2]);
	weight.SetD11Arr(ppBufs[3]);
	heat.SetD11Arr(ppBufs[4]);
	health.SetD11Arr(ppBufs[5]);
	matIdx.SetD11Arr(ppBufs[6]);
	color.SetD11Arr(ppBufs[7]);
}

void Particle::AmpArrays::WaitForCopies()
{
	flags.copyFuture.wait();
	flags.copyFuture.wait();
	position.copyFuture.wait();
	velocity.copyFuture.wait();
	weight.copyFuture.wait();
	heat.copyFuture.wait();
	health.copyFuture.wait();
	matIdx.copyFuture.wait();
	color.copyFuture.wait();
}


template<typename T> inline void ReplaceArray(ampArray<T>& arr, ID3D11Buffer* pNewBuf, int32 size, int32 copyCnt)
{
	ampArray<T> temp = pNewBuf ? 
		Concurrency::direct3d::make_array<T>(ampExtent(size), arr.accelerator_view, pNewBuf) :
		ampArray<T>(size, arr.accelerator_view);
	if (copyCnt)
	{
		if (arr.extent[0] <= size) Concurrency::copy(arr, temp);
		else Concurrency::copy(arr.section(0, copyCnt), temp);
	}
	arr = temp;
}

void Particle::CopyBufferRangeToAmpArrays(Buffers& bufs, AmpArrays& arrs,
	int32 first, int32 last)
{
	const int32 size = last - first;
	if (size <= 0) return;
	amp::copy(bufs.flags, arrs.flags.arr, first, size);
	amp::copy(bufs.position, arrs.position.arr, first, size);
	amp::copy(bufs.velocity, arrs.velocity.arr, first, size);
	amp::copy(bufs.force, arrs.force.arr, first, size);
	amp::copy(bufs.heat, arrs.heat.arr, first, size);
	amp::copy(bufs.health, arrs.health.arr, first, size);
	amp::copy(bufs.mass, arrs.mass.arr, first, size);
	amp::copy(bufs.invMass, arrs.invMass.arr, first, size);
	amp::copy(bufs.staticPressure, arrs.staticPressure.arr, first, size);
	amp::copy(bufs.depth, arrs.depth.arr, first, size);
	amp::copy(bufs.matIdx, arrs.matIdx.arr, first, size);
	amp::copy(bufs.groupIdx, arrs.groupIdx.arr, first, size);
	amp::copy(bufs.color, arrs.color.arr, first, size);
}


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