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


Particle::AmpArrays::AmpArrays(int32 cap, const ampAccelView& accView) :
	flags(cap, accView),
	position(cap, accView), velocity(cap, accView), force(cap, accView),
	accumulationVec3(cap, accView),
	weight(cap, accView), heat(cap, accView), health(cap, accView),
	mass(cap, accView), invMass(cap, accView),
	staticPressure(cap, accView), accumulation(cap, accView), depth(cap, accView),
	matIdx(cap, accView), groupIdx(cap, accView), color(cap, accView),
	contactCnt(cap, accView), bodyContactCnt(cap, accView), proxy(cap, accView),
	contact(cap, MAX_CONTACTS_PER_PARTICLE, accView),
	bodyContact(cap, MAX_CONTACTS_PER_PARTICLE, accView),
	groundContact(cap, accView)
{}

void Particle::AmpArrays::Resize(int32 capacity, int32 copyCnt)
{
	amp::resize(flags, capacity, copyCnt);
	amp::resize(position, capacity, copyCnt);
	amp::resize(velocity, capacity, copyCnt);
	amp::resize(force, capacity, copyCnt);
	amp::resize(accumulationVec3, capacity);

	amp::resize(weight, capacity);
	amp::resize(heat, capacity, copyCnt);
	amp::resize(health, capacity, copyCnt);
	amp::resize(mass, capacity, copyCnt);
	amp::resize(invMass, capacity, copyCnt);
	amp::resize(staticPressure, capacity, copyCnt);
	amp::resize(accumulation, capacity);
	amp::resize(depth, capacity);

	amp::resize(matIdx, capacity, copyCnt);
	amp::resize(groupIdx, capacity, copyCnt);
	amp::resize(color, capacity, copyCnt);

	amp::resize(contactCnt, capacity);
	amp::resize(bodyContactCnt, capacity);
	amp::resize(proxy, capacity);

	amp::resize(contact, capacity);
	amp::resize(bodyContact, capacity);
	amp::resize(groundContact, capacity);
}

void Particle::CopyBufferRangeToAmpArrays(Buffers& bufs, AmpArrays& arrs, int32 first, int32 last)
{
	const int32 size = last - first;
	if (size <= 0) return;
	amp::copy(bufs.flags, arrs.flags, first, size);
	amp::copy(bufs.position, arrs.position, first, size);
	amp::copy(bufs.velocity, arrs.velocity, first, size);
	amp::copy(bufs.force, arrs.force, first, size);
	amp::copy(bufs.heat, arrs.heat, first, size);
	amp::copy(bufs.health, arrs.health, first, size);
	amp::copy(bufs.mass, arrs.mass, first, size);
	amp::copy(bufs.invMass, arrs.invMass, first, size);
	amp::copy(bufs.staticPressure, arrs.staticPressure, first, size);
	amp::copy(bufs.depth, arrs.depth, first, size);
	amp::copy(bufs.matIdx, arrs.matIdx, first, size);
	amp::copy(bufs.groupIdx, arrs.groupIdx, first, size);
}

void Particle::CopyAmpArraysToD11Buffers(D11Device& device, AmpArrays& arrs, D11Buffers& d11Bufs, int32 cnt)
{
	if (cnt <= 0) return;
	bool didCopy = false;

	ID3D11Buffer* pFlags = reinterpret_cast<ID3D11Buffer*>(concurrency::direct3d::get_buffer(arrs.flags));
	if (pFlags && d11Bufs.flags)
		didCopy |= device.copyRegion<uint32>(pFlags, d11Bufs.flags, 0, cnt);
	ID3D11Buffer* pPosition = reinterpret_cast<ID3D11Buffer*>(concurrency::direct3d::get_buffer(arrs.position));
	if (pPosition && d11Bufs.position)
		didCopy |= device.copyRegion<Vec3>(pPosition , d11Bufs.position, 0, cnt);
	ID3D11Buffer* pVelocity = reinterpret_cast<ID3D11Buffer*>(concurrency::direct3d::get_buffer(arrs.velocity));
	if (pVelocity && d11Bufs.velocity)
		didCopy |= device.copyRegion<Vec3>(pVelocity, d11Bufs.velocity, 0, cnt);
	ID3D11Buffer* pWeight = reinterpret_cast<ID3D11Buffer*>(concurrency::direct3d::get_buffer(arrs.weight));
	if (pWeight && d11Bufs.weight)
		didCopy |= device.copyRegion<float32>(pWeight, d11Bufs.weight, 0, cnt);
	ID3D11Buffer* pHeat = reinterpret_cast<ID3D11Buffer*>(concurrency::direct3d::get_buffer(arrs.heat));
	if (pHeat && d11Bufs.heat)
		didCopy |= device.copyRegion<float32>(pHeat, d11Bufs.heat, 0, cnt);
	ID3D11Buffer* pHealth = reinterpret_cast<ID3D11Buffer*>(concurrency::direct3d::get_buffer(arrs.health));
	if (pHealth && d11Bufs.health)
		didCopy |= device.copyRegion<float32>(pHealth, d11Bufs.health, 0, cnt);
	ID3D11Buffer* pMatIdx = reinterpret_cast<ID3D11Buffer*>(concurrency::direct3d::get_buffer(arrs.matIdx));
	if (pMatIdx && d11Bufs.matIdx)
		didCopy |= device.copyRegion<int32>(pMatIdx, d11Bufs.matIdx, 0, cnt);
	ID3D11Buffer* pColor = reinterpret_cast<ID3D11Buffer*>(concurrency::direct3d::get_buffer(arrs.color));
	if (pColor && d11Bufs.color)
		didCopy |= device.copyRegion<int32>(pColor, d11Bufs.color, 0, cnt);

	if (didCopy) device.Flush();
	
	if (pFlags) pFlags->Release();
	if (pPosition) pPosition->Release();
	if (pVelocity) pVelocity->Release();
	if (pWeight) pWeight->Release();
	if (pHeat) pHeat->Release();
	if (pHealth) pHealth->Release();
	if (pMatIdx) pMatIdx->Release();
	if (pColor) pColor->Release();
}

void Particle::D11Buffers::Set(ID3D11Buffer** bufPtrs)
{
	if (bufPtrs && *bufPtrs)
	{
		flags = bufPtrs[0];
		position = bufPtrs[1];
		velocity = bufPtrs[2];
		weight = bufPtrs[3];
		heat = bufPtrs[4];
		health = bufPtrs[5];
		matIdx = bufPtrs[6];
		color = bufPtrs[7];
	}
	else
	{
		flags = position = velocity = weight = heat = health = matIdx = color = nullptr;
	}
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