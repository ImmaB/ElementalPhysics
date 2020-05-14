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
	weight(cap), heat(cap), health(cap),
	mass(cap), invMass(cap),
	staticPressure(cap), accumulation(cap), depth(cap),
	matIdx(cap), groupIdx(cap), color(cap)
{}

void Particle::Buffers::Resize(int32 capacity)
{
	flags.resize(capacity);
	position.resize(capacity);
	velocity.resize(capacity);
	force.resize(capacity);
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
}


Particle::AmpArrays::AmpArrays(const ampAccelView& accView) :
	m_flags(accView),
	m_position(accView), m_velocity(accView),
	m_weight(accView), m_heat(accView), m_health(accView),
	m_matIdx(accView), m_color(accView),
	m_force(accView),
	m_accumulationVec3(accView),
	m_mass(accView), m_invMass(accView),
	m_staticPressure(accView), m_accumulation(accView),
	m_depth(accView),
	m_groupIdx(accView),
	m_proxy(accView),
	m_count(0), m_capacity(0)
{}

bool Particle::AmpArrays::Resize(int32 size)
{
	const int32 lastCnt = m_count;
	m_count = size;
	if (!AdjustCapacityToSize(m_capacity, size, MIN_PART_CAPACITY)) return false;

	m_iterations = 1 + ((m_capacity - 1) / CONTACT_THREADS);

	m_flags.Resize(m_capacity, lastCnt);
	m_position.Resize(m_capacity, lastCnt);
	m_velocity.Resize(m_capacity, lastCnt);
	m_force.Resize(m_capacity, lastCnt);
	m_accumulationVec3.Resize(m_capacity);

	m_weight.Resize(m_capacity);
	m_heat.Resize(m_capacity, lastCnt);
	m_health.Resize(m_capacity, lastCnt);
	m_mass.Resize(m_capacity, lastCnt);
	m_invMass.Resize(m_capacity, lastCnt);
	m_staticPressure.Resize(m_capacity, lastCnt);
	m_accumulation.Resize(m_capacity);
	m_depth.Resize(m_capacity);

	m_matIdx.Resize(m_capacity, lastCnt);
	m_groupIdx.Resize(m_capacity, lastCnt);
	m_color.Resize(m_capacity, lastCnt);

	m_proxy.Resize(m_capacity);

	return true;
}

void Particle::AmpArrays::SetD11Buffers(ID3D11Buffer** ppBufs)
{
	if (!ppBufs)
	{
		m_flags.SetD11Arr(nullptr);
		m_position.SetD11Arr(nullptr);
		m_velocity.SetD11Arr(nullptr);
		m_weight.SetD11Arr(nullptr);
		m_heat.SetD11Arr(nullptr);
		m_health.SetD11Arr(nullptr);
		m_matIdx.SetD11Arr(nullptr);
		m_color.SetD11Arr(nullptr);
	}
	else
	{
		m_flags.SetD11Arr(ppBufs[0]);
		m_position.SetD11Arr(ppBufs[1]);
		m_velocity.SetD11Arr(ppBufs[2]);
		m_weight.SetD11Arr(ppBufs[3]);
		m_heat.SetD11Arr(ppBufs[4]);
		m_health.SetD11Arr(ppBufs[5]);
		m_matIdx.SetD11Arr(ppBufs[6]);
		m_color.SetD11Arr(ppBufs[7]);
	}
}

void Particle::AmpArrays::WaitForCopies()
{
	m_flags.copyFuture.wait();
	m_position.copyFuture.wait();
	m_velocity.copyFuture.wait();
	m_weight.copyFuture.wait();
	m_heat.copyFuture.wait();
	m_health.copyFuture.wait();
	m_matIdx.copyFuture.wait();
	m_color.copyFuture.wait();
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
	amp::copy(bufs.flags, arrs.m_flags.arr, first, size);
	amp::copy(bufs.position, arrs.m_position.arr, first, size);
	amp::copy(bufs.velocity, arrs.m_velocity.arr, first, size);
	amp::copy(bufs.force, arrs.m_force.arr, first, size);
	amp::copy(bufs.heat, arrs.m_heat.arr, first, size);
	amp::copy(bufs.health, arrs.m_health.arr, first, size);
	amp::copy(bufs.mass, arrs.m_mass.arr, first, size);
	amp::copy(bufs.invMass, arrs.m_invMass.arr, first, size);
	amp::copy(bufs.staticPressure, arrs.m_staticPressure.arr, first, size);
	amp::copy(bufs.depth, arrs.m_depth.arr, first, size);
	amp::copy(bufs.matIdx, arrs.m_matIdx.arr, first, size);
	amp::copy(bufs.groupIdx, arrs.m_groupIdx.arr, first, size);
	amp::copy(bufs.color, arrs.m_color.arr, first, size);
}


Particle::Mat::Mat(const Particle::Mat::Def& def)
{
	m_flags = def.flags;
	m_mass = def.mass;
	m_invMass = 1 / def.mass;
	m_stability = def.stability;
	m_invStability = 1 / def.stability;
	m_heatConductivity = def.heatConductivity;
	m_strength = def.strength;
}

Particle::MatArray::MatArray(const ampAccelView& accelView) :
	m_array(accelView, b2_minPartMatBufferCapacity),
	m_capacity(0), m_count(0)
{}

int32 Particle::MatArray::Add(Particle::Mat::Def& def)
{
	const int32 idx = m_count++;
	if (m_count > m_capacity)
	{
		if (!m_capacity) m_capacity = 1;
		else m_capacity *= 2;
		m_array.Resize(m_capacity);
		m_vector.resize(m_capacity);
	}
	m_vector[idx] = Particle::Mat(def);

	amp::copy(m_vector[idx], m_array.arr, idx);
	return idx;
}

void Particle::MatArray::AddChange(const int32 idx, const Particle::Mat::ChangeDef& changeDef)
{
	m_vector[idx].SetMatChanges(changeDef);
	amp::copy(m_vector[idx], m_array.arr, idx);
}
