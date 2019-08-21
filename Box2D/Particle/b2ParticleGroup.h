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

#include <Box2D/Particle/b2Particle.h>
#include <Box2D/Collision/Shapes/b2Shape.h>

struct b2Shape;
class b2World;
class b2ParticleSystem;
struct ParticleGroup;
class b2ParticleColor;

/// A group of particles. ParticleGroup::CreateParticleGroup creates these.
struct ParticleGroup
{
	enum Flag
	{
		/// Prevents overlapping or leaking.
		Solid = 1 << 0,
		/// Keeps its shape.
		Rigid = 1 << 1,
		/// Won't be destroyed if it gets empty.
		CanBeEmpty = 1 << 2,
		/// Will be destroyed on next simulation step.
		WillBeDestroyed = 1 << 3,
		/// Updates depth data on next simulation step.
		NeedsUpdateDepth = 1 << 4,
		InternalMask = WillBeDestroyed | NeedsUpdateDepth,
	};

	struct Def
	{
		Def()
		{
			idx = 0;
			flags = 0;
			groupFlags = 0;
			transform = b2Transform();
			linearVelocity = b2Vec3_zero;
			angularVelocity = 0;
			color = 0;
			shapeType = b2Shape::e_typeCount;
			shapeIdx = INVALID_IDX;
			shapeCount = 0;
			stride = 0;
			particleCount = 0;
			groupIdx = INVALID_IDX;
			matIdx = INVALID_IDX;
			collisionGroup = 0;
			heat = 0.0f;
			health = 1.0f;
			timestamp = INVALID_IDX;
		}
		int32 idx;

		uint32 flags;
		uint32 groupFlags;

		/// The world position of the group.
		/// Moves the group's shape a distance equal to the value of position.
		b2Transform transform;
		b2Vec3 linearVelocity;
		float32 angularVelocity;

		int32 color;

		/// The shape where particles will be added.
		b2Shape::Type shapeType;
		int32 shapeIdx;
		/// The number of shapes.
		int32 shapeCount;
		/// The interval of particles in the shape.
		/// If it is 0, b2_particleStride * particleDiameter is used instead.
		float32 stride;

		/// The initial positions of the particleCount particles.
		int32 particleCount;
		std::vector<b2Vec3> positionData;
		std::vector<int32> colorData;

		/// An existing particle group to which the particles will be added.
		int32 groupIdx;
		int32 matIdx;
		int32 collisionGroup;

		float32 heat;
		float32 health;

		int32 timestamp;
	};

	/// Get the number of particles.
	int32 GetParticleCount() const;
	int32 GetFirstIndex() const;
	int32 GetLastIndex() const;

	/// Get the offset of this group in the global particle buffer
	int32 GetBufferIndex() const;

	/// Does this group contain the particle.
	bool ContainsParticle(int32 index) const;
	
	/// Get the construction flags for the group.
	uint32 GetGroupFlags() const;
	inline bool HasFlag(uint32 flag) const { return m_groupFlags & flag; }
	inline bool HasFlag(uint32 flag) const restrict(amp) { return m_groupFlags & flag; }
	
	int32 GetMaterialIdx() const;

	/// Get the position of the group's origin and rotation.
	/// Used only with groups of rigid particles.
	const b2Transform& GetTransform() const;

	/// Get position of the particle group as a whole.
	/// Used only with groups of rigid particles.
	const b2Vec2& GetPosition() const;

	/// Get the rotational angle of the particle group as a whole.
	/// Used only with groups of rigid particles.
	float32 GetAngle() const;

	/// Get the user data pointer that was provided in the group definition.
	int32 GetUserData() const;

	/// Set the user data. Use this to store your application specific data.
	void SetUserData(int32 data);

	ParticleGroup()
	{
		m_firstIndex = 0;
		m_lastIndex = 0;
		m_groupFlags = 0;
		m_strength = 1.0f;
		m_matIdx = INVALID_IDX;
		m_collisionGroup = 0;
		m_timestamp = INVALID_IDX;
		m_mass = 0;
		m_inertia = 0;
		m_center = b2Vec2_zero;
		m_linearVelocity = b2Vec2_zero;
		m_angularVelocity = 0;
		m_transform.SetIdentity();

		m_userData = NULL;
	};

	//b2ParticleSystem* m_system;
	int32 m_firstIndex, m_lastIndex;
	uint32 m_groupFlags;
	float32 m_strength;
	int32 m_matIdx;
	int32 m_collisionGroup;

	int32 m_userData;

	mutable int32 m_timestamp;
	mutable float32 m_mass;
	mutable float32 m_inertia;
	mutable b2Vec2 m_center;
	mutable b2Vec2 m_linearVelocity;
	mutable float32 m_angularVelocity;
	mutable b2Transform m_transform;

};

//inline ParticleGroup* ParticleGroup::GetNext()
//{
//	return m_next;
//}
//
//inline const ParticleGroup* ParticleGroup::GetNext() const
//{
//	return m_next;
//}

//inline b2ParticleSystem* ParticleGroup::GetParticleSystem()
//{
//	return m_system;
//}
//
//inline const b2ParticleSystem* ParticleGroup::GetParticleSystem() const
//{
//	return m_system;
//}

inline int32 ParticleGroup::GetParticleCount() const
{
	return m_lastIndex - m_firstIndex;
}

inline int32 ParticleGroup::GetFirstIndex() const
{
	return m_firstIndex;
}

inline int32 ParticleGroup::GetLastIndex() const
{
	return m_lastIndex;
}

inline bool ParticleGroup::ContainsParticle(int32 index) const
{
	return m_firstIndex <= index && index < m_lastIndex;
}

inline int32 ParticleGroup::GetBufferIndex() const
{
  return m_firstIndex;
}

inline uint32 ParticleGroup::GetGroupFlags() const
{
	return m_groupFlags & ~InternalMask;
}

inline int32 ParticleGroup::GetMaterialIdx() const
{
	return m_matIdx;
}

inline const b2Transform& ParticleGroup::GetTransform() const
{
	return m_transform;
}

inline const b2Vec2& ParticleGroup::GetPosition() const
{
	return m_transform.p;
}

inline float32 ParticleGroup::GetAngle() const
{
	return m_transform.q.GetAngle();
}


inline int32 ParticleGroup::GetUserData() const
{
	return m_userData;
}

inline void ParticleGroup::SetUserData(int32 data)
{
	m_userData = data;
}
