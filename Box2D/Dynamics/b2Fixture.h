/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Collision/Shapes/b2Shape.h>

class b2BlockAllocator;
class b2Body;
class b2BroadPhase;
class b2Fixture;
struct Fixture;

/// This holds contact filtering data.
struct b2Filter
{
	b2Filter()
	{
		categoryBits = 0x00000001;
		maskBits = 0xFFFFFFFF;
		groupIndex = 0;
	}

	/// The collision category bits. Normally you would just set one bit.
	uint32 categoryBits;

	/// The collision mask bits. This states the categories that this
	/// shape would accept for collision.
	uint32 maskBits;

	/// Collision groups allow a certain group of objects to never collide (negative)
	/// or always collide (positive). Zero means no collision group. Non-zero group
	/// filtering always wins against the mask bits.
	int32 groupIndex;
};

/// A fixture definition is used to create a fixture. This class defines an
/// abstract fixture definition. You can reuse fixture definitions safely.
struct b2FixtureDef
{
	/// The constructor sets the default fixture definition values.
	b2FixtureDef()
	{
		bodyIdx = b2_invalidIndex;
		shapeType = b2Shape::Type::e_typeCount;
		shapeIdx = b2_invalidIndex;
		friction = 0.2f;
		restitution = 0.0f;
		density = 0.0f;
		isSensor = false;
	}

	int32 bodyIdx;

	/// The shape, this must be set. The shape will be cloned, so you
	/// can create the shape on the stack.
	b2Shape::Type shapeType;
	int32 shapeIdx;

	/// The friction coefficient, usually in the range [0,1].
	float32 friction;

	/// The restitution (elasticity) usually in the range [0,1].
	float32 restitution;

	/// The density, usually in kg/m^2.
	float32 density;

	/// A sensor shape collects contact information but never generates a collision
	/// response.
	bool isSensor;

	/// Contact filtering data.
	b2Filter filter;
};

/// This proxy is used internally to connect fixtures to the broad-phase.
struct b2FixtureProxy
{
	b2AABB aabb;
	int32 fixtureIdx;
	int32 childIndex;
	int32 proxyId;

	b2FixtureProxy()
	{
		fixtureIdx = b2_invalidIndex;
		proxyId = b2_invalidIndex;
		childIndex = b2_invalidIndex;
	}
};

struct Fixture
{
	int32 m_idx;

	float32 m_density;

	int32 m_bodyIdx;
	int32 m_idxInBody;

	b2Shape::Type m_shapeType;
	int32 m_shapeIdx;

	float32 m_friction;
	float32 m_restitution;

	int32 m_proxyCount;

	b2Filter m_filter;

	bool m_isSensor;

	float32 m_zPos;
	float32 m_height;

	bool TestZPos(float32 z) const
	{
		return m_zPos <= z && z <= m_zPos + m_height;
	}
	bool TestZPos(float32 z) const restrict(amp)
	{
		return m_zPos <= z && z <= m_zPos + m_height;
	}


	void Set(const b2FixtureDef& def, const int32 idxInBody);

	/// Set the density of this fixture. This will _not_ automatically adjust the mass
	/// of the body. You must call b2Body::ResetMassData to update the body's mass.
	void SetDensity(float32 density);
};

inline void Fixture::SetDensity(float32 density)
{
	b2Assert(b2IsValid(density) && density >= 0.0f);
	m_density = density;
}
