/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <Box2D/Dynamics/Joints/b2Joint.h>



void Body::Set(const b2BodyDef def)
{
	b2Assert(def.position.IsValid());
	b2Assert(def.linearVelocity.IsValid());
	b2Assert(b2IsValid(def.angle));
	b2Assert(b2IsValid(def.angularVelocity));
	b2Assert(b2IsValid(def.angularDamping) && def.angularDamping >= 0.0f);
	b2Assert(b2IsValid(def.linearDamping) && def.linearDamping >= 0.0f);
	b2Assert(b2IsValid(def.heat));
	b2Assert(def.material == NULL);

	m_flags = def.flags;
	if (def.awake)
		m_flags |= b2_awakeBody;
	if (def.active)
		m_flags |= b2_activeBody;

	m_xf.p = def.position;
	m_xf.q.Set(def.angle);
	m_xf0 = m_xf;

	m_sweep.localCenter.SetZero();
	m_sweep.c0 = m_xf.p;
	m_sweep.c = m_xf.p;
	m_sweep.a0 = def.angle;
	m_sweep.a = def.angle;
	m_sweep.alpha0 = 0.0f;

	m_jointList = NULL;
	m_contactList = NULL;

	m_linearVelocity = def.linearVelocity;
	m_angularVelocity = def.angularVelocity;

	m_linearDamping = def.linearDamping;
	m_angularDamping = def.angularDamping;
	m_gravityScale = def.gravityScale;

	m_force.SetZero();
	m_torque = 0.0f;

	m_sleepTime = 0.0f;

	m_type = def.type;

	if (m_type == b2_dynamicBody)
	{
		m_mass = 1.0f;
		m_invMass = 1.0f;
	}
	else
	{
		m_mass = 0.0f;
		m_invMass = 0.0f;
	}

	m_I = 0.0f;
	m_invI = 0.0f;
	
	m_materialIdx = def.materialIdx;
	m_heat = def.heat;

	m_health = def.health;

	m_fixtureIdxBuffer = vector<int32>();
}

void b2Body::SetType(b2BodyType type)
{
	b2Assert(m_world.IsLocked() == false);
	if (m_world.IsLocked() == true)
	{
		return;
	}

	if (m_type == type)
	{
		return;
	}

	m_type = type;

	ResetMassData();

	if (m_type == b2_staticBody)
	{
		m_linearVelocity.SetZero();
		m_angularVelocity = 0.0f;
		m_sweep.a0 = m_sweep.a;
		m_sweep.c0 = m_sweep.c;
		SynchronizeFixtures();
	}

	SetAwake(true);

	m_force.SetZero();
	m_torque = 0.0f;

	// Delete the attached contacts.
	b2ContactEdge* ce = m_contactList;
	while (ce)
	{
		b2ContactEdge* ce0 = ce;
		ce = ce->next;
		m_world.m_contactManager.Destroy(ce0->contact);
	}
	m_contactList = NULL;

	// Touch the proxies so that new contacts will be created (when appropriate)
	b2BroadPhase* broadPhase = &m_world.m_contactManager.m_broadPhase;
	for each (int32 fIdx in m_fixtureIdxBuffer)
	{
		b2Fixture& f = m_world.m_fixtureBuffer[fIdx];
		int32 proxyCount = f.m_proxyCount;
		for (int32 i = 0; i < proxyCount; ++i)
		{
			broadPhase->TouchProxy(f.m_proxies[i].proxyId);
		}
	}
}

int32 b2Body::CreateFixture(int32 bodyIdx, b2FixtureDef& def)
{
	b2Assert(m_world.IsLocked() == false);
	if (m_world.IsLocked() == true) return b2_invalidIndex;
	
	const int32 fIdx = m_world.GetBufferInsertIdx(m_world.m_fixtureBuffer, m_world.m_freeFixtureIdxs);
	Fixture& f = m_world.m_fixtureBuffer[fIdx];
	f.Set(def, bodyIdx);

	if (m_flags & b2_activeBody)
	{
		b2BroadPhase* broadPhase = &m_world.m_contactManager.m_broadPhase;
		fixture.CreateProxies(broadPhase, m_xf);
	}

	// Adjust mass properties if needed.
	if (f.m_density > 0.0f)
	{
		ResetMassData();
	}

	// Let the world know we have a new fixture. This will cause new contacts
	// to be created at the beginning of the next time step.
	m_world.m_flags |= b2World::e_newFixture;

	return fIdx;
}

int32 b2Body::CreateFixture(int32 bodyIdx, const b2Shape* shape, float32 density)
{
	b2FixtureDef def;
	def.shape = shape;
	def.density = density;

	return CreateFixture(bodyIdx, def);
}

void b2Body::DestroyFixture(int32 idx)
{
	b2Assert(m_world.IsLocked() == false);
	if (m_world.IsLocked() == true) return;

	Fixture& fixture = m_world.m_fixtureBuffer[idx];

	b2Assert(fixture.m_body.m_idx == m_idx);
	m_fixtureIdxBuffer[idx] = b2_invalidIndex;
	m_fixtureIdxFreeSlots.push_back(idx);
	// Remove the fixture from this body's singly linked list.
	b2Assert(m_fixtureCount > 0);

	// You tried to remove a shape that is not attached to this body.
	b2Assert(found);

	// Destroy any contacts associated with the fixture.
	b2ContactEdge* edge = m_contactList;
	while (edge)
	{
		b2Contact* c = edge->contact;
		edge = edge->next;

		b2Fixture* fixtureA = c->GetFixtureA();
		b2Fixture* fixtureB = c->GetFixtureB();

		if (fixture.m_idx == fixtureA->m_idx || fixture.m_idx == fixtureB->m_idx)
		{
			// This destroys the contact and removes it from
			// this body's contact list.
			m_world.m_contactManager.Destroy(c);
		}
	}

	b2BlockAllocator* allocator = &m_world.m_blockAllocator;

	if (m_flags & b2_activeBody)
	{
		b2BroadPhase* broadPhase = &m_world.m_contactManager.m_broadPhase;
		fixture.DestroyProxies(broadPhase);
	}

	fixture.Destroy(allocator);
	m_world.RemoveFromBuffer(idx, m_world.m_fixtureBuffer, m_world.m_freeFixtureIdxs);
	fixture.m_idx = b2_invalidIndex;

	// Reset the mass data.
	ResetMassData();
}

void Body::SetMassData(const b2MassData& massData)
{
	if (m_type != b2_dynamicBody)
		return;

	m_invMass = 0.0f;
	m_I = 0.0f;
	m_invI = 0.0f;

	m_mass = massData.mass;
	if (m_mass <= 0.0f)
		m_mass = 1.0f;

	m_invMass = 1.0f / m_mass;

	if (massData.I > 0.0f && (m_flags & b2_fixedRotationBody) == 0)
	{
		m_I = massData.I - m_mass * b2Dot(massData.center, massData.center);
		b2Assert(m_I > 0.0f);
		m_invI = 1.0f / m_I;
	}

	// Move center of mass.
	b2Vec2 oldCenter = m_sweep.c;
	m_sweep.localCenter = massData.center;
	m_sweep.c0 = m_sweep.c = b2Mul(m_xf, m_sweep.localCenter);

	// Update center of mass velocity.
	m_linearVelocity += b2Cross(m_angularVelocity, m_sweep.c - oldCenter);
}

bool Body::ShouldCollide(const Body& other) const
{
	// At least one body should be dynamic.
	if (m_type != b2_dynamicBody && other.m_type != b2_dynamicBody)
		return false;

	// Does a joint prevent collision?
	for (b2JointEdge* jn = m_jointList; jn; jn = jn->next)
	{
		if (jn->other->m_idx == other.m_idx)
		{
			if (jn->joint->m_collideConnected == false)
				return false;
		}
	}

	return true;
}

void b2Body::SetTransform(const b2Vec2& position, float32 angle)
{
	b2Assert(m_world.IsLocked() == false);
	if (m_world.IsLocked() == true)
	{
		return;
	}

	m_xf.q.Set(angle);
	m_xf.p = position;
	m_xf0 = m_xf;

	m_sweep.c = b2Mul(m_xf, m_sweep.localCenter);
	m_sweep.a = angle;

	m_sweep.c0 = m_sweep.c;
	m_sweep.a0 = angle;

	b2BroadPhase* broadPhase = &m_world.m_contactManager.m_broadPhase;
	for each (int32 fIdx in m_fixtureIdxBuffer)
	{
		b2Fixture& f = m_world.m_fixtureBuffer[fIdx];
		f.Synchronize(broadPhase, m_xf, m_xf);
	}
}

void Body::SetActive(bool flag)
{
	b2Assert(m_world.IsLocked() == false);

	if (flag == IsActive()) return;

	if (flag)
	{
		m_flags |= b2_activeBody;

		// Create all proxies.
		b2BroadPhase* broadPhase = &m_world.m_contactManager.m_broadPhase;
		for each (int32 fIdx in m_fixtureIdxBuffer)
		{
			b2Fixture& f = m_world.m_fixtureBuffer[fIdx];
			f.CreateProxies(broadPhase, m_xf);
		}

		// Contacts are created the next time step.
	}
	else
	{
		m_flags &= ~b2_activeBody;

		// Destroy all proxies.
		b2BroadPhase* broadPhase = &m_world.m_contactManager.m_broadPhase;
		for each (int32 fIdx in m_fixtureIdxBuffer)
		{
			b2Fixture& f = m_world.m_fixtureBuffer[fIdx];
			f.DestroyProxies(broadPhase);
		}

		// Destroy the attached contacts.
		b2ContactEdge* ce = m_contactList;
		while (ce)
		{
			b2ContactEdge* ce0 = ce;
			ce = ce->next;
			m_world.m_contactManager.Destroy(ce0->contact);
		}
		m_contactList = NULL;
	}
}

void b2Body::SetFixedRotation(bool flag)
{
	bool status = (m_flags & b2_fixedRotationBody) == b2_fixedRotationBody;
	if (status == flag)
		return;
	if (flag)
		m_flags |= b2_fixedRotationBody;
	else
		m_flags &= ~b2_fixedRotationBody;

	m_angularVelocity = 0.0f;
	ResetMassData();
}
