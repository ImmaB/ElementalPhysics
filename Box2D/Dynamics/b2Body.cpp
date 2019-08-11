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



void Body::Set(const Body::Def& def)
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
	if (def.awake) AddFlag(Flag::Awake);
	if (def.active) AddFlag(Flag::Active);

	m_xf = def.transform;
	m_xf0 = m_xf;

	m_sweep.localCenter.SetZero();
	m_sweep.c0 = m_xf.p;
	m_sweep.c = m_xf.p;
	m_sweep.a0 = def.transform.q.GetAngle();
	m_sweep.a = m_sweep.a0;
	m_sweep.alpha0 = 0.0f;

	m_linearVelocity = def.linearVelocity;
	m_angularVelocity = def.angularVelocity;

	m_linearDamping = def.linearDamping;
	m_angularDamping = def.angularDamping;
	m_gravityScale = def.gravityScale;

	m_force.SetZero();
	m_torque = 0.0f;

	m_sleepTime = 0.0f;

	m_type = def.type;
	
	if (m_type == Dynamic)
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
	
	m_matIdx = def.materialIdx;
	m_heat = def.heat;
	m_health = def.health;
}

void Body::SetMassData(const b2MassData& massData)
{
	if (m_type != Dynamic)
		return;

	m_invMass = 0.0f;
	m_I = 0.0f;
	m_invI = 0.0f;

	m_mass = massData.mass;
	if (m_mass <= 0.0f)
		m_mass = 1.0f;

	m_invMass = 1.0f / m_mass;

	if (massData.I > 0.0f && !HasFlag(FixedRotation))
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
