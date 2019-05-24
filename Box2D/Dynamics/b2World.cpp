/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Dynamics/b2Island.h>
#include <Box2D/Dynamics/Joints/b2PulleyJoint.h>
#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <Box2D/Dynamics/Contacts/b2ContactSolver.h>
#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Collision/b2BroadPhase.h>
#include <Box2D/Collision/Shapes/b2CircleShape.h>
#include <Box2D/Collision/Shapes/b2EdgeShape.h>
#include <Box2D/Collision/Shapes/b2ChainShape.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>
#include <Box2D/Collision/b2TimeOfImpact.h>
#include <Box2D/Common/b2Draw.h>
#include <Box2D/Common/b2Timer.h>
#include <new>

b2World::b2World(const b2Vec2& gravity) : m_contactManager(*this)
{
	Init(gravity);
}

b2World::~b2World()
{
	// Some shapes allocate using b2Alloc.
	for (Body& b : m_bodyBuffer)
	{
		if (b.m_idx == b2_invalidIndex)
		{
			for each (int32 fIdx in b.m_fixtureIdxBuffer)
			{
				if (fIdx != b2_invalidIndex)
				{
					Fixture& f = m_fixtureBuffer[fIdx];
					f.m_proxyCount = 0;
					Destroy(f);
				}
			}
		}
	}

	while (m_particleSystemList)
	{
		DestroyParticleSystem(m_particleSystemList);
	}

	// Even though the block allocator frees them for us, for safety,
	// we should ensure that all buffers have been freed.
	b2Assert(m_blockAllocator.GetNumGiantAllocations() == 0);
}

void b2World::SetDestructionListener(b2DestructionListener* listener)
{
	m_destructionListener = listener;
}

void b2World::SetContactFilter(b2ContactFilter* filter)
{
	m_contactManager.m_contactFilter = filter;
}

void b2World::SetContactListener(b2ContactListener* listener)
{
	m_contactManager.m_contactListener = listener;
}

void b2World::SetDebugDraw(b2Draw* debugDraw)
{
	m_debugDraw = debugDraw;
}

int32 b2World::CreateBody(const b2BodyDef& def)
{
	b2Assert(IsLocked() == false);
	if (IsLocked()) return b2_invalidIndex;

	const int32 idx = GetBodyInsertIdx();
	m_bodyBuffer[idx].Set(def);

	return idx;
}

void b2World::DestroyBody(int32 idx)
{
	b2Assert(m_bodyCount > 0);
	b2Assert(IsLocked() == false);
	if (IsLocked()) return;

	// Delete the attached joints.
	Body& b = m_bodyBuffer[idx];
	b2JointEdge* je = b.m_jointList;
	while (je)
	{
		b2JointEdge* je0 = je;
		je = je->next;

		if (m_destructionListener)
		{
			m_destructionListener->SayGoodbye(je0->joint);
		}

		DestroyJoint(je0->joint);

		b.m_jointList = je;
	}
	b.m_jointList = NULL;

	// Delete the attached contacts.
	b2ContactEdge* ce = b.m_contactList;
	while (ce)
	{
		b2ContactEdge* ce0 = ce;
		ce = ce->next;
		m_contactManager.Destroy(*ce0->contact);
	}
	b.m_contactList = NULL;

	// Delete the attached fixtures. This destroys broad-phase proxies.

	for each (int32 fIdx in b.m_fixtureIdxBuffer)
	{
		if (fIdx != b2_invalidIndex)
		{
			Fixture& f = m_fixtureBuffer[fIdx];
			f.m_idx = b2_invalidIndex;
			DestroyProxies(f);
			RemoveFromBuffer(fIdx, m_fixtureBuffer, m_freeFixtureIdxs);
		}
	}
	RemoveFromBuffer(idx, m_bodyBuffer, m_freeBodyIdxs);
	b.m_idx = b2_invalidIndex;
}

b2Joint* b2World::CreateJoint(const b2JointDef* def)
{
	b2Assert(IsLocked() == false);
	if (IsLocked()) return NULL;

	b2Joint* j = b2Joint::Create(def, &m_blockAllocator);

	// Connect to the world list.
	j->m_prev = NULL;
	j->m_next = m_jointList;
	if (m_jointList)
		m_jointList->m_prev = j;
	m_jointList = j;
	++m_jointCount;

	// Connect to the bodies' doubly linked lists.
	j->m_edgeA.joint = j;
	j->m_edgeA.other = j->m_bodyB;
	j->m_edgeA.prev = NULL;
	j->m_edgeA.next = j->m_bodyA->m_jointList;
	if (j->m_bodyA->m_jointList) j->m_bodyA->m_jointList->prev = &j->m_edgeA;
	j->m_bodyA->m_jointList = &j->m_edgeA;

	j->m_edgeB.joint = j;
	j->m_edgeB.other = j->m_bodyA;
	j->m_edgeB.prev = NULL;
	j->m_edgeB.next = j->m_bodyB->m_jointList;
	if (j->m_bodyB->m_jointList) j->m_bodyB->m_jointList->prev = &j->m_edgeB;
	j->m_bodyB->m_jointList = &j->m_edgeB;

	Body* bodyA = def->bodyA;
	Body* bodyB = def->bodyB;

	// If the joint prevents collisions, then flag any contacts for filtering.
	if (def->collideConnected == false)
	{
		b2ContactEdge* edge = bodyB->m_contactList;
		while (edge)
		{
			if (edge->other->m_idx == bodyA->m_idx)
			{
				// Flag the contact for filtering at the next time step (where either
				// body is awake).
				edge->contact->FlagForFiltering();
			}
			edge = edge->next;
		}
	}

	// Note: creating a joint doesn't wake the bodies.

	return j;
}

void b2World::DestroyJoint(b2Joint* j)
{
	b2Assert(IsLocked() == false);
	if (IsLocked())
	{
		return;
	}

	bool collideConnected = j->m_collideConnected;

	// Remove from the doubly linked list.
	if (j->m_prev)
	{
		j->m_prev->m_next = j->m_next;
	}

	if (j->m_next)
	{
		j->m_next->m_prev = j->m_prev;
	}

	if (j == m_jointList)
	{
		m_jointList = j->m_next;
	}

	// Disconnect from island graph.
	Body* bodyA = j->m_bodyA;
	Body* bodyB = j->m_bodyB;

	// Wake up connected bodies.
	bodyA->SetAwake(true);
	bodyB->SetAwake(true);

	// Remove from body 1.
	if (j->m_edgeA.prev)
	{
		j->m_edgeA.prev->next = j->m_edgeA.next;
	}

	if (j->m_edgeA.next)
	{
		j->m_edgeA.next->prev = j->m_edgeA.prev;
	}

	if (&j->m_edgeA == bodyA->m_jointList)
	{
		bodyA->m_jointList = j->m_edgeA.next;
	}

	j->m_edgeA.prev = NULL;
	j->m_edgeA.next = NULL;

	// Remove from body 2
	if (j->m_edgeB.prev)
	{
		j->m_edgeB.prev->next = j->m_edgeB.next;
	}

	if (j->m_edgeB.next)
	{
		j->m_edgeB.next->prev = j->m_edgeB.prev;
	}

	if (&j->m_edgeB == bodyB->m_jointList)
	{
		bodyB->m_jointList = j->m_edgeB.next;
	}

	j->m_edgeB.prev = NULL;
	j->m_edgeB.next = NULL;

	b2Joint::Destroy(j, &m_blockAllocator);

	b2Assert(m_jointCount > 0);
	--m_jointCount;

	// If the joint prevents collisions, then flag any contacts for filtering.
	if (collideConnected == false)
	{
		b2ContactEdge* edge = bodyB->m_contactList;
		while (edge)
		{
			if (edge->other == bodyA)
			{
				// Flag the contact for filtering at the next time step (where either
				// body is awake).
				edge->contact->FlagForFiltering();
			}

			edge = edge->next;
		}
	}
}

b2ParticleSystem* b2World::CreateParticleSystem(const b2ParticleSystemDef& def)
{
	b2Assert(IsLocked() == false);
	if (IsLocked()) return NULL;

	void* mem = m_blockAllocator.Allocate(sizeof(b2ParticleSystem));
	b2ParticleSystem* p = new (mem) b2ParticleSystem(def, *this, m_bodyBuffer, m_fixtureBuffer);

	// Add to world doubly linked list.
	p->m_prev = NULL;
	p->m_next = m_particleSystemList;
	if (m_particleSystemList)
		m_particleSystemList->m_prev = p;
	m_particleSystemList = p;

	return p;
}

void b2World::DestroyParticleSystem(b2ParticleSystem* p)
{
	b2Assert(m_particleSystemList != NULL);
	b2Assert(IsLocked() == false);
	if (IsLocked())
	{
		return;
	}

	// Remove world particleSystem list.
	if (p->m_prev)
	{
		p->m_prev->m_next = p->m_next;
	}

	if (p->m_next)
	{
		p->m_next->m_prev = p->m_prev;
	}

	if (p == m_particleSystemList)
	{
		m_particleSystemList = p->m_next;
	}

	p->~b2ParticleSystem();
	m_blockAllocator.Free(p, sizeof(b2ParticleSystem));
}

//
void b2World::SetAllowSleeping(bool flag)
{
	if (flag == m_allowSleep)
	{
		return;
	}

	m_allowSleep = flag;
	if (m_allowSleep == false)
	{
		for (Body& b : m_bodyBuffer)
			if (b.m_idx != b2_invalidIndex)
				b.SetAwake(true);
	}
}

// Initialize the world with a specified gravity.
void b2World::Init(const b2Vec2& gravity)
{
	m_destructionListener = NULL;
	m_debugDraw = NULL;

	m_jointList = NULL;
	m_particleSystemList = NULL;

	m_jointCount = 0;

	m_warmStarting = true;
	m_continuousPhysics = true;
	m_subStepping = false;

	m_stepComplete = true;

	m_allowSleep = true;
	m_gravity = gravity;

	m_flags = e_clearForces;

	m_inv_dt0 = 0.0f;

	m_contactManager.m_allocator = &m_blockAllocator;

	m_liquidFunVersion = &b2_liquidFunVersion;
	m_liquidFunVersionString = b2_liquidFunVersionString;

	memset(&m_profile, 0, sizeof(b2Profile));
}

// Find islands, integrate and solve constraints, solve position constraints
void b2World::Solve(const b2TimeStep& step)
{
	// update previous transforms
	for (Body& b : m_bodyBuffer)
		if (b.m_idx != b2_invalidIndex)
			b.m_xf0 = b.m_xf;
	

	m_profile.solveInit = 0.0f;
	m_profile.solveVelocity = 0.0f;
	m_profile.solvePosition = 0.0f;

	// Size the island for the worst case.
	b2Island island(m_bodyBuffer.size(),
					m_contactManager.m_contactCount,
					m_jointCount,
					&m_stackAllocator,
					m_contactManager.m_contactListener, *this);

	// Clear all the island flags.
	for (Body& b : m_bodyBuffer)
		if (b.m_idx != b2_invalidIndex)
			b.m_flags &= ~b2_islandBody;

	for (b2Contact* c = m_contactManager.m_contactList; c; c = c->m_next)
		c->m_flags &= ~b2Contact::e_islandFlag;

	for (b2Joint* j = m_jointList; j; j = j->m_next)
		j->m_islandFlag = false;

	// Build and simulate all awake islands.
	int32 stackSize = m_bodyBuffer.size();
	std::vector<Body*> stack(stackSize);
	for (Body& seed : m_bodyBuffer)
	{
		if (seed.m_idx != b2_invalidIndex)
		{
			if (seed.m_flags & b2_islandBody)
				continue;

			if (seed.IsAwake() == false || seed.IsActive() == false)
				continue;

			// The seed can be dynamic or kinematic.
			if (seed.m_type == b2_staticBody)
				continue;

			// Reset island and stack.
			island.Clear();
			int32 stackCount = 0;
			stack[stackCount++] = &seed;
			seed.m_flags |= b2_islandBody;

			// Perform a depth first search (DFS) on the constraint graph.
			while (stackCount > 0)
			{
				// Grab the next body off the stack and add it to the island.
				Body* b = stack[--stackCount];
				b2Assert(b->IsActive() == true);
				island.Add(b);

				// Make sure the body is awake.
				b->SetAwake(true);

				// To keep islands as small as possible, we don't
				// propagate islands across static bodies.
				if (b->m_type == b2_staticBody)
					continue;

				// Search all contacts connected to this body.
				for (b2ContactEdge* ce = b->m_contactList; ce; ce = ce->next)
				{
					b2Contact* contact = ce->contact;

					// Has this contact already been added to an island?
					if (contact->m_flags & b2Contact::e_islandFlag)
						continue;

					// Is this contact solid and touching?
					if (contact->IsEnabled() == false ||
						contact->IsTouching() == false)
						continue;

					// Skip sensors.
					bool sensorA = contact->m_fixtureA->m_isSensor;
					bool sensorB = contact->m_fixtureB->m_isSensor;
					if (sensorA || sensorB)
						continue;

					island.Add(contact);
					contact->m_flags |= b2Contact::e_islandFlag;

					Body* other = ce->other;

					// Was the other body already added to this island?
					if (other->m_flags & b2_islandBody)
						continue;

					b2Assert(stackCount < stackSize);
					stack[stackCount++] = other;
					other->m_flags |= b2_islandBody;
				}

				// Search all joints connect to this body.
				for (b2JointEdge* je = b->m_jointList; je; je = je->next)
				{
					if (je->joint->m_islandFlag == true)
						continue;

					Body* other = je->other;

					// Don't simulate joints connected to inactive bodies.
					if (other->IsActive() == false)
						continue;

					island.Add(je->joint);
					je->joint->m_islandFlag = true;

					if (other->m_flags & b2_islandBody)
						continue;

					b2Assert(stackCount < stackSize);
					stack[stackCount++] = other;
					other->m_flags |= b2_islandBody;
				}
			}

			b2Profile profile;
			island.Solve(profile, step, m_gravity, m_dampingStrength, m_allowSleep);
			m_profile.solveInit += profile.solveInit;
			m_profile.solveVelocity += profile.solveVelocity;
			m_profile.solvePosition += profile.solvePosition;

			// Post solve cleanup.
			for (int32 i = 0; i < island.m_bodyCount; ++i)
			{
				// Allow static bodies to participate in other islands.
				Body* b = island.m_bodies[i];
				if (b->m_type == b2_staticBody)
					b->m_flags &= ~b2_islandBody;
			}
		}
	}
	stack.clear();

	{
		b2Timer timer;
		// Synchronize fixtures, check for out of range bodies.

		for (Body& b : m_bodyBuffer)
		{
			if (b.m_idx != b2_invalidIndex)
			{
				// If a body was not in an island then it did not move.
				if ((b.m_flags & b2_islandBody) == 0)
					continue;

				if (b.m_type == b2_staticBody)
					continue;

				// Update fixtures (for broad-phase).
				SynchronizeFixtures(b);
			}
		}

		// Look for new contacts.
		m_contactManager.FindNewContacts();
		m_profile.broadphase = timer.GetMilliseconds();
	}
}

// Find TOI contacts and solve them.
void b2World::SolveTOI(const b2TimeStep& step)
{
	b2Island island(2 * b2_maxTOIContacts, b2_maxTOIContacts, 0, &m_stackAllocator, m_contactManager.m_contactListener, *this);

	if (m_stepComplete)
	{
		for (Body& b : m_bodyBuffer)
		{
			if (b.m_idx != b2_invalidIndex)
			{
				b.m_flags &= ~b2_islandBody;
				b.m_sweep.alpha0 = 0.0f;
			}
		}

		for (b2Contact* c = m_contactManager.m_contactList; c; c = c->m_next)
		{
			// Invalidate TOI
			c->m_flags &= ~(b2Contact::e_toiFlag | b2Contact::e_islandFlag);
			c->m_toiCount = 0;
			c->m_toi = 1.0f;
		}
	}

	// Find TOI events and solve them.
	for (;;)
	{
		// Find the first TOI.
		b2Contact* minContact = NULL;
		float32 minAlpha = 1.0f;

		for (b2Contact* c = m_contactManager.m_contactList; c; c = c->m_next)
		{
			// Is this contact disabled?
			if (c->IsEnabled() == false)
			{
				continue;
			}

			// Prevent excessive sub-stepping.
			if (c->m_toiCount > b2_maxSubSteps)
			{
				continue;
			}

			float32 alpha = 1.0f;
			if (c->m_flags & b2Contact::e_toiFlag)
			{
				// This contact has a valid cached TOI.
				alpha = c->m_toi;
			}
			else
			{
				Fixture* fA = c->GetFixtureA();
				Fixture* fB = c->GetFixtureB();

				// Is there a sensor?
				if (fA->m_isSensor || fB->m_isSensor)
					continue;

				Body& bA = m_bodyBuffer[fA->m_bodyIdx];
				Body& bB = m_bodyBuffer[fB->m_bodyIdx];

				b2BodyType typeA = bA.m_type;
				b2BodyType typeB = bB.m_type;
				b2Assert(typeA == b2_dynamicBody || typeB == b2_dynamicBody);

				bool activeA = bA.IsAwake() && typeA != b2_staticBody;
				bool activeB = bB.IsAwake() && typeB != b2_staticBody;

				// Is at least one body active (awake and dynamic or kinematic)?
				if (activeA == false && activeB == false)
				{
					continue;
				}

				bool collideA = bA.IsBullet() || typeA != b2_dynamicBody;
				bool collideB = bB.IsBullet() || typeB != b2_dynamicBody;

				// Are these two non-bullet dynamic bodies?
				if (collideA == false && collideB == false)
				{
					continue;
				}

				// Compute the TOI for this contact.
				// Put the sweeps onto the same time interval.
				float32 alpha0 = bA.m_sweep.alpha0;

				if (bA.m_sweep.alpha0 < bB.m_sweep.alpha0)
				{
					alpha0 = bB.m_sweep.alpha0;
					bA.m_sweep.Advance(alpha0);
				}
				else if (bB.m_sweep.alpha0 < bA.m_sweep.alpha0)
				{
					alpha0 = bA.m_sweep.alpha0;
					bB.m_sweep.Advance(alpha0);
				}

				b2Assert(alpha0 < 1.0f);

				int32 indexA = c->GetChildIndexA();
				int32 indexB = c->GetChildIndexB();

				// Compute the time of impact in interval [0, minTOI]
				b2TOIInput input;
				Shape& shapeA = m_shapeBuffer[fA->m_shapeIdx];
				switch (shapeA.m_type)
				{
				case Shape::e_chain:
					input.proxyA.Set(m_chainShapeBuffer[shapeA.m_idx], indexA); break;
				case Shape::e_circle:
					input.proxyA.Set(m_circleShapeBuffer[shapeA.m_idx], indexA); break;
				case Shape::e_edge:
					input.proxyA.Set(m_edgeShapeBuffer[shapeA.m_idx], indexA); break;
				case Shape::e_polygon:
					input.proxyA.Set(m_polygonShapeBuffer[shapeA.m_idx], indexA); break;
				}
				Shape& shapeB = m_shapeBuffer[fB->m_shapeIdx];
				switch (shapeB.m_type)
				{
				case Shape::e_chain:
					input.proxyB.Set(m_chainShapeBuffer[shapeB.m_idx], indexB); break;
				case Shape::e_circle:
					input.proxyB.Set(m_circleShapeBuffer[shapeB.m_idx], indexB); break;
				case Shape::e_edge:
					input.proxyB.Set(m_edgeShapeBuffer[shapeB.m_idx], indexB); break;
				case Shape::e_polygon:
					input.proxyB.Set(m_polygonShapeBuffer[shapeB.m_idx], indexB); break;
				}
				input.sweepA = bA.m_sweep;
				input.sweepB = bB.m_sweep;
				input.tMax = 1.0f;

				b2TOIOutput output;
				b2TimeOfImpact(&output, &input);

				// Beta is the fraction of the remaining portion of the .
				float32 beta = output.t;
				if (output.state == b2TOIOutput::e_touching)
				{
					alpha = b2Min(alpha0 + (1.0f - alpha0) * beta, 1.0f);
				}
				else
				{
					alpha = 1.0f;
				}

				c->m_toi = alpha;
				c->m_flags |= b2Contact::e_toiFlag;
			}

			if (alpha < minAlpha)
			{
				// This is the minimum TOI found so far.
				minContact = c;
				minAlpha = alpha;
			}
		}

		if (minContact == NULL || 1.0f - 10.0f * b2_epsilon < minAlpha)
		{
			// No more TOI events. Done!
			m_stepComplete = true;
			break;
		}

		// Advance the bodies to the TOI.
		Fixture* fA = minContact->GetFixtureA();
		Fixture* fB = minContact->GetFixtureB();
		Body& bA = m_bodyBuffer[fA->m_bodyIdx];
		Body& bB = m_bodyBuffer[fB->m_bodyIdx];

		b2Sweep backup1 = bA.m_sweep;
		b2Sweep backup2 = bB.m_sweep;

		bA.Advance(minAlpha);
		bB.Advance(minAlpha);

		// The TOI contact likely has some new contact points.
		Update(*minContact);
		minContact->m_flags &= ~b2Contact::e_toiFlag;
		++minContact->m_toiCount;

		// Is the contact solid?
		if (minContact->IsEnabled() == false || minContact->IsTouching() == false)
		{
			// Restore the sweeps.
			minContact->SetEnabled(false);
			bA.m_sweep = backup1;
			bB.m_sweep = backup2;
			bA.SynchronizeTransform();
			bB.SynchronizeTransform();
			continue;
		}

		bA.SetAwake(true);
		bB.SetAwake(true);

		// Build the island
		island.Clear();
		island.Add(&bA);
		island.Add(&bB);
		island.Add(minContact);

		bA.m_flags |= b2_islandBody;
		bB.m_flags |= b2_islandBody;
		minContact->m_flags |= b2Contact::e_islandFlag;

		// Get contacts on bodyA and bodyB.
		for (Body* body : { &bA, &bB })
		{
			if (body->m_type == b2_dynamicBody)
			{
				for (b2ContactEdge* ce = body->m_contactList; ce; ce = ce->next)
				{
					if (island.m_bodyCount == island.m_bodyCapacity)
					{
						break;
					}

					if (island.m_contactCount == island.m_contactCapacity)
					{
						break;
					}

					b2Contact* contact = ce->contact;

					// Has this contact already been added to the island?
					if (contact->m_flags & b2Contact::e_islandFlag)
					{
						continue;
					}

					// Only add static, kinematic, or bullet bodies.
					Body* other = ce->other;
					if (other->m_type == b2_dynamicBody &&
						body->IsBullet() == false && other->IsBullet() == false)
					{
						continue;
					}

					// Skip sensors.
					bool sensorA = contact->m_fixtureA->m_isSensor;
					bool sensorB = contact->m_fixtureB->m_isSensor;
					if (sensorA || sensorB)
					{
						continue;
					}

					// Tentatively advance the body to the TOI.
					b2Sweep backup = other->m_sweep;
					if ((other->m_flags & b2_islandBody) == 0)
					{
						other->Advance(minAlpha);
					}

					// Update the contact points
					Update(*contact);

					// Was the contact disabled by the user?
					if (contact->IsEnabled() == false)
					{
						other->m_sweep = backup;
						other->SynchronizeTransform();
						continue;
					}

					// Are there contact points?
					if (contact->IsTouching() == false)
					{
						other->m_sweep = backup;
						other->SynchronizeTransform();
						continue;
					}

					// Add the contact to the island
					contact->m_flags |= b2Contact::e_islandFlag;
					island.Add(contact);

					// Has the other body already been added to the island?
					if (other->m_flags & b2_islandBody)
					{
						continue;
					}

					// Add the other body to the island.
					other->m_flags |= b2_islandBody;

					if (other->m_type != b2_staticBody)
					{
						other->SetAwake(true);
					}

					island.Add(other);
				}
			}
		}

		b2TimeStep subStep;
		subStep.dt = (1.0f - minAlpha) * step.dt;
		subStep.inv_dt = 1.0f / subStep.dt;
		subStep.dtRatio = 1.0f;
		subStep.positionIterations = 20;
		subStep.velocityIterations = step.velocityIterations;
		subStep.particleIterations = step.particleIterations;
		subStep.warmStarting = false;
		island.SolveTOI(subStep, bA.m_islandIndex, bB.m_islandIndex);

		// Reset island flags and synchronize broad-phase proxies.
		for (int32 i = 0; i < island.m_bodyCount; ++i)
		{
			Body* body = island.m_bodies[i];
			body->m_flags &= ~b2_islandBody;

			if (body->m_type != b2_dynamicBody)
			{
				continue;
			}

			SynchronizeFixtures(*body);

			// Invalidate all contact TOIs on this displaced body.
			for (b2ContactEdge* ce = body->m_contactList; ce; ce = ce->next)
			{
				ce->contact->m_flags &= ~(b2Contact::e_toiFlag | b2Contact::e_islandFlag);
			}
		}

		// Commit fixture proxy movements to the broad-phase so that new contacts are created.
		// Also, some contacts can be destroyed.
		m_contactManager.FindNewContacts();

		if (m_subStepping)
		{
			m_stepComplete = false;
			break;
		}
	}
}

void b2World::SetStepParams(float32 dt,
	int32 velocityIterations,
	int32 positionIterations,
	int32 particleIterations)
{
	m_step.dt = dt; 
	if (dt > 0.0f)
	{
		m_step.inv_dt = 1.0f / dt;
	}
	else
	{
		m_step.inv_dt = 0.0f;
	}
	m_step.dtRatio = m_inv_dt0 * dt;
	m_step.warmStarting = m_warmStarting;
	m_step.velocityIterations = velocityIterations;
	m_step.positionIterations = positionIterations;
	m_step.particleIterations = particleIterations;

	// Set Particle System m_step
	for (b2ParticleSystem* p = m_particleSystemList; p; p = p->GetNext())
	{
		p->SetStep(m_step);
	}
}

void b2World::StepPreParticle()
{
	// If new fixtures were added, we need to find the new contacts.
	if (m_flags & e_newFixture)
	{
		m_contactManager.FindNewContacts();
		m_flags &= ~e_newFixture;
	}

	m_flags |= e_locked;

	// Update contacts. This is where some contacts are destroyed.
	{
		b2Timer timer;
		m_contactManager.Collide();
		m_profile.collide = timer.GetMilliseconds();
	}
}

void b2World::StepPostParticle()
{
	if (m_stepComplete && m_step.dt > 0.0f)
		Solve(m_step);

	// Handle TOI events.
	if (m_continuousPhysics && m_step.dt > 0.0f)
	{
		b2Timer timer;
		SolveTOI(m_step);
		m_profile.solveTOI = timer.GetMilliseconds();
	}

	if (m_step.dt > 0.0f)
	{
		m_inv_dt0 = m_step.inv_dt;
	}

	if (m_flags & e_clearForces)
	{
		ClearForces();
	}

	m_flags &= ~e_locked;
}

void b2World::ClearForces()
{

	for (Body& body : m_bodyBuffer)
	{
		if (body.m_idx != b2_invalidIndex)
		{
			body.m_force.SetZero();
			body.m_torque = 0.0f;
		}
	}
}

struct b2WorldQueryWrapper
{
	bool QueryCallback(int32 proxyId)
	{
		b2FixtureProxy* proxy = (b2FixtureProxy*)broadPhase->GetUserData(proxyId);
		return callback->ReportFixture(proxy->fixtureIdx);
	}

	const b2BroadPhase* broadPhase;
	b2QueryCallback* callback;
};

void b2World::QueryAABB(b2QueryCallback* callback, const b2AABB& aabb) const
{
	b2WorldQueryWrapper wrapper;
	wrapper.broadPhase = &m_contactManager.m_broadPhase;
	wrapper.callback = callback;
	m_contactManager.m_broadPhase.Query(&wrapper, aabb);
	for (b2ParticleSystem* p = m_particleSystemList; p; p = p->GetNext())
	{
		if (callback->ShouldQueryParticleSystem(p))
		{
			p->QueryAABB(callback, aabb);
		}
	}
}
void b2World::AmpQueryAABB(b2QueryCallback* callback, const b2AABB& aabb) const
{
	b2WorldQueryWrapper wrapper;
	wrapper.broadPhase = &m_contactManager.m_broadPhase;
	wrapper.callback = callback;
	m_contactManager.m_broadPhase.Query(&wrapper, aabb);
}

void b2World::QueryShapeAABB(b2QueryCallback* callback, const b2Shape& shape,
                             const b2Transform& xf) const
{
	b2AABB aabb;
	shape.ComputeAABB(aabb, xf, 0);
	QueryAABB(callback, aabb);
}

struct b2WorldRayCastWrapper
{
	b2WorldRayCastWrapper(const b2World& world) : m_world(world) {};

	float32 RayCastCallback(const b2RayCastInput& input, int32 proxyId)
	{
		b2FixtureProxy* proxy = (b2FixtureProxy*)broadPhase->GetUserData(proxyId);
		int32 fixtureIdx = proxy->fixtureIdx;
		b2RayCastOutput output;
		Fixture& f = *proxy->fixture;
		bool hit = m_world.RayCast(f, output, input, proxy->childIndex);

		if (hit)
		{
			float32 fraction = output.fraction;
			b2Vec2 point = (1.0f - fraction) * input.p1 + fraction * input.p2;
			return callback->ReportFixture(f, point, output.normal, fraction);
		}

		return input.maxFraction;
	}

	const b2World& m_world;
	const b2BroadPhase* broadPhase;
	b2RayCastCallback* callback;
};

void b2World::RayCast(b2RayCastCallback& callback, const b2Vec2& point1, const b2Vec2& point2) const
{
	b2WorldRayCastWrapper wrapper(*this);
	wrapper.broadPhase = &m_contactManager.m_broadPhase;
	wrapper.callback = &callback;
	b2RayCastInput input;
	input.maxFraction = 1.0f;
	input.p1 = point1;
	input.p2 = point2;
	m_contactManager.m_broadPhase.RayCast(wrapper, input);
	for (b2ParticleSystem* p = m_particleSystemList; p; p = p->GetNext())
		if (callback.ShouldQueryParticleSystem(p))
			p->RayCast(callback, point1, point2);
}

void b2World::DrawShape(Fixture& fixture, const b2Transform& xf, const b2Color& color)
{
	Shape& s = m_shapeBuffer[fixture.m_shapeIdx];
	switch (s.m_type)
	{
	case Shape::Type::e_circle:
		{
			b2CircleShape& circle = m_circleShapeBuffer[s.m_idx];
			b2Vec2 center = b2Mul(xf, circle.m_p);
			float32 radius = circle.m_radius;
			b2Vec2 axis = b2Mul(xf.q, b2Vec2(1.0f, 0.0f));

			m_debugDraw->DrawSolidCircle(center, radius, axis, color);
		}
		break;

	case Shape::Type::e_edge:
		{
			b2EdgeShape& edge = m_edgeShapeBuffer[s.m_idx];
			b2Vec2 v1 = b2Mul(xf, edge.m_vertex1);
			b2Vec2 v2 = b2Mul(xf, edge.m_vertex2);
			m_debugDraw->DrawSegment(v1, v2, color);
		}
		break;

	case Shape::Type::e_chain:
		{
			b2ChainShape& chain = m_chainShapeBuffer[s.m_idx];
			int32 count = chain.m_count;
			const b2Vec2* vertices = chain.m_vertices;

			b2Vec2 v1 = b2Mul(xf, vertices[0]);
			for (int32 i = 1; i < count; ++i)
			{
				b2Vec2 v2 = b2Mul(xf, vertices[i]);
				m_debugDraw->DrawSegment(v1, v2, color);
				m_debugDraw->DrawCircle(v1, 0.05f, color);
				v1 = v2;
			}
		}
		break;

	case Shape::Type::e_polygon:
		{
			b2PolygonShape& poly = m_polygonShapeBuffer[s.m_idx];
			int32 vertexCount = poly.m_count;
			b2Assert(vertexCount <= b2_maxPolygonVertices);
			b2Vec2 vertices[b2_maxPolygonVertices];

			for (int32 i = 0; i < vertexCount; ++i)
				vertices[i] = b2Mul(xf, poly.m_vertices[i]);

			m_debugDraw->DrawSolidPolygon(vertices, vertexCount, color);
		}
		break;

	default:
		break;
	}
}

void b2World::DrawJoint(b2Joint* joint)
{
	Body* bodyA = joint->GetBodyA();
	Body* bodyB = joint->GetBodyB();
	b2Vec2 x1 = bodyA->m_xf.p;
	b2Vec2 x2 = bodyB->m_xf.p;
	b2Vec2 p1 = joint->GetAnchorA();
	b2Vec2 p2 = joint->GetAnchorB();

	b2Color color(0.5f, 0.8f, 0.8f);

	switch (joint->GetType())
	{
	case e_distanceJoint:
		m_debugDraw->DrawSegment(p1, p2, color);
		break;

	case e_pulleyJoint:
		{
			b2PulleyJoint* pulley = (b2PulleyJoint*)joint;
			b2Vec2 s1 = pulley->GetGroundAnchorA();
			b2Vec2 s2 = pulley->GetGroundAnchorB();
			m_debugDraw->DrawSegment(s1, p1, color);
			m_debugDraw->DrawSegment(s2, p2, color);
			m_debugDraw->DrawSegment(s1, s2, color);
		}
		break;

	case e_mouseJoint:
		// don't draw this
		break;

	default:
		m_debugDraw->DrawSegment(x1, p1, color);
		m_debugDraw->DrawSegment(p1, p2, color);
		m_debugDraw->DrawSegment(x2, p2, color);
	}
}

void b2World::DrawParticleSystem(const b2ParticleSystem& system)
{
	int32 particleCount = system.GetParticleCount();
	if (particleCount)
	{
		float32 radius = system.GetRadius();
		const b2Vec3* posBuf = system.GetPositionBuffer();
		if (system.m_colorBuffer.data())
		{
			const int32* colorBuffer = system.GetColorBuffer();
			m_debugDraw->DrawParticles(posBuf, radius, colorBuffer, particleCount);
		}
		else
		{
			m_debugDraw->DrawParticles(posBuf, radius, NULL, particleCount);
		}
	}
}

void b2World::DrawDebugData()
{
	if (m_debugDraw == NULL)
	{
		return;
	}

	uint32 flags = m_debugDraw->GetFlags();

	if (flags & b2Draw::e_shapeBit)
	{
		
		for (Body& b : m_bodyBuffer)
		{
			if (b.m_idx != b2_invalidIndex)
			{
				const b2Transform& xf = b.m_xf;
				
				for (int32 fIdx : b.m_fixtureIdxBuffer)
				{
					if (fIdx != b2_invalidIndex)
					{
						Fixture& f = m_fixtureBuffer[fIdx];
						if (b.IsActive() == false)
							DrawShape(f, xf, b2Color(0.5f, 0.5f, 0.3f));
						else if (b.m_type == b2_staticBody)
							DrawShape(f, xf, b2Color(0.5f, 0.9f, 0.5f));
						else if (b.m_type == b2_kinematicBody)
							DrawShape(f, xf, b2Color(0.5f, 0.5f, 0.9f));
						else if (b.IsAwake() == false)
							DrawShape(f, xf, b2Color(0.6f, 0.6f, 0.6f));
						else
							DrawShape(f, xf, b2Color(0.9f, 0.7f, 0.7f));
					}
				}
			}
		}
	}

	if (flags & b2Draw::e_particleBit)
	{
		for (b2ParticleSystem* p = m_particleSystemList; p; p = p->GetNext())
		{
			DrawParticleSystem(*p);
		}
	}

	if (flags & b2Draw::e_jointBit)
	{
		for (b2Joint* j = m_jointList; j; j = j->GetNext())
		{

			DrawJoint(j);
		}
	}

	if (flags & b2Draw::e_pairBit)
	{
		b2Color color(0.3f, 0.9f, 0.9f);
		for (b2Contact* c = m_contactManager.m_contactList; c; c = c->GetNext())
		{
			//b2Fixture* fixtureA = c->GetFixtureA();
			//b2Fixture* fixtureB = c->GetFixtureB();

			//b2Vec2 cA = fixtureA->GetAABB().GetCenter();
			//b2Vec2 cB = fixtureB->GetAABB().GetCenter();

			//m_debugDraw->DrawSegment(cA, cB, color);
		}
	}

	if (flags & b2Draw::e_aabbBit)
	{
		b2Color color(0.9f, 0.3f, 0.9f);
		b2BroadPhase* bp = &m_contactManager.m_broadPhase;

		for (Body& b : m_bodyBuffer)
		{
			if (b.m_idx != b2_invalidIndex)
			{
				if (b.IsActive() == false)
				{
					continue;
				}

				for (int32 fIdx : b.m_fixtureIdxBuffer)
				{
					if (fIdx != b2_invalidIndex)
					{
						Fixture& f = m_fixtureBuffer[fIdx];
						auto& proxies = m_fixtureProxiesBuffer[f.m_idx];
						for (int32 i = 0; i < f.m_proxyCount; ++i)
						{
							b2FixtureProxy& proxy = proxies[i];
							b2AABB aabb = bp->GetFatAABB(proxy.proxyId);
							b2Vec2 vs[4];
							vs[0].Set(aabb.lowerBound.x, aabb.lowerBound.y);
							vs[1].Set(aabb.upperBound.x, aabb.lowerBound.y);
							vs[2].Set(aabb.upperBound.x, aabb.upperBound.y);
							vs[3].Set(aabb.lowerBound.x, aabb.upperBound.y);

							m_debugDraw->DrawPolygon(vs, 4, color);
						}
					}
				}
			}
		}
	}

	if (flags & b2Draw::e_centerOfMassBit)
	{
		for (Body& b : m_bodyBuffer)
		{
			if (b.m_idx != b2_invalidIndex)
			{
				b.m_xf.p = b.GetWorldCenter();
				m_debugDraw->DrawTransform(b.m_xf);
			}
		}
	}
}

static float32 GetSmallestRadius(const b2World* world)
{
	float32 smallestRadius = b2_maxFloat;
	for (const b2ParticleSystem* system = world->GetParticleSystemList();
		 system != NULL;
		 system = system->GetNext())
	{
		smallestRadius = b2Min(smallestRadius, system->GetRadius());
	}
	return smallestRadius;
}

int b2World::CalculateReasonableParticleIterations(float32 timeStep) const
{
	if (m_particleSystemList == NULL)
		return 1;

	// Use the smallest radius, since that represents the worst-case.
	return b2CalculateParticleIterations(m_gravity.Length(),
										 GetSmallestRadius(this),
										 timeStep);
}

int32 b2World::GetProxyCount() const
{
	return m_contactManager.m_broadPhase.GetProxyCount();
}

int32 b2World::GetTreeHeight() const
{
	return m_contactManager.m_broadPhase.GetTreeHeight();
}

int32 b2World::GetTreeBalance() const
{
	return m_contactManager.m_broadPhase.GetTreeBalance();
}

float32 b2World::GetTreeQuality() const
{
	return m_contactManager.m_broadPhase.GetTreeQuality();
}

void b2World::ShiftOrigin(const b2Vec2& newOrigin)
{
	b2Assert((m_flags & e_locked) == 0);
	if ((m_flags & e_locked) == e_locked)
	{
		return;
	}

	for (Body& b : m_bodyBuffer)
	{
		if (b.m_idx != b2_invalidIndex)
		{
			b.m_xf.p -= newOrigin;
			b.m_sweep.c0 -= newOrigin;
			b.m_sweep.c -= newOrigin;
		}
	}

	for (b2Joint* j = m_jointList; j; j = j->m_next)
	{
		j->ShiftOrigin(newOrigin);
	}

	m_contactManager.m_broadPhase.ShiftOrigin(newOrigin);
}

int32 b2World::AddBodyMaterial(b2BodyMaterialDef def)
{
	const int32 idx = m_bodyMaterials.size();
	m_bodyMaterials.push_back(b2BodyMaterial(def));

	m_allMaterialFlags |= def.matFlags;

	return idx;
}

template <typename T>
int32 b2World::GetBufferInsertIdx(const vector<T> buf, const set<int32> freeIdxs) const
{
	int32 idx;
	if (freeIdxs.empty())
	{
		idx = buf.size();
		buf.resize(idx + 1);
	}
	else
	{
		idx = *freeIdxs.rbegin();
		freeIdxs.erase(--freeIdxs.end());
	}
	return idx;
}

int32 b2World::GetBodyInsertIdx()
{
	int32 idx = GetBufferInsertIdx(m_bodyBuffer, m_freeBodyIdxs);
	m_bodyBuffer[idx].m_idx = idx;
	return idx;
}
int32 b2World::GetFixtureInsertIdx()
{
	int32 idx = GetBufferInsertIdx(m_fixtureBuffer, m_freeFixtureIdxs);
	m_fixtureBuffer[idx].m_idx = idx;
	return idx;
}

template<typename T>
void b2World::RemoveFromBuffer(const int32 idx, vector<T>& buffer, set<int32>& freeIdxs) const
{
	if (idx == buffer.size())
	{
		buffer.pop_back();
		while (!freeIdxs.empty() && *freeIdxs.rbegin() == buffer.size())
		{
			buffer.pop_back();
			freeIdxs.erase(--freeIdxs.end());
		}
	}
	else
		freeIdxs.insert(idx);
	buffer[idx].m_idx = b2_invalidIndex;
}

b2Shape& b2World::GetSubShape(const Shape& s)
{
	switch (s.m_type)
	{
	case Shape::e_chain:
		return m_chainShapeBuffer[s.m_idx];
	case Shape::e_circle:
		return m_circleShapeBuffer[s.m_idx];
	case Shape::e_edge:
		return m_edgeShapeBuffer[s.m_idx];
	case Shape::e_polygon:
		return m_polygonShapeBuffer[s.m_idx];
	}
}


int32 b2World::CreateFixture(Body& b, b2FixtureDef& def)
{
	b2Assert(IsLocked() == false);
	if (IsLocked() == true) return b2_invalidIndex;

	const int32 fIdx = GetFixtureInsertIdx();
	Fixture& f = m_fixtureBuffer[fIdx];
	f.Set(def, b.m_idx);
	// Reserve proxy space
	b2Shape& shape = GetSubShape(m_shapeBuffer[fIdx]);
	int32 childCount = shape.GetChildCount();
	auto& proxies = m_fixtureProxiesBuffer[fIdx];
	proxies.clear();
	proxies.resize(childCount);
	f.m_proxyCount = 0;

	if (m_flags & b2_activeBody)
		CreateProxies(f, m_contactManager.m_broadPhase, b.m_xf);

	// Adjust mass properties if needed.
	if (f.m_density > 0.0f)
		ResetMassData(b);

	// Let the world know we have a new fixture. This will cause new contacts
	// to be created at the beginning of the next time step.
	m_flags |= b2World::e_newFixture;

	return fIdx;
}

int32 b2World::CreateFixture(Body& b, const int32 shapeIdx, float32 density)
{
	b2FixtureDef def;
	def.shapeIdx = shapeIdx;
	def.density = density;

	return CreateFixture(b, def);
}

void b2World::DestroyFixture(Body& b, int32 idx)
{
	b2Assert(IsLocked() == false);
	if (IsLocked() == true) return;

	int& fIdx = b.m_fixtureIdxBuffer[idx];
	Fixture& fixture = m_fixtureBuffer[fIdx];
	b2Assert(fixture.m_idx == fIdx);
	

	RemoveFromBuffer(fIdx, m_fixtureBuffer, m_freeFixtureIdxs);
	fIdx = b2_invalidIndex;

	// Remove the fixture from this body's singly linked list.
	b2Assert(b.m_fixtureCount > 0);

	// Destroy any contacts associated with the fixture.
	b2ContactEdge* edge = b.m_contactList;
	while (edge)
	{
		b2Contact* c = edge->contact;
		edge = edge->next;

		Fixture* fixtureA = c->GetFixtureA();
		Fixture* fixtureB = c->GetFixtureB();

		if (fixture.m_idx == fixtureA->m_idx || fixture.m_idx == fixtureB->m_idx)
			// This destroys the contact and removes it from
			// this body's contact list.
			m_contactManager.Destroy(*c);
	}

	b2BlockAllocator* allocator = &m_blockAllocator;

	if (m_flags & b2_activeBody)
		DestroyProxies(fixture);

	Destroy(fixture);
	fixture.m_idx = b2_invalidIndex;

	// Reset the mass data.
	ResetMassData(b);
}

void b2World::SetTransform(Body& b, const b2Vec2& position, float32 angle)
{
	b2Assert(IsLocked() == false);
	if (IsLocked() == true) return;

	b.m_xf.q.Set(angle);
	b.m_xf.p = position;
	b.m_xf0 = b.m_xf;

	b.m_sweep.c = b2Mul(b.m_xf, b.m_sweep.localCenter);
	b.m_sweep.a = angle;

	b.m_sweep.c0 = b.m_sweep.c;
	b.m_sweep.a0 = angle;

	b2BroadPhase& broadPhase = m_contactManager.m_broadPhase;
	for each (int32 fIdx in b.m_fixtureIdxBuffer)
	{
		Fixture& f = m_fixtureBuffer[fIdx];
		Synchronize(f, broadPhase, b.m_xf, b.m_xf);
	}
}

void b2World::SetActive(Body& b, bool flag)
{
	b2Assert(m_world.IsLocked() == false);

	if (flag == b.IsActive()) return;

	if (flag)
	{
		b.m_flags |= b2_activeBody;

		// Create all proxies.
		b2BroadPhase& broadPhase = m_contactManager.m_broadPhase;
		for each (int32 fIdx in b.m_fixtureIdxBuffer)
			CreateProxies(m_fixtureBuffer[fIdx], broadPhase, b.m_xf);

		// Contacts are created the next time step.
	}
	else
	{
		m_flags &= ~b2_activeBody;

		// Destroy all proxies.
		for each (int32 fIdx in b.m_fixtureIdxBuffer)
			DestroyProxies(m_fixtureBuffer[fIdx]);

		// Destroy the attached contacts.
		b2ContactEdge* ce = b.m_contactList;
		while (ce)
		{
			b2ContactEdge* ce0 = ce;
			ce = ce->next;
			m_contactManager.Destroy(*ce0->contact);
		}
		b.m_contactList = NULL;
	}
}

void b2World::SetFixedRotation(Body& b, bool flag)
{
	bool status = (b.m_flags & b2_fixedRotationBody) == b2_fixedRotationBody;
	if (status == flag)
		return;
	if (flag)
		b.m_flags |= b2_fixedRotationBody;
	else
		b.m_flags &= ~b2_fixedRotationBody;

	b.m_angularVelocity = 0.0f;
	ResetMassData(b);
}


void b2World::SynchronizeFixtures(const Body& b)
{
	b2Transform xf1;
	xf1.q.Set(b.m_sweep.a0);
	xf1.p = b.m_sweep.c0 - b2Mul(xf1.q, b.m_sweep.localCenter);

	b2BroadPhase& broadPhase = m_contactManager.m_broadPhase;
	for each (int32 fIdx in b.m_fixtureIdxBuffer)
	{
		Fixture& f = m_fixtureBuffer[fIdx];
		Synchronize(f, broadPhase, xf1, b.m_xf);
	}
}


void b2World::ResetMassData(Body& b)
{
	// Compute mass data from shapes. Each shape has its own density.
	b.m_mass = 0.0f;
	b.m_invMass = 0.0f;
	b.m_I = 0.0f;
	b.m_invI = 0.0f;
	b.m_sweep.localCenter.SetZero();

	// Static and kinematic bodies have zero mass.
	if (b.m_type == b2_staticBody || b.m_type == b2_kinematicBody)
	{
		b.m_sweep.c0 = b.m_xf.p;
		b.m_sweep.c = b.m_xf.p;
		b.m_sweep.a0 = b.m_sweep.a;
		return;
	}

	b2Assert(b.m_type == b2_dynamicBody);

	// Accumulate mass over all fixtures.
	b2Vec2 localCenter = b2Vec2_zero;
	for (int32 fIdx : b.m_fixtureIdxBuffer)
	{
		Fixture& f = m_fixtureBuffer[fIdx];
		if (f.m_density == 0.0f)
			continue;

		b2MassData massData = GetMassData(f);
		b.m_mass += massData.mass;
		localCenter += massData.mass * massData.center;
		b.m_I += massData.I;
	}

	// Compute center of mass.
	if (b.m_mass > 0.0f)
	{
		b.m_invMass = 1.0f / b.m_mass;
		localCenter *= b.m_invMass;
	}
	else
	{
		// Force all dynamic bodies to have a positive mass.
		b.m_mass = 1.0f;
		b.m_invMass = 1.0f;
	}

	if (b.m_I > 0.0f && (b.m_flags & b2_fixedRotationBody) == 0)
	{
		// Center the inertia about the center of mass.
		b.m_I -= b.m_mass * b2Dot(localCenter, localCenter);
		b2Assert(m_I > 0.0f);
		b.m_invI = 1.0f / b.m_I;

	}
	else
	{
		b.m_I = 0.0f;
		b.m_invI = 0.0f;
	}

	// Move center of mass.
	b2Vec2 oldCenter = b.m_sweep.c;
	b.m_sweep.localCenter = localCenter;
	b.m_sweep.c0 = b.m_sweep.c = b2Mul(b.m_xf, b.m_sweep.localCenter);

	// Update center of mass velocity.
	b.m_linearVelocity += b2Cross(b.m_angularVelocity, b.m_sweep.c - oldCenter);
}


inline Shape::Type b2World::GetType(const Fixture& f) const
{
	return m_shapeBuffer[f.m_shapeIdx].m_type;
}

inline bool b2World::TestPoint(const Fixture& f, const b2Vec2& p) const
{
	const Shape& s = m_shapeBuffer[f.m_shapeIdx];
	const Body& b = m_bodyBuffer[f.m_bodyIdx];
	switch (s.m_type)
	{
	case Shape::Type::e_chain:
		return m_chainShapeBuffer[s.m_idx].TestPoint(b.m_xf, p);
	case Shape::Type::e_circle:
		return m_circleShapeBuffer[s.m_idx].TestPoint(b.m_xf, p);
	case Shape::Type::e_edge:
		return m_edgeShapeBuffer[s.m_idx].TestPoint(b.m_xf, p);
	case Shape::Type::e_polygon:
		return m_polygonShapeBuffer[s.m_idx].TestPoint(b.m_xf, p);
	}
}

inline void b2World::ComputeDistance(const Fixture& f, const b2Vec2& p, float32* d, b2Vec2* n, int32 childIndex) const
{
	const Shape& s = m_shapeBuffer[f.m_shapeIdx];
	const Body& b = m_bodyBuffer[f.m_bodyIdx];
	switch (s.m_type)
	{
	case Shape::Type::e_chain:
		return m_chainShapeBuffer[s.m_idx].ComputeDistance(b.m_xf, p, d, n, childIndex);
	case Shape::Type::e_circle:
		return m_circleShapeBuffer[s.m_idx].ComputeDistance(b.m_xf, p, d, n, childIndex);
	case Shape::Type::e_edge:
		return m_edgeShapeBuffer[s.m_idx].ComputeDistance(b.m_xf, p, d, n, childIndex);
	case Shape::Type::e_polygon:
		return m_polygonShapeBuffer[s.m_idx].ComputeDistance(b.m_xf, p, d, n, childIndex);
	}
}
inline bool b2World::RayCast(const Fixture& f, b2RayCastOutput& output, const b2RayCastInput& input, int32 childIndex) const
{
	const Shape& s = m_shapeBuffer[f.m_shapeIdx];
	const Body& b = m_bodyBuffer[f.m_bodyIdx];
	switch (s.m_type)
	{
	case Shape::Type::e_chain:
		return m_chainShapeBuffer[s.m_idx].RayCast(output, input, b.m_xf, childIndex);
	case Shape::Type::e_circle:
		return m_circleShapeBuffer[s.m_idx].RayCast(output, input, b.m_xf, childIndex);
	case Shape::Type::e_edge:
		return m_edgeShapeBuffer[s.m_idx].RayCast(output, input, b.m_xf, childIndex);
	case Shape::Type::e_polygon:
		return m_polygonShapeBuffer[s.m_idx].RayCast(output, input, b.m_xf, childIndex);
	}
}

inline b2MassData b2World::GetMassData(const Fixture& f) const
{
	const Shape& s = m_shapeBuffer[f.m_shapeIdx];
	switch (s.m_type)
	{
	case Shape::Type::e_chain:
		return m_chainShapeBuffer[s.m_idx].ComputeMass(f.m_density);
	case Shape::Type::e_circle:
		return m_circleShapeBuffer[s.m_idx].ComputeMass(f.m_density);
	case Shape::Type::e_edge:
		return m_edgeShapeBuffer[s.m_idx].ComputeMass(f.m_density);
	case Shape::Type::e_polygon:
		return m_polygonShapeBuffer[s.m_idx].ComputeMass(f.m_density);
	}
}

inline const b2AABB& b2World::GetAABB(const Fixture& f, int32 childIndex) const
{	
	return m_fixtureProxiesBuffer[f.m_idx][childIndex].aabb;
}

void b2World::SetSensor(Fixture& f, bool sensor)
{
	if (sensor != f.m_isSensor)
	{
		m_bodyBuffer[f.m_bodyIdx].SetAwake(true);
		f.m_isSensor = sensor;
	}
}

void b2World::SetFilterData(Fixture& f, const b2Filter& filter)
{
	f.m_filter = filter;
	Refilter(f);
}

void b2World::Refilter(Fixture& f)
{
	const Body& b = m_bodyBuffer[f.m_bodyIdx];
	if (b.m_idx == b2_invalidIndex) return;

	// Flag associated contacts for filtering.
	b2ContactEdge* edge = b.m_contactList;
	while (edge)
	{
		b2Contact* contact = edge->contact;
		Fixture* fixtureA = contact->GetFixtureA();
		Fixture* fixtureB = contact->GetFixtureB();
		if (fixtureA->m_idx == f.m_idx || fixtureB->m_idx == f.m_idx)
			contact->FlagForFiltering();

		edge = edge->next;
	}

	// Touch each proxy so that new pairs may be created

	auto& proxies = m_fixtureProxiesBuffer[f.m_idx];
	for (int32 i = 0; i < f.m_proxyCount; ++i)
		m_contactManager.m_broadPhase.TouchProxy(proxies[i].proxyId);
}

void b2World::Synchronize(Fixture& f, b2BroadPhase& broadPhase, const b2Transform& transform1, const b2Transform& transform2)
{
	if (f.m_proxyCount == 0)
		return;

	auto& proxies = m_fixtureProxiesBuffer[f.m_idx];
	b2Shape& shape = GetSubShape(m_shapeBuffer[f.m_idx]);
	for (int32 i = 0; i < f.m_proxyCount; ++i)
	{
		b2FixtureProxy& proxy = proxies[i];

		// Compute an AABB that covers the swept shape (may miss some rotation effect).
		b2AABB aabb1, aabb2;
		shape.ComputeAABB(aabb1, transform1, proxy.childIndex);
		shape.ComputeAABB(aabb2, transform2, proxy.childIndex);

		proxy.aabb.Combine(aabb1, aabb2);

		b2Vec2 displacement = transform2.p - transform1.p;

		broadPhase.MoveProxy(proxy.proxyId, proxy.aabb, displacement);
	}
}


void b2World::CreateProxies(Fixture& f, b2BroadPhase& broadPhase, const b2Transform& xf)
{
	b2Assert(f.m_proxyCount == 0);

	// Create proxies in the broad-phase.
	Shape& s = m_shapeBuffer[f.m_shapeIdx];
	b2Shape& subShape = GetSubShape(s);
	f.m_proxyCount = subShape.GetChildCount();
	auto& proxies = m_fixtureProxiesBuffer[f.m_idx];
	for (int32 i = 0; i < f.m_proxyCount; ++i)
	{
		b2FixtureProxy& proxy = proxies[i];
		subShape.ComputeAABB(proxy.aabb, xf, i);

		proxy.proxyId = broadPhase.CreateProxy(proxy.aabb, &proxy);
		proxy.fixture = &f;
		proxy.fixtureIdx = f.m_idx;
		proxy.childIndex = i;
	}
}

void b2World::DestroyProxies(Fixture& f)
{
	// Destroy proxies in the broad-phase.
	auto& proxies = m_fixtureProxiesBuffer[f.m_idx];
	for (int32 i = 0; i < f.m_proxyCount; ++i)
	{
		b2FixtureProxy& proxy = proxies[i];
		m_contactManager.m_broadPhase.DestroyProxy(proxy.proxyId);
		proxy.proxyId = b2BroadPhase::e_nullProxy;
	}
	f.m_proxyCount = 0;
}

void b2World::Destroy(Fixture& f)
{
	// The proxies must be destroyed before calling this.
	b2Assert(f.m_proxyCount == 0);

	// Free the proxy array.
	b2Shape& subShape = GetSubShape(m_shapeBuffer[f.m_shapeIdx]);
	int32 childCount = subShape.GetChildCount();
	auto& proxies = m_fixtureProxiesBuffer[f.m_idx];
	proxies.clear();
	f.m_shapeIdx = b2_invalidIndex;
}

void b2World::GetWorldManifold(b2Contact& c, b2WorldManifold* worldManifold) const
{
	
	const Body& bodyA = m_bodyBuffer[c.m_fixtureA->m_bodyIdx];
	const Body& bodyB = m_bodyBuffer[c.m_fixtureB->m_bodyIdx];
	const Shape& shapeA = m_shapeBuffer[c.m_fixtureA->m_shapeIdx];
	const Shape& shapeB = m_shapeBuffer[c.m_fixtureB->m_shapeIdx];

	worldManifold->Initialize(&c.m_manifold, bodyA.m_xf, shapeA.m_radius, bodyB.m_xf, shapeB.m_radius);
}

b2Contact* b2World::CreateContact(Fixture& fixtureA, int32 indexA, Fixture& fixtureB, int32 indexB, b2BlockAllocator* allocator)
{
	if (b2Contact::s_initialized == false)
	{
		b2Contact::InitializeRegisters();
		b2Contact::s_initialized = true;
	}

	Shape::Type type1 = GetType(fixtureA);
	Shape::Type type2 = GetType(fixtureB);

	b2Assert(0 <= type1 && type1 < b2Shape::e_typeCount);
	b2Assert(0 <= type2 && type2 < b2Shape::e_typeCount);

	b2ContactCreateFcn* createFcn = b2Contact::s_registers[type1][type2].createFcn;
	if (createFcn)
	{
		if (b2Contact::s_registers[type1][type2].primary)
			return createFcn(fixtureA, indexA, fixtureB, indexB, allocator);
		else
			return createFcn(fixtureB, indexB, fixtureA, indexA, allocator);
	}
	else
		return nullptr;
}

void b2World::Destroy(b2Contact& c, b2BlockAllocator* allocator)
{
	b2Assert(s_initialized == true);

	Fixture* fixtureA = c.m_fixtureA;
	Fixture* fixtureB = c.m_fixtureB;

	if (c.m_manifold.pointCount > 0 &&
		fixtureA->IsSensor() == false &&
		fixtureB->IsSensor() == false)
	{
		m_bodyBuffer[fixtureA->m_bodyIdx].SetAwake(true);
		m_bodyBuffer[fixtureB->m_bodyIdx].SetAwake(true);
	}

	Shape::Type typeA = m_shapeBuffer[fixtureA->m_shapeIdx].m_type;
	Shape::Type typeB = m_shapeBuffer[fixtureB->m_shapeIdx].m_type;

	b2Assert(0 <= typeA && typeB < b2Shape::e_typeCount);
	b2Assert(0 <= typeA && typeB < b2Shape::e_typeCount);

	b2ContactDestroyFcn* destroyFcn = b2Contact::s_registers[typeA][typeB].destroyFcn;
	destroyFcn(c, allocator);
}

void b2World::Evaluate(b2Contact& c, b2Manifold& manifold, const b2Transform& xfA, const b2Transform& xfB)
{
	b2Shape& shapeA = GetSubShape(m_shapeBuffer[c.m_fixtureA->m_shapeIdx]);
	b2Shape& shapeB = GetSubShape(m_shapeBuffer[c.m_fixtureB->m_shapeIdx]);
	
	if (shapeA.m_type == Shape::e_chain && shapeB.m_type == Shape::e_circle)
	{
		b2EdgeShape edge;
		((b2ChainShape&)shapeA).GetChildEdge(edge, c.m_indexA);
		if (shapeB.m_type == Shape::e_circle)
			b2CollideEdgeAndCircle(manifold, edge, xfA, (b2CircleShape&)shapeB, xfB);
		else if (shapeB.m_type == Shape::e_polygon)
			b2CollideEdgeAndPolygon(manifold, edge, xfA, (b2PolygonShape&)shapeB, xfB);
	}
	else if (shapeA.m_type == Shape::e_circle && shapeB.m_type == Shape::e_circle)
		b2CollideCircles(manifold, (b2CircleShape&)shapeA, xfA, (b2CircleShape&)shapeB, xfB);

	else if (shapeA.m_type == Shape::e_edge && shapeB.m_type == Shape::e_circle)
		b2CollideEdgeAndCircle(manifold, (b2EdgeShape&)shapeA, xfA, (b2CircleShape&)shapeB, xfB);

	else if (shapeA.m_type == Shape::e_edge && shapeB.m_type == Shape::e_polygon)
		b2CollideEdgeAndPolygon(manifold, (b2EdgeShape&)shapeA, xfA, (b2PolygonShape&)shapeB, xfB);

	else if (shapeA.m_type == Shape::e_polygon && shapeB.m_type == Shape::e_circle)
		b2CollidePolygonAndCircle(manifold, (b2PolygonShape&)shapeA, xfA, (b2CircleShape&)shapeB, xfB);

	else if (shapeA.m_type == Shape::e_polygon && shapeB.m_type == Shape::e_polygon)
		b2CollidePolygons(manifold, (b2PolygonShape&)shapeA, xfA, (b2PolygonShape&)shapeB, xfB);
}

// Update the contact manifold and touching status.
// Note: do not assume the fixture AABBs are overlapping or are valid.
void b2World::Update(b2Contact& c)
{
	b2Manifold oldManifold = c.m_manifold;

	// Re-enable this contact.
	c.m_flags |= b2Contact::e_enabledFlag;

	bool touching = false;
	bool wasTouching = (c.m_flags & b2Contact::e_touchingFlag) == b2Contact::e_touchingFlag;

	bool sensorA = c.m_fixtureA->m_isSensor;
	bool sensorB = c.m_fixtureB->m_isSensor;
	bool sensor = sensorA || sensorB;

	Body& bodyA = m_bodyBuffer[c.m_fixtureA->m_bodyIdx];
	Body& bodyB = m_bodyBuffer[c.m_fixtureB->m_bodyIdx];
	const b2Transform& xfA = bodyA.m_xf;
	const b2Transform& xfB = bodyB.m_xf;

	// Is this contact a sensor?
	if (sensor)
	{
		const Shape& shapeA = m_shapeBuffer[c.m_fixtureA->m_shapeIdx];
		const Shape& shapeB = m_shapeBuffer[c.m_fixtureB->m_shapeIdx];
		touching = b2TestOverlap(shapeA, c.m_indexA, shapeB, c.m_indexB, xfA, xfB);

		// Sensors don't generate manifolds.
		c.m_manifold.pointCount = 0;
	}
	else
	{
		Evaluate(c, c.m_manifold, xfA, xfB);
		touching = c.m_manifold.pointCount > 0;

		// Match old contact ids to new contact ids and copy the
		// stored impulses to warm start the solver.
		for (int32 i = 0; i < c.m_manifold.pointCount; ++i)
		{
			b2ManifoldPoint* mp2 = c.m_manifold.points + i;
			mp2->normalImpulse = 0.0f;
			mp2->tangentImpulse = 0.0f;
			b2ContactID id2 = mp2->id;

			for (int32 j = 0; j < oldManifold.pointCount; ++j)
			{
				b2ManifoldPoint* mp1 = oldManifold.points + j;

				if (mp1->id.key == id2.key)
				{
					mp2->normalImpulse = mp1->normalImpulse;
					mp2->tangentImpulse = mp1->tangentImpulse;
					break;
				}
			}
		}

		if (touching != wasTouching)
		{
			bodyA.SetAwake(true);
			bodyB.SetAwake(true);
		}
	}

	if (touching)
		c.m_flags |= b2Contact::e_touchingFlag;
	else
		c.m_flags &= ~b2Contact::e_touchingFlag;

	if (wasTouching == false && touching == true && m_contactManager.m_contactListener)
		m_contactManager.m_contactListener->BeginContact(c);

	if (wasTouching == true && touching == false && m_contactManager.m_contactListener)
		m_contactManager.m_contactListener->EndContact(c);

	if (sensor == false && touching && m_contactManager.m_contactListener)
		m_contactManager.m_contactListener->PreSolve(c, &oldManifold);
}
