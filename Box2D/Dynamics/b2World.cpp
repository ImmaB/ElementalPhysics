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

b2World::b2World() :
	m_contactManager(*this),
	m_ampBodyMaterials(16, amp::getGpuAccelView())
{
	m_destructionListener = NULL;
	m_debugDraw = NULL;

	m_jointList = NULL;
	m_particleSystem = NULL;

	m_jointCount = 0;

	m_warmStarting = true;
	m_continuousPhysics = true;
	m_subStepping = false;

	m_stepComplete = true;

	m_allowSleep = true;

	m_flags = e_clearForces;

	m_inv_dt0 = 0.0f;

	m_contactManager.m_allocator = &m_blockAllocator;

	m_liquidFunVersion = &b2_liquidFunVersion;
	m_liquidFunVersionString = b2_liquidFunVersionString;

	m_allBodyMaterialFlags = 0;

	memset(&m_profile, 0, sizeof(b2Profile));
}

b2World::~b2World()
{
	// Some shapes allocate using b2Alloc.
	for (Body& b : m_bodyBuffer)
	{
		if (b.m_idx == b2_invalidIndex)
		{
			
			for each (int32 fIdx in m_bodyFixtureIdxsBuffer[b.m_idx])
			{
				if (fIdx != b2_invalidIndex)
				{
					Fixture& f = m_fixtureBuffer[fIdx];
					f.m_proxyCount = 0;
					DestroyShape(f);
				}
			}
		}
	}

	m_ground->~Ground();
	free(m_ground);
	DestroyParticleSystem();

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
	b2Assert(!IsLocked());
	if (IsLocked()) return b2_invalidIndex;

	int32 idx;
	Body& b = InsertIntoBuffers(m_bodyBuffer, m_bodyFixtureIdxsBuffer, m_bodyFreeFixtureIdxsBuffer, m_bodyJointListBuffer, m_bodyContactListBuffer, m_freeBodyIdxs, idx);
	b.m_idx = idx;
	m_bodyFixtureIdxsBuffer[idx].clear();
	m_bodyFreeFixtureIdxsBuffer[idx].clear();
	m_bodyJointListBuffer[idx] = NULL;
	m_bodyContactListBuffer[idx] = NULL;
	b.Set(def);

	return idx;
}


void b2World::DestroyBody(int32 idx)
{
	b2Assert(m_bodyCount > 0);
	b2Assert(!IsLocked());
	if (IsLocked()) return;

	// Delete the attached joints.
	Body& b = m_bodyBuffer[idx];
	b2JointEdge* je = m_bodyJointListBuffer[b.m_idx];
	while (je)
	{
		b2JointEdge* je0 = je;
		je = je->next;

		if (m_destructionListener)
			m_destructionListener->SayGoodbye(je0->joint);

		DestroyJoint(je0->joint);

		m_bodyJointListBuffer[b.m_idx] = je;
	}
	m_bodyJointListBuffer[b.m_idx] = NULL;

	// Delete the attached contacts.
	b2ContactEdge* ce = m_bodyContactListBuffer[b.m_idx];
	while (ce)
	{
		b2ContactEdge* ce0 = ce;
		ce = ce->next;
		m_contactManager.Destroy(*ce0->contact);
	}
	m_bodyContactListBuffer[b.m_idx] = NULL;

	// Delete the attached fixtures. This destroys broad-phase proxies.
	for each (int32 fIdx in m_bodyFixtureIdxsBuffer[b.m_idx])
		if (fIdx != b2_invalidIndex)
			DestroyFixture(b, fIdx);
	
	RemoveFromBuffers(idx, m_bodyBuffer, m_bodyContactListBuffer, m_bodyFixtureIdxsBuffer, m_bodyFreeFixtureIdxsBuffer, m_bodyJointListBuffer, m_freeBodyIdxs);
	b.m_idx = b2_invalidIndex;
}


Ground* b2World::CreateGround(const Ground::def& gd)
{
	m_ground = new Ground(*this, gd);
	return m_ground;
}

b2Joint* b2World::CreateJoint(const b2JointDef* def)
{
	b2Assert(!IsLocked());
	if (IsLocked()) return NULL;

	b2Joint* j = b2Joint::Create(def, *this, m_blockAllocator);

	// Connect to the world list.
	j->m_prev = NULL;
	j->m_next = m_jointList;
	if (m_jointList)
		m_jointList->m_prev = j;
	m_jointList = j;
	++m_jointCount;

	// Connect to the bodies' doubly linked lists.
	j->m_edgeA.joint = j;
	j->m_edgeA.otherIdx = j->m_bodyBIdx;
	j->m_edgeA.prev = NULL;
	j->m_edgeA.next = m_bodyJointListBuffer[j->m_bodyAIdx];
	if (j->m_edgeA.next) j->m_edgeA.next->prev = &j->m_edgeA;
	m_bodyJointListBuffer[j->m_bodyAIdx] = &j->m_edgeA;

	j->m_edgeB.joint = j;
	j->m_edgeB.otherIdx = j->m_bodyAIdx;
	j->m_edgeB.prev = NULL;
	j->m_edgeB.next = m_bodyJointListBuffer[j->m_bodyBIdx];
	if (j->m_edgeB.next) j->m_edgeB.next->prev = &j->m_edgeB;
	m_bodyJointListBuffer[j->m_bodyBIdx] = &j->m_edgeB;

	int32 bodyAIdx = def->bodyAIdx;
	int32 bodyBIdx = def->bodyBIdx;

	// If the joint prevents collisions, then flag any contacts for filtering.
	if (!def->collideConnected)
	{
		b2ContactEdge* edge = m_bodyContactListBuffer[bodyBIdx];
		while (edge)
		{
			if (edge->otherIdx == bodyAIdx)
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
	b2Assert(!IsLocked());
	if (IsLocked())
		return;

	bool collideConnected = j->m_collideConnected;

	// Remove from the doubly linked list.
	if (j->m_prev)
		j->m_prev->m_next = j->m_next;

	if (j->m_next)
		j->m_next->m_prev = j->m_prev;

	if (j == m_jointList)
		m_jointList = j->m_next;

	// Disconnect from island graph.
	const int32 bodyAIdx = j->m_bodyAIdx;
	const int32 bodyBIdx = j->m_bodyBIdx;

	// Wake up connected bodies.
	m_bodyBuffer[bodyAIdx].SetAwake(true);
	m_bodyBuffer[bodyBIdx].SetAwake(true);

	// Remove from body 1.
	if (j->m_edgeA.prev)
		j->m_edgeA.prev->next = j->m_edgeA.next;

	if (j->m_edgeA.next)
		j->m_edgeA.next->prev = j->m_edgeA.prev;

	if (&j->m_edgeA == m_bodyJointListBuffer[bodyAIdx])
		m_bodyJointListBuffer[bodyAIdx] = j->m_edgeA.next;

	j->m_edgeA.prev = NULL;
	j->m_edgeA.next = NULL;

	// Remove from body 2
	if (j->m_edgeB.prev)
		j->m_edgeB.prev->next = j->m_edgeB.next;

	if (j->m_edgeB.next)
		j->m_edgeB.next->prev = j->m_edgeB.prev;

	if (&j->m_edgeB == m_bodyJointListBuffer[bodyBIdx])
		m_bodyJointListBuffer[bodyBIdx] = j->m_edgeB.next;

	j->m_edgeB.prev = NULL;
	j->m_edgeB.next = NULL;

	b2Joint::Destroy(j, m_blockAllocator);

	b2Assert(m_jointCount > 0);
	--m_jointCount;

	// If the joint prevents collisions, then flag any contacts for filtering.
	if (!collideConnected)
	{
		b2ContactEdge* edge = m_bodyContactListBuffer[bodyBIdx];
		while (edge)
		{
			if (edge->otherIdx == bodyAIdx)
			{
				// Flag the contact for filtering at the next time step (where either
				// body is awake).
				edge->contact->FlagForFiltering();
			}

			edge = edge->next;
		}
	}
}

b2ParticleSystem* b2World::CreateParticleSystem()
{
	b2Assert(!IsLocked());
	if (IsLocked()) return NULL;

	m_particleSystem = new b2ParticleSystem(*this, m_step, m_bodyBuffer, m_fixtureBuffer);
	return m_particleSystem;
}

void b2World::DestroyParticleSystem()
{
	if (IsLocked() || m_particleSystem == NULL)
		return;
	m_particleSystem->~b2ParticleSystem();
	free(m_particleSystem);
	m_particleSystem = NULL;
}

//
void b2World::SetAllowSleeping(bool flag)
{
	if (flag == m_allowSleep)
		return;

	m_allowSleep = flag;
	if (!m_allowSleep)
		for (Body& b : m_bodyBuffer)
			if (b.m_idx != b2_invalidIndex)
				b.SetAwake(true);
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
			b.RemFlag(Body::Flag::island);

	for (b2Contact* c = m_contactManager.m_contactList; c; c = c->m_next)
		c->RemFlag(b2Contact::e_islandFlag);

	for (b2Joint* j = m_jointList; j; j = j->m_next)
		j->m_islandFlag = false;

	// Build and simulate all awake islands.
	int32 stackSize = m_bodyBuffer.size();
	std::vector<int32> stack(stackSize);
	for (Body& seed : m_bodyBuffer)
	{
		if (seed.m_idx == b2_invalidIndex) continue;

		if (seed.HasFlag(Body::Flag::island))
			continue;

		if (!seed.IsAwake() || !seed.IsActive())
			continue;

		// The seed can be dynamic or kinematic.
		if (seed.m_type == b2_staticBody)
			continue;

		// Reset island and stack.
		island.Clear();
		int32 stackCount = 0;
		stack[stackCount++] = seed.m_idx;
		seed.AddFlag(Body::Flag::island);

		// Perform a depth first search (DFS) on the constraint graph.
		while (stackCount > 0)
		{
			// Grab the next body off the stack and add it to the island.
			Body& b = m_bodyBuffer[stack[--stackCount]];
			b2Assert(b->IsActive());
			island.Add(b);

			// Make sure the body is awake.
			b.SetAwake(true);

			// To keep islands as small as possible, we don't
			// propagate islands across static bodies.
			if (b.m_type == b2_staticBody)
				continue;

			// Search all contacts connected to this body.
			for (b2ContactEdge* ce = m_bodyContactListBuffer[b.m_idx]; ce; ce = ce->next)
			{
				b2Contact* contact = ce->contact;

				// Has this contact already been added to an island?
				if (contact->m_flags & b2Contact::e_islandFlag)
					continue;

				// Is this contact solid and touching?
				if (!contact->IsEnabled() ||
					!contact->IsTouching())
					continue;

				// Skip sensors.
					
				bool sensorA = m_fixtureBuffer[contact->m_fixtureIdxA].m_isSensor;
				bool sensorB = m_fixtureBuffer[contact->m_fixtureIdxB].m_isSensor;
				if (sensorA || sensorB)
					continue;

				island.Add(contact);
				contact->m_flags |= b2Contact::e_islandFlag;

				Body& other = m_bodyBuffer[ce->otherIdx];

				// Was the other body already added to this island?
				if (other.HasFlag(Body::Flag::island))
					continue;

				b2Assert(stackCount < stackSize);
				stack[stackCount++] = other.m_idx;
				other.AddFlag(Body::Flag::island);
			}

			// Search all joints connect to this body.
			for (b2JointEdge* je = m_bodyJointListBuffer[b.m_idx]; je; je = je->next)
			{
				if (je->joint->m_islandFlag)
					continue;

				Body& other = m_bodyBuffer[je->otherIdx];

				// Don't simulate joints connected to inactive bodies.
				if (!other.IsActive())
					continue;

				island.Add(je->joint);
				je->joint->m_islandFlag = true;

				if (other.HasFlag(Body::Flag::island))
					continue;

				b2Assert(stackCount < stackSize);
				stack[stackCount++] = other.m_idx;
				other.AddFlag(Body::Flag::island);
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
			Body& b = m_bodyBuffer[island.m_bodyIdxs[i]];
			if (b.IsType(b2_staticBody))
				b.RemFlag(Body::Flag::island);
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
				if (!b.HasFlag(Body::Flag::island))
					continue;

				if (b.IsType(b2_staticBody))
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
void b2World::SolveTOI(const b2TimeStep& step)	// TOI = Time Of Impact
{
	b2Island island(2 * b2_maxTOIContacts, b2_maxTOIContacts, 0, &m_stackAllocator, m_contactManager.m_contactListener, *this);

	if (m_stepComplete)
	{
		for (Body& b : m_bodyBuffer)
		{
			if (b.m_idx != b2_invalidIndex)
			{
				b.RemFlag(Body::Flag::island);
				b.m_sweep.alpha0 = 0.0f;
			}
		}

		for (b2Contact* c = m_contactManager.m_contactList; c; c = c->m_next)
		{
			// Invalidate TOI
			c->RemFlag(b2Contact::e_toiFlag | b2Contact::e_islandFlag);
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
			if (!c->IsEnabled())
				continue;

			// Prevent excessive sub-stepping.
			if (c->m_toiCount > b2_maxSubSteps)
				continue;

			float32 alpha = 1.0f;
			if (c->m_flags & b2Contact::e_toiFlag) // This contact has a valid cached TOI.
				alpha = c->m_toi;
			else
			{
				Fixture& fA = m_fixtureBuffer[c->m_fixtureIdxA];
				Fixture& fB = m_fixtureBuffer[c->m_fixtureIdxB];

				// Is there a sensor?
				if (fA.m_isSensor || fB.m_isSensor)
					continue;

				Body& bA = m_bodyBuffer[fA.m_bodyIdx];
				Body& bB = m_bodyBuffer[fB.m_bodyIdx];

				b2BodyType typeA = bA.m_type;
				b2BodyType typeB = bB.m_type;
				b2Assert(typeA == b2_dynamicBody || typeB == b2_dynamicBody);

				bool activeA = bA.IsAwake() && typeA != b2_staticBody;
				bool activeB = bB.IsAwake() && typeB != b2_staticBody;

				// Is at least one body active (awake and dynamic or kinematic)?
				if (!activeA && !activeB)
					continue;

				bool collideA = bA.IsBullet() || typeA != b2_dynamicBody;
				bool collideB = bB.IsBullet() || typeB != b2_dynamicBody;

				// Are these two non-bullet dynamic bodies?
				if (!collideA && !collideB)
					continue;

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
				input.proxyA.Set(GetShape(fA), indexA);
				input.proxyB.Set(GetShape(fB), indexB);

				input.sweepA = bA.m_sweep;
				input.sweepB = bB.m_sweep;
				input.tMax = 1.0f;

				b2TOIOutput output;
				b2TimeOfImpact(output, input);

				// Beta is the fraction of the remaining portion of the .
				float32 beta = output.t;
				if (output.state == b2TOIOutput::e_touching)
					alpha = b2Min(alpha0 + (1.0f - alpha0) * beta, 1.0f);
				else
					alpha = 1.0f;

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
		Body& bA = GetFixtureBody(minContact->m_fixtureIdxA);
		Body& bB = GetFixtureBody(minContact->m_fixtureIdxB);

		b2Sweep backup1 = bA.m_sweep;
		b2Sweep backup2 = bB.m_sweep;

		bA.Advance(minAlpha);
		bB.Advance(minAlpha);

		// The TOI contact likely has some new contact points.
		Update(*minContact);
		minContact->RemFlag(b2Contact::e_toiFlag);
		++minContact->m_toiCount;

		// Is the contact solid?
		if (!minContact->IsEnabled() || !minContact->IsTouching())
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
		island.Add(bA);
		island.Add(bB);
		island.Add(minContact);

		bA.AddFlag(Body::Flag::island);
		bB.AddFlag(Body::Flag::island);
		minContact->m_flags |= b2Contact::e_islandFlag;

		// Get contacts on bodyA and bodyB.
		for (Body* body : { &bA, &bB })
		{
			if (body->m_type == b2_dynamicBody)
			{
				for (b2ContactEdge* ce = m_bodyContactListBuffer[body->m_idx]; ce; ce = ce->next)
				{
					if (island.m_bodyCount == island.m_bodyCapacity)
						break;

					if (island.m_contactCount == island.m_contactCapacity)
						break;

					b2Contact* contact = ce->contact;

					// Has this contact already been added to the island?
					if (contact->m_flags & b2Contact::e_islandFlag)
						continue;

					// Only add static, kinematic, or bullet bodies.
					Body& other = m_bodyBuffer[ce->otherIdx];
					if (other.m_type == b2_dynamicBody &&
						!body->IsBullet() && !other.IsBullet())
						continue;

					// Skip sensors.
					bool sensorA = m_fixtureBuffer[contact->m_fixtureIdxA].m_isSensor;
					bool sensorB = m_fixtureBuffer[contact->m_fixtureIdxB].m_isSensor;
					if (sensorA || sensorB)
						continue;

					// Tentatively advance the body to the TOI.
					b2Sweep backup = other.m_sweep;
					if (!other.HasFlag(Body::Flag::island))
						other.Advance(minAlpha);

					// Update the contact points
					Update(*contact);

					// Was the contact disabled by the user?
					if (!contact->IsEnabled())
					{
						other.m_sweep = backup;
						other.SynchronizeTransform();
						continue;
					}

					// Are there contact points?
					if (!contact->IsTouching())
					{
						other.m_sweep = backup;
						other.SynchronizeTransform();
						continue;
					}

					// Add the contact to the island
					contact->m_flags |= b2Contact::e_islandFlag;
					island.Add(contact);

					// Has the other body already been added to the island?
					if (other.HasFlag(Body::Flag::island))
						continue;

					// Add the other body to the island.
					other.AddFlag(Body::Flag::island);

					if (other.m_type != b2_staticBody)
						other.SetAwake(true);

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
			Body& body = m_bodyBuffer[island.m_bodyIdxs[i]];
			body.RemFlag(Body::Flag::island);

			if (!body.IsType(b2_dynamicBody))
				continue;

			SynchronizeFixtures(body);

			// Invalidate all contact TOIs on this displaced body.
			for (b2ContactEdge* ce = m_bodyContactListBuffer[body.m_idx]; ce; ce = ce->next)
				ce->contact->RemFlag(b2Contact::e_toiFlag | b2Contact::e_islandFlag);
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
		m_step.inv_dt = 1.0f / dt;
	else
		m_step.inv_dt = 0.0f;
	m_step.dtRatio = m_inv_dt0 * dt;
	m_step.warmStarting = m_warmStarting;
	m_step.velocityIterations = velocityIterations;
	m_step.positionIterations = positionIterations;
	m_step.particleIterations = particleIterations;
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

void b2World::QueryAABB(b2QueryCallback* callback, const b2AABB& aabb) const
{
	m_contactManager.m_broadPhase.Query(aabb, [=](int32 proxyId) -> bool
	{
		b2FixtureProxy* proxy = m_contactManager.m_broadPhase.GetUserData(proxyId);
		return callback->ReportFixture(proxy->fixtureIdx);
	});
	m_particleSystem->QueryAABB(callback, aabb);
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
	b2WorldRayCastWrapper(const b2World& world, const b2BroadPhase& broadPhase, b2RayCastCallback& callback)
		: m_world(world), m_broadPhase(broadPhase), m_callback(callback) {};

	float32 RayCastCallback(const b2RayCastInput& input, int32 proxyId)
	{
		b2FixtureProxy* proxy = (b2FixtureProxy*)m_broadPhase.GetUserData(proxyId);
		int32 fixtureIdx = proxy->fixtureIdx;
		b2RayCastOutput output;
		const Fixture& f = m_world.GetFixture(proxy->fixtureIdx);
		bool hit = m_world.RayCast(f, output, input, proxy->childIndex);

		if (hit)
		{
			float32 fraction = output.fraction;
			b2Vec2 point = (1.0f - fraction) * input.p1 + fraction * input.p2;
			return m_callback.ReportFixture(f, point, output.normal, fraction);
		}

		return input.maxFraction;
	}

	const b2World& m_world;
	const b2BroadPhase& m_broadPhase;
	b2RayCastCallback& m_callback;
};

void b2World::RayCast(b2RayCastCallback& callback, const b2Vec2& point1, const b2Vec2& point2) const
{
	b2WorldRayCastWrapper wrapper(*this, m_contactManager.m_broadPhase, callback);
	b2RayCastInput input;
	input.maxFraction = 1.0f;
	input.p1 = point1;
	input.p2 = point2;
	m_contactManager.m_broadPhase.RayCast(wrapper, input);
	m_particleSystem->RayCast(callback, point1, point2);
}

void b2World::DrawShape(Fixture& fixture, const b2Transform& xf, const b2Color& color)
{
	switch (fixture.m_shapeType)
	{
	case b2Shape::Type::e_circle:
		{
			b2CircleShape& circle = m_circleShapeBuffer[fixture.m_shapeIdx];
			b2Vec2 center = b2Mul(xf, circle.m_p);
			float32 radius = circle.m_radius;
			b2Vec2 axis = b2Mul(xf.q, b2Vec2(1.0f, 0.0f));

			m_debugDraw->DrawSolidCircle(center, radius, axis, color);
		}
		break;

	case b2Shape::Type::e_edge:
		{
			b2EdgeShape& edge = m_edgeShapeBuffer[fixture.m_shapeIdx];
			b2Vec2 v1 = b2Mul(xf, edge.m_vertex1);
			b2Vec2 v2 = b2Mul(xf, edge.m_vertex2);
			m_debugDraw->DrawSegment(v1, v2, color);
		}
		break;

	case b2Shape::Type::e_chain:
		{
			b2ChainShape& chain = m_chainShapeBuffer[fixture.m_shapeIdx];
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

	case b2Shape::Type::e_polygon:
		{
			b2PolygonShape& poly = m_polygonShapeBuffer[fixture.m_shapeIdx];
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
	Body& bodyA = joint->GetBodyA();
	Body& bodyB = joint->GetBodyB();
	b2Vec2 x1 = bodyA.m_xf.p;
	b2Vec2 x2 = bodyB.m_xf.p;
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
			m_debugDraw->DrawParticles(posBuf, radius, system.GetColorBuffer(), particleCount);
		else
			m_debugDraw->DrawParticles(posBuf, radius, NULL, particleCount);
	}
}

void b2World::DrawDebugData()
{
	if (m_debugDraw == NULL)
		return;

	uint32 flags = m_debugDraw->GetFlags();

	if (flags & b2Draw::e_shapeBit)
	{
		
		for (Body& b : m_bodyBuffer)
		{
			if (b.m_idx != b2_invalidIndex)
			{
				const b2Transform& xf = b.m_xf;
				
				for (int32 fIdx : m_bodyFixtureIdxsBuffer[b.m_idx])
				{
					if (fIdx != b2_invalidIndex)
					{
						Fixture& f = m_fixtureBuffer[fIdx];
						if (!b.IsActive())
							DrawShape(f, xf, b2Color(0.5f, 0.5f, 0.3f));
						else if (b.m_type == b2_staticBody)
							DrawShape(f, xf, b2Color(0.5f, 0.9f, 0.5f));
						else if (b.m_type == b2_kinematicBody)
							DrawShape(f, xf, b2Color(0.5f, 0.5f, 0.9f));
						else if (!b.IsAwake())
							DrawShape(f, xf, b2Color(0.6f, 0.6f, 0.6f));
						else
							DrawShape(f, xf, b2Color(0.9f, 0.7f, 0.7f));
					}
				}
			}
		}
	}

	if (flags & b2Draw::e_particleBit)
		DrawParticleSystem(*m_particleSystem);

	if (flags & b2Draw::e_jointBit)
		for (b2Joint* j = m_jointList; j; j = j->GetNext())
			DrawJoint(j);

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
				if (!b.IsActive())
					continue;

				for (int32 fIdx : m_bodyFixtureIdxsBuffer[b.m_idx])
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
	auto system = world->GetParticleSystem();
	if (system != nullptr)
		smallestRadius = b2Min(smallestRadius, system->GetRadius());
	return smallestRadius;
}

int b2World::CalculateReasonableParticleIterations(float32 timeStep) const
{
	if (m_particleSystem == NULL)
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

int32 b2World::CreateBodyMaterial(const Body::Mat::Def& def)
{
	// Try finding duplicat first
	for (int idx = 0; idx < m_bodyMaterials.size(); idx++)
		if (const Body::Mat& bm = m_bodyMaterials[idx];
			def.matFlags == bm.m_matFlags &&
			def.density == bm.m_density &&
			def.friction == bm.m_friction &&
			def.bounciness == bm.m_bounciness &&
			def.stability == bm.m_stability &&
			def.extinguishingPoint == bm.m_extinguishingPoint &&
			def.meltingPoint == bm.m_meltingPoint &&
			def.ignitionPoint == bm.m_ignitionPoint &&
			def.heatConductivity == bm.m_heatConductivity)
			return idx;

	const int32 idx = m_bodyMaterials.size();
	Body::Mat newMat(def);
	m_bodyMaterials.push_back(newMat);
	
	if (m_ampBodyMaterials.extent[0] <= idx) amp::resize(m_ampBodyMaterials, m_ampBodyMaterials.extent[0] * 2, m_ampBodyMaterials.extent[0]);
	amp::copy(newMat, m_ampBodyMaterials, idx);
	
	m_allBodyMaterialFlags |= def.matFlags;

	return idx;
}

template <typename T>
T& b2World::InsertIntoBuffer(vector<T>& buf, set<int32>& freeIdxs, int32& outIdx)
{
	if (freeIdxs.empty())
	{
		outIdx = buf.size();
		uint32 newSize = outIdx + 1;
		buf.resize(newSize);
	}
	else
	{
		outIdx = *freeIdxs.rbegin();
		freeIdxs.erase((--freeIdxs.end()));
	}
	return buf[outIdx];
}
template <typename T1, typename T2>
T1& b2World::InsertIntoBuffers(vector<T1>& buf1, vector<T2>& buf2, set<int32>& freeIdxs, int32& outIdx)
{
	if (freeIdxs.empty())
	{
		outIdx = buf1.size();
		uint32 newSize = outIdx + 1;
		buf1.resize(newSize);
		buf2.resize(newSize);
	}
	else
	{
		outIdx = *freeIdxs.rbegin();
		freeIdxs.erase((--freeIdxs.end()));
	}
	return buf1[outIdx];
}
template<typename T1, typename T2, typename T3, typename T4, typename T5>
T1& b2World::InsertIntoBuffers(vector<T1>& buf1, vector<T2>& buf2, vector<T3>& buf3, vector<T4>& buf4, vector<T5>& buf5, set<int32>& freeIdxs, int32& outIdx)
{
	if (freeIdxs.empty())
	{
		outIdx = buf1.size();
		uint32 newSize = outIdx + 1;
		buf1.resize(newSize);
		buf2.resize(newSize);
		buf3.resize(newSize);
		buf4.resize(newSize);
		buf5.resize(newSize);
	}
	else
	{
		outIdx = *freeIdxs.rbegin();
		freeIdxs.erase((--freeIdxs.end()));
	}
	return buf1[outIdx];
}
template <typename T>
int32 b2World::InsertIntoBuffer(T& value, vector<T>& buf, set<int32>& freeIdxs)
{
	int32 idx;
	if (freeIdxs.empty())
	{
		idx = buf.size();
		buf.push_back(value);
	}
	else
	{
		idx = *freeIdxs.rbegin();
		freeIdxs.erase((--freeIdxs.end()));
		buf[idx] = value;
	}
	return idx;
}

template<typename T>
void b2World::RemoveFromBuffer(const int32 idx, vector<T>& buffer, set<int32>& freeIdxs)
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
}

template<typename T1, typename T2>
void b2World::RemoveFromBuffers(const int32 idx, vector<T1>& buf1, vector<T2>& buf2, set<int32>& freeIdxs)
{
	if (idx == buf1.size())
	{
		buf1.pop_back();
		buf2.pop_back();
		while (!freeIdxs.empty() && *freeIdxs.rbegin() == buf1.size())
		{
			buf1.pop_back();
			buf2.pop_back();
			freeIdxs.erase(--freeIdxs.end());
		}
	}
	else
		freeIdxs.insert(idx);
}
template<typename T1, typename T2, typename T3, typename T4, typename T5>
void b2World::RemoveFromBuffers(const int32 idx, vector<T1>& buf1, vector<T2>& buf2, vector<T3>& buf3, vector<T4>& buf4, vector<T5>& buf5, set<int32>& freeIdxs)
{
	if (idx == buf1.size())
	{
		buf1.pop_back();
		buf2.pop_back();
		buf3.pop_back();
		buf4.pop_back();
		buf5.pop_back();
		while (!freeIdxs.empty() && *freeIdxs.rbegin() == buf1.size())
		{
			buf1.pop_back();
			buf2.pop_back();
			buf3.pop_back();
			buf4.pop_back();
			buf5.pop_back();
			freeIdxs.erase(--freeIdxs.end());
		}
	}
	else
		freeIdxs.insert(idx);
}


int32 b2World::CreateFixture(int32 bodyIdx, b2Shape::Type shapeType, int32 shapeIdx, float32 density)
{
	b2FixtureDef def;
	def.bodyIdx = bodyIdx;
	def.shapeType = shapeType;
	def.shapeIdx = shapeIdx;
	def.density = density;

	return CreateFixture(def);
}
int32 b2World::CreateFixture(b2FixtureDef& def)
{
	b2Assert(!IsLocked());
	if (IsLocked()) return b2_invalidIndex;


	// Update Fixture List of Body
	int32 idx, idxInBody;

	Fixture& f = InsertIntoBuffers(m_fixtureBuffer, m_fixtureProxiesBuffer, m_freeFixtureIdxs, idx);
	f.m_idx = idx;

	Body& b = m_bodyBuffer[def.bodyIdx];
	idxInBody = InsertIntoBuffer(idx, m_bodyFixtureIdxsBuffer[b.m_idx], m_bodyFreeFixtureIdxsBuffer[b.m_idx]);

	f.Set(def, idxInBody, m_bodyMaterials[b.m_matIdx]);


	// Reserve proxy space
	f.m_proxyCount = GetShape(f).GetChildCount();
	b2FixtureProxy* proxyBuffer = m_fixtureProxiesBuffer[f.m_idx] = (b2FixtureProxy*)m_blockAllocator.Allocate(f.m_proxyCount * sizeof(b2FixtureProxy));
	for (int32 i = 0; i < f.m_proxyCount; ++i)
	{
		proxyBuffer[i].fixtureIdx = b2_invalidIndex;
		proxyBuffer[i].proxyId = b2BroadPhase::e_nullProxy;
	}
	f.m_proxyCount = 0;

	if (b.HasFlag(Body::Flag::active))
		CreateProxies(f, m_contactManager.m_broadPhase, b.m_xf);

	// Adjust mass properties if needed.
	if (f.m_density > 0.0f)
		ResetMassData(b);


	// Let the world know we have a new fixture. This will cause new contacts
	// to be created at the beginning of the next time step.
	m_flags |= b2World::e_newFixture;

	return idx;
}

void b2World::DestroyShape(b2Shape::Type type, int32 idx)
{
	switch (type)
	{
		case b2Shape::e_chain:	 RemoveFromBuffer(idx, m_chainShapeBuffer, m_freeChainShapeIdxs); break;
		case b2Shape::e_circle:	 RemoveFromBuffer(idx, m_circleShapeBuffer, m_freeCircleShapeIdxs); break;
		case b2Shape::e_edge:	 RemoveFromBuffer(idx, m_edgeShapeBuffer, m_freeEdgeShapeIdxs); break;
		case b2Shape::e_polygon: RemoveFromBuffer(idx, m_polygonShapeBuffer, m_freePolygonShapeIdxs); break;
	}
}
void b2World::DestroyFixture(int32 bodyIdx, int32 idx)
{
	DestroyFixture(m_bodyBuffer[bodyIdx], idx);
}
void b2World::DestroyFixture(Body& b, int32 fIdx)
{
	b2Assert(!IsLocked());
	if (IsLocked()) return;

	Fixture& f = m_fixtureBuffer[fIdx];
	b2Assert(f.m_idx == fIdx);

	auto& bodyFixtureIdxs = m_bodyFixtureIdxsBuffer[b.m_idx];
	bodyFixtureIdxs[f.m_idxInBody] = b2_invalidIndex;
	RemoveFromBuffer(f.m_idxInBody, bodyFixtureIdxs, m_bodyFreeFixtureIdxsBuffer[b.m_idx]);

	// Remove the fixture from this body's singly linked list.
	b2Assert(b.m_fixtureCount > 0);

	// Reset the mass data.
	ResetMassData(b);

	// Destroy any contacts associated with the fixture.
	b2ContactEdge* edge = m_bodyContactListBuffer[b.m_idx];
	while (edge)
	{
		b2Contact* c = edge->contact;
		edge = edge->next;

		Fixture& fixtureA = m_fixtureBuffer[c->m_fixtureIdxA];
		Fixture& fixtureB = m_fixtureBuffer[c->m_fixtureIdxB];

		if (f.m_idx == fixtureA.m_idx || f.m_idx == fixtureB.m_idx)
			// This destroys the contact and removes it from
			// this body's contact list.
			m_contactManager.Destroy(*c);
	}

	f.m_proxyCount = GetShape(fIdx).GetChildCount();
	if (b.IsActive())
		DestroyProxies(f);
	m_blockAllocator.Free(m_fixtureProxiesBuffer[fIdx], f.m_proxyCount * sizeof(b2FixtureProxy));
	DestroyShape(f);

	f.m_idx = b2_invalidIndex;
	RemoveFromBuffers(fIdx, m_fixtureBuffer, m_fixtureProxiesBuffer, m_freeFixtureIdxs);
}

void b2World::SetTransform(Body& b, const b2Transform& transform)
{
	if (IsLocked()) return;

	b.m_xf = transform;
	b.m_xf0 = b.m_xf;

	const float32 angle = transform.q.GetAngle();
	b.m_sweep.c = b2Mul(b.m_xf, b.m_sweep.localCenter);
	b.m_sweep.a = angle;

	b.m_sweep.c0 = b.m_sweep.c;
	b.m_sweep.a0 = angle;

	b2BroadPhase& broadPhase = m_contactManager.m_broadPhase;
	for each (int32 fIdx in m_bodyFixtureIdxsBuffer[b.m_idx])
	{
		Fixture& f = m_fixtureBuffer[fIdx];
		Synchronize(f, broadPhase, b.m_xf, b.m_xf);
	}
}

void b2World::SetActive(Body& b, bool flag)
{
	b2Assert(m_world.!IsLocked());

	if (flag == b.IsActive()) return;

	if (flag)
	{
		b.AddFlag(Body::Flag::active);

		// Create all proxies.
		b2BroadPhase& broadPhase = m_contactManager.m_broadPhase;
		for each (int32 fIdx in m_bodyFixtureIdxsBuffer[b.m_idx])
			CreateProxies(m_fixtureBuffer[fIdx], broadPhase, b.m_xf);

		// Contacts are created the next time step.
	}
	else
	{
		b.RemFlag(Body::Flag::active);

		// Destroy all proxies.
		for each (int32 fIdx in m_bodyFixtureIdxsBuffer[b.m_idx])
			DestroyProxies(m_fixtureBuffer[fIdx]);

		// Destroy the attached contacts.
		b2ContactEdge* ce = m_bodyContactListBuffer[b.m_idx];
		while (ce)
		{
			b2ContactEdge* ce0 = ce;
			ce = ce->next;
			m_contactManager.Destroy(*ce0->contact);
		}
		m_bodyContactListBuffer[b.m_idx] = NULL;
	}
}

void b2World::SetFixedRotation(Body& b, bool flag)
{
	if (b.HasFlag(Body::Flag::fixedRotation) == flag)
		return;
	if (flag)
		b.AddFlag(Body::Flag::fixedRotation);
	else
		b.RemFlag(Body::Flag::fixedRotation);

	b.m_angularVelocity = 0.0f;
	ResetMassData(b);
}

bool b2World::ShouldBodiesCollide(int32 bodyAIdx, int32 bodyBIdx) const
{
	return ShouldCollide(m_bodyBuffer[bodyAIdx], m_bodyBuffer[bodyBIdx]);
}
bool b2World::ShouldCollide(const Body& b, const Body& other) const
{
	// At least one body should be dynamic.
	if (b.m_type != b2_dynamicBody && other.m_type != b2_dynamicBody)
		return false;

	// Does a joint prevent collision?
	for (b2JointEdge* jn = m_bodyJointListBuffer[b.m_idx]; jn; jn = jn->next)
		if (jn->otherIdx == other.m_idx)
			if (!jn->joint->m_collideConnected)
				return false;

	return true;
}


void b2World::SynchronizeFixtures(const Body& b)
{
	b2Transform xf1;
	xf1.q.Set(b.m_sweep.a0);
	xf1.p = b.m_sweep.c0 - b2Mul(xf1.q, b.m_sweep.localCenter);

	b2BroadPhase& broadPhase = m_contactManager.m_broadPhase;
	for each (int32 fIdx in m_bodyFixtureIdxsBuffer[b.m_idx])
	{
		if (fIdx == b2_invalidIndex) continue;
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
	for (int32 fIdx : m_bodyFixtureIdxsBuffer[b.m_idx])
	{
		if (fIdx == b2_invalidIndex) continue;
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

	if (b.m_I > 0.0f && !b.HasFlag(Body::Flag::fixedRotation))
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

bool b2World::TestPoint(const Fixture& f, const b2Vec2& p) const
{
	const Body& b = m_bodyBuffer[f.m_bodyIdx];
	return GetShape(f).TestPoint(b.m_xf, p);
}

void b2World::ComputeDistance(const Fixture& f, const b2Vec2& p, float32& d, b2Vec2& n, int32 childIndex) const
{
	const Body& b = m_bodyBuffer[f.m_bodyIdx];
	GetShape(f).ComputeDistance(b.m_xf, p, d, n, childIndex);
}
bool b2World::RayCast(const Fixture& f, b2RayCastOutput& output, const b2RayCastInput& input, int32 childIndex) const
{
	const Body& b = m_bodyBuffer[f.m_bodyIdx];
	return GetShape(f).RayCast(output, input, b.m_xf, childIndex);
}

b2MassData b2World::GetMassData(const Fixture& f) const
{
	return GetShape(f).ComputeMass(f.m_density);
}

const b2AABB& b2World::GetAABB(const Fixture& f, int32 childIdx) const
{	
	return m_fixtureProxiesBuffer[f.m_idx][childIdx].aabb;
}
const b2AABB b2World::GetAABB(const Fixture& f) const
{
	b2AABB aabb;
	const int32 childCnt = GetShape(f).GetChildCount();
	for (int32 childIdx = 0; childIdx < childCnt; childIdx++)
		aabb.Combine(GetAABB(f, childIdx));
	return aabb;
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
	b2ContactEdge* edge = m_bodyContactListBuffer[b.m_idx];
	while (edge)
	{
		b2Contact* contact = edge->contact;
		Fixture& fixtureA = m_fixtureBuffer[contact->m_fixtureIdxA];
		Fixture& fixtureB = m_fixtureBuffer[contact->m_fixtureIdxB];
		if (fixtureA.m_idx == f.m_idx || fixtureB.m_idx == f.m_idx)
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
	const b2Shape& shape = GetShape(f);
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
	const b2Shape& subShape = GetShape(f);
	f.m_proxyCount = subShape.GetChildCount();
	b2FixtureProxy* proxyBuffer = m_fixtureProxiesBuffer[f.m_idx];

	// Create proxies in the broad-phase.
	for (int32 i = 0; i < f.m_proxyCount; ++i)
	{
		b2FixtureProxy* proxy = proxyBuffer + i;
		subShape.ComputeAABB(proxy->aabb, xf, i);

		proxy->proxyId = broadPhase.CreateProxy(proxy->aabb, proxy);
		proxy->fixtureIdx = f.m_idx;
		proxy->childIndex = i;
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

int32 b2World::CreateShape(b2Shape::Def& shapeDef)
{
	int32 idx;
	b2Shape& subShape = InsertSubShapeIntoBuffer(shapeDef.type, idx);
	subShape.Set(shapeDef);
	return idx;
}
b2Shape& b2World::InsertSubShapeIntoBuffer(b2Shape::Type shapeType, int32& outIdx)
{
	switch (shapeType)
	{
		case b2Shape::Type::e_chain:	 return InsertIntoBuffer(m_chainShapeBuffer, m_freeChainShapeIdxs, outIdx);
		case b2Shape::Type::e_circle:  return InsertIntoBuffer(m_circleShapeBuffer, m_freeCircleShapeIdxs, outIdx);
		case b2Shape::Type::e_edge:	 return InsertIntoBuffer(m_edgeShapeBuffer, m_freeEdgeShapeIdxs, outIdx);
		case b2Shape::Type::e_polygon: return InsertIntoBuffer(m_polygonShapeBuffer, m_freePolygonShapeIdxs, outIdx);
	}
	return b2ChainShape();
}

void b2World::DestroyShape(Fixture& f)
{
	// The proxies must be destroyed before calling this.
	b2Assert(f.m_proxyCount == 0);

	RemoveSubShapeFromBuffer(f.m_shapeType, f.m_shapeIdx);
	f.m_shapeType = b2Shape::Type::e_typeCount;
	f.m_shapeIdx = b2_invalidIndex;
}

void b2World::RemoveSubShapeFromBuffer(b2Shape::Type shapeType, int32 idx)
{
	switch (shapeType)
	{
	case b2Shape::Type::e_chain:	 return RemoveFromBuffer(idx, m_chainShapeBuffer, m_freeChainShapeIdxs);
	case b2Shape::Type::e_circle:  return RemoveFromBuffer(idx, m_circleShapeBuffer, m_freeCircleShapeIdxs);
	case b2Shape::Type::e_edge:	 return RemoveFromBuffer(idx, m_edgeShapeBuffer, m_freeEdgeShapeIdxs);
	case b2Shape::Type::e_polygon: return RemoveFromBuffer(idx, m_polygonShapeBuffer, m_freePolygonShapeIdxs);
	}
}

void b2World::GetWorldManifold(b2Contact& c, b2WorldManifold* worldManifold) const
{
	const Fixture& fixtureA = m_fixtureBuffer[c.m_fixtureIdxA];
	const Fixture& fixtureB = m_fixtureBuffer[c.m_fixtureIdxB];
	const Body& bodyA = m_bodyBuffer[fixtureA.m_bodyIdx];
	const Body& bodyB = m_bodyBuffer[fixtureB.m_bodyIdx];
	const b2Shape& shapeA = GetShape(fixtureA); 
	const b2Shape& shapeB = GetShape(fixtureB);

	worldManifold->Initialize(&c.m_manifold, bodyA.m_xf, shapeA.m_radius, bodyB.m_xf, shapeB.m_radius);
}

b2Contact* b2World::CreateContact(Fixture& fixtureA, int32 indexA, Fixture& fixtureB, int32 indexB, b2BlockAllocator* allocator)
{
	if (!b2Contact::s_initialized)
	{
		b2Contact::InitializeRegisters();
		b2Contact::s_initialized = true;
	}

	b2Shape::Type type1 = fixtureA.m_shapeType;
	b2Shape::Type type2 = fixtureB.m_shapeType;

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
	b2Assert(s_initialized);

	const Fixture& fixtureA = m_fixtureBuffer[c.m_fixtureIdxA];
	const Fixture& fixtureB = m_fixtureBuffer[c.m_fixtureIdxB];

	if (c.m_manifold.pointCount > 0 &&
		!fixtureA.m_isSensor &&
		!fixtureB.m_isSensor)
	{
		m_bodyBuffer[fixtureA.m_bodyIdx].SetAwake(true);
		m_bodyBuffer[fixtureB.m_bodyIdx].SetAwake(true);
	}

	b2Shape::Type typeA = fixtureA.m_shapeType;
	b2Shape::Type typeB = fixtureB.m_shapeType;

	b2Assert(0 <= typeA && typeB < b2Shape::e_typeCount);
	b2Assert(0 <= typeA && typeB < b2Shape::e_typeCount);

	b2ContactDestroyFcn* destroyFcn = b2Contact::s_registers[typeA][typeB].destroyFcn;
	destroyFcn(c, allocator);
}

void b2World::Evaluate(b2Contact& c, const b2Transform& xfA, const b2Transform& xfB)
{
	const b2Shape& shapeA = GetShape(c.m_fixtureIdxA);
	const b2Shape& shapeB = GetShape(c.m_fixtureIdxB);
	
	if (shapeA.m_type == b2Shape::e_chain && shapeB.m_type == b2Shape::e_circle)
	{
		b2EdgeShape edge;
		((b2ChainShape&)shapeA).GetChildEdge(edge, c.m_indexA);
		if (shapeB.m_type == b2Shape::e_circle)
			b2CollideEdgeAndCircle(c.m_manifold, edge, xfA, (b2CircleShape&)shapeB, xfB);
		else if (shapeB.m_type == b2Shape::e_polygon)
			b2CollideEdgeAndPolygon(c.m_manifold, edge, xfA, (b2PolygonShape&)shapeB, xfB);
	}
	else if (shapeA.m_type == b2Shape::e_circle && shapeB.m_type == b2Shape::e_circle)
		b2CollideCircles(c.m_manifold, (b2CircleShape&)shapeA, xfA, (b2CircleShape&)shapeB, xfB);

	else if (shapeA.m_type == b2Shape::e_edge && shapeB.m_type == b2Shape::e_circle)
		b2CollideEdgeAndCircle(c.m_manifold, (b2EdgeShape&)shapeA, xfA, (b2CircleShape&)shapeB, xfB);

	else if (shapeA.m_type == b2Shape::e_edge && shapeB.m_type == b2Shape::e_polygon)
		b2CollideEdgeAndPolygon(c.m_manifold, (b2EdgeShape&)shapeA, xfA, (b2PolygonShape&)shapeB, xfB);

	else if (shapeA.m_type == b2Shape::e_polygon && shapeB.m_type == b2Shape::e_circle)
		b2CollidePolygonAndCircle(c.m_manifold, (b2PolygonShape&)shapeA, xfA, (b2CircleShape&)shapeB, xfB);

	else if (shapeA.m_type == b2Shape::e_polygon && shapeB.m_type == b2Shape::e_polygon)
		b2CollidePolygons(c.m_manifold, (b2PolygonShape&)shapeA, xfA, (b2PolygonShape&)shapeB, xfB);
}

// Update the contact manifold and touching status.
// Note: do not assume the fixture AABBs are overlapping or are valid.
void b2World::Update(b2Contact& c)
{
	b2Manifold oldManifold = c.m_manifold;

	const Fixture& fixtureA = m_fixtureBuffer[c.m_fixtureIdxA];
	const Fixture& fixtureB = m_fixtureBuffer[c.m_fixtureIdxB];

	// Re-enable this contact.
	c.AddFlag(b2Contact::e_enabledFlag);

	bool touching = false;
	bool wasTouching = c.HasFlag(b2Contact::e_touchingFlag);
	bool sensor = fixtureA.m_isSensor || fixtureB.m_isSensor;

	Body& bodyA = m_bodyBuffer[fixtureA.m_bodyIdx];
	Body& bodyB = m_bodyBuffer[fixtureB.m_bodyIdx];
	const b2Transform& xfA = bodyA.m_xf;
	const b2Transform& xfB = bodyB.m_xf;

	// Is this contact a sensor?
	if (sensor)
	{
		touching = b2TestOverlap(fixtureA, c.m_indexA, fixtureB, c.m_indexB, xfA, xfB);

		// Sensors don't generate manifolds.
		c.m_manifold.pointCount = 0;
	}
	else
	{
		Evaluate(c, xfA, xfB);
		touching = c.m_manifold.pointCount > 0;

		// Match old contact ids to new contact ids and copy the
		// stored impulses to warm start the solver.
		for (int32 i = 0; i < c.m_manifold.pointCount; ++i)
		{
			b2ManifoldPoint& mp2 = c.m_manifold.points[i];
			mp2.normalImpulse = 0.0f;
			mp2.tangentImpulse = 0.0f;
			b2ContactID id2 = mp2.id;

			for (int32 j = 0; j < oldManifold.pointCount; ++j)
			{
				const b2ManifoldPoint& mp1 = oldManifold.points[j];

				if (mp1.id.key == id2.key)
				{
					mp2.normalImpulse = mp1.normalImpulse;
					mp2.tangentImpulse = mp1.tangentImpulse;
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
		c.AddFlag(b2Contact::e_touchingFlag);
	else
		c.RemFlag(b2Contact::e_touchingFlag);

	if (!wasTouching && touching && m_contactManager.m_contactListener)
		m_contactManager.m_contactListener->BeginContact(c);

	if (wasTouching && !touching && m_contactManager.m_contactListener)
		m_contactManager.m_contactListener->EndContact(c);

	if (!sensor && touching && m_contactManager.m_contactListener)
		m_contactManager.m_contactListener->PreSolve(c, &oldManifold);
}

inline void b2World::ResetFriction(b2Contact& c)
{
	
	c.m_friction = b2MixFriction(m_fixtureBuffer[c.m_fixtureIdxA].m_friction, m_fixtureBuffer[c.m_fixtureIdxB].m_friction);
}

inline void b2World::ResetRestitution(b2Contact& c)
{
	c.m_restitution = b2MixRestitution(m_fixtureBuffer[c.m_fixtureIdxA].m_restitution, m_fixtureBuffer[c.m_fixtureIdxB].m_restitution);
}


bool b2World::b2TestOverlap(const Fixture& fixtureA, int32 indexA,
							const Fixture& fixtureB, int32 indexB,
							const b2Transform& xfA, const b2Transform& xfB)
{
	b2DistanceInput input;
	
	auto& subShapeA = GetShape(fixtureA);
	auto& subShapeB = GetShape(fixtureB);
	input.proxyA.Set(subShapeA, indexA);
	input.proxyB.Set(subShapeB, indexB);
	input.transformA = xfA;
	input.transformB = xfB;
	input.useRadii = true;

	b2SimplexCache cache;
	cache.count = 0;

	b2DistanceOutput output;

	b2Distance(output, cache, input);

	return output.distance < 10.0f * b2_epsilon;
}
