/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
* Copyright (c) 2014 Google, Inc.
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

#include <Box2D/Dynamics/b2ContactManager.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Dynamics/b2WorldCallbacks.h>
#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <Box2D/Dynamics/b2World.h>

b2ContactFilter b2_defaultFilter;
b2ContactListener b2_defaultListener;

b2ContactManager::b2ContactManager(b2World& world) : m_world(world)
{
	m_contactList = NULL;
	m_contactCount = 0;
	m_contactFilter = &b2_defaultFilter;
	m_contactListener = &b2_defaultListener;
	m_allocator = NULL;
}

void b2ContactManager::Destroy(b2Contact& c)
{
	Body& bodyA = m_world.GetFixtureBody(c.m_fixtureIdxA);
	Body& bodyB = m_world.GetFixtureBody(c.m_fixtureIdxB);

	if (m_contactListener && c.IsTouching())
		m_contactListener->EndContact(c);

	// Remove from the world.
	if (c.m_prev)
		c.m_prev->m_next = c.m_next;

	if (c.m_next)
		c.m_next->m_prev = c.m_prev;

	if (&c == m_contactList)
		m_contactList = c.m_next;

	// Remove from body 1
	if (c.m_nodeA.prev)
		c.m_nodeA.prev->next = c.m_nodeA.next;

	if (c.m_nodeA.next)
		c.m_nodeA.next->prev = c.m_nodeA.prev;

	
	if (&c.m_nodeA == m_world.m_bodyContactListBuffer[bodyA.m_idx])
		m_world.m_bodyContactListBuffer[bodyA.m_idx] = c.m_nodeA.next;

	// Remove from body 2
	if (c.m_nodeB.prev)
		c.m_nodeB.prev->next = c.m_nodeB.next;

	if (c.m_nodeB.next)
		c.m_nodeB.next->prev = c.m_nodeB.prev;

	if (&c.m_nodeB == m_world.m_bodyContactListBuffer[bodyB.m_idx])
		m_world.m_bodyContactListBuffer[bodyB.m_idx] = c.m_nodeB.next;

	// Call the factory.
	m_world.Destroy(c, m_allocator);
	--m_contactCount;
}

// This is the top level collision call for the time step. Here
// all the narrow phase collision is processed for the world
// contact list.
void b2ContactManager::Collide()
{
	// Update awake contacts.
	b2Contact* c = m_contactList;
	while (c)
	{
		Fixture& fixtureA = m_world.GetFixture(c->m_fixtureIdxA);
		Fixture& fixtureB = m_world.GetFixture(c->m_fixtureIdxB);
		int32 indexA = c->GetChildIndexA();
		int32 indexB = c->GetChildIndexB();
		const Body& bodyA = m_world.m_bodyBuffer[fixtureA.m_bodyIdx];
		const Body& bodyB = m_world.m_bodyBuffer[fixtureB.m_bodyIdx];
		 
		// Is this contact flagged for filtering?
		if (c->m_flags & b2Contact::e_filterFlag)
		{
			// Should these bodies collide?
			if (!m_world.ShouldCollide(bodyB, bodyA))
			{
				b2Contact* cNuke = c;
				c = cNuke->GetNext();
				Destroy(*cNuke);
				continue;
			}

			// Check user filtering.
			if (m_contactFilter && !m_contactFilter->ShouldCollide(fixtureA, fixtureB))
			{
				b2Contact* cNuke = c;
				c = cNuke->GetNext();
				Destroy(*cNuke);
				continue;
			}

			// Clear the filtering flag.
			c->m_flags &= ~b2Contact::e_filterFlag;
		}

		bool activeA = bodyA.IsAwake() && !bodyA.IsType(Body::Type::Static);
		bool activeB = bodyB.IsAwake() && !bodyB.IsType(Body::Type::Static);

		// At least one body must be awake and it must be dynamic or kinematic.
		if (!activeA && !activeB)
		{
			c = c->GetNext();
			continue;
		}

		
		int32 proxyIdA = m_world.m_fixtureProxiesBuffer[fixtureA.m_idx][indexA].proxyId;
		int32 proxyIdB = m_world.m_fixtureProxiesBuffer[fixtureB.m_idx][indexB].proxyId;
		bool overlap = m_broadPhase.TestOverlap(proxyIdA, proxyIdB);

		// Here we destroy contacts that cease to overlap in the broad-phase.
		if (!overlap)
		{
			b2Contact* cNuke = c;
			c = cNuke->GetNext();
			Destroy(*cNuke);
			continue;
		}

		// The contact persists.
		m_world.Update(*c);
		c = c->GetNext();
	}
}

void b2ContactManager::FindNewContacts()
{
	m_broadPhase.UpdatePairs([=](b2FixtureProxy* proxyA, b2FixtureProxy* proxyB)
	{
		int32 fixtureAIdx = proxyA->fixtureIdx;
		int32 fixtureBIdx = proxyB->fixtureIdx;
		Fixture& fixtureA = m_world.m_fixtureBuffer[fixtureAIdx];
		Fixture& fixtureB = m_world.m_fixtureBuffer[fixtureBIdx];

		int32 bodyAIdx = fixtureA.m_bodyIdx;
		int32 bodyBIdx = fixtureB.m_bodyIdx;

		// Are the fixtures on the same body?
		if (bodyAIdx == bodyBIdx)
			return;

		int32 indexA = proxyA->childIndex;
		int32 indexB = proxyB->childIndex;

		// TODO_ERIN use a hash table to remove a potential bottleneck when both
		// bodies have a lot of contacts.
		// Does a contact already exist?
		b2ContactEdge* edge = m_world.m_bodyContactListBuffer[bodyBIdx];
		while (edge)
		{
			if (edge->otherIdx == bodyAIdx)
			{
				int32 fA = edge->contact->m_fixtureIdxA;
				int32 fB = edge->contact->m_fixtureIdxB;
				int32 iA = edge->contact->GetChildIndexA();
				int32 iB = edge->contact->GetChildIndexB();

				// A contact already exists.
				if (fA == fixtureAIdx && fB == fixtureBIdx && iA == indexA && iB == indexB)
					return;
				if (fA == fixtureBIdx && fB == fixtureAIdx && iA == indexB && iB == indexA)
					return;
			}

			edge = edge->next;
		}

		// Does a joint override collision? Is at least one body dynamic?
		if (!m_world.ShouldBodiesCollide(bodyAIdx, bodyBIdx))
			return;

		// Check user filtering.
		if (m_contactFilter && !m_contactFilter->ShouldCollide(fixtureA, fixtureB))
			return;

		// Call the factory.
		b2Contact* c = m_world.CreateContact(fixtureA, indexA, fixtureB, indexB, m_allocator);
		if (c == nullptr)
			return;

		// Contact creation may swap fixtures.
		const Fixture& fixtureA2 = m_world.GetFixture(c->m_fixtureIdxA);
		const Fixture& fixtureB2 = m_world.GetFixture(c->m_fixtureIdxB);

		bodyAIdx = fixtureA2.m_bodyIdx;
		bodyBIdx = fixtureB2.m_bodyIdx;

		// Insert into the world.
		c->m_prev = nullptr;
		c->m_next = m_contactList;
		if (m_contactList != nullptr)
			m_contactList->m_prev = c;
		m_contactList = c;

		// Connect to island graph.

		// Connect to body A
		c->m_nodeA.contact = c;
		c->m_nodeA.otherIdx = bodyBIdx;

		b2ContactEdge* bodyAContactList = m_world.m_bodyContactListBuffer[bodyAIdx];
		c->m_nodeA.prev = nullptr;
		c->m_nodeA.next = bodyAContactList;
		if (bodyAContactList != nullptr)
			bodyAContactList->prev = &c->m_nodeA;
		m_world.m_bodyContactListBuffer[bodyAIdx] = &c->m_nodeA;

		// Connect to body B
		c->m_nodeB.contact = c;
		c->m_nodeB.otherIdx = bodyAIdx;

		b2ContactEdge* bodyBContactList = m_world.m_bodyContactListBuffer[bodyBIdx];
		c->m_nodeB.prev = nullptr;
		c->m_nodeB.next = bodyBContactList;
		if (bodyBContactList != nullptr)
			bodyBContactList->prev = &c->m_nodeB;
		m_world.m_bodyContactListBuffer[bodyBIdx] = &c->m_nodeB;

		// Wake up the bodies
		if (!fixtureA2.m_isSensor && !fixtureB2.m_isSensor)
		{
			m_world.m_bodyBuffer[bodyAIdx].SetAwake(true);
			m_world.m_bodyBuffer[bodyBIdx].SetAwake(true);
		}

		++m_contactCount;
	});
}
