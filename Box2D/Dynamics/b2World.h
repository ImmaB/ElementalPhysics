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

#ifndef B2_WORLD_H
#define B2_WORLD_H

#include <Box2D/Common/b2Math.h>
#include <Box2D/Common/b2BlockAllocator.h>
#include <Box2D/Common/b2StackAllocator.h>
#include <Box2D/Dynamics/b2ContactManager.h>
#include <Box2D/Dynamics/b2WorldCallbacks.h>
#include <Box2D/Dynamics/b2TimeStep.h>
#include <Box2D/Particle/b2ParticleSystem.h>
#include <Box2D/Particle/b2Material.h>

struct b2AABB;
struct b2Color;
struct b2JointDef;
struct b2Filter;
class b2Draw;
class b2Joint;
class b2ParticleGroup;
class b2Contact;

/// The world class manages all physics entities, dynamic simulation,
/// and asynchronous queries. The world also contains efficient memory
/// management facilities.
class b2World
{
public:
	/// Construct a world object.
	/// @param gravity the world gravity vector.
	b2World(const b2Vec2& gravity, float32 lowerHeightLimit, float32 upperHeightLimit, bool deleteOutsideLimit);

	/// Destruct the world. All physics entities are destroyed and all heap memory is released.
	~b2World();

	/// Damping is used to reduce the velocity of particles. The damping
	/// parameter can be larger than 1.0f but the damping effect becomes
	/// sensitive to the time step when the damping parameter is large.
	void SetDamping(float32 damping);

	/// Get damping for particles
	float32 GetDamping() const;

	/// Register a destruction listener. The listener is owned by you and must
	/// remain in scope.
	void SetDestructionListener(b2DestructionListener* listener);

	/// Register a contact filter to provide specific control over collision.
	/// Otherwise the default filter is used (b2_defaultFilter). The listener is
	/// owned by you and must remain in scope.
	void SetContactFilter(b2ContactFilter* filter);

	/// Register a contact event listener. The listener is owned by you and must
	/// remain in scope.
	void SetContactListener(b2ContactListener* listener);

	/// Register a routine for debug drawing. The debug draw functions are called
	/// inside with b2World::DrawDebugData method. The debug draw object is owned
	/// by you and must remain in scope.
	void SetDebugDraw(b2Draw* debugDraw);

	/// Create a rigid body given a definition. No reference to the definition
	/// is retained.
	/// @warning This function is locked during callbacks.
	int32 CreateBody(const b2BodyDef& def);

	/// Destroy a rigid body.
	/// This function is locked during callbacks.
	/// @warning This automatically deletes all associated shapes and joints.
	/// @warning This function is locked during callbacks.
	void DestroyBody(int32 idx);

	/// Create a joint to constrain bodies together. No reference to the definition
	/// is retained. This may cause the connected bodies to cease colliding.
	/// @warning This function is locked during callbacks.
	b2Joint* CreateJoint(const b2JointDef* def);

	/// Destroy a joint. This may cause the connected bodies to begin colliding.
	/// @warning This function is locked during callbacks.
	void DestroyJoint(b2Joint* joint);

	/// Create a particle system given a definition. No reference to the
	/// definition is retained.
	/// @warning This function is locked during callbacks.
	b2ParticleSystem* CreateParticleSystem(const b2ParticleSystemDef& def);

	/// Destroy a particle system.
	/// @warning This function is locked during callbacks.
	void DestroyParticleSystem(b2ParticleSystem* p);

	void SetStepParams(	float32 timeStep,
						int32 velocityIterations,
						int32 positionIterations,
						int32 particleIterations);

	/// Take a time step. This performs collision detection, integration,
	/// and constraint solution.
	/// For the numerical stability of particles, minimize the following
	/// dimensionless gravity acceleration:
	///     gravity / particleRadius * (timeStep / particleIterations)^2
	/// b2CalculateParticleIterations() or
	/// CalculateReasonableParticleIterations() help to determine the optimal
	/// particleIterations.
	/// @param timeStep the amount of time to simulate, this should not vary.
	/// @param velocityIterations for the velocity constraint solver.
	/// @param positionIterations for the position constraint solver.
	/// @param particleIterations for the particle simulation.
	void StepPreParticle();
	void StepPostParticle();

	/// Recommend a value to be used in `Step` for `particleIterations`.
	/// This calculation is necessarily a simplification and should only be
	/// used as a starting point. Please see "Particle Iterations" in the
	/// Programmer's Guide for details.
	/// @param timeStep is the value to be passed into `Step`.
	int CalculateReasonableParticleIterations(float32 timeStep) const;

	/// Manually clear the force buffer on all bodies. By default, forces are cleared automatically
	/// after each call to Step. The default behavior is modified by calling SetAutoClearForces.
	/// The purpose of this function is to support sub-stepping. Sub-stepping is often used to maintain
	/// a fixed sized time step under a variable frame-rate.
	/// When you perform sub-stepping you will disable auto clearing of forces and instead call
	/// ClearForces after all sub-steps are complete in one pass of your game loop.
	/// @see SetAutoClearForces
	void ClearForces();

	/// Call this to draw shapes and other debug draw data. This is intentionally non-const.
	void DrawDebugData();

	/// Query the world for all fixtures that potentially overlap the
	/// provided AABB.
	/// @param callback a user implemented callback class.
	/// @param aabb the query box.
	void QueryAABB(b2QueryCallback* callback, const b2AABB& aabb) const;
	void AmpQueryAABB(b2QueryCallback* callback, const b2AABB& aabb) const;

	/// Query the world for all fixtures that potentially overlap the
	/// provided shape's AABB. Calls QueryAABB internally.
	/// @param callback a user implemented callback class.
	/// @param shape the query shape
	/// @param xf the transform of the AABB
	void QueryShapeAABB(b2QueryCallback* callback, const b2Shape& shape,
	                    const b2Transform& xf) const;

	/// Ray-cast the world for all fixtures in the path of the ray. Your callback
	/// controls whether you get the closest point, any point, or n-points.
	/// The ray-cast ignores shapes that contain the starting point.
	/// @param callback a user implemented callback class.
	/// @param point1 the ray starting point
	/// @param point2 the ray ending point
	void RayCast(b2RayCastCallback& callback, const b2Vec2& point1, const b2Vec2& point2) const;

	std::vector<Body> GetBodyBuffer();
	const std::vector<Body> GetBodyBuffer() const;
	Body& GetBody(int32 idx);

	std::vector<Fixture> GetFixtureBuffer();
	const std::vector<Fixture> GetFixtureBuffer() const;
	
	/// Get the world joint list. With the returned joint, use b2Joint::GetNext to get
	/// the next joint in the world list. A NULL joint indicates the end of the list.
	/// @return the head of the world joint list.
	b2Joint* GetJointList();
	const b2Joint* GetJointList() const;

	/// Get the world particle-system list. With the returned body, use
	/// b2ParticleSystem::GetNext to get the next particle-system in the world
	/// list. A NULL particle-system indicates the end of the list.
	/// @return the head of the world particle-system list.
	b2ParticleSystem* GetParticleSystemList();
	const b2ParticleSystem* GetParticleSystemList() const;

	/// Get the world contact list. With the returned contact, use b2Contact::GetNext to get
	/// the next contact in the world list. A NULL contact indicates the end of the list.
	/// @return the head of the world contact list.
	/// @warning contacts are created and destroyed in the middle of a time step.
	/// Use b2ContactListener to avoid missing contacts.
	b2Contact* GetContactList();
	const b2Contact* GetContactList() const;

	/// Enable/disable sleep.
	void SetAllowSleeping(bool flag);
	bool GetAllowSleeping() const { return m_allowSleep; }

	/// Enable/disable warm starting. For testing.
	void SetWarmStarting(bool flag) { m_warmStarting = flag; }
	bool GetWarmStarting() const { return m_warmStarting; }

	/// Enable/disable continuous physics. For testing.
	void SetContinuousPhysics(bool flag) { m_continuousPhysics = flag; }
	bool GetContinuousPhysics() const { return m_continuousPhysics; }

	/// Enable/disable single stepped continuous physics. For testing.
	void SetSubStepping(bool flag) { m_subStepping = flag; }
	bool GetSubStepping() const { return m_subStepping; }

	/// Get the number of broad-phase proxies.
	int32 GetProxyCount() const;

	/// Get the number of bodies.
	int32 GetBodyCount() const;

	/// Get the number of joints.
	int32 GetJointCount() const;

	/// Get the number of contacts (each may have 0 or more contact points).
	int32 GetContactCount() const;

	/// Get the height of the dynamic tree.
	int32 GetTreeHeight() const;

	/// Get the balance of the dynamic tree.
	int32 GetTreeBalance() const;

	/// Get the quality metric of the dynamic tree. The smaller the better.
	/// The minimum is 1.
	float32 GetTreeQuality() const;

	/// Change the global gravity vector.
	void SetGravity(const b2Vec2& gravity);

	/// Get the global gravity vector.
	b2Vec2 GetGravity() const;

	/// Is the world locked (in the middle of a time step).
	bool IsLocked() const;

	/// Set flag to control automatic clearing of forces after each time step.
	void SetAutoClearForces(bool flag);

	/// Get the flag that controls automatic clearing of forces after each time step.
	bool GetAutoClearForces() const;

	/// Shift the world origin. Useful for large worlds.
	/// The body shift formula is: position -= newOrigin
	/// @param newOrigin the new origin with respect to the old origin
	void ShiftOrigin(const b2Vec2& newOrigin);

	/// Get the contact manager for testing.
	const b2ContactManager& GetContactManager() const;

	/// Get the current profile.
	const b2Profile& GetProfile() const;


	vector<b2BodyMaterial> m_bodyMaterials;

	int32 m_allMaterialFlags;

	int32 AddBodyMaterial(b2BodyMaterialDef def);

	bool m_stepComplete;

	/// Get API version.
	const b2Version* GetVersion() const {
		return m_liquidFunVersion;
	}

	/// Get API version string.
	const char* GetVersionString() const {
		return m_liquidFunVersionString;
	}

	template<typename T>
	void RemoveFromBuffer(const int32 idx, vector<T>& buffer, set<int32>& freeIdxs) const;
	template<typename T>
	T& InsertIntoBuffer(vector<T> buf, set<int32> freeIdxs, int32& outIdx) const;
	template<typename T>
	int32 InsertIntoBuffer(T& value, vector<T> buf, set<int32> freeIdxs) const;

	const b2Shape& GetSubShape(const Shape& s) const;

#if LIQUIDFUN_EXTERNAL_LANGUAGE_API
public:
	/// Constructor which takes direct floats.
	b2World(float32 gravityX, float32 gravityY);

	/// Set gravity with direct floats.
	void SetGravity(float32 gravityX, float32 gravityY);
#endif // LIQUIDFUN_EXTERNAL_LANGUAGE_API

private:

	// m_flags
	enum
	{
		e_newFixture	= 0x0001,
		e_locked		= 0x0002,
		e_clearForces	= 0x0004
	};

	friend class b2Body;
	friend class b2Fixture;
	friend class b2ContactManager;
	friend class b2Controller;
	friend class b2ParticleSystem;

	void Solve(const b2TimeStep& step);
	void SolveTOI(const b2TimeStep& step);

	void DrawJoint(b2Joint* joint);
	void DrawShape(Fixture& shape, const b2Transform& xf, const b2Color& color);

	void DrawParticleSystem(const b2ParticleSystem& system);

	b2TimeStep m_step;

	b2BlockAllocator m_blockAllocator;
	b2StackAllocator m_stackAllocator;

	int32 m_flags;

	b2ContactManager m_contactManager;

	b2Joint* m_jointList;
	b2ParticleSystem* m_particleSystemList;

	int32 m_jointCount;

	b2Vec3 m_gravity;
	float32 m_lowerHeightLimit;
	float32 m_upperHeightLimit;
	float32 m_deleteOutsideLimit;
	float32 m_dampingStrength;
	bool m_allowSleep;

	b2DestructionListener* m_destructionListener;
	AFDestructionListener* afDestructionListener;
	b2Draw* m_debugDraw;

	// This is used to compute the time step ratio to
	// support a variable time step.
	float32 m_inv_dt0;

	// These are for debugging the solver.
	bool m_warmStarting;
	bool m_continuousPhysics;
	bool m_subStepping;


	b2Profile m_profile;

	/// Used to reference b2_LiquidFunVersion so that it's not stripped from
	/// the static library.
	const b2Version *m_liquidFunVersion;
	const char *m_liquidFunVersionString;



	void SynchronizeFixtures(const Body& b);

	/// This resets the mass properties to the sum of the mass properties of the fixtures.
	/// This normally does not need to be called unless you called SetMassData to override
	/// the mass and you later want to reset the mass.
	void ResetMassData(Body& b);

	b2Shape& InsertSubShapeIntoBuffer(Shape::Type shapeType, int32& outIdx) const;


public:
	vector<Body>			m_bodyBuffer;
	vector<Fixture>			m_fixtureBuffer;
	vector<vector<b2FixtureProxy>> m_fixtureProxiesBuffer;

	vector<Shape>			m_shapeBuffer;
	vector<b2ChainShape>	m_chainShapeBuffer;
	vector<b2CircleShape>	m_circleShapeBuffer;
	vector<b2EdgeShape>		m_edgeShapeBuffer;
	vector<b2PolygonShape>	m_polygonShapeBuffer;
	vector<b2Vec2>			m_shapePositionBuffer;
	vector<b2Vec2>			m_shapeNormalBuffer;

	set<int32> m_freeBodyIdxs;
	set<int32> m_freeFixtureIdxs;
	set<int32> m_freeShapeIdxs;
	set<int32> m_freeChainShapeIdxs;
	set<int32> m_freeCircleShapeIdxs;
	set<int32> m_freeEdgeShapeIdxs;
	set<int32> m_freePolygonShapeIdxs;
	set<int32> m_freeShapePositionIdxs;
	set<int32> m_freeShapeNormalIdxs;

	// Body

	/// Creates a fixture and attach it to this body. Use this function if you need
	/// to set some fixture parameters, like friction. Otherwise you can create the
	/// fixture directly from a shape.
	/// If the density is non-zero, this function automatically updates the mass of the body.
	/// Contacts are not created until the next time step.
	/// @param def the fixture definition.
	/// @warning This function is locked during callbacks.
	int32 CreateFixture(int32 bodyIdx, b2FixtureDef& def);

	/// Creates a fixture from a shape and attach it to this body.
	/// This is a convenience function. Use b2FixtureDef if you need to set parameters
	/// like friction, restitution, user data, or filtering.
	/// If the density is non-zero, this function automatically updates the mass of the body.
	/// @param shape the shape to be cloned.
	/// @param density the shape density (set to zero for static bodies).
	/// @warning This function is locked during callbacks.
	int32 CreateFixture(int32 bodyIdx, const int32 shape, float32 density);

	void DestroyShape(int32 shapeIdx);

	/// Destroy a fixture. This removes the fixture from the broad-phase and
	/// destroys all contacts associated with this fixture. This will
	/// automatically adjust the mass of the body if the body is dynamic and the
	/// fixture has positive density.
	/// All fixtures attached to a body are implicitly destroyed when the body is destroyed.
	/// @param fixture the fixture to be removed.
	/// @warning This function is locked during callbacks.
	void DestroyFixture(int32 bodyIdx, int32 idx);

	/// Set the position of the body's origin and rotation.
	/// Manipulating a body's transform may cause non-physical behavior.
	/// Note: contacts are updated on the next call to b2World::Step.
	/// @param position the world position of the body's local origin.
	/// @param angle the world rotation in radians.
	void SetTransform(Body& b, const b2Vec2& position, float32 angle);

	/// Set the active state of the body. An inactive body is not
	/// simulated and cannot be collided with or woken up.
	/// If you pass a flag of true, all fixtures will be added to the
	/// broad-phase.
	/// If you pass a flag of false, all fixtures will be removed from
	/// the broad-phase and all contacts will be destroyed.
	/// Fixtures and joints are otherwise unaffected. You may continue
	/// to create/destroy fixtures and joints on inactive bodies.
	/// Fixtures on an inactive body are implicitly inactive and will
	/// not participate in collisions, ray-casts, or queries.
	/// Joints connected to an inactive body are implicitly inactive.
	/// An inactive body is still owned by a b2World object and remains
	/// in the body list.
	void SetActive(Body& b, bool flag);

	/// Set this body to have fixed rotation. This causes the mass
	/// to be reset.
	void SetFixedRotation(Body& b, bool flag);

	// Fixture
	Shape::Type GetType(const Fixture& f) const;

	/// Test a point for containment in this fixture.
	/// @param p a point in world coordinates.
	bool TestPoint(const Fixture& f, const b2Vec2& p) const;

	/// Compute the distance from this fixture.
	/// @param p a point in world coordinates.
	void ComputeDistance(const Fixture& f, const b2Vec2& p, float32& distance, b2Vec2& normal, int32 childIndex) const;

	/// Cast a ray against this shape.
	/// @param output the ray-cast results.
	/// @param input the ray-cast input parameters.
	bool RayCast(const Fixture& f, b2RayCastOutput& output, const b2RayCastInput& input, int32 childIndex) const;

	/// Get the mass data for this fixture. The mass data is based on the density and
	/// the shape. The rotational inertia is about the shape's origin. This operation
	/// may be expensive.
	b2MassData GetMassData(const Fixture& f) const;

	/// Get the fixture's AABB. This AABB may be enlarge and/or stale.
	/// If you need a more accurate AABB, compute it using the shape and
	/// the body transform.
	const b2AABB& GetAABB(const Fixture& f, int32 childIndex) const;

	/// Set if this fixture is a sensor.
	void SetSensor(Fixture& f, bool sensor);

	/// Set the contact filtering data. This will not update contacts until the next time
	/// step when either parent body is active and awake.
	/// This automatically calls Refilter.
	void SetFilterData(Fixture& f, const b2Filter& filter);

	/// Call this if you want to establish collision that was previously disabled by b2ContactFilter::ShouldCollide.
	void Refilter(Fixture& f);

	void Synchronize(Fixture& f, b2BroadPhase& broadPhase, const b2Transform& xf1, const b2Transform& xf2);

	/// These support body activation/deactivation.
	void CreateProxies(Fixture& f, b2BroadPhase& broadPhase, const b2Transform& xf);
	void DestroyProxies(Fixture& f);

	int32 CreateShape(b2ShapeDef& shapeDef);

	void DestroyShape(Fixture& f);



	// Contact

	/// Get the world manifold.
	void GetWorldManifold(b2Contact& c, b2WorldManifold* worldManifold) const;
	b2Contact* CreateContact(Fixture& fixtureA, int32 indexA, Fixture& fixtureB, int32 indexB, b2BlockAllocator* allocator);
	void Destroy(b2Contact& c, b2BlockAllocator* allocator);

	/// Evaluate this contact with your own manifold and transforms.
	void Evaluate(b2Contact& c, b2Manifold& manifold, const b2Transform& xfA, const b2Transform& xfB);

	void Update(b2Contact& c);

	// Collision

	/// Determine if two generic shapes overlap.
	bool b2TestOverlap(const Shape& shapeA, int32 indexA,
					   const Shape& shapeB, int32 indexB,
					   const b2Transform& xfA, const b2Transform& xfB);
};

inline void b2World::SetDamping(float32 damping)
{
	m_dampingStrength = damping;
}

inline float32 b2World::GetDamping() const
{
	return m_dampingStrength;
}

inline std::vector<Body> b2World::GetBodyBuffer()
{
	return m_bodyBuffer;
}
inline const std::vector<Body> b2World::GetBodyBuffer() const
{
	return m_bodyBuffer;
}
inline Body& b2World::GetBody(int32 idx)
{
	return m_bodyBuffer[idx];
}


inline std::vector<Fixture> b2World::GetFixtureBuffer()
{
	return m_fixtureBuffer;
}
inline const std::vector<Fixture> b2World::GetFixtureBuffer() const
{
	return m_fixtureBuffer;
}

inline b2Joint* b2World::GetJointList()
{
	return m_jointList;
}

inline const b2Joint* b2World::GetJointList() const
{
	return m_jointList;
}

inline b2ParticleSystem* b2World::GetParticleSystemList()
{
	return m_particleSystemList;
}

inline const b2ParticleSystem* b2World::GetParticleSystemList() const
{
	return m_particleSystemList;
}

inline b2Contact* b2World::GetContactList()
{
	return m_contactManager.m_contactList;
}

inline const b2Contact* b2World::GetContactList() const
{
	return m_contactManager.m_contactList;
}

inline int32 b2World::GetBodyCount() const
{
	return m_bodyBuffer.size() - m_freeBodyIdxs.size();
}

inline int32 b2World::GetJointCount() const
{
	return m_jointCount;
}

inline int32 b2World::GetContactCount() const
{
	return m_contactManager.m_contactCount;
}

inline void b2World::SetGravity(const b2Vec2& gravity)
{
	m_gravity = gravity;
}

inline b2Vec2 b2World::GetGravity() const
{
	return m_gravity;
}

inline bool b2World::IsLocked() const
{
	return (m_flags & e_locked) == e_locked;
}

inline void b2World::SetAutoClearForces(bool flag)
{
	if (flag)
	{
		m_flags |= e_clearForces;
	}
	else
	{
		m_flags &= ~e_clearForces;
	}
}

/// Get the flag that controls automatic clearing of forces after each time step.
inline bool b2World::GetAutoClearForces() const
{
	return (m_flags & e_clearForces) == e_clearForces;
}

inline const b2ContactManager& b2World::GetContactManager() const
{
	return m_contactManager;
}

inline const b2Profile& b2World::GetProfile() const
{
	return m_profile;
}

#if LIQUIDFUN_EXTERNAL_LANGUAGE_API
inline b2World::b2World(float32 gravityX, float32 gravityY)
{
	Init(b2Vec2(gravityX, gravityY));
}

inline void b2World::SetGravity(float32 gravityX, float32 gravityY)
{
	SetGravity(b2Vec2(gravityX, gravityY));
}
#endif // LIQUIDFUN_EXTERNAL_LANGUAGE_API

#endif
