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

#ifndef B2_BODY_H
#define B2_BODY_H

#include <Box2D/Common/b2Math.h>
#include <Box2D/Collision/Shapes/b2Shape.h>
#include <memory>
#include <vector>

class b2Fixture;
class b2BodyMaterial;
class b2Joint;
class b2Contact;
class b2Controller;
class b2World;
struct b2FixtureDef;
struct b2JointEdge;
struct b2ContactEdge;

/// The body type.
/// static: zero mass, zero velocity, may be manually moved
/// kinematic: zero mass, non-zero velocity set by user, moved by solver
/// dynamic: positive mass, non-zero velocity determined by forces, moved by solver
enum b2BodyType
{
	b2_staticBody = 0,
	b2_kinematicBody = 1,
	b2_dynamicBody = 2

	// TODO_ERIN
	//b2_bulletBody,
};


/// The particle type. Can be combined with the | operator.
enum b2BodyFlag
{
	b2_islandBody = 1 << 0,
	b2_awakeBody = 1 << 1,
	b2_autoSleepBody = 1 << 2,	//also called allow sleep
	b2_bulletBody = 1 << 3,
	b2_fixedRotationBody = 1 << 4,
	b2_activeBody = 1 << 5,
	b2_toiBody = 1 << 6,
	b2_breakableBody = 1 << 7,
	b2_inflammableBody = 1 << 8,
	b2_burningBody = 1 << 9,
	b2_wetBody = 1 << 10
};

/// A body definition holds all the data needed to construct a rigid body.
/// You can safely re-use body definitions. Shapes are added to a body after construction.
struct b2BodyDef
{
	/// This constructor sets the body definition default values.
	b2BodyDef()
	{
		position.Set(0.0f, 0.0f, 0.0f);
		angle = 0.0f;
		linearVelocity.Set(0.0f, 0.0f);
		angularVelocity = 0.0f;
		linearDamping = 0.0f;
		angularDamping = 0.0f;
		materialIdx = b2_invalidIndex;
		heat = 15.f;
		health = 1.0f;
		flags = 0;
		allowSleep = true;
		awake = true;
		fixedRotation = false;
		bullet = false;
		type = b2_staticBody;
		active = true;
		gravityScale = 1.0f;
	}

#if LIQUIDFUN_EXTERNAL_LANGUAGE_API
	/// Set position with direct floats.
	void SetPosition(float32 positionX, float32 positionY);
#endif // LIQUIDFUN_EXTERNAL_LANGUAGE_API

	/// The body type: static, kinematic, or dynamic.
	/// Note: if a dynamic body would have zero mass, the mass is set to one.
	b2BodyType type;

	int32 materialIdx;

	/// The world position of the body. Avoid creating bodies at the origin
	/// since this can lead to many overlapping shapes.
	b2Vec3 position;

	/// The world angle of the body in radians.
	float32 angle;

	/// The linear velocity of the body's origin in world co-ordinates.
	b2Vec2 linearVelocity;

	/// The angular velocity of the body.
	float32 angularVelocity;

	/// Linear damping is use to reduce the linear velocity. The damping parameter
	/// can be larger than 1.0f but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	float32 linearDamping;

	/// Angular damping is use to reduce the angular velocity. The damping parameter
	/// can be larger than 1.0f but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	float32 angularDamping;


	/// Set this flag to false if this body should never fall asleep. Note that
	/// this increases CPU usage.
	bool allowSleep;

	/// Is this body initially awake or sleeping?
	bool awake;

	/// Should this body be prevented from rotating? Useful for characters.
	bool fixedRotation;

	/// Is this a fast moving body that should be prevented from tunneling through
	/// other moving bodies? Note that all bodies are prevented from tunneling through
	/// kinematic and static bodies. This setting is only considered on dynamic bodies.
	/// @warning You should use this flag sparingly since it increases processing time.
	bool bullet;

	/// Does this body start out active?
	bool active;

	/// Scale the gravity applied to this body.
	float32 gravityScale;

	float32 heat;
	float32 health;
	uint32 flags;
};


struct Body
{
	int32 m_idx;

	b2BodyType m_type;

	uint32 m_flags;

	int32 m_islandIndex;

	b2Transform m_xf;		// the body origin transform
	b2Transform m_xf0;		// the previous transform for particle simulation
	b2Sweep m_sweep;		// the swept motion for CCD

	b2Vec3 m_linearVelocity;
	float32 m_angularVelocity;

	b2Vec3 m_force;
	float32 m_torque;

	float32 m_mass, m_invMass;

	// Rotational inertia about the center of mass.
	float32 m_I, m_invI;

	float32 m_linearDamping;
	float32 m_angularDamping;
	float32 m_gravityScale;

	float32 m_sleepTime;

	int32 m_materialIdx;
	float32 m_heat;

	float32 m_health;

	void Set(const b2BodyDef def);
	
	/// Set the sleep state of the body. A sleeping body has very
	/// low CPU cost.
	/// @param flag set to true to wake the body, false to put it to sleep.
	void SetAwake(bool flag);
	/// Get the sleeping state of this body.
	/// @return true if the body is awake.
	bool IsAwake() const;

	/// Get the active state of the body.
	bool IsActive() const;

	/// Should this body be treated like a bullet for continuous collision detection?
	void SetBullet(bool flag);
	/// Is this body treated like a bullet for continuous collision detection?
	bool IsBullet() const;

	/// Get the world body origin position.
	/// @return the world position of the body's origin.
	const b2Vec2 GetPosition() const;

	/// Get the angle in radians.
	/// @return the current world rotation angle in radians.
	float32 GetAngle() const;

	/// Get the world position of the center of mass.
	const b2Vec2 GetWorldCenter() const;
	const b2Vec2 GetWorldCenter() const restrict(amp);

	/// Get the local position of the center of mass.
	const b2Vec2 GetLocalCenter() const;
	const b2Vec2 GetLocalCenter() const restrict(amp);


	/// Set the linear velocity of the center of mass.
	/// @param v the new linear velocity of the center of mass.
	void SetLinearVelocity(const b2Vec3& v);

	/// Set the angular velocity.
	/// @param omega the new angular velocity in radians/second.
	void SetAngularVelocity(float32 omega);


	void Advance(float32 t);

	void SynchronizeTransform();




	/// Apply a force at a world point. If the force is not
	/// applied at the center of mass, it will generate a torque and
	/// affect the angular velocity. This wakes up the body.
	/// @param force the world force vector, usually in Newtons (N).
	/// @param point the world position of the point of application.
	/// @param wake also wake up the body
	void ApplyForce(const b2Vec2& force, const b2Vec2& point, bool wake);

	/// Apply a force to the center of mass. This wakes up the body.
	/// @param force the world force vector, usually in Newtons (N).
	/// @param wake also wake up the body
	void ApplyForceToCenter(const b2Vec2& force, bool wake);
	void ApplyImpulseToCenter(const b2Vec2& impulse, bool wake);

	/// Apply a torque. This affects the angular velocity
	/// without affecting the linear velocity of the center of mass.
	/// This wakes up the body.
	/// @param torque about the z-axis (out of the screen), usually in N-m.
	/// @param wake also wake up the body
	void ApplyTorque(float32 torque, bool wake);

	/// Apply an impulse at a point. This immediately modifies the velocity.
	/// It also modifies the angular velocity if the point of application
	/// is not at the center of mass. This wakes up the body.
	/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
	/// @param point the world position of the point of application.
	/// @param wake also wake up the body
	void ApplyLinearImpulse(const b2Vec2& impulse, const b2Vec2& point, bool wake);

	/// Apply an angular impulse.
	/// @param impulse the angular impulse in units of kg*m*m/s
	/// @param wake also wake up the body
	void ApplyAngularImpulse(float32 impulse, bool wake);
	
	/// Get the rotational inertia of the body about the local origin.
	/// @return the rotational inertia, usually in kg-m^2.
	float32 GetInertia() const;
	float32 GetInertia() const restrict (amp);

	/// Get the mass data of the body.
	/// @return a struct containing the mass, inertia and center of the body.
	b2MassData GetMassData() const;

	/// Set the mass properties to override the mass properties of the fixtures.
	/// Note that this changes the center of mass position.
	/// Note that creating or destroying fixtures can also alter the mass.
	/// This function has no effect if the body isn't dynamic.
	/// @param massData the mass properties.
	void SetMassData(const b2MassData& data);

	/// Get the world coordinates of a point given the local coordinates.
	/// @param localPoint a point on the body measured relative the the body's origin.
	/// @return the same point expressed in world coordinates.
	b2Vec2 GetWorldPoint(const b2Vec2& localPoint) const;

	/// Get the world coordinates of a vector given the local coordinates.
	/// @param localVector a vector fixed in the body.
	/// @return the same vector expressed in world coordinates.
	b2Vec2 GetWorldVector(const b2Vec2& localVector) const;

	/// Gets a local point relative to the body's origin given a world point.
	/// @param a point in world coordinates.
	/// @return the corresponding local point relative to the body's origin.
	b2Vec2 GetLocalPoint(const b2Vec2& worldPoint) const;

	/// Gets a local vector given a world vector.
	/// @param a vector in world coordinates.
	/// @return the corresponding local vector.
	b2Vec2 GetLocalVector(const b2Vec2& worldVector) const;

	/// Get the world linear velocity of a world point attached to this body.
	/// @param a point in world coordinates.
	/// @return the world velocity of a point.
	b2Vec2 GetLinearVelocityFromWorldPoint(const b2Vec2& worldPoint) const;

	/// Get the world velocity of a local point.
	/// @param a point in local coordinates.
	/// @return the world velocity of a point.
	b2Vec2 GetLinearVelocityFromLocalPoint(const b2Vec2& localPoint) const;

	/// You can disable sleeping on this body. If you disable sleeping, the
	/// body will be woken.
	void SetSleepingAllowed(bool flag);

	/// Is this body allowed to sleep
	bool IsSleepingAllowed() const;

	/// Does this body have fixed rotation?
	bool IsFixedRotation() const;

	void AddFlags(uint16 flags);
	void RemFlags(uint16 flags);
};

/// A rigid body. These are created via b2World::CreateBody.
class b2Body
{
public:
};

inline void Body::AddFlags(uint16 flags)
{
	m_flags |= flags;
}
inline void Body::RemFlags(uint16 flags)
{
	m_flags &= ~flags;
}

inline const b2Vec2 Body::GetPosition() const
{
	return m_xf.p;
}

inline float32 Body::GetAngle() const
{
	return m_sweep.a;
}

inline const b2Vec2 Body::GetWorldCenter() const
{
	return m_sweep.c;
}
inline const b2Vec2 Body::GetWorldCenter() const restrict(amp)
{
	return m_sweep.c;
}

inline const b2Vec2 Body::GetLocalCenter() const
{
	return m_sweep.localCenter;
}
inline const b2Vec2 Body::GetLocalCenter() const restrict(amp)
{
	return m_sweep.localCenter;
}

inline void Body::SetLinearVelocity(const b2Vec3& v)
{
	if (m_type == b2_staticBody)
		return;
	if (b2Dot(v, v) > 0.0f)
		SetAwake(true);
	m_linearVelocity = v;
}

inline void Body::SetAngularVelocity(float32 w)
{
	if (m_type == b2_staticBody)
		return;
	if (w * w > 0.0f)
		SetAwake(true);
	m_angularVelocity = w;
}

inline float32 Body::GetInertia() const
{
	return m_I + m_mass * b2Dot(m_sweep.localCenter, m_sweep.localCenter);
}
inline float32 Body::GetInertia() const restrict(amp)
{
	return m_I + m_mass * b2Dot(m_sweep.localCenter, m_sweep.localCenter);
}


inline b2MassData Body::GetMassData() const
{
	return b2MassData(
		m_mass,
		m_sweep.localCenter,
		m_I + m_mass * b2Dot(m_sweep.localCenter, m_sweep.localCenter)
	);
}

inline b2Vec2 Body::GetWorldPoint(const b2Vec2& localPoint) const
{
	return b2Mul(m_xf, localPoint);
}

inline b2Vec2 Body::GetWorldVector(const b2Vec2& localVector) const
{
	return b2Mul(m_xf.q, localVector);
}

inline b2Vec2 Body::GetLocalPoint(const b2Vec2& worldPoint) const
{
	return b2MulT(m_xf, worldPoint);
}

inline b2Vec2 Body::GetLocalVector(const b2Vec2& worldVector) const
{
	return b2MulT(m_xf.q, worldVector);
}

inline b2Vec2 Body::GetLinearVelocityFromWorldPoint(const b2Vec2& worldPoint) const
{
	return m_linearVelocity + b2Cross(m_angularVelocity, worldPoint - m_sweep.c);
}

inline b2Vec2 Body::GetLinearVelocityFromLocalPoint(const b2Vec2& localPoint) const
{
	return GetLinearVelocityFromWorldPoint(GetWorldPoint(localPoint));
}

inline void Body::SetBullet(bool flag)
{
	if (flag)
		m_flags |= b2_bulletBody;
	else
		m_flags &= ~b2_bulletBody;
}
inline bool Body::IsBullet() const
{
	return (m_flags & b2_bulletBody) == b2_bulletBody;
}

inline void Body::SetAwake(bool flag)
{
	if (flag)
	{
		if ((m_flags & b2_awakeBody) == 0)
		{
			m_flags |= b2_awakeBody;
			m_sleepTime = 0.0f;
		}
	}
	else
	{
		m_flags &= ~b2_awakeBody;
		m_sleepTime = 0.0f;
		m_linearVelocity.SetZero();
		m_angularVelocity = 0.0f;
		m_force.SetZero();
		m_torque = 0.0f;
	}
}
inline bool Body::IsAwake() const
{
	return (m_flags & b2_awakeBody) == b2_awakeBody;
}

inline bool Body::IsActive() const
{
	return (m_flags & b2_activeBody) == b2_activeBody;
}

inline bool Body::IsFixedRotation() const
{
	return (m_flags & b2_fixedRotationBody) == b2_fixedRotationBody;
}

inline void Body::SetSleepingAllowed(bool flag)
{
	if (flag)
		m_flags |= b2_autoSleepBody;
	else
	{
		m_flags &= ~b2_autoSleepBody;
		SetAwake(true);
	}
}
inline bool Body::IsSleepingAllowed() const
{
	return (m_flags & b2_autoSleepBody) == b2_autoSleepBody;
}

inline void Body::ApplyForce(const b2Vec2& force, const b2Vec2& point, bool wake)
{
	if (m_type != b2_dynamicBody)
		return;

	if (wake && (m_flags & b2_awakeBody) == 0)
		SetAwake(true);

	// Don't accumulate a force if the body is sleeping.
	if (m_flags & b2_awakeBody)
	{
		m_force += force;
		m_torque += b2Cross(point - m_sweep.c, force);
	}
}

inline void Body::ApplyForceToCenter(const b2Vec2& force, bool wake)
{
	if (m_type != b2_dynamicBody)
		return;

	if (wake && (m_flags & b2_awakeBody) == 0)
		SetAwake(true);

	// Don't accumulate a force if the body is sleeping
	if (m_flags & b2_awakeBody)
		m_force += force;
}
inline void Body::ApplyImpulseToCenter(const b2Vec2& impulse, bool wake)
{
	if (m_type != b2_dynamicBody)
		return;

	if (wake && (m_flags & b2_awakeBody) == 0)
		SetAwake(true);

	// Don't accumulate a force if the body is sleeping
	if (m_flags & b2_awakeBody)
		m_linearVelocity += m_invMass * impulse;
}

inline void Body::ApplyTorque(float32 torque, bool wake)
{
	if (m_type != b2_dynamicBody)
		return;

	if (wake && (m_flags & b2_awakeBody) == 0)
		SetAwake(true);

	// Don't accumulate a force if the body is sleeping
	if (m_flags & b2_awakeBody)
		m_torque += torque;
}

inline void Body::ApplyLinearImpulse(const b2Vec2& impulse, const b2Vec2& point, bool wake)
{
	if (m_type != b2_dynamicBody)
		return;

	if (wake && (m_flags & b2_awakeBody) == 0)
		SetAwake(true);

	// Don't accumulate velocity if the body is sleeping
	if (m_flags & b2_awakeBody)
	{
		m_linearVelocity += m_invMass * impulse;
		m_angularVelocity += m_invI * b2Cross(point - m_sweep.c, impulse);
	}
}

inline void Body::ApplyAngularImpulse(float32 impulse, bool wake)
{
	if (m_type != b2_dynamicBody)
		return;

	if (wake && (m_flags & b2_awakeBody) == 0)
		SetAwake(true);

	// Don't accumulate velocity if the body is sleeping
	if (m_flags & b2_awakeBody)
		m_angularVelocity += m_invI * impulse;
}

inline void Body::SynchronizeTransform()
{
	m_xf.q.Set(m_sweep.a);
	m_xf.p = m_sweep.c - b2Mul(m_xf.q, m_sweep.localCenter);
}

inline void Body::Advance(float32 alpha)
{
	// Advance to the new safe time. This doesn't sync the broad-phase.
	m_sweep.Advance(alpha);
	m_sweep.c = m_sweep.c0;
	m_sweep.a = m_sweep.a0;
	m_xf.q.Set(m_sweep.a);
	m_xf.p = m_sweep.c - b2Mul(m_xf.q, m_sweep.localCenter);
}

#if LIQUIDFUN_EXTERNAL_LANGUAGE_API
inline void b2BodyDef::SetPosition(float32 positionX, float32 positionY)
{
	position.Set(positionX, positionY);
}

inline void b2Body::SetTransform(float32 positionX, float32 positionY, float32 angle)
{
	SetTransform(b2Vec2(positionX, positionY), angle);
}
#endif // LIQUIDFUN_EXTERNAL_LANGUAGE_API

#endif
