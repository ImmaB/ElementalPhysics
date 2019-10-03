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

#pragma once

#include <Box2D/Common/b2Math.h>
#include <Box2D/Collision/Shapes/b2Shape.h>
#include <Box2D/Amp/ampAlgorithms.h>
#include <memory>
#include <vector>

class b2Joint;
class b2Contact;
class b2Controller;
class b2World;
struct b2JointEdge;
struct b2ContactEdge;

struct Body
{
	struct Mat
	{
		struct Def
		{
			Def()
			{
				matFlags = 0;
				density = 1.0f;
				friction = 0.1f;
				bounciness = 0.2f;
				stability = 1.0f;
				heatConductivity = 0.0f;
				hotThreshold = 1000.0f;
				coldThreshold = 0.0f;
				ignitionThreshold = 0.0f;
			}

			uint32 matFlags;
			float32 density;
			float32 friction;
			float32 bounciness;
			float32 stability;
			float32 heatConductivity;
			float32 hotThreshold;
			float32 coldThreshold;
			float32 ignitionThreshold;
		};

		enum Flag
		{
			WaterRepellent = 1 << 6,
			Inflammable = 1 << 23,
			Extinguishing = 1 << 24,
			HeatConducting = 1 << 25,
			ElectricityConducting = 1 << 26,

			ChangeWhenCold = 1 << 30,
			ChangeWhenHot = 1 << 31,
		};

		uint32 m_matFlags;
		float32 m_density;
		float32 m_friction;
		float32 m_bounciness;	// same as restitution
		float32 m_stability;
		float32 m_invStability;
		float32 m_heatConductivity;
		float32 m_hotThreshold;
		float32 m_coldThreshold;
		float32 m_ignitionThreshold;

		Mat(const Def& def)
			: m_matFlags(def.matFlags),
			m_density(def.density),
			m_friction(def.friction),
			m_bounciness(def.bounciness),
			m_stability(def.stability), m_invStability(1 / def.stability),
			m_heatConductivity(def.heatConductivity),
			m_hotThreshold(def.hotThreshold),
			m_coldThreshold(def.coldThreshold),
			m_ignitionThreshold(def.ignitionThreshold)
		{}
		~Mat() {}

		bool HasFlag(const Flag flag) const { return m_matFlags & flag; }
		bool HasFlag(const Flag flag) const restrict(amp) { return m_matFlags & flag; }
	};

	enum Type
	{
		Static = 0,		// zero mass, zero velocity, may be manually moved
		Kinematic = 1,	// zero mass, non-zero velocity set by user, moved by solver
		Dynamic = 2		// positive mass, non-zero velocity determined by forces, moved by solver

		// TODO_ERIN
		//b2_bulletBody,
	};

	enum Flag
	{
		Island = 1 << 0,
		Awake = 1 << 1,
		AutoSleep = 1 << 2,	//also called allow sleep
		Bullet = 1 << 3,
		FixedRotation = 1 << 4,
		Active = 1 << 5,
		Toi = 1 << 6,
		Breakable = 1 << 7,
		Burning = 1 << 8,
		Wet = 1 << 9
	};


	/// A body definition holds all the data needed to construct a rigid body.
	/// You can safely re-use body definitions. Shapes are added to a body after construction.
	struct Def
	{
		/// This constructor sets the body definition default values.
		Def()
		{
			transform.p.SetZero();
			transform.q.SetIdentity();
			linearVelocity.Set(0.0f, 0.0f);
			angularVelocity = 0.0f;
			linearDamping = 0.0f;
			angularDamping = 0.0f;
			materialIdx = INVALID_IDX;
			heat = 15.f;
			surfaceHeat = heat;
			health = 1.0f;
			flags = 0;
			allowSleep = true;
			awake = true;
			fixedRotation = false;
			bullet = false;
			type = Type::Static;
			active = true;
			gravityScale = 1.0f;
		}

		/// The body type: static, kinematic, or dynamic.
		/// Note: if a dynamic body would have zero mass, the mass is set to one.
		Body::Type type;

		int32 materialIdx;

		/// The world position of the body. Avoid creating bodies at the origin
		/// since this can lead to many overlapping shapes.
		b2Transform transform;

		/// The linear velocity of the body's origin in world co-ordinates.
		Vec2 linearVelocity;

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
		float32 surfaceHeat;
		float32 health;
		uint32 flags;
	};

	int32 m_idx;

	Body::Type m_type;

	uint32 m_flags;

	int32 m_islandIndex;

	b2Transform m_xf;		// the body origin transform
	b2Transform m_xf0;		// the previous transform for particle simulation
	b2Sweep m_sweep;		// the swept motion for CCD

	Vec3 m_linearVelocity;
	float32 m_angularVelocity;

	Vec3 m_force;
	float32 m_torque;

	float32 m_mass, m_invMass;

	// Rotational inertia about the center of mass.
	float32 m_I, m_invI;

	float32 m_linearDamping;
	float32 m_angularDamping;
	float32 m_gravityScale;

	float32 m_sleepTime;

	int32 m_matIdx;
	float32 m_heat;

	float32 m_health;

	float32 m_surfaceMass;
	float32 m_surfaceInvMass;
	float32 m_surfaceHeat;

	void Set(const Body::Def& def);
	
	/// Set the sleep state of the body. A sleeping body has very
	/// low CPU cost.
	/// @param flag set to true to wake the body, false to put it to sleep.
	void SetAwake(bool flag);
	void SetAwake(bool flag) restrict(amp);
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
	const Vec2 GetPosition() const;

	/// Get the angle in radians.
	/// @return the current world rotation angle in radians.
	float32 GetAngle() const;

	/// Get the world position of the center of mass.
	const Vec2 GetWorldCenter() const;
	const Vec2 GetWorldCenter() const restrict(amp);

	/// Get the local position of the center of mass.
	const Vec2 GetLocalCenter() const;
	const Vec2 GetLocalCenter() const restrict(amp);


	/// Set the linear velocity of the center of mass.
	/// @param v the new linear velocity of the center of mass.
	void SetLinearVelocity(const Vec3& v);

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
	void ApplyForce(const Vec2& force, const Vec2& point, bool wake);

	/// Apply a force to the center of mass. This wakes up the body.
	/// @param force the world force vector, usually in Newtons (N).
	/// @param wake also wake up the body
	void ApplyForceToCenter(const Vec2& force, bool wake);
	void ApplyImpulseToCenter(const Vec2& impulse, bool wake);

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
	void ApplyLinearImpulse(const Vec2& impulse, const Vec2& point, bool wake);
	void ApplyLinearImpulse(const Vec2& impulse, const Vec2& point, bool wake) restrict(amp);

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
	Vec2 GetWorldPoint(const Vec2& localPoint) const;

	/// Get the world coordinates of a vector given the local coordinates.
	/// @param localVector a vector fixed in the body.
	/// @return the same vector expressed in world coordinates.
	Vec2 GetWorldVector(const Vec2& localVector) const;

	/// Gets a local point relative to the body's origin given a world point.
	/// @param a point in world coordinates.
	/// @return the corresponding local point relative to the body's origin.
	Vec2 GetLocalPoint(const Vec2& worldPoint) const;

	/// Gets a local vector given a world vector.
	/// @param a vector in world coordinates.
	/// @return the corresponding local vector.
	Vec2 GetLocalVector(const Vec2& worldVector) const;

	/// Get the world linear velocity of a world point attached to this body.
	/// @param a point in world coordinates.
	/// @return the world velocity of a point.
	Vec2 GetLinearVelocityFromWorldPoint(const Vec2& worldPoint) const;
	Vec2 GetLinearVelocityFromWorldPoint(const Vec2& worldPoint) const restrict(amp);

	/// Get the world velocity of a local point.
	/// @param a point in local coordinates.
	/// @return the world velocity of a point.
	Vec2 GetLinearVelocityFromLocalPoint(const Vec2& localPoint) const;

	/// You can disable sleeping on this body. If you disable sleeping, the
	/// body will be woken.
	void SetSleepingAllowed(bool flag);

	/// Is this body allowed to sleep
	bool IsSleepingAllowed() const;

	/// Does this body have fixed rotation?
	bool IsFixedRotation() const;
	bool IsFixedRotation() const restrict(amp);

	inline bool IsType(Body::Type t) const { return m_type == t; }

	inline void AddFlag(Flag flags) { m_flags |= flags; }
	inline void AddFlag(Flag flags) restrict(amp) { m_flags |= flags; }
	inline void RemFlag(Flag flags) { m_flags &= ~flags; }
	inline void RemFlag(Flag flags) restrict(amp) { m_flags &= ~flags; }

	inline bool HasFlag(Flag flag) const { return m_flags & flag; }
	inline bool HasFlag(Flag flag) const restrict(amp) { return m_flags & flag; }

	inline bool IsAwake() const restrict(amp) { return HasFlag(Flag::Awake); }

	inline bool atomicAddFlag(Flag flag) restrict(amp)
	{
		if (amp::atomicAddFlag(m_flags, flag)) return true;
		return false;
	}
};

/// A rigid body. These are created via b2World::CreateBody.
class b2Body
{
public:
};

inline const Vec2 Body::GetPosition() const
{
	return m_xf.p;
}

inline float32 Body::GetAngle() const
{
	return m_sweep.a;
}

inline const Vec2 Body::GetWorldCenter() const
{
	return m_sweep.c;
}
inline const Vec2 Body::GetWorldCenter() const restrict(amp)
{
	return m_sweep.c;
}

inline const Vec2 Body::GetLocalCenter() const
{
	return m_sweep.localCenter;
}
inline const Vec2 Body::GetLocalCenter() const restrict(amp)
{
	return m_sweep.localCenter;
}

inline void Body::SetLinearVelocity(const Vec3& v)
{
	if (m_type == Type::Static)
		return;
	if (b2Dot(v, v) > 0.0f)
		SetAwake(true);
	m_linearVelocity = v;
}

inline void Body::SetAngularVelocity(float32 w)
{
	if (m_type == Type::Static)
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
		m_mass, m_surfaceMass,
		m_sweep.localCenter,
		m_I + m_mass * b2Dot(m_sweep.localCenter, m_sweep.localCenter)
	);
}

inline Vec2 Body::GetWorldPoint(const Vec2& localPoint) const
{
	return b2Mul(m_xf, localPoint);
}

inline Vec2 Body::GetWorldVector(const Vec2& localVector) const
{
	return b2Mul(m_xf.q, localVector);
}

inline Vec2 Body::GetLocalPoint(const Vec2& worldPoint) const
{
	return b2MulT(m_xf, worldPoint);
}

inline Vec2 Body::GetLocalVector(const Vec2& worldVector) const
{
	return b2MulT(m_xf.q, worldVector);
}

inline Vec2 Body::GetLinearVelocityFromWorldPoint(const Vec2& worldPoint) const
{
	return m_linearVelocity + b2Cross(m_angularVelocity, worldPoint - m_sweep.c);
}
inline Vec2 Body::GetLinearVelocityFromWorldPoint(const Vec2& worldPoint) const restrict(amp)
{
	return m_linearVelocity + b2Cross(m_angularVelocity, worldPoint - m_sweep.c);
}

inline Vec2 Body::GetLinearVelocityFromLocalPoint(const Vec2& localPoint) const
{
	return GetLinearVelocityFromWorldPoint(GetWorldPoint(localPoint));
}

inline void Body::SetBullet(bool flag)
{
	if (flag) AddFlag(Flag::Bullet); else RemFlag(Flag::Bullet);
}
inline bool Body::IsBullet() const
{
	return (m_flags & Bullet);
}

inline void Body::SetAwake(bool flag)
{
	if (flag)
	{
		if (!IsAwake())
		{
			m_flags |= Awake;
			m_sleepTime = 0.0f;
		}
	}
	else
	{
		m_flags &= ~Awake;
		m_sleepTime = 0.0f;
		m_linearVelocity.SetZero();
		m_angularVelocity = 0.0f;
		m_force.SetZero();
		m_torque = 0.0f;
	}
}
inline void Body::SetAwake(bool flag) restrict(amp)
{
	if (flag)
	{
		if ((m_flags & Awake) == 0)
		{
			m_flags |= Awake;
			m_sleepTime = 0.0f;
		}
	}
	else
	{
		m_flags &= ~Awake;
		m_sleepTime = 0.0f;
		m_linearVelocity.SetZero();
		m_angularVelocity = 0.0f;
		m_force.SetZero();
		m_torque = 0.0f;
	}
}
inline bool Body::IsAwake() const
{
	return HasFlag(Flag::Awake);
}

inline bool Body::IsActive() const
{
	return HasFlag(Flag::Active);
}

inline bool Body::IsFixedRotation() const
{
	return HasFlag(Flag::FixedRotation);
}
inline bool Body::IsFixedRotation() const restrict(amp)
{
	return HasFlag(Flag::FixedRotation);
}

inline void Body::SetSleepingAllowed(bool flag)
{
	if (flag)
		AddFlag(Flag::AutoSleep);
	else
	{
		RemFlag(Flag::AutoSleep);
		SetAwake(true);
	}
}
inline bool Body::IsSleepingAllowed() const
{
	return HasFlag(AutoSleep);
}

inline void Body::ApplyForce(const Vec2& force, const Vec2& point, bool wake)
{
	if (m_type != Type::Dynamic)
		return;

	if (wake && !IsAwake())
		SetAwake(true);

	// Don't accumulate a force if the body is sleeping.
	if (IsAwake())
	{
		m_force += force;
		m_torque += b2Cross(point - m_sweep.c, force);
	}
}

inline void Body::ApplyForceToCenter(const Vec2& force, bool wake)
{
	if (m_type != Type::Dynamic)
		return;

	if (wake && !IsAwake())
		SetAwake(true);

	// Don't accumulate a force if the body is sleeping
	if (IsAwake())
		m_force += force;
}
inline void Body::ApplyImpulseToCenter(const Vec2& impulse, bool wake)
{
	if (m_type != Type::Dynamic)
		return;

	if (wake && !IsAwake())
		SetAwake(true);

	// Don't accumulate a force if the body is sleeping
	if (IsAwake())
		m_linearVelocity += m_invMass * impulse;
}

inline void Body::ApplyTorque(float32 torque, bool wake)
{
	if (m_type != Type::Dynamic)
		return;

	if (wake && !IsAwake())
		SetAwake(true);

	// Don't accumulate a force if the body is sleeping
	if (IsAwake())
		m_torque += torque;
}

inline void Body::ApplyLinearImpulse(const Vec2& impulse, const Vec2& point, bool wake)
{
	if (m_type != Type::Dynamic)
		return;

	if (wake && !IsAwake())
		SetAwake(true);

	// Don't accumulate velocity if the body is sleeping
	if (IsAwake())
	{
		m_linearVelocity += m_invMass * impulse;
		if (!IsFixedRotation())
			m_angularVelocity += m_invI * b2Cross(point - m_sweep.c, impulse);
	}
}
inline void Body::ApplyLinearImpulse(const Vec2& impulse, const Vec2& point, bool wake) restrict(amp)
{
	if (m_type != Type::Dynamic)
		return;

	if (wake && !IsAwake())
		SetAwake(true);

	// Don't accumulate velocity if the body is sleeping
	if (IsAwake())
	{
		amp::atomicAdd(m_linearVelocity, m_invMass * impulse);
		if (!IsFixedRotation())
			amp::atomicAdd(m_angularVelocity, m_invI * b2Cross(point - m_sweep.c, impulse));
	}
}

inline void Body::ApplyAngularImpulse(float32 impulse, bool wake)
{
	if (m_type != Type::Dynamic)
		return;

	if (wake && !IsAwake())
		SetAwake(true);

	// Don't accumulate velocity if the body is sleeping
	if (IsAwake())
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
