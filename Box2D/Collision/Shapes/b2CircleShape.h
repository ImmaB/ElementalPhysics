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

#include <Box2D/Collision/Shapes/b2Shape.h>

struct b2CircleShapeDef : public b2Shape::Def
{
	Vec2 p;
};

/// A circle shape.
struct b2CircleShape : public b2Shape
{
	/// Position
	Vec2 m_p;

	void Set(const b2Shape::Def& shapeDef);

	b2CircleShape();

	/// Implement b2Shape.
	b2Shape* Clone(b2BlockAllocator* allocator) const;

	/// @see b2Shape::GetChildCount
	int32 GetChildCount() const;

	/// Implement b2Shape.
	bool TestPoint(const b2Transform& transform, const Vec3& p) const;

	// @see b2Shape::ComputeDistance
	void ComputeDistance(const b2Transform& xf, const Vec2& p, float32& distance, Vec2& normal, int32 childIndex) const;
	
	/// Implement b2Shape.
	bool RayCast(b2RayCastOutput& output, const b2RayCastInput& input,
				const b2Transform& transform, int32 childIndex) const;

	/// @see b2Shape::ComputeAABB
	void ComputeAABB(b2AABB& aabb, const b2Transform& transform, int32 childIndex) const;

	/// @see b2Shape::ComputeMass
	b2MassData ComputeMass(float32 density, float32 surfaceThickness, float32 massMult) const;

	/// Get the supporting vertex index in the given direction.
	int32 GetSupport(const Vec2& d) const;

	/// Get the supporting vertex in the given direction.
	const Vec2& GetSupportVertex(const Vec2& d) const;

	/// Get the vertex count.
	int32 GetVertexCount() const { return 1; }

	/// Get a vertex by index. Used by b2Distance.
	const Vec2& GetVertex(int32 index) const;
};

struct AmpCircleShape
{
	int32 _vfptr[2];
	b2Shape::Type m_type;
	float32 m_radius;
	float32 m_zPos;
	float32 m_height;
	float32 m_area;
	int32 _placeholder;

	Vec2 m_p;

	bool FindCollision(const b2Transform& xf, const Vec3& p,
		float32& distance, Vec3& normal, float32 maxDist) const restrict(amp)
	{
		if (FindZCollision(xf, p.z, distance, normal, maxDist))
		{
			const Vec2 center = xf.p + b2Mul(xf.q, m_p);
			const Vec2 d = p - center;
			const float32 dl = d.Length();
			const float32 dist = dl - m_radius;
			if (dist < 0 && distance > -b2_linearSlop)
				return true;
			distance = dist;
			normal = Vec3(1 / dl * d, 0);
			return distance < maxDist;
		}
		return false;
	}

	bool FindZCollision(const b2Transform& xf, float32 z,
		float32& distance, Vec3& normal, float32 maxDist) const restrict(amp)
	{
		const float32 halfHeight = m_height / 2;
		const float32 zRelToCenter = z - (xf.z + m_zPos + halfHeight);
		const float32 absZ = b2Abs(zRelToCenter);
		distance = absZ - halfHeight;
		normal = Vec3(0, 0, zRelToCenter / absZ);
		return distance < maxDist;
	}

	bool RayCast(RayCastOutput& output, const RayCastInput& input,
		const b2Transform& xf) const restrict(amp)
	{

		// TODO if (p1 out, p2 in)
		// add to z normal and dist
		const float32 bot = xf.z + m_zPos;
		const float32 top = bot + m_height;
		if (const float32 p1Diff = input.p1.z - top; p1Diff > 0)
		{
			if (const float32 p2Diff = input.p2.z - top; p2Diff < 0)
			{
				output.normal = Vec3(0, 0, 1);
				output.fraction = p1Diff / (p1Diff - p2Diff);
				return true;
			}
			return false;
		}
		else if (const float32 p1Diff = top - input.p1.z; p1Diff > 0)
		{
			if (const float32 p2Diff = top - input.p2.z; p2Diff < 0)
			{
				output.normal = Vec3(0, 0, -1);
				output.fraction = p1Diff / (p1Diff - p2Diff);
				return true;
			}
			return false;
		}

		Vec2 position = xf.p + b2Mul(xf.q, m_p);
		Vec2 s = input.p1 - position;
		float32 b = b2Dot(s, s) - m_radius * m_radius;

		// Solve quadratic equation.
		Vec2 r = input.p2 - input.p1;
		float32 c = b2Dot(s, r);
		float32 rr = b2Dot(r, r);
		float32 sigma = c * c - rr * b;

		// Check for negative discriminant and short segment.
		if (sigma < 0.0f || rr < b2_epsilon)
			return false;

		// Find the point of intersection of the line with the circle.
		float32 a = -(c + ampSqrt(sigma));

		// Is the intersection point on the segment?
		if (0.0f <= a && a <= input.maxFraction * rr)
		{
			a /= rr;
			output.fraction = a;
			output.normal = Vec3(s + a * r, 0);
			output.normal.Normalize();
			return true;
		}
		return false;
	}

	bool TestPoint(const b2Transform& xf, const Vec3& p) const restrict(amp)
	{
		Vec2 center = xf.p + b2Mul(xf.q, m_p);
		Vec2 d = p - center;
		return b2Dot(d, d) <= m_radius * m_radius;
	}
};

inline b2CircleShape::b2CircleShape()
{
	m_type = b2Shape::e_circle;
	m_radius = 0.0f;
	m_p.SetZero();
}

inline int32 b2CircleShape::GetSupport(const Vec2 &d) const
{
	B2_NOT_USED(d);
	return 0;
}

inline const Vec2& b2CircleShape::GetSupportVertex(const Vec2 &d) const
{
	B2_NOT_USED(d);
	return m_p;
}

inline const Vec2& b2CircleShape::GetVertex(int32 index) const
{
	B2_NOT_USED(index);
	b2Assert(index == 0);
	return m_p;
}
