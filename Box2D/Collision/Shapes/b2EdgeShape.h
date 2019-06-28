/*
* Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

/// A line segment (edge) shape. These can be connected in chains or loops
/// to other edge shapes. The connectivity information is used to ensure
/// correct contact normals.
struct b2EdgeShape : public b2Shape
{
	/// These are the edge vertices
	b2Vec2 m_vertex1, m_vertex2;

	/// Optional adjacent vertices. These are used for smooth collision.
	b2Vec2 m_vertex0, m_vertex3;
	int32 m_hasVertex0, m_hasVertex3;

	b2EdgeShape();

	/// Set this as an isolated edge.
	void Set(const b2Vec2& v1, const b2Vec2& v2);

	/// Implement b2Shape.
	b2Shape* Clone(b2BlockAllocator* allocator) const;

	/// @see b2Shape::GetChildCount
	int32 GetChildCount() const;

	/// @see b2Shape::TestPoint
	bool TestPoint(const b2Transform& transform, const b2Vec3& p) const;

	// @see b2Shape::ComputeDistance
	void ComputeDistance(const b2Transform& xf, const b2Vec2& p, float32& distance, b2Vec2& normal, int32 childIndex) const;

	/// Implement b2Shape.
	bool RayCast(b2RayCastOutput& output, const b2RayCastInput& input,
				const b2Transform& transform, int32 childIndex) const;
	
	/// @see b2Shape::ComputeAABB
	void ComputeAABB(b2AABB& aabb, const b2Transform& transform, int32 childIndex) const;

	/// @see b2Shape::ComputeMass
	b2MassData ComputeMass(float32 density) const;
};

struct AmpEdgeShape
{
	int32 _vfptr[2];
	b2Shape::Type m_type;
	float32 m_radius;

	/// These are the edge vertices
	b2Vec2 m_vertex1, m_vertex2;

	/// Optional adjacent vertices. These are used for smooth collision.
	b2Vec2 m_vertex0, m_vertex3;
	int32 m_hasVertex0, m_hasVertex3;

	AmpEdgeShape() restrict(amp);

	void ComputeDistance(const b2Transform& xf, const b2Vec2& p,
		float32& distance,b2Vec2& normal) const restrict(amp)
	{
		b2Vec2 v1 = b2Mul(xf, m_vertex1);
		b2Vec2 v2 = b2Mul(xf, m_vertex2);

		b2Vec2 d = p - v1;
		b2Vec2 s = v2 - v1;
		float32 ds = b2Dot(d, s);
		if (ds > 0)
		{
			float32 s2 = b2Dot(s, s);
			if (ds > s2)
				d = p - v2;
			else
				d -= ds / s2 * s;
		}

		float32 d1 = d.Length();
		distance = d1;
		normal = d1 > 0 ? 1 / d1 * d : b2Vec2(0, 0);
	}

	bool RayCast(b2RayCastOutput& output, const b2RayCastInput& input,
		const b2Transform& xf) const restrict(amp)
	{
		// Put the ray into the edge's frame of reference.
		b2Vec2 p1 = b2MulT(xf.q, input.p1 - xf.p);
		b2Vec2 p2 = b2MulT(xf.q, input.p2 - xf.p);
		b2Vec2 d = p2 - p1;

		b2Vec2 v1 = m_vertex1;
		b2Vec2 v2 = m_vertex2;
		b2Vec2 e = v2 - v1;
		b2Vec2 normal(e.y, -e.x);
		normal.Normalize();

		// q = p1 + t * d
		// dot(normal, q - v1) = 0
		// dot(normal, p1 - v1) + t * dot(normal, d) = 0
		float32 numerator = b2Dot(normal, v1 - p1);
		float32 denominator = b2Dot(normal, d);

		if (denominator == 0.0f)
			return false;

		float32 t = numerator / denominator;
		if (t < 0.0f || input.maxFraction < t)
			return false;

		b2Vec2 q = p1 + t * d;

		// q = v1 + s * r
		// s = dot(q - v1, r) / dot(r, r)
		b2Vec2 r = v2 - v1;
		float32 rr = b2Dot(r, r);
		if (rr == 0.0f)
			return false;

		float32 s = b2Dot(q - v1, r) / rr;
		if (s < 0.0f || 1.0f < s)
			return false;

		output.fraction = t;
		if (numerator > 0.0f)
			output.normal = -b2Mul(xf.q, normal);
		else
			output.normal = b2Mul(xf.q, normal);
		return true;
	}
};

inline b2EdgeShape::b2EdgeShape()
{
	m_type = b2Shape::e_edge;
	m_radius = b2_polygonRadius;
	m_vertex0.x = 0.0f;
	m_vertex0.y = 0.0f;
	m_vertex3.x = 0.0f;
	m_vertex3.y = 0.0f;
	m_hasVertex0 = false;
	m_hasVertex3 = false;
}
inline AmpEdgeShape::AmpEdgeShape() restrict(amp)
{
	m_type = b2Shape::e_edge;
	m_radius = b2_polygonRadius;
	m_vertex0.x = 0.0f;
	m_vertex0.y = 0.0f;
	m_vertex3.x = 0.0f;
	m_vertex3.y = 0.0f;
	m_hasVertex0 = false;
	m_hasVertex3 = false;
}

#if LIQUIDFUN_EXTERNAL_LANGUAGE_API
inline void b2EdgeShape::Set(float32 vx1,
														 float32 vy1,
														 float32 vx2,
														 float32 vy2) {
	Set(b2Vec2(vx1, vy1), b2Vec2(vx2, vy2));
}
#endif // LIQUIDFUN_EXTERNAL_LANGUAGE_API
