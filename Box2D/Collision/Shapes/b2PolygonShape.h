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

#ifndef B2_POLYGON_SHAPE_H
#define B2_POLYGON_SHAPE_H

#include <Box2D/Collision/Shapes/b2Shape.h>
#include <array>

//using polyVec2s = std::array<Vec2, b2_maxPolygonVertices>;

struct b2PolygonShapeDef : public b2Shape::Def
{
	Vec2 centroid;
	Vec2 vertices[b2_maxPolygonVertices];
	Vec2 normals[b2_maxPolygonVertices];
	int32 count;

	void SetAsBox(const Vec2& size, const Vec2& center, float32 angle);
};

/// A convex polygon. It is assumed that the interior of the polygon is to
/// the left of each edge.
/// Polygons have a maximum number of vertices equal to b2_maxPolygonVertices.
/// In most cases you should not need many vertices for a convex polygon.
struct b2PolygonShape : public b2Shape
{
	Vec2 m_centroid;
	Vec2 m_vertices[b2_maxPolygonVertices];
	Vec2 m_normals[b2_maxPolygonVertices];
	int32 m_count;

	/// Create a convex hull from the given array of local points.
	/// The count must be in the range [3, b2_maxPolygonVertices].
	/// @warning the points may be re-ordered, even if they form a convex polygon
	/// @warning collinear points are handled but not removed. Collinear points
	/// may lead to poor stacking behavior.
	void Set(const b2Shape::Def& shapeDef);

	b2PolygonShape();

	/// Implement b2Shape.
	b2Shape* Clone(b2BlockAllocator* allocator) const;

	/// @see b2Shape::GetChildCount
	int32 GetChildCount() const;


	/// Build vertices to represent an axis-aligned box centered on the local origin.
	/// @param hx the half-width.
	/// @param hy the half-height.
	void SetAsBox(const Vec2& size);

	/// Build vertices to represent an oriented box.
	/// @param hx the half-width.
	/// @param hy the half-height.
	/// @param center the center of the box in local coordinates.
	/// @param angle the rotation of the box in local coordinates.
	void SetAsBox(const Vec2& size, const Vec2& center, float32 angle);

	/// @see b2Shape::TestPoint
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

	/// Get the vertex count.
	int32 GetVertexCount() const { return m_count; }

	/// Get a vertex by index.
	const Vec2& GetVertex(int32 index) const;

	/// Validate convexity. This is a very time consuming operation.
	/// @returns true if valid
	bool Validate() const;
};

struct AmpPolygonShape
{
	int32 _vfptr[2];
	b2Shape::Type m_type;
	float32 m_radius;
	float32 m_zPos;
	float32 m_height;
	float32 m_area;
	int32 _placeholder;

	Vec2 m_centroid;
	Vec2 m_vertices[b2_maxPolygonVertices];
	Vec2 m_normals[b2_maxPolygonVertices];
	int32 m_count;
	int32 _placeholder2;

	void ComputeDistance(const b2Transform& xf, const Vec2& p,
		float32& distance, Vec2& normal) const restrict(amp)
	{
		const Vec2 pLocal = b2MulT(xf.q, p - xf.p);
		float32 maxDistance = -FLT_MAX;
		Vec2 normalForMaxDistance = pLocal;

		for (int32 i = 0; i < m_count; ++i)
		{
			float32 dot = b2Dot(m_normals[i], pLocal - m_vertices[i]);
			if (dot > maxDistance)
			{
				maxDistance = dot;
				normalForMaxDistance = m_normals[i];
			}
		}

		if (maxDistance > 0)
		{
			Vec2 minDistance = normalForMaxDistance;
			float32 minDistance2 = maxDistance * maxDistance;
			for (int32 i = 0; i < m_count; ++i)
			{
				const Vec2 distance = pLocal - m_vertices[i];
				const float32 distance2 = distance.LengthSquared();
				if (minDistance2 > distance2)
				{
					minDistance = distance;
					minDistance2 = distance2;
				}
			}

			distance = ampSqrt(minDistance2);
			normal = b2Mul(xf.q, minDistance);
			normal.Normalize();
		}
		else
		{
			distance = maxDistance;
			normal = b2Mul(xf.q, normalForMaxDistance);
		}
	}

	bool RayCast(b2RayCastOutput& output, const b2RayCastInput& input,
		const b2Transform& xf) const restrict(amp)
	{
		// Put the ray into the polygon's frame of reference.
		Vec2 p1 = b2MulT(xf.q, input.p1 - xf.p);
		Vec2 p2 = b2MulT(xf.q, input.p2 - xf.p);
		Vec2 d = p2 - p1;

		float32 lower = 0.0f, upper = input.maxFraction;

		int32 index = -1;

		for (int32 i = 0; i < m_count; ++i)
		{
			// p = p1 + a * d
			// dot(normal, p - v) = 0
			// dot(normal, p1 - v) + a * dot(normal, d) = 0
			float32 numerator = b2Dot(m_normals[i], m_vertices[i] - p1);
			float32 denominator = b2Dot(m_normals[i], d);

			if (denominator == 0.0f)
			{
				if (numerator < 0.0f)
					return false;
			}
			else
			{
				// Note: we want this predicate without division:
				// lower < numerator / denominator, where denominator < 0
				// Since denominator < 0, we have to flip the inequality:
				// lower < numerator / denominator <==> denominator * lower > numerator.
				if (denominator < 0.0f && numerator < lower * denominator)
				{
					// Increase lower.
					// The segment enters this half-space.
					lower = numerator / denominator;
					index = i;
				}
				else if (denominator > 0.0f && numerator < upper * denominator)
				{
					// Decrease upper.
					// The segment exits this half-space.
					upper = numerator / denominator;
				}
			}

			// The use of epsilon here causes the assert on lower to trip
			// in some cases. Apparently the use of epsilon was to make edge
			// shapes work, but now those are handled separately.
			//if (upper < lower - b2_epsilon)
			if (upper < lower)
				return false;
		}

		if (index >= 0)
		{
			output.fraction = lower;
			output.normal = b2Mul(xf.q, m_normals[index]);
			return true;
		}
		return false;
	}

	bool TestPoint(const b2Transform& xf, const Vec3& p) const restrict(amp)
	{
		Vec2 pLocal = b2MulT(xf.q, p - xf.p);

		for (int32 i = 0; i < m_count; ++i)
		{
			float32 dot = b2Dot(m_normals[i], pLocal - m_vertices[i]);
			if (dot > 0.0f)
				return false;
		}
		return true;
	}
	bool TestZ(const b2Transform& xf, float32 z) const restrict(amp)
	{
		z -= (m_zPos + xf.z);
		return z >= 0 && z <= m_height;
	}
};

inline b2PolygonShape::b2PolygonShape()
{
	m_type = b2Shape::e_polygon;
	m_radius = b2_polygonRadius;
	m_count = 0;
	m_centroid.SetZero();
}

inline const Vec2& b2PolygonShape::GetVertex(int32 index) const
{
	b2Assert(0 <= index && index < m_count);
	return m_vertices[index];
}

#if LIQUIDFUN_EXTERNAL_LANGUAGE_API
inline void b2PolygonShape::SetCentroid(float32 x, float32 y)
{
	m_centroid.Set(x, y);
}

inline void b2PolygonShape::SetAsBox(float32 hx,
										 float32 hy,
										 float32 centerX,
										 float32 centerY,
										 float32 angle) {
	SetAsBox(hx, hy, Vec2(centerX, centerY), angle);
}
#endif // LIQUIDFUN_EXTERNAL_LANGUAGE_API

#endif
