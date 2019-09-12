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

#include <Box2D/Common/b2BlockAllocator.h>
#include <Box2D/Common/b2Math.h>
#include <Box2D/Collision/b2Collision.h>

/// This holds the mass data computed for a shape.
struct b2MassData
{
	/// The mass of the shape, usually in kilograms.
	float32 mass;
	float32 surfaceMass;

	/// The position of the shape's centroid relative to the shape's origin.
	b2Vec2 center;

	/// The rotational inertia of the shape about the local origin.
	float32 I;


	b2MassData(const float32 mass, const float32 surfaceMass, const b2Vec2& center, const float32 I)
		: mass(mass), surfaceMass(surfaceMass), center(center), I(I) {};
};


/// A shape is used for collision detection. You can create a shape however you like.
/// Shapes used for simulation in b2World are created automatically when a b2Fixture
/// is created. Shapes may encapsulate a one or more child shapes.
struct b2Shape
{
	enum Type : int32
	{
		e_circle = 0,
		e_edge = 1,
		e_polygon = 2,
		e_chain = 3,
		e_typeCount = 4
	};
	struct Def
	{
		b2Shape::Type type;
		float32 radius;
		float32 zPos;
		float32 height;
	};

	Type m_type = Type::e_typeCount;
	float32 m_radius = 0;
	float32 m_zPos = 0;
	float32 m_height = 0;
	float32 m_area = 0;

	virtual void Set(const Def& shapeDef) {};

	virtual ~b2Shape() {}

	/// Clone the concrete shape using the provided allocator.
	virtual b2Shape* Clone(b2BlockAllocator* allocator) const = 0;

	/// Get the number of child primitives.
	virtual int32 GetChildCount() const = 0;

	/// Test a point for containment in this shape. This only works for convex shapes.
	/// @param xf the shape world transform.
	/// @param p a point in world coordinates.
	virtual bool TestPoint(const b2Transform& xf, const b2Vec3& p) const = 0;


	/// Compute the distance from the current shape to the specified point. This only works for convex shapes.
	/// @param xf the shape world transform.
	/// @param p a point in world coordinates.
	/// @param distance returns the distance from the current shape.
	/// @param normal returns the direction in which the distance increases.
	virtual void ComputeDistance(const b2Transform& xf, const b2Vec2& p, float32& distance, b2Vec2& normal, int32 childIndex) const= 0;

	/// Cast a ray against a child shape.
	/// @param output the ray-cast results.
	/// @param input the ray-cast input parameters.
	/// @param transform the transform to be applied to the shape.
	/// @param childIndex the child shape index
	virtual bool RayCast(b2RayCastOutput& output, const b2RayCastInput& input,
						const b2Transform& transform, int32 childIndex) const = 0;

	/// Given a transform, compute the associated axis aligned bounding box for a child shape.
	/// @param aabb returns the axis aligned box.
	/// @param xf the world transform of the shape.
	/// @param childIndex the child shape
	virtual void ComputeAABB(b2AABB& aabb, const b2Transform& xf, int32 childIndex) const = 0;

	/// Compute the mass properties of this shape using its dimensions and density.
	/// The inertia tensor is computed about the local origin.
	/// @param massData returns the mass data for this shape.
	/// @param density the density in kilograms per meter squared.
	virtual b2MassData ComputeMass(float32 density, float32 surfaceThickness, float32 massMult) const = 0;
};
