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

#include <Box2D/Collision/Shapes/b2CircleShape.h>
#include <new>

b2Shape* b2CircleShape::Clone(b2BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(b2CircleShape));
	b2CircleShape* clone = new (mem) b2CircleShape;
	*clone = *this;
	return clone;
}

int32 b2CircleShape::GetChildCount() const
{
	return 1;
}

bool b2CircleShape::TestPoint(const b2Transform& transform, const b2Vec2& p) const
{
	b2Vec2 center = transform.p + b2Mul(transform.q, m_p);
	b2Vec2 d = p - center;
	return b2Dot(d, d) <= m_radius * m_radius;
}
af::array b2CircleShape::AFTestPoints(const b2Transform& transform, const af::array& px, const af::array& py) const
{
	float32 centerX = transform.p.x + b2MulX(transform.q, m_p.x, m_p.y);
	float32 centerY = transform.p.y + b2MulY(transform.q, m_p.x, m_p.y);
	af::array dx = px - centerX;
	af::array dy = py - centerY;
	return b2Dot(dx, dy, dx, dy) <= m_radius * m_radius;
}

void b2CircleShape::ComputeDistance(const b2Transform& transform, const b2Vec2& p, float32* distance, b2Vec2* normal, int32 childIndex) const
{
	B2_NOT_USED(childIndex);

	b2Vec2 center = transform.p + b2Mul(transform.q, m_p);
	b2Vec2 d = p - center;
	float32 d1 = d.Length();
	*distance = d1 - m_radius;
	*normal = 1 / d1 * d;
}
void b2CircleShape::AFComputeDistance(const b2Transform& transform, const af::array& px, const af::array& py, af::array& distance, af::array& normalX, af::array& normalY, int32 childIndex) const
{
	B2_NOT_USED(childIndex);

	b2Vec2 center = transform.p + b2Mul(transform.q, m_p);
	af::array dx = px - center.x;
	af::array dy = px - center.y;

	af::array d1 = af::sqrt(dx * dx + dy * dy);
	distance = d1 - m_radius;
	normalX = 1 / d1 * dx;
	normalY = 1 / d1 * dy;
}

// Collision Detection in Interactive 3D Environments by Gino van den Bergen
// From Section 3.1.2
// x = s + a * r
// norm(x) = radius
bool b2CircleShape::RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
							const b2Transform& transform, int32 childIndex) const
{
	B2_NOT_USED(childIndex);

	b2Vec2 position = transform.p + b2Mul(transform.q, m_p);
	b2Vec2 s = input.p1 - position;
	float32 b = b2Dot(s, s) - m_radius * m_radius;

	// Solve quadratic equation.
	b2Vec2 r = input.p2 - input.p1;
	float32 c =  b2Dot(s, r);
	float32 rr = b2Dot(r, r);
	float32 sigma = c * c - rr * b;

	// Check for negative discriminant and short segment.
	if (sigma < 0.0f || rr < b2_epsilon)
	{
		return false;
	}

	// Find the point of intersection of the line with the circle.
	float32 a = -(c + b2Sqrt(sigma));

	// Is the intersection point on the segment?
	if (0.0f <= a && a <= input.maxFraction * rr)
	{
		a /= rr;
		output->fraction = a;
		output->normal = s + a * r;
		output->normal.Normalize();
		return true;
	}

	return false;
}

af::array b2CircleShape::AFRayCast(afRayCastOutput* output, const afRayCastInput& input,
	const b2Transform& transform, int32 childIndex) const
{
	B2_NOT_USED(childIndex);
	
	b2Vec2 position = transform.p + b2Mul(transform.q, m_p);
	af::array sx = input.p1x - position.x;
	af::array sy = input.p1y - position.y;
	af::array b = b2Dot(sx, sy, sx, sy) - m_radius * m_radius;

	// Solve quadratic equation.
	af::array rx = input.p2x - input.p1x;
	af::array ry = input.p2y - input.p1y;
	af::array c = b2Dot(sx, sy, rx, ry);
	af::array rr = b2Dot(rx, ry, rx, ry);
	af::array sigma = c * c - rr * b;

	// Check for negative discriminant and short segment.
	af::array cond = sigma < 0.0f || rr < b2_epsilon;
	af::array remainIdxs = af::where(!cond);
	if (!remainIdxs.isempty())
	{


		// Find the point of intersection of the line with the circle.
		af::array a = -(c + af::sqrt(sigma(remainIdxs)));

		// Is the intersection point on the segment?
		af::array condIdxs = af::where(!(0.0f <= a && a <= input.maxFraction * rr));
		if (!condIdxs.isempty())
		{
			remainIdxs = remainIdxs(condIdxs);
			a = a(condIdxs) / rr(remainIdxs);
			rx = rx(remainIdxs);
			ry = ry(remainIdxs);
			output->fraction = a;
			output->normalX = sx + a * rx;
			output->normalY = sy + a * ry;
			
			// Normalize
			af::array nx = output->normalX;
			af::array ny = output->normalY;
			af::array length = af::sqrt(nx * nx + ny * ny);
			af::array toShort = length < b2_epsilon;
			length(af::where(toShort)) = 0.0f;
			af::array validIdxs = af::where(!toShort);
			if (!validIdxs.isempty())
			{
				af::array invLength = 1.0f / length(validIdxs);
				output->normalX(validIdxs) *= invLength;
				output->normalY(validIdxs) *= invLength;
			}
			return remainIdxs;
		}
	}
	return af::array();
}

void b2CircleShape::ComputeAABB(b2AABB* aabb, const b2Transform& transform, int32 childIndex) const
{
	B2_NOT_USED(childIndex);

	b2Vec2 p = transform.p + b2Mul(transform.q, m_p);
	aabb->lowerBound.Set(p.x - m_radius, p.y - m_radius);
	aabb->upperBound.Set(p.x + m_radius, p.y + m_radius);
}

void b2CircleShape::ComputeMass(b2MassData* massData, float32 density) const
{
	massData->mass = density * b2_pi * m_radius * m_radius;
	massData->center = m_p;

	// inertia about the local origin
	massData->I = massData->mass * (0.5f * m_radius * m_radius + b2Dot(m_p, m_p));
}
