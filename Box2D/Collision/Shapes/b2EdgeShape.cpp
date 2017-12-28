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

#include <Box2D/Collision/Shapes/b2EdgeShape.h>
#include <new>

void b2EdgeShape::Set(const b2Vec2& v1, const b2Vec2& v2)
{
	m_vertex1 = v1;
	m_vertex2 = v2;
	m_hasVertex0 = false;
	m_hasVertex3 = false;
}

b2Shape* b2EdgeShape::Clone(b2BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(b2EdgeShape));
	b2EdgeShape* clone = new (mem) b2EdgeShape;
	*clone = *this;
	return clone;
}

int32 b2EdgeShape::GetChildCount() const
{
	return 1;
}

bool b2EdgeShape::TestPoint(const b2Transform& xf, const b2Vec2& p) const
{
	B2_NOT_USED(xf);
	B2_NOT_USED(p);
	return false;
}
af::array b2EdgeShape::AFTestPoints(const b2Transform& xf, const af::array& px, const af::array& py) const
{
	B2_NOT_USED(xf);
	B2_NOT_USED(px);
	B2_NOT_USED(py);
	return af::constant(0, px.elements(), af::dtype::b8);
}

void b2EdgeShape::ComputeDistance(const b2Transform& xf, const b2Vec2& p, float32* distance, b2Vec2* normal, int32 childIndex) const
{
	B2_NOT_USED(childIndex);

	b2Vec2 v1 = b2Mul(xf, m_vertex1);
	b2Vec2 v2 = b2Mul(xf, m_vertex2);

	b2Vec2 d = p - v1;
	b2Vec2 s = v2 - v1;
	float32 ds = b2Dot(d, s);
	if (ds > 0)
	{
		float32 s2 = b2Dot(s, s);
		if (ds > s2)
		{
			d = p - v2;
		}
		else
		{
			d -= ds / s2 * s;
		}
	}

	float32 d1 = d.Length();
	*distance = d1;
	*normal = d1 > 0 ? 1 / d1 * d : b2Vec2_zero;

}
void b2EdgeShape::AFComputeDistance(const b2Transform& xf, const af::array& px, const af::array& py, af::array& distance, af::array& normalX, af::array& normalY, int32 childIndex) const
{
	B2_NOT_USED(childIndex);

	b2Vec2 v1 = b2Mul(xf, m_vertex1);
	b2Vec2 v2 = b2Mul(xf, m_vertex2);

	af::array dx = px - v1.x;
	af::array dy = py - v1.y;
	float32 sx = v2.x - v1.x;
	float32 sy = v2.y - v1.y;
	af::array ds = b2Dot(dx, dy, sx, sy);
	af::array condIdxs = af::where(ds > 0);
	if (!condIdxs.isempty())
	{
		float32 s2 = b2Dot(sx, sy, sx, sy);
		af::array cond = ds(condIdxs) > s2;
		af::array ifIdxs = af::where(cond);
		af::array elseIdxs = af::where(!cond);
		if (!ifIdxs.isempty())
		{
			dx(ifIdxs) = px(ifIdxs) - v2.x;
			dy(ifIdxs) = py(ifIdxs) - v2.y;
		}
		if (!elseIdxs.isempty())
		{
			dx(elseIdxs) -= ds(elseIdxs) / s2 * sx;
			dy(elseIdxs) -= ds(elseIdxs) / s2 * sy;
		}
	}
	af::array d1 = af::sqrt(dx * dx + dy * dy);
	distance = d1;
	af::array cond = d1 > 0;
	normalX = af::select(cond, 1 / d1 * dx, 0.0f);
	normalY = af::select(cond, 1 / d1 * dy, 0.0f);

}

// p = p1 + t * d
// v = v1 + s * e
// p1 + t * d = v1 + s * e
// s * e - t * d = p1 - v1
bool b2EdgeShape::RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
							const b2Transform& xf, int32 childIndex) const
{
	B2_NOT_USED(childIndex);

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
	{
		return false;
	}

	float32 t = numerator / denominator;
	if (t < 0.0f || input.maxFraction < t)
	{
		return false;
	}

	b2Vec2 q = p1 + t * d;

	// q = v1 + s * r
	// s = dot(q - v1, r) / dot(r, r)
	b2Vec2 r = v2 - v1;
	float32 rr = b2Dot(r, r);
	if (rr == 0.0f)
	{
		return false;
	}

	float32 s = b2Dot(q - v1, r) / rr;
	if (s < 0.0f || 1.0f < s)
	{
		return false;
	}

	output->fraction = t;
	if (numerator > 0.0f)
	{
		output->normal = -b2Mul(xf.q, normal);
	}
	else
	{
		output->normal = b2Mul(xf.q, normal);
	}
	return true;
}

af::array b2EdgeShape::AFRayCast(afRayCastOutput* output, const afRayCastInput& input,
	const b2Transform& xf, int32 childIndex) const
{
	B2_NOT_USED(childIndex);
	
	// Put the ray into the edge's frame of reference.
	af::array p1x = b2MulTX(xf.q, input.p1x - xf.p.x, input.p1y - xf.p.y);
	af::array p1y = b2MulTY(xf.q, input.p1x - xf.p.x, input.p1y - xf.p.y);

	af::array p2x = b2MulTX(xf.q, input.p2x - xf.p.x, input.p2y - xf.p.y);
	af::array p2y = b2MulTY(xf.q, input.p2x - xf.p.x, input.p2y - xf.p.y);

	af::array dx =  p2x - p1x;
	af::array dy =  p2y - p1y;

	b2Vec2 v1 = m_vertex1;
	b2Vec2 v2 = m_vertex2;
	b2Vec2 e = v2 - v1;
	b2Vec2 normal(e.y, -e.x);
	normal.Normalize();

	// q = p1 + t * d
	// dot(normal, q - v1) = 0
	// dot(normal, p1 - v1) + t * dot(normal, d) = 0
	af::array numerator = b2Dot(normal.x, normal.y, v1.x - p1x, v1.y - p1y);
	af::array denominator = b2Dot(normal.x, normal.y, v1.x - p1x, v1.y - p1y);

	af::array remainIdxs = af::where(!(denominator == 0.0f));
	if (!remainIdxs.isempty())
	{
		numerator = numerator(remainIdxs);
		denominator = denominator(remainIdxs);
		af::array t = numerator / denominator;

		af::array remain2Idxs = af::where(!(t < 0.0f || input.maxFraction < t));
		if (!remain2Idxs.isempty())
		{
			remainIdxs = remainIdxs(remain2Idxs);
			t = t(remain2Idxs);
			numerator = numerator(remain2Idxs);

			af::array qx = p1x(remainIdxs) + t(remain2Idxs) * dx(remainIdxs);
			af::array qy = p1y(remainIdxs) + t(remain2Idxs) * dy(remainIdxs);

			// q = v1 + s * r
			// s = dot(q - v1, r) / dot(r, r)
			b2Vec2 r = v2 - v1;
			float32 rr = b2Dot(r, r);
			if (rr == 0.0f)
			{
				return remainIdxs;
			}
			else
			{
				af::array s = b2Dot(qx - v1.x, qy - v1.y, r.x, r.y) / rr;

				af::array remain3Idxs = af::where(!(s < 0.0f || 1.0f < s));
				if (!remain3Idxs.isempty())
				{
					t = t(remain3Idxs);
					numerator = numerator(remain3Idxs);
					remainIdxs = remainIdxs(remain3Idxs);

					output->fraction = t;
					af::array cond = numerator > 0.0f;
					output->normalX(af::where(cond))  = -b2MulX(xf.q, normal.x, normal.y);
					output->normalY(af::where(cond)) = -b2MulY(xf.q, normal.x, normal.y);
					output->normalX(af::where(!cond)) = b2MulX(xf.q, normal.x, normal.y);
					output->normalY(af::where(!cond)) = b2MulY(xf.q, normal.x, normal.y);
					
					return remainIdxs;
				}
			}
		}
	}
	return af::array();
}

void b2EdgeShape::ComputeAABB(b2AABB* aabb, const b2Transform& xf, int32 childIndex) const
{
	B2_NOT_USED(childIndex);

	b2Vec2 v1 = b2Mul(xf, m_vertex1);
	b2Vec2 v2 = b2Mul(xf, m_vertex2);

	b2Vec2 lower = b2Min(v1, v2);
	b2Vec2 upper = b2Max(v1, v2);

	b2Vec2 r(m_radius, m_radius);
	aabb->lowerBound = lower - r;
	aabb->upperBound = upper + r;
}

void b2EdgeShape::ComputeMass(b2MassData* massData, float32 density) const
{
	B2_NOT_USED(density);

	massData->mass = 0.0f;
	massData->center = 0.5f * (m_vertex1 + m_vertex2);
	massData->I = 0.0f;
}
