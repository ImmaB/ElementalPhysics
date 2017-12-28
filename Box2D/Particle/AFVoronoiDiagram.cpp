/*
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
#define NOMINMAX
#include <Box2D/Particle/AFVoronoiDiagram.h>
#include <Box2D/Particle/b2StackQueue.h>
#include <Box2D/Collision/b2Collision.h>

AFVoronoiDiagram::AFVoronoiDiagram(int32 generatorCapacity)
{
	m_generatorCapacity = generatorCapacity;
	m_generatorCount = 0;
	m_countX = 0;
	m_countY = 0;
	m_diagram = NULL;
	afGenCenterXBuf(0, af::dtype::f32);
	afGenCenterYBuf(0, af::dtype::f32);
	afGenTagBuf(0, af::dtype::s32);
	afGenNecessaryBuf(0, af::dtype::b8);
}

AFVoronoiDiagram::~AFVoronoiDiagram()
{
	afGenCenterXBuf = af::array();
	afGenCenterYBuf = af::array();
	afGenTagBuf = af::array();
	afGenNecessaryBuf = af::array();
	afDiagGenCenterXBuf = af::array();
	afDiagGenCenterYBuf = af::array();
	afDiagGenTagBuf = af::array();
	afDiagGenNecessaryBuf = af::array();
}

void AFVoronoiDiagram::AddGenerators(
	const af::array& centerX, const af::array& centerY,
	const af::array& tag, const af::array& necessary)
{
	b2Assert(m_generatorCount < m_generatorCapacity);
	af::join(1, afGenCenterXBuf, centerX);
	af::join(1, afGenCenterYBuf, centerY);
	af::join(1, afGenTagBuf, tag);
	af::join(1, afGenNecessaryBuf, necessary);
	m_generatorCount += centerX.elements();
}

void AFVoronoiDiagram::Generate(float32 radius, float32 margin)
{
	b2Assert(m_diagram == NULL);
	float32 inverseRadius = 1 / radius;
	float32 lowerX, lowerY, upperX, upperY;
	af::array condIdxs = af::where(afGenNecessaryBuf);
	if (!condIdxs.isempty())
	{
		const af::array condGenCenterX = afGenCenterXBuf(condIdxs);
		const af::array condGenCenterY = afGenCenterYBuf(condIdxs);
		lowerX = af::min<float32>(condGenCenterX);
		lowerY = af::min<float32>(condGenCenterY);
		upperX = af::max<float32>(condGenCenterX);
		upperY = af::max<float32>(condGenCenterY);
	}
	lowerX -= margin;
	lowerY -= margin;
	upperX += margin;
	upperY += margin;
	m_countX = 1 + (int32) (inverseRadius * (upperX - lowerX));
	m_countY = 1 + (int32) (inverseRadius * (upperY - lowerY));

	int32 diagSize = m_countX * m_countY;
	afGenIdxDiagram = af::constant(invalidGeneratorIndex, diagSize, af::dtype::s32);

	// (4 * m_countX * m_countY) is the queue capacity that is experimentally
	// known to be necessary and sufficient for general particle distributions.
	af::array afQueueDiagTaskX(4 * diagSize, af::dtype::s32);
	af::array afQueueDiagTaskY(4 * diagSize, af::dtype::s32);
	af::array afQueueDiagTaskI(4 * diagSize, af::dtype::s32);
	af::array afQueueDiagTaskGenIdx(4 * diagSize, af::dtype::s32);

	afGenCenterXBuf = inverseRadius * (afGenCenterXBuf - lowerX);
	afGenCenterYBuf = inverseRadius * (afGenCenterYBuf - lowerY);
	
	// cast to int
	af::array x = af::select(afGenCenterXBuf > 0, af::floor(afGenCenterXBuf), af::ceil(afGenCenterXBuf));
	af::array y = af::select(afGenCenterYBuf > 0, af::floor(afGenCenterYBuf), af::ceil(afGenCenterYBuf));
	
	condIdxs = af::where(x >= 0 && y >= 0 && x < m_countX && y < m_countY);
	if (!condIdxs.isempty())
	{
		x = x(condIdxs);
		y = y(condIdxs);
		afQueueDiagTaskX = x;
		afQueueDiagTaskY = y;
		afQueueDiagTaskI = x + y * m_countX;
		afQueueDiagTaskGenIdx = condIdxs;
	}
	bool diagTasksLeft = true;
	while (diagTasksLeft)
	{
		diagTasksLeft = false;
		af::array x = afQueueDiagTaskX;
		af::array y = afQueueDiagTaskY;
		af::array i = afQueueDiagTaskI;
		af::array g = afQueueDiagTaskGenIdx;

		//clear old
		afQueueDiagTaskX = af::array();
		afQueueDiagTaskY = af::array();
		afQueueDiagTaskI = af::array();
		afQueueDiagTaskGenIdx = af::array();

		af::array emptyDiagIdxs = af::where(emptyDiagIdxs == invalidGeneratorIndex);
		if (!emptyDiagIdxs.isempty())
		{
			x = x(emptyDiagIdxs);
			y = y(emptyDiagIdxs);
			i = i(emptyDiagIdxs);
			g = g(emptyDiagIdxs);

			afGenIdxDiagram(emptyDiagIdxs) = g;

			af::array condIdxs = af::where(x > 0);
			if (!condIdxs.isempty())
			{
				diagTasksLeft = true;
				af::join(1, afQueueDiagTaskX, x(condIdxs) - 1);
				af::join(1, afQueueDiagTaskY, y(condIdxs));
				af::join(1, afQueueDiagTaskI, i(condIdxs) - 1);
				af::join(1, afQueueDiagTaskGenIdx, g(condIdxs));
			}
			condIdxs = af::where(y > 0);
			if (!condIdxs.isempty())
			{
				diagTasksLeft = true;
				af::join(1, afQueueDiagTaskX, x(condIdxs));
				af::join(1, afQueueDiagTaskY, y(condIdxs) - 1);
				af::join(1, afQueueDiagTaskI, i(condIdxs) - m_countX);
				af::join(1, afQueueDiagTaskGenIdx, g(condIdxs));
			}
			condIdxs = af::where(x < m_countX - 1);
			if (!condIdxs.isempty())
			{
				diagTasksLeft = true;
				af::join(1, afQueueDiagTaskX, x(condIdxs) + 1);
				af::join(1, afQueueDiagTaskY, y(condIdxs));
				af::join(1, afQueueDiagTaskI, i(condIdxs) + 1);
				af::join(1, afQueueDiagTaskGenIdx, g(condIdxs));
			}
			condIdxs = af::where(y < m_countY - 1);
			if (!condIdxs.isempty())
			{
				diagTasksLeft = true;
				af::join(1, afQueueDiagTaskX, x(condIdxs));
				af::join(1, afQueueDiagTaskY, y(condIdxs) + 1);
				af::join(1, afQueueDiagTaskI, i(condIdxs) + m_countX);
				af::join(1, afQueueDiagTaskGenIdx, g(condIdxs));
			}
		}
	}

	x = af::seq(0, m_countX - 1);
	y = af::seq(0, m_countY);
	x = af::tile(x, m_countY);
	y = af::tile(y, 1, m_countX - 1);
	y = af::transpose(x);
	y = af::flat(y);
	af::array i = x + y * m_countX;
	af::array a = afGenIdxDiagram(i);
	af::array b = afGenIdxDiagram(i + 1);
	condIdxs = af::where(a != b);
	if (!condIdxs.isempty())
	{
		af::join(1, afQueueDiagTaskX, x(condIdxs));
		af::join(1, afQueueDiagTaskY, y(condIdxs));
		af::join(1, afQueueDiagTaskI, i(condIdxs));
		af::join(1, afQueueDiagTaskGenIdx, b(condIdxs));

		af::join(1, afQueueDiagTaskX, x(condIdxs) + 1);
		af::join(1, afQueueDiagTaskY, y(condIdxs));
		af::join(1, afQueueDiagTaskI, i(condIdxs) + 1);
		af::join(1, afQueueDiagTaskGenIdx, a(condIdxs));
	}

	x = af::seq(0, m_countX);
	y = af::seq(0, m_countY - 1);
	x = af::tile(x, m_countY - 1);
	y = af::tile(y, 1, m_countX);
	y = af::transpose(y);
	y = af::flat(y);
	i = x + y * m_countX;
	a = afGenIdxDiagram(i);
	b = afGenIdxDiagram(i + m_countX);
	condIdxs = af::where(a != b);
	if (!condIdxs.isempty())
	{
		af::join(1, afQueueDiagTaskX, x(condIdxs));
		af::join(1, afQueueDiagTaskY, y(condIdxs));
		af::join(1, afQueueDiagTaskI, i(condIdxs));
		af::join(1, afQueueDiagTaskGenIdx, b(condIdxs));

		af::join(1, afQueueDiagTaskX, x(condIdxs));
		af::join(1, afQueueDiagTaskY, y(condIdxs) + 1);
		af::join(1, afQueueDiagTaskI, i(condIdxs) + m_countX);
		af::join(1, afQueueDiagTaskGenIdx, a(condIdxs));
	}

	diagTasksLeft = true;
	while (diagTasksLeft)
	{
		diagTasksLeft = false;
		af::array x = afQueueDiagTaskX;
		af::array y = afQueueDiagTaskY;
		af::array i = afQueueDiagTaskI;
		af::array k = afQueueDiagTaskGenIdx;

		//clear old
		afQueueDiagTaskX = af::array();
		afQueueDiagTaskY = af::array();
		afQueueDiagTaskI = af::array();
		afQueueDiagTaskGenIdx = af::array();

		af::array a = afGenIdxDiagram(i);
		af::array b = k;

		af::array outerOuterCondIdxs = af::where(a != b);
		if (!outerOuterCondIdxs.isempty())
		{
			x = x(outerOuterCondIdxs);
			y = y(outerOuterCondIdxs);
			i = i(outerOuterCondIdxs);
			a = a(outerOuterCondIdxs);
			b = b(outerOuterCondIdxs);

			af::array ax = afGenCenterXBuf(a) - x;
			af::array ay = afGenCenterYBuf(a) - y;
			af::array bx = afGenCenterXBuf(b) - x;
			af::array by = afGenCenterYBuf(b) - y;
			af::array a2 = ax * ax + ay * ay;
			af::array b2 = bx * bx + by * by;

			af::array outerCondIdxs = af::where(a2 > b2);
			if (!outerCondIdxs.isempty())
			{
				x = x(outerCondIdxs);
				y = y(outerCondIdxs);
				i = i(outerCondIdxs);
				b = b(outerCondIdxs);

				afGenIdxDiagram(i) = b;

				af::array condIdxs = af::where(x > 0);
				if (!condIdxs.isempty())
				{
					diagTasksLeft = true;
					af::join(1, afQueueDiagTaskX, x(condIdxs) - 1);
					af::join(1, afQueueDiagTaskY, y(condIdxs));
					af::join(1, afQueueDiagTaskI, i(condIdxs) - 1);
					af::join(1, afQueueDiagTaskGenIdx, b(condIdxs));
				}
				condIdxs = af::where(y > 0);
				if (!condIdxs.isempty())
				{
					diagTasksLeft = true;
					af::join(1, afQueueDiagTaskX, x(condIdxs));
					af::join(1, afQueueDiagTaskY, y(condIdxs) - 1);
					af::join(1, afQueueDiagTaskI, i(condIdxs) - m_countX);
					af::join(1, afQueueDiagTaskGenIdx, b(condIdxs));
				}
				condIdxs = af::where(x < m_countX - 1);
				if (!condIdxs.isempty())
				{
					diagTasksLeft = true;
					af::join(1, afQueueDiagTaskX, x(condIdxs) + 1);
					af::join(1, afQueueDiagTaskY, y(condIdxs));
					af::join(1, afQueueDiagTaskI, i(condIdxs) + 1);
					af::join(1, afQueueDiagTaskGenIdx, b(condIdxs));
				}
				condIdxs = af::where(y < m_countY - 1);
				if (!condIdxs.isempty())
				{
					diagTasksLeft = true;
					af::join(1, afQueueDiagTaskX, x(condIdxs));
					af::join(1, afQueueDiagTaskY, y(condIdxs) + 1);
					af::join(1, afQueueDiagTaskI, i(condIdxs) + m_countX);
					af::join(1, afQueueDiagTaskGenIdx, b(condIdxs));
				}
			}
		}
	}
}

void AFVoronoiDiagram::GetNodes(NodeCallback& callback) const
{
	af::array x = af::seq(0, m_countX - 1);
	af::array y = af::seq(0, m_countY - 1);
	x = af::tile(x, m_countY - 1);
	y = af::tile(y, 1, m_countX - 1);
	y = af::transpose(y);
	y = af::flat(y);
	af::array i = x + y * m_countX;

	af::array a = afGenIdxDiagram(i);
	af::array b = afGenIdxDiagram(i + 1);
	af::array c = afGenIdxDiagram(i + m_countX);
	af::array d = afGenIdxDiagram(i + 1 + m_countX);

	af::array outerCondIdxs = af::where(b != c);
	if (!outerCondIdxs.isempty())
	{
		a = a(outerCondIdxs);
		b = b(outerCondIdxs);
		c = c(outerCondIdxs);
		d = d(outerCondIdxs);

		af::array condIdxs = af::where(a != b && a != c &&
			(afGenNecessaryBuf(a) || afGenNecessaryBuf(b) || afGenNecessaryBuf(c)));
		if (!condIdxs.isempty())
			callback(afGenTagBuf(a), afGenTagBuf(b), afGenTagBuf(c));

		condIdxs = af::where(d != b && d != c && 
			(afGenNecessaryBuf(b) || afGenNecessaryBuf(d) || afGenNecessaryBuf(c)));
		if (!condIdxs.isempty())
			callback(afGenTagBuf(b), afGenTagBuf(d), afGenTagBuf(c));
	}
}
