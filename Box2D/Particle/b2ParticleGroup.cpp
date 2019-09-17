/*
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
#include <Box2D/Particle/b2ParticleGroup.h>
#include <Box2D/Particle/b2ParticleSystem.h>
#include <Box2D/Dynamics/b2World.h>

#if LIQUIDFUN_EXTERNAL_LANGUAGE_API
#include <Box2D/Collision/Shapes/b2CircleShape.h>
#endif //LIQUIDFUN_EXTERNAL_LANGUAGE_API


#if LIQUIDFUN_EXTERNAL_LANGUAGE_API
void ParticleGroup::Def::FreeShapesMemory() {
	if (circleShapes)
	{
		delete[] circleShapes;
		circleShapes = NULL;
	}
	if (ownShapesArray && shapes)
	{
		delete[] shapes;
		shapes = NULL;
		ownShapesArray = false;
	}
}

void ParticleGroup::Def::SetCircleShapesFromVertexList(void* inBuf,
	int numShapes,
	float radius)
{
	float* points = (float*)inBuf;
	// Create circle shapes from vertex list and radius
	b2CircleShape* pCircleShapes = new b2CircleShape[numShapes];
	b2Shape** pShapes = new b2Shape*[numShapes];
	for (int i = 0; i < numShapes; ++i) {
		pCircleShapes[i].m_radius = radius;
		pCircleShapes[i].m_p = Vec2(points[i * 2], points[i * 2 + 1]);
		pShapes[i] = &pCircleShapes[i];
	}

	// Clean up existing buffers
	FreeShapesMemory();

	// Assign to newly created buffers
	ownShapesArray = true;
	circleShapes = pCircleShapes;
	shapes = pShapes;
	shapeCount = numShapes;
}
#endif // LIQUIDFUN_EXTERNAL_LANGUAGE_API
