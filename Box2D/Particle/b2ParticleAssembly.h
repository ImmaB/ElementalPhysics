/*
* Copyright (c) 2014 Google, Inc.
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
#ifndef B2_PARTICLE_ASSEMBLY_H
#define B2_PARTICLE_ASSEMBLY_H

#include <Box2D/Common/b2GrowableBuffer.h>
#include <Box2D/Particle/b2Particle.h>
#include <Box2D/Particle/b2ParticleContact.h>
#include <Box2D/Common/b2Math.h>


struct FindContactCheck
{
    uint16 particleIndex;
    uint16 comparatorIndex;
};

struct FindContactInput
{
    uint32 proxyIndex;
	float32 positionX;
	float32 positionY;
};

enum { NUM_V32_SLOTS = 4 };

#ifdef __cplusplus
extern "C" {
#endif

extern int CalculateTags_Simd(const Vec2* positions,
                              int count,
                              const float& inverseDiameter,
                              uint32* outTags);

extern void FindContactsFromChecks_Simd(
	const FindContactInput* reordered,
	const FindContactCheck* checks,
	int numChecks,
  const float& particleDiameterSq,
  const float& particleDiameterInv,
  const uint32* flags,
	b2GrowableBuffer<Particle::Contact>& contacts);

#ifdef __cplusplus
} // extern "C"
#endif

#endif