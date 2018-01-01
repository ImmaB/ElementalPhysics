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
#ifndef B2_PARTICLE_SYSTEM_H
#define B2_PARTICLE_SYSTEM_H

#include <Box2D/Common/b2SlabAllocator.h>
#include <Box2D/Common/b2GrowableBuffer.h>
#include <Box2D/Particle/b2Particle.h>
#include <Box2D/Dynamics/b2TimeStep.h>
#include <Box2D/Particle/AFVoronoiDiagram.h>
#include <vector>
#include <numeric>

#include <arrayfire.h>
#include <af/compatible.h>

#if LIQUIDFUN_UNIT_TESTS
#include <gtest/gtest.h>
#endif // LIQUIDFUN_UNIT_TESTS

#if LIQUIDFUN_EXTERNAL_LANGUAGE_API
#include <cstring>
#endif // LIQUIDFUN_EXTERNAL_LANGUAGE_API

#define B2PARTICLECOLOR_BITS_PER_COMPONENT (sizeof(uint8) << 3)

class b2World;
class b2Body;
class b2Shape;
class b2ParticleGroup;
class b2ParticleMaterial;
class b2BlockAllocator;
class b2StackAllocator;
class b2QueryCallback;
class afQueryCallback;
class b2RayCastCallback;
class b2Fixture;
class b2ContactFilter;
class b2ContactListener;
class b2ParticlePairSet;
class FixtureParticleSet;
struct b2ParticleGroupDef;
struct b2Vec2;
struct b2AABB;
struct FindContactInput;
struct FindContactCheck;

using namespace std;

struct b2ParticleContact
{
private:
	// 16-bit particle indices consume less memory and thus improve
	// performance. We iterate through m_contactBuffer many times during
	// b2ParticleSystem::Solve, so reducing the amount of data we churn
	// through speeds things up. Also, FindContactsFromChecks_Simd takes
	// advantage of the reduced size for specific optimizations.
	#ifdef B2_USE_16_BIT_PARTICLE_INDICES
		typedef int16 b2ParticleIndex;
	#else
		typedef int32 b2ParticleIndex;
	#endif

	/// Indices of the respective particles making contact.
	b2ParticleIndex indexA, indexB;

	/// Weight of the contact. A value between 0.0f and 1.0f.
	/// 0.0f ==> particles are just barely touching
	/// 1.0f ==> particles are perfectly on top of each other
	float32 weight;

	float32 mass;

	/// The normalized direction from A to B.
	b2Vec2 normal;

	/// The logical sum of the particle behaviors that have been set.
	/// See the b2ParticleFlag enum.
	uint32 flags;

	uint32 matFlags;

public:
	void SetIndices(int32 a, int32 b);
	void SetWeight(float32 w) { weight = w; }
	void SetMass(float32 m) { mass = m; }
	void SetNormal(const b2Vec2& n) { normal = n; }
	void SetFlags(uint32 f) { flags = f; }
	void SetMatFlags(uint32 f) { matFlags = f; }

	int32 GetIndexA() const { return indexA; }
	int32 GetIndexB() const { return indexB; }
	float32 GetWeight() const { return weight; }
	float32 GetMass() const { return mass; }
	const b2Vec2& GetNormal() const { return normal; }
	uint32 GetFlags() const { return flags; }
	uint32 GetMatFlags() const { return matFlags; }

	bool operator==(const b2ParticleContact& rhs) const;
	bool operator!=(const b2ParticleContact& rhs) const { return !operator==(rhs); }
	bool ApproximatelyEqual(const b2ParticleContact& rhs) const;
};

/*struct b2ParticleBodyContact
{
	/// Index of the particle making contact.
	int32 index;

	/// The body making contact.
	b2Body* body;

	/// The specific fixture making contact
	b2Fixture* fixture;

	/// Weight of the contact. A value between 0.0f and 1.0f.
	float32 weight;

	/// The normalized direction from the particle to the body.
	b2Vec2 normal;

	/// The effective mass used in calculating force.
	float32 mass;
};*/

/// Connection between two particles
/*struct b2ParticlePair
{
	/// Indices of the respective particles making pair.
	int32 indexA, indexB;

	/// The logical sum of the particle flags. See the b2ParticleFlag enum.
	uint32 flags;

	/// The strength of cohesion among the particles.
	float32 strength;

	/// The initial distance of the particles.
	float32 distance;
};*/

/// Connection between three particles
/*struct b2ParticleTriad
{
	/// Indices of the respective particles making triad.
	int32 indexA, indexB, indexC;

	/// The logical sum of the particle flags. See the b2ParticleFlag enum.
	uint32 flags;

	/// The strength of cohesion among the particles.
	float32 strength;

	/// Values used for calculation.
	b2Vec2 pa, pb, pc;
	float32 ka, kb, kc, s;
};*/

struct b2ParticleSystemDef
{
	b2ParticleSystemDef()
	{
		strictContactCheck = false;
		density = 1.0f;
		gravityScale = 1.0f;
		radius = 1.0f;
		maxCount = 0;

		// Initialize physical coefficients to the maximum values that
		// maintain numerical stability.
		pressureStrength = 0.05f;
		dampingStrength = 1.0f;
		elasticStrength = 0.25f;
		springStrength = 0.25f;
		viscousStrength = 0.25f;
		surfaceTensionPressureStrength = 0.2f;
		surfaceTensionNormalStrength = 0.2f;
		repulsiveStrength = 1.0f;
		powderStrength = 0.5f;
		ejectionStrength = 0.5f;
		staticPressureStrength = 0.2f;
		staticPressureRelaxation = 0.2f;
		staticPressureIterations = 8;
		colorMixingStrength = 0.5f;
		destroyByAge = true;
		lifetimeGranularity = 1.0f / 60.0f;
	}

	/// Enable strict Particle/Body contact check.
	/// See SetStrictContactCheck for details.
	bool strictContactCheck;

	/// Set the particle density.
	/// See SetDensity for details.
	float32 density;

	/// Change the particle gravity scale. Adjusts the effect of the global
	/// gravity vector on particles. Default value is 1.0f.
	float32 gravityScale;

	/// Particles behave as circles with this radius. In Box2D units.
	float32 radius;

	/// Set the maximum number of particles.
	/// By default, there is no maximum. The particle buffers can continue to
	/// grow while b2World's block allocator still has memory.
	/// See SetMaxParticleCount for details.
	int32 maxCount;

	/// Increases pressure in response to compression
	/// Smaller values allow more compression
	float32 pressureStrength;

	/// Reduces velocity along the collision normal
	/// Smaller value reduces less
	float32 dampingStrength;

	/// Restores shape of elastic particle groups
	/// Larger values increase elastic particle velocity
	float32 elasticStrength;

	/// Restores length of spring particle groups
	/// Larger values increase spring particle velocity
	float32 springStrength;

	/// Reduces relative velocity of viscous particles
	/// Larger values slow down viscous particles more
	float32 viscousStrength;

	/// Produces pressure on tensile particles
	/// 0~0.2. Larger values increase the amount of surface tension.
	float32 surfaceTensionPressureStrength;

	/// Smoothes outline of tensile particles
	/// 0~0.2. Larger values result in rounder, smoother, water-drop-like
	/// clusters of particles.
	float32 surfaceTensionNormalStrength;

	/// Produces additional pressure on repulsive particles
	/// Larger values repulse more
	/// Negative values mean attraction. The range where particles behave
	/// stably is about -0.2 to 2.0.
	float32 repulsiveStrength;

	/// Produces repulsion between powder particles
	/// Larger values repulse more
	float32 powderStrength;

	/// Pushes particles out of solid particle group
	/// Larger values repulse more
	float32 ejectionStrength;

	/// Produces static pressure
	/// Larger values increase the pressure on neighboring partilces
	/// For a description of static pressure, see
	/// http://en.wikipedia.org/wiki/Static_pressure#Static_pressure_in_fluid_dynamics
	float32 staticPressureStrength;

	/// Reduces instability in static pressure calculation
	/// Larger values make stabilize static pressure with fewer iterations
	float32 staticPressureRelaxation;

	/// Computes static pressure more precisely
	/// See SetStaticPressureIterations for details
	int32 staticPressureIterations;

	/// Determines how fast colors are mixed
	/// 1.0f ==> mixed immediately
	/// 0.5f ==> mixed half way each simulation step (see b2World::Step())
	float32 colorMixingStrength;

	/// Whether to destroy particles by age when no more particles can be
	/// created.  See #b2ParticleSystem::SetDestructionByAge() for
	/// more information.
	bool destroyByAge;

	/// Granularity of particle lifetimes in seconds.  By default this is
	/// set to (1.0f / 60.0f) seconds.  b2ParticleSystem uses a 32-bit signed
	/// value to track particle lifetimes so the maximum lifetime of a
	/// particle is (2^32 - 1) / (1.0f / lifetimeGranularity) seconds.
	/// With the value set to 1/60 the maximum lifetime or age of a particle is
	/// 2.27 years.
	float32 lifetimeGranularity;
};
struct b2ParticleMaterialDef
{
	uint32 matFlags;
	float32 mass;
	float32 stability;
	float32 extinguishingPoint;
	float32 meltingPoint;
	float32 boilingPoint;
	float32 ignitionPoint;
	float32 heatConductivity;
};


class b2ParticleSystem
{
private:
	b2TimeStep m_step;
	b2TimeStep subStep;

public:
	int MyIndex;
	void SetIndex(int ind);

	void SetStep(b2TimeStep step) 
	{
		m_step = step;
	}

	void AFSolveInit();
	void AFSolveIterationPart1(int32 iteration);
	void AFSolveIterationPart2();
	void AFSolveIterationPart3();
	void AFSolveIterationPart4();
	void AFSolveIterationPart5();
	void AFSolveEnd();

	void SolveInit();
	void SolveIteration(int32 iteration);
	void SolveEnd();

	template <class T>
	void CopyFromAFArrayToVector(const int32& count, const af::array& afArray, std::vector<T>& vec);

	/// Create a particle whose properties have been defined.
	/// No reference to the definition is retained.
	/// A simulation step must occur before it's possible to interact with a
	/// newly created particle.  For example, DestroyParticleInShape() will
	/// not destroy a particle until b2World::Step() has been called.
	/// @warning This function is locked during callbacks.
	/// @return the index of the particle.
	int32 CreateParticle(const b2ParticleDef& def);
	int32 AFCreateParticle(const afParticleDef& def);

	const uint32 GetLayerMaskFromZ(float32 z);
	const af::array AFGetLayerMaskFromZ(const af::array& z);

	/// Retrieve a handle to the particle at the specified index.
	/// Please see #b2ParticleHandle for why you might want a handle.
	const b2ParticleHandle* GetParticleHandleFromIndex(const int32 index);

	/// Destroy a particle.
	/// The particle is removed after the next simulation step (see
	/// b2World::Step()).
	void DestroyParticle(int32 index)
	{
		DestroyParticle(index, false);
	}

	/// Destroy a particle.
	/// The particle is removed after the next step.
	/// @param Index of the particle to destroy.
	/// @param Whether to call the destruction listener just before the
	/// particle is destroyed.
	void DestroyParticle(int32 index, bool callDestructionListener);
	void AFDestroyParticles(af::array idxs, bool callDestructionListener = false);

	/// Destroy the Nth oldest particle in the system.
	/// The particle is removed after the next b2World::Step().
	/// @param Index of the Nth oldest particle to destroy, 0 will destroy the
	/// oldest particle in the system, 1 will destroy the next oldest
	/// particle etc.
	/// @param Whether to call the destruction listener just before the
	/// particle is destroyed.
	void DestroyOldestParticle(const int32 index,
							   const bool callDestructionListener);

	/// Destroy particles inside a shape without enabling the destruction
	/// callback for destroyed particles.
	/// This function is locked during callbacks.
	/// For more information see
	/// DestroyParticleInShape(const b2Shape&, const b2Transform&,bool).
	/// @param Shape which encloses particles that should be destroyed.
	/// @param Transform applied to the shape.
	/// @warning This function is locked during callbacks.
	/// @return Number of particles destroyed.
	int32 DestroyParticlesInShape(const b2Shape& shape, const b2Transform& xf)
	{
		return DestroyParticlesInShape(shape, xf, false);
	}

	/// Destroy particles inside a shape.
	/// This function is locked during callbacks.
	/// In addition, this function immediately destroys particles in the shape
	/// in constrast to DestroyParticle() which defers the destruction until
	/// the next simulation step.
	/// @param Shape which encloses particles that should be destroyed.
	/// @param Transform applied to the shape.
	/// @param Whether to call the world b2DestructionListener for each
	/// particle destroyed.
	/// @warning This function is locked during callbacks.
	/// @return Number of particles destroyed.
	int32 DestroyParticlesInShape(const b2Shape& shape, const b2Transform& xf,
	                              bool callDestructionListener);

	void FilterRealGroups();
	void AFFilterRealGroups();

	int32 CreateParticleMaterial(const b2ParticleMaterialDef& def);

	/// Create a particle group whose properties have been defined. No
	/// reference to the definition is retained.
	/// @warning This function is locked during callbacks.
	int32 CreateParticleGroup(const b2ParticleGroupDef& def);
	int32 AFCreateParticleGroup(const b2ParticleGroupDef& def);

	/// Join two particle groups.
	/// @param the first group. Expands to encompass the second group.
	/// @param the second group. It is destroyed.
	/// @warning This function is locked during callbacks.
	void JoinParticleGroups(int32 groupAIdx, int32 groupBIdx);

	/// Split particle group into multiple disconnected groups.
	/// @param the group to be split.
	/// @warning This function is locked during callbacks.
	void SplitParticleGroup(b2ParticleGroup* group);

	/// Get the world linear velocity of a world point, from the average linear
	/// and angular velocities of the particle group.
	/// @param a point in world coordinates.
	/// @return the world velocity of a point.
	inline b2Vec2 b2ParticleSystem::GetGroupLinearVelocityFromWorldPoint(int32 groupIdx,
		const b2Vec2& worldPoint)
	{
		UpdateGroupStatistics(groupIdx);
		return b2Vec2(m_groupLinVelXBuf[groupIdx], m_groupLinVelYBuf[groupIdx])
			+ b2Cross(m_groupAngVelBuf[groupIdx], worldPoint - b2Vec2(m_groupCenterXBuf[groupIdx],
					  m_groupCenterYBuf[groupIdx]));
	}

	void UpdateGroupStatistics(int32 groupIdx);
	
	inline float32 b2ParticleSystem::GetGroupMass(int32 groupIdx)
	{
		UpdateGroupStatistics(groupIdx);
		return m_groupMassBuf[groupIdx];
	}
	inline float32 b2ParticleSystem::GetGroupInertia(int32 groupIdx)
	{
		UpdateGroupStatistics(groupIdx);
		return m_groupInertiaBuf[groupIdx];
	}
	inline b2Vec2 b2ParticleSystem::GetGroupCenter(int32 groupIdx)
	{
		UpdateGroupStatistics(groupIdx);
		return b2Vec2(m_groupCenterXBuf[groupIdx], m_groupCenterYBuf[groupIdx]);
	}
	/*inline b2Vec2 b2ParticleSystem::GetGroupLinearVelocity(int32 groupIdx) const
	{
		UpdateStatistics(groupIdx);
		return m_groupLinVelBuf[groupIdx];
	}*/

	inline int32 GetGroupMaterialIdx(int32 groupIdx)
	{
		return m_groupMatIdxBuf[groupIdx];
	}
	inline int32 GetGroupUserData(int32 groupIdx)
	{
		return m_groupUserDataBuf[groupIdx];
	}

	/// Get the world particle group list. With the returned group, use
	/// b2ParticleGroup::GetNext to get the next group in the world list.
	/// A NULL group indicates the end of the list.
	/// @return the head of the world particle group list.
	b2ParticleGroup* GetParticleGroupList();
	const b2ParticleGroup* GetParticleGroupList() const;

	/// Get the number of particle groups.
	int32 GetParticleGroupCount() const;

	/// Get the number of particles.
	int32 GetParticleCount() const;

	/// Get the maximum number of particles.
	int32 GetMaxParticleCount() const;

	/// Set the maximum number of particles.
	/// A value of 0 means there is no maximum. The particle buffers can
	/// continue to grow while b2World's block allocator still has memory.
	/// Note: If you try to CreateParticle() with more than this count,
	/// b2_invalidParticleIndex is returned unless
	/// SetDestructionByAge() is used to enable the destruction of the
	/// oldest particles in the system.
	void SetMaxParticleCount(int32 count);

	/// Get all existing particle flags.
	uint32 GetAllParticleFlags() const;

	/// Get all existing particle group flags.
	uint32 GetAllGroupFlags() const;

	/// Pause or unpause the particle system. When paused, b2World::Step()
	/// skips over this particle system. All b2ParticleSystem function calls
	/// still work.
	/// @param paused is true to pause, false to un-pause.
	void SetPaused(bool paused);

	/// @return true if the particle system is being updated in
	/// b2World::Step().
	/// Initially, true, then, the last value passed into SetPaused().
	bool GetPaused() const;

	/// Change the particle density.
	/// Particle density affects the mass of the particles, which in turn
	/// affects how the particles interact with b2Bodies. Note that the density
	/// does not affect how the particles interact with each other.
	void SetDensity(float32 density);

	/// Get the particle density.
	float32 GetDensity() const;

	/// Change the particle gravity scale. Adjusts the effect of the global
	/// gravity vector on particles.
	void SetGravityScale(float32 gravityScale);

	/// Get the particle gravity scale.
	float32 GetGravityScale() const;

	/// Damping is used to reduce the velocity of particles. The damping
	/// parameter can be larger than 1.0f but the damping effect becomes
	/// sensitive to the time step when the damping parameter is large.
	void SetDamping(float32 damping);

	/// Get damping for particles
	float32 GetDamping() const;

	/// Change the number of iterations when calculating the static pressure of
	/// particles. By default, 8 iterations. You can reduce the number of
	/// iterations down to 1 in some situations, but this may cause
	/// instabilities when many particles come together. If you see particles
	/// popping away from each other like popcorn, you may have to increase the
	/// number of iterations.
	/// For a description of static pressure, see
	/// http://en.wikipedia.org/wiki/Static_pressure#Static_pressure_in_fluid_dynamics
	void SetStaticPressureIterations(int32 iterations);

	/// Get the number of iterations for static pressure of particles.
	int32 GetStaticPressureIterations() const;


	void SetRoomTemperature(float32 roomTemp);
	float32 GetRoomTemperature() const;

	void SetHeatLossRatio(float32 heatLossRatio);
	//float32 GetHeatLossRatio() const;

	void SetPointsPerLayer(float32 pointsPerLayer);
	//float32 GetPointsPerLayer() const;

	void SetLowestPoint(float32 lowestLayer);
	//float32 GetLowestPoint() const;

	void CalcLayerValues();

	/// Change the particle radius.
	/// You should set this only once, on world start.
	/// If you change the radius during execution, existing particles may
	/// explode, shrink, or behave unexpectedly.
	void SetRadius(float32 radius);

	/// Get the particle radius.
	float32 GetRadius() const;

	/// Get the position of each particle
	/// Array is length GetParticleCount()
	/// @return the pointer to the head of the particle positions array.
	float32* GetPositionXBuffer();
	const float32* GetPositionXBuffer() const;
	float32* GetPositionYBuffer();
	const float32* GetPositionYBuffer() const;
	float32* GetPositionZBuffer();
	const float32* GetPositionZBuffer() const;

	/// Get the velocity of each particle
	/// Array is length GetParticleCount()
	/// @return the pointer to the head of the particle velocities array.
	float32* GetVelocityXBuffer();
	const float32* GetVelocityXBuffer() const;
	float32* GetVelocityYBuffer();
	const float32* GetVelocityYBuffer() const;

	/// Get the color of each particle
	/// Array is length GetParticleCount()
	/// @return the pointer to the head of the particle colors array.
	int32* GetColorBuffer();
	const int32* GetColorBuffer() const;

	/// Get the particle-group of each particle.
	/// Array is length GetParticleCount()
	/// @return the pointer to the head of the particle group array.
	int32* GetGroupIdxBuffer();
	const int32* GetGroupIdxBuffer() const;

	/// Get the weight of each particle
	/// Array is length GetParticleCount()
	/// @return the pointer to the head of the particle positions array.
	float32* GetWeightBuffer();
	const float32* GetWeightBuffer() const;

	/// Get the temperature of each particle
	/// Array is length GetParticleCount()
	/// @return the pointer to the head of the particle temperature array.
	float32* GetHeatBuffer();
	const float32* GetHeatBuffer() const;

	/// Get the health of each particle
	/// Array is length GetParticleCount()
	/// @return the pointer to the head of the particle health array.
	float32* GetHealthBuffer();
	const float32* GetHealthBuffer() const;

	/// Get the user-specified data of each particle.
	/// Array is length GetParticleCount()
	/// @return the pointer to the head of the particle user-data array.
	int32* GetUserDataBuffer();
	const int32* GetUserDataBuffer() const;

	/// Get the flags for each particle. See the b2ParticleFlag enum.
	/// Array is length GetParticleCount()
	/// @return the pointer to the head of the particle-flags array.
	const uint32* GetFlagsBuffer() const;

	const uint32* GetCollisionLayerBuffer() const;
	uint32* GetCollisionLayerBuffer();

	/// Set flags for a particle. See the b2ParticleFlag enum.
	void SetParticleFlags(int32 index, uint32 flags);
	void AFSetParticleFlags(const af::array& idxs, uint32 flags);
	void AddParticleFlags(int32 index, uint32 flags);
	/// Get flags for a particle. See the b2ParticleFlag enum.
	uint32 GetParticleFlags(const int32 index);

	/// Set an external buffer for particle data.
	/// Normally, the b2World's block allocator is used for particle data.
	/// However, sometimes you may have an OpenGL or Java buffer for particle
	/// data. To avoid data duplication, you may supply this external buffer.
	///
	/// Note that, when b2World's block allocator is used, the particle data
	/// buffers can grow as required. However, when external buffers are used,
	/// the maximum number of particles is clamped to the size of the smallest
	/// external buffer.
	///
	/// @param buffer is a pointer to a block of memory.
	/// @param size is the number of values in the block.
	void SetFlagsBuffer(uint32* buffer, int32 capacity);
	void SetPositionBuffer(float* bufferX, float* bufferY, int32 capacity);
	void SetVelocityBuffer(float* bufferX, float* bufferY, int32 capacity);
	void SetColorBuffer(int32* buffer, int32 capacity);
	void SetUserDataBuffer(int32* buffer, int32 capacity);

	/// Get contacts between particles
	/// Contact data can be used for many reasons, for example to trigger
	/// rendering or audio effects.
	const int32* GetContactIdxAs() const;
	const int32* GetContactIdxBs() const;
	const int32 GetContactCount() const;

	b2Body** GetBodyBuffer();
	b2Fixture** GetFixtureBuffer();

	/// Get contacts between particles and bodies
	/// Contact data can be used for many reasons, for example to trigger
	/// rendering or audio effects.
	const int32* GetBodyContactIdxs() const;
	const int32* GetBodyContactBodyIdxs() const;
	const int32* GetBodyContactFixtureIdxs() const;
	const float32* GetBodyContactNormalXs() const;
	const float32* GetBodyContactNormalYs() const;
	const float32* GetBodyContactWeights() const;

	int32 GetBodyContactCount() const;

	/// Get array of particle pairs. The particles in a pair:
	///   (1) are contacting,
	///   (2) are in the same particle group,
	///   (3) are part of a rigid particle group, or are spring, elastic,
	///       or wall particles.
	///   (4) have at least one particle that is a spring or barrier
	///       particle (i.e. one of the types in k_pairFlags),
	///   (5) have at least one particle that returns true for
	///       ConnectionFilter::IsNecessary,
	///   (6) are not zombie particles.
	/// Essentially, this is an array of spring or barrier particles that
	/// are interacting. The array is sorted by b2ParticlePair's indexA,
	/// and then indexB. There are no duplicate entries.
	//const b2ParticlePair* GetPairs() const;
	//int32 GetPairCount() const;

	/// Get array of particle triads. The particles in a triad:
	///   (1) are in the same particle group,
	///   (2) are in a Voronoi triangle together,
	///   (3) are within b2_maxTriadDistance particle diameters of each
	///       other,
	///   (4) return true for ConnectionFilter::ShouldCreateTriad
	///   (5) have at least one particle of type elastic (i.e. one of the
	///       types in k_triadFlags),
	///   (6) are part of a rigid particle group, or are spring, elastic,
	///       or wall particles.
	///   (7) are not zombie particles.
	/// Essentially, this is an array of elastic particles that are
	/// interacting. The array is sorted by b2ParticleTriad's indexA,
	/// then indexB, then indexC. There are no duplicate entries.
	//const b2ParticleTriad* GetTriads() const;
	//int32 GetTriadCount() const;

	/// Set an optional threshold for the maximum number of
	/// consecutive particle iterations that a particle may contact
	/// multiple bodies before it is considered a candidate for being
	/// "stuck". Setting to zero or less disables.
	void SetStuckThreshold(int32 iterations);
	void AFSetStuckThreshold(int32 iterations);

	/// Get potentially stuck particles from the last step; the user must
	/// decide if they are stuck or not, and if so, delete or move them
	const int32* GetStuckCandidates() const;

	/// Get the number of stuck particle candidates from the last step.
	int32 GetStuckCandidateCount() const;

	/// Compute the kinetic energy that can be lost by damping force
	float32 ComputeCollisionEnergy() const;

	/// Set strict Particle/Body contact check.
	/// This is an option that will help ensure correct behavior if there are
	/// corners in the world model where Particle/Body contact is ambiguous.
	/// This option scales at n*log(n) of the number of Particle/Body contacts,
	/// so it is best to only enable if it is necessary for your geometry.
	/// Enable if you see strange particle behavior around b2Body
	/// intersections.
	void SetStrictContactCheck(bool enabled);
	/// Get the status of the strict contact check.
	bool GetStrictContactCheck() const;

	/// Set the lifetime (in seconds) of a particle relative to the current
	/// time.  A lifetime of less than or equal to 0.0f results in the particle
	/// living forever until it's manually destroyed by the application.
	void SetParticleLifetime(const int32 index, const float32 lifetime);
	/// Get the lifetime (in seconds) of a particle relative to the current
	/// time.  A value > 0.0f is returned if the particle is scheduled to be
	/// destroyed in the future, values <= 0.0f indicate the particle has an
	/// infinite lifetime.
	float32 GetParticleLifetime(const int32 index);

	/// Enable / disable destruction of particles in CreateParticle() when
	/// no more particles can be created due to a prior call to
	/// SetMaxParticleCount().  When this is enabled, the oldest particle is
	/// destroyed in CreateParticle() favoring the destruction of particles
	/// with a finite lifetime over particles with infinite lifetimes.
	/// This feature is enabled by default when particle lifetimes are
	/// tracked.  Explicitly enabling this feature using this function enables
	/// particle lifetime tracking.
	void SetDestructionByAge(const bool enable);
	/// Get whether the oldest particle will be destroyed in CreateParticle()
	/// when the maximum number of particles are present in the system.
	bool GetDestructionByAge() const;

	/// Convert a expiration time value in returned by
	/// GetExpirationTimeBuffer() to a time in seconds relative to the
	/// current simulation time.
	float32 ExpirationTimeToLifetime(const int32 expirationTime) const;
	/// Get the array of particle indices ordered by reverse lifetime.
	/// The oldest particle indexes are at the end of the array with the
	/// newest at the start.  Particles with infinite lifetimes
	/// (i.e expiration times less than or equal to 0) are placed at the start
	///  of the array.
	/// ExpirationTimeToLifetime(GetExpirationTimeBuffer()[index])
	/// is equivalent to GetParticleLifetime(index).
	/// GetParticleCount() items are in the returned array.
	vector<int32> GetIndexByExpirationTimeBuffer();


	float32 GetParticleMass() const;
	float32 GetParticleInvMass() const;


	/// Apply an impulse to one particle. This immediately modifies the
	/// velocity. Similar to b2Body::ApplyLinearImpulse.
	/// @param index the particle that will be modified.
	/// @param impulse the world impulse vector, usually in N-seconds or
    ///        kg-m/s.
	void ParticleApplyLinearImpulse(int32 index, const b2Vec2& impulse);

	/// Apply an impulse to all particles between 'firstIndex' and 'lastIndex'.
	/// This immediately modifies the velocity. Note that the impulse is
	/// applied to the total mass of all particles. So, calling
	/// ParticleApplyLinearImpulse(0, impulse) and
	/// ParticleApplyLinearImpulse(1, impulse) will impart twice as much
	/// velocity as calling just ApplyLinearImpulse(0, 1, impulse).
	/// @param firstIndex the first particle to be modified.
	/// @param lastIndex the last particle to be modified.
	/// @param impulse the world impulse vector, usually in N-seconds or
    ///        kg-m/s.
	void ApplyLinearImpulse(int32 firstIndex, int32 lastIndex,
							const b2Vec2& impulse);

	/// Apply a force to the center of a particle.
	/// @param index the particle that will be modified.
	/// @param force the world force vector, usually in Newtons (N).
	void ParticleApplyForce(int32 index, float32 forceX, float32 forceY);
	void AFParticleApplyForce(const af::array& idxs, const af::array& forceX, const af::array& forceY);

	/// Distribute a force across several particles. The particles must not be
	/// wall particles. Note that the force is distributed across all the
	/// particles, so calling this function for indices 0..N is not the same as
	/// calling ParticleApplyForce(i, force) for i in 0..N.
	/// @param firstIndex the first particle to be modified.
	/// @param lastIndex the last particle to be modified.
	/// @param force the world force vector, usually in Newtons (N).
	void ApplyForce(int32 firstIndex, int32 lastIndex, float32 forceX, float32 forceY);
	void AFApplyForce(int32 firstIndex, int32 lastIndex, float32 forceX, float32 forceY);

	/// Get the next particle-system in the world's particle-system list.
	b2ParticleSystem* GetNext();
	const b2ParticleSystem* GetNext() const;

	/// Query the particle system for all particles that potentially overlap
	/// the provided AABB. b2QueryCallback::ShouldQueryParticleSystem is
	/// ignored.
	/// @param callback a user implemented callback class.
	/// @param aabb the query box.
	void QueryAABB(b2QueryCallback* callback, const b2AABB& aabb) const;
	void AFQueryAABB(afQueryCallback* callback, const b2AABB& aabb) const;

	/// Query the particle system for all particles that potentially overlap
	/// the provided shape's AABB. Calls QueryAABB internally.
	/// b2QueryCallback::ShouldQueryParticleSystem is ignored.
	/// @param callback a user implemented callback class.
	/// @param shape the query shape
	/// @param xf the transform of the AABB
	void QueryShapeAABB(b2QueryCallback* callback, const b2Shape& shape,
						const b2Transform& xf) const;

	/// Ray-cast the particle system for all particles in the path of the ray.
	/// Your callback controls whether you get the closest point, any point, or
	/// n-points. The ray-cast ignores particles that contain the starting
	/// point. b2RayCastCallback::ShouldQueryParticleSystem is ignored.
	/// @param callback a user implemented callback class.
	/// @param point1 the ray starting point
	/// @param point2 the ray ending point
	void RayCast(b2RayCastCallback* callback, const b2Vec2& point1,
				 const b2Vec2& point2) const;

	/// Compute the axis-aligned bounding box for all particles contained
	/// within this particle system.
	/// @param aabb Returns the axis-aligned bounding box of the system.
	void ComputeAABB(b2AABB* const aabb) const;
	void AFComputeAABB(b2AABB* const aabb) const;

#if LIQUIDFUN_EXTERNAL_LANGUAGE_API
public:
	enum b2ExceptionType
	{
		b2_bufferTooSmall,
		b2_particleIndexOutOfBounds,
		b2_numErrors,
		b2_noExceptions,
	};

	/// Set the velocity of particle at index with direct floats.
	void SetParticleVelocity(int32 index, float32 vx, float32 vy);

	/// Get the x-coordinate of particle at index.
	float GetParticlePositionX(int32 index) const;

	/// Get the y-coordinate of particle at index.
	float GetParticlePositionY(int32 index) const;

	/// Copy position buffer into a specified buffer, starting from startIndex.
	int CopyPositionBuffer(int startIndex, int numParticles, void* outBuf,
						   int size) const;

	/// Copy color buffer into a specified buffer, starting from startIndex.
	int CopyColorBuffer(int startIndex, int numParticles, void* outBuf,
					    int size) const;

	/// Copy color buffer into a specified buffer, starting from startIndex.
	int CopyWeightBuffer(int startIndex, int numParticles, void* outBuf,
						 int size) const;

private:
	/// Helper function for buffer copies.
	int CopyBuffer(int startIndex, int numParticles, void* inBufWithOffset,
				   void* outBuf, int outBufSize, int copySize) const;

	/// Check if buffer copy is valid for the Get*Buffer functions that have
	/// a user-supplied output buffer.
	b2ExceptionType IsBufCopyValid(int startIndex, int numParticles,
								   int copySize, int bufSize) const;
#endif // LIQUIDFUN_EXTERNAL_LANGUAGE_API

private:
	friend class b2World;
	friend class b2ParticleGroup;
	friend class b2ParticleBodyContactRemovePredicate;
	friend class b2FixtureParticleQueryCallback;
	friend class afFixtureParticleQueryCallback;
#ifdef LIQUIDFUN_UNIT_TESTS
	FRIEND_TEST(FunctionTests, GetParticleMass);
	FRIEND_TEST(FunctionTests, AreProxyBuffersTheSame);
#endif // LIQUIDFUN_UNIT_TESTS

	template <typename T>
	struct UserOverridableBuffer
	{
		UserOverridableBuffer()
		{
			data = NULL;
			userSuppliedCapacity = 0;
		}
		T* data;
		int32 userSuppliedCapacity;
	};

	/// Used for detecting particle contacts
	struct Proxy
	{
		int32 index;
		uint32 tag;
		friend inline bool operator<(const Proxy &a, const Proxy &b)
		{
			return a.tag < b.tag;
		}
		friend inline bool operator<(uint32 a, const Proxy &b)
		{
			return a < b.tag;
		}
		friend inline bool operator<(const Proxy &a, uint32 b)
		{
			return a.tag < b;
		}
	};

	/// Class for filtering pairs or triads.
	class ConnectionFilter
	{
	public:
		virtual ~ConnectionFilter() {}
		/// Is the particle necessary for connection?
		/// A pair or a triad should contain at least one 'necessary' particle.
		virtual bool IsNecessary(int32 index) const
		{
			B2_NOT_USED(index);
			return true;
		}
		/// An additional condition for creating a pair.
		virtual bool ShouldCreatePair(int32 a, int32 b) const
		{
			B2_NOT_USED(a);
			B2_NOT_USED(b);
			return true;
		}
		/// An additional condition for creating a triad.
		virtual bool ShouldCreateTriad(int32 a, int32 b, int32 c) const
		{
			B2_NOT_USED(a);
			B2_NOT_USED(b);
			B2_NOT_USED(c);
			return true;
		}
	}; 
	class AFConnectionFilter
	{
	public:
		virtual ~AFConnectionFilter() {}
		/// Is the particle necessary for connection?
		/// A pair or a triad should contain at least one 'necessary' particle.
		virtual af::array IsNecessary(af::array idxs) const
		{
			B2_NOT_USED(idxs);
			return af::constant(1, idxs.elements(), af::dtype::b8);
		}
		/// An additional condition for creating a pair.
		virtual af::array ShouldCreatePair(af::array a, af::array b) const
		{
			B2_NOT_USED(a);
			B2_NOT_USED(b);
			return af::constant(1, a.elements(), af::dtype::b8);
		}
		/// An additional condition for creating a triad.
		virtual af::array AFShouldCreateTriad(const af::array& a, const af::array& b, const af::array& c) const
		{
			B2_NOT_USED(a);
			B2_NOT_USED(b);
			B2_NOT_USED(c);
			return af::constant(1, a.elements(), af::dtype::b8);
		}
	};


	/// InsideBoundsEnumerator enumerates all particles inside the given bounds.
	class InsideBoundsEnumerator
	{
	public:
		const b2ParticleSystem *m_partSys;
		/// Construct an enumerator with bounds of tags and a range of proxies.
		/*/
		InsideBoundsEnumerator(
			uint32 lower, uint32 upper,
			const Proxy* first, const Proxy* last);
		/*/
		InsideBoundsEnumerator(const b2ParticleSystem* partSys,
			uint32 lower, uint32 upper,
			int32 firstPos, int32 lastPos);
		//*/
		/// Get index of the next particle. Returns b2_invalidParticleIndex if
		/// there are no more particles.
		int32 GetNext();
	private:
		/// The lower and upper bound of x component in the tag.
		uint32 m_xLower, m_xUpper;
		/// The lower and upper bound of y component in the tag.
		uint32 m_yLower, m_yUpper;
		/// The range of proxies.
		int32 m_firstPos;
		int32 m_lastPos;
	};

	/// Node of linked lists of connected particles
	struct ParticleListNode
	{
		/// The head of the list.
		ParticleListNode* list;
		/// The next node in the list.
		ParticleListNode* next;
		/// Number of entries in the list. Valid only for the node at the head
		/// of the list.
		int32 count;
		/// Particle index.
		int32 index;
	};

	/// All particle types that require creating pairs
	static const int32 k_pairFlags =
		b2_springParticle |
		b2_barrierParticle;
	/// All particle types that require creating triads
	static const int32 k_triadFlags =
		b2_elasticParticle;
	/// All particle types that do not produce dynamic pressure
	static const int32 k_noPressureFlags =
		b2_powderParticle |
		b2_tensileParticle;
	/// All particle types that apply extra damping force with bodies
	static const int32 k_extraDampingFlags =
		b2_staticPressureParticle;

	b2ParticleSystem(const b2ParticleSystemDef* def, b2World* world,
					vector<b2Body*>& bodyBuffer,
					vector<b2Fixture*>& fixtureBuffer);
	~b2ParticleSystem();

	template <typename T> void FreeBuffer(T** b, int capacity);
	template <typename T> void FreeUserOverridableBuffer(
		UserOverridableBuffer<T>* b);
	template <typename T> T* ReallocateBuffer(T* buffer, int32 oldCapacity,
											  int32 newCapacity);
	template <typename T> T* ReallocateBuffer(
		T* buffer, int32 userSuppliedCapacity, int32 oldCapacity,
		int32 newCapacity, bool deferred);
	template <typename T> T* ReallocateBuffer(
		UserOverridableBuffer<T>* buffer, int32 oldCapacity, int32 newCapacity,
		bool deferred);
	template <typename T> void RequestBuffer(vector<T>& buf, bool& hasBuf);
	void AFRequestBuffer(af::array& buf, bool& hasBuf);

	/// Reallocate the handle / index map and schedule the allocation of a new
	/// pool for handle allocation.
	void ReallocateHandleBuffers(int32 newCapacity);

	void ResizePartMatBuffers(int32 size);
	void ResizeParticleBuffers(int32 size);
	void ResizeGroupBuffers(int32 size);
	void ResizeContactBuffers(int32 size);
	void ResizeBodyContactBuffers(int32 size);
	void ResizePairBuffers(int32 size);
	void ResizeTriadBuffers(int32 size);
	int32 CreateParticleForGroup(
		const b2ParticleGroupDef& groupDef,
		const b2Transform& xf, const b2Vec2& position);
	int32 AFCreateParticleForGroup(
		const b2ParticleGroupDef& groupDef,
		const b2Transform& xf, const af::array& positionX, const af::array& positionY);
	int32 CreateParticleForGroup(
		const b2ParticleGroupDef& groupDef,
		const b2Transform& xf, const b2Vec2& position, const float32 height, int32 color);
	int32 AFCreateParticleForGroup(
		const b2ParticleGroupDef& groupDef,
		const b2Transform& xf, const af::array& positionX, const af::array& positionY, const af::array& positionZ, const af::array& color);
	void CreateParticlesStrokeShapeForGroup(
		const b2Shape* shape,
		const b2ParticleGroupDef& groupDef, const b2Transform& xf);
	void AFCreateParticlesStrokeShapeForGroup(
		const b2Shape* shape,
		const b2ParticleGroupDef& groupDef, const b2Transform& xf);
	void CreateParticlesFillShapeForGroup(
		const b2Shape* shape,
		const b2ParticleGroupDef& groupDef, const b2Transform& xf);
	void AFCreateParticlesFillShapeForGroup(
		const b2Shape* shape,
		const b2ParticleGroupDef& groupDef, const b2Transform& xf);
	void CreateParticlesWithShapeForGroup(
		const b2Shape* shape,
		const b2ParticleGroupDef& groupDef, const b2Transform& xf);
	void AFCreateParticlesWithShapeForGroup(
		const b2Shape* shape,
		const b2ParticleGroupDef& groupDef, const b2Transform& xf);
	void CreateParticlesWithShapesForGroup(
		const b2Shape* const* shapes, int32 shapeCount,
		const b2ParticleGroupDef& groupDef, const b2Transform& xf);
	void AFCreateParticlesWithShapesForGroup(
		const b2Shape* const* shapes, int32 shapeCount,
		const b2ParticleGroupDef& groupDef, const b2Transform& xf);
	int32 CloneParticle(int32 index, int32 groupIdx);
	void DestroyParticleGroup(int32 groupIdx);

	void UpdatePairsAndTriads(
		int32 firstIndex, int32 lastIndex, const ConnectionFilter& filter);
	void AFUpdatePairsAndTriads(
		int32 firstIndex, int32 lastIndex, const AFConnectionFilter& filter);
	void UpdatePairsAndTriadsWithReactiveParticles();
	void AFUpdatePairsAndTriadsWithReactiveParticles();
	static void InitializeParticleLists(
		const b2ParticleGroup* group, ParticleListNode* nodeBuffer);
	void MergeParticleListsInContact(
		const b2ParticleGroup* group, ParticleListNode* nodeBuffer) const;
	static void MergeParticleLists(
		ParticleListNode* listA, ParticleListNode* listB);
	static ParticleListNode* FindLongestParticleList(
		const b2ParticleGroup* group, ParticleListNode* nodeBuffer);
	void MergeZombieParticleListNodes(
		const b2ParticleGroup* group, ParticleListNode* nodeBuffer,
		ParticleListNode* survivingList) const;
	static void MergeParticleListAndNode(
		ParticleListNode* list, ParticleListNode* node);
	void CreateParticleGroupsFromParticleList(
		const b2ParticleGroup* group, ParticleListNode* nodeBuffer,
		const ParticleListNode* survivingList);
	void UpdatePairsAndTriadsWithParticleList(
		const b2ParticleGroup* group, const ParticleListNode* nodeBuffer);

	void ComputeDepth();
	void AFComputeDepth();

	InsideBoundsEnumerator GetInsideBoundsEnumerator(const b2AABB& aabb) const;
	af::array AFGetInsideBoundsEnumerator(const b2AABB& aabb) const;

	void UpdateAllParticleFlags();
	void AFUpdateAllParticleFlags();
	void UpdateAllGroupFlags();
	void b2ParticleSystem::AddContact(int32 a, int32 b, int32& contactCount);
	bool b2ParticleSystem::ShouldCollide(int32 a, int32 b) const;
	bool b2ParticleSystem::ShouldCollide(int32 i, b2Fixture* f) const;
	af::array b2ParticleSystem::AFShouldCollide(const af::array& a, const af::array& b) const;
	af::array b2ParticleSystem::AFShouldCollide(const af::array& a, b2Fixture* f) const;
	void b2ParticleSystem::AFAddContacts(const af::array& a, const af::array& b);
	af::array afInvSqrt(af::array x);
	void FindContacts(int32& contactCount);
	void AFFindContacts();
	void AFFindContactsbyTags(vector<int32>& aIdxs, vector<int32>& bIdxs);
	void UpdateProxies();
	void SortProxies();
	void AFSort(af::array& keys, af::array& array, bool ascending);
	void AFSort(af::array& keys, vector<af::array>& arrays);
	void AFReorder(af::array& idxs, vector<af::array>& arrays);
	void AFJoin(af::array& array, const af::array& newArray, af::dtype type = af::dtype::f32);
	template <class UnaryPredicate>
	static vector<int32> GetValidIdxs(int32 vSize, UnaryPredicate isValidFunc);
	template <class UnaryPredicate>
	vector<int32> GetSortedIdxs(int32 vSize, const UnaryPredicate& compFunc);
	af::array AFGetSortedPairIdxs() const;
	template <class UnaryPredicate>
	vector<int32> GetStableSortedIdxs(int32 vSize, const UnaryPredicate compFunc);
	template <class UnaryPredicate>
	vector<int32> GetUniqueIdxs(int32 vSize, const UnaryPredicate compFunc);
	af::array AFGetUniquePairIdxs();
	af::array AFGetUniqueTriadIdxs();
	template<class T>
	void reorder(vector<T>& v, const vector<int32>& order);
	void FilterContacts();
	void NotifyContactListenerPreContact(
		b2ParticlePairSet* particlePairs) const;
	void NotifyContactListenerPostContact(b2ParticlePairSet& particlePairs);
	void UpdateContacts(bool exceptZombie);
	void AFUpdateContacts(bool exceptZombie);
	void NotifyBodyContactListenerPreContact(
		FixtureParticleSet* fixtureSet) const;
	void NotifyBodyContactListenerPostContact(FixtureParticleSet& fixtureSet);
	void UpdateBodyContacts();
	void AFUpdateBodyContacts();

	void CopyParticlesToGPU();
	void CopyParticlesToCPU();
	void CopyProxiesToGPU();
	void CopyProxiesToCPU();
	void CopyGroupsToGPU();
	void CopyGroupsToCPU();
	void CopyMatsToGPU();
	void CopyMatsToCPU();
	void CopyContactsToGPU();
	void CopyContactsToCPU();
	void CopyBodyContactsToGPU();
	void CopyBodyContactsToCPU();
	void CopyBodiesToGPU();

	void SolveCollision(const b2TimeStep& step);
	void AFSolveCollision(const b2TimeStep& step);
	void LimitVelocity(const b2TimeStep& step);
	void SolveGravity(const b2TimeStep& step);
	void AFSolveGravity(const b2TimeStep& step);
	void SolveBarrier(const b2TimeStep& step);
	void SolveStaticPressure(const b2TimeStep& step);
	void AFSolveStaticPressure(const b2TimeStep& step);
	void ComputeWeight();
	void AFComputeWeight();
	void SolvePressure(const b2TimeStep& step);
	void AFSolvePressure(const b2TimeStep& step);
	void SolveDamping(const b2TimeStep& step); 
	void SolveSlowDown(const b2TimeStep& step);
	void AFSolveSlowDown(const b2TimeStep& step);
	void SolveRigidDamping();
	void SolveExtraDamping();
	void SolveWall();
	void SolveRigid(const b2TimeStep& step);
	void AFSolveRigid(const b2TimeStep& step);
	void SolveElastic(const b2TimeStep& step);
	void AFSolveElastic(const b2TimeStep& step);
	void SolveSpring(const b2TimeStep& step);
	void SolveTensile(const b2TimeStep& step);
	void AFSolveTensile(const b2TimeStep& step);
	void SolveViscous();
	void AFSolveViscous();
	void SolveRepulsive(const b2TimeStep& step);
	void AFSolveRepulsive(const b2TimeStep& step);
	void SolvePowder(const b2TimeStep& step);
	void AFSolvePowder(const b2TimeStep& step);
	void SolveSolid(const b2TimeStep& step);
	void AFSolveSolid(const b2TimeStep& step);
	void SolveForce(const b2TimeStep& step);
	void AFSolveForce(const b2TimeStep& step);
	void SolveColorMixing();
	void SolveHeatConduct(const b2TimeStep& step);
	void SolveLooseHeat(const b2TimeStep& step);
	void SolveFlame(const b2TimeStep& step);
	void SolveBurning(const b2TimeStep& step);
	void SolveIgnite();
	void SolveExtinguish();
	void SolveWater();
	void SolveEvaporate();
	void SolveFreeze();
	void SolveDestroyDead();
	void SolveZombie();
	void AFSolveZombie();
	void SolveFalling(const b2TimeStep& step);
	void SolveRising(const b2TimeStep& step);
	/// Destroy all particles which have outlived their lifetimes set by
	/// SetParticleLifetime().
	void SolveLifetimes(const b2TimeStep& step);
	void AFSolveLifetimes(const b2TimeStep& step);
	void RotateBuffer(int32 start, int32 mid, int32 end);
	void AFRotateBuffer(int32 start, int32 mid, int32 end);

	template <class T1, class UnaryPredicate>
	static void RemoveFromVectorIf(vector<T1>& vectorToTest,
		int32& size, UnaryPredicate pred, bool adjustSize);
	template <class T1, class T2, class UnaryPredicate>
	static void RemoveFromVectorsIf(vector<T1>& vectorToTest, vector<T2>& v2,
		int32& size, UnaryPredicate pred, bool adjustSize);
	template <class T1, class T2, class T3, class T4, class T5, class T6, class T7, class UnaryPredicate>
	static void RemoveFromVectorsIf(
		vector<T1>& v1, vector<T2>& v2, vector<T3>& v3, vector<T4>& v4, vector<T5>& v5, vector<T6>& v6, vector<T7>& v7,
		int32& size, UnaryPredicate pred, bool adjustSize);
	template <class T1, class T2, class T3, class T4, class T5, class T6, class T7, class T8, class UnaryPredicate>
	static void RemoveFromVectorsIf(
		vector<T1>& v1, vector<T2>& v2, vector<T3>& v3, vector<T4>& v4, vector<T5>& v5, vector<T6>& v6, vector<T7>& v7, vector<T8>& v8,
		int32& size, UnaryPredicate pred, bool adjustSize);
	template <class T1, class T2, class T3, class T4, class T5, class T6, class T7, class T8, class UnaryPredicate1, class UnaryPredicate2> 
	static void RemoveFromVectorsIf(
		vector<T1>& v1, vector<T2>& v2, vector<T3>& v3, vector<T4>& v4, vector<T5>& v5, vector<T6>& v6, vector<T7>& v7, vector<T8>& v8,
		int32& size, UnaryPredicate1 pred1, UnaryPredicate2 pred2, bool adjustSize);

	float32 GetCriticalVelocity(const b2TimeStep& step) const;
	float32 GetCriticalVelocitySquared(const b2TimeStep& step) const;
	float32 GetCriticalPressure(const b2TimeStep& step) const;
	float32 GetParticleStride() const;

	// Get the world's contact filter if any particles with the
	// b2_contactFilterParticle flag are present in the system.
	b2ContactFilter* GetFixtureContactFilter() const;

	// Get the world's contact filter if any particles with the
	// b2_particleContactFilterParticle flag are present in the system.
	b2ContactFilter* GetParticleContactFilter() const;

	// Get the world's contact listener if any particles with the
	// b2_fixtureContactListenerParticle flag are present in the system.
	b2ContactListener* GetFixtureContactListener() const;

	// Get the world's contact listener if any particles with the
	// b2_particleContactListenerParticle flag are present in the system.
	b2ContactListener* GetParticleContactListener() const;

	template <typename T> void SetUserOverridableBuffer(
		UserOverridableBuffer<T>* buffer, T* newBufferData, int32 newCapacity);

	void SetGroupFlags(int32 groupIdx, uint32 flags);
	void AFSetGroupFlags(const af::array& afGroupIdxs, af::array& afNewFlags);

	void RemoveSpuriousBodyContacts();
	//static bool BodyContactCompare(int32 lhsIdx, int32 rhsIdx);

	void DetectStuckParticle(int32 particle);
	void AFDetectStuckParticle(const af::array& particles);

	/// Determine whether a particle index is valid.
	bool ValidateParticleIndex(const int32 index) const;

	/// Get the time elapsed in b2ParticleSystemDef::lifetimeGranularity.
	int32 GetQuantizedTimeElapsed() const;
	/// Convert a lifetime in seconds to an expiration time.
	int64 LifetimeToExpirationTime(const float32 lifetime) const;

	bool ForceCanBeApplied(uint32 flags) const;
	af::array AFForceCanBeApplied(const af::array& flags) const;
	void PrepareForceBuffer();
	void AFPrepareForceBuffer();

	bool IsRigidGroup(int32 groupIdx) const;
	b2Vec2 GetLinearVelocity(
		int32 groupIdx, int32 particleIndex,
		const b2Vec2 &point);
	void InitDampingParameter(
		float32* invMass, float32* invInertia, float32* tangentDistance,
		float32 mass, float32 inertia, const b2Vec2& center,
		const b2Vec2& point, const b2Vec2& normal) const;
	void InitDampingParameterWithRigidGroupOrParticle(
		float32* invMass, float32* invInertia, float32* tangentDistance,
		bool isRigidGroup, int32 groupIdx, int32 particleIndex,
		const b2Vec2& point, const b2Vec2& normal);
	float32 ComputeDampingImpulse(
		float32 invMassA, float32 invInertiaA, float32 tangentDistanceA,
		float32 invMassB, float32 invInertiaB, float32 tangentDistanceB,
		float32 normalVelocity) const;
	void ApplyDamping(
		float32 invMass, float32 invInertia, float32 tangentDistance,
		bool isRigidGroup, int32 groupIdx, int32 particleIndex,
		float32 impulse, float32 normalX, float32 normalY);

	vector<b2Body*>& m_bodyBuffer;
	vector<b2Fixture*>& m_fixtureBuffer;

	bool m_paused;
	int32 m_timestamp;
	int32 m_allParticleFlags;
	bool m_needsUpdateAllParticleFlags;
	int32 m_allGroupFlags;
	bool m_needsUpdateAllGroupFlags;
	bool m_hasForce;
	int32 m_iterationIndex;
	float32 m_inverseDensity;
	float32 m_particleDiameter;
	float32 m_inverseDiameter;
	float32 m_squaredDiameter;

	float32 m_roomTemp;
	float32 m_heatLossRatio;

	float32 m_pointsPerLayer;
	float32 m_invPointsPerLayer;
	float32 m_lowestPoint;
	float32 m_layerGraphB;

	int32 m_count;
	int32 m_particleBufferSize;
	int32 m_groupCount;
	int32 m_groupBufferSize;
	int32 m_partMatCount;
	int32 m_partMatBufferSize;
	int32 m_contactCount;
	int32 m_contactBufferSize;
	int32 m_bodyContactCount;
	int32 m_bodyContactBufferSize;
	int32 m_pairCount;
	int32 m_pairBufferSize;
	int32 m_triadCount;
	int32 m_triadBufferSize;

	bool afHasColorBuf,
		 afHasUserDataBuf,
		 afHasHandleIdxBuf,
		 afHasStaticPresBuf,
		 afHasExpireTimeBuf,
		 afHasIdxByExpireTimeBuf,
		 afHasAccumulation2XBuf,
		 afHasAccumulation2YBuf,
		 afHasDepthBuf,
		 afHasLastBodyContactBuf,
		 afHasBodyContactCountBuf,
		 afHasConsecutiveContactStepsBuf;

	/// Allocator for b2ParticleHandle instances.
	b2SlabAllocator<b2ParticleHandle> m_handleAllocator;
	/// Maps particle indicies to  handles.
	bool hasHandleIndexBuffer;
	vector<b2ParticleHandle*> m_handleIndexBuffer;
	vector<uint32>	m_flagsBuffer,
					m_collisionLayerBuffer;
	vector<float32> m_positionXBuffer,
					m_positionYBuffer,
					m_positionZBuffer,
					m_velocityXBuffer,
					m_velocityYBuffer,
					m_weightBuffer,
					m_heatBuffer,
					m_healthBuffer,
					m_forceXBuffer,
					m_forceYBuffer;
	vector<int32>	m_colorBuffer;

	vector<int32>	m_partMatIdxBuffer;
	vector<int32>   m_groupIdxBuffer;

	vector<int32>	m_freeGroupIdxBuffer;
	vector<int32>	m_realGroupIdxBuffer;

	//vector<b2ParticleGroup> m_groupBuffer;
	vector<int32>	m_groupFirstIdxBuf,
					m_groupLastIdxBuf;
	vector<uint32>	m_groupFlagsBuf;
	vector<int32>	m_groupColGroupBuf;
	vector<float32>	m_groupStrengthBuf;
	vector<int32>	m_groupMatIdxBuf;
	vector<int32>	m_groupTimestampBuf;
	vector<float32>	m_groupMassBuf,
					m_groupInertiaBuf,
					m_groupCenterXBuf,
					m_groupCenterYBuf,
					m_groupLinVelXBuf,
					m_groupLinVelYBuf;
	vector<float32> m_groupAngVelBuf;
	vector<b2Transform> m_groupTransformBuf;
	vector<int32>	m_groupUserDataBuf;
			
	vector<uint32>  m_partMatFlagsBuf;
	vector<float32> m_partMatMassBuf;
	vector<float32> m_partMatInvMassBuf;
	vector<float32> m_partMatStabilityBuf;
	vector<float32> m_partMatInvStabilityBuf;
	vector<float32> m_partMatExtinguishingPointBuf;
	vector<float32> m_partMatMeltingPointBuf;
	vector<float32> m_partMatBoilingPointBuf;
	vector<float32> m_partMatIgnitionPointBuf;
	vector<float32> m_partMatHeatConductivityBuf;

	/// When any particles have the flag b2_staticPressureParticle,
	/// m_staticPressureBuffer is first allocated and used in
	/// SolveStaticPressure() and SolvePressure().  It will be reallocated on
	/// subsequent CreateParticle() calls.
	bool hasStaticPressureBuf;
	vector<float32> m_staticPressureBuf;
	/// m_accumulationBuffer is used in many functions as a temporary buffer
	/// for scalar values.
	vector<float32> m_accumulationBuf;
	/// When any particles have the flag b2_tensileParticle,
	/// m_accumulation2Buffer is first allocated and used in SolveTensile()
	/// as a temporary buffer for vector values.  It will be reallocated on
	/// subsequent CreateParticle() calls.
	bool hasAccumulation2Buf;
	vector<b2Vec2> m_accumulation2Buf;
	/// When any particle groups have the flag b2_solidParticleGroup,
	/// m_depthBuffer is first allocated and populated in ComputeDepth() and
	/// used in SolveSolid(). It will be reallocated on subsequent
	/// CreateParticle() calls.
	vector<float32> m_depthBuffer;
	vector<int32> m_userDataBuffer;

	af::array	
		afPosXBuf, afPosYBuf, afPosZBuf,
		afFlagBuf,
		afColLayBuf,
		afVelXBuf, afVelYBuf,
		afForceXBuf, afForceYBuf,
		afWeightBuf,
		afHeatBuf,
		afHealthBuf,
		afColorBuf,
		afUserDataBuf, //TODO add to copy
		afGroupIdxBuf,
		afPartMatIdxBuf,

		afAccumulationBuf,
		afAccumulation2XBuf,
		afAccumulation2YBuf,
		afStaticPressureBuf,
		afDepthBuf,

		afExpireTimeBuf,
		afIdxByExpireTimeBuf,
		afHandleIdxBuf,
		
		afLastBodyContactStepBuf,
		afBodyContactCountBuf,
		afConsecutiveContactStepsBuf,
		afStuckParticleBuf,

		afProxyIdxBuf,
		afProxyTagBuf,

		//afFreeGroupIdxBuffer,
		afRealGroupIdxBuf,

		afGroupFirstIdxBuf, afGroupLastIdxBuf,
		afGroupFlagsBuf,
		afGroupColGroupBuf,
		afGroupStrengthBuf,
		afGroupMatIdxBuf,
		afGroupTimestampBuf,
		afGroupMassBuf,
		afGroupInertiaBuf,
		afGroupCenterXBuf, afGroupCenterYBuf,
		afGroupLinVelXBuf, afGroupLinVelYBuf,
		afGroupAngVelBuf,
		afGroupTransformBuf,
		afGroupUserDataBuf,

		afContactIdxABuf, afContactIdxBBuf,
		afContactWeightBuf, afContactMassBuf,
		afContactNormalXBuf, afContactNormalYBuf,
		afContactFlagsBuf, afContactMatFlagsBuf,

		afBodyContactIdxBuf,
		afBodyContactBodyIdxBuf,
		afBodyContactFixtIdxBuf,
		afBodyContactWeightBuf,
		afBodyContactNormalXBuf, afBodyContactNormalYBuf,
		afBodyContactMassBuf,

		afPartMatFlagsBuf,
		afPartMatMassBuf, afPartMatInvMassBuf,
		afPartMatStabilityBuf,
		afPartMatInvStabilityBuf,
		afPartMatExtgshPntBuf,
		afPartMatMeltingPntBuf,
		afPartMatBoilingPntBuf,
		afPartMatIgnitPntBuf,
		afPartMatHeatCondBuf,

		afPairIdxABuf, afPairIdxBBuf,
		afPairFlagsBuf,
		afPairStrengthBuf,
		afPairDistanceBuf,

		afTriadIdxABuf, afTriadIdxBBuf, afTriadIdxCBuf,
		afTriadFlagsBuf,
		afTriadStrengthBuf,
		afTriadPAXBuf, afTriadPAYBuf,
		afTriadPBXBuf, afTriadPBYBuf,
		afTriadPCXBuf, afTriadPCYBuf,
		afTriadKABuf, afTriadKBBuf, afTriadKCBuf, afTriadSBuf;







				

	/// Stuck particle detection parameters and record keeping
	int32 m_stuckThreshold;
	int32 m_stuckParticleCount;
	bool hasLastBodyContactStepBuffer;
	bool hasBodyContactCountBuffer;
	bool hasConsecutiveContactStepsBuffer;
	vector<int32> m_lastBodyContactStepBuffer;
	vector<int32> m_bodyContactCountBuffer;
	vector<int32> m_consecutiveContactStepsBuffer;
	vector<int32> m_stuckParticleBuffer;

	vector<int32> m_proxyIdxBuffer;
	vector<uint32> m_proxyTagBuffer;

	vector<int32>	m_contactIdxABuffer;
	vector<int32>	m_contactIdxBBuffer;
	vector<float32> m_contactWeightBuffer;
	vector<float32> m_contactMassBuffer;
	vector<float32> m_contactNormalXBuffer;
	vector<float32> m_contactNormalYBuffer;
	vector<uint32>	m_contactFlagsBuffer;
	vector<uint32>	m_contactMatFlagsBuffer;

	vector<int32>	m_bodyContactIdxBuffer;
	vector<int32>	m_bodyContactBodyIdxBuffer;
	vector<int32>	m_bodyContactFixtureIdxBuffer;
	vector<float32>	m_bodyContactWeightBuffer;
	vector<float32>	m_bodyContactNormalXBuffer;
	vector<float32>	m_bodyContactNormalYBuffer;
	vector<float32>	m_bodyContactMassBuffer;

	//vector<b2ParticleBodyContact> m_bodyContactBuffer;

	//b2GrowableBuffer<b2ParticlePair> m_pairBuffer;
	vector<int32>   m_pairIdxABuf, m_pairIdxBBuf;
	vector<uint32>  m_pairFlagsBuf;
	vector<float32> m_pairStrengthBuf;	   /// The strength of cohesion among the particles.
	vector<float32> m_pairDistanceBuf;	   /// The initial distance of the particles.
					
	//b2GrowableBuffer<b2ParticleTriad> m_triadBuffer;
	vector<int32>   m_triadIdxABuf, m_triadIdxBBuf, m_triadIdxCBuf;
	vector<uint32>  m_triadFlagsBuf;		/// The logical sum of the particle flags. See the b2ParticleFlag enum.
	vector<float32> m_triadStrengthBuf;		/// The strength of cohesion among the particles.
	vector<float32> m_triadPAXBuf, m_triadPAYBuf, 	/// Values used for calculation.
				    m_triadPBXBuf, m_triadPBYBuf,
				    m_triadPCXBuf, m_triadPCYBuf;
	vector<float32> m_triadKABuf, m_triadKBBuf, m_triadKCBuf, m_triadSBuf;

	/// Time each particle should be destroyed relative to the last time
	/// m_timeElapsed was initialized.  Each unit of time corresponds to
	/// b2ParticleSystemDef::lifetimeGranularity seconds.
	vector<int32> m_expireTimeBuf;
	/// List of particle indices sorted by expiration time.
	vector<int32> m_idxByExpireTimeBuf;
	/// Time elapsed in 32:32 fixed point.  Each non-fractional unit of time
	/// corresponds to b2ParticleSystemDef::lifetimeGranularity seconds.
	int64 m_timeElapsed;
	/// Whether the expiration time buffer has been modified and needs to be
	/// resorted.
	bool m_expirationTimeBufferRequiresSorting;

	b2ParticleSystemDef m_def;

	b2World* m_world;
	b2ParticleSystem* m_prev;
	b2ParticleSystem* m_next;
};

inline void b2ParticleContact::SetIndices(int32 a, int32 b)
{
	b2Assert(a <= b2_maxParticleIndex && b <= b2_maxParticleIndex);
	indexA = (b2ParticleIndex)a;
	indexB = (b2ParticleIndex)b;
}


inline bool b2ParticleContact::operator==(
	const b2ParticleContact& rhs) const
{
	return indexA == rhs.indexA
		&& indexB == rhs.indexB
		&& flags == rhs.flags
		&& weight == rhs.weight
		&& normal == rhs.normal;
}

// The reciprocal sqrt function differs between SIMD and non-SIMD, but they
// should create approximately equal results.
inline bool b2ParticleContact::ApproximatelyEqual(
	const b2ParticleContact& rhs) const
{
	static const float MAX_WEIGHT_DIFF = 0.01f; // Weight 0 ~ 1, so about 1%
	static const float MAX_NORMAL_DIFF = 0.01f; // Normal length = 1, so 1%
	return indexA == rhs.indexA
		&& indexB == rhs.indexB
		&& flags == rhs.flags
		&& b2Abs(weight - rhs.weight) < MAX_WEIGHT_DIFF
		&& (normal - rhs.normal).Length() < MAX_NORMAL_DIFF;
}

inline int32 b2ParticleSystem::GetParticleGroupCount() const
{
	return m_groupCount;
}

inline int32 b2ParticleSystem::GetParticleCount() const
{
	return m_count;
}

inline void b2ParticleSystem::SetPaused(bool paused)
{
	m_paused = paused;
}

inline bool b2ParticleSystem::GetPaused() const
{
	return m_paused;
}

inline const int32* b2ParticleSystem::GetContactIdxAs() const
{
	return m_contactIdxABuffer.data();
}
inline const int32* b2ParticleSystem::GetContactIdxBs() const
{
	return m_contactIdxBBuffer.data();
}

inline const int32 b2ParticleSystem::GetContactCount() const
{
	return m_contactCount;
}

inline b2Body** b2ParticleSystem::GetBodyBuffer()
{
	return m_bodyBuffer.data();
}
inline b2Fixture** b2ParticleSystem::GetFixtureBuffer()
{
	return m_fixtureBuffer.data();
}

inline const int32* b2ParticleSystem::GetBodyContactIdxs() const
{
	return m_bodyContactIdxBuffer.data();
}
inline const int32* b2ParticleSystem::GetBodyContactBodyIdxs() const
{
	return m_bodyContactBodyIdxBuffer.data();
}
inline const int32* b2ParticleSystem::GetBodyContactFixtureIdxs()const
{
	return m_bodyContactFixtureIdxBuffer.data();
}
inline const float32* b2ParticleSystem::GetBodyContactNormalXs() const
{
	return m_bodyContactNormalXBuffer.data();
}
inline const float32* b2ParticleSystem::GetBodyContactNormalYs() const
{
	return m_bodyContactNormalYBuffer.data();
}
inline const float32* b2ParticleSystem::GetBodyContactWeights() const
{
	return m_bodyContactWeightBuffer.data();
}

inline int32 b2ParticleSystem::GetBodyContactCount() const
{
	return m_bodyContactCount;
}

/*
inline const b2ParticlePair* b2ParticleSystem::GetPairs() const
{
	return m_pairBuffer.Data();
}

inline const b2ParticleTriad* b2ParticleSystem::GetTriads() const
{
	return m_triadBuffer.Data();
}*/

inline b2ParticleSystem* b2ParticleSystem::GetNext()
{
	return m_next;
}

inline const b2ParticleSystem* b2ParticleSystem::GetNext() const
{
	return m_next;
}

inline const int32* b2ParticleSystem::GetStuckCandidates() const
{
	return m_stuckParticleBuffer.data();
}

inline int32 b2ParticleSystem::GetStuckCandidateCount() const
{
	return m_stuckParticleCount;
}

inline void b2ParticleSystem::SetStrictContactCheck(bool enabled)
{
	m_def.strictContactCheck = enabled;
}

inline bool b2ParticleSystem::GetStrictContactCheck() const
{
	return m_def.strictContactCheck;
}



inline void b2ParticleSystem::SetRoomTemperature(float32 roomTemp)
{
	m_roomTemp = roomTemp;
}
inline float32 b2ParticleSystem::GetRoomTemperature() const
{
	return m_roomTemp;
}

inline void b2ParticleSystem::SetHeatLossRatio(float32 heatLossRatio)
{
	m_heatLossRatio = heatLossRatio;
}

inline void b2ParticleSystem::SetPointsPerLayer(float32 pointsPerLayer)
{
	m_pointsPerLayer = pointsPerLayer;
}

inline void b2ParticleSystem::SetLowestPoint(float32 lowestPoint)
{
	m_lowestPoint = lowestPoint;
}

inline void b2ParticleSystem::CalcLayerValues()
{
	m_invPointsPerLayer = 1 / m_pointsPerLayer;
	m_layerGraphB = -m_lowestPoint / m_pointsPerLayer;
}


inline void b2ParticleSystem::SetRadius(float32 radius)
{
	m_particleDiameter = 2 * radius;
	m_squaredDiameter = m_particleDiameter * m_particleDiameter;
	m_inverseDiameter = 1 / m_particleDiameter;
}

inline void b2ParticleSystem::SetDensity(float32 density)
{
	m_def.density = density;
	m_inverseDensity =  1 / m_def.density;
}

inline float32 b2ParticleSystem::GetDensity() const
{
	return m_def.density;
}

inline void b2ParticleSystem::SetGravityScale(float32 gravityScale)
{
	m_def.gravityScale = gravityScale;
}

inline float32 b2ParticleSystem::GetGravityScale() const
{
	return m_def.gravityScale;
}

inline void b2ParticleSystem::SetDamping(float32 damping)
{
	m_def.dampingStrength = damping;
}

inline float32 b2ParticleSystem::GetDamping() const
{
	return m_def.dampingStrength;
}

inline void b2ParticleSystem::SetStaticPressureIterations(int32 iterations)
{
	m_def.staticPressureIterations = iterations;
}

inline int32 b2ParticleSystem::GetStaticPressureIterations() const
{
	return m_def.staticPressureIterations;
}

inline float32 b2ParticleSystem::GetRadius() const
{
	return m_particleDiameter / 2;
}

inline float32 b2ParticleSystem::GetCriticalVelocity(const b2TimeStep& step) const
{
	return m_particleDiameter * step.inv_dt;
}

inline float32 b2ParticleSystem::GetCriticalVelocitySquared(
	const b2TimeStep& step) const
{
	float32 velocity = GetCriticalVelocity(step);
	return velocity * velocity;
}

inline float32 b2ParticleSystem::GetCriticalPressure(const b2TimeStep& step) const
{
	return m_def.density * GetCriticalVelocitySquared(step);
}

inline float32 b2ParticleSystem::GetParticleStride() const
{
	return b2_particleStride * m_particleDiameter;
}

inline float32 b2ParticleSystem::GetParticleMass() const
{
	float32 stride = GetParticleStride();
	return m_def.density * stride * stride;
}

inline float32 b2ParticleSystem::GetParticleInvMass() const
{
	// mass = density * stride^2, so we take the inverse of this.
	float32 inverseStride = m_inverseDiameter * (1.0f / b2_particleStride);
	return m_inverseDensity * inverseStride * inverseStride;
}

inline float32* b2ParticleSystem::GetPositionXBuffer()
{
	return m_positionXBuffer.data();
}
inline float32* b2ParticleSystem::GetPositionYBuffer()
{
	return m_positionYBuffer.data();
}
inline float32* b2ParticleSystem::GetPositionZBuffer()
{
	return m_positionZBuffer.data();
}
inline const float* b2ParticleSystem::GetPositionXBuffer() const
{
	return m_positionXBuffer.data();
}
inline const float* b2ParticleSystem::GetPositionYBuffer() const
{
	return m_positionYBuffer.data();
}
inline const float* b2ParticleSystem::GetPositionZBuffer() const
{
	return m_positionZBuffer.data();
}

inline float32* b2ParticleSystem::GetVelocityXBuffer()
{
	return m_velocityXBuffer.data();
}
inline float32* b2ParticleSystem::GetVelocityYBuffer()
{
	return m_velocityYBuffer.data();
}
inline const float32* b2ParticleSystem::GetVelocityXBuffer() const
{
	return m_velocityXBuffer.data();
}
inline const float32* b2ParticleSystem::GetVelocityYBuffer() const
{
	return m_velocityYBuffer.data();
}

inline float32* b2ParticleSystem::GetWeightBuffer()
{
	return m_weightBuffer.data();
}

inline const float32* b2ParticleSystem::GetHeatBuffer() const
{
	return m_heatBuffer.data();
}

inline float32* b2ParticleSystem::GetHeatBuffer()
{
	return m_heatBuffer.data();
}

inline const float32* b2ParticleSystem::GetHealthBuffer() const
{
	return m_healthBuffer.data();
}

inline float32* b2ParticleSystem::GetHealthBuffer()
{
	return m_healthBuffer.data();
}

inline int32 b2ParticleSystem::GetMaxParticleCount() const
{
	return m_def.maxCount;
}

inline void b2ParticleSystem::SetMaxParticleCount(int32 count)
{
	b2Assert(m_count <= count);
	m_def.maxCount = count;
}

inline uint32 b2ParticleSystem::GetAllParticleFlags() const
{
	return m_allParticleFlags;
}

inline uint32 b2ParticleSystem::GetAllGroupFlags() const
{
	return m_allGroupFlags;
}

inline const uint32* b2ParticleSystem::GetFlagsBuffer() const
{
	return m_flagsBuffer.data();
}

inline const uint32* b2ParticleSystem::GetCollisionLayerBuffer() const
{
	return m_collisionLayerBuffer.data();
}
inline uint32* b2ParticleSystem::GetCollisionLayerBuffer()
{
	return m_collisionLayerBuffer.data();
}

inline const int32* b2ParticleSystem::GetColorBuffer() const
{
	return m_colorBuffer.data();
}

inline const int32* b2ParticleSystem::GetGroupIdxBuffer() const
{
	return m_groupIdxBuffer.data();
}
inline int32* b2ParticleSystem::GetGroupIdxBuffer()
{
	return m_groupIdxBuffer.data();
}


inline const float32* b2ParticleSystem::GetWeightBuffer() const
{
	return m_weightBuffer.data();
}

inline const int32* b2ParticleSystem::GetUserDataBuffer() const
{
	return ((b2ParticleSystem*) this)->GetUserDataBuffer();
}

inline uint32 b2ParticleSystem::GetParticleFlags(int32 index)
{
	return GetFlagsBuffer()[index];
}

inline bool b2ParticleSystem::ValidateParticleIndex(const int32 index) const
{
	return index >= 0 && index < GetParticleCount() &&
		index != b2_invalidParticleIndex;
}

inline bool b2ParticleSystem::GetDestructionByAge() const
{
	return m_def.destroyByAge;
}

inline void b2ParticleSystem::ParticleApplyLinearImpulse(int32 index,
														 const b2Vec2& impulse)
{
	ApplyLinearImpulse(index, index + 1, impulse);
}


// Note: These functions must go in the header so the unit tests will compile
// them. b2ParticleSystem.cpp does not compile with this #define.
#if LIQUIDFUN_EXTERNAL_LANGUAGE_API

inline void b2ParticleSystem::SetParticleVelocity(int32 index,
												  float32 vx,
												  float32 vy)
{
	b2Vec2& v = GetVelocityBuffer()[index];
	v.x = vx;
	v.y = vy;
}

inline float b2ParticleSystem::GetParticlePositionX(int32 index) const
{
	return GetPositionBuffer()[index].x;
}

inline float b2ParticleSystem::GetParticlePositionY(int32 index) const
{
	return GetPositionBuffer()[index].y;
}

inline int b2ParticleSystem::CopyPositionBuffer(int startIndex,
												int numParticles,
												void* outBuf,
												int size) const
{
	int copySize = numParticles * sizeof(b2Vec2);
	void* inBufWithOffset = (void*) (GetPositionBuffer() + startIndex);
	return CopyBuffer(startIndex, numParticles, inBufWithOffset, outBuf, size,
					  copySize);
}

inline int b2ParticleSystem::CopyColorBuffer(int startIndex,
											 int numParticles,
											 void* outBuf,
											 int size) const
{
	int copySize = numParticles * sizeof(b2ParticleColor);
	void* inBufWithOffset = (void*) (GetColorBuffer() + startIndex);
	return CopyBuffer(startIndex, numParticles, inBufWithOffset, outBuf, size,
					  copySize);
}

inline int b2ParticleSystem::CopyWeightBuffer(int startIndex,
											  int numParticles,
											  void* outBuf,
											  int size) const
{
	int copySize = numParticles * sizeof(float32);
	void* inBufWithOffset = (void*) (GetWeightBuffer() + startIndex);
	return CopyBuffer(startIndex, numParticles, inBufWithOffset, outBuf, size,
					  copySize);
}

inline int b2ParticleSystem::CopyBuffer(int startIndex, int numParticles,
										void* inBufWithOffset, void* outBuf,
										int outBufSize, int copySize) const
{
	b2ExceptionType exception = IsBufCopyValid(startIndex, numParticles,
											   copySize, outBufSize);
	if (exception != b2_noExceptions)
	{
		return exception;
	}

	memcpy(outBuf, inBufWithOffset, copySize);
	return b2_noExceptions;
}

#endif // LIQUIDFUN_EXTERNAL_LANGUAGE_API

#endif

