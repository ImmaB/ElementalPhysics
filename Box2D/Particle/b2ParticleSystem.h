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
#pragma once

#include <Box2D/Common/b2SlabAllocator.h>
#include <Box2D/Common/b2GrowableBuffer.h>
#include <Box2D/Common/b2GlobalVariables.h>
#include <Box2D/Particle/b2Particle.h>
#include <Box2D/Particle/b2ParticleGroup.h>
#include <Box2D/Amp/ampAlgorithms.h>
#include <Box2D/Dynamics/b2TimeStep.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Collision/Shapes/b2CircleShape.h>
#include <Box2D/Collision/Shapes/b2EdgeShape.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>
#include <Box2D/Collision/Shapes/b2ChainShape.h>
#include <vector>
#include <array> 
#include <numeric>
#include <process.h>
#include <Windows.h>
#include <amp.h>

class b2World;
struct b2Shape;
struct ParticleGroup;
class b2BlockAllocator;
class b2StackAllocator;
class b2QueryCallback;
class b2RayCastCallback;
class b2ContactFilter;
class b2ContactListener;
class b2ParticlePairSet;
class FixtureParticleSet;
struct Vec2;
struct b2AABB;
struct FindContactInput;
struct FindContactCheck;

struct Particle::Mat;
struct ParticleGroup::Def;
struct Body::Mat;

using namespace std;

/// Connection between two particles
struct b2ParticlePair
{
	/// Indices of the respective particles making pair.
	int32 indexA, indexB;

	/// The logical sum of the particle flags. See the b2ParticleFlag enum.
	uint32 flags;

	/// The strength of cohesion among the particles.
	float32 strength;

	/// The initial distance of the particles.
	float32 distance;

	inline bool HasFlag(const uint32 f) const { return flags & f; }
	inline bool HasFlag(const uint32 f) const restrict(amp) { return flags & f; }
};

/// Connection between three particles
struct b2ParticleTriad
{
	/// Indices of the respective particles making triad.
	int32 indexA, indexB, indexC;

	/// The logical sum of the particle flags. See the b2ParticleFlag enum.
	uint32 flags;

	/// The strength of cohesion among the particles.
	float32 strength;

	/// Values used for calculation.
	Vec2 pa, pb, pc;
	float32 ka, kb, kc, s;

	inline bool HasFlag(const uint32 f) const { return flags & f; }
	inline bool HasFlag(const uint32 f) const restrict(amp) { return flags & f; }
};

struct b2ParticleSystemDef
{
	b2ParticleSystemDef()
	{
		accelerate = true;
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

	bool accelerate;

	/// Enable strict Particle/Body contact check.
	/// See SetStrictContactCheck for details.
	/// Set strict Particle/Body contact check.
	/// This is an option that will help ensure correct behavior if there are
	/// corners in the world model where Particle/Body contact is ambiguous.
	/// This option scales at n*log(n) of the number of Particle/Body contacts,
	/// so it is best to only enable if it is necessary for your geometry.
	/// Enable if you see strange particle behavior around b2Body
	/// intersections.
	bool strictContactCheck;

	/// Set the particle density.
	/// See SetDensity for details.
	float32 density;
	float32 invDensity;

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
	/// Damping is used to reduce the velocity of particles. The damping
	/// parameter can be larger than 1.0f but the damping effect becomes
	/// sensitive to the time step when the damping parameter is large.
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
	/// Change the number of iterations when calculating the static pressure of
	/// particles. By default, 8 iterations. You can reduce the number of
	/// iterations down to 1 in some situations, but this may cause
	/// instabilities when many particles come together. If you see particles
	/// popping away from each other like popcorn, you may have to increase the
	/// number of iterations.
	/// For a description of static pressure, see
	/// http://en.wikipedia.org/wiki/Static_pressure#Static_pressure_in_fluid_dynamics
	int32 staticPressureIterations;

	/// Determines how fast colors are mixed
	/// 1.0f ==> mixed immediately
	/// 0.5f ==> mixed half way each simulation step (see b2World::Step())
	float32 colorMixingStrength;

	/// Whether to destroy particles by age when no more particles can be
	/// created.  See #b2ParticleSystem::SetDestructionByAge() for
	/// more information.
	/// Enable / disable destruction of particles in CreateParticle() when
	/// no more particles can be created due to a prior call to
	/// SetMaxParticleCount().  When this is enabled, the oldest particle is
	/// destroyed in CreateParticle() favoring the destruction of particles
	/// with a finite lifetime over particles with infinite lifetimes.
	/// This feature is enabled by default when particle lifetimes are
	/// tracked.  Explicitly enabling this feature using this function enables
	bool destroyByAge;

	/// Granularity of particle lifetimes in seconds.  By default this is
	/// set to (1.0f / 60.0f) seconds.  b2ParticleSystem uses a 32-bit signed
	/// value to track particle lifetimes so the maximum lifetime of a
	/// particle is (2^32 - 1) / (1.0f / lifetimeGranularity) seconds.
	/// With the value set to 1/60 the maximum lifetime or age of a particle is
	/// 2.27 years.
	float32 lifetimeGranularity;

	float32 airResistanceFactor;
	float32 minAirSpeed;

	float32 heatLossRatio;
	float32 flameHealthLossPerSec;
	float32 flameHeatGainPerSec;
};

class ParticleSystem
{
private:
	int32 m_iteration;
	b2TimeStep& m_step;
	b2TimeStep m_subStep;

	HANDLE findBodyContactsThread;


public:
	typedef void(__stdcall* ResizeCallback)(int32);
	ResizeCallback m_resizeCallback;
	b2ParticleSystemDef m_def;
	bool m_accelerate;
	bool m_debugContacts;

	Particle::Buffers m_buffers;
	Particle::AmpArrays m_ampArrays;
	Particle::D11Buffers m_d11Buffers;

	bool ShouldSolve();
	void SolveInit();

	void InitStep();
	void SortProxies();
	void UpdateContacts(bool exceptZombie);
	void ReduceContacts();
	void ComputeDepth();
	void UpdatePairsAndTriadsWithReactiveParticles();

	// Velocity
	void SolveForce();
	void WaitForUpdateBodyContacts();
	void ComputeWeight();
	void SolveViscous();
	void SolveRepulsive();
	void SolvePowder();
	void WaitForComputeWeight();
	void SolveTensile();			// Needs: weight
	void SolveSolid();
	void SolveGravity();			// Needs: Mass		| Modifies: Velocity
	void SolveWind();				// Needs: Mass		| Modifies: Velocity
	void SolveStaticPressure();
	void SolvePressure();
	void SolveDamping();			// Needs: Velocity, Mass, Position | Modifies: Velocity
	void SolveExtraDamping();
	void SolveAirResistance();		// Needs: Mass		| Modifies: Velocity
	void SolveElastic();
	void SolveSpring();
	void LimitVelocity();
	void SolveRigidDamping();
	void SolveBarrier();
	void SolveCollision();			
	void SolveRigid();
	void SolveWall();
	void CopyVelocities();
	void SolveKillNotMoving();		// Needs: Velocity	| Modifies: Flags
	void SolveFluid();				// Needs: Flags		| Modifies: Flags

	// Burning and Heat
	void SolveFlame();
	void SolveIgnite();
	void SolveExtinguish();
	void SolveHeatConduct();
	void SolveLooseHeat();
	void CopyHeats();
	void SolveChangeMat();

	// Find Dead
	void SolveHealth();
	void CopyHealths();
	void SolvePosition();
	void SolveOutOfBounds();		// Needs: Position	| Modifies: Flags
	void CopyFlags();
	void CopyBodies();

	void IncrementIteration();

	void SolveEnd();

public:
	/// Retrieve a handle to the particle at the specified index.
	/// Please see #b2ParticleHandle for why you might want a handle.
	const b2ParticleHandle* GetParticleHandleFromIndex(const int32 index);

	/// Destroy a particle.
	/// The particle is removed after the next step.
	/// @param Index of the particle to destroy.
	/// @param Whether to call the destruction listener just before the
	/// particle is destroyed.
	void DestroyParticle(int32 index);

	void DestroyAllParticles();

	void DestroyParticlesInGroup(const int32 groupIdx);
	void DestroyParticlesInGroup(const ParticleGroup& group);


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
	int32 DestroyParticlesInFixture(const Fixture& fixture, const b2Transform& xf,
	                              bool callDestructionListener = false);

	void ClearMaterials();
	int32 CreateMaterial(Particle::Mat::Def& def);
	void AddPartMatChange(const int32 matIdx, const Particle::Mat::ChangeDef& changeDef);

	/// Create a particle group whose properties have been defined. No
	/// reference to the definition is retained.
	/// @warning This function is locked during callbacks.
	int32 CreateGroup(ParticleGroup::Def& def);
	void DestroyGroup(int32 groupIdx, int32 timestamp = INVALID_IDX, bool destroyParticles = false);

	/// Join two particle groups.
	/// @param the first group. Expands to encompass the second group.
	/// @param the second group. It is destroyed.
	/// @warning This function is locked during callbacks.
	void JoinParticleGroups(int32 groupAIdx, int32 groupBIdx);

	/// Split particle group into multiple disconnected groups.
	/// @param the group to be split.
	/// @warning This function is locked during callbacks.
	void SplitParticleGroup(ParticleGroup& group);

	/// Get the world particle group list. With the returned group, use
	/// ParticleGroup::GetNext to get the next group in the world list.
	/// A NULL group indicates the end of the list.
	/// @return the head of the world particle group list.
	ParticleGroup* GetParticleGroups();
	const ParticleGroup* GetParticleGroups() const;

	/// Get the number of particle groups.
	int32 GetParticleGroupCount() const;

	/// Get the number of particles.
	int32 GetCount() const { return m_count; }
	int32 GetCapacity() const { return m_capacity; }

	/// Set the maximum number of particles.
	/// A value of 0 means there is no maximum. The particle buffers can
	/// continue to grow while b2World's block allocator still has memory.
	/// Note: If you try to CreateParticle() with more than this count,
	/// b2_invalidParticleIndex is returned unless
	/// SetDestructionByAge() is used to enable the destruction of the
	/// oldest particles in the system.
	void SetMaxParticleCount(uint32 count);

	/// Get all existing particle flags.
	uint32 GetAllParticleFlags() const;
	uint32 GetAllParticleFlags(const ParticleGroup& group) const;

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

	bool IsLastIteration() const { return m_iteration + 1 == m_step.particleIterations; }

	/// Change the particle density.
	/// Particle density affects the mass of the particles, which in turn
	/// affects how the particles interact with b2Bodies. Note that the density
	/// does not affect how the particles interact with each other.
	void SetDensity(float32 density);

	/// Change the particle radius.
	/// You should set this only once, on world start.
	/// If you change the radius during execution, existing particles may
	/// explode, shrink, or behave unexpectedly.
	void SetRadius(float32 radius);

	/// Get the particle radius.
	float32 GetRadius() const;

	void CopyBox2DToGPUAsync();
	void WaitForCopyBox2DToGPU();

	/// Set flags for a particle. See the b2ParticleFlag enum.
	void SetParticleFlags(int32 index, uint32 flags);
	void AddParticleFlags(int32 index, uint32 flags);
	void RemovePartFlagFromAll(const uint32 flags);

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
	void SetPositionBuffer(Vec3* buffer, int32 capacity);
	void SetVelocityBuffer(Vec3* buffer, int32 capacity);
	void SetColorBuffer(int32* buffer, int32 capacity);

	/// Get contacts between particles
	/// Contact data can be used for many reasons, for example to trigger
	/// rendering or audio effects.
	const Particle::Contact* GetContacts() const;
	Particle::Contact* GetContacts();
	const int32 GetContactCount() const;

	/// Get contacts between particles and bodies
	/// Contact data can be used for many reasons, for example to trigger
	/// rendering or audio effects.
	//const Particle::BodyContact* GetBodyContacts() const;

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
	const b2ParticlePair* GetPairs() const;

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
	const b2ParticleTriad* GetTriads() const;

	/// Set an optional threshold for the maximum number of
	/// consecutive particle iterations that a particle may contact
	/// multiple bodies before it is considered a candidate for being
	/// "stuck". Setting to zero or less disables.
	void SetStuckThreshold(int32 iterations);

	/// Get potentially stuck particles from the last step; the user must
	/// decide if they are stuck or not, and if so, delete or move them
	const int32* GetStuckCandidates() const;

	/// Get the number of stuck particle candidates from the last step.
	int32 GetStuckCandidateCount() const;

	/// Compute the kinetic energy that can be lost by damping force
	//float32 ComputeCollisionEnergy() const;

	/// Set the lifetime (in seconds) of a particle relative to the current
	/// time.  A lifetime of less than or equal to 0.0f results in the particle
	/// living forever until it's manually destroyed by the application.
	void SetParticleLifetime(const int32 index, const float32 lifetime);
	/// Get the lifetime (in seconds) of a particle relative to the current
	/// time.  A value > 0.0f is returned if the particle is scheduled to be
	/// destroyed in the future, values <= 0.0f indicate the particle has an
	/// infinite lifetime.
	float32 GetParticleLifetime(const int32 index);

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


	float32 GetMassFromDensity(const float32 density) const;


	/// Apply an impulse to one particle. This immediately modifies the
	/// velocity. Similar to b2Body::ApplyLinearImpulse.
	/// @param index the particle that will be modified.
	/// @param impulse the world impulse vector, usually in N-seconds or
    ///        kg-m/s.
	void ParticleApplyLinearImpulse(int32 index, const Vec2& impulse);

	void ApplyLinearImpulse(const ParticleGroup& group,
		const Vec2& impulse);

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
							const Vec2& impulse);

	/// Distribute force between 2 Particles, using their mass
	void DistributeForce(int32 a, int32 b, const Vec2& f);
	void DistributeForce(int32 a, int32 b, const Vec3& f);
	void DistributeForceDamp(int32 a, int32 b, const Vec2& f);
	
	/// Apply a force to the center of a particle.
	/// @param index the particle that will be modified.
	/// @param force the world force vector, usually in Newtons (N).
	void ParticleApplyForce(int32 index, const Vec3& force);
	
	/// Distribute a force across several particles. The particles must not be
	/// wall particles. Note that the force is distributed across all the
	/// particles, so calling this function for indices 0..N is not the same as
	/// calling ParticleApplyForce(i, force) for i in 0..N.
	/// @param firstIndex the first particle to be modified.
	/// @param lastIndex the last particle to be modified.
	/// @param force the world force vector, usually in Newtons (N).
	void ApplyForce(const ParticleGroup& group, const Vec3& force);
	void ApplyForce(int32 firstIndex, int32 lastIndex, const Vec3& force);
	void PullIntoCircle(const Vec3& pos, const float32 radius,
		float32 strength, uint32 flag, bool ignoreMass, float32 step);

	/// Get the next particle-system in the world's particle-system list.
	ParticleSystem* GetNext();
	const ParticleSystem* GetNext() const;

	/// Query the particle system for all particles that potentially overlap
	/// the provided AABB. b2QueryCallback::ShouldQueryParticleSystem is
	/// ignored.
	/// @param callback a user implemented callback class.
	/// @param aabb the query box.
	void QueryAABB(b2QueryCallback* callback, const b2AABB& aabb) const;
	
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
	void RayCast(b2RayCastCallback& callback, const Vec2& point1,
				 const Vec2& point2) const;

	/// Compute the axis-aligned bounding box for all particles contained
	/// within this particle system.
	/// @param aabb Returns the axis-aligned bounding box of the system.
	void ComputeAABB(b2AABB* const aabb) const;
	void AmpComputeAABB(b2AABB& aabb, bool addVel = false) const;
	
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
	friend struct ParticleGroup;
	friend class b2ParticleBodyContactRemovePredicate;
	friend class b2FixtureParticleQueryCallback;
	friend class AmpFixtureParticleQueryCallback;
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
public:
	/// InsideBoundsEnumerator enumerates all particles inside the given bounds.
	class InsideBoundsEnumerator
	{
	public:
		/// Construct an enumerator with bounds of tags and a range of proxies.
		InsideBoundsEnumerator(
			uint32 lower, uint32 upper,
			const Proxy* first, const Proxy* last);
		
		/// Get index of the next particle. Returns b2_invalidParticleIndex if
		/// there are no more particles.
		int32 GetNext();
	private:
		/// The lower and upper bound of x component in the tag.
		uint32 m_xLower, m_xUpper;
		/// The lower and upper bound of y component in the tag.
		uint32 m_yLower, m_yUpper;
		/// The range of proxies.
		const Proxy* m_first;
		const Proxy* m_last;
	};

private:
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

	ParticleSystem(b2World& world, b2TimeStep& step, vector<Body>& bodyBuffer, vector<Fixture>& fixtureBuffer);
	~ParticleSystem();

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
	template <typename T> void AmpRequestBuffer(ampArray<T>& buf, bool& hasBuf);
	
	/// Reallocate the handle / index map and schedule the allocation of a new
	/// pool for handle allocation.
	void ReallocateHandleBuffers(int32 newCapacity);

	template<typename F> void ForEachGroup(const F& function) const;

	template<typename F> void AmpForEachParticle(const F& function) const;
	template<typename F> void AmpForEachParticle(uint32 flag, const F& function) const;
	template<typename F> void AmpForEachContact(const F& function) const;
	template<typename F> void AmpForEachContact(const uint32 flag, const F& function) const;
	template<typename F> void AmpForEachContactShuffled(const F& function) const;
	template<typename F> void AmpForEachBodyContact(const F& function) const;
	template<typename F> void AmpForEachBodyContact(const uint32 flag, const F& function) const;
	template<typename F> void AmpForEachGroundContact(const F& function) const;
	template<typename F> void AmpForEachGroundContact(const uint32 partFlag, const F& function) const;
	template<typename F> void AmpForEachPair(const F& function) const;
	template<typename F> void AmpForEachTriad(const F& function) const;

	
	boolean AdjustCapacityToSize(int32& capacity, int32 size, const int32 minCapacity) const;
	void ResizePartMatBuffers(int32 size);
	void ResizeParticleBuffers(int32 size);
	void ResizeGroupBuffers(int32 size);
	void ResizeContactBuffers(int32 size);
	void ResizeBodyContactBuffers(int32 size);
	void ResizePairBuffers(int32 size);
	void ResizeTriadBuffers(int32 size);
	pair<int32, int32> CreateParticlesWithPositions(const ParticleGroup::Def& groupDef);
	pair<int32, int32> CreateParticlesStrokeShapeForGroup(
		const b2Shape& shape,
		const ParticleGroup::Def& groupDef, const b2Transform& xf);
	pair<int32, int32> CreateParticlesFillShapeForGroup(
		const b2Shape& shape,
		ParticleGroup::Def& groupDef);
	pair<int32, int32> CreateParticlesWithShapeForGroup(ParticleGroup::Def& gd);
	int32 CloneParticle(int32 index, int32 groupIdx);

	void UpdatePairsAndTriads(
		int32 firstIndex, int32 lastIndex, const ConnectionFilter& filter);
	void AmpUpdatePairsAndTriads(int32 firstIndex, int32 lastIndex);
	static bool ComparePairIndices(const b2ParticlePair& a, const b2ParticlePair& b);
	static bool MatchPairIndices(const b2ParticlePair& a, const b2ParticlePair& b);
	static bool CompareTriadIndices(const b2ParticleTriad& a, const b2ParticleTriad& b);
	static bool MatchTriadIndices(const b2ParticleTriad& a, const b2ParticleTriad& b);

	static void InitializeParticleLists(
		const ParticleGroup& group, ParticleListNode* nodeBuffer);
	void MergeParticleListsInContact(
		const ParticleGroup& group, ParticleListNode* nodeBuffer) const;
	static void MergeParticleLists(
		ParticleListNode* listA, ParticleListNode* listB);
	static ParticleListNode* FindLongestParticleList(
		const ParticleGroup& group, ParticleListNode* nodeBuffer);
	void MergeZombieParticleListNodes(
		const ParticleGroup& group, ParticleListNode* nodeBuffer,
		ParticleListNode* survivingList) const;
	static void MergeParticleListAndNode(
		ParticleListNode* list, ParticleListNode* node);
	void CreateParticleGroupsFromParticleList(
		const ParticleGroup& group, ParticleListNode* nodeBuffer,
		const ParticleListNode* survivingList);
	void UpdatePairsAndTriadsWithParticleList(
		const ParticleGroup& group, const ParticleListNode* nodeBuffer);

public:
	InsideBoundsEnumerator GetInsideBoundsEnumerator(const b2AABB& aabb) const;
	void AddFlagInsideFixture(const Particle::Flag flag, const int32 matIdx,
		const Fixture& fixture);

private:
	void CopyShapeToGPU(b2Shape::Type type, int32 idx);

	void BoundProxyToTagBound(const b2AABBFixtureProxy& aabb, b2TagBounds& tagBounds);
	template<typename F>
	void AmpForEachInsideBounds(const b2AABB& aabb, const F& function);
	template<typename F>
	void AmpForEachInsideBounds(const vector<b2AABBFixtureProxy>& aabbs, const F& function);
	template<typename F>
	void AmpForEachInsideCircle(const b2CircleShape& circle,
		const b2Transform& transform, const F& function);
	void UpdateAllParticleFlags();
	void AmpUpdateAllParticleFlags();
	void UpdateAllGroupFlags();
	bool AddContact(int32 a, int32 b, int32& contactCount);
	bool ShouldCollide(int32 i, const Fixture& f) const;
	void FindContacts();
	void FindParticleContacts(int32 a, int32 c);
	void AmpFindContacts(bool exceptZombie);
	template<class T>
	void reorder(vector<T>& v, const vector<int32>& order);
	template<class T1, class T2>
	void reorder(vector<T1>& v1, vector<T2>& v2, const vector<int32>& order);
	void UpdateBodyContacts();
	void AmpUpdateBodyContacts();

	void AmpUpdateGroundContacts();

	//void AddBodyContactResults(ampArray<float32> dst, const ampArray<float32> bodyRes);
	//void AddBodyContactResults(ampArray<Vec3> dst, const ampArray<Vec3> bodyRes);

	void SolveSlowDown(const b2TimeStep& step);
	void SolveColorMixing();
	void SolveFreeze();
	void SolveZombie();
	void AmpSolveZombie();
	/// Destroy all particles which have outlived their lifetimes set by
	/// SetParticleLifetime().
	void SolveLifetimes(const b2TimeStep& step);
	void RotateBuffer(int32 start, int32 mid, int32 end);
	
	void AddZombieRange(int32 firstIdx, int32 lastIdx);
	uint32 GetWriteIdx(int32 particleCnt);

	template <class T1, class UnaryPredicate>
	static void RemoveFromVectorIf(vector<T1>& vectorToTest,
		int32& size, UnaryPredicate pred, bool adjustSize = true);
	//void AmpRemoveZombieContacts();
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

	template <typename T> void SetUserOverridableBuffer(
		UserOverridableBuffer<T>* buffer, T* newBufferData, int32 newCapacity);

	void SetGroupFlags(ParticleGroup& group, uint32 flags);
	void UpdateStatistics(const ParticleGroup& group) const;
	/// Get the total mass of the group: the sum of all particles in it.
	float32 GetMass(const ParticleGroup& group) const;
	/// Get the moment of inertia for the group.
	float32 GetInertia(const ParticleGroup& group) const;
	/// Get the center of gravity for the group.
	Vec2 GetCenter(const ParticleGroup& group) const;
	/// Get the linear velocity of the group.
	Vec2 GetLinearVelocity(const ParticleGroup& group) const;
	/// Get the angular velocity of the group.
	float32 GetAngularVelocity(const ParticleGroup& group) const;
	/// Get the world linear velocity of a world point, from the average linear
	/// and angular velocities of the particle group.
	/// @param a point in world coordinates.
	/// @return the world velocity of a point.
	Vec2 GetLinearVelocityFromWorldPoint(const ParticleGroup& group, const Vec2& worldPoint) const;
	
	const ampArrayView<Particle::Mat> GetMats() { return ampArrayView<Particle::Mat>(m_ampMats); }
	const ampArrayView<const Particle::Mat> GetConstMats() { return ampArrayView<const Particle::Mat>(m_ampMats); }

	const ampArrayView<Fixture> GetFixtures() { return ampArrayView<Fixture>(m_ampFixtures); }
	const ampArrayView<const Fixture> GetConstFixtures() { return ampArrayView<const Fixture>(m_ampFixtures); }
	
	const ampArrayView<Body> GetBodies() { return ampArrayView<Body>(m_ampBodies); }
	const ampArrayView<const Body> GetConstBodies() { return ampArrayView<const Body>(m_ampBodies); }
	
	const ampArrayView<Body::Mat> GetBodyMats();
	const ampArrayView<const Body::Mat> GetConstBodyMats();

	const ampArrayView<ParticleGroup> GetGroups() { return ampArrayView<ParticleGroup>(m_ampGroups); }
	const ampArrayView<const ParticleGroup> GetConstGroups() { return ampArrayView<const ParticleGroup>(m_ampGroups); }
	const ampArrayView<const b2ParticlePair> GetConstPairs() { return ampArrayView<const b2ParticlePair>(m_ampPairs); }
	const ampArrayView<const b2ParticleTriad> GetConstTriads() { return ampArrayView<const b2ParticleTriad>(m_ampTriads); }

	const ampArrayView<const AmpChainShape> GetConstChainShapes() { return ampArrayView<const AmpChainShape>(m_ampChainShapes); }
	const ampArrayView<const AmpCircleShape> GetConstCircleShapes() { return ampArrayView<const AmpCircleShape>(m_ampCircleShapes); }
	const ampArrayView<const AmpEdgeShape> GetConstEdgeShapes() { return ampArrayView<const AmpEdgeShape>(m_ampEdgeShapes); }
	const ampArrayView<const AmpPolygonShape> GetConstPolygonShapes() { return ampArrayView<const AmpPolygonShape>(m_ampPolygonShapes); }

	void RemoveSpuriousBodyContacts();
	//static bool BodyContactCompare(int32 lhsIdx, int32 rhsIdx);

	void DetectStuckParticle(int32 particle);

	/// Determine whether a particle index is valid.
	bool ValidateParticleIndex(const int32 index) const;

	/// Get the time elapsed in b2ParticleSystemDef::lifetimeGranularity.
	int32 GetQuantizedTimeElapsed() const;
	/// Convert a lifetime in seconds to an expiration time.
	int64 LifetimeToExpirationTime(const float32 lifetime) const;

	bool ForceCanBeApplied(uint32 flags) const;
	void PrepareForceBuffer();

	bool IsRigidGroup(const ParticleGroup& group) const;
	Vec2 GetLinearVelocity(
		const ParticleGroup& group, int32 particleIndex,
		const Vec2 &point);
	void InitDampingParameter(
		float32& invMass, float32& invInertia, float32& tangentDistance,
		float32 mass, float32 inertia, const Vec2& center,
		const Vec2& point, const Vec2& normal) const;
	void InitDampingParameterWithRigidGroupOrParticle(
		float32& invMass, float32& invInertia, float32& tangentDistance,
		bool isRigidGroup, const ParticleGroup& group, int32 particleIndex,
		const Vec2& point, const Vec2& normal);
	float32 ComputeDampingImpulse(
		float32 invMassA, float32 invInertiaA, float32 tangentDistanceA,
		float32 invMassB, float32 invInertiaB, float32 tangentDistanceB,
		float32 normalVelocity) const;
	void ApplyDamping(
		float32 invMass, float32 invInertia, float32 tangentDistance,
		bool isRigidGroup, ParticleGroup& group, int32 particleIndex,
		float32 impulse, const Vec2& normal);

	ampAccelView m_cpuAccelView = amp::getCpuAccelView();
	ampAccelView m_gpuAccelView = amp::getGpuAccelView();

	ampArray<Fixture>		  m_ampFixtures;
	ampArray<Body>			  m_ampBodies;
	ampArray<int32>			  m_ampBodyParticles;
	ampArray<AmpChainShape>	  m_ampChainShapes;
	ampArray<AmpCircleShape>  m_ampCircleShapes;
	ampArray<AmpEdgeShape>	  m_ampEdgeShapes;
	ampArray<AmpPolygonShape> m_ampPolygonShapes;

	amp::CopyFuture m_ampCopyFutGroups,
					m_ampCopyFutContacts,
					m_ampCopyFutBodies,
					m_ampCopyFutFixtures,
					m_ampCopyFutChainShapes,
					m_ampCopyFutCircleShapes,
					m_ampCopyFutEdgeShapes,
					m_ampCopyFutPolygonShapes;
	

	Future m_futureUpdateBodyContacts,
		   m_futureUpdateGroundContacts,
		   m_futureComputeWeight;


	bool m_paused;
	int32 m_timestamp;
	int32 m_allFlags;
	bool m_needsUpdateAllParticleFlags;
	int32 m_allGroupFlags;
	bool m_needsUpdateAllGroupFlags;
	bool m_hasForce;
	bool m_hasDepth;
	float32 m_particleDiameter;
	float32 m_particleRadius;
	float32 m_particleVolume;
	float32 m_atmosphereParticleMass;
	float32 m_atmosphereParticleInvMass;
	float32 m_inverseDiameter;
	float32 m_inverseRadius;
	float32 m_squaredDiameter;
	float32 m_cubicDiameter;


	//int32 m_tileCnt;
	//int32 m_contactTileCnt;

	int32 m_count;
	int32 m_capacity;
	int32 m_groupCount;
	int32 m_groupCapacity;
	int32 m_partMatCount;
	int32 m_partMatCapacity;
	int32 m_contactCount;
	int32 m_contactCapacity;
	int32 m_bodyContactCount;
	int32 m_bodyContactCapacity;
	int32 m_pairCount;
	int32 m_pairCapacity;
	int32 m_triadCount;
	int32 m_triadCapacity;

	ampExtent m_groupExtent;
	ampExtent m_pairTilableExtent;
	ampExtent m_triadTilableExtent;

	/// Allocator for b2ParticleHandle instances.
	b2SlabAllocator<b2ParticleHandle> m_handleAllocator;
	/// Maps particle indicies to  handles.
	bool hasHandleIndexBuffer;

	vector<b2ParticleHandle*> m_handleIndexBuffer;



	amp::CopyFuture m_ampCopyFutDepths;
	amp::CopyFuture m_ampCopyFutPairs;
	amp::CopyFuture m_ampCopyFutTriads;

	bool m_hasColorBuf;

	list<pair<int32, int32>>	m_zombieRanges;
	vector<int32>				m_freeGroupIdxs;
	vector<ParticleGroup>		m_groupBuffer;
	vector<uint32>		m_groupHasAlive;
	ampArray<ParticleGroup>	m_ampGroups;
	ampArray<uint32>	m_ampGroupHasAlive;
	
	vector<Particle::Mat> m_mats;
	ampArray<Particle::Mat>  m_ampMats;

	/// When any particles have the flag b2_staticPressureParticle,
	/// m_staticPressureBuffer is first allocated and used in
	/// SolveStaticPressure() and SolvePressure().  It will be reallocated on
	/// subsequent CreateParticle() calls.
	bool hasStaticPressureBuf;
	/// m_accumulationBuffer is used in many functions as a temporary buffer
	/// for scalar values.
	/// When any particles have the flag b2_tensileParticle,
	/// m_accumulation2Buffer is first allocated and used in SolveTensile()
	/// as a temporary buffer for vector values.  It will be reallocated on
	/// subsequent CreateParticle() calls.
	bool hasAccumulation2Buf;
	/// When any particle groups have the flag b2_solidParticleGroup,
	/// m_depthBuffer is first allocated and populated in ComputeDepth() and
	/// used in SolveSolid(). It will be reallocated on subsequent
	/// CreateParticle() calls.

	/// Stuck particle detection parameters and record keeping
	int32 m_stuckThreshold;
	int32 m_stuckParticleCount;
	bool hasLastBodyContactStepBuffer;
	bool hasBodyContactCountBuffer;
	bool hasConsecutiveContactStepsBuffer;
	vector<int32>  m_lastBodyContactStepBuffer;
	vector<int32>  m_consecutiveContactStepsBuffer;
	vector<int32>  m_stuckParticleBuffer;

	//vector<int32>  m_findContactCountBuf;
	//vector<std::array<int32, MAX_CONTACTS_PER_PARTICLE>>  m_findContactIdxABuf;
	//vector<std::array<int32, MAX_CONTACTS_PER_PARTICLE>>  m_findContactIdxBBuf;

	int32 m_bodyContactFixtureCnt = 0;

	vector<Particle::Contact> m_contacts;
	vector<Particle::BodyContact> m_bodyContacts;
	vector<Particle::ContactIdx> m_bodyContactIdxs;
	//ampArray<Particle::BodyContact> m_ampBodyContacts;

	//vector<b2ParticleBodyContact> m_bodyContactBuffer;

	vector<b2ParticlePair> m_pairBuffer;
	ampArray<b2ParticlePair> m_ampPairs;
	//vector<int32>   m_pairIdxABuf, m_pairIdxBBuf;
	//vector<uint32>  m_pairFlagsBuf;
	//vector<float32> m_pairStrengthBuf;	   /// The strength of cohesion among the particles.
	//vector<float32> m_pairDistanceBuf;	   /// The initial distance of the particles.
					
	vector<b2ParticleTriad> m_triadBuffer;
	ampArray<b2ParticleTriad> m_ampTriads;
	//vector<int32>   m_triadIdxABuf, m_triadIdxBBuf, m_triadIdxCBuf;
	//vector<uint32>  m_triadFlagsBuf;		/// The logical sum of the particle flags. See the b2ParticleFlag enum.
	//vector<float32> m_triadStrengthBuf;		/// The strength of cohesion among the particles.
	//vector<float32> m_triadPAXBuf, m_triadPAYBuf, 	/// Values used for calculation.
	//			    m_triadPBXBuf, m_triadPBYBuf,
	//			    m_triadPCXBuf, m_triadPCYBuf;
	//vector<float32> m_triadKABuf, m_triadKBBuf, m_triadKCBuf, m_triadSBuf;

	/// Time elapsed in 32:32 fixed point.  Each non-fractional unit of time
	/// corresponds to b2ParticleSystemDef::lifetimeGranularity seconds.
	int64 m_timeElapsed;
	/// Whether the expiration time buffer has been modified and needs to be
	/// resorted.
	bool m_expirationTimeBufferRequiresSorting;

	b2World& m_world;
	ParticleSystem* m_prev;
	ParticleSystem* m_next;
};

inline float32 ParticleSystem::GetMass(const ParticleGroup& group) const
{
	UpdateStatistics(group);
	return group.m_mass;
}
inline float32 ParticleSystem::GetInertia(const ParticleGroup& group) const
{
	UpdateStatistics(group);
	return group.m_inertia;
}
inline Vec2 ParticleSystem::GetCenter(const ParticleGroup& group) const
{
	UpdateStatistics(group);
	return group.m_center;
}
inline Vec2 ParticleSystem::GetLinearVelocity(const ParticleGroup& group) const
{
	UpdateStatistics(group);
	return group.m_linearVelocity;
}
inline float32 ParticleSystem::GetAngularVelocity(const ParticleGroup& group) const
{
	UpdateStatistics(group);
	return group.m_angularVelocity;
}
inline Vec2 ParticleSystem::GetLinearVelocityFromWorldPoint(
	const ParticleGroup& group, const Vec2& worldPoint) const
{
	UpdateStatistics(group);
	return group.m_linearVelocity + b2Cross(group.m_angularVelocity, worldPoint - group.m_center);
}

inline ParticleGroup* ParticleSystem::GetParticleGroups()
{
	return m_groupBuffer.data();
}
inline const ParticleGroup* ParticleSystem::GetParticleGroups() const
{
	return m_groupBuffer.data();
}

inline int32 ParticleSystem::GetParticleGroupCount() const
{
	return m_groupCount;
}

inline void ParticleSystem::SetPaused(bool paused)
{
	m_paused = paused;
}

inline bool ParticleSystem::GetPaused() const
{
	return m_paused;
}

inline const Particle::Contact* ParticleSystem::GetContacts() const
{
	return m_contacts.data();
}
inline Particle::Contact* ParticleSystem::GetContacts()
{
	return m_contacts.data();
}

inline const int32 ParticleSystem::GetContactCount() const
{
	return m_contactCount;
}

//inline const Particle::BodyContact* ParticleSystem::GetBodyContacts() const
//{
//	return m_bodyContacts.data();
//}

inline int32 ParticleSystem::GetBodyContactCount() const
{
	return m_bodyContactCount;
}


inline const b2ParticlePair* ParticleSystem::GetPairs() const
{
	return m_pairBuffer.data();
}

inline const b2ParticleTriad* ParticleSystem::GetTriads() const
{
	return m_triadBuffer.data();
}

inline ParticleSystem* ParticleSystem::GetNext()
{
	return m_next;
}

inline const ParticleSystem* ParticleSystem::GetNext() const
{
	return m_next;
}

inline const int32* ParticleSystem::GetStuckCandidates() const
{
	return m_stuckParticleBuffer.data();
}

inline int32 ParticleSystem::GetStuckCandidateCount() const
{
	return m_stuckParticleCount;
}

inline void ParticleSystem::SetDensity(float32 density)
{
	m_def.density = density;
	m_def.invDensity = 1 / density;
}

inline float32 ParticleSystem::GetRadius() const
{
	return m_particleDiameter / 2;
}

inline float32 ParticleSystem::GetCriticalVelocity(const b2TimeStep& step) const
{
	return m_particleDiameter * step.inv_dt;
}

inline float32 ParticleSystem::GetCriticalVelocitySquared(
	const b2TimeStep& step) const
{
	float32 velocity = GetCriticalVelocity(step);
	return velocity * velocity;
}

inline float32 ParticleSystem::GetCriticalPressure(const b2TimeStep& step) const
{
	return m_def.density * GetCriticalVelocitySquared(step);
}

inline float32 ParticleSystem::GetParticleStride() const
{
	return b2_particleStride * m_particleDiameter;
}

inline float32 ParticleSystem::GetMassFromDensity(const float32 density) const
{
	return density * m_particleVolume;
}

inline void ParticleSystem::SetMaxParticleCount(uint32 count)
{
	b2Assert(m_count <= count);
	m_def.maxCount = count;
}

inline uint32 ParticleSystem::GetAllParticleFlags() const
{
	return m_allFlags;
}
inline uint32 ParticleSystem::GetAllParticleFlags(const ParticleGroup& group) const
{
	uint32 flags = 0;
	for (int32 i = group.m_firstIndex; i < group.m_lastIndex; i++)
	{
		flags |= m_buffers.flags[i];
	}
	return flags;
}

inline uint32 ParticleSystem::GetAllGroupFlags() const
{
	return m_allGroupFlags;
}

inline bool ParticleSystem::ValidateParticleIndex(const int32 index) const
{
	return index >= 0 && index < GetCount() &&
		index != INVALID_IDX;
}

inline void ParticleSystem::ParticleApplyLinearImpulse(int32 index,
														 const Vec2& impulse)
{
	ApplyLinearImpulse(index, index + 1, impulse);
}

inline void ParticleSystem::DistributeForce(int32 a, int32 b, const Vec2& f) 
{
	m_buffers.velocity[a] -= m_buffers.mass[b] * f;
	m_buffers.velocity[b] += m_buffers.mass[a] * f;
}
inline void ParticleSystem::DistributeForce(int32 a, int32 b, const Vec3& f)
{
	m_buffers.velocity[a] -= m_buffers.mass[b] * f;
	m_buffers.velocity[b] += m_buffers.mass[a] * f;
}
inline void ParticleSystem::DistributeForceDamp(int32 a, int32 b, const Vec2& f)
{
	m_buffers.velocity[a] += m_buffers.mass[b] * f;
	m_buffers.velocity[b] -= m_buffers.mass[a] * f;
}


// Note: These functions must go in the header so the unit tests will compile
// them. b2ParticleSystem.cpp does not compile with this #define.
#if LIQUIDFUN_EXTERNAL_LANGUAGE_API

inline void b2ParticleSystem::SetParticleVelocity(int32 index,
												  float32 vx,
												  float32 vy)
{
	Vec2& v = GetVelocityBuffer()[index];
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
	int copySize = numParticles * sizeof(Vec2);
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


