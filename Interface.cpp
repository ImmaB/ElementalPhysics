#include <Box2D/Box2D.h>
#include <utility>
#include <Box2D/Common/b2Math.h>
#include <Box2D/Common/b2GlobalVariables.h>
#include <Box2D/Particle/b2ParticleSystem.h>
#include <math.h>
#include <vector>
#include <string.h>
#include <d3d11.h>
#include <ppl.h>

#define EXPORT extern "C" __declspec(dllexport)


class InterfaceContactListener : public b2ContactListener
{
public:

	InterfaceContactListener(ContactCallback beginCallback) : m_beginCallback(beginCallback) {}

	void BeginContact(b2Contact& c)
	{
		LFContact contact;
		contact.bodyAIdx = c.GetBodyIdxA();
		contact.bodyBIdx = c.GetBodyIdxB();
		std::async(launch::deferred, m_beginCallback, &contact);
	}

	ContactCallback m_beginCallback;
};

class b2NewRaycastCallback : public b2RayCastCallback {
public:
    b2NewRaycastCallback(int32 m, bool shouldQ) : numFixtures(0), numParticles(0), mode(m), shouldQuery(shouldQ) {}
    virtual float32 ReportFixture(const Fixture& fixture, const Vec2& point,
                                  const Vec2& normal, float32 fraction) {
        ++numFixtures;
        fixturesArray.push_back(point.x);
        fixturesArray.push_back(point.y);
        fixturesArray.push_back(normal.x);
        fixturesArray.push_back(normal.y);
        fixturesArray.push_back(fraction);
        return mode;
    }
    virtual float32 ReportParticle(const ParticleSystem* partSys,
                                   int32 index, const Vec2& point,
                                   const Vec2& normal, float32 fraction) {
        ++numParticles;
        particlesArray.push_back(0);
        particlesArray.push_back(index);
        particlesArray.push_back(point.x);
        particlesArray.push_back(point.y);
        particlesArray.push_back(normal.x);
        particlesArray.push_back(normal.y);
        particlesArray.push_back(fraction);
        return mode;
    }
    virtual bool ShouldQueryParticleSystem(const ParticleSystem* partSys) {
        return shouldQuery;
    }
    float32* GetData() {
        lengthsArray.push_back(numFixtures);
        lengthsArray.push_back(numParticles);
        returnArray.reserve(lengthsArray.size() + fixturesArray.size() + particlesArray.size());
        returnArray.insert(returnArray.end(), lengthsArray.begin(), lengthsArray.end());
        returnArray.insert(returnArray.end(), fixturesArray.begin(), fixturesArray.end());
        returnArray.insert(returnArray.end(), particlesArray.begin(), particlesArray.end());
        positionArray = &returnArray[0];
        return positionArray;
    }
private:
    float32* positionArray;
    int32 numFixtures;
    int32 numParticles;
    int32 mode;
    bool shouldQuery;
    std::vector<float> fixturesArray;
    std::vector<float> particlesArray;
    std::vector<float> lengthsArray;
    std::vector<float> returnArray;
};

#pragma region GlobalVariables
b2World* pWorld = nullptr;
Ground* pGround = nullptr;
ParticleSystem* pPartSys = nullptr;
b2NewRaycastCallback* newRC;
InterfaceContactListener* pContactListener;
#pragma endregion

#pragma region API World
EXPORT void CreateWorld()
{
	amp::CheckCompatibilty();
	if (!pWorld) pWorld = new b2World();
	pWorld->UnLock();
}
EXPORT void SetStepParams(float32 timeStep, int32 velocityIterations, int32 positionIterations, int32 particleIterations)
{
	pWorld->SetStepParams(timeStep, velocityIterations, positionIterations, particleIterations);
}
EXPORT void SetDamping(float32 damping) { pWorld->m_dampingStrength = damping; }
EXPORT void SetMassProperties(float32 surfThick, float32 mult) { pWorld->m_surfaceThickness = surfThick; pWorld->m_massMultiplicator = mult;  }
EXPORT void SetSurfaceThickness(float32 thickness) { pWorld->m_surfaceThickness = thickness; }

EXPORT void SetBorders(Vec3 lower, Vec3 upper, bool deleteOutside) { pWorld->SetBorders(lower, upper, deleteOutside); }
EXPORT void SetGravity(Vec3 gravity, float32 riseFactor) { pWorld->m_gravity = gravity; pWorld->m_riseFactor = riseFactor; }
EXPORT void SetRoomTemperature(float32 temperature) { pWorld->m_roomTemperature = temperature; }
EXPORT void SetInnerHeatExchange(float32 factor) { pWorld->m_innerHeatExchangeFactor = factor; }
EXPORT void SetAtmosphericDensity(float32 density) { pWorld->m_atmosphericDensity = density; }
EXPORT void SetWind(Vec3 wind) { pWorld->m_wind = wind; }

EXPORT void ClearWorld() { if (pWorld) pWorld->ClearAll(); }
EXPORT void DestroyWorld() { if (pWorld) delete pWorld; }

EXPORT bool IsWorldCreated() { return pWorld != nullptr; }

EXPORT void StepPreParticle() { pWorld->StepPreParticle(); }
EXPORT void StepPostParticle() { pWorld->StepPostParticle(); }

EXPORT bool GetAllowSleeping(b2World* pWorld) { return pWorld->GetAllowSleeping(); }
EXPORT void SetAllowSleeping(b2World* pWorld, bool flag) { pWorld->SetAllowSleeping(flag); }

EXPORT Vec3 GetWorldGravity() { return pWorld->m_gravity; }

EXPORT void ClearBodyMaterials() { pWorld->ClearBodyMaterials(); }
EXPORT int32 CreateBodyMaterial(uint32 materialFlags, float32 density, float32 friction, float32 bounciness,
	float32 stability, float32 heatConductivity, float32 hotThreshold, float32 coldThreshold, float32 ignitionThreshold)
{
	Body::Mat::Def md;
	md.matFlags = materialFlags;
	md.density = density;
	md.friction = friction;
	md.bounciness = bounciness;
	md.stability = stability;
	md.heatConductivity = heatConductivity;
	md.hotThreshold = hotThreshold;
	md.coldThreshold = coldThreshold;
	md.ignitionThreshold = ignitionThreshold;
	return pWorld->CreateBodyMaterial(md);
}
EXPORT void AddBodyMatChange()
{

}
EXPORT void DestroyBodyMaterial(int32 idx)
{
	return;
}

EXPORT void* GetDebug() { return static_cast<void*>(debugString); }

EXPORT int32 GetContactCount()
{
	return pPartSys->GetContactCount();
}
EXPORT void SetContactCallback(ContactCallback callback)
{
	pContactListener = new InterfaceContactListener(callback);
	pWorld->SetContactListener(pContactListener);
}

EXPORT void SetDebugContacts(bool debug) { pPartSys->m_debugContacts = debug; }

EXPORT void GetPartContacts(int32* pCnt, Particle::Contact** pContacts)
{
	*pCnt = pPartSys->GetContactCount();
	*pContacts = pPartSys->GetContacts();
}
EXPORT int32 GetPartContactCnt() { return pPartSys->GetContactCount(); }

#pragma endregion

#pragma region API Particle Systems

EXPORT void CreateParticleSystem(bool accelerate, float32 radius, float32 damping,
	float32 gravityScale, float32 airResFactor, float32 minAirSpeed, float32 heatLossRatio,
	float32 flameHealthLossPerSec, float32 flameHeatGainPerSec)
{
	if (!pPartSys) pPartSys = pWorld->CreateParticleSystem();
	pPartSys->m_accelerate = accelerate;
	pPartSys->SetRadius(radius);
	pPartSys->m_def.dampingStrength = damping;
	pPartSys->m_def.gravityScale = gravityScale;
	pPartSys->m_def.airResistanceFactor = airResFactor;
	pPartSys->m_def.minAirSpeed = minAirSpeed;
	pPartSys->m_def.heatLossRatio = heatLossRatio;
	pPartSys->m_def.flameHealthLossPerSec = flameHealthLossPerSec;
	pPartSys->m_def.flameHeatGainPerSec = flameHeatGainPerSec;
}
EXPORT void DestroyAllParticles() { if (pPartSys) pPartSys->DestroyAllParticles(); }

EXPORT void DestroyParticleSystem() { pWorld->DestroyParticleSystem(); }

EXPORT void SetAccelerate(bool acc) { pPartSys->m_accelerate = acc; }

EXPORT int32 GetParticleIterations(float32 gravity, float32 particleRadius, float32 timeStep) {
	return b2CalculateParticleIterations(gravity, particleRadius, timeStep);
}

EXPORT bool ShouldSolveParticleSystem() { return pPartSys != nullptr && pPartSys->ShouldSolve(); }

EXPORT void SolveInit()	{ pPartSys->SolveInit(); }

EXPORT void InitStep() { pPartSys->InitStep(); }
EXPORT void SortProxies() { pPartSys->SortProxies(); }
EXPORT void UpdateContacts() { pPartSys->UpdateContacts(true); }
EXPORT void ReduceContacts() { pPartSys->ReduceContacts(); }
EXPORT void ComputeDepth() { pPartSys->ComputeDepth(); }
EXPORT void UpdatePairsAndTriadsWithReactiveParticles() { pPartSys->UpdatePairsAndTriadsWithReactiveParticles(); }

EXPORT void SolveForce() { pPartSys->SolveForce(); }
EXPORT void WaitForUpdateBodyContacts() { pPartSys->WaitForUpdateBodyContacts(); }
EXPORT void ComputeWeight() { pPartSys->ComputeWeight(); }
EXPORT void SolveViscous() { pPartSys->SolveViscous(); }
EXPORT void SolveRepulsive() { pPartSys->SolveRepulsive(); }
EXPORT void SolvePowder() { pPartSys->SolvePowder(); }
EXPORT void WaitForComputeWeight() { pPartSys->WaitForComputeWeight(); }
EXPORT void SolveTensile() { pPartSys->SolveTensile(); }
EXPORT void SolveSolid() { pPartSys->SolveSolid(); }
EXPORT void SolveGravity() { pPartSys->SolveGravity(); }
EXPORT void SolveWind() { pPartSys->SolveWind(); }
EXPORT void SolveStaticPressure() { pPartSys->SolveStaticPressure(); }
EXPORT void SolvePressure() { pPartSys->SolvePressure(); }
EXPORT void SolveDamping() { pPartSys->SolveDamping(); }
EXPORT void SolveExtraDamping() { pPartSys->SolveExtraDamping(); }
EXPORT void SolveAirResistance() { pPartSys->SolveAirResistance(); }
EXPORT void SolveElastic() { pPartSys->SolveElastic(); }
EXPORT void SolveSpring() { pPartSys->SolveSpring(); }
EXPORT void LimitVelocity() { pPartSys->LimitVelocity(); }
EXPORT void SolveRigidDamping() { pPartSys->SolveRigidDamping(); }
EXPORT void SolveBarrier() { pPartSys->SolveBarrier(); }
EXPORT void SolveCollision() { pPartSys->SolveCollision(); }
EXPORT void SolveRigid() { pPartSys->SolveRigid(); }
EXPORT void SolveWall() { pPartSys->SolveWall(); }
EXPORT void CopyVelocities() { pPartSys->CopyVelocities(); }
EXPORT void SolveKillNotMoving() { pPartSys->SolveKillNotMoving(); }
EXPORT void SolveFluid() { pPartSys->SolveFluid(); }

EXPORT void SolveFlame() { pPartSys->SolveFlame(); }
EXPORT void SolveIgnite() { pPartSys->SolveIgnite(); }
EXPORT void SolveExtinguish() { pPartSys->SolveExtinguish(); }
EXPORT void SolveHeatConduct() { pPartSys->SolveHeatConduct(); }
EXPORT void SolveLooseHeat() { pPartSys->SolveLooseHeat(); }
EXPORT void CopyHeats() { pPartSys->CopyHeats(); }
EXPORT void SolveChangeMat() { pPartSys->SolveChangeMat(); }

EXPORT void SolveHealth() { pPartSys->SolveHealth(); }
EXPORT void CopyHealths() { pPartSys->CopyHealths(); }
EXPORT void SolvePosition() { pPartSys->SolvePosition(); }
EXPORT void SolveOutOfBounds() { pPartSys->SolveOutOfBounds(); }
EXPORT void CopyFlags() { pPartSys->CopyFlags(); }
EXPORT void CopyBodies() { pPartSys->CopyBodies(); }

EXPORT void IncrementIteration() { pPartSys->IncrementIteration(); }

EXPORT void SolveEnd() { pPartSys->SolveEnd(); }


EXPORT void SetStaticPressureIterations(int32 iterations) { pPartSys->m_def.staticPressureIterations = iterations; }

EXPORT void SetDestroyStuck(bool toggle)
{
	//TODO
	//parts->SetDestroyStuck(toggle);
}

EXPORT int32 DestroyParticlesInFixture(int32 fixtureIdx, b2Transform transform, bool call)
{
    return pPartSys->DestroyParticlesInFixture(pWorld->GetFixture(fixtureIdx), transform, call);
}

EXPORT void SetAllParticleFlags(void* partSysPtr, int32 flags) {
    ParticleSystem* partSys = static_cast<ParticleSystem*>(partSysPtr);
    int32 numParts = partSys->GetCount();
    for (int32 i = 0; i < numParts; ++i) {
        partSys->SetParticleFlags(i, flags);
    }
}

EXPORT int32 GetStuckCandidateCount(void* partSysPtr) {
    ParticleSystem* partSys = static_cast<ParticleSystem*>(partSysPtr);
    return partSys->GetStuckCandidateCount();
}
EXPORT void SetStuckThreshold(void* partSysPtr, int32 iterations) {
    ParticleSystem* partSys = static_cast<ParticleSystem*>(partSysPtr);
    partSys->SetStuckThreshold(iterations);
}
EXPORT void SetMaxParticleCount(int32 count) { pPartSys->SetMaxParticleCount(count); }

#pragma endregion

#pragma region API Particles
EXPORT void AddFlagsToPartsWithMatInFixture(Particle::Flag flag, int32 matIdx, int32 fixtureIdx)
{
	pPartSys->AddFlagInsideFixture(flag, matIdx, pWorld->GetFixture(fixtureIdx));
}

EXPORT void RemoveFlagFromAll(uint32 flags) { pPartSys->RemovePartFlagFromAll(flags); }

EXPORT void PullPartsIntoCircle(Vec3 pos, float32 radius, float32 strength, int32 flag, bool ignoreMass, float32 step)
{
	pPartSys->PullIntoCircle(pos, radius, strength, flag, ignoreMass, step);
}

EXPORT int32 GetParticleCapacity() { return pPartSys->GetCapacity(); }
EXPORT int32 GetParticleCount() { return pPartSys->GetCount(); }

EXPORT void SetParticleBufferResizeCallback(ParticleSystem::ResizeCallback callback)
{
	pPartSys->m_resizeCallback = callback;
	pPartSys->m_resizeCallback(pPartSys->GetCapacity());
}
EXPORT void SetParticlesBuffers(ID3D11Buffer** bufPtrs)
{
	pPartSys->m_d11Buffers.Set(bufPtrs, pPartSys->m_debugContacts);
}
EXPORT void SetContactIdxBuffer(ID3D11Buffer* bufPtr) { pPartSys->m_d11Buffers.contactIdx = bufPtr; }


#pragma endregion

#pragma region ParticleGroups

EXPORT int32 CreateParticleGroup(uint32 partFlags, uint32 groupFlags, int32 matIdx, int32 collisionGroup,
	float32 angVel, Vec3 linVel, b2Shape::Type shapeType, int32 shapeIdx,
	b2Transform transform, int32 color, float32 stride, float32 health, float32 heat, int32 timestamp)
{
	ParticleGroup::Def gd;
	gd.flags = partFlags;
	gd.groupFlags = groupFlags;
	gd.matIdx = matIdx;
	gd.collisionGroup = collisionGroup;
	gd.shapeType = shapeType;
	gd.shapeIdx = shapeIdx;
	gd.transform = transform;
	gd.angularVelocity = angVel;
	gd.linearVelocity = linVel;
	gd.stride = stride;
	gd.health = health;
	gd.color = color;
	gd.heat = heat;
	gd.timestamp = timestamp;
	int32 idx = pPartSys->CreateGroup(gd);
	return idx;
}
EXPORT int32 CreateParticles(int32 partCount, Vec3* poss, int32* cols, b2Transform transform,
	uint32 partFlags, uint32 groupFlags, int32 matIdx, int32 collisionGroup, Vec3 vel, float32 health, float32 heat, int32 timestamp)
{
	ParticleGroup::Def pd;
	pd.particleCount = partCount;
	if (poss != nullptr) pd.positions = std::vector<Vec3>(poss, poss + partCount);
	if (cols != nullptr) pd.colors = std::vector<int32>(cols, cols + partCount);
	pd.transform = transform;
	pd.flags = partFlags;
	pd.groupFlags = groupFlags;
	pd.matIdx = matIdx;
	pd.collisionGroup = collisionGroup;
	pd.linearVelocity = vel;
	pd.health = health;
	pd.heat = heat;
	pd.timestamp = timestamp;
	return pPartSys->CreateGroup(pd);
}
EXPORT void JoinParticleGroups(void* partSysPtr, int32 groupAIdx, int32 groupBIdx)
{
    ParticleSystem* system = static_cast<ParticleSystem*>(partSysPtr);
    system->JoinParticleGroups(groupAIdx, groupBIdx);
}
EXPORT void ApplyForceToParticleGroup(ParticleGroup* pGroup, Vec3 force)
{
	pPartSys->ApplyForce(*pGroup, force);
}
EXPORT void ApplyLinearImpulseToParticleGroup(void* partSysPtr, void* groupPointer, float32 forceX, float32 forceY) {
	ParticleSystem* system = static_cast<ParticleSystem*>(partSysPtr);
	ParticleGroup* group = static_cast<ParticleGroup*>(groupPointer);
    Vec2 impulse = Vec2(forceX, forceY);
	system->ApplyLinearImpulse(*group, impulse);
}
EXPORT void DestroyParticleGroup(int32 groupIdx, int32 timestamp)
{
	pPartSys->DestroyGroup(groupIdx, timestamp, true);
}

#pragma endregion


#pragma region Shapes

EXPORT bool IsPointInFixture(int32 fixtureIdx, Vec3 p, float32 angle)
{
	const b2Shape& s = pWorld->GetShape(pWorld->GetFixture(fixtureIdx));
	b2Transform transform = b2Transform(p, b2Rot(angle));
	return s.TestPoint(transform, p);
}
EXPORT int32 AddBoxShape(Vec2 size, float32 height, Vec3 pos, float32 angle)
{
	b2PolygonShapeDef sd;
	sd.type = b2Shape::e_polygon;
	sd.zPos = pos.z;
	sd.height = height;
	sd.SetAsBox(size, pos, angle);
	return pWorld->CreateShape(sd);
}
EXPORT int32 AddPolygonShape(Vec2* vertices, int32 count, float32 zPos, float32 height)
{
	b2PolygonShapeDef sd;
	sd.type = b2Shape::e_polygon;
	sd.zPos = zPos;
	sd.height = height;
	std::memcpy(sd.vertices, vertices, sizeof Vec2 * count);
	sd.count = count;
	return pWorld->CreateShape(sd);
}
EXPORT int32 AddCircleShape(float32 radius, float32 height, Vec3 pos)
{
	b2CircleShapeDef sd;
	sd.type = b2Shape::e_circle;
	sd.zPos = pos.z;
	sd.height = height;
	sd.p = pos;
	sd.radius = radius;
	return pWorld->CreateShape(sd);
}
EXPORT void* GetChainShapeDef(float32* vertArray, bool loop, float32 zPos, float32 zHeight) {
    b2ChainShape* shape = new b2ChainShape();
    int32 numberOfVertices = vertArray[0];
    Vec2* vertices = new Vec2[numberOfVertices];
    for (int32 i = 1, j = 0; i < (vertArray[0] * 2); i += 2)
        vertices[j++] = Vec2(vertArray[i], vertArray[i + 1]);
    if (loop)
        shape->CreateLoop(vertices, numberOfVertices);
    else
        shape->CreateChain(vertices, numberOfVertices);
    return static_cast<void*>(shape);
}
EXPORT void* GetEdgeShapeDef(float32 x1, float32 y1, float32 x2, float32 y2, float32 zPos, float32 zHeight) {
    Vec2 vec1 = Vec2(x1, y1);
    Vec2 vec2 = Vec2(x2, y2);
    b2EdgeShape* shape = new b2EdgeShape();
    shape->Set(vec1, vec2);
    return static_cast<void*>(shape);
}
EXPORT void DestroyShape(b2Shape::Type shapeType, int32 shapeIdx)
{
	pWorld->RemoveSubShapeFromBuffer(shapeType, shapeIdx);
}
EXPORT void SetCirclePosition(int32 shapeIdx, Vec3 pos)
{
	pWorld->m_circleShapeBuffer[shapeIdx].m_p = pos;
}

#pragma endregion

#pragma region Body

EXPORT int32 CreateBody(Body::Type type, b2Transform transform, float32 linearDamping, float32 angularDamping,
	int32 materialIdx, float32 heat, float32 surfaceHeat, float32 health, uint32 flags, float32 gravityScale)
{
    Body::Def bd;
    bd.type = type;
    bd.transform = transform;
    bd.linearDamping = linearDamping;
    bd.angularDamping = angularDamping;
	bd.materialIdx = materialIdx;
	bd.heat = heat;
	bd.surfaceHeat = surfaceHeat;
	bd.health = health;
	bd.flags = flags;
	bd.gravityScale = gravityScale;

    return pWorld->CreateBody(bd);
}
EXPORT Body* GetBody(int32 idx) { return &pWorld->m_bodyBuffer[idx]; }
EXPORT Body* GetBodies(int32* outCnt)
{
	*outCnt = pWorld->m_bodyBuffer.size();
	return pWorld->m_bodyBuffer.data();
}

EXPORT void SetBodyAwake(Body* pBody, bool isAwake)
{
	pBody->SetAwake(isAwake);
}
EXPORT bool GetBodyAwake(b2World* pWorld, int32 bodyIdx)
{
	const Body& body = pWorld->GetBody(bodyIdx);
    return body.IsAwake();
}
EXPORT void SetBodyActive(Body* pBody, bool isActive)
{
	pWorld->SetActive(*pBody, isActive);
}
EXPORT bool GetBodyActive(b2World* pWorld, int32 bodyIdx)
{
    const Body& body = pWorld->GetBody(bodyIdx);
    return body.IsActive();
}
EXPORT void ApplyForceToBody(int32 bodyIdx, Vec3 force, bool wake)
{
	pWorld->GetBody(bodyIdx).ApplyForceToCenter(force, wake);
}
EXPORT void ApplyImpulseToBody(int32 bodyIdx, Vec3 impulse, bool wake)
{
	pWorld->GetBody(bodyIdx).ApplyImpulseToCenter(impulse, wake);
}
EXPORT void ApplyAngularImpulseToBody(int32 bodyIdx, float32 impulse, bool wake)
{
	pWorld->GetBody(bodyIdx).ApplyAngularImpulse(impulse, wake);
}
EXPORT void ApplyForceToBodyAtPoint(int32 bodyIdx, Vec2 force, Vec2 pos, bool wake)
{
	pWorld->GetBody(bodyIdx).ApplyForce(force, pos, wake);
}
EXPORT void ApplyImpulseToBodyAtPoint(int32 bodyIdx, Vec3 impulse, Vec2 pos, bool wake)
{
	pWorld->GetBody(bodyIdx).ApplyLinearImpulse(impulse, pos, wake);
}
EXPORT void ApplyTorqueToBody(Body* pBody, float32 torque, bool wake)
{
    pBody->ApplyTorque(torque, wake);
}

EXPORT void SetBodyTransform(int32 bodyIdx, b2Transform transform)
{
	pWorld->SetTransform(pWorld->GetBody(bodyIdx), transform);
}
EXPORT void ApplyDamageToBody(int32 bIdx, float32 damage) { pWorld->GetBody(bIdx).m_health -= damage; }
EXPORT void ApplyHeatToBody(int32 bIdx, float32 heat) { pWorld->GetBody(bIdx).m_heat += heat; }
EXPORT void ApplyHeatToBodySurface(int32 bIdx, float32 heat) { pWorld->GetBody(bIdx).m_surfaceHeat += heat; }

EXPORT void DestroyBody(int32 idx) { pWorld->DestroyBody(idx); }

EXPORT void SetBodyVelocity(int32 idx, Vec3 vel) { pWorld->GetBody(idx).SetLinearVelocity(vel); }


#pragma endregion


#pragma region Ground

EXPORT void CreateGround(int32 xSize, int32 ySize, float32 stride)
{
	Ground::def gd;
	gd.xSize = xSize;
	gd.ySize = ySize;
	gd.stride = stride;
	pGround = pWorld->CreateGround(gd);
}
EXPORT void SetGroundBuffer(ID3D11Buffer* pD11Tiles) { if (pGround) pGround->m_d11Tiles = pD11Tiles; }
EXPORT void SetGroundTiles(Ground::Tile* pTiles)
{
	pGround->SetTiles(pTiles);
}
EXPORT void SetGroundTileChangeCallback(Ground::ChangeCallback callback)
{
	pGround->m_changeCallback = callback;
}
EXPORT void ExtractGroundParticles(int32 fixtureIdx, int32 partMatIdx, uint32 partFlags, float32 strength)
{
	pGround->ExtractParticles(pWorld->GetShape(fixtureIdx), b2Transform(), partMatIdx, partFlags, strength);
}
EXPORT int32 AddGroundMaterial(float32 friction, float32 bounciness, int32 particleCapacity, uint32 flags)
{
	Ground::Mat::def gmd;
	gmd.friction = friction;
	gmd.bounciness = bounciness;
	gmd.particleCapacity = particleCapacity;
	gmd.flags = flags;
	return pGround->CreateMaterial(gmd);
}

EXPORT void CopyGroundTiles() { pGround->CopyChangedTiles(); }

#pragma endregion

#pragma region Fixture

EXPORT int32 AddFixture(int32 bodyIdx, b2Shape::Type shapeType, int32 shapeIdx,
	bool isSensor, int32 collisionGroup,uint16 categoryBits, uint16 maskBits)
{
    Fixture::Def fd;
	fd.isSensor = isSensor;
	fd.bodyIdx = bodyIdx;
	fd.shapeType = shapeType;
    fd.shapeIdx = shapeIdx;
    fd.filter.collisionGroup = collisionGroup;
    fd.filter.maskBits = maskBits;
    fd.filter.categoryBits = categoryBits;
    int32 idx = pWorld->CreateFixture(fd);

    return idx;
}
EXPORT Fixture* GetFixture(int32 idx)
{
	return &pWorld->m_fixtureBuffer[idx];
}

EXPORT bool TestPoint(Fixture* pFixture, float32 x, float32 y)
{
	return pWorld->TestPoint(*pFixture, Vec2(x, y));
}

EXPORT void SetFixtureFilterData(int32 idx, int32 collisionGroup, uint16 categoryBits, uint16 maskBits)
{
    b2Filter filter = b2Filter();
    filter.collisionGroup = collisionGroup;
    filter.maskBits = maskBits;
    filter.categoryBits = categoryBits;
	Fixture& fixture = pWorld->GetFixture(idx);
	pWorld->SetFilterData(fixture, filter);
}
EXPORT uint16 GetFixtureCollisionGroup(int32 idx)
{
    return pWorld->GetFixture(idx).m_filter.collisionGroup;
}
EXPORT uint16 GetFixtureMaskBits(Fixture* pFixture)
{
    return pFixture->m_filter.maskBits;
}
EXPORT uint16 GetFixtureCategoryBits(Fixture* pFixture)
{
    return pFixture->m_filter.categoryBits;
}

EXPORT void SetFixtureIsSensor(Fixture* pFixture, bool flag)
{
	pWorld->SetSensor(*pFixture, flag);
}

EXPORT float32 GetFixtureDensity(Fixture* pFixture)
{
    return pFixture->m_density;
}
EXPORT void SetFixtureDensity(Fixture* pFixture, float32 density)
{
    pFixture->SetDensity(density);
}

EXPORT void DestroyFixture(int32 idx)
{
	pWorld->DestroyFixture(idx);
}
#pragma endregion

#pragma region Joints

#pragma region DistanceJoints
EXPORT b2DistanceJoint* CreateDistanceJoint(int32 bodyAIdx, int32 bodyBIdx, float32 anchorAX, float32 anchorAY, float32 anchorBX, float32 anchorBY, float32 length, bool collideConnected)
{
    b2DistanceJoint* dj;
    b2DistanceJointDef jd;
    jd.bodyAIdx = bodyAIdx;
    jd.bodyBIdx = bodyBIdx;
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    jd.collideConnected = collideConnected;
    jd.length = length;
    dj = (b2DistanceJoint*)pWorld->CreateJoint(&jd);
    return dj;
}
EXPORT void SetDistanceJointFrequency(b2DistanceJoint* pJoint, float32 frequency)
{
	pJoint->SetFrequency(frequency);
}
EXPORT float32 GetDistanceJointFrequency(b2DistanceJoint* pJoint)
{
    return pJoint->GetFrequency();
}
EXPORT void SetDistanceJointDampingRatio(b2DistanceJoint* pJoint, float32 ratio)
{
	pJoint->SetDampingRatio(ratio);
}
EXPORT float32 GetDistanceJointDampingRatio(b2DistanceJoint* pJoint)
{
    return pJoint->GetDampingRatio();
}
EXPORT void SetDistanceJointLength(b2DistanceJoint* pJoint, float32 length)
{
	pJoint->SetLength(length);
}
EXPORT float32 GetDistanceJointLength(b2DistanceJoint* pJoint)
{
    return pJoint->GetLength();
}
#pragma endregion

#pragma region RevoluteJoints
EXPORT void* CreateRevoluteJoint(int32 bodyAIdx, int32 bodyBIdx, float32 anchorAX, float32 anchorAY, float32 anchorBX, float32 anchorBY, bool collideConnected)
{
    b2RevoluteJoint* rj;
    b2RevoluteJointDef jd;
    jd.bodyAIdx = bodyAIdx;
    jd.bodyBIdx = bodyBIdx;
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    jd.collideConnected = collideConnected;
    rj = (b2RevoluteJoint*)pWorld->CreateJoint(&jd);
    return static_cast<void*>(rj);
}
EXPORT void SetRevoluteJointLimits(b2RevoluteJoint* pJoint, float32 lower, float32 upper)
{
	pJoint->SetLimits(lower, upper);
}
EXPORT void SetRevoluteJointMotorSpeed(b2RevoluteJoint* pJoint, float32 speed)
{
	pJoint->SetMotorSpeed(speed);
}
EXPORT void SetRevoluteJointMaxMotorTorque(b2RevoluteJoint* pJoint, float32 torque)
{
	pJoint->SetMaxMotorTorque(torque);
}
EXPORT void EnableRevoluteJointMotor(b2RevoluteJoint* pJoint, bool motor)
{
	pJoint->EnableMotor(motor);
}
EXPORT void EnableRevoluteJointLimits(b2RevoluteJoint* pJoint, bool limit)
{
	pJoint->EnableLimit(limit);
}
EXPORT float32 GetRevoluteJointUpperLimit(b2RevoluteJoint* pJoint)
{
    return pJoint->GetUpperLimit();
}
EXPORT float32 GetRevoluteJointLowerLimit(b2RevoluteJoint* pJoint)
{
    return pJoint->GetLowerLimit();
}
EXPORT bool IsRevoluteJointMotorEnabled(b2RevoluteJoint* pJoint)
{
    return pJoint->IsMotorEnabled();
}
EXPORT float32 GetRevoluteJointMotorSpeed(b2RevoluteJoint* pJoint)
{
    return pJoint->GetMotorSpeed();
}
EXPORT float32 GetRevoluteJointMotorTorque(b2RevoluteJoint* pJoint, float32 invDt)
{
    return pJoint->GetMotorTorque(invDt);
}
EXPORT float32 GetRevoluteJointMaxMotorTorque(b2RevoluteJoint* pJoint)
{
    return pJoint->GetMaxMotorTorque();
}
#pragma endregion

#pragma region PrismaticJoints
EXPORT void* CreatePrismaticJoint(int32 bodyAIdx, int32 bodyBIdx, float32 anchorAX, float32 anchorAY, float32 anchorBX, float32 anchorBY, float32 axisX, float32 axisY, bool collideConnect)
{
    b2PrismaticJoint* pj;
    b2PrismaticJointDef jd;
    jd.bodyAIdx = bodyAIdx;
    jd.bodyBIdx = bodyBIdx;
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    jd.localAxisA.Set(axisX, axisY);
    jd.collideConnected = collideConnect;
    pj = (b2PrismaticJoint*)pWorld->CreateJoint(&jd);
    return static_cast<void*>(pj);
}
EXPORT void SetPrismaticJointLimits(b2PrismaticJoint* pJoint, float32 lower, float32 upper)
{
	pJoint->SetLimits(lower, upper);
}
EXPORT void SetPrismaticJointMotorSpeed(b2PrismaticJoint* pJoint, float32 speed)
{
	pJoint->SetMotorSpeed(speed);
}
EXPORT void SetPrismaticJointMaxMotorForce(b2PrismaticJoint* pJoint, float32 force)
{
	pJoint->SetMaxMotorForce(force);
}
EXPORT void EnablePrismaticJointMotor(b2PrismaticJoint* pJoint, bool motor)
{
	pJoint->EnableMotor(motor);
}
EXPORT void EnablePrismaticJointLimits(b2PrismaticJoint* pJoint, bool limit)
{
	pJoint->EnableLimit(limit);
}
EXPORT float32 GetPrismaticJointUpperLimit(b2PrismaticJoint* pJoint)
{
    return pJoint->GetUpperLimit();
}
EXPORT float32 GetPrismaticJointLowerLimit(b2PrismaticJoint* pJoint)
{
    return pJoint->GetLowerLimit();
}
EXPORT bool IsPrismaticJointMotorEnabled(b2PrismaticJoint* pJoint)
{
    return pJoint->IsMotorEnabled();
}
EXPORT float32 GetPrismaticJointMotorSpeed(b2PrismaticJoint* pJoint)
{
    return pJoint->GetMotorSpeed();
}
EXPORT float32 GetPrismaticJointMotorTorque(b2PrismaticJoint* pJoint, float32 invDt)
{
    return pJoint->GetMotorForce(invDt);
}
EXPORT float32 GetPrismaticJointMaxMotorForce(b2PrismaticJoint* pJoint)
{
    return pJoint->GetMaxMotorForce();
}
EXPORT float32 GetPrismaticJointMotorForce(b2PrismaticJoint* pJoint,float32 its)
{
    return pJoint->GetMotorForce(its);
}
EXPORT float32 GetPrismaticJointSpeed(b2PrismaticJoint* pJoint)
{
    return pJoint->GetJointSpeed();
}
#pragma endregion

#pragma region PulleyJoints
EXPORT void* CreatePulleyJoint(int32 bodyAIdx, int32 bodyBIdx, float32 groundAnchorAX,
	float32 groundAanchorAY, float32 groundAnchorBX, float32 groundAanchorBY, float32 anchorAX, float32 anchorAY, float32 anchorBX,
	float32 anchorBY, float32 ratio, float32 lengthA, float32 lengthB, bool collideConnect)
{


    b2PulleyJoint* pj;
    b2PulleyJointDef jd;
    jd.bodyAIdx = bodyAIdx;
    jd.bodyBIdx = bodyBIdx;
    jd.groundAnchorA.Set(groundAnchorAX, groundAanchorAY);
    jd.groundAnchorB.Set(groundAnchorBX, groundAanchorBY);
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    jd.ratio = ratio;
    jd.collideConnected = collideConnect;
    jd.lengthA = lengthA;
    jd.lengthB = lengthB;
    pj = (b2PulleyJoint*)pWorld->CreateJoint(&jd);
    return static_cast<void*>(pj);
}
EXPORT float32 GetPulleyJointLengthA(b2PulleyJoint* pJoint)
{
    return pJoint->GetLengthA();
}
EXPORT float32 GetPulleyJointLengthB(b2PulleyJoint* pJoint)
{
    return pJoint->GetLengthB();
}
#pragma endregion

#pragma region GearJoints
EXPORT b2Joint* CreateGearJoint(int32 bodyAIdx, int32 bodyBIdx,
	b2Joint* pJointA, bool isARevolute, b2Joint* pJointB, bool isBRevolute, float32 ratio, bool collideConnect)
{
    b2GearJointDef jd;
    jd.bodyAIdx = bodyAIdx;
    jd.bodyBIdx = bodyBIdx;
    jd.joint1 = pJointA;
    jd.joint2 = pJointB;
    jd.ratio = ratio;
    jd.collideConnected = collideConnect;
	return pWorld->CreateJoint(&jd);
}
EXPORT void SetGearJointRatio(b2GearJoint* pJoint, float32 ratio)
{
	pJoint->SetRatio(ratio);
}
EXPORT float32 GetGearJointRatio(b2GearJoint* pJoint)
{
    return pJoint->GetRatio();
}
#pragma endregion

#pragma region WheelJoints
EXPORT b2Joint* CreateWheelJoint(int32 bodyAIdx, int32 bodyBIdx,
	float32 anchorAX, float32 anchorAY, float32 anchorBX, float32 anchorBY, float32 axisA, float32 axisB, bool collideConnect)
{
    b2WheelJointDef jd;
    jd.bodyAIdx = bodyAIdx;
    jd.bodyBIdx = bodyBIdx;
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    jd.localAxisA.Set(axisA, axisB);
    jd.collideConnected = collideConnect;
    return pWorld->CreateJoint(&jd);
}
EXPORT void SetWheelJointSpringDampingRatio(b2WheelJoint* pJoint, float32 ratio)
{
    pJoint->SetSpringDampingRatio(ratio);
}
EXPORT float32 GetWheelJointSpringDampingRatio(b2WheelJoint* pJoint)
{
    return pJoint->GetSpringDampingRatio();
}
EXPORT void SetWheelJointSpringFrequency(b2WheelJoint* pJoint, float32 frequency)
{
    pJoint->SetSpringFrequencyHz(frequency);
}
EXPORT float32 GetWheelJointSpringFrequency(b2WheelJoint* pJoint)
{
    return pJoint->GetSpringFrequencyHz();
}
EXPORT void SetWheelJointMotorSpeed(b2WheelJoint* pJoint, float32 speed)
{
    pJoint->SetMotorSpeed(speed);
}
EXPORT void SetWheelJointMaxMotorTorque(b2WheelJoint* pJoint, float32 torque)
{
    pJoint->SetMaxMotorTorque(torque);
}
EXPORT void EnableWheelJointMotor(b2WheelJoint* pJoint, bool motor)
{
    pJoint->EnableMotor(motor);
}
EXPORT bool IsWheelJointMotorEnabled(b2WheelJoint* pJoint)
{
    return pJoint->IsMotorEnabled();
}
EXPORT float32 GetWheelJointMotorSpeed(b2WheelJoint* pJoint)
{
    return pJoint->GetMotorSpeed();
}
EXPORT float32 GetWheelJointMotorTorque(b2WheelJoint* pJoint, float32 invDt)
{
    return pJoint->GetMotorTorque(invDt);
}
EXPORT float32 GetWheelJointMaxMotorTorque(b2WheelJoint* pJoint)
{
    return pJoint->GetMaxMotorTorque();
}
#pragma endregion

#pragma region WeldJoints
EXPORT b2Joint* CreateWeldJoint(int32 bodyAIdx, int32 bodyBIdx, float32 anchorAX, float32 anchorAY, float32 anchorBX, float32 anchorBY)
{
    b2WeldJointDef jd;
    jd.bodyAIdx = bodyAIdx;
    jd.bodyBIdx = bodyBIdx;
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    return pWorld->CreateJoint(&jd);
}
EXPORT float32 GetWeldJointFrequency(b2WeldJoint* pJoint)
{
    return pJoint->GetFrequency();
}
EXPORT float32 GetWeldJointDampingRatio(b2WeldJoint* pJoint)
{
    return pJoint->GetDampingRatio();
}
EXPORT void SetWeldJointFrequency(b2WeldJoint* pJoint, float32 frequency)
{
    pJoint->SetFrequency(frequency);
}
EXPORT void SetWeldJointDampingRatio(b2WeldJoint* pJoint, float32 ratio)
{
    pJoint->SetDampingRatio(ratio);
}
#pragma endregion

#pragma region FrictionJoints
EXPORT b2Joint* CreateFrictionJoint(int32 bodyAIdx, int32 bodyBIdx, float32 anchorAX, float32 anchorAY, float32 anchorBX, float32 anchorBY, bool collideConnect)
{
    b2FrictionJointDef jd;
    jd.bodyAIdx = bodyAIdx;
    jd.bodyBIdx = bodyBIdx;
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    jd.collideConnected = collideConnect;
	return pWorld->CreateJoint(&jd);
}
EXPORT float32 GetFrictionJointMaxForce(b2FrictionJoint* pJoint)
{
    return pJoint->GetMaxForce();
}
EXPORT float32 GetFrictionJointMaxTorque(b2FrictionJoint* pJoint)
{
    return pJoint->GetMaxTorque();
}
EXPORT void SetFrictionJointMaxForce(b2FrictionJoint* pJoint, float32 force)
{
    pJoint->SetMaxForce(force);
}
EXPORT void SetFrictionJointMaxTorque(b2FrictionJoint* pJoint, float32 torque)
{
    pJoint->SetMaxTorque(torque);
}
#pragma endregion

#pragma region RopeJoints
EXPORT b2Joint* CreateRopeJoint(int32 bodyAIdx, int32 bodyBIdx, float32 anchorAX,
	float32 anchorAY, float32 anchorBX, float32 anchorBY, float32 maxLength, bool collideConnect)
{
    b2RopeJointDef jd;
    jd.bodyAIdx = bodyAIdx;
    jd.bodyBIdx = bodyBIdx;
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    jd.maxLength = maxLength;
    jd.collideConnected = collideConnect;
    return pWorld->CreateJoint(&jd);
}
EXPORT float32 GetRopeJointMaxLength(b2RopeJoint* pJoint)
{
    return pJoint->GetMaxLength();
}
EXPORT void SetRopeJointMaxLength(b2RopeJoint* pJoint, float32 length)
{
	pJoint->SetMaxLength(length);
}
#pragma endregion

#pragma region MouseJoints
EXPORT b2Joint* CreateMouseJoint(int32 bodyAIdx,
	int32 bodyBIdx, float32 targetX, float32 targetY, bool collideConnect)
{
    Vec2 target = Vec2(targetX, targetY);
    b2MouseJointDef jd;
    jd.bodyAIdx = bodyAIdx;
    jd.bodyBIdx = bodyBIdx;
    jd.target = target;
    jd.collideConnected = collideConnect;
	return pWorld->CreateJoint(&jd);
}
EXPORT float32 GetMouseJointFrequency(b2MouseJoint* pJoint)
{
    return pJoint->GetFrequency();
}
EXPORT float32 GetMouseJointMaxForce(b2MouseJoint* pJoint)
{
    return pJoint->GetMaxForce();
}
EXPORT float32 GetMouseJointDampingRatio(b2MouseJoint* pJoint)
{
    return pJoint->GetDampingRatio();
}
EXPORT void SetMouseJointFrequency(b2MouseJoint* pJoint, float32 frequency)
{
	pJoint->SetFrequency(frequency);
}
EXPORT void SetMouseJointMaxForce(b2MouseJoint* pJoint, float32 maxForce)
{
	pJoint->SetMaxForce(maxForce);
}
EXPORT void SetMouseJointDampingRatio(b2MouseJoint* pJoint, float32 dampingRatio)
{
	pJoint->SetDampingRatio(dampingRatio);
}
EXPORT void SetMouseJointTarget(b2MouseJoint* pJoint, float32 targetX, float32 targetY)
{
    Vec2 target = Vec2(targetX, targetY);
	pJoint->SetTarget(target);
}
#pragma endregion

#pragma region GenericFunctions
EXPORT void DeleteJoint(b2Joint* pJoint)
{
    pWorld->DestroyJoint(pJoint);
}
EXPORT bool GetJointCollideConnected(b2Joint* pJoint)
{
    return pJoint->GetCollideConnected();
}
EXPORT void ShiftJointOrigin(b2Joint* pJoint, float32 x, float32 y)
{
	pJoint->ShiftOrigin(Vec2(x, y));
}
#pragma endregion

#pragma endregion

#pragma region Raycasting
EXPORT float32* Raycast(Vec2 start, Vec2 end, int32 mode, bool shouldQuery)
{
    newRC = new b2NewRaycastCallback(mode, shouldQuery);
    pWorld->RayCast(*newRC, start, end);
    return newRC->GetData();
}
#pragma endregion

#pragma region MemoryReleasing
EXPORT int32 ReleaseFloatArray(float32* floatArray)
{
    delete[] floatArray;
    return 0;
}
EXPORT int32 ReleaseIntArray(int* intArray)
{
    delete[] intArray;
    return 0;
}
#pragma endregion

#pragma region test
EXPORT int32 TestInt()
{
    return 114;
}
#pragma endregion



#pragma region Material

EXPORT void ClearParticleMaterials() { pPartSys->ClearMaterials(); }

EXPORT int32 CreateParticleMaterial(uint32 flags, float32 density, float32 stability,
	float32 heatConductivity, float32 strength)
{
	Particle::Mat::Def md;
	md.flags = flags;
	md.density = density;
	md.stability = stability;
	md.heatConductivity = heatConductivity;
	md.strength = strength;
	return pPartSys->CreateMaterial(md);
}
EXPORT void AddParticleMatChanges(int32 matIdx, float32 coldThreshold, int32 coldMatIdx,
	float32 hotThreshold, int32 hotMatIdx, float32 ignitionThreshold, int32 burnedMatIdx,
	int32 fireMatIdx, int32 deadMatIdx)
{
	Particle::Mat::ChangeDef mcd;
	mcd.coldThreshold = coldThreshold;
	mcd.coldMatIdx = coldMatIdx;
	mcd.hotThreshold = hotThreshold;
	mcd.hotMatIdx = hotMatIdx;
	mcd.ignitionThreshold = ignitionThreshold;
	mcd.burnedMatIdx = burnedMatIdx;
	mcd.fireMatIdx = fireMatIdx;
	mcd.deadMatIdx = deadMatIdx;
	pPartSys->AddPartMatChange(matIdx, mcd);
}

#pragma endregion



