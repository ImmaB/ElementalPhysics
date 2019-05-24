#include <Box2D/Box2D.h>
#include <utility>
#include <Box2D/Common/b2Math.h>
#include <Box2D/Common/b2GlobalVariables.h>
#include <math.h>
#include <vector>
#include <string.h>

class b2NewRaycastCallback : public b2RayCastCallback {
public:
    b2NewRaycastCallback(int32 m, bool shouldQ) : numFixtures(0), numParticles(0), mode(m), shouldQuery(shouldQ) {}
    virtual float32 ReportFixture(Fixture& fixture, const b2Vec2& point,
                                  const b2Vec2& normal, float32 fraction) {
        ++numFixtures;
        fixturesArray.push_back(point.x);
        fixturesArray.push_back(point.y);
        fixturesArray.push_back(normal.x);
        fixturesArray.push_back(normal.y);
        fixturesArray.push_back(fraction);
        return mode;
    }
    virtual float32 ReportParticle(const b2ParticleSystem* partSys,
                                   int32 index, const b2Vec2& point,
                                   const b2Vec2& normal, float32 fraction) {
        ++numParticles;
        partSys->GetUserDataBuffer ();
        particlesArray.push_back(partSys->MyIndex);
        particlesArray.push_back(index);
        particlesArray.push_back(point.x);
        particlesArray.push_back(point.y);
        particlesArray.push_back(normal.x);
        particlesArray.push_back(normal.y);
        particlesArray.push_back(fraction);
        return mode;
    }
    virtual bool ShouldQueryParticleSystem(const b2ParticleSystem* partSys) {
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
/*
class b2NewContactListener : public b2ContactListener {
public:
    b2NewContactListener() : fixtureContacts(0), mixedContacts(0), particleContacts(0) {}
    virtual void BeginContact(b2Contact* contact) {
        //if (contact->IsTouching()) {
        ++fixtureContacts;
        b2WorldManifold* m = new b2WorldManifold();
        contact->GetWorldManifold(m);
        fixturesArray.push_back((int64)contact->GetFixtureA()->GetBody()->GetUserData());
        fixturesArray.push_back((int64)contact->GetFixtureB()->GetBody()->GetUserData());
        fixturesArray.push_back((int64)contact->GetFixtureA()->GetUserData());
        fixturesArray.push_back((int64)contact->GetFixtureB()->GetUserData());
        fixturesArray.push_back(m->points[0].x);
        fixturesArray.push_back(m->points[0].y);
        fixturesArray.push_back(m->points[1].x);
        fixturesArray.push_back(m->points[1].y);
        fixturesArray.push_back(m->normal.x);
        fixturesArray.push_back(m->normal.y);
        if (contact->IsTouching()) fixturesArray.push_back(1.0f);
        else fixturesArray.push_back(0.0f);
        
        delete m;
        //}
    }
    virtual void BeginContact(b2ParticleSystem* partSys,
                              int32 particleBodyContactIdx) {
        ++mixedContacts;
        mixedArray.push_back(partSys->MyIndex); 
        mixedArray.push_back(partSys->GetBodyContactIdxs()[particleBodyContactIdx]);
		int32 bodyIdx = partSys->GetBodyContactBodyIdxs()[particleBodyContactIdx];
		b2Body* body = partSys->GetBodyBuffer()[bodyIdx];
		mixedArray.push_back((int64)body->GetUserData());
		int32 fixtureIdx = partSys->GetBodyContactFixtureIdxs()[particleBodyContactIdx];
		b2Fixture* fixture = partSys->GetFixtureBuffer()[bodyIdx];
        mixedArray.push_back((int64)fixture->GetUserData());
        mixedArray.push_back(partSys->GetBodyContactNormalXs()[particleBodyContactIdx]);
        mixedArray.push_back(partSys->GetBodyContactNormalYs()[particleBodyContactIdx]);
    }
    virtual void BeginContact(b2ParticleSystem* partSys,
                              b2ParticleContact* particleContact) {
        ++particleContacts;
        particleArray.push_back(partSys->MyIndex);
        particleArray.push_back(particleContact->GetIndexA());
        particleArray.push_back(particleContact->GetIndexB());
        //particleArray.push_back(particleContact->GetNormal().x);
        //particleArray.push_back(particleContact->GetNormal().y);
    }
    
    float32* GetData() {
        
        lengthsArray.push_back(fixtureContacts);
        lengthsArray.push_back(mixedContacts);
        lengthsArray.push_back(particleContacts);
        
        returnArray.reserve(lengthsArray.size() + fixturesArray.size() + mixedArray.size() + particleArray.size());
        returnArray.insert(returnArray.end(), lengthsArray.begin(), lengthsArray.end());
        returnArray.insert(returnArray.end(), fixturesArray.begin(), fixturesArray.end());
        returnArray.insert(returnArray.end(), mixedArray.begin(), mixedArray.end());
        returnArray.insert(returnArray.end(), particleArray.begin(), particleArray.end());
        
        lengthsArray.clear();
        fixturesArray.clear();
        mixedArray.clear();
        particleArray.clear();
        
        fixtureContacts = 0;
        mixedContacts = 0;
        particleContacts = 0;
        
        infoArray = &returnArray[0];
        returnArray.clear();
        return infoArray;
    }
private:
    int32 fixtureContacts;
    int32 mixedContacts;
    int32 particleContacts;
    float32* infoArray;
    std::vector<float> fixturesArray;
    std::vector<float> mixedArray;
    std::vector<float> particleArray;
    std::vector<float> lengthsArray;
    std::vector<float> returnArray;
};*/


#pragma region GlobalVariables
b2World* pWorld;
b2NewRaycastCallback* newRC;
float32* positionArray;
int* returnArray;
#pragma endregion

#pragma region API World
extern "C" __declspec(dllexport) b2World* CreateWorld(float32 gravX, float32 gravY) {
    b2Vec2 gravity(gravX, gravY);
	pWorld = new b2World(gravity);
    return pWorld;
}
extern "C" __declspec(dllexport) int32 DestroyWorld(b2World* pWorld)
{
	if (pWorld)
		delete pWorld;
	return 0;
}

extern "C" __declspec(dllexport) void SetStepParams(b2World* pWorld, float32 timeStep, int32 velocityIterations, int32 positionIterations, int32 particleIterations) {
	b2World* world = pWorld;
	pWorld->SetStepParams(timeStep, velocityIterations, positionIterations, particleIterations);
}
extern "C" __declspec(dllexport) void StepPreParticle(b2World* pWorld) {
	b2World* world = pWorld;
	pWorld->StepPreParticle();
}
extern "C" __declspec(dllexport) void StepPostParticle(b2World* pWorld) {
	b2World* world = pWorld;
	pWorld->StepPostParticle();
}

extern "C" __declspec(dllexport) bool GetAllowSleeping(b2World* pWorld)
{
	return pWorld->GetAllowSleeping();
}
extern "C" __declspec(dllexport) void SetAllowSleeping(b2World* pWorld, bool flag)
{
	pWorld->SetAllowSleeping(flag);
}

extern "C" __declspec(dllexport) float32* GetWorldGravity(b2World* pWorld)
{
	float32* returnArray = new float[2];
	returnArray[0] = pWorld->GetGravity().x;
	returnArray[1] = pWorld->GetGravity().y;
	return returnArray;
}
extern "C" __declspec(dllexport) void SetWorldGravity(b2World* pWorld, float32 x, float32 y)
{
	pWorld->SetGravity(b2Vec2(x, y));
}
extern "C" __declspec(dllexport) void SetWorldDamping(b2World* pWorld, float32 s)
{
	pWorld->SetDamping(s);
}

extern "C" __declspec(dllexport) int32 AddParticleMaterial(void* partSysPtr, int32 matFlags, int32 partFlags, float32 density, float32 stability, float32 heatConductivity) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	b2ParticleMaterialDef md;
	md.matFlags = matFlags;
	md.partFlags = partFlags;
	md.mass = density * partSys->GetParticleMass();
	md.stability = stability;
	md.heatConductivity = heatConductivity;
	return partSys->CreateParticleMaterial(md);
}
extern "C" __declspec(dllexport) void AddParticleMatChangeMats(void* partSysPtr, int32 matIdx, 
	float32 colderThan, int32 changeToColdMat, float32 hotterThan, int32 changeToHotMat, float32 ignitionPoint, int32 burnToMat) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	partSys->PartMatChangeMats(matIdx, colderThan, changeToColdMat, hotterThan, changeToHotMat, ignitionPoint, burnToMat);
}


extern "C" __declspec(dllexport) int32 AddBodyMaterial(b2World* worldPtr, int32 materialFlags, float32 density, float32 friction, float32 bounciness, float32 stability, float32 extinguishingPoint, float32 meltingPoint, float32 ignitionPoint, float32 heatConductivity) {
	b2BodyMaterialDef md;
	md.matFlags = materialFlags;
	md.density = density;
	md.friction = friction;
	md.bounciness = bounciness;
	md.stability = stability;
	md.extinguishingPoint = extinguishingPoint;
	md.meltingPoint = meltingPoint;
	md.ignitionPoint = ignitionPoint;
	md.heatConductivity = heatConductivity;
	const int32 idx = worldPtr->AddBodyMaterial(md);
	return idx;
}

extern "C" __declspec(dllexport) void* GetDebug() {
	return static_cast<void*>(debugString);
}



/*
extern "C" __declspec(dllexport) void* SetContactListener(b2World* pWorld)
{
	b2NewContactListener* cL = new b2NewContactListener();
	pWorld->SetContactListener(cL);
	return static_cast<void*>(cL);
}
extern "C" __declspec(dllexport) float32* UpdateContactListener(void* contactPointer) {
	b2NewContactListener* cL = static_cast<b2NewContactListener*>(contactPointer);
	return cL->GetData();
}*/
#pragma endregion

#pragma region API Particle Systems

extern "C" __declspec(dllexport) void* CreateParticleSystem(b2World* pWorld, bool accelerate, float32 radius, float32 damping, float32 gravityScale, int32 userData, float32 roomTemp, float32 heatLossRatio, float32 pointsPerLayer, int32 lowestLayer, int32 highestLayer)
{
	b2World* world = pWorld;
	const b2ParticleSystemDef particleSystemDef;
	b2ParticleSystem* partSys = pWorld->CreateParticleSystem(particleSystemDef);
	partSys->SetAccelerate(accelerate);
	partSys->SetRadius(radius);
	partSys->SetDamping(damping);
	partSys->SetGravityScale(gravityScale);
	partSys->SetIndex(userData);
	partSys->SetRoomTemperature(roomTemp);
	partSys->SetHeatLossRatio(heatLossRatio);
	partSys->SetPointsPerLayer(pointsPerLayer);
	partSys->SetLowestLayer(lowestLayer);
	partSys->SetHighestLayer(highestLayer);
	partSys->CalcLayerValues();
	partSys->MyIndex = userData;
	return static_cast<void*>(partSys);
}
extern "C" __declspec(dllexport) void SetAccelerate(void* partsysPointer, bool acc) {
	b2ParticleSystem* sys = static_cast<b2ParticleSystem*>(partsysPointer);
	sys->SetAccelerate(acc);
}
extern "C" __declspec(dllexport) void SetParticleSystemIndex(void* partsysPointer, int32 userData) {
	b2ParticleSystem* sys = static_cast<b2ParticleSystem*>(partsysPointer);

	sys->MyIndex = userData;
}
extern "C" __declspec(dllexport) int32 GetParticleIterations(float32 gravity, float32 particleRadius, float32 timeStep) {
	return b2CalculateParticleIterations(gravity, particleRadius, timeStep);
}


extern "C" __declspec(dllexport) void SolveInit(void* partSysPtr) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	partSys->SolveInit();
}
extern "C" __declspec(dllexport) void UpdateContacts(void* partSysPtr, int32 iteration) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	partSys->UpdateContacts(false);
}
extern "C" __declspec(dllexport) void SolveIteration(void* partSysPtr, int32 iteration) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	partSys->SolveIteration(iteration);
}
extern "C" __declspec(dllexport) void SolveIteration2(void* partSysPtr, int32 iteration) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	partSys->SolveIteration2(iteration);
}
extern "C" __declspec(dllexport) void SolveEnd(void* partSysPtr) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	partSys->SolveEnd();
}

extern "C" __declspec(dllexport) void SetStaticPressureIterations(void* systemPointer,int32 iterations)
{
    b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(systemPointer);
    parts->SetStaticPressureIterations(iterations);
}
extern "C" __declspec(dllexport) void SetDestructionByAge(void* systemPointer, bool toggle) {
    b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(systemPointer);
    parts->SetDestructionByAge(toggle);
}
extern "C" __declspec(dllexport) void SetDestroyStuck(void* systemPointer, bool toggle) {
	b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(systemPointer);
	//TODO
	//parts->SetDestroyStuck(toggle);
}

extern "C" __declspec(dllexport) bool GetDestructionByAge(void* systemPointer) {
    b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(systemPointer);
    return parts->GetDestructionByAge();
}
extern "C" __declspec(dllexport) int32 DestroyParticlesInShape(void* systemPointer, void* shapePointer, float32 x, float32 y, float32 rot, bool call) {
    b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(systemPointer);
    b2Shape* shape = static_cast<b2Shape*>(shapePointer);
    b2Vec2 position = b2Vec2(x, y);
    b2Rot rotation = b2Rot(rot);
    b2Transform transform = b2Transform(position, rotation);
    return parts->DestroyParticlesInShape(*shape, transform, call);
}

extern "C" __declspec(dllexport) void DeleteParticleSystem(b2World* pWorld, void* partSysPtr)
{
    b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
    pWorld->DestroyParticleSystem(partSys);
}
extern "C" __declspec(dllexport) int32 GetParticleCount(b2ParticleSystem* pPartSys)
{
    return pPartSys->GetParticleCount();
}
extern "C" __declspec(dllexport) int32 GetContactCount(b2ParticleSystem* pPartSys)
{
	return pPartSys->GetContactCount();
}
extern "C" __declspec(dllexport) void SetAllParticleFlags(void* partSysPtr, int32 flags) {
    b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
    int32 numParts = partSys->GetParticleCount();
    for (int32 i = 0; i < numParts; ++i) {
        partSys->SetParticleFlags(i, flags);
    }
}

extern "C" __declspec(dllexport) int32 GetStuckCandidateCount(void* partSysPtr) {
    b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
    return partSys->GetStuckCandidateCount();
}
extern "C" __declspec(dllexport) void SetStuckThreshold(void* partSysPtr, int32 iterations) {
    b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
    partSys->SetStuckThreshold(iterations);
}
extern "C" __declspec(dllexport) void SetMaxParticleCount(void* partSysPtr, int32 count) {
    b2ParticleSystem* system = static_cast<b2ParticleSystem*>(partSysPtr);
    system->SetMaxParticleCount(count);
}
extern "C" __declspec(dllexport) int32 GetMaxParticleCount(void* partSysPtr) {
    b2ParticleSystem* system = static_cast<b2ParticleSystem*>(partSysPtr);
    return system->GetMaxParticleCount();
}

#pragma endregion

#pragma region API Particles
extern "C" __declspec(dllexport) void AddFlagsToPartsInShape(void* partSysPtr, int32 flags, void* shapePtr, float32 shapeX, float32 shapeY, float32 shapeRot) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	b2Shape* shape = static_cast<b2Shape*>(shapePtr);
	b2Vec2 position = b2Vec2(shapeX, shapeY);
	b2Rot rotation = b2Rot(shapeRot);
	b2Transform transform = b2Transform(position, rotation); 
	b2ParticleFlag pf;
	pf = static_cast<b2ParticleFlag>(flags);

	const b2Vec3* pos = partSys->GetPositionBuffer();

	b2AABB aabb;
	int32 childCount = shape->GetChildCount();
	for (int32 childIndex = 0; childIndex < childCount; childIndex++)
	{
		shape->ComputeAABB(aabb, transform, childIndex);
		b2ParticleSystem::InsideBoundsEnumerator enumerator =
			partSys->GetInsideBoundsEnumerator(aabb);

		int32 i;
		while ((i = enumerator.GetNext()) >= 0)
		{
			if (shape->TestPoint(transform, pos[i]))
			{
				partSys->AddParticleFlags(i, pf);
			}
		}
	}
}
extern "C" __declspec(dllexport) void AddFlagsToPartsWithMatInShape(void* partSysPtr, uint32 flag, int32 matIdx, void* shapePtr, float32 shapeX, float32 shapeY, float32 shapeRot) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	b2Shape& shape = *(static_cast<b2Shape*>(shapePtr));
	partSys->AddFlagInsideShape(flag, matIdx, shape, b2Transform(b2Vec2(shapeX, shapeY), b2Rot(shapeRot)));
}

extern "C" __declspec(dllexport) void RemoveFlagsFromAll(void* partSysPtr, int32 flags) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	partSys->RemovePartFlagsFromAll(flags);
}
extern "C" __declspec(dllexport) void ApplyForceInDirIfHasFlag(void* partSysPtr, b2Vec3 pos, float32 strength, int32 flag) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	partSys->ApplyForceInDirIfHasFlag(pos, strength, flag);
}

extern "C" __declspec(dllexport) void GetAmpPositions(void* partSysPtr, void** dstPtr)
{
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	ID3D11Buffer* dst = static_cast<ID3D11Buffer*>(*dstPtr);
	partSys->CopyAmpPositions(dst);
};

extern "C" __declspec(dllexport) void GetPartBufPtrs(void* partSysPtr, 
													 int32** matIdxBufPtr,
													 b2Vec3** posBufPtr,
													 b2Vec3** velBufPtr,
													 float32** weightBufPtr,
													 float32** healthBufPtr,
													 float32** heatBufPtr)
{
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	*matIdxBufPtr = partSys->GetPartMatIdxBuffer();
	*posBufPtr	  = partSys->GetPositionBuffer();
	*velBufPtr	  = partSys->GetVelocityBuffer();
	*weightBufPtr = partSys->GetWeightBuffer();
	*healthBufPtr = partSys->GetHealthBuffer();
	*heatBufPtr   = partSys->GetHeatBuffer();
};


#pragma endregion

#pragma region ParticleGroups


extern "C" __declspec(dllexport) int32 CreatePG(void* partSysPtr, int32 partFlags, int32 groupFlags, int32 matIdx, int32 collisionGroup, float32 angle, float32 strength, float32 angVel, float32 linVelX, float32 linVelY, void* shape, int32 color, float32 stride, float32 lifetime, float32 health, float32 heat, int32 userData) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	b2Shape* m_shape = static_cast<b2Shape*>(shape);
	b2ParticleGroupFlag bgf = static_cast<b2ParticleGroupFlag>(groupFlags);
	b2ParticleFlag bf = static_cast<b2ParticleFlag>(partFlags);
	b2ParticleGroupDef pd;
	pd.flags = bf;
	pd.groupFlags = bgf;
	pd.matIdx = matIdx;
	pd.collisionGroup = collisionGroup;
	pd.shape = m_shape;
	pd.angle = angle;
	pd.strength = strength;
	pd.angularVelocity = angVel;
	pd.linearVelocity = b2Vec2(linVelX, linVelY);
	pd.stride = stride;
	pd.lifetime = lifetime;
	pd.health = health;
	pd.color = color;
	pd.userData = userData;
	pd.heat = heat;
	int32 ret = partSys->CreateGroup(pd);
	return ret;
}
extern "C" __declspec(dllexport) int32 CreatePG2(void* partSysPtr, int32 partCount, int32 partFlags, int32 groupFlags, int32 matIdx, int32 collisionGroup, float32 strength, b2Vec3* poss, float32 velX, float32 velY, int* col, float32 lifetime, float32 health, float32 heat, int32 userData) {
	b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(partSysPtr);
	b2ParticleGroupFlag bgf;
	bgf = static_cast<b2ParticleGroupFlag>(groupFlags);
	b2ParticleFlag bf;
	bf = static_cast<b2ParticleFlag>(partFlags);
	b2ParticleGroupDef pd;
	pd.flags = bf;
	pd.groupFlags = bgf;
	pd.matIdx = matIdx;
	pd.collisionGroup = collisionGroup;
	pd.strength = strength;
	pd.linearVelocity = b2Vec2(velX, velY);
	pd.lifetime = lifetime;
	pd.health = health;
	pd.userData = userData;
	pd.heat = heat;
	pd.particleCount = partCount;
	pd.positionData = poss;
	pd.colorData = col;
	int32 ret = parts->CreateGroup(pd);
	return ret;
}
extern "C" __declspec(dllexport) void JoinParticleGroups(void* partSysPtr, int32 groupAIdx, int32 groupBIdx)
{
    b2ParticleSystem* system = static_cast<b2ParticleSystem*>(partSysPtr);
    system->JoinParticleGroups(groupAIdx, groupBIdx);
}
extern "C" __declspec(dllexport) void ApplyForceToParticleGroup(void* partSysPtr, void* groupPointer, b2Vec3 force) {
	b2ParticleSystem* system = static_cast<b2ParticleSystem*>(partSysPtr);
	b2ParticleGroup* group = static_cast<b2ParticleGroup*>(groupPointer);
	system->ApplyForce(*group, force);
}
extern "C" __declspec(dllexport) void ApplyLinearImpulseToParticleGroup(void* partSysPtr, void* groupPointer, float32 forceX, float32 forceY) {
	b2ParticleSystem* system = static_cast<b2ParticleSystem*>(partSysPtr);
	b2ParticleGroup* group = static_cast<b2ParticleGroup*>(groupPointer);
    b2Vec2 impulse = b2Vec2(forceX, forceY);
	system->ApplyLinearImpulse(*group, impulse);
}
extern "C" __declspec(dllexport) void DeleteParticlesInGroup(void* partSysPtr, void* particleGroupPointer) {
	b2ParticleSystem* system = static_cast<b2ParticleSystem*>(partSysPtr);
	b2ParticleGroup* group = static_cast<b2ParticleGroup*>(particleGroupPointer);
	system->DestroyParticlesInGroup(*group);
}

#pragma endregion


#pragma region Shapes

extern "C" __declspec(dllexport) bool IsPointInShape(void* shapePointer, float32 x, float32 y, float32 rot)
{
	b2Shape* shape = static_cast<b2Shape*>(shapePointer);
	b2Vec2 position = b2Vec2(x, y);
	b2Rot rotation = b2Rot(rot);
	b2Transform transform = b2Transform(position, rotation);
	return shape->TestPoint(transform, position);
}
extern "C" __declspec(dllexport) b2PolygonShape* GetBoxShapeDef(float32 width, float32 height, float32 centreX, float32 centreY, float32 angle, float32 zPos, float32 zHeight) {
    b2Vec2 centre = b2Vec2(centreX, centreY);
    b2PolygonShape* shape = new b2PolygonShape();
    shape->SetAsBox(width, height, centre, angle);
    return shape;
}
extern "C" __declspec(dllexport) b2CircleShape* GetCircleShapeDef(float32 radius, float32 centreX, float32 centreY, float32 zPos, float32 zHeight) {
    b2Vec2 centre = b2Vec2(centreX, centreY);
    b2CircleShape* shape = new b2CircleShape();
    shape->m_p = centre;
    shape->m_radius = radius;
    return shape;
}
extern "C" __declspec(dllexport) void* GetChainShapeDef(float32* vertArray, bool loop, float32 zPos, float32 zHeight) {
    b2ChainShape* shape = new b2ChainShape();
    int32 numberOfVertices = vertArray[0];
    b2Vec2* vertices = new b2Vec2[numberOfVertices];
    for (int32 i = 1, j = 0; i < (vertArray[0] * 2); i += 2)
        vertices[j++] = b2Vec2(vertArray[i], vertArray[i + 1]);
    if (loop)
        shape->CreateLoop(vertices, numberOfVertices);
    else
        shape->CreateChain(vertices, numberOfVertices);
    return static_cast<void*>(shape);
}
extern "C" __declspec(dllexport) void* GetPolygonShapeDef(float32* vertArray, float32 zPos, float32 zHeight) {
    b2PolygonShape* shape = new b2PolygonShape();
    int32 numberOfVertices = vertArray[0];
    b2Vec2* vertices = new b2Vec2[numberOfVertices];
    
    for (int32 i = 1, j = 0; i < (vertArray[0] * 2); i += 2)
        vertices[j++] = b2Vec2(vertArray[i], vertArray[i + 1]);
    
    shape->Set(vertices, numberOfVertices);
    return static_cast<void*>(shape);
}
extern "C" __declspec(dllexport) void* GetEdgeShapeDef(float32 x1, float32 y1, float32 x2, float32 y2, float32 zPos, float32 zHeight) {
    b2Vec2 vec1 = b2Vec2(x1, y1);
    b2Vec2 vec2 = b2Vec2(x2, y2);
    b2EdgeShape* shape = new b2EdgeShape();
    shape->Set(vec1, vec2);
    return static_cast<void*>(shape);
}
extern "C" __declspec(dllexport) void* GetEllipseShapeDef(float32 outerRadius, float32 divisions, float32 zPos, float32 zHeight) {
    /*b2ChainShape* chainShape;
     std::vector<b2Vec2> vertices;
     const float32 SPIKE_DEGREE = 2 * 3.14159265358979323846f / 180;
     for (int32 idx = 0; idx < divisions; idx++) {
     float32 angle = ((3.14159265358979323846f * 2) / divisions)*idx;
     float32 xPos, yPos;
     
     xPos = outerRadius*cosf(angle);
     yPos = outerRadius*sinf(angle);
     vertices.push_back(b2Vec2(xPos, yPos));
     }
     vertices.push_back(vertices[0]);
     chainShape->CreateChain(&vertices[0], vertices.size());
     return static_cast<void*>(chainShape);*/
    
    b2PolygonShape* shape = new b2PolygonShape();
    int32 numberOfVertices = divisions;
    b2Vec2* vertices = new b2Vec2[numberOfVertices];
    
    for (int32 idx = 0; idx < divisions; idx++) {
        float32 angle = ((3.14159265358979323846f * 2) / divisions)*idx;
        float32 xPos, yPos;
        
        xPos = outerRadius*cos(angle);
        yPos = outerRadius*sin(angle);
        vertices[idx] = b2Vec2(xPos, yPos);
    }
    
    shape->Set(vertices, numberOfVertices);
    return static_cast<void*>(shape);
    
}
extern "C" __declspec(dllexport) float32* GetPolyShapeCentroid(void* shapePointer) {
    float32 * positionArray = new float[2];
    b2PolygonShape* m_shape = static_cast<b2PolygonShape*>(shapePointer);
    positionArray[0] = m_shape->m_centroid.x;
    positionArray[1] = m_shape->m_centroid.y;
    return positionArray;
}

#pragma endregion

#pragma region Body

extern "C" __declspec(dllexport) int32 CreateBody(b2World* pWorld, int32 type, float32 xPosition, float32 yPosition, float32 angle, float32 linearDamping, float32 angularDamping, int32 materialIdx, float32 heat, float32 health, int32 flags, float32 gravityScale, int32 userData) {
    b2Vec2 position = b2Vec2(xPosition, yPosition);
    b2BodyType bodyType;
	b2BodyFlag bf;
	bf = static_cast<b2BodyFlag>(flags);
    
    if (type == 1)
        bodyType = b2_dynamicBody;
    else if (type == 2)
        bodyType = b2_kinematicBody;
    else
        bodyType = b2_staticBody;
    
    b2BodyDef bd;
    bd.type = bodyType;
    bd.position = position;
    bd.angle = angle;
    bd.linearDamping = linearDamping;
    bd.angularDamping = angularDamping;
	bd.materialIdx = materialIdx;
	bd.heat = heat;
	bd.health = health;
	bd.flags = bf;
    
	bd.gravityScale = gravityScale;

    int32 bodyIdx = pWorld->CreateBody(bd);
    return bodyIdx;
}
extern "C" __declspec(dllexport) float32* GetBodyInfo(Body* pBody)
{
    if (positionArray != NULL)
        delete positionArray;

    positionArray = new float[6];
    positionArray[0] = pBody->GetPosition().x;
    positionArray[1] = pBody->GetPosition().y;
    positionArray[2] = pBody->GetAngle();
	positionArray[3] = pBody->m_health;
	positionArray[4] = pBody->m_heat;
	positionArray[5] = pBody->m_flags;
    
    return positionArray;
}
extern "C" __declspec(dllexport) float32* GetAllBodyInfo(b2World* pWorld, int* bodyIdxs, int32 numbodies)
{
	b2World* world = pWorld;
    if (positionArray != NULL)
    {
        delete positionArray;
    }
    
    positionArray = new float[numbodies*6];
    
	const std::vector<Body>& bodies = pWorld->GetBodyBuffer();
    for (int32 i = 0; i < numbodies; i++)
    {
		const Body& m_body = bodies[bodyIdxs[i]];
        positionArray[i*6] = m_body.GetPosition().x;
        positionArray[(i*6) + 1] = m_body.GetPosition().y;
        positionArray[(i*6) + 2] = m_body.GetAngle();
		positionArray[(i * 6) + 3] = m_body.m_health;
		positionArray[(i * 6) + 4] = m_body.m_heat;
		positionArray[(i * 6) + 5] = m_body.m_flags;
    }
    
    return positionArray;
}
extern "C" __declspec(dllexport) void ApplyForceToCentreOfBody(Body* pBody, float32 impulseX, float32 impulseY) {
    b2Vec2 impulse = b2Vec2(impulseX, impulseY);
	pBody->ApplyForceToCenter(impulse, true);
}
extern "C" __declspec(dllexport) void SetBodyAwake(Body* pBody, bool isAwake)
{
	pBody->SetAwake(isAwake);
}
extern "C" __declspec(dllexport) void SetBodyType(Body* pBody, int32 type)
{  
    if (type == 1)
		pBody->m_type = b2_dynamicBody;
    else if (type == 2)
		pBody->m_type = b2_kinematicBody;
    else
		pBody->m_type = b2_staticBody;
}
extern "C" __declspec(dllexport) bool GetBodyAwake(b2World* pWorld, int32 bodyIdx)
{
	const Body& body = pWorld->GetBody(bodyIdx);
    return body.IsAwake();
}
extern "C" __declspec(dllexport) void SetBodyActive(Body* pBody, bool isActive)
{
	pWorld->SetActive(*pBody, isActive);
}
extern "C" __declspec(dllexport) bool GetBodyActive(b2World* pWorld, int32 bodyIdx)
{
    const Body& body = pWorld->GetBody(bodyIdx);
    return body.IsActive();
}
extern "C" __declspec(dllexport) void SetBodyPosition(Body* pBody, float32 x, float32 y)
{
	pWorld->SetTransform(*pBody, b2Vec2(x, y), pBody->GetAngle());
}
extern "C" __declspec(dllexport) void SetBodyRotation(Body* pBody, float32 rotation) {
	pWorld->SetTransform(*pBody, pBody->GetPosition(), rotation);
}
extern "C" __declspec(dllexport) void SetBodyLinearVelocity(b2World* pWorld, int32 idx, float32 x, float32 y)
{
    b2World* world = pWorld;
	Body& body = pWorld->GetBodyBuffer()[idx];
    b2Vec2 vel = b2Vec2(x, y);
	body.SetLinearVelocity(vel);
}

extern "C" __declspec(dllexport) void ApplyAngularImpulseToBody(Body* pBody, float32 impulse, bool wake)
{
	pBody->ApplyAngularImpulse(impulse, wake);
}
extern "C" __declspec(dllexport) void ApplyForceToBody(Body* pBody, float32 forceX, float32 forceY, float32 posX, float32 posY, bool wake)
{
    b2Vec2 force = b2Vec2(forceX, forceY);
    b2Vec2 position = b2Vec2(posX, posY);
	pBody->ApplyForce(force, position, wake);
}
extern "C" __declspec(dllexport) void ApplyLinearImpulseToBody(Body* pBody, float32 forceX, float32 forceY, float32 posX, float32 posY, bool wake)
{
    b2Vec2 force = b2Vec2(forceX, forceY);
    b2Vec2 position = b2Vec2(posX, posY);
	pBody->ApplyLinearImpulse(force, position, wake);
}
extern "C" __declspec(dllexport) void ApplyTorqueToBody(Body* pBody, float32 torque, bool wake)
{
    pBody->ApplyTorque(torque, wake);
}

extern "C" __declspec(dllexport) void SetBodyTransform(Body* pBody, float32 x, float32 y, float32 angle)
{
	pWorld->SetTransform(*pBody, b2Vec2(x, y), angle);
}

extern "C" __declspec(dllexport) void DeleteBody(b2World* pWorld, int32 idx)
{
	pWorld->DestroyBody(idx);
}

#pragma endregion

#pragma region Fixture

extern "C" __declspec(dllexport) int32 AddFixture(b2World* pWorld, int32 bodyIdx, int32 shapeType, int32 shapeIdx, bool isSensor)
{
	b2World* world = pWorld;
	Body& body = pWorld->GetBodyBuffer()[bodyIdx];
    b2FixtureDef fd;
	fd.isSensor = isSensor;
    fd.shapeIdx = shapeIdx;
    int32 idx = pWorld->CreateFixture(body, fd);
    return idx;
}

extern "C" __declspec(dllexport) bool TestPoint(Fixture* pFixture, float32 x, float32 y)
{
	return pWorld->TestPoint(*pFixture, b2Vec2(x, y));
}

extern "C" __declspec(dllexport) void SetFixtureFilterData(b2World* pWorld, int32 idx, int32 groupIndex, uint16 categoryBits, uint16 maskBits)
{
    b2Filter filter = b2Filter();
    filter.groupIndex = groupIndex;
    filter.maskBits = maskBits;
    filter.categoryBits = categoryBits;
	Fixture& fixture = pWorld->GetFixtureBuffer()[idx];
	pWorld->SetFilterData(fixture, filter);
}
extern "C" __declspec(dllexport) uint16 GetFixtureGroupIndex(Fixture* pFixture)
{
    return pFixture->m_filter.groupIndex;
}
extern "C" __declspec(dllexport) uint16 GetFixtureMaskBits(Fixture* pFixture)
{
    return pFixture->m_filter.maskBits;
}
extern "C" __declspec(dllexport) uint16 GetFixtureCategoryBits(Fixture* pFixture)
{
    return pFixture->m_filter.categoryBits;
}

extern "C" __declspec(dllexport) bool GetFixtureIsSensor(Fixture* pFixture)
{
    return pFixture->IsSensor();
}
extern "C" __declspec(dllexport) void SetFixtureIsSensor(Fixture* pFixture, bool flag)
{
	pWorld->SetSensor(*pFixture, flag);
}

extern "C" __declspec(dllexport) float32 GetFixtureDensity(Fixture* pFixture)
{
    return pFixture->m_density;
}
extern "C" __declspec(dllexport) void SetFixtureDensity(Fixture* pFixture, float32 density)
{
    pFixture->SetDensity(density);
}

extern "C" __declspec(dllexport) void DeleteFixture(Body* pBody, int32 idx)
{
	pWorld->DestroyFixture(*pBody, idx);
}
#pragma endregion

#pragma region Joints

#pragma region DistanceJoints
extern "C" __declspec(dllexport) b2DistanceJoint* CreateDistanceJoint(b2World* pWorld, Body* pBodyA, Body* pBodyB, float32 anchorAX, float32 anchorAY, float32 anchorBX, float32 anchorBY, float32 length, bool collideConnected)
{
    b2DistanceJoint* dj;
    b2DistanceJointDef jd;
    jd.bodyA = pBodyA;
    jd.bodyB = pBodyB;
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    jd.collideConnected = collideConnected;
    jd.length = length;
    dj = (b2DistanceJoint*)pWorld->CreateJoint(&jd);
    return dj;
}
extern "C" __declspec(dllexport) void SetDistanceJointFrequency(b2DistanceJoint* pJoint, float32 frequency)
{
	pJoint->SetFrequency(frequency);
}
extern "C" __declspec(dllexport) float32 GetDistanceJointFrequency(b2DistanceJoint* pJoint)
{
    return pJoint->GetFrequency();
}
extern "C" __declspec(dllexport) void SetDistanceJointDampingRatio(b2DistanceJoint* pJoint, float32 ratio)
{
	pJoint->SetDampingRatio(ratio);
}
extern "C" __declspec(dllexport) float32 GetDistanceJointDampingRatio(b2DistanceJoint* pJoint)
{
    return pJoint->GetDampingRatio();
}
extern "C" __declspec(dllexport) void SetDistanceJointLength(b2DistanceJoint* pJoint, float32 length)
{
	pJoint->SetLength(length);
}
extern "C" __declspec(dllexport) float32 GetDistanceJointLength(b2DistanceJoint* pJoint)
{
    return pJoint->GetLength();
}
#pragma endregion

#pragma region RevoluteJoints
extern "C" __declspec(dllexport) void* CreateRevoluteJoint(b2World* pWorld, Body* pBodyA, Body* pBodyB, float32 anchorAX, float32 anchorAY, float32 anchorBX, float32 anchorBY, bool collideConnected)
{
    b2RevoluteJoint* rj;
    b2RevoluteJointDef jd;
    jd.bodyA = pBodyA;
    jd.bodyB = pBodyB;
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    jd.collideConnected = collideConnected;
    rj = (b2RevoluteJoint*)pWorld->CreateJoint(&jd);
    return static_cast<void*>(rj);
}
extern "C" __declspec(dllexport) void SetRevoluteJointLimits(b2RevoluteJoint* pJoint, float32 lower, float32 upper)
{
	pJoint->SetLimits(lower, upper);
}
extern "C" __declspec(dllexport) void SetRevoluteJointMotorSpeed(b2RevoluteJoint* pJoint, float32 speed)
{
	pJoint->SetMotorSpeed(speed);
}
extern "C" __declspec(dllexport) void SetRevoluteJointMaxMotorTorque(b2RevoluteJoint* pJoint, float32 torque)
{
	pJoint->SetMaxMotorTorque(torque);
}
extern "C" __declspec(dllexport) void EnableRevoluteJointMotor(b2RevoluteJoint* pJoint, bool motor)
{
	pJoint->EnableMotor(motor);
}
extern "C" __declspec(dllexport) void EnableRevoluteJointLimits(b2RevoluteJoint* pJoint, bool limit)
{
	pJoint->EnableLimit(limit);
}
extern "C" __declspec(dllexport) float32 GetRevoluteJointUpperLimit(b2RevoluteJoint* pJoint)
{
    return pJoint->GetUpperLimit();
}
extern "C" __declspec(dllexport) float32 GetRevoluteJointLowerLimit(b2RevoluteJoint* pJoint)
{
    return pJoint->GetLowerLimit();
}
extern "C" __declspec(dllexport) bool IsRevoluteJointMotorEnabled(b2RevoluteJoint* pJoint)
{
    return pJoint->IsMotorEnabled();
}
extern "C" __declspec(dllexport) float32 GetRevoluteJointMotorSpeed(b2RevoluteJoint* pJoint)
{
    return pJoint->GetMotorSpeed();
}
extern "C" __declspec(dllexport) float32 GetRevoluteJointMotorTorque(b2RevoluteJoint* pJoint, float32 invDt)
{
    return pJoint->GetMotorTorque(invDt);
}
extern "C" __declspec(dllexport) float32 GetRevoluteJointMaxMotorTorque(b2RevoluteJoint* pJoint)
{
    return pJoint->GetMaxMotorTorque();
}
#pragma endregion

#pragma region PrismaticJoints
extern "C" __declspec(dllexport) void* CreatePrismaticJoint(b2World* pWorld, Body* pBodyA, Body* pBodyB, float32 anchorAX, float32 anchorAY, float32 anchorBX, float32 anchorBY, float32 axisX, float32 axisY, bool collideConnect)
{
    b2PrismaticJoint* pj;
    b2PrismaticJointDef jd;
    jd.bodyA = pBodyA;
    jd.bodyB = pBodyB;
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    jd.localAxisA.Set(axisX, axisY);
    jd.collideConnected = collideConnect;
    pj = (b2PrismaticJoint*)pWorld->CreateJoint(&jd);
    return static_cast<void*>(pj);
}
extern "C" __declspec(dllexport) void SetPrismaticJointLimits(b2PrismaticJoint* pJoint, float32 lower, float32 upper)
{
	pJoint->SetLimits(lower, upper);
}
extern "C" __declspec(dllexport) void SetPrismaticJointMotorSpeed(b2PrismaticJoint* pJoint, float32 speed)
{
	pJoint->SetMotorSpeed(speed);
}
extern "C" __declspec(dllexport) void SetPrismaticJointMaxMotorForce(b2PrismaticJoint* pJoint, float32 force)
{
	pJoint->SetMaxMotorForce(force);
}
extern "C" __declspec(dllexport) void EnablePrismaticJointMotor(b2PrismaticJoint* pJoint, bool motor)
{
	pJoint->EnableMotor(motor);
}
extern "C" __declspec(dllexport) void EnablePrismaticJointLimits(b2PrismaticJoint* pJoint, bool limit)
{
	pJoint->EnableLimit(limit);
}
extern "C" __declspec(dllexport) float32 GetPrismaticJointUpperLimit(b2PrismaticJoint* pJoint)
{
    return pJoint->GetUpperLimit();
}
extern "C" __declspec(dllexport) float32 GetPrismaticJointLowerLimit(b2PrismaticJoint* pJoint)
{
    return pJoint->GetLowerLimit();
}
extern "C" __declspec(dllexport) bool IsPrismaticJointMotorEnabled(b2PrismaticJoint* pJoint)
{
    return pJoint->IsMotorEnabled();
}
extern "C" __declspec(dllexport) float32 GetPrismaticJointMotorSpeed(b2PrismaticJoint* pJoint)
{
    return pJoint->GetMotorSpeed();
}
extern "C" __declspec(dllexport) float32 GetPrismaticJointMotorTorque(b2PrismaticJoint* pJoint, float32 invDt)
{
    return pJoint->GetMotorForce(invDt);
}
extern "C" __declspec(dllexport) float32 GetPrismaticJointMaxMotorForce(b2PrismaticJoint* pJoint)
{
    return pJoint->GetMaxMotorForce();
}
extern "C" __declspec(dllexport) float32 GetPrismaticJointMotorForce(b2PrismaticJoint* pJoint,float32 its)
{
    return pJoint->GetMotorForce(its);
}
extern "C" __declspec(dllexport) float32 GetPrismaticJointSpeed(b2PrismaticJoint* pJoint)
{
    return pJoint->GetJointSpeed();
}
#pragma endregion

#pragma region PulleyJoints
extern "C" __declspec(dllexport) void* CreatePulleyJoint(b2World* pWorld, Body* pBodyA, Body* pBodyB, float32 groundAnchorAX,
	float32 groundAanchorAY, float32 groundAnchorBX, float32 groundAanchorBY, float32 anchorAX, float32 anchorAY, float32 anchorBX,
	float32 anchorBY, float32 ratio, float32 lengthA, float32 lengthB, bool collideConnect)
{


    b2PulleyJoint* pj;
    b2PulleyJointDef jd;
    jd.bodyA = pBodyA;
    jd.bodyB = pBodyB;
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
extern "C" __declspec(dllexport) float32 GetPulleyJointLengthA(b2PulleyJoint* pJoint)
{
    return pJoint->GetLengthA();
}
extern "C" __declspec(dllexport) float32 GetPulleyJointLengthB(b2PulleyJoint* pJoint)
{
    return pJoint->GetLengthB();
}
#pragma endregion

#pragma region GearJoints
extern "C" __declspec(dllexport) b2Joint* CreateGearJoint(b2World* pWorld, Body* pBodyA, Body* pBodyB,
	b2Joint* pJointA, bool isARevolute, b2Joint* pJointB, bool isBRevolute, float32 ratio, bool collideConnect)
{
    b2GearJointDef jd;
    jd.bodyA = pBodyA;
    jd.bodyB = pBodyB;
    jd.joint1 = pJointA;
    jd.joint2 = pJointB;
    jd.ratio = ratio;
    jd.collideConnected = collideConnect;
	return pWorld->CreateJoint(&jd);
}
extern "C" __declspec(dllexport) void SetGearJointRatio(b2GearJoint* pJoint, float32 ratio)
{
	pJoint->SetRatio(ratio);
}
extern "C" __declspec(dllexport) float32 GetGearJointRatio(b2GearJoint* pJoint)
{
    return pJoint->GetRatio();
}
#pragma endregion

#pragma region WheelJoints
extern "C" __declspec(dllexport) b2Joint* CreateWheelJoint(b2World* pWorld, Body* pBodyA, Body* pBodyB,
	float32 anchorAX, float32 anchorAY, float32 anchorBX, float32 anchorBY, float32 axisA, float32 axisB, bool collideConnect)
{
    b2WheelJointDef jd;
    jd.bodyA = pBodyA;
    jd.bodyB = pBodyB;
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    jd.localAxisA.Set(axisA, axisB);
    jd.collideConnected = collideConnect;
    return pWorld->CreateJoint(&jd);
}
extern "C" __declspec(dllexport) void SetWheelJointSpringDampingRatio(b2WheelJoint* pJoint, float32 ratio)
{
    pJoint->SetSpringDampingRatio(ratio);
}
extern "C" __declspec(dllexport) float32 GetWheelJointSpringDampingRatio(b2WheelJoint* pJoint)
{
    return pJoint->GetSpringDampingRatio();
}
extern "C" __declspec(dllexport) void SetWheelJointSpringFrequency(b2WheelJoint* pJoint, float32 frequency)
{
    pJoint->SetSpringFrequencyHz(frequency);
}
extern "C" __declspec(dllexport) float32 GetWheelJointSpringFrequency(b2WheelJoint* pJoint)
{
    return pJoint->GetSpringFrequencyHz();
}
extern "C" __declspec(dllexport) void SetWheelJointMotorSpeed(b2WheelJoint* pJoint, float32 speed)
{
    pJoint->SetMotorSpeed(speed);
}
extern "C" __declspec(dllexport) void SetWheelJointMaxMotorTorque(b2WheelJoint* pJoint, float32 torque)
{
    pJoint->SetMaxMotorTorque(torque);
}
extern "C" __declspec(dllexport) void EnableWheelJointMotor(b2WheelJoint* pJoint, bool motor)
{
    pJoint->EnableMotor(motor);
}
extern "C" __declspec(dllexport) bool IsWheelJointMotorEnabled(b2WheelJoint* pJoint)
{
    return pJoint->IsMotorEnabled();
}
extern "C" __declspec(dllexport) float32 GetWheelJointMotorSpeed(b2WheelJoint* pJoint)
{
    return pJoint->GetMotorSpeed();
}
extern "C" __declspec(dllexport) float32 GetWheelJointMotorTorque(b2WheelJoint* pJoint, float32 invDt)
{
    return pJoint->GetMotorTorque(invDt);
}
extern "C" __declspec(dllexport) float32 GetWheelJointMaxMotorTorque(b2WheelJoint* pJoint)
{
    return pJoint->GetMaxMotorTorque();
}
#pragma endregion

#pragma region WeldJoints
extern "C" __declspec(dllexport) b2Joint* CreateWeldJoint(b2World* pWorld, Body* pBodyA, Body* pBodyB, float32 anchorAX, float32 anchorAY, float32 anchorBX, float32 anchorBY)
{
    b2WeldJointDef jd;
    jd.bodyA = pBodyA;
    jd.bodyB = pBodyB;
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    return pWorld->CreateJoint(&jd);
}
extern "C" __declspec(dllexport) float32 GetWeldJointFrequency(b2WeldJoint* pJoint)
{
    return pJoint->GetFrequency();
}
extern "C" __declspec(dllexport) float32 GetWeldJointDampingRatio(b2WeldJoint* pJoint)
{
    return pJoint->GetDampingRatio();
}
extern "C" __declspec(dllexport) void SetWeldJointFrequency(b2WeldJoint* pJoint, float32 frequency)
{
    pJoint->SetFrequency(frequency);
}
extern "C" __declspec(dllexport) void SetWeldJointDampingRatio(b2WeldJoint* pJoint, float32 ratio)
{
    pJoint->SetDampingRatio(ratio);
}
#pragma endregion

#pragma region FrictionJoints
extern "C" __declspec(dllexport) b2Joint* CreateFrictionJoint(b2World* pWorld, Body* pBodyA, Body* pBodyB, float32 anchorAX, float32 anchorAY, float32 anchorBX, float32 anchorBY, bool collideConnect)
{
    b2FrictionJointDef jd;
    jd.bodyA = pBodyA;
    jd.bodyB = pBodyB;
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    jd.collideConnected = collideConnect;
	return pWorld->CreateJoint(&jd);
}
extern "C" __declspec(dllexport) float32 GetFrictionJointMaxForce(b2FrictionJoint* pJoint)
{
    return pJoint->GetMaxForce();
}
extern "C" __declspec(dllexport) float32 GetFrictionJointMaxTorque(b2FrictionJoint* pJoint)
{
    return pJoint->GetMaxTorque();
}
extern "C" __declspec(dllexport) void SetFrictionJointMaxForce(b2FrictionJoint* pJoint, float32 force)
{
    pJoint->SetMaxForce(force);
}
extern "C" __declspec(dllexport) void SetFrictionJointMaxTorque(b2FrictionJoint* pJoint, float32 torque)
{
    pJoint->SetMaxTorque(torque);
}
#pragma endregion

#pragma region RopeJoints
extern "C" __declspec(dllexport) b2Joint* CreateRopeJoint(b2World* pWorld, Body* pBodyA, Body* pBodyB, float32 anchorAX,
	float32 anchorAY, float32 anchorBX, float32 anchorBY, float32 maxLength, bool collideConnect)
{
    b2RopeJointDef jd;
    jd.bodyA = pBodyA;
    jd.bodyB = pBodyB;
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    jd.maxLength = maxLength;
    jd.collideConnected = collideConnect;
    return pWorld->CreateJoint(&jd);
}
extern "C" __declspec(dllexport) float32 GetRopeJointMaxLength(b2RopeJoint* pJoint)
{
    return pJoint->GetMaxLength();
}
extern "C" __declspec(dllexport) void SetRopeJointMaxLength(b2RopeJoint* pJoint, float32 length)
{
	pJoint->SetMaxLength(length);
}
#pragma endregion

#pragma region MouseJoints
extern "C" __declspec(dllexport) b2Joint* CreateMouseJoint(b2World* pWorld, Body* pBodyA,
	Body* pBodyB, float32 targetX, float32 targetY, bool collideConnect)
{
    b2Vec2 target = b2Vec2(targetX, targetY);
    b2MouseJointDef jd;
    jd.bodyA = pBodyA;
    jd.bodyB = pBodyB;
    jd.target = target;
    jd.collideConnected = collideConnect;
	return pWorld->CreateJoint(&jd);
}
extern "C" __declspec(dllexport) float32 GetMouseJointFrequency(b2MouseJoint* pJoint)
{
    return pJoint->GetFrequency();
}
extern "C" __declspec(dllexport) float32 GetMouseJointMaxForce(b2MouseJoint* pJoint)
{
    return pJoint->GetMaxForce();
}
extern "C" __declspec(dllexport) float32 GetMouseJointDampingRatio(b2MouseJoint* pJoint)
{
    return pJoint->GetDampingRatio();
}
extern "C" __declspec(dllexport) void SetMouseJointFrequency(b2MouseJoint* pJoint, float32 frequency)
{
	pJoint->SetFrequency(frequency);
}
extern "C" __declspec(dllexport) void SetMouseJointMaxForce(b2MouseJoint* pJoint, float32 maxForce)
{
	pJoint->SetMaxForce(maxForce);
}
extern "C" __declspec(dllexport) void SetMouseJointDampingRatio(b2MouseJoint* pJoint, float32 dampingRatio)
{
	pJoint->SetDampingRatio(dampingRatio);
}
extern "C" __declspec(dllexport) void SetMouseJointTarget(b2MouseJoint* pJoint, float32 targetX, float32 targetY)
{
    b2Vec2 target = b2Vec2(targetX, targetY);
	pJoint->SetTarget(target);
}
#pragma endregion

#pragma region GenericFunctions
extern "C" __declspec(dllexport) void DeleteJoint(b2World* pWorld, b2Joint* pJoint)
{
    pWorld->DestroyJoint(pJoint);
}
extern "C" __declspec(dllexport) bool GetJointCollideConnected(b2Joint* pJoint)
{
    return pJoint->GetCollideConnected();
}
extern "C" __declspec(dllexport) void ShiftJointOrigin(b2Joint* pJoint, float32 x, float32 y)
{
	pJoint->ShiftOrigin(b2Vec2(x, y));
}
#pragma endregion

#pragma endregion

#pragma region Raycasting
extern "C" __declspec(dllexport) float32* RaycastWorld(b2World* pWorld, float32 x1, float32 y1, float32 x2, float32 y2, int32 mode, bool shouldQuery)
{
    newRC = new b2NewRaycastCallback(mode, shouldQuery);
    b2Vec2 pos1 = b2Vec2(x1, y1);
    b2Vec2 pos2 = b2Vec2(x2, y2);
    pWorld->RayCast(*newRC, pos1, pos2);
    return newRC->GetData();
}
#pragma endregion

#pragma region MemoryReleasing
extern "C" __declspec(dllexport) int32 ReleaseFloatArray(float32* floatArray)
{
    delete[] floatArray;
    return 0;
}
extern "C" __declspec(dllexport) int32 ReleaseIntArray(int* intArray)
{
    delete[] intArray;
    return 0;
}
extern "C" __declspec(dllexport) int32 ReleaseShape(b2Shape* shape)
{
    delete shape;
    return 0;
}
#pragma endregion

#pragma region test
extern "C" __declspec(dllexport) int32 TestInt()
{
    return 114;
}
#pragma endregion