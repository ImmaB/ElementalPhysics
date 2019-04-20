#include <Box2D/Box2D.h>
#include <utility>
#include <Box2D/Common/b2Math.h>
#include <Box2D/Common/b2GlobalVariables.h>
#include <math.h>
#include <vector>
#include <string.h>

class b2NewRaycastCallback : public b2RayCastCallback {
public:
    b2NewRaycastCallback(int m, bool shouldQ) : numFixtures(0), numParticles(0), mode(m), shouldQuery(shouldQ) {}
    virtual float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point,
                                  const b2Vec2& normal, float32 fraction) {
        ++numFixtures;
        fixturesArray.push_back((int64)fixture->GetBody()->GetUserData());
        fixturesArray.push_back((int64)fixture->GetUserData());
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
    float* GetData() {
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
    float* positionArray;
    int numFixtures;
    int numParticles;
    int mode;
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
    
    float* GetData() {
        
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
    int fixtureContacts;
    int mixedContacts;
    int particleContacts;
    float* infoArray;
    std::vector<float> fixturesArray;
    std::vector<float> mixedArray;
    std::vector<float> particleArray;
    std::vector<float> lengthsArray;
    std::vector<float> returnArray;
};*/


#pragma region GlobalVariables
b2World* world;
b2NewRaycastCallback* newRC;
float* positionArray;
int* returnArray;
#pragma endregion

#pragma region API World
extern "C" __declspec(dllexport) void* CreateWorld(float gravX, float gravY) {
    b2Vec2 gravity(gravX, gravY);
	world = new b2World(gravity);
    return static_cast<void*>(world);
}
extern "C" __declspec(dllexport) int DestroyWorld(void* worldPointer) {
	b2World* world = static_cast<b2World*>(worldPointer);
	if (world)
		delete world;
	return 0;
}

extern "C" __declspec(dllexport) void SetStepParams(void* worldPtr, float32 timeStep, int32 velocityIterations, int32 positionIterations, int32 particleIterations) {
	b2World* world = static_cast<b2World*>(worldPtr);
	world->SetStepParams(timeStep, velocityIterations, positionIterations, particleIterations);
}
extern "C" __declspec(dllexport) void StepPreParticle(void* worldPtr) {
	b2World* world = static_cast<b2World*>(worldPtr);
	world->StepPreParticle();
}
extern "C" __declspec(dllexport) void StepPostParticle(void* worldPtr) {
	b2World* world = static_cast<b2World*>(worldPtr);
	world->StepPostParticle();
}

extern "C" __declspec(dllexport) bool GetAllowSleeping(void* worldPointer) {
	b2World* world = static_cast<b2World*>(worldPointer);
	return world->GetAllowSleeping();
}
extern "C" __declspec(dllexport) void SetAllowSleeping(void* worldPointer, bool flag) {
	b2World* world = static_cast<b2World*>(worldPointer);
	world->SetAllowSleeping(flag);
}

extern "C" __declspec(dllexport) float* GetWorldGravity(void* worldPointer) {
	b2World* world = static_cast<b2World*>(worldPointer);
	float* returnArray = new float[2];
	returnArray[0] = world->GetGravity().x;
	returnArray[1] = world->GetGravity().y;
	return returnArray;
}
extern "C" __declspec(dllexport) void SetWorldGravity(void* worldPointer, float x, float y) {
	b2World* world = static_cast<b2World*>(worldPointer);
	world->SetGravity(b2Vec2(x, y));
}
extern "C" __declspec(dllexport) void SetWorldDamping(void* worldPointer, float s) {
	b2World* world = static_cast<b2World*>(worldPointer);
	world->SetDamping(s);
}

extern "C" __declspec(dllexport) int AddParticleMaterial(void* partSysPtr, int matFlags, int partFlags, float density, float stability, float heatConductivity) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	b2ParticleMaterialDef md;
	md.matFlags = matFlags;
	md.partFlags = partFlags;
	md.mass = density * partSys->GetParticleMass();
	md.stability = stability;
	md.heatConductivity = heatConductivity;
	return partSys->CreateParticleMaterial(md);
}
extern "C" __declspec(dllexport) void AddParticleMatChangeMats(void* partSysPtr, int matIdx, 
	float colderThan, int changeToColdMat, float hotterThan, int changeToHotMat, float ignitionPoint, int burnToMat) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	partSys->PartMatChangeMats(matIdx, colderThan, changeToColdMat, hotterThan, changeToHotMat, ignitionPoint, burnToMat);
}


extern "C" __declspec(dllexport) void* AddBodyMaterial(void* worldPointer, int materialFlags, float density, float friction, float bounciness, float stability, float extinguishingPoint, float meltingPoint, float ignitionPoint, float heatConductivity) {
	b2World* world = static_cast<b2World*>(worldPointer);
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
	b2BodyMaterial* matPtr = world->AddBodyMaterial(md);
	return static_cast<void*>(matPtr);
}

extern "C" __declspec(dllexport) void* GetDebug() {
	return static_cast<void*>(debugString);
}



/*
extern "C" __declspec(dllexport)  void* SetContactListener(void* worldPointer) {
	b2World* world = static_cast<b2World*>(worldPointer);
	b2NewContactListener* cL = new b2NewContactListener();
	world->SetContactListener(cL);
	return static_cast<void*>(cL);
}
extern "C" __declspec(dllexport)  float* UpdateContactListener(void* contactPointer) {
	b2NewContactListener* cL = static_cast<b2NewContactListener*>(contactPointer);
	return cL->GetData();
}*/
#pragma endregion

#pragma region API Particle Systems

extern "C" __declspec(dllexport)  void* CreateParticleSystem(void* worldPtr, bool accelerate, float radius, float damping, float gravityScale, int userData, float roomTemp, float heatLossRatio, float pointsPerLayer, int lowestLayer, int highestLayer)
{
	b2World* world = static_cast<b2World*>(worldPtr);
	const b2ParticleSystemDef particleSystemDef;
	b2ParticleSystem* partSys = world->CreateParticleSystem(&particleSystemDef);
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
extern "C" __declspec(dllexport)  void SetAccelerate(void* partsysPointer, bool acc) {
	b2ParticleSystem* sys = static_cast<b2ParticleSystem*>(partsysPointer);
	sys->SetAccelerate(acc);
}
extern "C" __declspec(dllexport)  void SetParticleSystemIndex(void* partsysPointer, int userData) {
	b2ParticleSystem* sys = static_cast<b2ParticleSystem*>(partsysPointer);

	sys->MyIndex = userData;
}
extern "C" __declspec(dllexport)  int32 GetParticleIterations(float32 gravity, float32 particleRadius, float32 timeStep) {
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

extern "C" __declspec(dllexport)  void SetStaticPressureIterations(void* systemPointer,int iterations)
{
    b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(systemPointer);
    parts->SetStaticPressureIterations(iterations);
}
extern "C" __declspec(dllexport)  void SetDestructionByAge(void* systemPointer, bool toggle) {
    b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(systemPointer);
    parts->SetDestructionByAge(toggle);
}
extern "C" __declspec(dllexport)  void SetDestroyStuck(void* systemPointer, bool toggle) {
	b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(systemPointer);
	//TODO
	//parts->SetDestroyStuck(toggle);
}

extern "C" __declspec(dllexport)  bool GetDestructionByAge(void* systemPointer) {
    b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(systemPointer);
    return parts->GetDestructionByAge();
}
extern "C" __declspec(dllexport)  int DestroyParticlesInShape(void* systemPointer, void* shapePointer, float x, float y, float rot, bool call) {
    b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(systemPointer);
    b2Shape* shape = static_cast<b2Shape*>(shapePointer);
    b2Vec2 position = b2Vec2(x, y);
    b2Rot rotation = b2Rot(rot);
    b2Transform transform = b2Transform(position, rotation);
    return parts->DestroyParticlesInShape(*shape, transform, call);
}

extern "C" __declspec(dllexport)  void DeleteParticleSystem(void* worldPointer, void* partSysPtr) {
    b2World* world = static_cast<b2World*>(worldPointer);
    b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
    world->DestroyParticleSystem(partSys);
}
extern "C" __declspec(dllexport)  int GetNumberOfParticles(void* partSysPtr) {
    b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
    
    return partSys->GetParticleCount();
}
extern "C" __declspec(dllexport)  void SetAllParticleFlags(void* partSysPtr, int flags) {
    b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
    int numParts = partSys->GetParticleCount();
    for (int i = 0; i < numParts; ++i) {
        partSys->SetParticleFlags(i, flags);
    }
}

extern "C" __declspec(dllexport)  int GetStuckCandidateCount(void* partSysPtr) {
    b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
    return partSys->GetStuckCandidateCount();
}
extern "C" __declspec(dllexport)  void SetStuckThreshold(void* partSysPtr, int iterations) {
    b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
    partSys->SetStuckThreshold(iterations);
}
extern "C" __declspec(dllexport)  void SetMaxParticleCount(void* partSysPtr, int count) {
    b2ParticleSystem* system = static_cast<b2ParticleSystem*>(partSysPtr);
    system->SetMaxParticleCount(count);
}
extern "C" __declspec(dllexport)  int GetMaxParticleCount(void* partSysPtr) {
    b2ParticleSystem* system = static_cast<b2ParticleSystem*>(partSysPtr);
    return system->GetMaxParticleCount();
}

#pragma endregion

#pragma region API Particles
extern "C" __declspec(dllexport)  void AddFlagsToPartsInShape(void* partSysPtr, int flags, void* shapePtr, float shapeX, float shapeY, float shapeRot) {
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
extern "C" __declspec(dllexport)  void AddFlagsToPartsWithMatInShape(void* partSysPtr, int flags, int matIdx, void* shapePtr, float shapeX, float shapeY, float shapeRot) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	b2Shape* shape = static_cast<b2Shape*>(shapePtr);
	b2Vec2 position = b2Vec2(shapeX, shapeY);
	b2Rot rotation = b2Rot(shapeRot);
	b2Transform transform = b2Transform(position, rotation);
	b2ParticleFlag pf;
	pf = static_cast<b2ParticleFlag>(flags);

	const b2Vec3* pos = partSys->GetPositionBuffer();
	const int32* matIdxs = partSys->GetPartMatIdxBuffer();

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
			if (matIdxs[i] == matIdx && shape->TestPoint(transform, pos[i]))
			{
				partSys->AddParticleFlags(i, pf);
			}
		}
	}
}

extern "C" __declspec(dllexport)  void RemoveFlagsFromAll(void* partSysPtr, int flags) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	partSys->RemovePartFlagsFromAll(flags);
}
extern "C" __declspec(dllexport)  void ApplyForceInDirIfHasFlag(void* partSysPtr, b2Vec3 pos, float strength, int flag) {
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


extern "C" __declspec(dllexport)  int CreatePG(void* partSysPtr, int partFlags, int groupFlags, int matIdx, int collisionGroup, float angle, float strength, float angVel, float linVelX, float linVelY, void* shape, int color, float stride, float lifetime, float health, float heat, int userData) {
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
	int ret = partSys->CreateGroup(pd);
	return ret;
}
extern "C" __declspec(dllexport)  int CreatePG2(void* partSysPtr, int partCount, int partFlags, int groupFlags, int matIdx, int collisionGroup, float strength, b2Vec3* poss, float velX, float velY, int* col, float lifetime, float health, float heat, int userData) {
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
	int ret = parts->CreateGroup(pd);
	return ret;
}
extern "C" __declspec(dllexport)  void JoinParticleGroups(void* partSysPtr, int groupAIdx, int groupBIdx)
{
    b2ParticleSystem* system = static_cast<b2ParticleSystem*>(partSysPtr);
    system->JoinParticleGroups(groupAIdx, groupBIdx);
}
extern "C" __declspec(dllexport)  void ApplyForceToParticleGroup(void* partSysPtr, void* groupPointer, b2Vec3 force) {
	b2ParticleSystem* system = static_cast<b2ParticleSystem*>(partSysPtr);
	b2ParticleGroup* group = static_cast<b2ParticleGroup*>(groupPointer);
	system->ApplyForce(*group, force);
}
extern "C" __declspec(dllexport)  void ApplyLinearImpulseToParticleGroup(void* partSysPtr, void* groupPointer, float forceX, float forceY) {
	b2ParticleSystem* system = static_cast<b2ParticleSystem*>(partSysPtr);
	b2ParticleGroup* group = static_cast<b2ParticleGroup*>(groupPointer);
    b2Vec2 impulse = b2Vec2(forceX, forceY);
	system->ApplyLinearImpulse(*group, impulse);
}
extern "C" __declspec(dllexport)  void DeleteParticlesInGroup(void* partSysPtr, void* particleGroupPointer) {
	b2ParticleSystem* system = static_cast<b2ParticleSystem*>(partSysPtr);
	b2ParticleGroup* group = static_cast<b2ParticleGroup*>(particleGroupPointer);
	system->DestroyParticlesInGroup(*group);
}

#pragma endregion


#pragma region Shapes

extern "C" __declspec(dllexport)  bool IsPointInShape(void* shapePointer, float x, float y, float rot)
{
	b2Shape* shape = static_cast<b2Shape*>(shapePointer);
	b2Vec2 position = b2Vec2(x, y);
	b2Rot rotation = b2Rot(rot);
	b2Transform transform = b2Transform(position, rotation);
	return shape->TestPoint(transform, position);
}
extern "C" __declspec(dllexport)  void* GetBoxShapeDef(float width, float height, float centreX, float centreY, float angle, float zPos, float zHeight) {
    b2Vec2 centre = b2Vec2(centreX, centreY);
    b2PolygonShape* shape = new b2PolygonShape();
    shape->SetAsBox(width, height, centre, angle);
	shape->m_zPos = zPos;
	shape->m_height = zHeight;
    return static_cast<void*>(shape);
}
extern "C" __declspec(dllexport)  void* GetCircleShapeDef(float radius, float centreX, float centreY, float zPos, float zHeight) {
    b2Vec2 centre = b2Vec2(centreX, centreY);
    b2CircleShape* shape = new b2CircleShape();
    shape->m_p = centre;
    shape->m_radius = radius;
	shape->m_zPos = zPos;
	shape->m_height = zHeight;
    return static_cast<void*>(shape);
}
extern "C" __declspec(dllexport)  void* GetChainShapeDef(float* vertArray, bool loop, float zPos, float zHeight) {
    b2ChainShape* shape = new b2ChainShape();
    int numberOfVertices = vertArray[0];
    b2Vec2* vertices = new b2Vec2[numberOfVertices];
    
    int j = 0;
    for (int i = 1; i < (vertArray[0] * 2); i += 2)
    {
        vertices[j] = b2Vec2(vertArray[i], vertArray[i + 1]);
        ++j;
    }
    
    if (loop) {
        shape->CreateLoop(vertices, numberOfVertices);
    }
    else {
        shape->CreateChain(vertices, numberOfVertices);
    }
	shape->m_zPos = zPos;
	shape->m_height = zHeight;
    return static_cast<void*>(shape);
}
extern "C" __declspec(dllexport)  void* GetPolygonShapeDef(float* vertArray, float zPos, float zHeight) {
    b2PolygonShape* shape = new b2PolygonShape();
    int numberOfVertices = vertArray[0];
    b2Vec2* vertices = new b2Vec2[numberOfVertices];
    
    int j = 0;
    for (int i = 1; i < (vertArray[0] * 2); i += 2)
    {
        vertices[j] = b2Vec2(vertArray[i], vertArray[i + 1]);
        ++j;
    }
    
    shape->Set(vertices, numberOfVertices);
	shape->m_zPos = zPos;
	shape->m_height = zHeight;
    return static_cast<void*>(shape);
}
extern "C" __declspec(dllexport)  void* GetEdgeShapeDef(float x1, float y1, float x2, float y2, float zPos, float zHeight) {
    b2Vec2 vec1 = b2Vec2(x1, y1);
    b2Vec2 vec2 = b2Vec2(x2, y2);
    b2EdgeShape* shape = new b2EdgeShape();
    shape->Set(vec1, vec2);
	shape->m_zPos = zPos;
	shape->m_height = zHeight;
    return static_cast<void*>(shape);
}
extern "C" __declspec(dllexport)  void* GetEllipseShapeDef(float outerRadius, float divisions, float zPos, float zHeight) {
    /*b2ChainShape* chainShape;
     std::vector<b2Vec2> vertices;
     const float32 SPIKE_DEGREE = 2 * 3.14159265358979323846f / 180;
     for (int idx = 0; idx < divisions; idx++) {
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
    int numberOfVertices = divisions;
    b2Vec2* vertices = new b2Vec2[numberOfVertices];
    
    for (int idx = 0; idx < divisions; idx++) {
        float32 angle = ((3.14159265358979323846f * 2) / divisions)*idx;
        float32 xPos, yPos;
        
        xPos = outerRadius*cos(angle);
        yPos = outerRadius*sin(angle);
        vertices[idx] = b2Vec2(xPos, yPos);
    }
    
    shape->Set(vertices, numberOfVertices);
	shape->m_zPos = zPos;
	shape->m_height = zHeight;
    return static_cast<void*>(shape);
    
}
extern "C" __declspec(dllexport)  float* GetPolyShapeCentroid(void* shapePointer) {
    float * positionArray = new float[2];
    b2PolygonShape* m_shape = static_cast<b2PolygonShape*>(shapePointer);
    positionArray[0] = m_shape->m_centroid.x;
    positionArray[1] = m_shape->m_centroid.y;
    return positionArray;
}

#pragma endregion

#pragma region Body

extern "C" __declspec(dllexport)  int CreateBody(void* worldPointer, int type, float xPosition, float yPosition, float angle, float linearDamping, float angularDamping, void* materialPointer, float heat, float health, int flags, float gravityScale, int userData) {
	b2World* world = static_cast<b2World*>(worldPointer);
	b2BodyMaterial* material = static_cast<b2BodyMaterial*>(materialPointer);
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
	bd.material = material;
	bd.heat = heat;
	bd.health = health;
	bd.flags = bf;
    
	bd.gravityScale = gravityScale;
    bd.userData = (void*)userData;

    int bodyIdx = world->CreateBody(&bd);
    return bodyIdx;
}
extern "C" __declspec(dllexport)  float* GetBodyInfo(void* bodyPointer) {
    
    if (positionArray != NULL)
    {
        delete positionArray;
    }
    
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    positionArray = new float[6];
    positionArray[0] = m_body->GetPosition().x;
    positionArray[1] = m_body->GetPosition().y;
    positionArray[2] = m_body->GetAngle();
	positionArray[3] = m_body->GetHealth();
	positionArray[4] = m_body->GetHeat();
	positionArray[5] = m_body->GetFlags();
    
    return positionArray;
}
extern "C" __declspec(dllexport)  float* GetAllBodyInfo(void* worldPtr, int* bodyIdxs, int numbodies)
{
	b2World* world = static_cast<b2World*>(worldPtr);
    if (positionArray != NULL)
    {
        delete positionArray;
    }
    
    positionArray = new float[numbodies*6];
    
	const std::vector<b2Body*>& bodies = world->GetBodyBuffer();
    for (int i = 0; i < numbodies; i++)
    {
		b2Body* m_body = bodies[bodyIdxs[i]];
        positionArray[i*6] = m_body->GetPosition().x;
        positionArray[(i*6) + 1] = m_body->GetPosition().y;
        positionArray[(i*6) + 2] = m_body->GetAngle();
		positionArray[(i * 6) + 3] = m_body->GetHealth();
		positionArray[(i * 6) + 4] = m_body->GetHeat();
		positionArray[(i * 6) + 5] = m_body->GetFlags();
    }
    
    return positionArray;
}
extern "C" __declspec(dllexport)  int* GetBodyContacts(void* bodyPointer)
{
    if (returnArray != NULL)
    {
        delete returnArray;
    }
    
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    //b2ContactEdge * contacts =  m_body->GetContactList() ;
    
    int numconts = 0;
    std::vector<int> conts;
    
    for ( b2ContactEdge * ce =  m_body->GetContactList(); ce; ce = ce->next)
    {
        numconts++;
        conts.push_back((int64)(ce->other->GetUserData()));
    }
    
    returnArray = new int[numconts+1];
    returnArray[0] = numconts;
    for (int i = 0; i < numconts; i++)
    {
        returnArray[i+1] = conts[i];
    }
    
    return returnArray;
    
}
extern "C" __declspec(dllexport)  void ApplyForceToCentreOfBody(void* bodyPointer, float impulseX, float impulseY) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    b2Vec2 impulse = b2Vec2(impulseX, impulseY);
    m_body->ApplyForceToCenter(impulse, true);
}
extern "C" __declspec(dllexport)  void SetBodyAwake(void* bodyPointer, bool isAwake) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    m_body->SetAwake(isAwake);
}
extern "C" __declspec(dllexport)  void SetBodyType(void* bodyPointer, int type) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    
    b2BodyType bodyType;
    
    if (type == 1)
        bodyType = b2_dynamicBody;
    else if (type == 2)
        bodyType = b2_kinematicBody;
    else
        bodyType = b2_staticBody;
    
    m_body->SetType(bodyType);
}
extern "C" __declspec(dllexport)  bool GetBodyAwake(void* worldPtr, int bodyIdx) {
	b2Body* bodyPtr = (static_cast<b2World*>(worldPtr))->GetBodyPtr(bodyIdx);
    return bodyPtr->IsAwake();
}
extern "C" __declspec(dllexport)  void SetBodyActive(void* bodyPointer, bool isActive) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    m_body->SetActive(isActive);
}
extern "C" __declspec(dllexport)  bool GetBodyActive(void* worldPtr, int bodyIdx) {
    b2Body* bodyPtr = (static_cast<b2World*>(worldPtr))->GetBodyPtr(bodyIdx);
    return bodyPtr->IsActive();
}
extern "C" __declspec(dllexport)  void** GetBodyFixtures(void* bodyPointer) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    std::vector<void*> fixturesVec;
    fixturesVec.clear();
	std::vector<int32>& fixtureIdxs = m_body->GetFixtureIdxBuffer();
	std::vector<b2Fixture*>& fixtures = m_body->GetFixtureBuffer();
    for each (int32 fIdx in fixtureIdxs)
	{
		if (fIdx != b2_invalidIndex)
		{
			fixturesVec.push_back(static_cast<void*>(fixtures[fIdx]));
		}
    }
    void** fixturesArray = &fixturesVec[0];
    return fixturesArray;
}
extern "C" __declspec(dllexport)  int GetBodyFixturesCount(void* bodyPointer) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    int i = 0;
	std::vector<int32>& fixtureIdxs = m_body->GetFixtureIdxBuffer();
	std::vector<b2Fixture*>& fixtures = m_body->GetFixtureBuffer();
	for each (int32 fIdx in fixtureIdxs)
	{
		if (fIdx != b2_invalidIndex)
		{
			++i;
		}
	}
    return i;
}
extern "C" __declspec(dllexport)  int GetBodyUserData(void* bodyPointer) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    return (int64)m_body->GetUserData();
}
extern "C" __declspec(dllexport)  void SetBodyPosition(void* bodyPointer, float x, float y) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    b2Vec2 pos = b2Vec2(x, y);
    m_body->SetTransform(pos, m_body->GetAngle());
}
extern "C" __declspec(dllexport)  void SetBodyRotation(void* bodyPointer, float rotation) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    m_body->SetTransform(m_body->GetPosition(), rotation);
}
extern "C" __declspec(dllexport)  void SetBodyLinearVelocity(void* worldPtr, int idx, float x, float y) {
    b2World* world = static_cast<b2World*>(worldPtr);
	b2Body* body = world->GetBodyBuffer()[idx];
    b2Vec2 vel = b2Vec2(x, y);
	body->SetLinearVelocity(vel);
}
extern "C" __declspec(dllexport)  float* GetBodyLinearVelocity(void* worldPtr, int idx) {
	b2World* world = static_cast<b2World*>(worldPtr);
	b2Body* body = world->GetBodyBuffer()[idx];
    if (positionArray != NULL)
        delete positionArray;
    
    positionArray = new float[2];
    positionArray[0] = body->GetLinearVelocity().x;
    positionArray[1] = body->GetLinearVelocity().y;
    return positionArray;
}
extern "C" __declspec(dllexport)  void SetBodyLinearDamping(void* bodyPointer, float lD) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    m_body->SetLinearDamping(lD);
}
extern "C" __declspec(dllexport)  float GetBodyLinearDamping(void* bodyPointer) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    return m_body->GetLinearDamping();
}
extern "C" __declspec(dllexport)  void SetBodyAngularDamping(void* bodyPointer, float aD) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    m_body->SetAngularDamping(aD);
}
extern "C" __declspec(dllexport)  float GetBodyAngularDamping(void* bodyPointer) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    return m_body->GetAngularDamping();
}
extern "C" __declspec(dllexport)  void SetBodyAngularVelocity(void* bodyPointer, float w) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    m_body->SetAngularVelocity(w);
}
extern "C" __declspec(dllexport)  float GetBodyAngularVelocity(void* bodyPointer) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    return m_body->GetAngularVelocity();
}
extern "C" __declspec(dllexport)  void SetBodyGravityScale(void* bodyPointer, float scale) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    m_body->SetGravityScale(scale);
}
extern "C" __declspec(dllexport)  float GetBodyGravityScale(void* bodyPointer) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    return m_body->GetGravityScale();
}
extern "C" __declspec(dllexport)  void SetBodyIsBullet(void* bodyPointer, bool isBullet) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    m_body->SetBullet(isBullet);
}
extern "C" __declspec(dllexport)  bool GetBodyIsBullet(void* bodyPointer) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    return m_body->IsBullet();
}
extern "C" __declspec(dllexport)  void SetBodyFixedRotation(void* bodyPointer, bool flag) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    m_body->SetFixedRotation(flag);
}
extern "C" __declspec(dllexport)  bool GetBodyFixedRotation(void* bodyPointer) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    return m_body->IsFixedRotation();
}

extern "C" __declspec(dllexport)  void ApplyAngularImpulseToBody(void* bodyPointer, float impulse, bool wake) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    m_body->ApplyAngularImpulse(impulse, wake);
}
extern "C" __declspec(dllexport)  void ApplyForceToBody(void* bodyPointer, float forceX, float forceY, float posX, float posY, bool wake) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    b2Vec2 force = b2Vec2(forceX, forceY);
    b2Vec2 position = b2Vec2(posX, posY);
    m_body->ApplyForce(force, position, wake);
}
extern "C" __declspec(dllexport)  void ApplyLinearImpulseToBody(void* bodyPointer, float forceX, float forceY, float posX, float posY, bool wake) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    b2Vec2 force = b2Vec2(forceX, forceY);
    b2Vec2 position = b2Vec2(posX, posY);
    m_body->ApplyLinearImpulse(force, position, wake);
}
extern "C" __declspec(dllexport)  void ApplyTorqueToBody(void* bodyPointer, float torque, bool wake) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    m_body->ApplyTorque(torque, wake);
}

extern "C" __declspec(dllexport)  float GetBodyMass(void* bodyPointer) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    return m_body->GetMass();
}
extern "C" __declspec(dllexport)  float GetBodyInertia(void* bodyPointer) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    return m_body->GetInertia();
}
extern "C" __declspec(dllexport)  void SetBodyTransform(void* bodyPointer, float x, float y, float angle) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    b2Vec2 pos = b2Vec2(x, y);
    m_body->SetTransform(pos, angle);
}

extern "C" __declspec(dllexport)  void DeleteBody(void* worldPtr, int idx) {
    b2World* world = static_cast<b2World*>(worldPtr);
    world->DestroyBody(idx);
}

#pragma endregion

#pragma region Fixture

extern "C" __declspec(dllexport)  int AddFixture(void* worldPtr, int bodyIdx, int shapeType, void* shapePointer, bool isSensor, int userData) {
	b2World* world = static_cast<b2World*>(worldPtr);
	b2Body* body = world->GetBodyBuffer()[bodyIdx];
    b2FixtureDef fd;
	body->GetMaterial()->m_density;
	body->GetMaterial()->m_friction;
	fd.isSensor = isSensor;
	body->GetMaterial()->m_bounciness;
    fd.userData = (void*)userData;
    if (shapeType == 0) {
        b2PolygonShape* aShape = static_cast<b2PolygonShape*>(shapePointer);
        b2PolygonShape shape = *aShape;
        fd.shape = &shape;
        int idx = body->CreateFixture(&fd);
        return idx;
    }
    else if (shapeType == 1) {
        b2CircleShape* aShape = static_cast<b2CircleShape*>(shapePointer);
        b2CircleShape shape = *aShape;
        fd.shape = &shape;
		int idx = body->CreateFixture(&fd);
        return idx;
    }
    else if (shapeType == 2) {
        b2EdgeShape* aShape = static_cast<b2EdgeShape*>(shapePointer);
        b2EdgeShape shape = *aShape;
        fd.shape = &shape;
		int idx = body->CreateFixture(&fd);
        return idx;
    }
    else {
        b2ChainShape* aShape = static_cast<b2ChainShape*>(shapePointer);
        b2ChainShape shape = *aShape;
        fd.shape = &shape;
		int idx = body->CreateFixture(&fd);
        return idx;
    }
    
}

extern "C" __declspec(dllexport)  float* GetFixtureInfo(void* fixturePointer) {
    
    if (positionArray != NULL)
    {
        delete positionArray;
    }
    
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    positionArray = new float[3];
    positionArray[0] = m_fixture->GetBody()->GetPosition().x;
    positionArray[1] = m_fixture->GetBody()->GetPosition().y;
    positionArray[2] = m_fixture->GetBody()->GetAngle();
    
    return positionArray;
}

extern "C" __declspec(dllexport)  int GetFixtureUserData(void* fixturePointer) {
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    return (int64)m_fixture->GetUserData();
}

extern "C" __declspec(dllexport)  bool TestPoint(void* fixturePointer, float x, float y) {
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    b2Vec2 pos = b2Vec2(x, y);
    return m_fixture->TestPoint(pos);
}

extern "C" __declspec(dllexport)  void SetFixtureFilterData(void* WorldPtr, int idx, int32 groupIndex, uint16 categoryBits, uint16 maskBits) {
	b2World* world = static_cast<b2World*>(WorldPtr);
    b2Filter filter = b2Filter();
    filter.groupIndex = groupIndex;
    filter.maskBits = maskBits;
    filter.categoryBits = categoryBits;
	b2Fixture* fixture = static_cast<b2Fixture*>(world->GetFixtureBuffer()[idx]);
	fixture->SetFilterData(filter);
}
extern "C" __declspec(dllexport)  uint16 GetFixtureGroupIndex(void* fixturePointer) {
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    return m_fixture->GetFilterData().groupIndex;
}
extern "C" __declspec(dllexport)  uint16 GetFixtureMaskBits(void* fixturePointer) {
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    return m_fixture->GetFilterData().maskBits;
}
extern "C" __declspec(dllexport)  uint16 GetFixtureCategoryBits(void* fixturePointer) {
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    return m_fixture->GetFilterData().categoryBits;
}

extern "C" __declspec(dllexport)  bool GetFixtureIsSensor(void* fixturePointer) {
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    return m_fixture->IsSensor();
}
extern "C" __declspec(dllexport)  void SetFixtureIsSensor(void* fixturePointer, bool flag) {
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    m_fixture->SetSensor(flag);
}

extern "C" __declspec(dllexport)  float GetFixtureDensity(void* fixturePointer) {
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    return m_fixture->GetDensity();
}
extern "C" __declspec(dllexport)  void SetFixtureDensity(void* fixturePointer, float density) {
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    m_fixture->SetDensity(density);
}

extern "C" __declspec(dllexport)  float GetFixtureFriction(void* fixturePointer) {
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    return m_fixture->GetFriction();
}
extern "C" __declspec(dllexport)  void SetFixtureFriction(void* fixturePointer, float friction) {
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    m_fixture->SetFriction(friction);
}

extern "C" __declspec(dllexport)  float GetFixtureRestitution(void* fixturePointer) {
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    return m_fixture->GetRestitution();
}
extern "C" __declspec(dllexport)  void SetFixtureRestitution(void* fixturePointer, float restitution) {
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    m_fixture->SetRestitution(restitution);
}

extern "C" __declspec(dllexport)  void DeleteFixture(void* bodyPtr, int idx) {
    b2Body* body = static_cast<b2Body*>(bodyPtr);
    body->DestroyFixture(idx);
}
#pragma endregion

#pragma region Joints

#pragma region DistanceJoints
extern "C" __declspec(dllexport)  void* CreateDistanceJoint(void* worldPointer, void* bodyA, void* bodyB, float anchorAX, float anchorAY, float anchorBX, float anchorBY, float length, bool collideConnected) {
    b2World* world = static_cast<b2World*>(worldPointer);
    b2Body* m_bodyA = static_cast<b2Body*>(bodyA);
    b2Body* m_bodyB = static_cast<b2Body*>(bodyB);
    b2DistanceJoint* dj;
    b2DistanceJointDef jd;
    jd.bodyA = m_bodyA;
    jd.bodyB = m_bodyB;
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    jd.collideConnected = collideConnected;
    jd.length = length;
    dj = (b2DistanceJoint*)world->CreateJoint(&jd);
    return static_cast<void*>(dj);
}
extern "C" __declspec(dllexport)  void SetDistanceJointFrequency(void* joint, float frequency) {
    b2DistanceJoint* m_joint = static_cast<b2DistanceJoint*>(joint);
    m_joint->SetFrequency(frequency);
}
extern "C" __declspec(dllexport)  float GetDistanceJointFrequency(void* joint) {
    b2DistanceJoint* m_joint = static_cast<b2DistanceJoint*>(joint);
    return m_joint->GetFrequency();
}
extern "C" __declspec(dllexport)  void SetDistanceJointDampingRatio(void* joint, float ratio) {
    b2DistanceJoint* m_joint = static_cast<b2DistanceJoint*>(joint);
    m_joint->SetDampingRatio(ratio);
}
extern "C" __declspec(dllexport)  float GetDistanceJointDampingRatio(void* joint) {
    b2DistanceJoint* m_joint = static_cast<b2DistanceJoint*>(joint);
    return m_joint->GetDampingRatio();
}
extern "C" __declspec(dllexport)  void SetDistanceJointLength(void* joint, float length) {
    b2DistanceJoint* m_joint = static_cast<b2DistanceJoint*>(joint);
    m_joint->SetLength(length);
}
extern "C" __declspec(dllexport)  float GetDistanceJointLength(void* joint) {
    b2DistanceJoint* m_joint = static_cast<b2DistanceJoint*>(joint);
    return m_joint->GetLength();
}
#pragma endregion

#pragma region RevoluteJoints
extern "C" __declspec(dllexport)  void* CreateRevoluteJoint(void* worldPointer, void* bodyA, void* bodyB, float anchorAX, float anchorAY, float anchorBX, float anchorBY, bool collideConnected) {
    b2World* world = static_cast<b2World*>(worldPointer);
    b2Body* m_bodyA = static_cast<b2Body*>(bodyA);
    b2Body* m_bodyB = static_cast<b2Body*>(bodyB);
    b2RevoluteJoint* rj;
    b2RevoluteJointDef jd;
    jd.bodyA = m_bodyA;
    jd.bodyB = m_bodyB;
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    jd.collideConnected = collideConnected;
    rj = (b2RevoluteJoint*)world->CreateJoint(&jd);
    return static_cast<void*>(rj);
}
extern "C" __declspec(dllexport)  void SetRevoluteJointLimits(void* joint, float lower, float upper) {
    b2RevoluteJoint* m_joint = static_cast<b2RevoluteJoint*>(joint);
    m_joint->SetLimits(lower, upper);
}
extern "C" __declspec(dllexport)  void SetRevoluteJointMotorSpeed(void* joint, float speed) {
    b2RevoluteJoint* m_joint = static_cast<b2RevoluteJoint*>(joint);
    m_joint->SetMotorSpeed(speed);
}
extern "C" __declspec(dllexport)  void SetRevoluteJointMaxMotorTorque(void* joint, float torque) {
    b2RevoluteJoint* m_joint = static_cast<b2RevoluteJoint*>(joint);
    m_joint->SetMaxMotorTorque(torque);
}
extern "C" __declspec(dllexport)  void EnableRevoluteJointMotor(void* joint, bool motor) {
    b2RevoluteJoint* m_joint = static_cast<b2RevoluteJoint*>(joint);
    m_joint->EnableMotor(motor);
}
extern "C" __declspec(dllexport)  void EnableRevoluteJointLimits(void* joint, bool limit) {
    b2RevoluteJoint* m_joint = static_cast<b2RevoluteJoint*>(joint);
    m_joint->EnableLimit(limit);
}
extern "C" __declspec(dllexport)  float GetRevoluteJointUpperLimit(void* joint) {
    b2RevoluteJoint* m_joint = static_cast<b2RevoluteJoint*>(joint);
    return m_joint->GetUpperLimit();
}
extern "C" __declspec(dllexport)  float GetRevoluteJointLowerLimit(void* joint) {
    b2RevoluteJoint* m_joint = static_cast<b2RevoluteJoint*>(joint);
    return m_joint->GetLowerLimit();
}
extern "C" __declspec(dllexport)  bool IsRevoluteJointMotorEnabled(void* joint) {
    b2RevoluteJoint* m_joint = static_cast<b2RevoluteJoint*>(joint);
    return m_joint->IsMotorEnabled();
}
extern "C" __declspec(dllexport)  float GetRevoluteJointMotorSpeed(void* joint) {
    b2RevoluteJoint* m_joint = static_cast<b2RevoluteJoint*>(joint);
    return m_joint->GetMotorSpeed();
}
extern "C" __declspec(dllexport)  float GetRevoluteJointMotorTorque(void* joint, float invDt) {
    b2RevoluteJoint* m_joint = static_cast<b2RevoluteJoint*>(joint);
    return m_joint->GetMotorTorque(invDt);
}
extern "C" __declspec(dllexport)  float GetRevoluteJointMaxMotorTorque(void* joint) {
    b2RevoluteJoint* m_joint = static_cast<b2RevoluteJoint*>(joint);
    return m_joint->GetMaxMotorTorque();
}
#pragma endregion

#pragma region PrismaticJoints
extern "C" __declspec(dllexport)  void* CreatePrismaticJoint(void* worldPointer, void* bodyA, void* bodyB, float anchorAX, float anchorAY, float anchorBX, float anchorBY, float axisX, float axisY, bool collideConnect) {
    b2World* world = static_cast<b2World*>(worldPointer);
    b2Body* m_bodyA = static_cast<b2Body*>(bodyA);
    b2Body* m_bodyB = static_cast<b2Body*>(bodyB);
    b2PrismaticJoint* pj;
    b2PrismaticJointDef jd;
    jd.bodyA = m_bodyA;
    jd.bodyB = m_bodyB;
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    jd.localAxisA.Set(axisX, axisY);
    jd.collideConnected = collideConnect;
    pj = (b2PrismaticJoint*)world->CreateJoint(&jd);
    return static_cast<void*>(pj);
}
extern "C" __declspec(dllexport)  void SetPrismaticJointLimits(void* joint, float lower, float upper) {
    b2PrismaticJoint* m_joint = static_cast<b2PrismaticJoint*>(joint);
    m_joint->SetLimits(lower, upper);
}
extern "C" __declspec(dllexport)  void SetPrismaticJointMotorSpeed(void* joint, float speed) {
    b2PrismaticJoint* m_joint = static_cast<b2PrismaticJoint*>(joint);
    m_joint->SetMotorSpeed(speed);
}
extern "C" __declspec(dllexport)  void SetPrismaticJointMaxMotorForce(void* joint, float force) {
    b2PrismaticJoint* m_joint = static_cast<b2PrismaticJoint*>(joint);
    m_joint->SetMaxMotorForce(force);
}
extern "C" __declspec(dllexport)  void EnablePrismaticJointMotor(void* joint, bool motor) {
    b2PrismaticJoint* m_joint = static_cast<b2PrismaticJoint*>(joint);
    m_joint->EnableMotor(motor);
}
extern "C" __declspec(dllexport)  void EnablePrismaticJointLimits(void* joint, bool limit) {
    b2PrismaticJoint* m_joint = static_cast<b2PrismaticJoint*>(joint);
    m_joint->EnableLimit(limit);
}
extern "C" __declspec(dllexport)  float GetPrismaticJointUpperLimit(void* joint) {
    b2PrismaticJoint* m_joint = static_cast<b2PrismaticJoint*>(joint);
    return m_joint->GetUpperLimit();
}
extern "C" __declspec(dllexport)  float GetPrismaticJointLowerLimit(void* joint) {
    b2PrismaticJoint* m_joint = static_cast<b2PrismaticJoint*>(joint);
    return m_joint->GetLowerLimit();
}
extern "C" __declspec(dllexport)  bool IsPrismaticJointMotorEnabled(void* joint) {
    b2PrismaticJoint* m_joint = static_cast<b2PrismaticJoint*>(joint);
    return m_joint->IsMotorEnabled();
}
extern "C" __declspec(dllexport)  float GetPrismaticJointMotorSpeed(void* joint) {
    b2PrismaticJoint* m_joint = static_cast<b2PrismaticJoint*>(joint);
    return m_joint->GetMotorSpeed();
}
extern "C" __declspec(dllexport)  float GetPrismaticJointMotorTorque(void* joint, float invDt) {
    b2PrismaticJoint* m_joint = static_cast<b2PrismaticJoint*>(joint);
    return m_joint->GetMotorForce(invDt);
}
extern "C" __declspec(dllexport)  float GetPrismaticJointMaxMotorForce(void* joint) {
    b2PrismaticJoint* m_joint = static_cast<b2PrismaticJoint*>(joint);
    return m_joint->GetMaxMotorForce();
}
extern "C" __declspec(dllexport)  float GetPrismaticJointMotorForce(void* joint,float its) {
    b2PrismaticJoint* m_joint = static_cast<b2PrismaticJoint*>(joint);
    return m_joint->GetMotorForce(its);
}
extern "C" __declspec(dllexport)  float GetPrismaticJointSpeed(void* joint) {
    b2PrismaticJoint* m_joint = static_cast<b2PrismaticJoint*>(joint);
    return m_joint->GetJointSpeed();
}
#pragma endregion

#pragma region PulleyJoints
extern "C" __declspec(dllexport)  void* CreatePulleyJoint(void* worldPointer, void* bodyA, void* bodyB, float groundAnchorAX, float groundAanchorAY, float groundAnchorBX, float groundAanchorBY, float anchorAX, float anchorAY, float anchorBX, float anchorBY, float ratio, float lengthA, float lengthB, bool collideConnect) {
    b2World* world = static_cast<b2World*>(worldPointer);
    b2Body* m_bodyA = static_cast<b2Body*>(bodyA);
    b2Body* m_bodyB = static_cast<b2Body*>(bodyB);
    b2PulleyJoint* pj;
    b2PulleyJointDef jd;
    jd.bodyA = m_bodyA;
    jd.bodyB = m_bodyB;
    jd.groundAnchorA.Set(groundAnchorAX, groundAanchorAY);
    jd.groundAnchorB.Set(groundAnchorBX, groundAanchorBY);
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    jd.ratio = ratio;
    jd.collideConnected = collideConnect;
    jd.lengthA = lengthA;
    jd.lengthB = lengthB;
    pj = (b2PulleyJoint*)world->CreateJoint(&jd);
    return static_cast<void*>(pj);
}
extern "C" __declspec(dllexport)  float GetPulleyJointLengthA(void* joint) {
    b2PulleyJoint* m_joint = static_cast<b2PulleyJoint*>(joint);
    return m_joint->GetLengthA();
}
extern "C" __declspec(dllexport)  float GetPulleyJointLengthB(void* joint) {
    b2PulleyJoint* m_joint = static_cast<b2PulleyJoint*>(joint);
    return m_joint->GetLengthB();
}
#pragma endregion

#pragma region GearJoints
extern "C" __declspec(dllexport)  void* CreateGearJoint(void* worldPointer, void* bodyA, void* bodyB, void* jointA, bool isARevolute, void* jointB, bool isBRevolute, float ratio, bool collideConnect) {
    b2World* world = static_cast<b2World*>(worldPointer);
    b2Body* m_bodyA = static_cast<b2Body*>(bodyA);
    b2Body* m_bodyB = static_cast<b2Body*>(bodyB);
    if (isARevolute && isBRevolute) {
        b2RevoluteJoint* m_jointA = static_cast<b2RevoluteJoint*>(jointA);
        b2RevoluteJoint* m_jointB = static_cast<b2RevoluteJoint*>(jointB);
        b2GearJoint* gj;
        b2GearJointDef jd;
        jd.bodyA = m_bodyA;
        jd.bodyB = m_bodyB;
        jd.joint1 = m_jointA;
        jd.joint2 = m_jointB;
        jd.ratio = ratio;
        jd.collideConnected = collideConnect;
        gj = (b2GearJoint*)world->CreateJoint(&jd);
        return static_cast<void*>(gj);
    }
    else if (!isARevolute && !isBRevolute) {
        b2PrismaticJoint* m_jointA = static_cast<b2PrismaticJoint*>(jointA);
        b2PrismaticJoint* m_jointB = static_cast<b2PrismaticJoint*>(jointB);
        b2GearJoint* gj;
        b2GearJointDef jd;
        jd.bodyA = m_bodyA;
        jd.bodyB = m_bodyB;
        jd.joint1 = m_jointA;
        jd.joint2 = m_jointB;
        jd.ratio = ratio;
        jd.collideConnected = collideConnect;
        gj = (b2GearJoint*)world->CreateJoint(&jd);
        return static_cast<void*>(gj);
    }
    else {
        if (isARevolute && !isBRevolute) {
            b2RevoluteJoint* m_jointA = static_cast<b2RevoluteJoint*>(jointA);
            b2PrismaticJoint* m_jointB = static_cast<b2PrismaticJoint*>(jointB);
            b2GearJoint* gj;
            b2GearJointDef jd;
            jd.bodyA = m_bodyA;
            jd.bodyB = m_bodyB;
            jd.joint1 = m_jointA;
            jd.joint2 = m_jointB;
            jd.ratio = ratio;
            jd.collideConnected = collideConnect;
            gj = (b2GearJoint*)world->CreateJoint(&jd);
            return static_cast<void*>(gj);
        }
        else {
            b2PrismaticJoint* m_jointA = static_cast<b2PrismaticJoint*>(jointA);
            b2RevoluteJoint* m_jointB = static_cast<b2RevoluteJoint*>(jointB);
            b2GearJoint* gj;
            b2GearJointDef jd;
            jd.bodyA = m_bodyA;
            jd.bodyB = m_bodyB;
            jd.joint1 = m_jointA;
            jd.joint2 = m_jointB;
            jd.ratio = ratio;
            jd.collideConnected = collideConnect;
            gj = (b2GearJoint*)world->CreateJoint(&jd);
            return static_cast<void*>(gj);
        }
    }
}
extern "C" __declspec(dllexport)  void SetGearJointRatio(void* joint, float ratio) {
    b2GearJoint* m_joint = static_cast<b2GearJoint*>(joint);
    m_joint->SetRatio(ratio);
}
extern "C" __declspec(dllexport)  float GetGearJointRatio(void* joint) {
    b2GearJoint* m_joint = static_cast<b2GearJoint*>(joint);
    return m_joint->GetRatio();
}
#pragma endregion

#pragma region WheelJoints
extern "C" __declspec(dllexport)  void* CreateWheelJoint(void* worldPointer, void* bodyA, void* bodyB, float anchorAX, float anchorAY, float anchorBX, float anchorBY, float axisA, float axisB, bool collideConnect) {
    b2World* world = static_cast<b2World*>(worldPointer);
    b2Body* m_bodyA = static_cast<b2Body*>(bodyA);
    b2Body* m_bodyB = static_cast<b2Body*>(bodyB);
    b2WheelJoint* wj;
    b2WheelJointDef jd;
    jd.bodyA = m_bodyA;
    jd.bodyB = m_bodyB;
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    jd.localAxisA.Set(axisA, axisB);
    jd.collideConnected = collideConnect;
    wj = (b2WheelJoint*)world->CreateJoint(&jd);
    return static_cast<void*>(wj);
}
extern "C" __declspec(dllexport)  void SetWheelJointSpringDampingRatio(void* joint, float ratio) {
    b2WheelJoint* m_joint = static_cast<b2WheelJoint*>(joint);
    m_joint->SetSpringDampingRatio(ratio);
}
extern "C" __declspec(dllexport)  float GetWheelJointSpringDampingRatio(void* joint) {
    b2WheelJoint* m_joint = static_cast<b2WheelJoint*>(joint);
    return m_joint->GetSpringDampingRatio();
}
extern "C" __declspec(dllexport)  void SetWheelJointSpringFrequency(void* joint, float frequency) {
    b2WheelJoint* m_joint = static_cast<b2WheelJoint*>(joint);
    m_joint->SetSpringFrequencyHz(frequency);
}
extern "C" __declspec(dllexport)  float GetWheelJointSpringFrequency(void* joint) {
    b2WheelJoint* m_joint = static_cast<b2WheelJoint*>(joint);
    return m_joint->GetSpringFrequencyHz();
}
extern "C" __declspec(dllexport)  void SetWheelJointMotorSpeed(void* joint, float speed) {
    b2WheelJoint* m_joint = static_cast<b2WheelJoint*>(joint);
    m_joint->SetMotorSpeed(speed);
}
extern "C" __declspec(dllexport)  void SetWheelJointMaxMotorTorque(void* joint, float torque) {
    b2WheelJoint* m_joint = static_cast<b2WheelJoint*>(joint);
    m_joint->SetMaxMotorTorque(torque);
}
extern "C" __declspec(dllexport)  void EnableWheelJointMotor(void* joint, bool motor) {
    b2WheelJoint* m_joint = static_cast<b2WheelJoint*>(joint);
    m_joint->EnableMotor(motor);
}
extern "C" __declspec(dllexport)  bool IsWheelJointMotorEnabled(void* joint) {
    b2WheelJoint* m_joint = static_cast<b2WheelJoint*>(joint);
    return m_joint->IsMotorEnabled();
}
extern "C" __declspec(dllexport)  float GetWheelJointMotorSpeed(void* joint) {
    b2WheelJoint* m_joint = static_cast<b2WheelJoint*>(joint);
    return m_joint->GetMotorSpeed();
}
extern "C" __declspec(dllexport)  float GetWheelJointMotorTorque(void* joint, float invDt) {
    b2WheelJoint* m_joint = static_cast<b2WheelJoint*>(joint);
    return m_joint->GetMotorTorque(invDt);
}
extern "C" __declspec(dllexport)  float GetWheelJointMaxMotorTorque(void* joint) {
    b2WheelJoint* m_joint = static_cast<b2WheelJoint*>(joint);
    return m_joint->GetMaxMotorTorque();
}
#pragma endregion

#pragma region WeldJoints
extern "C" __declspec(dllexport)  void* CreateWeldJoint(void* worldPointer, void* bodyA, void* bodyB, float anchorAX, float anchorAY, float anchorBX, float anchorBY) {
    b2World* world = static_cast<b2World*>(worldPointer);
    b2Body* m_bodyA = static_cast<b2Body*>(bodyA);
    b2Body* m_bodyB = static_cast<b2Body*>(bodyB);
    b2WeldJoint* wj;
    b2WeldJointDef jd;
    jd.bodyA = m_bodyA;
    jd.bodyB = m_bodyB;
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    wj = (b2WeldJoint*)world->CreateJoint(&jd);
    return static_cast<void*>(wj);
}
extern "C" __declspec(dllexport)  float GetWeldJointFrequency(void* joint) {
    b2WeldJoint* m_joint = static_cast<b2WeldJoint*>(joint);
    return m_joint->GetFrequency();
}
extern "C" __declspec(dllexport)  float GetWeldJointDampingRatio(void* joint) {
    b2WeldJoint* m_joint = static_cast<b2WeldJoint*>(joint);
    return m_joint->GetDampingRatio();
}
extern "C" __declspec(dllexport)  void SetWeldJointFrequency(void* joint, float frequency) {
    b2WeldJoint* m_joint = static_cast<b2WeldJoint*>(joint);
    m_joint->SetFrequency(frequency);
}
extern "C" __declspec(dllexport)  void SetWeldJointDampingRatio(void* joint, float ratio) {
    b2WeldJoint* m_joint = static_cast<b2WeldJoint*>(joint);
    m_joint->SetDampingRatio(ratio);
}
#pragma endregion

#pragma region FrictionJoints
extern "C" __declspec(dllexport)  void* CreateFrictionJoint(void* worldPointer, void* bodyA, void* bodyB, float anchorAX, float anchorAY, float anchorBX, float anchorBY, bool collideConnect) {
    b2World* world = static_cast<b2World*>(worldPointer);
    b2Body* m_bodyA = static_cast<b2Body*>(bodyA);
    b2Body* m_bodyB = static_cast<b2Body*>(bodyB);
    b2FrictionJoint* fj;
    b2FrictionJointDef jd;
    jd.bodyA = m_bodyA;
    jd.bodyB = m_bodyB;
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    jd.collideConnected = collideConnect;
    fj = (b2FrictionJoint*)world->CreateJoint(&jd);
    return static_cast<void*>(fj);
}
extern "C" __declspec(dllexport)  float GetFrictionJointMaxForce(void* joint) {
    b2FrictionJoint* m_joint = static_cast<b2FrictionJoint*>(joint);
    return m_joint->GetMaxForce();
}
extern "C" __declspec(dllexport)  float GetFrictionJointMaxTorque(void* joint) {
    b2FrictionJoint* m_joint = static_cast<b2FrictionJoint*>(joint);
    return m_joint->GetMaxTorque();
}
extern "C" __declspec(dllexport)  void SetFrictionJointMaxForce(void* joint, float force) {
    b2FrictionJoint* m_joint = static_cast<b2FrictionJoint*>(joint);
    m_joint->SetMaxForce(force);
}
extern "C" __declspec(dllexport)  void SetFrictionJointMaxTorque(void* joint, float torque) {
    b2FrictionJoint* m_joint = static_cast<b2FrictionJoint*>(joint);
    m_joint->SetMaxTorque(torque);
}
#pragma endregion

#pragma region RopeJoints
extern "C" __declspec(dllexport)  void* CreateRopeJoint(void* worldPointer, void* bodyA, void* bodyB, float anchorAX, float anchorAY, float anchorBX, float anchorBY, float maxLength, bool collideConnect) {
    b2World* world = static_cast<b2World*>(worldPointer);
    b2Body* m_bodyA = static_cast<b2Body*>(bodyA);
    b2Body* m_bodyB = static_cast<b2Body*>(bodyB);
    b2RopeJoint* rj;
    b2RopeJointDef jd;
    jd.bodyA = m_bodyA;
    jd.bodyB = m_bodyB;
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    jd.maxLength = maxLength;
    jd.collideConnected = collideConnect;
    rj = (b2RopeJoint*)world->CreateJoint(&jd);
    return static_cast<void*>(rj);
}
extern "C" __declspec(dllexport)  float GetRopeJointMaxLength(void* joint) {
    b2RopeJoint* m_joint = static_cast<b2RopeJoint*>(joint);
    return m_joint->GetMaxLength();
}
extern "C" __declspec(dllexport)  void SetRopeJointMaxLength(void* joint, float length) {
    b2RopeJoint* m_joint = static_cast<b2RopeJoint*>(joint);
    m_joint->SetMaxLength(length);
}
#pragma endregion

#pragma region MouseJoints
extern "C" __declspec(dllexport)  void* CreateMouseJoint(void* worldPointer, void* bodyA, void* bodyB, float targetX, float targetY, bool collideConnect) {
    b2World* world = static_cast<b2World*>(worldPointer);
    b2Body* m_bodyA = static_cast<b2Body*>(bodyA);
    b2Body* m_bodyB = static_cast<b2Body*>(bodyB);
    b2Vec2 target = b2Vec2(targetX, targetY);
    b2MouseJoint* rj;
    b2MouseJointDef jd;
    jd.bodyA = m_bodyA;
    jd.bodyB = m_bodyB;
    jd.target = target;
    jd.collideConnected = collideConnect;
    rj = (b2MouseJoint*)world->CreateJoint(&jd);
    return static_cast<void*>(rj);
}
extern "C" __declspec(dllexport)  float GetMouseJointFrequency(void* joint) {
    b2MouseJoint* m_joint = static_cast<b2MouseJoint*>(joint);
    return m_joint->GetFrequency();
}
extern "C" __declspec(dllexport)  float GetMouseJointMaxForce(void* joint) {
    b2MouseJoint* m_joint = static_cast<b2MouseJoint*>(joint);
    return m_joint->GetMaxForce();
}
extern "C" __declspec(dllexport)  float GetMouseJointDampingRatio(void* joint) {
    b2MouseJoint* m_joint = static_cast<b2MouseJoint*>(joint);
    return m_joint->GetDampingRatio();
}
extern "C" __declspec(dllexport)  void SetMouseJointFrequency(void* joint, float frequency) {
    b2MouseJoint* m_joint = static_cast<b2MouseJoint*>(joint);
    m_joint->SetFrequency(frequency);
}
extern "C" __declspec(dllexport)  void SetMouseJointMaxForce(void* joint, float maxForce) {
    b2MouseJoint* m_joint = static_cast<b2MouseJoint*>(joint);
    m_joint->SetMaxForce(maxForce);
}
extern "C" __declspec(dllexport)  void SetMouseJointDampingRatio(void* joint, float dampingRatio) {
    b2MouseJoint* m_joint = static_cast<b2MouseJoint*>(joint);
    m_joint->SetDampingRatio(dampingRatio);
}
extern "C" __declspec(dllexport)  void SetMouseJointTarget(void* joint, float targetX, float targetY) {
    b2MouseJoint* m_joint = static_cast<b2MouseJoint*>(joint);
    b2Vec2 target = b2Vec2(targetX, targetY);
    m_joint->SetTarget(target);
}
#pragma endregion

#pragma region GenericFunctions
extern "C" __declspec(dllexport)  void DeleteJoint(void* worldPointer, void* jointPointer) {
    b2World* world = static_cast<b2World*>(worldPointer);
    b2Joint* joint = static_cast<b2Joint*>(jointPointer);
    world->DestroyJoint(joint);
}
extern "C" __declspec(dllexport)  bool GetJointCollideConnected(void* jointPointer) {
    b2Joint* joint = static_cast<b2Joint*>(jointPointer);
    return joint->GetCollideConnected();
}
extern "C" __declspec(dllexport)  void ShiftJointOrigin(void* joint, float x, float y) {
    b2Joint* m_joint = static_cast<b2Joint*>(joint);
    b2Vec2 origin = b2Vec2(x, y);
    m_joint->ShiftOrigin(origin);
}
#pragma endregion

#pragma endregion

#pragma region Raycasting
extern "C" __declspec(dllexport)  float* RaycastWorld(void* world, float x1, float y1, float x2, float y2, int mode, bool shouldQuery) {
    b2World* m_world = static_cast<b2World*>(world);
    newRC = new b2NewRaycastCallback(mode, shouldQuery);
    b2Vec2 pos1 = b2Vec2(x1, y1);
    b2Vec2 pos2 = b2Vec2(x2, y2);
    m_world->RayCast(newRC, pos1, pos2);
    return newRC->GetData();
}
#pragma endregion

#pragma region MemoryReleasing
extern "C" __declspec(dllexport)  int ReleaseFloatArray(float* floatArray)
{
    delete[] floatArray;
    return 0;
}
extern "C" __declspec(dllexport)  int ReleaseIntArray(int* intArray)
{
    delete[] intArray;
    return 0;
}
extern "C" __declspec(dllexport)  int ReleaseShape(b2Shape* shape)
{
    delete shape;
    return 0;
}
#pragma endregion

#pragma region test
extern "C" __declspec(dllexport)  int TestInt()
{
    return 114;
}
#pragma endregion