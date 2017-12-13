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
                              b2ParticleBodyContact* particleBodyContact) {
        ++mixedContacts;
        mixedArray.push_back(partSys->MyIndex);
        mixedArray.push_back(particleBodyContact->index);
        mixedArray.push_back((int64)particleBodyContact->body->GetUserData());
        mixedArray.push_back((int64)particleBodyContact->fixture->GetUserData());
        mixedArray.push_back(particleBodyContact->normal.x);
        mixedArray.push_back(particleBodyContact->normal.y);
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
};


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
extern "C" __declspec(dllexport) int End(void* worldPointer) {
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

extern "C" __declspec(dllexport) int AddParticleMaterial(void* partSysPtr, int materialFlags, float density, float stability, float extinguishingPoint, float meltingPoint, float boilingPoint, float ignitionPoint, float heatConductivity) {
	b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(partSysPtr);
	b2ParticleMaterialDef md;
	md.matFlags = materialFlags;
	md.mass = density * parts->GetParticleMass();
	md.stability = stability;
	md.extinguishingPoint = extinguishingPoint;
	md.meltingPoint = meltingPoint;
	md.boilingPoint = boilingPoint;
	md.ignitionPoint = ignitionPoint;
	md.heatConductivity = heatConductivity;
	return parts->CreateParticleMaterial(md);
	return -1;
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
	return NULL;
}

extern "C" __declspec(dllexport) void* GetDebug() {
	return static_cast<void*>(debugString);
}




extern "C" __declspec(dllexport)  void* SetContactListener(void* worldPointer) {
	b2World* world = static_cast<b2World*>(worldPointer);
	b2NewContactListener* cL = new b2NewContactListener();
	world->SetContactListener(cL);
	return static_cast<void*>(cL);
}
extern "C" __declspec(dllexport)  float* UpdateContactListener(void* contactPointer) {
	b2NewContactListener* cL = static_cast<b2NewContactListener*>(contactPointer);
	return cL->GetData();
}
#pragma endregion

#pragma region API Particle Systems

extern "C" __declspec(dllexport)  void* CreateParticleSystem(void* worldPtr, float radius, float damping, float gravityScale, int userData, float roomTemp, float heatLossRatio, float pointsPerLayer, float lowestPoint)
{
	b2World* world = static_cast<b2World*>(worldPtr);
	const b2ParticleSystemDef particleSystemDef;
	b2ParticleSystem* partSys = world->CreateParticleSystem(&particleSystemDef);
	partSys->SetRadius(radius);
	partSys->SetDamping(damping);
	partSys->SetGravityScale(gravityScale);
	partSys->SetIndex(userData);
	partSys->SetRoomTemperature(roomTemp);
	partSys->SetHeatLossRatio(heatLossRatio);
	partSys->SetPointsPerLayer(pointsPerLayer);
	partSys->SetLowestPoint(lowestPoint);
	partSys->CalcLayerValues();
	partSys->MyIndex = userData;
	return static_cast<void*>(partSys);
}
extern "C" __declspec(dllexport)  void SetParticleSystemIndex(void* partsysPointer, int userData) {
	b2ParticleSystem* sys = static_cast<b2ParticleSystem*>(partsysPointer);

	sys->MyIndex = userData;
}
extern "C" __declspec(dllexport)  int32 GetParticleIterations(float32 gravity, float32 particleRadius, float32 timeStep) {
	return b2CalculateParticleIterations(gravity, particleRadius, timeStep);
}

extern "C" __declspec(dllexport) void StepParticleInit(void* partSysPtr) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	partSys->SolveInit();
}
extern "C" __declspec(dllexport) void StepParticleIterationPart1(void* partSysPtr, int32 iteration) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	partSys->SolveIterationPart1(iteration);
}
extern "C" __declspec(dllexport) void StepParticleIterationPart2(void* partSysPtr) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	partSys->SolveIterationPart2();
}
extern "C" __declspec(dllexport) void StepParticleIterationPart3(void* partSysPtr) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	partSys->SolveIterationPart3();
}
extern "C" __declspec(dllexport) void StepParticleIterationPart4(void* partSysPtr) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	partSys->SolveIterationPart4();
}
extern "C" __declspec(dllexport) void StepParticleIterationPart5(void* partSysPtr) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	partSys->SolveIterationPart5();
}
extern "C" __declspec(dllexport) void StepParticleEnd(void* partSysPtr) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	partSys->SolveEnd();
}


extern "C" __declspec(dllexport)  void SetStaticPressureIterations(void* systemPointer,int iterations)
{
    b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(systemPointer);
    parts->SetStaticPressureIterations(iterations);
}
extern "C" __declspec(dllexport)  void SetAllParticleLifetimes(void* systemPointer, float time) {
    b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(systemPointer);
    int numberOfParticles = parts->GetParticleCount();
    for (int i = 0; i < numberOfParticles; ++i) {
        parts->SetParticleLifetime(i, time);
    }
    parts->SetDestructionByAge(true);
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

extern "C" __declspec(dllexport)  int* GetParticlesInShape(void* systemPointer, void* shapePointer, float x, float y, float rot)
{
    if (returnArray != NULL)
    {
        delete returnArray;
    }
    b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(systemPointer);
    b2Shape* shape = static_cast<b2Shape*>(shapePointer);
    b2Vec2 position = b2Vec2(x, y);
    b2Rot rotation = b2Rot(rot);
    b2Transform transform = b2Transform(position, rotation);
    
    const float* posXBuf = parts->GetPositionXBuffer();
	const float* posYBuf = parts->GetPositionYBuffer();
	uint32* layers = parts->GetCollisionLayerBuffer();
    int numparts = parts->GetParticleCount();
    
    std::vector<int> infoVector;
    int count = 0;
    
    for (int i = 0; i < numparts; i++)
    {
        if (shape->m_layerMask & layers[i] && shape->TestPoint(transform, b2Vec2(posXBuf[i], posYBuf[i])))
        {
            infoVector.push_back(i);
            count++;
        }
    }
    
    returnArray = new int[count + 1];
    returnArray[0] = count;
    for (int i = 0; i < count; i++)
    {
        returnArray[i + 1] = infoVector[i];
    }
    
    return returnArray;
}
extern "C" __declspec(dllexport)  float* GetParticlePositionsAndColors(void* partSysPtr) {
    
    if (positionArray != NULL)
    {
        delete positionArray;
    }
    
    b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
    
    int numberOfParticles = partSys->GetParticleCount();
    positionArray = new float[(numberOfParticles * 7) + 1];
    positionArray[0] = (float)numberOfParticles;
    float32* particlePositionXBuffer = partSys->GetPositionXBuffer();
	float32* particlePositionYBuffer = partSys->GetPositionYBuffer();
	float32* particlePositionZBuffer = partSys->GetPositionZBuffer();

    b2ParticleColor* particleColorBuffer = partSys->GetColorBuffer().data();
    
    int j = 1;
    for (int i = 0; i < numberOfParticles; ++i)
    {
		positionArray[j] = particlePositionXBuffer[i];
        positionArray[j + 1] = particlePositionYBuffer[i];
		positionArray[j + 2] = particlePositionZBuffer[i];
        positionArray[j + 3] = particleColorBuffer[i].r;
        positionArray[j + 4] = particleColorBuffer[i].g;
        positionArray[j + 5] = particleColorBuffer[i].b;
        positionArray[j + 6] = particleColorBuffer[i].a;
        j += 7;
    }
    return positionArray;
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
    //partSys->LiquidFunUpdatePairsAndTriads(partSys->GetParticleCount() - 1); HACK
}
extern "C" __declspec(dllexport)  int* GetParticleSystemContacts(void* partSysPtr) {
    
    if (returnArray != NULL)
    {
        delete returnArray;
    }
    
    b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
    int numContacts = partSys->GetContactCount();
    const int32* contactIdxAList = partSys->GetContactIdxAs();
	const int32* contactIdxBList = partSys->GetContactIdxBs();
    int32* userdata = partSys->GetUserDataBuffer();
    
    returnArray = new int[(numContacts * 4) + 1];
    returnArray[0] = numContacts;
    for (int i = 0; i < numContacts; ++i)
    {
        returnArray[(i * 4) + 1] = contactIdxAList[i];
        returnArray[(i * 4) + 2] = contactIdxBList[i];
        returnArray[(i * 4) + 3] = (int64)(userdata[contactIdxAList[i]]);
        returnArray[(i * 4) + 4] = (int64)(userdata[contactIdxBList[i]]);
    }
    return returnArray;
}
extern "C" __declspec(dllexport)  float* GetParticleSystemBodyContacts(void* partSysPtr) {
    
    if (positionArray != NULL)
    {
        delete positionArray;
    }
    
    b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
    
    int count = partSys->GetBodyContactCount();
    const int32* contactIdxs		  = partSys->GetBodyContactIdxs();
	b2Body** contactBodys		  = partSys->GetBodyContactBodys();
	b2Fixture** contactFixtures = partSys->GetBodyContactFixtures();
	const float32* contactNormalXs	  = partSys->GetBodyContactNormalXs();
	const float32* contactNormalYs	  = partSys->GetBodyContactNormalYs();
	const float32* contactWeights	  = partSys->GetBodyContactWeights();
    int32* userdata = partSys->GetUserDataBuffer();
    
    positionArray = new float[(count*7) +1];
    positionArray[0] = (float)count;
    
    for (int i = 0; i < count; ++i)
    {
        positionArray[(i * 7) + 1] = (float)contactIdxs[i];
        positionArray[(i * 7) + 2] = (float)((int64)(userdata[contactIdxs[i]]));
        positionArray[(i * 7) + 3] = (float)((int64)(contactBodys[i]->GetUserData()));
        positionArray[(i * 7) + 4] = (float)((int64)(contactFixtures[i]->GetUserData()));
        positionArray[(i * 7) + 5] = contactNormalXs[i];
        positionArray[(i * 7) + 6] = contactNormalYs[i];
        positionArray[(i * 7) + 7] = contactWeights[i];
    }
    
    return positionArray;
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
extern "C" __declspec(dllexport)  void CreateParticleInSystem(void* systemPointer, int flags, float posX, float posY, float posZ, float velX, float velY, int r, int g, int b, int a, float lifetime, float health) {
    b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(systemPointer);
    b2ParticleDef pd;
    pd.flags = static_cast<b2ParticleFlag>(flags);
    pd.positionX = posX;
	pd.positionY = posY;
	pd.positionZ = posZ;
    pd.velocityX = velX;
	pd.velocityY = velY;
    pd.lifetime = lifetime;
	pd.health = health;
    pd.color.Set((uint8)r, (uint8)g, (uint8)b, (uint8)a);
    parts->CreateParticle(pd);
}
extern "C" __declspec(dllexport)  void CreateParticleInGroup(void* systemPointer, int groupIdx, int flags, float posX, float posY, float posZ, float velX, float velY, int r, int g, int b, int a, float lifetime, float health, float heat, int userdata) {
	b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(systemPointer);
	b2ParticleDef pd;
	pd.flags = flags;
	pd.positionX = posX;
	pd.positionY = posY;
	pd.positionZ = posZ;
	pd.velocityX = velX;
	pd.velocityY = velY;
	pd.color.Set((uint8)r, (uint8)g, (uint8)b, (uint8)a);
	pd.lifetime = lifetime;
	pd.heat = heat;
	pd.health = health;
	pd.groupIdx = groupIdx;
	pd.matIdx = parts->GetGroupMaterialIdx(groupIdx);
	pd.userData = parts->GetGroupUserData(groupIdx);
	parts->CreateParticle(pd);
}
extern "C" __declspec(dllexport)  void DestroySelectedParticles(void* partSysPtr, int* indexArray) {
    b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
    for (int i = 1; i < indexArray[0]+1; ++i) {
        partSys->DestroyParticle(indexArray[i]);
    }
}

extern "C" __declspec(dllexport)  void SetSelectedParticleFlags(void* partSysPtr, int* indexArray, int flags) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	for (int i = 1; i < indexArray[0] + 1; ++i) {
		partSys->SetParticleFlags(indexArray[i], flags);
	}
	//partSys->LiquidFunUpdatePairsAndTriads(partSys->GetParticleCount() - 1); HACK
}
extern "C" __declspec(dllexport)  void AddSelectedParticleFlags(void* systemPointer, int* indexArray, int flags) {
	b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(systemPointer);
	for (int i = 1; i < indexArray[0] + 1; ++i) {
		parts->AddParticleFlags(indexArray[i], flags);
	}
}
extern "C" __declspec(dllexport)  void AddParticleFlagsInShape(void* systemPointer, void* shapePtr, float shapeX, float shapeY, float shapeRot, int flags) {
	b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(systemPointer);
	b2Shape* shape = static_cast<b2Shape*>(shapePtr);
	b2Vec2 position = b2Vec2(shapeX, shapeY);
	b2Rot rotation = b2Rot(shapeRot);
	b2Transform transform = b2Transform(position, rotation); 
	b2ParticleFlag pf;
	pf = static_cast<b2ParticleFlag>(flags);

	const float32* posX = parts->GetPositionXBuffer();
	const float32* posY = parts->GetPositionYBuffer();
	int numparts = parts->GetParticleCount();
	for (int i = 0; i < numparts; i++)
	{
		if (shape->TestPoint(transform, b2Vec2(posX[i], posY[i])))
		{
			parts->AddParticleFlags(i, pf);
		}
	}
}

extern "C" __declspec(dllexport)  void SetSelectedParticleColor(void* partSysPtr, int* indexArray, int r, int g, int b, int a) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	b2ParticleColor* colours = partSys->GetColorBuffer().data();

	for (int i = 1; i < indexArray[0] + 1; ++i)
	{
		colours[indexArray[i]].r = r;
		colours[indexArray[i]].g = g;
		colours[indexArray[i]].b = b;
		colours[indexArray[i]].a = a;
	}
}
extern "C" __declspec(dllexport)  void SetSelectedParticleUserData(void* partSysPtr, int* indexArray, int ud) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	int32* datas = partSys->GetUserDataBuffer();
	for (int i = 1; i < indexArray[0] + 1; ++i)
	{
		datas[indexArray[i]] = ud;
	}
}
extern "C" __declspec(dllexport)  void SetSelectedParticleTemperature(void* partSysPtr, int* indexArray, float t) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	float* datas = partSys->GetHeatBuffer();
	for (int i = 1; i < indexArray[0] + 1; ++i)
	{
		datas[indexArray[i]] = t;
	}
}

extern "C" __declspec(dllexport)  void ApplyForceToSelectedParticles(void* partSysPtr, int* indexArray, float x, float y) {
    b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
    for (int i = 1; i < indexArray[0]+1; ++i) {
        partSys->ParticleApplyForce(indexArray[i], x, y);
    }
}
extern "C" __declspec(dllexport)  void ApplyLinearImpulseToSelectedParticles(void* partSysPtr, int* indexArray, float x, float y) {
    b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
    b2Vec2 force = b2Vec2(x, y);
    for (int i = 1; i < indexArray[0]+1; ++i) {
        partSys->ParticleApplyLinearImpulse(indexArray[i], force);
    }
}
extern "C" __declspec(dllexport)  void ExplodeSelectedParticles(void* partSysPtr, int* indexArray, float centreX, float centreY, float strenght) {
    b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
    float32* ppbx = partSys->GetPositionXBuffer();
	float32* ppby = partSys->GetPositionYBuffer();
    for (int i = 1; i < indexArray[0]+1; ++i)
    {
        int ind = indexArray[i];
        b2Vec2 force = b2Vec2(ppbx[ind] - centreX, ppby[ind] - centreY);
        force.Normalize();
        force *= strenght;
        partSys->ParticleApplyForce(ind, force.x, force.y);
    }
}
extern "C" __declspec(dllexport)  void ExplodeParticlesInShape(void* partSysPtr, float centreX, float centreY, void* shapePtr, float shapeX, float shapeY, float shapeRot, float strenght) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	b2Shape* shape = static_cast<b2Shape*>(shapePtr);
	b2Vec2 position = b2Vec2(shapeX, shapeY);
	b2Rot rotation = b2Rot(shapeRot);
	b2Transform transform = b2Transform(position, rotation);

	const float32* posX = partSys->GetPositionXBuffer();
	const float32* posY = partSys->GetPositionYBuffer();
	int numparts = partSys->GetParticleCount();
	for (int i = 0; i < numparts; i++)
	{
		if (shape->TestPoint(transform, b2Vec2(posX[i], posY[i])))
		{
			b2Vec2 force = b2Vec2(posX[i] - centreX, posY[i] - centreY);
			force.Normalize();
			force *= strenght;
			partSys->ParticleApplyForce(i, force.x, force.y);
		}
	}
}
extern "C" __declspec(dllexport)  void ExplodeParticlesWithFlag(void* partSysPtr, float centreX, float centreY, int partFlags, float strenght) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	float32* posX = partSys->GetPositionXBuffer();
	float32* posY = partSys->GetPositionYBuffer();
	b2ParticleFlag bf;
	bf = static_cast<b2ParticleFlag>(partFlags);
	const uint32* FlagBuffer = partSys->GetFlagsBuffer();
	
	int numparts = partSys->GetParticleCount();
	for (int i = 0; i < numparts; i++)
	{
		if (FlagBuffer[i] & partFlags)
		{
			b2Vec2 force = b2Vec2(posX[i] - centreX, posY[i] - centreY);
			force.Normalize();
			force *= strenght;
			partSys->ParticleApplyForce(i, force.x, force.y);
		}
	}
}

enum WhichDataToGet
{
	GetPositions = 1 << 0,
	GetColors = 1 << 1,
	GetLifeTimes = 1 << 2,
	GetHealth = 1 << 3,
	GetWeights = 1 << 4,
	GetVelocities = 1 << 5,
	GetUserData = 1 << 6,
	GetHeat = 1 << 7
};
extern "C" __declspec(dllexport)  float* GetSelectedParticlesDetails(void* partSysPtr, int* indexArray, bool position, bool color, bool age, bool weight, bool velocity, bool userdata, bool heat, bool health)
{
	//TODO
	return 0;
}
extern "C" __declspec(dllexport)  void GetParticlesDetails(void* partSysPtr, int whichDataMask, float** partPosXPtr, float** partPosYPtr, float** partPosZPtr, int** partColorPtr,  float** partLifeTimePtr, float** partHealthPtr, float** partWeightPtr, float** partVelocityXPtr, float**  partVelocityYPtr, int**  partUserDataPtr, float**  partHeatPtr) {
	b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
	int partNum = partSys->GetParticleCount();
	int floatArraySize = partNum * sizeof(float);
	int intArraySize = partNum * sizeof(int);

	if (whichDataMask & GetPositions)
	{
		memcpy(*partPosXPtr, partSys->GetPositionXBuffer(), floatArraySize);
		memcpy(*partPosYPtr, partSys->GetPositionYBuffer(), floatArraySize);
		memcpy(*partPosZPtr, partSys->GetPositionZBuffer(), floatArraySize);
	}
	if (whichDataMask & GetColors)
	{
		b2ParticleColor* colorBuffer = partSys->GetColorBuffer().data();
		uint32* c = new uint32[partNum];
		for (int i = 0; i < partNum; i++)
		{
			c[i] = colorBuffer[i].r  
				 | colorBuffer[i].g << 8
				 | colorBuffer[i].b << 16
				 | colorBuffer[i].a << 24;
		}
		memcpy(*partColorPtr, c, intArraySize);
		delete c;
	}
	if (whichDataMask & GetLifeTimes)
	{
		float* lifeTimeBuffer = new float[partNum];
		for (int i = 0; i < partNum; i++)
			lifeTimeBuffer[i] = partSys->GetParticleLifetime(i);
		memcpy(*partLifeTimePtr, lifeTimeBuffer, intArraySize);
		delete lifeTimeBuffer;
	}
	if (whichDataMask & GetHealth)
	{
		memcpy(*partHealthPtr, partSys->GetHealthBuffer(), intArraySize);
	}
	if (whichDataMask & GetWeights)
	{
		memcpy(*partWeightPtr, partSys->GetWeightBuffer(), intArraySize);
	}
	if (whichDataMask & GetVelocities)
	{
		memcpy(*partVelocityXPtr, partSys->GetVelocityXBuffer(), floatArraySize);
		memcpy(*partVelocityYPtr, partSys->GetVelocityYBuffer(), floatArraySize);
	}
	if (whichDataMask & GetUserData)
	{
		memcpy(*partUserDataPtr, partSys->GetUserDataBuffer(), intArraySize);
	}
	if (whichDataMask & GetHeat)
	{
		memcpy(*partHeatPtr, partSys->GetHeatBuffer(), intArraySize);
	}

}
#pragma endregion

#pragma region ParticleGroups
/*
extern "C" __declspec(dllexport)  void* CreateParticleGroup(void* partSysPtr, int particleTypes, int groupTypes, float angle, float strength, float angVel, float linVelX, float linVelY, void* shape, int r, int g, int b, int a, float stride, float lifetime, int userData) {
    b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(partSysPtr);
    b2Shape* m_shape = static_cast<b2Shape*>(shape);
    b2ParticleGroupFlag bgf;
    bgf = static_cast<b2ParticleGroupFlag>(groupTypes);
    b2ParticleFlag bf;
    bf = static_cast<b2ParticleFlag>(particleTypes);
    b2ParticleGroupDef pd;
    pd.flags = bf;
    pd.groupFlags = bgf;
    pd.shape = m_shape;
    pd.angle = angle;
    pd.strength = strength;
    pd.angularVelocity = angVel;
    pd.linearVelocity = b2Vec2(linVelX, linVelY);
    pd.stride = stride;
    pd.lifetime = lifetime;
    pd.color.Set((uint8)r, (uint8)g, (uint8)b, (uint8)a);
    pd.userData = (void*)userData;
    b2ParticleGroup* particleGroup = parts->CreateParticleGroup(pd);
    return static_cast<void*>(particleGroup);
}*/
extern "C" __declspec(dllexport)  int CreatePG(void* partSysPtr, int partFlags, int groupFlags, int matIdx, int collisionGroup, int layer, float angle, float strength, float angVel, float linVelX, float linVelY, void* shape, int r, int g, int b, int a, float stride, float lifetime, float health, float heat, int userData) {
	b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(partSysPtr);
	b2Shape* m_shape = static_cast<b2Shape*>(shape);
	b2ParticleGroupFlag bgf = static_cast<b2ParticleGroupFlag>(groupFlags);
	b2ParticleFlag bf = static_cast<b2ParticleFlag>(partFlags);
	b2ParticleGroupDef pd;
	pd.flags = bf;
	pd.groupFlags = bgf;
	pd.matIdx = matIdx;
	pd.collisionGroup = collisionGroup;
	pd.layer = layer;
	pd.shape = m_shape;
	pd.angle = angle;
	pd.strength = strength;
	pd.angularVelocity = angVel;
	pd.linearVelocity = b2Vec2(linVelX, linVelY);
	pd.stride = stride;
	pd.lifetime = lifetime;
	pd.health = health;
	pd.color.Set((uint8)r, (uint8)g, (uint8)b, (uint8)a);
	pd.userData = userData;
	pd.heat = heat;
	int32 particleGroup = parts->CreateParticleGroup(pd);
	return particleGroup;
}
extern "C" __declspec(dllexport)  int CreatePG2(void* partSysPtr, int partCount, int partFlags, int groupFlags, int matIdx, int collisionGroup, float strength, float* posX, float* posY, float* posZ, float velX, float velY, int* r, int* g, int* b, int* a, float lifetime, float health, float heat, int userData) {
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
	pd.positionData = new b2Vec3[partCount];
	pd.colorData = new b2ParticleColor[partCount];
	for (int32 i = 0; i < partCount; i++)
	{
		pd.positionData[i].Set(posX[i], posY[i], posZ[i]);
		pd.colorData[i].Set((uint8)r[i], (uint8)g[i], (uint8)b[i], (uint8)a[i]);
	}
	int32 particleGroup = parts->CreateParticleGroup(pd);
	return particleGroup;
}
extern "C" __declspec(dllexport)  void JoinParticleGroups(void* partSysPtr, int groupAIdx, int groupBIdx)
{
    b2ParticleSystem* system = static_cast<b2ParticleSystem*>(partSysPtr);
    system->JoinParticleGroups(groupAIdx, groupBIdx);
}
extern "C" __declspec(dllexport)  int* AreParticlesInGroup(void* groupPointer, int* indices) {
    b2ParticleGroup* group = static_cast<b2ParticleGroup*>(groupPointer);
    std::vector<int> flags;
    for (int i = 1; i < indices[0]+1; ++i) {
        flags.push_back(group->ContainsParticle(indices[i]) ? 1 : 0);
    }
    int* flagArray = &flags[0];
    return flagArray;
}
extern "C" __declspec(dllexport)  void SetParticleFlagsInGroup(void* partSysPtr, void* groupPointer, int flags) {
    b2ParticleGroup* group = static_cast<b2ParticleGroup*>(groupPointer);
    b2ParticleSystem* system = static_cast<b2ParticleSystem*>(partSysPtr);
    int numParts = system->GetParticleCount();
    for (int i = 0; i < numParts; ++i) {
        if (group->ContainsParticle(i)) {
            system->SetParticleFlags(i, flags);
        }
    }
    //system->LiquidFunUpdatePairsAndTriads(system->GetParticleCount() - 1); HACK
}
extern "C" __declspec(dllexport)  void SetParticleLifetimesInGroup(void* partSysPtr, void* groupPointer, int lifetime) {
    b2ParticleGroup* group = static_cast<b2ParticleGroup*>(groupPointer);
    b2ParticleSystem* system = static_cast<b2ParticleSystem*>(partSysPtr);
    int numParts = system->GetParticleCount();
    for (int i = 0; i < numParts; ++i) {
        if (group->ContainsParticle(i)) {
            system->SetParticleLifetime(i, lifetime);
        }
    }
}
extern "C" __declspec(dllexport)  int GetParticleGroupCount(void* groupPointer)
{
    b2ParticleGroup* group = static_cast<b2ParticleGroup*>(groupPointer);
    return group->GetParticleCount();
}
extern "C" __declspec(dllexport)  int GetParticleGroupFirstIndex(void* groupPointer)
{
	b2ParticleGroup* group = static_cast<b2ParticleGroup*>(groupPointer);
	return group->GetFirstIndex();
}
extern "C" __declspec(dllexport)  int GetParticleGroupLastIndex(void* groupPointer)
{
	b2ParticleGroup* group = static_cast<b2ParticleGroup*>(groupPointer);
	return group->GetLastIndex();
}
extern "C" __declspec(dllexport)  void ApplyForceToParticleGroup(void* groupPointer, float forceX, float forceY) {
    b2ParticleGroup* group = static_cast<b2ParticleGroup*>(groupPointer);
    group->ApplyForce(forceX, forceY);
}
extern "C" __declspec(dllexport)  void ApplyLinearImpulseToParticleGroup(void* groupPointer, float forceX, float forceY) {
    b2ParticleGroup* group = static_cast<b2ParticleGroup*>(groupPointer);
    b2Vec2 impulse = b2Vec2(forceX, forceY);
    group->ApplyLinearImpulse(impulse);
}
extern "C" __declspec(dllexport)  float* GetParticleGroupPosition(void* particleGroupPointer)
{
    
    if (positionArray != NULL)
    {
        delete positionArray;
    }
    
    b2ParticleGroup* pGroup = static_cast<b2ParticleGroup*>(particleGroupPointer);
    b2Vec2 vec = pGroup->GetPosition();
    positionArray = new float[2];
    positionArray[0] = vec.x;
    positionArray[1] = vec.y;
    
    return positionArray;
}
extern "C" __declspec(dllexport)  float* GetPGVelocity(void* partSysPtr, int pgIdx)
{
    //TODO
    return positionArray;
}
extern "C" __declspec(dllexport)  int* GetParticleGroupParticles(void* particleGroupPointer)
{
	if (returnArray != NULL)
	{
		delete returnArray;
	}
	b2ParticleGroup* pGroup = static_cast<b2ParticleGroup*>(particleGroupPointer);

	std::vector<int> infoVector;
	int count = 0;
	for (int i = pGroup->GetBufferIndex(); i < pGroup->GetBufferIndex() + pGroup->GetParticleCount(); i++)
	{
		infoVector.push_back(i);
		count++;
	}
	returnArray = new int[count + 1];
	returnArray[0] = count;
	for (int i = 0; i < count; i++)
	{
		returnArray[i + 1] = infoVector[i];
	}

	return returnArray;
}
extern "C" __declspec(dllexport)  void DeleteParticlesInGroup(void* particleGroupPointer) {
    b2ParticleGroup* pGroup = static_cast<b2ParticleGroup*>(particleGroupPointer);
    pGroup->DestroyParticles();
}

#pragma endregion

#pragma region Particles Superceded

extern "C" __declspec(dllexport)  void SetSingleParticleLifetime(void* systemPointer, int index, float time) {
    b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(systemPointer);
    parts->SetParticleLifetime(index, time);
}
/*
 extern "C"  int* GetParticlesInShape(void* systemPointer, void* shapePointer, float x, float y, float rot) {
 b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(systemPointer);
 b2Shape* shape = static_cast<b2Shape*>(shapePointer);
 b2Vec2 position = b2Vec2(x, y);
 b2Rot rotation = b2Rot(rot);
 b2Transform transform = b2Transform(position, rotation);
 std::vector<int> ints = parts->GetParticlesInShape(*shape, transform);
 int* arr = &ints[0];
 int* arr1 = new int[1];
 arr1[0] = 42;
 return arr1;
 }
 */
extern "C" __declspec(dllexport)  void SetParticleFlagsUpToLimit(void* partSysPtr, int flags, int limit) {
    b2ParticleSystem* partSys = static_cast<b2ParticleSystem*>(partSysPtr);
    for (int i = 0; i < limit; ++i) {
        partSys->SetParticleFlags(i, flags);
    }
    //partSys->LiquidFunUpdatePairsAndTriads(partSys->GetParticleCount() - 1); HACK
}

extern "C" __declspec(dllexport)  void ApplyLinearImpulseToParticles(void* partSysPtr, int first, int last, float impulseX, float impulseY) {
    b2ParticleSystem* system = static_cast<b2ParticleSystem*>(partSysPtr);
    b2Vec2 impulse = b2Vec2(impulseX, impulseY);
    system->ApplyLinearImpulse(first, last, impulse);
}
extern "C" __declspec(dllexport)  void ApplyForceToParticles(void* partSysPtr, int first, int last, float impulseX, float impulseY) {
    b2ParticleSystem* system = static_cast<b2ParticleSystem*>(partSysPtr);
    system->ApplyForce(first, last, impulseX, impulseY);
}
extern "C" __declspec(dllexport)  void ApplyForceToParticle(void* partSysPtr, int index, float impulseX, float impulseY) {
    b2ParticleSystem* system = static_cast<b2ParticleSystem*>(partSysPtr);
    system->ParticleApplyForce(index, impulseX, impulseY);
}
extern "C" __declspec(dllexport)  void ApplyLinearImpulseToParticle(void* partSysPtr, int index, float impulseX, float impulseY) {
    b2ParticleSystem* system = static_cast<b2ParticleSystem*>(partSysPtr);
    b2Vec2 impulse = b2Vec2(impulseX, impulseY);
    system->ParticleApplyLinearImpulse(index, impulse);
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
extern "C" __declspec(dllexport)  void* GetBoxShapeDef(float width, float height, float centreX, float centreY, float angle, int layerMask) {
    b2Vec2 centre = b2Vec2(centreX, centreY);
    b2PolygonShape* shape = new b2PolygonShape();
    shape->SetAsBox(width, height, centre, angle);
	shape->m_layerMask = layerMask;
    return static_cast<void*>(shape);
}
extern "C" __declspec(dllexport)  void* GetCircleShapeDef(float radius, float centreX, float centreY, int layerMask) {
    b2Vec2 centre = b2Vec2(centreX, centreY);
    b2CircleShape* shape = new b2CircleShape();
    shape->m_p = centre;
    shape->m_radius = radius;
	shape->m_layerMask = layerMask;
    return static_cast<void*>(shape);
}
extern "C" __declspec(dllexport)  void* GetChainShapeDef(float* vertArray, bool loop, int layerMask) {
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
	shape->m_layerMask = layerMask;
    return static_cast<void*>(shape);
}
extern "C" __declspec(dllexport)  void* GetPolygonShapeDef(float* vertArray, int layerMask) {
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
	shape->m_layerMask = layerMask;
    return static_cast<void*>(shape);
}
extern "C" __declspec(dllexport)  void* GetEdgeShapeDef(float x1, float y1, float x2, float y2, int layerMask) {
    b2Vec2 vec1 = b2Vec2(x1, y1);
    b2Vec2 vec2 = b2Vec2(x2, y2);
    b2EdgeShape* shape = new b2EdgeShape();
    shape->Set(vec1, vec2);
	shape->m_layerMask = layerMask;
    return static_cast<void*>(shape);
}
extern "C" __declspec(dllexport)  void* GetEllipseShapeDef(float outerRadius, float divisions, int layerMask) {
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
	shape->m_layerMask = layerMask;
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

extern "C" __declspec(dllexport)  void* CreateBody(void* worldPointer, int type, float xPosition, float yPosition, float angle, float linearDamping, float angularDamping, void* materialPointer, float heat, float health, int flags, float gravityScale, int userData) {
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

    b2Body* m_body = world->CreateBody(&bd);
    return static_cast<void*>(m_body);
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
extern "C" __declspec(dllexport)  float* GetAllBodyInfo(void** bodPointers, int numbodies)
{
    if (positionArray != NULL)
    {
        delete positionArray;
    }
    
    positionArray = new float[numbodies*6];
    
    for (int i = 0; i < numbodies; i++)
    {
        b2Body* m_body = static_cast<b2Body*>(bodPointers[i]);
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
extern "C" __declspec(dllexport)  int GetBodyContactsCount(void* bodyPointer)
{
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    //b2ContactEdge * contacts =  m_body->GetContactList() ;
    
    int numconts = 0;
    //std::vector<int> conts;
    //conts.push_back(numconts);
    
    for ( b2ContactEdge* ce =  m_body->GetContactList(); ce; ce = ce->next)
    {
        numconts++;
        //conts.push_back((int)ce->other->GetUserData());
    }
    //conts[0] = numconts;
    
    //int* returnArray = &conts[0];
    return numconts;
    
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
extern "C" __declspec(dllexport)  bool GetBodyAwake(void* bodyPointer) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    return m_body->IsAwake();
}
extern "C" __declspec(dllexport)  void SetBodyActive(void* bodyPointer, bool isActive) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    m_body->SetActive(isActive);
}
extern "C" __declspec(dllexport)  bool GetBodyActive(void* bodyPointer) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    return m_body->IsActive();
}
extern "C" __declspec(dllexport)  void** GetBodyFixtures(void* bodyPointer) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    std::vector<void*> fixturesVec;
    fixturesVec.clear();
    for (b2Fixture* f = m_body->GetFixtureList(); f; f = f->GetNext()) {
        fixturesVec.push_back(static_cast<void*>(f));
    }
    void** fixturesArray = &fixturesVec[0];
    return fixturesArray;
}
extern "C" __declspec(dllexport)  int GetBodyFixturesCount(void* bodyPointer) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    int i = 0;
    for (b2Fixture* f = m_body->GetFixtureList(); f; f = f->GetNext()) {
        ++i;
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
extern "C" __declspec(dllexport)  void SetBodyLinearVelocity(void* bodyPointer, float x, float y) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    b2Vec2 vel = b2Vec2(x, y);
    m_body->SetLinearVelocity(vel);
}
extern "C" __declspec(dllexport)  float* GetBodyLinearVelocity(void* bodyPointer) {
    
    if (positionArray != NULL)
    {
        delete positionArray;
    }
    
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    positionArray = new float[2];
    positionArray[0] = m_body->GetLinearVelocity().x;
    positionArray[1] = m_body->GetLinearVelocity().y;
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

extern "C" __declspec(dllexport)  void DeleteBody(void* worldPointer, void* bodyPointer) {
    b2World* world = static_cast<b2World*>(worldPointer);
    b2Body* body = static_cast<b2Body*>(bodyPointer);
    world->DestroyBody(body);
}

#pragma endregion

#pragma region Fixture

extern "C" __declspec(dllexport)  void* AddFixture(void* bodyPointer, int shapeType, void* shapePointer, int collisionLayers, bool isSensor, int userData) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
	b2CollisionLayerMask clm;
	clm = static_cast<b2CollisionLayerMask>(collisionLayers);
    b2FixtureDef fd;
	m_body->GetMaterial()->m_density;
	m_body->GetMaterial()->m_friction;
	fd.isSensor = isSensor;
	m_body->GetMaterial()->m_bounciness;
    fd.userData = (void*)userData;
	fd.collisionLayers = clm;
    if (shapeType == 0) {
        b2PolygonShape* aShape = static_cast<b2PolygonShape*>(shapePointer);
        b2PolygonShape shape = *aShape;
        fd.shape = &shape;
        b2Fixture* m_fixture = m_body->CreateFixture(&fd);
        return static_cast<void*>(m_fixture);
    }
    else if (shapeType == 1) {
        b2CircleShape* aShape = static_cast<b2CircleShape*>(shapePointer);
        b2CircleShape shape = *aShape;
        fd.shape = &shape;
        b2Fixture* m_fixture = m_body->CreateFixture(&fd);
        return static_cast<void*>(m_fixture);
    }
    else if (shapeType == 2) {
        b2EdgeShape* aShape = static_cast<b2EdgeShape*>(shapePointer);
        b2EdgeShape shape = *aShape;
        fd.shape = &shape;
        b2Fixture* m_fixture = m_body->CreateFixture(&fd);
        return static_cast<void*>(m_fixture);
    }
    else {
        b2ChainShape* aShape = static_cast<b2ChainShape*>(shapePointer);
        b2ChainShape shape = *aShape;
        fd.shape = &shape;
        b2Fixture* m_fixture = m_body->CreateFixture(&fd);
        return static_cast<void*>(m_fixture);
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

extern "C" __declspec(dllexport)  void SetFixtureFilterData(void* fixturePointer, int32 groupIndex, uint16 categoryBits, uint16 maskBits) {
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    b2Filter filter = b2Filter();
    filter.groupIndex = groupIndex;
    filter.maskBits = maskBits;
    filter.categoryBits = categoryBits;
    m_fixture->SetFilterData(filter);
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

extern "C" __declspec(dllexport)  void DeleteFixture(void* bodyPointer, void* fixturePointer) {
    b2Fixture* fixture = static_cast<b2Fixture*>(fixturePointer);
    b2Body* body = static_cast<b2Body*>(bodyPointer);
    body->DestroyFixture(fixture);
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