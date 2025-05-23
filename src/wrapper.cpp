#include "pch.h"
#include "wrapper.h"

#include <iostream>
#include <vector>
#include <cstring>

#include <PxPhysicsAPI.h>
#include <extensions/PxDefaultCpuDispatcher.h>
#include <cooking/PxCooking.h>
#include <extensions/PxRigidActorExt.h>
#include <extensions/PxDefaultStreams.h>
#include <geometry/PxTriangleMesh.h>
#include <geometry/PxConvexMesh.h>
#include <PxSimulationEventCallback.h>
#include <PxFiltering.h>

using namespace physx;

static PxTriggerCallback  g_triggerCb = nullptr;

static PxCollisionCallback g_enterCb  = nullptr;
static PxCollisionCallback g_stayCb   = nullptr;
static PxCollisionCallback g_exitCb   = nullptr;

static std::vector<uint32_t> g_layerMasks(32, 0xFFFFFFFFu);
static uint32_t               g_layerCount = 32;

struct LocalErrorCallback : PxErrorCallback {
    void reportError(
        PxErrorCode::Enum code,
        const char* message,
        const char* file,
        int line
    ) override {
        std::cerr
            << "[PhysX Error] Code=" << static_cast<int>(code)
            << " Message=\"" << message << "\""
            << " (at " << file << ":" << line << ")"
            << std::endl;
    }
};

API void RegisterTriggerCallback(PxTriggerCallback cb) {
    g_triggerCb = cb;
}

API PxFoundationHandle CreateFoundation() {
    static PxDefaultAllocator alloc;
    static LocalErrorCallback errCb;
    return PxCreateFoundation(PX_PHYSICS_VERSION, alloc, errCb);
}
API void ReleaseFoundation(PxFoundationHandle f_) {
    reinterpret_cast<PxFoundation*>(f_)->release();
}

API PxPhysicsHandle CreatePhysics(PxFoundationHandle f_) {
    PxFoundation* f = reinterpret_cast<PxFoundation*>(f_);
    static PxTolerancesScale scale;
    return PxCreatePhysics(PX_PHYSICS_VERSION, *f, scale);
}
API void ReleasePhysics(PxPhysicsHandle p_) {
    reinterpret_cast<PxPhysics*>(p_)->release();
}

API PxCookingHandle CreateCooking(PxFoundationHandle f_) {
    PxFoundation* f = reinterpret_cast<PxFoundation*>(f_);
    PxTolerancesScale scale;
    PxCookingParams params(scale);
    return PxCreateCooking(PX_PHYSICS_VERSION, *f, params);
}
API void ReleaseCooking(PxCookingHandle c_) {
    reinterpret_cast<PxCooking*>(c_)->release();
}

API int32_t CookTriangleMesh(
    PxCookingHandle cook_,
    const float* vertices, int32_t vertCount,
    const int32_t* indices, int32_t idxCount,
    void* outBuffer,
    int32_t         bufferSize)
{
    PxCooking* cooker = reinterpret_cast<PxCooking*>(cook_);
    PxTriangleMeshDesc desc;
    desc.points.count = vertCount / 3;
    desc.points.stride = sizeof(float) * 3;
    desc.points.data = vertices;
    desc.triangles.count = idxCount / 3;
    desc.triangles.stride = sizeof(int32_t) * 3;
    desc.triangles.data = indices;

    PxDefaultMemoryOutputStream stream;
    if (!cooker->cookTriangleMesh(desc, stream))
        return -1;

    PxU32 size = stream.getSize();
    if (!outBuffer || bufferSize < int32_t(size))
        return int32_t(size);

    memcpy(outBuffer, stream.getData(), size);
    return int32_t(size);
}


API void ReleaseTriangleMesh(PxTriangleMeshHandle tm_) {
    reinterpret_cast<PxTriangleMesh*>(tm_)->release();
}

API int32_t CookConvexMesh(
    PxCookingHandle cook_,
    const float* vertices,
    int32_t         vertCount,
    void* outBuffer,
    int32_t         bufferSize)
{
    PxCooking* cooker = reinterpret_cast<PxCooking*>(cook_);
    PxConvexMeshDesc desc;
    desc.points.count = vertCount / 3;
    desc.points.stride = sizeof(float) * 3;
    desc.points.data = vertices;
    desc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

    PxDefaultMemoryOutputStream stream;
    if (!cooker->cookConvexMesh(desc, stream))
        return -1;

    PxU32 size = stream.getSize();
    if (!outBuffer || bufferSize < int32_t(size))
        return int32_t(size);

    memcpy(outBuffer, stream.getData(), size);
    return int32_t(size);
}

API PxTriangleMeshHandle CreateTriangleMesh(
    PxPhysicsHandle phys_,
    const void* cookedData,
    int32_t         dataSize)
{
    PxPhysics* physics = reinterpret_cast<PxPhysics*>(phys_);
    PxDefaultMemoryInputData input(
        const_cast<PxU8*>(
            reinterpret_cast<const PxU8*>(cookedData)
            ),
        static_cast<PxU32>(dataSize)
    );
    return physics->createTriangleMesh(input);
}

API PxConvexMeshHandle CreateConvexMesh(
    PxPhysicsHandle phys_,
    const void* cookedData,
    int32_t         dataSize)
{
    PxPhysics* physics = reinterpret_cast<PxPhysics*>(phys_);
    PxDefaultMemoryInputData input(
        const_cast<PxU8*>(
            reinterpret_cast<const PxU8*>(cookedData)
            ),
        static_cast<PxU32>(dataSize)
    );
    return physics->createConvexMesh(input);
}

API void ReleaseConvexMesh(PxConvexMeshHandle cm_) {
    reinterpret_cast<PxConvexMesh*>(cm_)->release();
}

API PxMaterialHandle CreateMaterial(
    PxPhysicsHandle p_,
    float staticFriction,
    float dynamicFriction,
    float restitution)
{
    PxPhysics* p = reinterpret_cast<PxPhysics*>(p_);
    return p->createMaterial(staticFriction, dynamicFriction, restitution);
}
API void ReleaseMaterial(PxMaterialHandle m_) {
    reinterpret_cast<PxMaterial*>(m_)->release();
}

API PxSceneHandle CreateScene(
    PxPhysicsHandle p_,
    float gx, float gy, float gz,
    int32_t numThreads)
{
    PxPhysics* p = reinterpret_cast<PxPhysics*>(p_);
    PxSceneDesc desc(p->getTolerancesScale());
    desc.gravity = PxVec3(gx, gy, gz);
    desc.filterShader = PxDefaultSimulationFilterShader;
    desc.cpuDispatcher = PxDefaultCpuDispatcherCreate(numThreads);
    return p->createScene(desc);
}
API void SimulateScene(PxSceneHandle s_, float dt) {
    reinterpret_cast<PxScene*>(s_)->simulate(dt);
}

API int32_t FetchResults(PxSceneHandle s_, int32_t block) {
    return reinterpret_cast<PxScene*>(s_)->fetchResults(block != 0) ? 1 : 0;
}

API void ReleaseScene(PxSceneHandle s_) {
    reinterpret_cast<PxScene*>(s_)->release();
}

API void SceneAddActor(PxSceneHandle s_, PxActorHandle a_) {
    reinterpret_cast<PxScene*>(s_)->addActor(*reinterpret_cast<PxActor*>(a_));
}

API PxActorHandle CreateRigidStatic(
    PxPhysicsHandle p_,
    float px, float py, float pz,
    float qx, float qy, float qz, float qw)
{
    PxPhysics* p = reinterpret_cast<PxPhysics*>(p_);
    PxTransform t(PxVec3(px, py, pz), PxQuat(qx, qy, qz, qw));
    return p->createRigidStatic(t);
}

API PxActorHandle CreateRigidDynamic(
    PxPhysicsHandle p_,
    float px, float py, float pz,
    float qx, float qy, float qz, float qw)
{
    PxPhysics* p = reinterpret_cast<PxPhysics*>(p_);
    PxTransform t(PxVec3(px, py, pz), PxQuat(qx, qy, qz, qw));
    return p->createRigidDynamic(t);
}

API void ReleaseActor(PxActorHandle a_) {
    reinterpret_cast<PxActor*>(a_)->release();
}

API PxShapeHandle CreateShapeBox(
    PxActorHandle     a_,
    PxMaterialHandle  m_,
    float hx, float hy, float hz)
{
    PxRigidActor* actor = reinterpret_cast<PxRigidActor*>(a_);
    PxMaterial* material = reinterpret_cast<PxMaterial*>(m_);
    return PxRigidActorExt::createExclusiveShape(*actor, PxBoxGeometry(hx, hy, hz), *material);
}

API PxShapeHandle CreateShapeSphere(
    PxActorHandle     a_,
    PxMaterialHandle  m_,
    float radius)
{
    PxRigidActor* actor = reinterpret_cast<PxRigidActor*>(a_);
    PxMaterial* material = reinterpret_cast<PxMaterial*>(m_);
    return PxRigidActorExt::createExclusiveShape(*actor, PxSphereGeometry(radius), *material);
}

API PxShapeHandle CreateShapeCapsule(
    PxActorHandle     a_,
    PxMaterialHandle  m_,
    float radius, float halfHeight)
{
    PxRigidActor* actor = reinterpret_cast<PxRigidActor*>(a_);
    PxMaterial* material = reinterpret_cast<PxMaterial*>(m_);
    return PxRigidActorExt::createExclusiveShape(*actor, PxCapsuleGeometry(radius, halfHeight), *material);
}

API PxShapeHandle CreateShapeTriangleMesh(
    PxActorHandle        a_,
    PxMaterialHandle     m_,
    PxTriangleMeshHandle tm_)
{
    PxRigidActor* actor = reinterpret_cast<PxRigidActor*>(a_);
    PxMaterial* material = reinterpret_cast<PxMaterial*>(m_);
    PxTriangleMesh* mesh = reinterpret_cast<PxTriangleMesh*>(tm_);
    return PxRigidActorExt::createExclusiveShape(*actor, PxTriangleMeshGeometry(mesh), *material);
}

API PxShapeHandle CreateShapeConvexMesh(
    PxActorHandle      a_,
    PxMaterialHandle   m_,
    PxConvexMeshHandle cm_)
{
    PxRigidActor* actor = reinterpret_cast<PxRigidActor*>(a_);
    PxMaterial* material = reinterpret_cast<PxMaterial*>(m_);
    PxConvexMesh* mesh = reinterpret_cast<PxConvexMesh*>(cm_);
    return PxRigidActorExt::createExclusiveShape(*actor, PxConvexMeshGeometry(mesh), *material);
}

API void ReleaseShape(PxShapeHandle s_) {
    reinterpret_cast<PxShape*>(s_)->release();
}

API void SetRigidBodyFlag(
    PxActorHandle a_,
    int32_t       flag,
    int32_t       value)
{
    reinterpret_cast<PxRigidBody*>(a_)
        ->setRigidBodyFlag(PxRigidBodyFlag::Enum(flag), value != 0);
}

API void SetGlobalPose(
    PxActorHandle a_,
    float px, float py, float pz,
    float qx, float qy, float qz, float qw)
{
    reinterpret_cast<PxRigidActor*>(a_)
        ->setGlobalPose(PxTransform(PxVec3(px, py, pz), PxQuat(qx, qy, qz, qw)));
}

API int32_t SceneOverlapCapsule(
    PxSceneHandle s_,
    float px, float py, float pz,
    float qx, float qy, float qz, float qw,
    float r, float hh,
    int32_t maxHits,
    float outNormals[][3],
    PxActorHandle outActors[])
{
    PxOverlapHit   bufHits[256];
    PxOverlapBuffer buf(bufHits, 256);
    reinterpret_cast<PxScene*>(s_)
        ->overlap(PxCapsuleGeometry(r, hh),
            PxTransform(PxVec3(px, py, pz), PxQuat(qx, qy, qz, qw)),
            buf, PxQueryFilterData());
    PxU32 n = buf.getNbTouches();
    for (PxU32 i = 0;i < n && i < (PxU32)maxHits;i++) {
        outNormals[i][0] = outNormals[i][1] = outNormals[i][2] = 0.0f;
        outActors[i] = buf.getTouches()[i].actor;
    }
    return int32_t(n);
}

API int32_t SceneSweepCapsule(
    PxSceneHandle s_,
    float px, float py, float pz,
    float qx, float qy, float qz, float qw,
    float r, float hh,
    float dx, float dy, float dz, float dist,
    int32_t maxHits,
    float outPoints[][3],
    float outNormals[][3],
    PxActorHandle outActors[])
{
    PxSweepHit   bufHits[256];
    PxSweepBuffer buf(bufHits, 256);
    reinterpret_cast<PxScene*>(s_)
        ->sweep(PxCapsuleGeometry(r, hh),
            PxTransform(PxVec3(px, py, pz), PxQuat(qx, qy, qz, qw)),
            PxVec3(dx, dy, dz), dist,
            buf,
            PxHitFlag::ePOSITION | PxHitFlag::eNORMAL,
            PxQueryFilterData());
    PxU32 n = buf.getNbTouches();
    for (PxU32 i = 0;i < n && i < (PxU32)maxHits;i++) {
        auto& h = buf.getTouches()[i];
        outPoints[i][0] = h.position.x; outPoints[i][1] = h.position.y; outPoints[i][2] = h.position.z;
        outNormals[i][0] = h.normal.x;   outNormals[i][1] = h.normal.y;   outNormals[i][2] = h.normal.z;
        outActors[i] = h.actor;
    }
    return int32_t(n);
}

API int32_t SceneRaycast(
    PxSceneHandle s_,
    float ox, float oy, float oz,
    float dx, float dy, float dz,
    float maxDistance,
    float outPoint[3],
    float outNormal[3],
    PxActorHandle* outActor)
{
    PxRaycastBuffer buf;
    bool ok = reinterpret_cast<PxScene*>(s_)
        ->raycast(PxVec3(ox, oy, oz), PxVec3(dx, dy, dz),
            maxDistance, buf,
            PxHitFlag::ePOSITION | PxHitFlag::eNORMAL,
            PxQueryFilterData());
    if (!ok || !buf.hasBlock) return 0;
    auto& h = buf.block;
    outPoint[0] = h.position.x; outPoint[1] = h.position.y; outPoint[2] = h.position.z;
    outNormal[0] = h.normal.x;   outNormal[1] = h.normal.y;   outNormal[2] = h.normal.z;
    *outActor = h.actor;
    return 1;
}

API int32_t ComputePenetration(
    float ax, float ay, float az,
    float aqx, float aqy, float aqz, float aqw,
    float ar, float ah,
    float bx, float by, float bz,
    float bqx, float bqy, float bqz, float bqw,
    float br, float bh,
    float outDir[3],
    float* outDepth)
{
    PxCapsuleGeometry A(ar, ah), B(br, bh);
    PxTransform TA(PxVec3(ax, ay, az), PxQuat(aqx, aqy, aqz, aqw));
    PxTransform TB(PxVec3(bx, by, bz), PxQuat(bqx, bqy, bqz, bqw));
    PxVec3 dir; float depth;
    if (!PxGeometryQuery::computePenetration(dir, depth, A, TA, B, TB))
        return 0;
    outDir[0] = dir.x; outDir[1] = dir.y; outDir[2] = dir.z;
    *outDepth = depth;
    return 1;
}

API void GetGlobalPose(
    PxActorHandle a_,
    float* px, float* py, float* pz,
    float* qx, float* qy, float* qz, float* qw)
{
    PxRigidActor* actor = reinterpret_cast<PxRigidActor*>(a_);
    PxTransform t = actor->getGlobalPose();
    *px = t.p.x; *py = t.p.y; *pz = t.p.z;
    *qx = t.q.x; *qy = t.q.y; *qz = t.q.z; *qw = t.q.w;
}

API int32_t GetShapeCount(PxActorHandle a_)
{
    PxRigidActor* actor = reinterpret_cast<PxRigidActor*>(a_);
    return static_cast<int32_t>(actor->getNbShapes());
}

API int32_t GetShapes(
    PxActorHandle a_,
    PxShapeHandle* outShapes,
    int32_t bufferSize)
{
    PxRigidActor* actor = reinterpret_cast<PxRigidActor*>(a_);
    PxU32 fetched = actor->getShapes(
        reinterpret_cast<PxShape**>(outShapes),
        static_cast<PxU32>(bufferSize));
    return static_cast<int32_t>(fetched);
}

API PxCpuDispatcherHandle CreateCpuDispatcher(int32_t numThreads)
{
    return PxDefaultCpuDispatcherCreate(static_cast<PxU32>(numThreads));
}

API void ReleaseCpuDispatcher(PxCpuDispatcherHandle d_)
{
    delete reinterpret_cast<PxDefaultCpuDispatcher*>(d_);
}

API void SceneSetSimulationEventCallback(
    PxSceneHandle s_,
    PxSimulationEventCallbackHandle cb_)
{
    reinterpret_cast<PxScene*>(s_)->setSimulationEventCallback(
        reinterpret_cast<PxSimulationEventCallback*>(cb_));
}

API int32_t SceneOverlapCapsuleFiltered(
    PxSceneHandle s_,
    float px, float py, float pz,
    float qx, float qy, float qz, float qw,
    float r, float hh,
    uint32_t group, uint32_t mask,
    int32_t maxHits,
    float outNormals[][3],
    PxActorHandle outActors[])
{
    PxOverlapHit   hitBuf[256];
    PxOverlapBuffer buf(hitBuf, 256);

    PxFilterData       fd(group, mask, 0, 0);
    PxQueryFilterData  qfd(fd, PxQueryFlags(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC));

    reinterpret_cast<PxScene*>(s_)->overlap(
        PxCapsuleGeometry(r, hh),
        PxTransform(PxVec3(px, py, pz), PxQuat(qx, qy, qz, qw)),
        buf,
        qfd
    );

    PxU32 n = buf.getNbTouches();
    for (PxU32 i = 0; i < n && i < (PxU32)maxHits; ++i) {
        outNormals[i][0] = outNormals[i][1] = outNormals[i][2] = 0.0f;
        outActors[i] = buf.getTouches()[i].actor;
    }
    return int32_t(n);
}

API int32_t SceneSweepCapsuleFiltered(
    PxSceneHandle s_,
    float px, float py, float pz,
    float qx, float qy, float qz, float qw,
    float r, float hh,
    float dx, float dy, float dz,
    float dist,
    uint32_t group, uint32_t mask,
    int32_t maxHits,
    float outHitPoints[][3],
    float outHitNormals[][3],
    PxActorHandle outActors[])
{
    PxSweepHit     hitBuf[256];
    PxSweepBuffer  buf(hitBuf, 256);

    PxFilterData       fd(group, mask, 0, 0);
    PxQueryFilterData  qfd(fd, PxQueryFlags(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC));

    reinterpret_cast<PxScene*>(s_)->sweep(
        PxCapsuleGeometry(r, hh),
        PxTransform(PxVec3(px, py, pz), PxQuat(qx, qy, qz, qw)),
        PxVec3(dx, dy, dz),
        dist,
        buf,
        PxHitFlag::ePOSITION | PxHitFlag::eNORMAL,
        qfd
    );

    PxU32 n = buf.getNbTouches();
    for (PxU32 i = 0; i < n && i < (PxU32)maxHits; ++i) {
        auto& h = buf.getTouches()[i];
        outHitPoints[i][0] = h.position.x;
        outHitPoints[i][1] = h.position.y;
        outHitPoints[i][2] = h.position.z;
        outHitNormals[i][0] = h.normal.x;
        outHitNormals[i][1] = h.normal.y;
        outHitNormals[i][2] = h.normal.z;
        outActors[i] = h.actor;
    }
    return int32_t(n);
}

API int32_t SceneRaycastFiltered(
    PxSceneHandle s_,
    float ox, float oy, float oz,
    float dx, float dy, float dz,
    float maxDistance,
    uint32_t group, uint32_t mask,
    float outHitPoint[3],
    float outHitNormal[3],
    PxActorHandle* outActor)
{
    PxRaycastBuffer buf;
    PxFilterData      fd(group, mask, 0, 0);
    PxQueryFilterData qfd(fd, PxQueryFlags(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC));
    bool ok = reinterpret_cast<PxScene*>(s_)->raycast(
        PxVec3(ox, oy, oz),
        PxVec3(dx, dy, dz),
        maxDistance,
        buf,
        PxHitFlag::ePOSITION | PxHitFlag::eNORMAL,
        qfd
    );
    if (!ok || !buf.hasBlock) {
        return 0;
    }
    auto& h = buf.block;
    outHitPoint[0] = h.position.x;
    outHitPoint[1] = h.position.y;
    outHitPoint[2] = h.position.z;
    outHitNormal[0] = h.normal.x;
    outHitNormal[1] = h.normal.y;
    outHitNormal[2] = h.normal.z;
    *outActor = h.actor;
    return 1;
}

API void RegisterCollisionEnterCallback(PxCollisionCallback cb) { g_enterCb = cb; }
API void RegisterCollisionStayCallback(PxCollisionCallback cb)  { g_stayCb  = cb; }
API void RegisterCollisionExitCallback(PxCollisionCallback cb)  { g_exitCb  = cb; }

struct LocalSimulationEventCallback : PxSimulationEventCallback
{
    void onAdvance(
        const PxRigidBody* const* /*bodyBuffer*/,
        const PxTransform*        /*poseBuffer*/,
        PxU32                      /*count*/
    ) override {
    }

    void onContact(
        const PxContactPairHeader& hdr,
        const PxContactPair*       pairs,
        PxU32                      nbPairs
    ) override {
        if (!g_enterCb && !g_stayCb && !g_exitCb) return;

        for (PxU32 i = 0; i < nbPairs; ++i) {
            const auto& pair = pairs[i];
            auto a0 = reinterpret_cast<PxActorHandle>(hdr.actors[0]);
            auto a1 = reinterpret_cast<PxActorHandle>(hdr.actors[1]);

            if (g_enterCb && (pair.events & PxPairFlag::eNOTIFY_TOUCH_FOUND))
                g_enterCb(a0, a1);

            if (g_stayCb  && (pair.events & PxPairFlag::eNOTIFY_TOUCH_PERSISTS))
                g_stayCb(a0, a1);

            if (g_exitCb  && (pair.events & PxPairFlag::eNOTIFY_TOUCH_LOST))
                g_exitCb(a0, a1);
        }
    }

    void onTrigger(PxTriggerPair* pairs, PxU32 count) override
    {
        if (!g_triggerCb)
            return;

        for (PxU32 i = 0; i < count; ++i)
        {
            bool entered = (pairs[i].status == PxPairFlag::eNOTIFY_TOUCH_FOUND);
            g_triggerCb(
                reinterpret_cast<PxActorHandle>(pairs[i].triggerActor),
                reinterpret_cast<PxActorHandle>(pairs[i].otherActor),
                entered
            );
        }
    }




    void onConstraintBreak(PxConstraintInfo* /*constraints*/, PxU32 /*count*/) override {}

    void onWake(PxActor** /*actors*/, PxU32 /*count*/) override {}

    void onSleep(PxActor** /*actors*/, PxU32 /*count*/) override {}
};


API PxSimulationEventCallbackHandle CreateSimulationEventCallback() {
    return reinterpret_cast<PxSimulationEventCallbackHandle>(new LocalSimulationEventCallback());
}

API void ReleaseSimulationEventCallback(PxSimulationEventCallbackHandle cb)
{
    delete reinterpret_cast<LocalSimulationEventCallback*>(cb);
}

API void SetShapeFilterData(
    PxShapeHandle s_, uint32_t w0, uint32_t w1, uint32_t w2, uint32_t w3)
{
    PxShape* shape = reinterpret_cast<PxShape*>(s_);
    shape->setSimulationFilterData(PxFilterData(w0, w1, w2, w3));
    shape->setQueryFilterData(PxFilterData(w0, w1, w2, w3));
}

API void SetShapeFlag(PxShapeHandle s_, int32_t flag, int32_t value) {
    auto* shape = reinterpret_cast<PxShape*>(s_);
    shape->setFlag(PxShapeFlag::Enum(flag), value != 0);
}

API void SetShapeFlags(PxShapeHandle s_, uint32_t flags)
{
    auto* shape = reinterpret_cast<PxShape*>(s_);
    shape->setFlags(PxShapeFlags(static_cast<PxShapeFlag::Enum>(flags)));
}

API void SetFilterLayerCount(uint32_t count)
{
    g_layerCount = count;
    g_layerMasks.assign(count, 0xFFFFFFFFu);
}

API void SetFilterLayerMask(uint32_t layer, uint32_t mask)
{
    if (layer < g_layerCount)
        g_layerMasks[layer] = mask;
}

static PxFilterFlags layerFilterShader(
    PxFilterObjectAttributes a0, PxFilterData d0,
    PxFilterObjectAttributes a1, PxFilterData d1,
    PxPairFlags&             pairFlags,
    const void*              /*constantBlock*/,
    PxU32                    /*constantBlockSize*/
){
    if (PxFilterObjectIsTrigger(a0) || PxFilterObjectIsTrigger(a1)) {
        pairFlags = PxPairFlag::eTRIGGER_DEFAULT | PxPairFlag::eNOTIFY_TOUCH_LOST;
        return PxFilterFlag::eDEFAULT;
    }

    uint32_t layerA = d0.word0, layerB = d1.word0;
    if (layerA >= g_layerCount || layerB >= g_layerCount ||
        !(g_layerMasks[layerA] & (1u << layerB)))
    {
        return PxFilterFlag::eSUPPRESS;
    }

    bool isKinematicPair = PxFilterObjectIsKinematic(a0) || PxFilterObjectIsKinematic(a1);

    if (isKinematicPair) {
        pairFlags = PxPairFlag::eCONTACT_DEFAULT
                  | PxPairFlag::eNOTIFY_TOUCH_FOUND
                  | PxPairFlag::eNOTIFY_TOUCH_PERSISTS
                  | PxPairFlag::eNOTIFY_TOUCH_LOST
                  | PxPairFlag::eDETECT_CCD_CONTACT;  // Possibly important for kinematic bodies
    } else {
        pairFlags = PxPairFlag::eCONTACT_DEFAULT
                  | PxPairFlag::eNOTIFY_TOUCH_FOUND
                  | PxPairFlag::eNOTIFY_TOUCH_PERSISTS
                  | PxPairFlag::eNOTIFY_TOUCH_LOST;
    }

    return PxFilterFlag::eDEFAULT;
}

API PxSimulationFilterShader GetLayerFilterShader()
{
    return layerFilterShader;
}

API PxSceneHandle CreateSceneWithLayerFilter(
    PxPhysicsHandle phys_,
    float           gx, float gy, float gz,
    int32_t         numThreads
){
    PxPhysics* p = reinterpret_cast<PxPhysics*>(phys_);
    PxSceneDesc desc(p->getTolerancesScale());
    desc.gravity       = PxVec3(gx, gy, gz);
    desc.cpuDispatcher = PxDefaultCpuDispatcherCreate(static_cast<PxU32>(numThreads));
    desc.filterShader  = layerFilterShader;

    return p->createScene(desc);
}

API void SetShapeSimulationFilterData(
    PxShapeHandle shape_,
    uint32_t      w0, uint32_t w1,
    uint32_t      w2, uint32_t w3
){
    PxShape* shape = reinterpret_cast<PxShape*>(shape_);
    shape->setSimulationFilterData(PxFilterData(w0, w1, w2, w3));
}

API void SetShapeQueryFilterData(
    PxShapeHandle shape_,
    uint32_t      w0, uint32_t w1,
    uint32_t      w2, uint32_t w3
){
    PxShape* shape = reinterpret_cast<PxShape*>(shape_);
    shape->setQueryFilterData(PxFilterData(w0, w1, w2, w3));
}

API void SetKinematicTarget(
    PxActorHandle dyn_,
    float px, float py, float pz,
    float qx, float qy, float qz, float qw
) {
    auto* rd = reinterpret_cast<PxRigidDynamic*>(dyn_);
    rd->setKinematicTarget(
      PxTransform(PxVec3(px,py,pz), PxQuat(qx,qy,qz,qw))
    );
}

API void SceneFlushQueryUpdates(PxSceneHandle s_) {
    reinterpret_cast<PxScene*>(s_)->flushQueryUpdates();
}

API void PxActor_DetachShape(
    PxActorHandle actorHandle,
    PxShapeHandle shapeHandle
) {
    PxRigidActor* rigidActor = reinterpret_cast<PxRigidActor*>(actorHandle);
    PxShape* shape = reinterpret_cast<PxShape*>(shapeHandle);

    if (rigidActor && shape) {
        rigidActor->detachShape(*shape);
    }
}

inline PxRigidDynamic* GetRigidDynamic(PxActorHandle actorHandle) {
    PxActor* actor = reinterpret_cast<PxActor*>(actorHandle);
    if (actor && actor->getConcreteType() == PxConcreteType::eRIGID_DYNAMIC) {
        return static_cast<PxRigidDynamic*>(actor);
    }
    return nullptr;
}

API void PxRigidDynamic_WakeUp(PxActorHandle actorHandle) {
    PxRigidDynamic* rigidDynamic = GetRigidDynamic(actorHandle);
    if (rigidDynamic) {
        rigidDynamic->wakeUp();
    }
}

API void PxRigidDynamic_PutToSleep(PxActorHandle actorHandle) {
    PxRigidDynamic* rigidDynamic = GetRigidDynamic(actorHandle);
    if (rigidDynamic) {
        rigidDynamic->putToSleep();
    }
}

API int32_t PxRigidDynamic_IsSleeping(PxActorHandle actorHandle) {
    PxRigidDynamic* rigidDynamic = GetRigidDynamic(actorHandle);
    if (rigidDynamic) {
        return rigidDynamic->isSleeping() ? 1 : 0;
    }
    return 0;
}

API void PxRigidDynamic_SetWakeCounter(PxActorHandle actorHandle, float wakeCounterValue) {
    PxRigidDynamic* rigidDynamic = GetRigidDynamic(actorHandle);
    if (rigidDynamic) {
        rigidDynamic->setWakeCounter(wakeCounterValue);
    }
}
API void PxRigidDynamic_GetWakeCounter(PxActorHandle actorHandle, float* outValue) {
    PxRigidDynamic* rigidDynamic = GetRigidDynamic(actorHandle);
    if (rigidDynamic && outValue) {
        *outValue = static_cast<float>(rigidDynamic->getWakeCounter());
    } else if (outValue) {
        *outValue = 0.0f;
    }
}
