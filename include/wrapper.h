// wrapper.h
#pragma once

#if defined(_WIN32) || defined(_WIN64)
  // CMake automatically defines PhysXWrapper_EXPORTS when compiling the PhysXWrapper target
  #ifdef PhysXWrapper_EXPORTS
    #define API __declspec(dllexport)
  #else
    #define API __declspec(dllimport)
  #endif
#else
  // non‐Windows platforms don’t need any decoration
  #define API
#endif

#include <stdint.h>

#include "PxFiltering.h"

#ifdef __cplusplus
extern "C" {
#endif
    // Opaque handles
    typedef void* PxFoundationHandle;
    typedef void* PxPhysicsHandle;
    typedef void* PxCookingHandle;
    typedef void* PxSceneHandle;
    typedef void* PxActorHandle;
    typedef void* PxMaterialHandle;
    typedef void* PxShapeHandle;
    typedef void* PxTriangleMeshHandle;
    typedef void* PxConvexMeshHandle;
    typedef void* PxCpuDispatcherHandle;
    typedef void* PxSimulationEventCallbackHandle;



    typedef void (*PxCollisionCallback)(PxActorHandle a0, PxActorHandle a1);

    typedef struct {
        float px, py, pz;
        float nx, ny, nz;
        float separation;
    } PxContactPoint;

    typedef void (*PxTriggerCallback)(PxActorHandle trigger, PxActorHandle other, bool entered);


    API PxFoundationHandle CreateFoundation();

    API void               ReleaseFoundation(PxFoundationHandle);

    API PxPhysicsHandle    CreatePhysics(PxFoundationHandle);

    API void               ReleasePhysics(PxPhysicsHandle);

    API PxCookingHandle    CreateCooking(PxFoundationHandle);

    API void               ReleaseCooking(PxCookingHandle);

    API int32_t            CookTriangleMesh(
        PxCookingHandle,
        const float* vertices, int32_t vertCount,
        const int32_t* indices, int32_t idxCount,
        void* outBuffer, int32_t bufferSize);

    API PxTriangleMeshHandle CreateTriangleMesh(
        PxPhysicsHandle,
        const void* cookedData, int32_t dataSize);

    API void               ReleaseTriangleMesh(PxTriangleMeshHandle);

    API int32_t            CookConvexMesh(
        PxCookingHandle,
        const float* vertices, int32_t vertCount,
        void* outBuffer, int32_t bufferSize);

    API PxConvexMeshHandle CreateConvexMesh(
        PxPhysicsHandle,
        const void* cookedData, int32_t dataSize);

    API void               ReleaseConvexMesh(PxConvexMeshHandle);

    API PxMaterialHandle   CreateMaterial(
        PxPhysicsHandle,
        float staticFriction,
        float dynamicFriction,
        float restitution);

    API void               ReleaseMaterial(PxMaterialHandle);

    API PxSceneHandle      CreateScene(
        PxPhysicsHandle,
        float gx, float gy, float gz,
        int32_t numThreads);

    API void               SimulateScene(
        PxSceneHandle,
        float deltaTime);

    API int32_t            FetchResults(
        PxSceneHandle,
        int32_t block);

    API void               ReleaseScene(PxSceneHandle);

    API void               SceneAddActor(
        PxSceneHandle,
        PxActorHandle);

    API PxActorHandle      CreateRigidStatic(
        PxPhysicsHandle,
        float px, float py, float pz,
        float qx, float qy, float qz, float qw);

    API PxActorHandle      CreateRigidDynamic(
        PxPhysicsHandle,
        float px, float py, float pz,
        float qx, float qy, float qz, float qw);

    API void               ReleaseActor(PxActorHandle);

    API PxShapeHandle      CreateShapeBox(
        PxActorHandle,
        PxMaterialHandle,
        float hx, float hy, float hz);

    API PxShapeHandle      CreateShapeSphere(
        PxActorHandle,
        PxMaterialHandle,
        float radius);

    API PxShapeHandle      CreateShapeCapsule(
        PxActorHandle,
        PxMaterialHandle,
        float radius,
        float halfHeight);

    API PxShapeHandle      CreateShapeTriangleMesh(
        PxActorHandle,
        PxMaterialHandle,
        PxTriangleMeshHandle);

    API PxShapeHandle      CreateShapeConvexMesh(
        PxActorHandle,
        PxMaterialHandle,
        PxConvexMeshHandle);

    API void               ReleaseShape(PxShapeHandle);

    API void PxActor_DetachShape(
        PxActorHandle actorHandle,
        PxShapeHandle shapeHandle
    );

    API void               SetRigidBodyFlag(
        PxActorHandle,
        int32_t flag,
        int32_t value);  // 0 or 1

    API void               SetGlobalPose(
        PxActorHandle,
        float px, float py, float pz,
        float qx, float qy, float qz, float qw);

    API int32_t            SceneOverlapCapsule(
        PxSceneHandle,
        float px, float py, float pz,
        float qx, float qy, float qz, float qw,
        float radius, float halfHeight,
        int32_t maxHits,
        float outNormals[][3],
        PxActorHandle outActors[]);

    API int32_t            SceneSweepCapsule(
        PxSceneHandle,
        float px, float py, float pz,
        float qx, float qy, float qz, float qw,
        float radius, float halfHeight,
        float dx, float dy, float dz,
        float distance,
        int32_t maxHits,
        float outHitPoints[][3],
        float outHitNormals[][3],
        PxActorHandle outActors[]);

    API int32_t            SceneRaycast(
        PxSceneHandle,
        float ox, float oy, float oz,
        float dx, float dy, float dz,
        float maxDistance,
        float outHitPoint[3],
        float outHitNormal[3],
        PxActorHandle* outActor);

    API int32_t            ComputePenetration(
        float ax, float ay, float az,
        float aqx, float aqy, float aqz, float aqw,
        float aRadius, float aHalfHeight,
        float bx, float by, float bz,
        float bqx, float bqy, float bqz, float bqw,
        float bRadius, float bHalfHeight,
        float outDirection[3],
        float* outPenetrationDepth);

    API void               GetGlobalPose(
        PxActorHandle,
        float* px, float* py, float* pz,
        float* qx, float* qy, float* qz, float* qw);

    API int32_t            GetShapeCount(PxActorHandle);

    API int32_t            GetShapes(
        PxActorHandle,
        PxShapeHandle* outShapes,
        int32_t bufferSize);

    API PxCpuDispatcherHandle CreateCpuDispatcher(int32_t numThreads);

    API void               ReleaseCpuDispatcher(PxCpuDispatcherHandle);

    API void               SceneSetSimulationEventCallback(
        PxSceneHandle,
        PxSimulationEventCallbackHandle);

    API int32_t            SceneOverlapCapsuleFiltered(
        PxSceneHandle,
        float px, float py, float pz,
        float qx, float qy, float qz, float qw,
        float radius, float halfHeight,
        uint32_t group, uint32_t mask,
        int32_t maxHits,
        float outNormals[][3],
        PxActorHandle outActors[]);

    API int32_t            SceneSweepCapsuleFiltered(
        PxSceneHandle,
        float px, float py, float pz,
        float qx, float qy, float qz, float qw,
        float radius, float halfHeight,
        float dx, float dy, float dz,
        float distance,
        uint32_t group, uint32_t mask,
        int32_t maxHits,
        float outHitPoints[][3],
        float outHitNormals[][3],
        PxActorHandle outActors[]);

    API int32_t            SceneRaycastFiltered(
        PxSceneHandle,
        float ox, float oy, float oz,
        float dx, float dy, float dz,
        float maxDistance,
        uint32_t group, uint32_t mask,
        float outHitPoint[3],
        float outHitNormal[3],
        PxActorHandle* outActor);

    API PxSimulationEventCallbackHandle CreateSimulationEventCallback();

    API void ReleaseSimulationEventCallback(PxSimulationEventCallbackHandle cb);

    API void SetShapeFilterData(
        PxShapeHandle shape,
        uint32_t word0, uint32_t word1,
        uint32_t word2, uint32_t word3);


    API void SetShapeFlag(PxShapeHandle s, int32_t flag, int32_t value);

    API void SetShapeFlags(PxShapeHandle shape, uint32_t flags);

    API void SetFilterLayerCount(uint32_t count);

    // For layer [0..count], set the 32‑bit bitmask of layers it collides with.
    API void SetFilterLayerMask(uint32_t layer, uint32_t mask);

    API physx::PxSimulationFilterShader GetLayerFilterShader();

    API PxSceneHandle CreateSceneWithLayerFilter(
        PxPhysicsHandle phys,
        float           gx, float gy, float gz,
        int32_t         numThreads
    );

    /// Instead of lumping sim+query together, set _only_ the simulation filters:
    API void SetShapeSimulationFilterData(
        PxShapeHandle shape,
        uint32_t      word0, uint32_t word1,
        uint32_t      word2, uint32_t word3
    );

    /// Likewise, set _only_ the query filters on the shape:
    API void SetShapeQueryFilterData(
        PxShapeHandle shape,
        uint32_t      word0, uint32_t word1,
        uint32_t      word2, uint32_t word3
    );

    // Called once when two shapes first begin contacting
    API void RegisterCollisionEnterCallback(PxCollisionCallback cb);

    API void RegisterCollisionStayCallback(PxCollisionCallback cb);

    // Called once when the shapes stop contacting
    API void RegisterCollisionExitCallback(PxCollisionCallback cb);

    API void RegisterTriggerCallback(PxTriggerCallback cb);

    API void SetKinematicTarget(
        PxActorHandle dyn_,
        float px, float py, float pz,
        float qx, float qy, float qz, float qw
    );

    API void SceneFlushQueryUpdates(PxSceneHandle s_);

    API void  PxRigidDynamic_WakeUp(PxActorHandle actorHandle);

    API void  PxRigidDynamic_PutToSleep(PxActorHandle actorHandle);

    API int32_t PxRigidDynamic_IsSleeping(PxActorHandle actorHandle);

    API void  PxRigidDynamic_SetWakeCounter(PxActorHandle actorHandle, float wakeCounterValue);

    API void PxRigidDynamic_GetWakeCounter(PxActorHandle actorHandle, float* outValue);

    API uint32_t GetFilterLayerMask(uint32_t layer);

    API void SetShapeLocalPose(
        PxShapeHandle s_,
        float px, float py, float pz,
        float qx, float qy, float qz, float qw
    );

    API void GetShapeLocalPose(
        PxShapeHandle s_,
        float* px, float* py, float* pz,
        float* qx, float* qy, float* qz, float* qw
    );

#ifdef __cplusplus
}
#endif
