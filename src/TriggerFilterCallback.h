#ifndef TRIGGERFILTERCALLBACK_H
#define TRIGGERFILTERCALLBACK_H
#include "PxQueryFiltering.h"
#include "PxShape.h"

using namespace physx;


class TriggerFilterCallback : public PxQueryFilterCallback {
public:
    enum TriggerInteraction {
        T_USE_GLOBAL_OR_COLLIDE = 0,
        T_IGNORE = 1,
        T_COLLIDE = 2
    };

    TriggerInteraction triggerMode;

    TriggerFilterCallback(TriggerInteraction mode) : triggerMode(mode) {}

    virtual PxQueryHitType::Enum preFilter(
        const PxFilterData& filterData,
        const PxShape* shape,
        const PxRigidActor* actor,
        PxHitFlags& queryHitFlags)
    {

        PxShapeFlags shapeFlags = shape->getFlags();
        bool isTrigger = shapeFlags & PxShapeFlag::eTRIGGER_SHAPE;

        if (isTrigger) {
            if (triggerMode == T_IGNORE) {
                return PxQueryHitType::eNONE;
            }
            queryHitFlags = PxHitFlag::eDEFAULT;
            return PxQueryHitType::eTOUCH;
        }

        queryHitFlags = PxHitFlag::eDEFAULT;
        return PxQueryHitType::eNONE;
    }

    virtual PxQueryHitType::Enum postFilter(const PxFilterData& filterData, const PxQueryHit& hit) {
        return PxQueryHitType::eNONE;
    }
};



#endif //TRIGGERFILTERCALLBACK_H
