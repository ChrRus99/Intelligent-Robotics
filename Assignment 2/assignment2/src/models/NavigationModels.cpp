#include "models/NavigationModels.h"

using namespace Point;

namespace NavigationModels {
    /* FIXED_NAVIGATION_WAY_POINT*/
    /* Fixed waypoints definition */
    const Pose FixedNavigationWayPoint::START_POSE{0.0, 0.0, 0.0};
    const Position FixedNavigationWayPoint::PICK_UP_AREA_POSITION{8.0, 0.0};
    const Position FixedNavigationWayPoint::PICK_UP_TABLE_LEFT_POSITION{9.0, -4.1};
    const Position FixedNavigationWayPoint::PICK_UP_TABLE_RIGHT_POSITION{9.0, -2.1}; // {9.0, -1.9}
    const Pose FixedNavigationWayPoint::DELIVERY_AREA_POSE{11.5, -2.0, 95.0};
}