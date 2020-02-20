#include "mode.h"
#include "Plane.h"

bool ModeLoiter_3D::_enter()
{
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;
    plane.do_loiter_3d();

    return true;
}

void ModeLoiter_3D::update()
{
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}

