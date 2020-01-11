#include "mode.h"
#include "Plane.h"

bool ModeEight_Plane::_enter()
{
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;
    plane.do_eight_plane();

    return true;
}

void ModeEight_Plane::update()
{
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}

