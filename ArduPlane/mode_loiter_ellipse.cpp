#include "mode.h"
#include "Plane.h"

bool ModeLoiter_Ellipse::_enter()
{
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;
    plane.do_loiter_ellipse();

    return true;
}

void ModeLoiter_Ellipse::update()
{
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}

