#include <iostream>
#include <unistd.h>
#include <math.h>
#include <libplayerc++/playerc++.h>
#include "RobotMover.h"

using namespace PlayerCc;



int main(int argc, char *argv[])
{
    using namespace PlayerCc;

    PlayerClient    robot(argv[1]);
    RangerProxy      sp(&robot, 0);
    Position2dProxy pp(&robot, 0);
   
    pid_bundle drive_coeffs = {0.1, 0, 0, 0};
    pid_bundle steer_coeffs = {1, 0.1, 0, 0};

    RobotMover* mover = new RobotMover( drive_coeffs, steer_coeffs, &robot, &pp);

    pp.SetMotorEnable(true);

    double angle = 0;
    
    mover->set_target( 10, 10 );

    while (true) {
    	mover->move();
    }
}


