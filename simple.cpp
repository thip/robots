/*
 * Copyright (c) 2005, Brad Kratochvil, Toby Collett, Brian Gerkey, Andrew Howard, ...
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright notice,
 *           this list of conditions and the following disclaimer.
 *               * Redistributions in binary form must reproduce the above copyright notice,
 *                     this list of conditions and the following disclaimer in the documentation
 *                           and/or other materials provided with the distribution.
 *                               * Neither the name of the Player Project nor the names of its contributors
 *                                     may be used to endorse or promote products derived from this software
 *                                           without specific prior written permission.
 *
 *                                           THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *                                           ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *                                           WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *                                           DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 *                                           ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *                                           (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *                                           LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *                                           ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *                                           (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *                                           SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *                                           */

#include <iostream>
#include <unistd.h>
#include <math.h>
#include <libplayerc++/playerc++.h>



using namespace PlayerCc;

double getAngleDiff(double angle1, double angle2){
  double a = angle2  - angle1;
         a = fmod( (a + 180), 360 ) - 180;


  return a;
}

int turn_to( double targetAngle, PlayerClient* robot, Position2dProxy* pp){
  robot->Read();
  
  double worldAngle;
  double prop = 99.9;
  
  while ( prop > 0.05 || prop < -0.05){
  
    robot->Read();

    worldAngle = rtod( pp->GetYaw() );
    prop = getAngleDiff( worldAngle, targetAngle );

    std::cout << prop << std::endl;

    pp->SetSpeed(0.00 ,dtor( 0.5 * prop));
    sleep(1); 
  }

  return 0;
}


int move_distance( double target_distance, 
    PlayerClient* robot, 
    Position2dProxy* pp){

  robot->Read();
  
  double startX = pp->GetXPos();
  double startY = pp->GetYPos();

  double currentX, currentY;

  double deltaX, deltaY;

  double distance;

  double prop = 99.9;
  while(prop > 0.01){
  
   robot->Read();
   
    currentX = pp->GetXPos();
    currentY = pp->GetYPos();

    deltaX = currentX - startX;
    deltaY = currentY - startY;

    distance = sqrt(pow(deltaX,2) +pow( deltaY,2));
    
    prop = target_distance - distance;

    pp->SetSpeed(0.3 * prop, 0);  
    std::cout << prop  << std::endl;
    sleep(1); 
  }

  return 0;
}

int main(int argc, char *argv[])
{
	using namespace PlayerCc;

	PlayerClient    robot("lisa.islnet");
	RangerProxy      sp(&robot,0);
	Position2dProxy pp(&robot,0);

	pp.SetMotorEnable(true);

  double angle = 0;
  turn_to ( angle, &robot, &pp);
  

  while (true) {
    move_distance( 2, &robot, &pp);
    angle = fmod(angle + 90, 360);
    turn_to( angle, &robot, &pp);
  }
}


