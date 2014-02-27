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
  double p = 99.9;
  double i = 0;
  double d = 0;
  double p_1 = 0;

  double pc = 0.6;
  double ic = 0.2;
  double dc = 0.0;
  
  while ( p > 0.002 || p < -0.002){
  
    robot->Read();

    worldAngle = rtod( pp->GetYaw() );
    p = getAngleDiff( worldAngle, targetAngle );

    d = p - p_1;
    p_1 = p;

    i = i + p;

    std::cout << "p: " <<  pc * p << " i: " << ic * i << std::endl;

    pp->SetSpeed(0, dtor( pc * p + ic * i + dc * d));
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

  double p = 99.9;
  double i = 0;

  while(p > 0.2){
      i = i + p;
  
      robot->Read();

      currentX = pp->GetXPos();
      currentY = pp->GetYPos();

      deltaX = currentX - startX;
      deltaY = currentY - startY;

      distance = sqrt(pow(deltaX,2) +pow( deltaY,2));

      p = target_distance - distance;

      pp->SetSpeed(0.1 * p + 0.1 * i, 0);  
      std::cout << p << i << std::endl;
      sleep(1); 
  }

  return 0;
}

void go_to( double x, double y ){
}

double distance_between( double x1, y1, x2, y2 ){
	double delta_x = x2 - x1;
	double delta_y = y2 - y1;
	return sqrt( pow( delta_x, 2 ) + pow( delta_y, 2);
}

void go_to( double x, double y, 
	PlayerClient* robot, 
	Position2dProxy* pp){

	double p = 99.9;
	double i = 0;

	while(p > 0.2){
		i = i + p;

		robot->Read();

		p = distance_between( pp->GetXPos(), pp->GetYPos(), x, y)

		pp->SetSpeed(0.1 * p + 0.1 * i, 0);  
		std::cout << p << i << std::endl;
		sleep(1); 
	}

	return 0;
}



int main(int argc, char *argv[])
{
    using namespace PlayerCc;

    PlayerClient    robot("lisa.islnet");
    SonarProxy      sp(&robot,0);
    Position2dProxy pp(&robot,0);

    pp.SetMotorEnable(true);

    double angle = 0;
    //turn_to ( angle, &robot, &pp);

    //angle = fmod(angle + 90, 360);
    //turn_to( angle, &robot, &pp);
    while (true) {
    	move_distance( 1, &robot, &pp);
    	angle = fmod(angle + 90, 360);
    	turn_to( angle, &robot, &pp);
    }
}


