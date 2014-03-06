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

double get_heading_between ( double x1, double y1, double x2, double y2 ){
    double delta_x = x2 - x1;
    double delta_y = y2 - y1;

    double rads = atan2( delta_x,  delta_y);

    return fmod (rtod(rads) + 360, 360) ;

}


double distance_between( double x1, double y1, double x2, double y2 ){
	double delta_x = x2 - x1;
	double delta_y = y2 - y1;
	return sqrt( pow( delta_x, 2 ) + pow( delta_y, 2));
}

typedef struct pid_bundle{
    double p_coef;
    double i_coef;
    double d_coef;
    double i_decay;
} pid_bundle;


class PIDController{
    private:
        double p, i, d;
        pid_bundle coeffs;
    public:

        PIDController(pid_bundle coeffs){
            this->coeffs = coeffs;
        }

        void update(double delta){
            p = delta;
            i = (i+p)*coeffs.decay;
            d = 0
        }

        double output(){
            return p * coeffs.p_coeff + i * coeffs.i_coeff + d * coeffs.d_coeff;
        }


class RobotMover{
	private:
		PlayerClient* robot;
		Position2dProxy* position_proxy;

        PIDController drive_pid, steer_pid;

        double x_target, y_target, x_start, y_start;
		
        bool in_front( double x, double y ){
            double world_heading = get_heading_between ( position_proxy->GetXPos(), position_proxy->GetYPos(), x_target, y_target );
            double local_heading = getAngleDiff( position_proxy->GetYaw(), world_heading );
            
            if (local_heading > -90 || local_heading < 90){
                return true;
            } else { 
                return false;
            }
        }


	public:
		RobotMover( pid_bundle drive_coeffs, pid_bundle steer_coeffs,
                    PlayerClient* robot, Position2dProxy* position_proxy ){
            this->robot = robot;
            this->position_proxy = position_proxy;

            drive_pid = new PIDController(drive_coeffs);
            steer_pid = new PIDController(steer_coeffs);
		}
		
        void set_target( double x_target, double y_target ){
            
          robot->Read();
          
            x_start = position_proxy->GetXPos();
            x_start = position_proxy->GetYPos();

            
            this->x_target = x_target + x_start;
            this->y_target = y_target + y_start;
        }


        void move() {
            
            

            robot->Read();

            double drive_delta = distance_between(position_proxy->GetXPos(), position_proxy->GetYPos(),
                                 x_target, y_target);
            
        
            if ( !in_front(x_target, y_target) ) drive_delta *= -1;



            double world_heading = get_heading_between ( position_proxy->GetXPos(), position_proxy->GetYPos(), x_target, y_target );
            double local_heading = getAngleDiff( rtod(position_proxy->GetYaw()), world_heading );

            //position_proxy->SetSpeed(0* 0.1 * p + 0.1 * i,0* 0.1*local_heading);  
            
            drive_pid.update(drive_delta);
            steer_pid.update(local_heading);


            position_proxy->SetSpeed(drive_pid.output(), steer_pid.output());  
            
            std::cout << p << std::endl;
        
        }
};

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

void go_to( double x, double y, 
	PlayerClient* robot, 
	Position2dProxy* pp){

	double p = 99.9;
	double i = 0;

	while(p > 0.2){
		i = i + p;

		robot->Read();

		p = distance_between( pp->GetXPos(), pp->GetYPos(), x, y);

		pp->SetSpeed(0.1 * p + 0.1 * i, 0);  
		std::cout << p << i << std::endl;
		sleep(1); 
	}
}



int main(int argc, char *argv[])
{
    using namespace PlayerCc;

    PlayerClient    robot(argv[1]);
    SonarProxy      sp(&robot, 0);
    Position2dProxy pp(&robot, 0);
    
    RobotMover* mover = new RobotMover( 0.1, 0, 0, &robot, &pp);

    pp.SetMotorEnable(true);

    double angle = 0;
    
    mover->set_target( 3, 1 );

    while (true) {
    	mover->move();
    }
}


