#include <stdlib.h>
#include <Ors/roboticsCourse.h>
#include <Gui/plot.h>

void holdSteady(){
  Simulator S("pegArm.ors");
//  Simulator S("pegArm2.ors");
  S.setDynamicSimulationNoise(1e-3);
  S.setDynamicGravity(true);
  uint n=S.getJointDimension();

  double tau=.01;
  uint T=1000;

  arr q,qdot;
  arr M,F,u(n);

  S.getJointAnglesAndVels(q, qdot);
  q = q+0.1;
  S.setJointAnglesAndVels(q,qdot);

  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  S.watch();

  for(uint t=0;t<=T;t++){
    S.getDynamics(M,F);

    // send zero torques
    u = 0.;




    // dynamic simulation (simple Euler integration of the system dynamics, look into the code)
    S.stepDynamics(u, tau);
    S.watch(false);
    S.getJointAnglesAndVels(q, qdot);

    cout  <<" t=" <<tau*t  <<"sec E=" <<S.getEnergy()  <<"  q = " <<q  <<endl;
  }
  S.watch();
}

int main(int argc,char **argv){
  holdSteady();
  return 0;
}
