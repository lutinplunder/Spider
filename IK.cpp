#include <iostream>
#include <cmath>
#include "FABRIK2D.h"
//#include "FABRIK2D.cpp"

using namespace std;

//Declaring global variables
double roll {0};
double pitch {0};
double yaw {0};
double bodyX {0};
double bodyY {0};
double bodyHeight {79.15};
double coxa1Angle {0};
double femur1Angle {0};
double tibia1Angle {0};
double leg1XYZ[3] {436.607, 192.864, 0};
double leg2XYZ[3] {436.607, -192.864, 0};
double leg3XYZ[3] {249.32, 401.35, 0};
double leg4XYZ[3] {249.32, -401.35, 0};
double leg5XYZ[3] {-226.421, 451.949, 0};
double leg6XYZ[3] {-226.421, -451.949, 0};
double leg7XYZ[3] {-485.512, 193.191, 0};
double leg8XYZ[3] {-485.512, -193.191, 0};
const double coxaLength = 62.2;// Length of coxa in mm
double lengths[3] {149.828, 148.971, 140.1};// Length of femur, tibia & tarsus in mm.
//extern double lengths

int main() {
    double initialBody1XYZ[3] = {77.058, 98.274, 0};
    double bodyIK1XYZ[3] = {0, 0, 0};
    double initialBody2XYZ[3] = {77.058, -98.274, 0};
    double bodyIK2XYZ[3] = {0, 0, 0};
    double initialBody3XYZ[3] = {0, 125.555, 0};
    double bodyIK3XYZ[3] = {0, 0, 0};
    double initialBody4XYZ[3] = {0, -125.555, 0};
    double bodyIK4XYZ[3] = {0, 0, 0};
    double initialBody5XYZ[3] = {-76.192, 111.869, 0};
    double bodyIK5XYZ[3] = {0, 0, 0};
    double initialBody6XYZ[3] = {-76.192, -111.869, 0};
    double bodyIK6XYZ[3] = {0, 0, 0};
    double initialBody7XYZ[3] = {-133.242, 74.327, 0};
    double bodyIK7XYZ[3] = {0, 0, 0};
    double initialBody8XYZ[3] = {-133.242, -74.327, 0};
    double bodyIK8XYZ[3] = {0, 0, 0};

    double coxa1IK[3] = {0, 0, 0};
    double coxa2IK[3] = {0, 0, 0};
    double coxa3IK[3] = {0, 0, 0};
    double coxa4IK[3] = {0, 0, 0};
    double coxa5IK[3] = {0, 0, 0};
    double coxa6IK[3] = {0, 0, 0};
    double coxa7IK[3] = {0, 0, 0};
    double coxa8IK[3] = {0, 0, 0};

    double leg1IKxz[2] {0, 0};
    double leg2IKxz[2] {0, 0};
    double leg3IKxz[2] {0, 0};
    double leg4IKxz[2] {0, 0};
    double leg5IKxz[2] {0, 0};
    double leg6IKxz[2] {0, 0};
    double leg7IKxz[2] {0, 0};
    double leg8IKxz[2] {0, 0};

    //double test[3] {0, 0, 0};

    //Inverse Kinematics
    //BodyIK
    //rpy&t of body1X
    bodyIK1XYZ[0] = ((initialBody1XYZ[0]*cos(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+initialBody1XYZ[1]*-cos(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+initialBody1XYZ[2]+sin(pitch*(M_PI/180)))+bodyX);
    //rpy&t of body1Y
    bodyIK1XYZ[1] = ((initialBody1XYZ[0]*(sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+cos(roll*(M_PI/180))*sin(yaw*(M_PI/180)))+initialBody1XYZ[1]*(sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+cos(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+initialBody1XYZ[2]*-sin(roll*(M_PI/180))*cos(pitch*(M_PI/180)))+bodyY);
    //rpy&t of body1Z
    bodyIK1XYZ[2] = ((initialBody1XYZ[0]*(cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+sin(roll*(M_PI/180))*sin(yaw*(M_PI/180)))+initialBody1XYZ[1]*(cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+sin(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+initialBody1XYZ[2]*cos(roll*(M_PI/180))*cos(pitch*(M_PI/180)))-bodyHeight);

    //rpy&t of body2X
    bodyIK2XYZ[0] = ((initialBody2XYZ[0]*cos(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+initialBody2XYZ[1]*-cos(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+initialBody2XYZ[2]+sin(pitch*(M_PI/180)))+bodyX);
    //rpy&t of body2Y
    bodyIK2XYZ[1] = ((initialBody2XYZ[0]*(sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+cos(roll*(M_PI/180))*sin(yaw*(M_PI/180)))+initialBody2XYZ[1]*(sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+cos(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+initialBody2XYZ[2]*-sin(roll*(M_PI/180))*cos(pitch*(M_PI/180)))+bodyY);
    //rpy&t of body2Z
    bodyIK2XYZ[2] = ((initialBody2XYZ[0]*(cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+sin(roll*(M_PI/180))*sin(yaw*(M_PI/180)))+initialBody2XYZ[1]*(cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+sin(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+initialBody2XYZ[2]*cos(roll*(M_PI/180))*cos(pitch*(M_PI/180)))-bodyHeight);

    //rpy&t of body3X
    bodyIK3XYZ[0] = ((initialBody3XYZ[0]*cos(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+initialBody3XYZ[1]*-cos(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+initialBody3XYZ[2]+sin(pitch*(M_PI/180)))+bodyX);
    //rpy&t of body3Y
    bodyIK3XYZ[1] = ((initialBody3XYZ[0]*(sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+cos(roll*(M_PI/180))*sin(yaw*(M_PI/180)))+initialBody3XYZ[1]*(sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+cos(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+initialBody3XYZ[2]*-sin(roll*(M_PI/180))*cos(pitch*(M_PI/180)))+bodyY);
    //rpy&t of body3Z
    bodyIK3XYZ[2] = ((initialBody3XYZ[0]*(cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+sin(roll*(M_PI/180))*sin(yaw*(M_PI/180)))+initialBody3XYZ[1]*(cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+sin(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+initialBody3XYZ[2]*cos(roll*(M_PI/180))*cos(pitch*(M_PI/180)))-bodyHeight);

    //rpy&t of body4X
    bodyIK4XYZ[0] = ((initialBody4XYZ[0]*cos(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+initialBody4XYZ[1]*-cos(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+initialBody4XYZ[2]+sin(pitch*(M_PI/180)))+bodyX);
    //rpy&t of body4Y
    bodyIK4XYZ[1] = ((initialBody4XYZ[0]*(sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+cos(roll*(M_PI/180))*sin(yaw*(M_PI/180)))+initialBody4XYZ[1]*(sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+cos(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+initialBody4XYZ[2]*-sin(roll*(M_PI/180))*cos(pitch*(M_PI/180)))+bodyY);
    //rpy&t of body4Z
    bodyIK4XYZ[2] = ((initialBody4XYZ[0]*(cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+sin(roll*(M_PI/180))*sin(yaw*(M_PI/180)))+initialBody4XYZ[1]*(cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+sin(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+initialBody4XYZ[2]*cos(roll*(M_PI/180))*cos(pitch*(M_PI/180)))-bodyHeight);

    //rpy&t of body5X
    bodyIK5XYZ[0] = ((initialBody5XYZ[0]*cos(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+initialBody5XYZ[1]*-cos(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+initialBody5XYZ[2]+sin(pitch*(M_PI/180)))+bodyX);
    //rpy&t of body5Y
    bodyIK5XYZ[1] = ((initialBody5XYZ[0]*(sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+cos(roll*(M_PI/180))*sin(yaw*(M_PI/180)))+initialBody5XYZ[1]*(sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+cos(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+initialBody5XYZ[2]*-sin(roll*(M_PI/180))*cos(pitch*(M_PI/180)))+bodyY);
    //rpy&t of body5Z
    bodyIK5XYZ[2] = ((initialBody5XYZ[0]*(cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+sin(roll*(M_PI/180))*sin(yaw*(M_PI/180)))+initialBody5XYZ[1]*(cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+sin(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+initialBody5XYZ[2]*cos(roll*(M_PI/180))*cos(pitch*(M_PI/180)))-bodyHeight);

    //rpy&t of body6X
    bodyIK6XYZ[0] = ((initialBody6XYZ[0]*cos(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+initialBody6XYZ[1]*-cos(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+initialBody6XYZ[2]+sin(pitch*(M_PI/180)))+bodyX);
    //rpy&t of body6Y
    bodyIK6XYZ[1] = ((initialBody1XYZ[0]*(sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+cos(roll*(M_PI/180))*sin(yaw*(M_PI/180)))+initialBody6XYZ[1]*(sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+cos(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+initialBody6XYZ[2]*-sin(roll*(M_PI/180))*cos(pitch*(M_PI/180)))+bodyY);
    //rpy&t of body6Z
    bodyIK6XYZ[2] = ((initialBody1XYZ[0]*(cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+sin(roll*(M_PI/180))*sin(yaw*(M_PI/180)))+initialBody6XYZ[1]*(cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+sin(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+initialBody6XYZ[2]*cos(roll*(M_PI/180))*cos(pitch*(M_PI/180)))-bodyHeight);

    //rpy&t of body7X
    bodyIK7XYZ[0] = ((initialBody7XYZ[0]*cos(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+initialBody7XYZ[1]*-cos(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+initialBody7XYZ[2]+sin(pitch*(M_PI/180)))+bodyX);
    //rpy&t of body7Y
    bodyIK7XYZ[1] = ((initialBody7XYZ[0]*(sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+cos(roll*(M_PI/180))*sin(yaw*(M_PI/180)))+initialBody7XYZ[1]*(sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+cos(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+initialBody7XYZ[2]*-sin(roll*(M_PI/180))*cos(pitch*(M_PI/180)))+bodyY);
    //rpy&t of body7Z
    bodyIK7XYZ[2] = ((initialBody7XYZ[0]*(cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+sin(roll*(M_PI/180))*sin(yaw*(M_PI/180)))+initialBody7XYZ[1]*(cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+sin(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+initialBody7XYZ[2]*cos(roll*(M_PI/180))*cos(pitch*(M_PI/180)))-bodyHeight);

    //rpy&t of body8X
    bodyIK8XYZ[0] = ((initialBody8XYZ[0]*cos(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+initialBody8XYZ[1]*-cos(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+initialBody8XYZ[2]+sin(pitch*(M_PI/180)))+bodyX);
    //rpy&t of body8Y
    bodyIK8XYZ[1] = ((initialBody8XYZ[0]*(sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+cos(roll*(M_PI/180))*sin(yaw*(M_PI/180)))+initialBody8XYZ[1]*(sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+cos(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+initialBody8XYZ[2]*-sin(roll*(M_PI/180))*cos(pitch*(M_PI/180)))+bodyY);
    //rpy&t of body8Z
    bodyIK8XYZ[2] = ((initialBody8XYZ[0]*(cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+sin(roll*(M_PI/180))*sin(yaw*(M_PI/180)))+initialBody8XYZ[1]*(cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+sin(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+initialBody8XYZ[2]*cos(roll*(M_PI/180))*cos(pitch*(M_PI/180)))-bodyHeight);



    //Coxa IK
    //rpy of Coxa1X
    coxa1IK[0] = ((coxaLength*cos(atan((leg1XYZ[1]-bodyIK1XYZ[1])/(leg1XYZ[0]-bodyIK1XYZ[0]))))+bodyIK1XYZ[0])*cos(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+((coxaLength*sin(atan((leg1XYZ[1]-bodyIK1XYZ[1])/(leg1XYZ[0]-bodyIK1XYZ[0]))))+bodyIK1XYZ[1])*-cos(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+(initialBody1XYZ[2])*sin(pitch*(M_PI/180));
    //rpy of Coxa1Y
    coxa1IK[1] = ((coxaLength*cos(atan((leg1XYZ[1]-bodyIK1XYZ[1])/(leg1XYZ[0]-bodyIK1XYZ[0]))))+bodyIK1XYZ[0])*(sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(roll*(M_PI/180))+cos(roll*(M_PI/180))*sin(yaw*(M_PI/180)))+((coxaLength*sin(atan((leg1XYZ[1]-bodyIK1XYZ[1])/(leg1XYZ[0]-bodyIK1XYZ[0]))))+bodyIK1XYZ[1])*(-sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+cos(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+(initialBody1XYZ[2])*-sin(roll*(M_PI/180))*cos(pitch*(M_PI/180));
    //rpy of Coxa1Z
    coxa1IK[2] = (((coxaLength*cos(atan((leg1XYZ[1]-bodyIK1XYZ[1])/(leg1XYZ[0]-bodyIK1XYZ[0]))))+bodyIK1XYZ[0])*(-cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+sin(roll*(M_PI/180))*sin(yaw*(M_PI/180)))+((coxaLength*sin(atan((leg1XYZ[1]-bodyIK1XYZ[1])/(leg1XYZ[0]-bodyIK1XYZ[0]))))+bodyIK1XYZ[1])*(cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+sin(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+(initialBody1XYZ[2])*cos(roll*(M_PI/180))*cos(pitch*(M_PI/180)))-bodyHeight;

    //rpy of Coxa2X
    coxa2IK[0] = ((coxaLength*cos(atan((leg2XYZ[1]-bodyIK2XYZ[1])/(leg2XYZ[0]-bodyIK2XYZ[0]))))+bodyIK2XYZ[0])*cos(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+((coxaLength*sin(atan((leg2XYZ[1]-bodyIK2XYZ[1])/(leg2XYZ[0]-bodyIK2XYZ[0]))))+bodyIK2XYZ[1])*-cos(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+(initialBody2XYZ[2])*sin(pitch*(M_PI/180));
    //rpy of Coxa2Y
    coxa2IK[1] = ((coxaLength*cos(atan((leg2XYZ[1]-bodyIK2XYZ[1])/(leg2XYZ[0]-bodyIK2XYZ[0]))))+bodyIK2XYZ[0])*(sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(roll*(M_PI/180))+cos(roll*(M_PI/180))*sin(yaw*(M_PI/180)))+((coxaLength*sin(atan((leg2XYZ[1]-bodyIK2XYZ[1])/(leg2XYZ[0]-bodyIK2XYZ[0]))))+bodyIK2XYZ[1])*(-sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+cos(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+(initialBody2XYZ[2])*-sin(roll*(M_PI/180))*cos(pitch*(M_PI/180));
    //rpy of Coxa2Z
    coxa2IK[2] = (((coxaLength*cos(atan((leg2XYZ[1]-bodyIK2XYZ[1])/(leg2XYZ[0]-bodyIK2XYZ[0]))))+bodyIK2XYZ[0])*(-cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+sin(roll*(M_PI/180))*sin(yaw*(M_PI/180)))+((coxaLength*sin(atan((leg2XYZ[1]-bodyIK2XYZ[1])/(leg2XYZ[0]-bodyIK2XYZ[0]))))+bodyIK2XYZ[1])*(cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+sin(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+(initialBody2XYZ[2])*cos(roll*(M_PI/180))*cos(pitch*(M_PI/180)))-bodyHeight;

    //rpy of Coxa3X
    coxa3IK[0] = ((coxaLength*cos(atan((leg3XYZ[1]-bodyIK3XYZ[1])/(leg3XYZ[0]-bodyIK3XYZ[0]))))+bodyIK3XYZ[0])*cos(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+((coxaLength*sin(atan((leg3XYZ[1]-bodyIK3XYZ[1])/(leg3XYZ[0]-bodyIK3XYZ[0]))))+bodyIK3XYZ[1])*-cos(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+(initialBody3XYZ[2])*sin(pitch*(M_PI/180));
    //rpy of Coxa3Y
    coxa3IK[1] = ((coxaLength*cos(atan((leg3XYZ[1]-bodyIK3XYZ[1])/(leg3XYZ[0]-bodyIK3XYZ[0]))))+bodyIK3XYZ[0])*(sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(roll*(M_PI/180))+cos(roll*(M_PI/180))*sin(yaw*(M_PI/180)))+((coxaLength*sin(atan((leg3XYZ[1]-bodyIK3XYZ[1])/(leg3XYZ[0]-bodyIK3XYZ[0]))))+bodyIK3XYZ[1])*(-sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+cos(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+(initialBody3XYZ[2])*-sin(roll*(M_PI/180))*cos(pitch*(M_PI/180));
    //rpy of Coxa3Z
    coxa3IK[2] = (((coxaLength*cos(atan((leg3XYZ[1]-bodyIK3XYZ[1])/(leg3XYZ[0]-bodyIK3XYZ[0]))))+bodyIK3XYZ[0])*(-cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+sin(roll*(M_PI/180))*sin(yaw*(M_PI/180)))+((coxaLength*sin(atan((leg3XYZ[1]-bodyIK3XYZ[1])/(leg3XYZ[0]-bodyIK3XYZ[0]))))+bodyIK3XYZ[1])*(cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+sin(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+(initialBody3XYZ[2])*cos(roll*(M_PI/180))*cos(pitch*(M_PI/180)))-bodyHeight;

    //rpy of Coxa4X
    coxa4IK[0] = ((coxaLength*cos(atan((leg4XYZ[1]-bodyIK4XYZ[1])/(leg4XYZ[0]-bodyIK4XYZ[0]))))+bodyIK4XYZ[0])*cos(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+((coxaLength*sin(atan((leg4XYZ[1]-bodyIK4XYZ[1])/(leg4XYZ[0]-bodyIK4XYZ[0]))))+bodyIK4XYZ[1])*-cos(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+(initialBody4XYZ[2])*sin(pitch*(M_PI/180));
    //rpy of Coxa4Y
    coxa4IK[1] = ((coxaLength*cos(atan((leg4XYZ[1]-bodyIK4XYZ[1])/(leg4XYZ[0]-bodyIK4XYZ[0]))))+bodyIK4XYZ[0])*(sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(roll*(M_PI/180))+cos(roll*(M_PI/180))*sin(yaw*(M_PI/180)))+((coxaLength*sin(atan((leg4XYZ[1]-bodyIK4XYZ[1])/(leg4XYZ[0]-bodyIK4XYZ[0]))))+bodyIK4XYZ[1])*(-sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+cos(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+(initialBody4XYZ[2])*-sin(roll*(M_PI/180))*cos(pitch*(M_PI/180));
    //rpy of Coxa4Z
    coxa4IK[2] = (((coxaLength*cos(atan((leg4XYZ[1]-bodyIK4XYZ[1])/(leg4XYZ[0]-bodyIK4XYZ[0]))))+bodyIK4XYZ[0])*(-cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+sin(roll*(M_PI/180))*sin(yaw*(M_PI/180)))+((coxaLength*sin(atan((leg4XYZ[1]-bodyIK4XYZ[1])/(leg4XYZ[0]-bodyIK4XYZ[0]))))+bodyIK4XYZ[1])*(cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+sin(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+(initialBody4XYZ[2])*cos(roll*(M_PI/180))*cos(pitch*(M_PI/180)))-bodyHeight;

    //rpy of Coxa5X
    coxa5IK[0] = -((coxaLength*cos(atan((leg5XYZ[1]-bodyIK5XYZ[1])/(leg5XYZ[0]-bodyIK5XYZ[0]))))-bodyIK5XYZ[0])*cos(pitch*(M_PI/180))*cos(yaw*(M_PI/180))-((coxaLength*sin(atan((leg5XYZ[1]-bodyIK5XYZ[1])/(leg5XYZ[0]-bodyIK5XYZ[0]))))+bodyIK5XYZ[1])*-cos(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+(initialBody5XYZ[2])*sin(pitch*(M_PI/180));
    //rpy of Coxa5Y
    coxa5IK[1] = -((coxaLength*cos(atan((leg5XYZ[1]-bodyIK5XYZ[1])/(leg5XYZ[0]-bodyIK5XYZ[0]))))-bodyIK5XYZ[0])*(sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(roll*(M_PI/180))+cos(roll*(M_PI/180))*sin(yaw*(M_PI/180)))-((coxaLength*sin(atan((leg5XYZ[1]-bodyIK5XYZ[1])/(leg5XYZ[0]-bodyIK5XYZ[0]))))-bodyIK5XYZ[1])*(-sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+cos(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+(initialBody5XYZ[2])*-sin(roll*(M_PI/180))*cos(pitch*(M_PI/180));
    //rpy of Coxa5Z
    coxa5IK[2] = -(((coxaLength*cos(atan((leg5XYZ[1]-bodyIK5XYZ[1])/(leg5XYZ[0]-bodyIK5XYZ[0]))))-bodyIK5XYZ[0])*(-cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+sin(roll*(M_PI/180))*sin(yaw*(M_PI/180)))-((coxaLength*sin(atan((leg5XYZ[1]-bodyIK5XYZ[1])/(leg5XYZ[0]-bodyIK5XYZ[0]))))-bodyIK5XYZ[1])*(cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+sin(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+(initialBody5XYZ[2])*cos(roll*(M_PI/180))*cos(pitch*(M_PI/180)))-bodyHeight;

    //rpy of Coxa6X
    coxa6IK[0] = -((coxaLength*cos(atan((leg6XYZ[1]-bodyIK6XYZ[1])/(leg6XYZ[0]-bodyIK6XYZ[0]))))-bodyIK6XYZ[0])*cos(pitch*(M_PI/180))*cos(yaw*(M_PI/180))-((coxaLength*sin(atan((leg6XYZ[1]-bodyIK6XYZ[1])/(leg6XYZ[0]-bodyIK6XYZ[0]))))+bodyIK6XYZ[1])*-cos(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+(initialBody6XYZ[2])*sin(pitch*(M_PI/180));
    //rpy of Coxa6Y
    coxa6IK[1] = -((coxaLength*cos(atan((leg6XYZ[1]-bodyIK6XYZ[1])/(leg6XYZ[0]-bodyIK6XYZ[0]))))-bodyIK6XYZ[0])*(sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(roll*(M_PI/180))+cos(roll*(M_PI/180))*sin(yaw*(M_PI/180)))-((coxaLength*sin(atan((leg6XYZ[1]-bodyIK6XYZ[1])/(leg6XYZ[0]-bodyIK6XYZ[0]))))-bodyIK6XYZ[1])*(-sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+cos(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+(initialBody6XYZ[2])*-sin(roll*(M_PI/180))*cos(pitch*(M_PI/180));
    //rpy of Coxa6Z
    coxa6IK[2] = -(((coxaLength*cos(atan((leg6XYZ[1]-bodyIK6XYZ[1])/(leg6XYZ[0]-bodyIK6XYZ[0]))))-bodyIK6XYZ[0])*(-cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+sin(roll*(M_PI/180))*sin(yaw*(M_PI/180)))-((coxaLength*sin(atan((leg6XYZ[1]-bodyIK6XYZ[1])/(leg6XYZ[0]-bodyIK6XYZ[0]))))-bodyIK6XYZ[1])*(cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+sin(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+(initialBody6XYZ[2])*cos(roll*(M_PI/180))*cos(pitch*(M_PI/180)))-bodyHeight;

    //rpy of Coxa7X
    coxa7IK[0] = -((coxaLength*cos(atan((leg7XYZ[1]-bodyIK7XYZ[1])/(leg7XYZ[0]-bodyIK7XYZ[0]))))-bodyIK7XYZ[0])*cos(pitch*(M_PI/180))*cos(yaw*(M_PI/180))-((coxaLength*sin(atan((leg7XYZ[1]-bodyIK7XYZ[1])/(leg7XYZ[0]-bodyIK7XYZ[0]))))+bodyIK7XYZ[1])*-cos(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+(initialBody7XYZ[2])*sin(pitch*(M_PI/180));
    //rpy of Coxa7Y
    coxa7IK[1] = -((coxaLength*cos(atan((leg7XYZ[1]-bodyIK7XYZ[1])/(leg7XYZ[0]-bodyIK7XYZ[0]))))-bodyIK7XYZ[0])*(sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(roll*(M_PI/180))+cos(roll*(M_PI/180))*sin(yaw*(M_PI/180)))-((coxaLength*sin(atan((leg7XYZ[1]-bodyIK7XYZ[1])/(leg7XYZ[0]-bodyIK7XYZ[0]))))-bodyIK7XYZ[1])*(-sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+cos(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+(initialBody7XYZ[2])*-sin(roll*(M_PI/180))*cos(pitch*(M_PI/180));
    //rpy of Coxa7Z
    coxa7IK[2] = -(((coxaLength*cos(atan((leg7XYZ[1]-bodyIK7XYZ[1])/(leg7XYZ[0]-bodyIK7XYZ[0]))))-bodyIK7XYZ[0])*(-cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+sin(roll*(M_PI/180))*sin(yaw*(M_PI/180)))-((coxaLength*sin(atan((leg7XYZ[1]-bodyIK7XYZ[1])/(leg7XYZ[0]-bodyIK7XYZ[0]))))-bodyIK7XYZ[1])*(cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+sin(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+(initialBody7XYZ[2])*cos(roll*(M_PI/180))*cos(pitch*(M_PI/180)))-bodyHeight;

    //rpy of Coxa8X
    coxa8IK[0] = -((coxaLength*cos(atan((leg8XYZ[1]-bodyIK8XYZ[1])/(leg8XYZ[0]-bodyIK8XYZ[0]))))-bodyIK8XYZ[0])*cos(pitch*(M_PI/180))*cos(yaw*(M_PI/180))-((coxaLength*sin(atan((leg8XYZ[1]-bodyIK8XYZ[1])/(leg8XYZ[0]-bodyIK8XYZ[0]))))-bodyIK8XYZ[1])*-cos(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+(initialBody8XYZ[2])*sin(pitch*(M_PI/180));
    //rpy of Coxa8Y
    coxa8IK[1] = -((coxaLength*cos(atan((leg8XYZ[1]-bodyIK8XYZ[1])/(leg8XYZ[0]-bodyIK8XYZ[0]))))-bodyIK8XYZ[0])*(sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(roll*(M_PI/180))+cos(roll*(M_PI/180))*sin(yaw*(M_PI/180)))-((coxaLength*sin(atan((leg8XYZ[1]-bodyIK8XYZ[1])/(leg8XYZ[0]-bodyIK8XYZ[0]))))-bodyIK8XYZ[1])*(-sin(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+cos(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+(initialBody8XYZ[2])*-sin(roll*(M_PI/180))*cos(pitch*(M_PI/180));
    //rpy of Coxa8Z
    coxa8IK[2] = -(((coxaLength*cos(atan((leg8XYZ[1]-bodyIK8XYZ[1])/(leg8XYZ[0]-bodyIK8XYZ[0]))))-bodyIK8XYZ[0])*(-cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*cos(yaw*(M_PI/180))+sin(roll*(M_PI/180))*sin(yaw*(M_PI/180)))-((coxaLength*sin(atan((leg8XYZ[1]-bodyIK8XYZ[1])/(leg8XYZ[0]-bodyIK8XYZ[0]))))-bodyIK8XYZ[1])*(cos(roll*(M_PI/180))*sin(pitch*(M_PI/180))*sin(yaw*(M_PI/180))+sin(roll*(M_PI/180))*cos(yaw*(M_PI/180)))+(initialBody8XYZ[2])*cos(roll*(M_PI/180))*cos(pitch*(M_PI/180)))-bodyHeight;

    //output for leg IK solver. Leg IK is decoupled from y so we need to convert x,y to hypotenuse to use as x in leg frame.
    leg1IKxz[0] = sqrt(pow(leg1XYZ[0]-bodyIK1XYZ[0], 2)+pow(leg1XYZ[1]-bodyIK1XYZ[1], 2))-coxaLength;
    leg1IKxz[1] = coxa1IK[2];
    leg2IKxz[0] = sqrt(pow(leg2XYZ[0]-bodyIK2XYZ[0], 2)+pow(leg2XYZ[1]-bodyIK2XYZ[1], 2))-coxaLength;
    leg2IKxz[1] = coxa2IK[2];
    leg3IKxz[0] = sqrt(pow(leg3XYZ[0]-bodyIK3XYZ[0], 2)+pow(leg3XYZ[1]-bodyIK3XYZ[1], 2))-coxaLength;
    leg3IKxz[1] = coxa3IK[2];
    leg4IKxz[0] = sqrt(pow(leg4XYZ[0]-bodyIK4XYZ[0], 2)+pow(leg4XYZ[1]-bodyIK4XYZ[1], 2))-coxaLength;
    leg4IKxz[1] = coxa4IK[2];
    leg5IKxz[0] = sqrt(pow(leg5XYZ[0]-bodyIK5XYZ[0], 2)+pow(leg5XYZ[1]-bodyIK5XYZ[1], 2))-coxaLength;
    leg5IKxz[1] = coxa5IK[2];
    leg6IKxz[0] = sqrt(pow(leg6XYZ[0]-bodyIK6XYZ[0], 2)+pow(leg6XYZ[1]-bodyIK6XYZ[1], 2))-coxaLength;
    leg6IKxz[1] = coxa6IK[2];
    leg7IKxz[0] = sqrt(pow(leg7XYZ[0]-bodyIK7XYZ[0], 2)+pow(leg7XYZ[1]-bodyIK7XYZ[1], 2))-coxaLength;
    leg7IKxz[1] = coxa7IK[2];
    leg8IKxz[0] = sqrt(pow(leg8XYZ[0]-bodyIK8XYZ[0], 2)+pow(leg8XYZ[1]-bodyIK8XYZ[1], 2))-coxaLength;
    leg8IKxz[1] = coxa8IK[2];

    //Leg IK
    // For a 2DOF arm, we have 2 links and 2+1 joints,
    // where the end effector counts as one joint in this case.
    Fabrik2D Fabrik2D(4, lengths); // 4 Joints in total

    for (int i=0; leg1IKxz == leg1IKxz, i>30 ; i++ ) {

        // Solve IK,
        Fabrik2D.solve( leg1IKxz[0],  leg1IKxz[1], lengths);

        // Get the angles (in radians [-pi,pi]) and convert them to degrees [-180,180]
        coxa1Angle = Fabrik2D.getAngle(0) * (M_PI/180); // In degrees;
        femur1Angle = Fabrik2D.getAngle(1) * (M_PI/180); // In degrees;
        tibia1Angle = Fabrik2D.getAngle(2) * (M_PI/180); // In degrees;

    }
    cout << leg1IKxz[0] << endl;
    cout << leg1IKxz[1] << endl;
    cout << Fabrik2D.getAngle(0) << endl;
    cout << Fabrik2D.getAngle(1) << endl;
    cout << Fabrik2D.getAngle(2) << endl;
    cout << coxa1Angle << endl;
    cout << femur1Angle << endl;
    cout << tibia1Angle << endl;
    cout << "done";
/*
    test[0] = ((coxaLength*cos(atan((leg8XYZ[1]-bodyIK8XYZ[1])/(leg8XYZ[0]-bodyIK8XYZ[0]))))-bodyIK8XYZ[0]);
    test[1] = ((coxaLength*sin(atan((leg8XYZ[1]-bodyIK8XYZ[1])/(leg8XYZ[0]-bodyIK8XYZ[0]))))-bodyIK8XYZ[1]);
    test[2] = (bodyIK1XYZ[2]);
*
     for (int i = 0; i < 4; ++i) {
       // cout << bodyIK8XYZ[i] << "  ";
        cout << lengths[i] << "  ";
    }
*/
      return 0;
}
