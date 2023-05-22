package frc.robot.lib.controllers.pid;

public interface BasePIDController {
    double getMeasurement(); 
    void setSetpoint(double setpoint); 
    double getOutput(); 

    double getP();
    double getI(); 
    double getD();
    
    void setP(double p);
    void setI(double i);
    void setD(double d); 

    void update(); 
}
