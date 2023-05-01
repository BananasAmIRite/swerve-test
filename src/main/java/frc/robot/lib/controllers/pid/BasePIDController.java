package frc.robot.lib.controllers.pid;

public abstract class BasePIDController {
    public abstract double getMeasurement(); 
    public abstract void setSetpoint(double setpoint); 


    public abstract double getP();
    public abstract double getI(); 
    public abstract double getD();
    
    public abstract void setP(double p);
    public abstract void setI(double i);
    public abstract void setD(double d); 
}
