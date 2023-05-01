package frc.robot.lib.controllers.pid;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

public class SparkPIDController extends BasePIDController {

    private CANSparkMax motor; 
    private SparkMaxPIDController controller; 

    private ControlType controlType; 
    
    private Supplier<Double> measurement; 

    public SparkPIDController(CANSparkMax motor, Supplier<Double> measurement, ControlType controlType) {
        this.motor = motor; 
        this.controller = motor.getPIDController(); 
        this.controlType = controlType; 
        this.measurement = measurement; 
    }

    @Override
    public double getMeasurement() {
        return this.measurement.get(); 
    }

    @Override
    public double getP() {
        return this.controller.getP(); 
    }

    @Override
    public double getI() {
        return this.controller.getI(); 
    }

    @Override
    public double getD() {
        return this.controller.getD(); 
    }

    @Override
    public void setP(double p) {
        this.controller.setP(p); 
    }

    @Override
    public void setI(double i) {
        this.controller.setI(i); 
    }

    @Override
    public void setD(double d) {
        this.controller.setD(d); 
    }

    @Override
    public void setSetpoint(double setpoint) {
        this.controller.setReference(setpoint, this.controlType); 
    }
    
}
