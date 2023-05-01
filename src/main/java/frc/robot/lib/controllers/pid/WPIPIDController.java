package frc.robot.lib.controllers.pid;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;

public class WPIPIDController extends BasePIDController {

    private PIDController controller; 

    private Supplier<Double> measurement; 

    public WPIPIDController(Supplier<Double> measurement) {
        this(new PIDController(0, 0, 0), measurement); 
    }

    public WPIPIDController(PIDController pidController, Supplier<Double> measurement) {
        this.controller = pidController; 
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
        this.controller.setSetpoint(setpoint);
    }
    
}
