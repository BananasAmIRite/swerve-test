package frc.robot.lib.controllers.pid;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;

public class WPIPIDController extends PIDController implements BasePIDController {

    private Supplier<Double> measurement; 

    private double output = 0; 

    public WPIPIDController(Supplier<Double> measurement) {
        this(0, 0, 0, measurement); 
    }

    public WPIPIDController(double p, double i, double d, Supplier<Double> measurement) {
        super(p, i, d); 
        this.measurement = measurement; 
    }

    @Override
    public double getMeasurement() {
        return this.measurement.get(); 
    }

    @Override
    public void update() {
        this.output = this.calculate(measurement.get()); 
    }

    @Override
    public double getOutput() {
        return this.output; 
    }
    
}
