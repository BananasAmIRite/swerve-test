package frc.robot.lib.drive.swerve;

import com.ctre.phoenix.sensors.CANCoder;

import frc.robot.lib.controllers.pid.WPIPIDController;

public class SwerveTurnPIDController extends WPIPIDController {
    private CANCoder encoder;
    
    private double maxOutput = 1; 
    private double minOutput = -1; 

    public SwerveTurnPIDController(CANCoder encoder, double p, double i, double d) {
        super(p, i, d, () -> encoder.getAbsolutePosition());
        this.encoder = encoder; 


        enableContinuousInput(0, 360);
    }

    public double calculate() {
        // TODO: clamp this
        return calculate(encoder.getAbsolutePosition()); 
    }

    public void setMaxOutput(double maxOutput) {
        this.maxOutput = maxOutput; 
    }

    public void setMinOutput(double minOutput) {
        this.minOutput = minOutput; 
    }
}
