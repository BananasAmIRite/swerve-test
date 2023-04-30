package frc.robot.lib.util.tune;

import edu.wpi.first.math.controller.PIDController;

import java.util.function.DoubleSupplier;

// TODO: test
public class WPIPIDTuner extends PIDTuner {
    private final PIDController controller;
    private final DoubleSupplier measurementSupplier;
    public WPIPIDTuner(String name, PIDController controller, DoubleSupplier measurementSupplier) {
        super(name, controller.getP(), controller.getI(), controller.getD());
        this.controller = controller;
        this.measurementSupplier = measurementSupplier;
    }

    @Override
    public void supplyPIDs(double p, double i, double d, double setpoint) {
        controller.setP(p);
        controller.setI(i);
        controller.setD(d);
        controller.setSetpoint(setpoint);
    }

    @Override
    public double getMeasurement() {
        return measurementSupplier.getAsDouble();
    }
}
