package frc.robot.lib.util.tune;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import java.util.function.DoubleSupplier;

public class SparkMaxPIDTuner extends PIDTuner {
    private final SparkMaxPIDController controller;
    private final CANSparkMax.ControlType controlType;
    private final DoubleSupplier measurementSupplier;
    public SparkMaxPIDTuner(String name, CANSparkMax motor, CANSparkMax.ControlType controlType, DoubleSupplier measurementSupplier) {
        super(name, motor.getPIDController().getP(), motor.getPIDController().getI(), motor.getPIDController().getD());
        this.measurementSupplier = measurementSupplier;
        this.controller = motor.getPIDController();
        this.controlType = controlType;
    }

    @Override
    public void supplyPIDs(double p, double i, double d, double setpoint) {
        controller.setP(p);
        controller.setI(i);
        controller.setD(d);
        controller.setReference(setpoint, controlType);
    }

    @Override
    public double getMeasurement() {
        return measurementSupplier.getAsDouble();
    }
}
