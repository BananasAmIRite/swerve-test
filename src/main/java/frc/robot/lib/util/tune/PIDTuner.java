package frc.robot.lib.util.tune;

import frc.robot.lib.controllers.pid.BasePIDController; 
public class PIDTuner<T extends BasePIDController> extends Tuner {
    private T controller;

    public PIDTuner(String name, T controller) {
        super(name);
        addTunableDouble("P", this.controller::setP);
        addTunableDouble("I", this.controller::setI);
        addTunableDouble("D", this.controller::setD);
        addDisplayValue("measurement", this::getMeasurement);
        addTunableDouble("setpoint", this.controller::setSetpoint);
    }

    public double getMeasurement() {
        return this.controller.getMeasurement(); 
    }
}
