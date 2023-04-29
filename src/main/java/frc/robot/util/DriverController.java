package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.lib.util.Controller;
import frc.robot.lib.util.ControllerUtils;

public class DriverController extends Controller {
    private double deadzone;

    // TODO: refactor
    public enum Mode {
        NORMAL,
        SLOW
    }
    
    private Mode mode;

    public DriverController(int port) {
        this(port, 0.05); 
        
    }

    public DriverController(int port, double deadzone) {
        super(port); 
        this.deadzone = deadzone; 
    }


    // TODO: dunno if this should being in Controller or DriverController
    public double getLeftStickX() {
        return ControllerUtils.applyDeadband(super.getLeftStickX(), deadzone);
    }

    public double getLeftStickY() {
        return ControllerUtils.applyDeadband(super.getLeftStickY(), deadzone);
    }

    public double getRightStickX() {
        return ControllerUtils.applyDeadband(super.getRightStickX(), deadzone); 
    }

    public double getRightStickY() {
        return ControllerUtils.applyDeadband(super.getRightStickY(), deadzone);
    }
    
    public Mode getSlowMode() {
        return this.mode; 
    }
}
