package frc.robot.lib.controllers.pid;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public interface FeedForwardController {
    void setFF(SimpleMotorFeedforward ff);
    SimpleMotorFeedforward getFF();  
}
