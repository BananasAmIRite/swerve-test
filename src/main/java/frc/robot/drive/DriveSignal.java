package frc.robot.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveSignal {
    private final ChassisSpeeds speeds;
    private final boolean fieldRelative;

    public DriveSignal(ChassisSpeeds speeds, boolean fieldRelative) {
        this.speeds = speeds; 
        this.fieldRelative = fieldRelative; 
    }

    public ChassisSpeeds getSpeeds() {
        return speeds; 
    }

    public boolean isFieldRelative() {
        return fieldRelative; 
    }
}
