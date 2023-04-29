package frc.robot.lib.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;

public abstract class DriveCommand extends CommandBase {

    private final DrivetrainBase drivetrain;

    public DriveCommand(DrivetrainBase drivetrain) {
        this.drivetrain = drivetrain;
    }
    public abstract DriveSignal update();

    @Override
    public void execute() {
        this.drivetrain.drive(this.update());
    }
}
