package org.mort11.commands.actions.drivetrain;

import org.mort11.subsystems.Drivetrain;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class GyroOrient extends Command {
    private final Drivetrain drivetrain;
    private final double targetAngle;

    public GyroOrient(double targetAngle) {
        this.drivetrain = Drivetrain.getInstance();
        this.targetAngle = targetAngle;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.getRotateToAngleController().reset(0, 0);
    }

    @Override
    public void execute() {
        double currentAngle = drivetrain.getIMURotation().getDegrees();
        double rotationSpeed = drivetrain.getRotateToAngleController().calculate(currentAngle, targetAngle);
        drivetrain.setDrive(new ChassisSpeeds(0, 0, rotationSpeed * 0.25));
    }

    @Override
    public boolean isFinished() {
        return drivetrain.getRotateToAngleController().atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setDrive(new ChassisSpeeds(0, 0, 0));
    }
}
