package org.mort11.commands.autons.timed.red;

import static org.mort11.configuration.constants.PhysicalConstants.Arm.*;
import static org.mort11.configuration.constants.PhysicalConstants.Intake.*;

import org.mort11.commands.actions.RobotStart;
import org.mort11.commands.actions.drivetrain.TimedDrive;
import org.mort11.commands.actions.endeffector.IntakeToVelocity;
import org.mort11.commands.actions.endeffector.armwrist.SetArmAndWristPos;
import org.mort11.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ScoreAmpR extends SequentialCommandGroup{
    // Drivetrain drivetrain;

    public ScoreAmpR() {
        // drivetrain = Drivetrain.getInstance();

        addCommands(
            new SequentialCommandGroup(
                new RobotStart(270),
                new TimedDrive(1, 0, .37, 0),//align with amp
                new TimedDrive(0.75, 1, 0, 0),//go to amp
                SetArmAndWristPos.amp().withTimeout(ARM_WRIST_TIMEOUT),
                new IntakeToVelocity(AMP_SHOOT_SPEED).withTimeout(.75),
                SetArmAndWristPos.rest().withTimeout(ARM_WRIST_TIMEOUT)
               
            )
        );

        // addCommands(
        //     new SequentialCommandGroup(
        //         new RobotStart(false, 270),
        //         new InstantCommand(() -> drivetrain.setIsAngleKept(true)),
        //         new InstantCommand(() -> drivetrain.setKeptAngle(90)),
        //         new TimedDrive(true, 1.65, 0, 1, 0),
        //         new TimedDrive(true, 0.75, 1, 0, 0),
        //         SetArmAndWristPos.amp().withTimeout(ARM_WRIST_TIMEOUT),
        //         new IntakeToVelocity(AMP_SHOOT_SPEED).withTimeout(1),
        //         SetArmAndWristPos.rest().withTimeout(ARM_WRIST_TIMEOUT)
               
        //     )
        // );
    }
}
