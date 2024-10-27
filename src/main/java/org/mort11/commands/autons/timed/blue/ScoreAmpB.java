package org.mort11.commands.autons.timed.blue;

import static org.mort11.configuration.constants.PhysicalConstants.Arm.*;
import static org.mort11.configuration.constants.PhysicalConstants.Intake.*;

import org.mort11.commands.actions.RobotStart;
import org.mort11.commands.actions.drivetrain.TimedDrive;
import org.mort11.commands.actions.endeffector.IntakeToVelocity;
import org.mort11.commands.actions.endeffector.armwrist.SetArmAndWristPos;
import org.mort11.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ScoreAmpB extends SequentialCommandGroup{
    // Drivetrain drivetrain;
    public ScoreAmpB() {
        // drivetrain = Drivetrain.getInstance();

        addCommands(
            new SequentialCommandGroup(
                new RobotStart(90),
                new TimedDrive(1, 0, .37, 0),
                new TimedDrive(0.75, -1, 0, 0),
                SetArmAndWristPos.amp().withTimeout(ARM_WRIST_TIMEOUT),
                new IntakeToVelocity(AMP_SHOOT_SPEED).withTimeout(0.75),
                SetArmAndWristPos.rest().withTimeout(ARM_WRIST_TIMEOUT)
               
            )
        );

        // addCommands(
        //     new SequentialCommandGroup(
        //         new RobotStart(true, 90),
        //         new InstantCommand(() -> drivetrain.setIsAngleKept(true)),
        //         new InstantCommand(() -> drivetrain.setKeptAngle(270)),
        //         new TimedDrive(true, 1.7, 0, 1, 0),
        //         new TimedDrive(true, 0.75, -1, 0, 0),
        //         SetArmAndWristPos.amp().withTimeout(ARM_WRIST_TIMEOUT),
        //         new IntakeToVelocity(AMP_SHOOT_SPEED).withTimeout(0.75),
        //         SetArmAndWristPos.rest().withTimeout(ARM_WRIST_TIMEOUT)
               
        //     )
        // );
    }
}