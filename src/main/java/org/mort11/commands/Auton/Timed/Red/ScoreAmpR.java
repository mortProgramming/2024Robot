package org.mort11.commands.Auton.Timed.Red;

import static org.mort11.configuration.Constants.Arm.*;
import static org.mort11.configuration.Constants.Intake.*;

import org.mort11.commands.Actions.RobotStart;
import org.mort11.commands.Actions.Drivetrain.TimedDrive;
import org.mort11.commands.Actions.EndEffector.IntakeToVelocity;
import org.mort11.commands.Actions.EndEffector.ArmWrist.SetArmAndWristPos;
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
                SetArmAndWristPos.score().withTimeout(ARM_WRIST_TIMEOUT),
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
        //         SetArmAndWristPos.score().withTimeout(ARM_WRIST_TIMEOUT),
        //         new IntakeToVelocity(AMP_SHOOT_SPEED).withTimeout(1),
        //         SetArmAndWristPos.rest().withTimeout(ARM_WRIST_TIMEOUT)
               
        //     )
        // );
    }
}
