package frc.robot.commands.Auton.Timed.Red;

import static frc.robot.utility.Constants.Arm.*;
import static frc.robot.utility.Constants.Intake.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Actions.RobotStart;
import frc.robot.commands.Actions.Drivetrain.TimedDrive;
import frc.robot.commands.Actions.EndEffector.IntakeToVelocity;
import frc.robot.commands.Actions.EndEffector.SetArmAndWristPos;
import frc.robot.subsystems.Drivetrain;

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
