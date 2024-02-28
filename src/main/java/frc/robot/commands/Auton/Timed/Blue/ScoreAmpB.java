package frc.robot.commands.Auton.Timed.Blue;

import static frc.robot.utility.Constants.Arm.*;
import static frc.robot.utility.Constants.Intake.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Actions.RobotStart;
import frc.robot.commands.Actions.Drivetrain.TimedDrive;
import frc.robot.commands.Actions.EndEffector.IntakeToVelocity;
import frc.robot.commands.Actions.EndEffector.SetArmAndWristPos;

public class ScoreAmpB extends SequentialCommandGroup{
    public ScoreAmpB() {
        addCommands(
            new SequentialCommandGroup(
                new RobotStart(90),
                // new RobotStart(0),
                // new WaitCommand(0.3),
                new TimedDrive(1.7, 0, 1, 0),
                new TimedDrive(0.75, -1, 0, 0),
                SetArmAndWristPos.score().withTimeout(ARM_WRIST_TIMEOUT),
                new IntakeToVelocity(AMP_SHOOT_SPEED).withTimeout(0.75),
                SetArmAndWristPos.rest().withTimeout(ARM_WRIST_TIMEOUT)
               
            )
        );

        // addCommands(
        //     new SequentialCommandGroup(
        //         new RobotStart(90),
        //         // new RobotStart(0),
        //         // new WaitCommand(0.3),
        //         new TimedDrive(true, 1.7, 0, 1, 0),
        //         new TimedDrive(true, 0.75, -1, 0, 0),
        //         SetArmAndWristPos.score().withTimeout(ARM_WRIST_TIMEOUT),
        //         new IntakeToVelocity(AMP_SHOOT_SPEED).withTimeout(0.75),
        //         SetArmAndWristPos.rest().withTimeout(ARM_WRIST_TIMEOUT)
               
        //     )
        // );
    }
}