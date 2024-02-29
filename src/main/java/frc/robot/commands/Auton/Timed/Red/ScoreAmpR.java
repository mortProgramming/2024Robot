package frc.robot.commands.Auton.Timed.Red;

import static frc.robot.utility.Constants.Arm.*;
import static frc.robot.utility.Constants.Intake.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Actions.RobotStart;
import frc.robot.commands.Actions.Drivetrain.TimedDrive;
import frc.robot.commands.Actions.EndEffector.IntakeToVelocity;
import frc.robot.commands.Actions.EndEffector.SetArmAndWristPos;

public class ScoreAmpR extends SequentialCommandGroup{
    public ScoreAmpR() {
        addCommands(
            new SequentialCommandGroup(
                new RobotStart(270),
                // new RobotStart(0),
                // new WaitCommand(0.3),
                new TimedDrive(1.65, 0, 1, 0),
                new TimedDrive(0.75, 1, 0, 0),
                SetArmAndWristPos.score().withTimeout(ARM_WRIST_TIMEOUT),
                new IntakeToVelocity(AMP_SHOOT_SPEED).withTimeout(1),
                SetArmAndWristPos.rest().withTimeout(ARM_WRIST_TIMEOUT)
               
            )
        );
    }
}
