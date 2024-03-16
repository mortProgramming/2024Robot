package frc.robot.commands.Auton.OdometryCentered.Blue;

import static frc.robot.utility.Constants.Arm.*;
import static frc.robot.utility.Constants.Intake.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Actions.RobotStart;
import frc.robot.commands.Actions.Drivetrain.MoveToPosition;
import frc.robot.commands.Actions.Drivetrain.TimedDrive;
import frc.robot.commands.Actions.EndEffector.IntakeToVelocity;
import frc.robot.commands.Actions.EndEffector.ArmWrist.SetArmAndWristPos;

public class ScoreAmpOB extends SequentialCommandGroup{
    public ScoreAmpOB() {
        addCommands( 
            new SequentialCommandGroup(
                // new RobotStart(true, 0.4, 7.5, 90),
                new RobotStart(0.4, 7.5, 90),
                new MoveToPosition(1.9, 10, 90)
                // new MoveToPosition(1.9, 7.8, 90),
                // SetArmAndWristPos.score().withTimeout(ARM_WRIST_TIMEOUT),
                // new IntakeToVelocity(AMP_SHOOT_SPEED).withTimeout(0.75),
                // SetArmAndWristPos.rest().withTimeout(ARM_WRIST_TIMEOUT)
            )
        );
    }
}
