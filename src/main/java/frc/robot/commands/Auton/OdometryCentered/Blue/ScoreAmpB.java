package frc.robot.commands.Auton.OdometryCentered.Blue;

import static frc.robot.utility.Constants.Arm.*;
import static frc.robot.utility.Constants.Intake.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Actions.RobotStart;
import frc.robot.commands.Actions.Drivetrain.MoveToPosition;
import frc.robot.commands.Actions.Drivetrain.TimedDrive;
import frc.robot.commands.Actions.EndEffector.IntakeToVelocity;
import frc.robot.commands.Actions.EndEffector.SetArmAndWristPos;

public class ScoreAmpB extends SequentialCommandGroup{
    public ScoreAmpB() {
        addCommands( 
            new SequentialCommandGroup(
                new RobotStart(0.4, 7.5, 90),
                new MoveToPosition(1.9, 7.5, 90),
                new MoveToPosition(1.9, 7.8, 90),

                // new TimedDrive(1.7, 0, 1, 0),
                // new TimedDrive(0.75, -1, 0, 0),
                SetArmAndWristPos.score().withTimeout(ARM_WRIST_TIMEOUT),
                new IntakeToVelocity(AMP_SHOOT_SPEED).withTimeout(0.75),
                SetArmAndWristPos.rest().withTimeout(ARM_WRIST_TIMEOUT)
               
            )
        );
    }
}
