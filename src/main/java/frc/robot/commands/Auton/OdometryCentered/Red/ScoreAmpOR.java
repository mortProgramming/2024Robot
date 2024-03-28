package frc.robot.commands.Auton.OdometryCentered.Red;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Actions.RobotStart;
import frc.robot.commands.Actions.Drivetrain.MoveToPosition;

public class ScoreAmpOR extends SequentialCommandGroup{
    public ScoreAmpOR() {
        addCommands( 
            new SequentialCommandGroup(
                // new RobotStart(true, 0.4, 7.5, 90),
                new RobotStart(15.1, 7.4, 270),
                new MoveToPosition(13, 7.4, 270)
                // new MoveToPosition(1.9, 7.8, 90),
                // SetArmAndWristPos.score().withTimeout(ARM_WRIST_TIMEOUT),
                // new IntakeToVelocity(AMP_SHOOT_SPEED).withTimeout(0.75),
                // SetArmAndWristPos.rest().withTimeout(ARM_WRIST_TIMEOUT)
            )
        );
    }
}
