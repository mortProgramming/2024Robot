package frc.robot.commands.Auton.Testing;

import static frc.robot.utility.Constants.Arm.ARM_WRIST_TIMEOUT;
import static frc.robot.utility.Constants.Intake.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Actions.RobotStart;
import frc.robot.commands.Actions.Drivetrain.Taxi;
import frc.robot.commands.Actions.Drivetrain.TimedDrive;
import frc.robot.commands.Actions.EndEffector.IntakeToVelocity;
import frc.robot.commands.Actions.EndEffector.SetArmAndWristPos;

public class ScoreAmp extends SequentialCommandGroup{
    public ScoreAmp() {
        addCommands(
            new SequentialCommandGroup(
                new RobotStart(270),
                new WaitCommand(0.3),
                new TimedDrive(1, 0, 1.8, 0),
                new TimedDrive(1, -0.55, 0, 0),
                SetArmAndWristPos.score().withTimeout(ARM_WRIST_TIMEOUT),
                new IntakeToVelocity(AMP_SHOOT_SPEED).withTimeout(0.6),
                SetArmAndWristPos.rest(),
                new WaitCommand(5)
            )
        );
    }
}
