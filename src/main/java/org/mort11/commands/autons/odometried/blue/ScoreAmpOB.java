package org.mort11.commands.autons.odometried.blue;

import static org.mort11.configuration.constants.PhysicalConstants.Arm.*;
import static org.mort11.configuration.constants.PhysicalConstants.Intake.*;

import org.mort11.commands.actions.RobotStart;
import org.mort11.commands.actions.drivetrain.MoveToPos;
import org.mort11.commands.actions.drivetrain.TimedDrive;
import org.mort11.commands.actions.endeffector.IntakeToVelocity;
import org.mort11.commands.actions.endeffector.armwrist.SetArmAndWristPos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreAmpOB extends SequentialCommandGroup{
    public ScoreAmpOB() {
        addCommands( 
            new SequentialCommandGroup(
                // new RobotStart(true, 0.4, 7.5, 90),
                new RobotStart(0.4, 7.4, 270),
                new MoveToPos(1.5, 7.9, 270)
                // new MoveToPosition(1.9, 7.8, 90),
                // SetArmAndWristPos.amp().withTimeout(ARM_WRIST_TIMEOUT),
                // new IntakeToVelocity(AMP_SHOOT_SPEED).withTimeout(0.75),
                // SetArmAndWristPos.rest().withTimeout(ARM_WRIST_TIMEOUT)
            )
        );
    }
}
