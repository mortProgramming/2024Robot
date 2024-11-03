package org.mort11.commands.autons.odometered;

import org.mort11.commands.actions.RobotStart;
import org.mort11.commands.actions.drivetrain.MoveToPos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreAmpRed extends SequentialCommandGroup{
    public ScoreAmpRed() {
        addCommands( 
            new SequentialCommandGroup(
                // new RobotStart(true, 0.4, 7.5, 90),
                new RobotStart(15.1, 7.4, 270),
                new MoveToPos(13, 7.4, 270)
                // new MoveToPosition(1.9, 7.8, 90),
                // SetArmAndWristPos.amp().withTimeout(ARM_WRIST_TIMEOUT),
                // new IntakeToVelocity(AMP_SHOOT_SPEED).withTimeout(0.75),
                // SetArmAndWristPos.rest().withTimeout(ARM_WRIST_TIMEOUT)
            )
        );
    }
}
