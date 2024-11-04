package org.mort11.commands.autons.timed.blue;

import static org.mort11.configuration.constants.PhysicalConstants.Arm.*;
import static org.mort11.configuration.constants.PhysicalConstants.Intake.*;

import org.mort11.commands.actions.drivetrain.Orient;
import org.mort11.commands.actions.drivetrain.TimedDrive;
import org.mort11.commands.actions.endeffector.pos.SetArmAndWristPos;
import org.mort11.commands.actions.endeffector.velocity.IntakeToVelocity;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreAmpB extends SequentialCommandGroup {
    public ScoreAmpB() {
       addCommands(
            new SequentialCommandGroup(
                new Orient(90),
                new TimedDrive(1, 0, 0.37, 0),
                new TimedDrive(0.75, -1, 0, 0),
                SetArmAndWristPos.amp().withTimeout(ARM_WRIST_TIMEOUT),
                new IntakeToVelocity(AMP_SHOOT_SPEED).withTimeout(0.75),
                SetArmAndWristPos.rest().withTimeout(ARM_WRIST_TIMEOUT)
               
            )
        );
    }
}