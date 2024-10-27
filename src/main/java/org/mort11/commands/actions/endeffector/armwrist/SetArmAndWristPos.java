package org.mort11.commands.actions.endeffector.armwrist;

import static org.mort11.configuration.constants.PhysicalConstants.Arm.*;
import static org.mort11.configuration.constants.PhysicalConstants.Wrist.*;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SetArmAndWristPos extends SequentialCommandGroup{
    public SetArmAndWristPos(double armSetpoint, double wristSetpoint){
        addCommands(
            new SequentialCommandGroup(
                new WristToPos(WRIST_REST_POS).withTimeout(0.3),
                new ArmToPos(armSetpoint).withTimeout(0.5),
                new ParallelCommandGroup(
                    new ArmToPos(armSetpoint).withTimeout(1),
                    new WristToPos(wristSetpoint).withTimeout(0.2)
                )
            )
        );
    }

    public final static SetArmAndWristPos amp(){
        return new SetArmAndWristPos(ARM_AMP_POS, WRIST_REST_POS);
    }

    public final static SetArmAndWristPos intake(){
        return new SetArmAndWristPos(ARM_REST_POS, WRIST_INTAKE_POS);
    }

    public final static SetArmAndWristPos rest(){
        return new SetArmAndWristPos(ARM_REST_POS, WRIST_REST_POS);
    }

    public final static SetArmAndWristPos preTrap(){
        return new SetArmAndWristPos(ARM_TRAP_POS, WRIST_TRAP_POS);
    }

    public final static SetArmAndWristPos trap(){
        return new SetArmAndWristPos(ARM_TRAP_POS, WRIST_TRAP_POS);
    }

    public final static SetArmAndWristPos floorTrap(){
        return new SetArmAndWristPos(ARM_FLOORTRAP_POS, WRIST_FLOORTRAP_POS);
    }
}
