package org.mort11.commands.actions.endeffector.pos;

import static org.mort11.config.constants.PhysicalConstants.Arm.*;
import static org.mort11.config.constants.PhysicalConstants.Wrist.*;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SetArmWristPos extends SequentialCommandGroup{
    public SetArmWristPos(double armSetpoint, double wristSetpoint){
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

    public final static SetArmWristPos amp(){
        return new SetArmWristPos(ARM_AMP_POS, WRIST_REST_POS);
    }

    public final static SetArmWristPos intake(){
        return new SetArmWristPos(ARM_REST_POS, WRIST_INTAKE_POS);
    }

    public final static SetArmWristPos rest(){
        return new SetArmWristPos(ARM_REST_POS, WRIST_REST_POS);
    }

    public final static SetArmWristPos preTrap(){
        return new SetArmWristPos(ARM_TRAP_POS, WRIST_TRAP_POS);
    }

    public final static SetArmWristPos trap(){
        return new SetArmWristPos(ARM_TRAP_POS, WRIST_TRAP_POS);
    }

    public final static SetArmWristPos floorTrap(){
        return new SetArmWristPos(ARM_FLOORTRAP_POS, WRIST_FLOORTRAP_POS);
    }
}
