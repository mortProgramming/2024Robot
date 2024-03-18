package frc.robot.commands.Actions.EndEffector.ArmWrist;

import static frc.robot.utility.Constants.Arm.*;
import static frc.robot.utility.Constants.Wrist.*;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SetArmAndWristPos extends SequentialCommandGroup{
    public SetArmAndWristPos(double armSetpoint, double wristSetpoint){
        addCommands(
            
            new ParallelCommandGroup(
                new ArmToPosition(armSetpoint).withTimeout(1),
                new WristToPosition(wristSetpoint).withTimeout(.3)
            )
        );
    }

    public final static SetArmAndWristPos drop(){
        return new SetArmAndWristPos(ARM_INTAKE_POSITION, WRIST_REST_POSITION);
    }

    public final static SetArmAndWristPos rest(){
        return new SetArmAndWristPos(ARM_REST_POSITION, WRIST_REST_POSITION);
    }

    public final static SetArmAndWristPos score(){
        return new SetArmAndWristPos(ARM_AMP_POSITION, WRIST_SCORE_POSITION);
    }

    public final static SetArmAndWristPos intake(){
        return new SetArmAndWristPos(ARM_INTAKE_POSITION, WRIST_INTAKE_POSITION);
    }

    public final static SetArmAndWristPos zero(){
        return new SetArmAndWristPos(0, 0);
    }
    public final static SetArmAndWristPos trap(){
        return new SetArmAndWristPos(ARM_TRAP_POSITION, WRIST_TRAP_POSITION);
    }
}
