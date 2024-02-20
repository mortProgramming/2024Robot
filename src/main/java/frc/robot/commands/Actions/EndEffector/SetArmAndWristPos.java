package frc.robot.commands.Actions.EndEffector;

import static frc.robot.utility.Constants.Arm.*;
import static frc.robot.utility.Constants.Wrist.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SetArmAndWristPos extends SequentialCommandGroup{
    public SetArmAndWristPos(double armSetpoint, double wristSetpoint){
        addCommands(new ArmToPosition(ARM_REST_POSITION).withTimeout(0.6), new WristToPosition(wristSetpoint),
        new ArmToPosition(armSetpoint));
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
}
