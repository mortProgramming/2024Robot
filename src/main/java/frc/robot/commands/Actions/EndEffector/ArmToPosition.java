package frc.robot.commands.Actions.EndEffector;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmToPosition extends Command{
    private Arm arm;
    private double target;

    public ArmToPosition(double target){
        this.target = target;
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setPIDorVelocity(false);
    }

    @Override
    public void execute() {
        arm.setSetPoint(target);
        arm.setPIDorVelocity(false);

    }

    @Override
    public void end(boolean interrupted) {
        arm.setPIDorVelocity(true);
    }


    @Override
    public boolean isFinished() {
        return arm.nearSetpoint();
    }
}
