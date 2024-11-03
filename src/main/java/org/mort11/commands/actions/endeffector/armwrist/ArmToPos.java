package org.mort11.commands.actions.endeffector.armwrist;

import org.mort11.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmToPos extends Command{
    private Arm arm;

    private double target;

    public ArmToPos(double target) {
        this.target = target;

        arm = Arm.getInstance();

        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setSetpoint(target);
    }

    @Override
    public boolean isFinished() {
        return arm.nearSetpoint();
    }
}
