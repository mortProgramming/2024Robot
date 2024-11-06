package org.mort11.commands.actions.endeffector.vel;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

import org.mort11.subsystems.Arm;

public class ArmToVel extends Command{
    private Arm arm;

    private DoubleSupplier velocity;

    public ArmToVel(DoubleSupplier velocity){
        this.velocity = velocity;

        arm = Arm.getInstance();

        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setArmVelocity(velocity.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        arm.setArmVelocity(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
