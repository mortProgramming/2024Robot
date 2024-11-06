package org.mort11.commands.autons.pathplanned.paths;

import static org.mort11.config.constants.PhysicalConstants.Arm.*;
import static org.mort11.config.constants.PhysicalConstants.Intake.*;
import static org.mort11.config.constants.PhysicalConstants.Wrist.*;

import org.mort11.commands.actions.drivetrain.Orient;
import org.mort11.commands.actions.endeffector.IntakeBeamBreak;
import org.mort11.commands.actions.endeffector.SpitNote;
import org.mort11.commands.actions.endeffector.pos.SetArmWristPos;
import org.mort11.commands.actions.endeffector.pos.WristToPos;
import org.mort11.commands.actions.endeffector.vel.IntakeToVel;
import org.mort11.config.IO;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class All {
    
    public static void setCommands () {
        NamedCommands.registerCommand("AutoActive", new SequentialCommandGroup(new InstantCommand(() -> System.out.println("PATH AUTON IS ACTIVE"))));

        NamedCommands.registerCommand("FieldOrient", new Orient(IO.isBlue() ?  270 : 90));

        NamedCommands.registerCommand("ScoreInAmp", 
            new SequentialCommandGroup(//Bring arm and wrist to score position, eject note, back to rest
                new WristToPos(WRIST_REST_POS).withTimeout(0.01),
                SetArmWristPos.amp().withTimeout(ARM_WRIST_TIMEOUT),
                new IntakeToVel(AUTO_SHOOT_SPEED).withTimeout(0.4),
                SetArmWristPos.rest().withTimeout(ARM_WRIST_TIMEOUT))
            .withTimeout(3.45));

        NamedCommands.registerCommand("Intake", 
            new ParallelCommandGroup(
                new PrintCommand("RUNNING INTAKE"),  
                new IntakeBeamBreak(WRIST_REST_POS))
            .withTimeout(2.2));
    
        NamedCommands.registerCommand("IntakeStayOut",
            new IntakeBeamBreak(WRIST_INTAKE_POS));

        NamedCommands.registerCommand("Spit", 
            new SpitNote());

        NamedCommands.registerCommand("Outtake", new IntakeToVel(-0.65)
            .withTimeout(0.75));
    }
}
