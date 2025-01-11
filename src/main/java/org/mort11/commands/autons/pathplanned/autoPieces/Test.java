package org.mort11.commands.autons.pathplanned.autoPieces;

import org.mort11.commands.actions.drivetrain.Angle2AprilTag;
import org.mort11.commands.autons.Integrated.AutoGenerator;
import org.mort11.commands.autons.pathplanned.GetPlanned;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Test extends SequentialCommandGroup{

    public Test(){
        new AutoGenerator("Test", 
        new ParallelCommandGroup(new Angle2AprilTag(0)));
    }



}