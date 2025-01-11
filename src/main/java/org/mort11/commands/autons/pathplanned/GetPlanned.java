package org.mort11.commands.autons.pathplanned;

import org.mort11.commands.autons.pathplanned.paths.All;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;

public class GetPlanned {

    public static Command getTest(){
        All.setCommands();
        return new PathPlannerAuto("Test");
    }
}
