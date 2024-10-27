package org.mort11.commands.autons.pathplanned;

import org.mort11.commands.autons.pathplanned.paths.All;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;

public class GetPlanned {

    public static Command getTwoPiece() {
        All.setCommands();

        return new PathPlannerAuto("PathPlanner2Piece");
    }

    public static Command getGackleyAuto() {
        All.setCommands();

        return new PathPlannerAuto("GackleyAuto1");
    }

    public static Command getBieryAuto() {
        All.setCommands();

        return new PathPlannerAuto("BieryWildAuto");
    }

    public static Command getChoreoOneNote() {
        All.setCommands();

        return new PathPlannerAuto("OneNote");
    }

    public static Command getTwoPieceAmpSide() {
        All.setCommands();

        return new PathPlannerAuto("TwoPieceAmpSide");
    }
}
