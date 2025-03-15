package frc.robot.Utilities;

import java.io.IOException;
import java.util.Optional;


import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain.Drive;

public class PoseBasedLocator {
    Drive drive;
    Optional<Alliance> alliance = DriverStation.getAlliance();
    
    boolean red = false;

    public PoseBasedLocator(Drive drive) {
        this.drive = drive;
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Blue) red = false;
        }
    }

    public int getClosestCoralTagToRobot() {
        Pose2d robotPose = drive.getPose();

        double closestMagnitude = 10^5;
        int possibleNewId = 0;
        for (int possibleCoralId: Constants.Location.REEF_IDS) {
            Pose2d coralPose = Constants.Location.APRIL_TAGS.get(possibleCoralId-1).pose.toPose2d();
            Transform2d differnacial = coralPose.minus(robotPose);

            double newMag = Math.sqrt(Math.pow(differnacial.getX(), 2) + Math.pow(differnacial.getY(), 2));
            if (closestMagnitude > newMag) {
                closestMagnitude = newMag;
                possibleNewId = possibleCoralId;

            }
        }

        // make sure we are not detecting the other teams april tags for this.
        if (red && possibleNewId > 11) return 0;
        // be sure to have a factor where you discard a 0
        return possibleNewId;
    }

    public int getClosestAlgaeStation() {
        return red ? 3 : 16;
    }

    public int getClosestReefStationToRobot() {
        Pose2d robotPose = drive.getPose();

        double closestMagnitude = 10^5;
        int possibleNewId = 0;

        for (int possibleCoralId: Constants.Location.CORAL_STATION_IDS) {
            Pose2d stationPose = Constants.Location.APRIL_TAGS.get(possibleCoralId-1).pose.toPose2d();
            Transform2d differnacial = stationPose.minus(robotPose);

            double newMag = Math.sqrt(Math.pow(differnacial.getX(), 2) + Math.pow(differnacial.getY(), 2));
            if (closestMagnitude > newMag) {
                closestMagnitude = newMag;
                possibleNewId = possibleCoralId;

            }
        }

        if (red && possibleNewId > 11) return 0;
        return possibleNewId;
    }

    public int getClosestTagToRobot(int[] tags) {
        if (tags.length == 0) {
            tags = new int[] {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22};
        }

        int result = 0;
        Pose2d robotPose = drive.getPose();
        double closestMagnitude = 10^5;

        for (int possibleId: tags) {
            Pose2d tagPose = Constants.Location.APRIL_TAGS.get(possibleId-1).pose.toPose2d();
            Transform2d differnacial = tagPose.minus(robotPose);

            double newMag = Math.sqrt(Math.pow(differnacial.getX(), 2) + Math.pow(differnacial.getY(), 2));
            if (closestMagnitude > newMag) {
                closestMagnitude = newMag;
                result = possibleId;
            }
        }

        if (red && result > 11) return 0;
        return result;
    }
}
