package frc.robot.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain.Drive;

public class AlignCommands {

    public static Command goToTagEight(Drive drive) {
        return new DriveToPoint(
            drive, 
            Constants.Location.APRIL_TAGS.get(7).pose.toPose2d()
                .plus(new Transform2d(1,0 , new Rotation2d(Math.toRadians(180)))));
    }

    public static Command goToClosestTag(Drive drive) {
        System.out.println("Created command");
        return new DriveToTag(
            new Transform2d(1, 0, new Rotation2d(Math.PI)),
             new int[] {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22}, 
             true, 
             drive, true);
    }

    public static Command goToTag(Drive drive, int tag, Transform2d offset) {
        return new DriveToPoint(drive,
         Constants.Location.APRIL_TAGS.get(tag - 1).pose.toPose2d()
         .plus(offset));
    }

    public static Command alignToTags(Drive drive, Transform2d offset, int[] tags) {
        return new DriveToTag(offset, tags, false, drive, true);
    }

    public static Command alignToReef(Drive drive, Transform2d offset) {
        return alignToTags(drive, offset, Constants.Location.REEF_IDS);
    }

    public static Command alignToCoralStation(Drive drive, Transform2d offset) {
        return alignToTags(drive, offset, Constants.Location.CORAL_STATION_IDS);
    }

    // only use these
    public static Command alignToProcessor(Drive drive) {
        return alignToTags(drive, Constants.Location.PROCESSOR_OFFSET, Constants.Location.PROCESSOR_IDS);
    }

    public static Command frontAlignToReefLeft(Drive drive) {
        return alignToReef(drive, Constants.Location.FRONT_REEF_LEFT_OFFSET);
    }

    public static Command frontAlignToReefRight(Drive drive) {
        return alignToReef(drive, Constants.Location.FRONT_REEF_RIGHT_OFFSET);
    }

    // public static Command backAlignToReefLeft(Drive drive) {
    //     return alignToReef(drive, Constants.Location.BACK_REEF_LEFT_OFFSET);
    // }

    // public static Command backAlignToReefRight(Drive drive) {
    //     return alignToReef(drive, Constants.Location.BACK_REEF_RIGHT_OFFSET);
    // }

    public static Command backAlignToCoralStationLeft(Drive drive) {
        return alignToCoralStation(drive, Constants.Location.CORAL_STATION_LEFT_OFFSET);
    }

    public static Command backAlignToCoralStationRight(Drive drive) {
        return alignToCoralStation(drive, Constants.Location.CORAL_STATION_RIGHT_OFFSET);
    }

    public static Command backAlignToCoralStationCenter(Drive drive) {
        return alignToCoralStation(drive, Constants.Location.CORAL_STATION_CENTER_OFFSET);
    }
}
