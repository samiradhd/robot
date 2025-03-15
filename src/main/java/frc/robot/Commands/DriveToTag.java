package frc.robot.Commands;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.LimeVision;
import frc.robot.Subsystems.Drivetrain.Drive;
import frc.robot.Utilities.LimeLightHelpers;
import frc.robot.Utilities.PoseBasedLocator;

public class DriveToTag extends Command {
    Transform2d poseOffset;
    int[] validIds;
    boolean closest;
    Drive drive;
    Pose2d goalPose;
    boolean closestBasedOnPose = false;

    DriveToPoint driveCommand;
    LimeLightHelpers.RawFiducial targetTag = null;



    public DriveToTag(
        Transform2d offset,
        int[] validIds,
        boolean closest,
        Drive drive
    ) {
        this.poseOffset = offset;
        this.validIds = validIds;
        this.closest = closest;
        this.drive = drive;
    }

    public DriveToTag(
        Transform2d offset,
        int[] validIds,
        boolean closest,
        Drive drive,
        boolean closestBasedOnPose
    ) {
        this(offset, validIds, closest, drive);
        this.closestBasedOnPose = closestBasedOnPose;
    }

    @Override
    public void initialize() {
        targetTag = null;
        goalPose = null;

        if (closestBasedOnPose) {
            int targetId;

            if (validIds[0] == 1) {
                targetId = drive.poseBasedLocator.getClosestReefStationToRobot();
            } else if(validIds[0] == 3) {
                targetId = drive.poseBasedLocator.getClosestAlgaeStation();
            } else {
                targetId = drive.poseBasedLocator.getClosestCoralTagToRobot();
            }
            System.out.println("Found the closest tag " + targetId);
            if (targetId == 0) return;
            goalPose = LimeVision.getTagPose(targetId).plus(poseOffset);
        } else {
            ArrayList<LimeLightHelpers.RawFiducial> tags = LimeVision.getTags();
            double ambiguity = 0;
            double offset = 1000;
    
            for (LimeLightHelpers.RawFiducial tag: tags) {
                boolean found = false;
                for (int validid: validIds) {
                    if (validid == tag.id) found = true;
                }
    
                if (!found) continue;
    
                if (closest) {
                    double currentOffset = tag.txnc + tag.tync;
                    if (currentOffset > offset) {continue;}
                    offset = currentOffset;
    
                } else {
                    if (tag.ambiguity < ambiguity) {continue;}
    
                    if (tag.ambiguity < 0.1) {continue;}
    
                    ambiguity = tag.ambiguity;
                }
                
                targetTag = tag;
                goalPose = LimeVision.getTagPose(targetTag.id).plus(poseOffset);
            }
        }
        if (goalPose == null) return;

        driveCommand = new DriveToPoint(drive, goalPose);
        driveCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    };
}
