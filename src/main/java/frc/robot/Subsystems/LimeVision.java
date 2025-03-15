package frc.robot.Subsystems;

import java.util.ArrayList;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain.Drive;
import frc.robot.Utilities.LimeLightHelpers;

public class LimeVision implements Sendable{

    static Transform3d robotToCam;
    static Drive drive;

    static String[] limeLights = {"limelight-one"};
    static LimeLightHelpers.PoseEstimate poseEstimate;

    static {
        //Cam mounted facing forward, -0.381 meter forward of center, 0.558 a meter up from center.
        robotToCam = new Transform3d(new Translation3d(-0.332613, 0.0, -0.1529588), new Rotation3d(Math.PI, -Math.PI/4, Math.PI)); 

        System.out.println("Fetching tags, please wait");
        for (int x = 1; x < (Constants.Location.APRIL_TAGS_COUNT+1); x++) {
            getTagPose(x);
            System.out.println("Fetched tag: " + x);
        }

        System.out.println("Fetched all tags");
    }

    public static ArrayList<LimeLightHelpers.RawFiducial> getTags() {
        ArrayList<LimeLightHelpers.RawFiducial> tags = new ArrayList<>();

        for (String limeName: limeLights) {
            LimeLightHelpers.RawFiducial[] tagsFromCurrentLimeLight = LimeLightHelpers.getRawFiducials(limeName);
            
            for (LimeLightHelpers.RawFiducial tag: tagsFromCurrentLimeLight) {
                tags.add(tag);
            }
        }

        return tags;
    }

    public static void setRobotOrientation() {
        double yaw = Gyro.getYaw();
        for (String limeName: limeLights) {
            LimeLightHelpers.SetRobotOrientation(limeName, yaw, 0, 0, 0, 0, 0);
        }
    }


    public static LimeLightHelpers.PoseEstimate getPoseEstimate() {
        SmartDashboard.putNumber("VisionUpdateTracker", Timer.getFPGATimestamp());

        setRobotOrientation();

        LimeLightHelpers.PoseEstimate estimateCache = null;

        for (String limeName: limeLights) {
            LimeLightHelpers.PoseEstimate tempCache = LimeLightHelpers.getBotPoseEstimate_wpiBlue(limeName);
            if(tempCache == null){
                continue;
            }
            boolean acceptCache = true;

            if (estimateCache!=null && estimateCache.tagCount == 1) {
                if (tempCache.tagCount!=1) {
                    acceptCache = false;
                } else if (tempCache.rawFiducials[0].ambiguity < estimateCache.rawFiducials[0].ambiguity) {
                    acceptCache = false;
                }
            }

            if (tempCache.tagCount!=1) {
                acceptCache = false;
            } else {
                if (tempCache.rawFiducials[0].ambiguity < 0.02) acceptCache = false;
                if (tempCache.rawFiducials[0].distToRobot > 2) acceptCache = false;
            }

            if (acceptCache) {
                estimateCache = tempCache;
            }
        }

        if (estimateCache!=null) {
            poseEstimate = estimateCache;
        } else {
            return null;
        }

        return poseEstimate;
    }


    public static LimeLightHelpers.PoseEstimate getPreviousCachedPoseEstimate() {
        return poseEstimate;
    }

    public static LimeLightHelpers.RawFiducial getClosestTag(boolean toCamera) {
            ArrayList<LimeLightHelpers.RawFiducial> currentTags = getTags();

        LimeLightHelpers.RawFiducial closest = null;
        double closestBy = 99999999;

        for (LimeLightHelpers.RawFiducial tag: currentTags) {
            double distance;

            if (toCamera) {
                distance = tag.distToCamera;
            } else {
                distance = tag.distToRobot;
            }

            System.out.println("TAG DISTANCE " + distance + " Needs to be less than " + closestBy);
            if (distance < closestBy) {
                closestBy = distance;
                closest = tag;
            }
        }
        return closest;
    }

    public static LimeLightHelpers.RawFiducial getClosestTag() {return getClosestTag(false);}

    public static Pose2d getTagPose(LimeLightHelpers.RawFiducial tag) {
       return getTagPose(tag.id);
    }

    public static Pose2d getTagPose() {return getTagPose(getClosestTag());}
    
    public static Pose2d getTagPose(int tagNumber) {
        Pose2d pose = (Pose2d) Constants.Location.TAG_PROPERTIES.get(tagNumber);

        if (pose == null) { return new Pose2d(); }

        return pose;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        LimeLightHelpers.PoseEstimate poseEstimate = getPoseEstimate();
        if (poseEstimate == null) {return;}

        Pose2d pose = poseEstimate.pose;
        if(pose != null){
            builder.addDoubleProperty("rpx", () -> pose.getX(), null);
            builder.addDoubleProperty("rpy", () -> pose.getY(), null);
            builder.addDoubleProperty("rpangle", () -> pose.getRotation().getDegrees(), null);
        }
        else {
            builder.addDoubleProperty("rpx", () -> Double.NaN, null);
            builder.addDoubleProperty("rpy", () -> Double.NaN, null);
            builder.addDoubleProperty("rpangle", () -> Double.NaN, null);
        }
    }
    
}
