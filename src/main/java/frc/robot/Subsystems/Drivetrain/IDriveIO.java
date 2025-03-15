package frc.robot.Subsystems.Drivetrain;

import java.util.Calendar;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;


public interface IDriveIO {
    @AutoLog
    public class DriveStates{
        double gyroAngleDeg;
        double[] odomPoses = {0, 0, 0};
        double[] lastCachedOdomPosePoint = {0,0,0};
        
        double[] chassisSpeeds = new double[3];

        double[] targetSpeeds = {0,0,0};
        double[] targetAutoSpeeds = {0, 0, 0};

        boolean recievedNewControls;

        double cacheTime = 0;
        double lastUpdatedTime = (double) (Calendar.getInstance().getTimeInMillis() / 1000);
        Pose2d position;
        
        public Pose2d swerveControllerTarget;
        public Pose2d targetCache;
        public Pose2d driveToPoitnEstimate;
    }
    public void updateDriveStates(DriveStatesAutoLogged states);
}
