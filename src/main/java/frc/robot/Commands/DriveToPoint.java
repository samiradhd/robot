package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain.Drive;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

public class DriveToPoint extends Command {
  private Drive m_drive;
  private Pose2d m_targetPose;
  private Joystick joyDrive = new Joystick(0);
  private Timer ourTimer = new Timer();
  private ProfiledPIDController m_driveController =
      new ProfiledPIDController(
          Constants.Drive.DriveToPoint.kDriveToPointP,
          Constants.Drive.DriveToPoint.kDriveToPointI,
          Constants.Drive.DriveToPoint.kDriveToPointD,
          new TrapezoidProfile.Constraints(
              Constants.Drive.DriveToPoint.kMaxLinearSpeed, Constants.Drive.DriveToPoint.kMaxLinearAcceleration));

  private ProfiledPIDController m_headingController =
      new ProfiledPIDController(
            Constants.Drive.DriveToPoint.kDriveToPointHeadingP,
            Constants.Drive.DriveToPoint.kDriveToPointHeadingI,
            Constants.Drive.DriveToPoint.kDriveToPointHeadingD,
          new TrapezoidProfile.Constraints(
              Constants.Drive.DriveToPoint.kMaxAngularSpeed, Constants.Drive.DriveToPoint.kMaxAngularAcceleration));

  private double m_ffMinRadius = 0.2, m_ffMaxRadius = 0.8;

  public DriveToPoint(Drive drive, Pose2d targetPose) {
    m_drive = drive;
    m_targetPose = targetPose;
    addRequirements(drive);

    m_headingController.enableContinuousInput(-Math.PI, Math.PI);

    m_driveController.setTolerance(Constants.Drive.DriveToPoint.metersTolerance);
    m_headingController.setTolerance(Constants.Drive.DriveToPoint.radiansTolerance);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = m_drive.getPose();

    if (m_targetPose == null) { return; }
    if (m_targetPose.getX() == 0 && m_targetPose.getY() == 0) { return; }
    ChassisSpeeds fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            m_drive.getChassisSpeeds(), currentPose.getRotation());
    m_driveController.reset(
        currentPose.getTranslation().getDistance(m_targetPose.getTranslation()),
        Math.min(
            0.0,
            -new Translation2d(fieldRelative.vxMetersPerSecond, fieldRelative.vyMetersPerSecond)
                .rotateBy(
                    m_targetPose
                        .getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    m_headingController.reset(
        currentPose.getRotation().getRadians(), fieldRelative.omegaRadiansPerSecond);

    ourTimer.start();
    ourTimer.reset();
    }

  @Override
  public void execute() {

    if (isFinished()) return;

    Pose2d currentPose = m_drive.getPose();

    if (m_targetPose == null) { return; }
    if (m_targetPose.getX() == 0 && m_targetPose.getY() == 0) { return; }

    double currentDistance =
        currentPose.getTranslation().getDistance(m_targetPose.getTranslation());

    double ffScaler =
        MathUtil.clamp(
            (currentDistance - m_ffMinRadius) / (m_ffMaxRadius - m_ffMinRadius), 0.0, 1.0);
    double driveVelocityScalar =
        m_driveController.getSetpoint().velocity * ffScaler
            + m_driveController.calculate(currentDistance, 0.0);
    if (currentDistance < m_driveController.getPositionTolerance()) {
      driveVelocityScalar = 0.0;
    }

    double headingError = currentPose.getRotation().minus(m_targetPose.getRotation()).getRadians();
    double headingVelocity =
        m_headingController.getSetpoint().velocity * ffScaler
            + m_headingController.calculate(
                currentPose.getRotation().getRadians(), m_targetPose.getRotation().getRadians());

    if (Math.abs(headingError) < m_headingController.getPositionTolerance()) {
      headingVelocity = 0.0;
    }

    // evil math
    // blame 254 for making this because i dont fully understand it
    Translation2d driveVelocity =
        new Pose2d(
                0.0,
                0.0,
                currentPose.getTranslation().minus(m_targetPose.getTranslation()).getAngle())
            .transformBy(new Transform2d(driveVelocityScalar, 0.0, new Rotation2d()))
            .getTranslation();
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), -headingVelocity, currentPose.getRotation());
    // ChassisSpeeds speeds = new ChassisSpeeds(driveVelocity.getX(), driveVelocity.getY(), headingVelocity);

    m_drive.setRobotSpeeds(speeds);

    Logger.recordOutput("DriveToPoint/TargetPose", m_targetPose);
    Logger.recordOutput("DriveToPoint/DriveDistance", currentDistance);
    Logger.recordOutput("DriveToPoint/HeadingError", headingError);

    Logger.recordOutput("DriveToPoint/DriveVelocityScalar", driveVelocityScalar);
    Logger.recordOutput("DriveToPoint/HeadingVelocity", headingVelocity);
    Logger.recordOutput("DriveToPoint/DriveVelocityX", driveVelocity.getX());
    Logger.recordOutput("DriveToPoint/DriveVelocityY", driveVelocity.getY());

    Logger.recordOutput("DriveToPoint/DriveSpeeds", speeds);

    m_drive.states.targetCache = m_targetPose;
    m_drive.states.driveToPoitnEstimate = currentPose;
  }

  @Override
  public boolean isFinished() {
    if (DriverStation.isAutonomous() && ourTimer.hasElapsed(1.75)) return true;
    if (joyDrive.getRawAxis(0)>0.2 || joyDrive.getRawAxis(1) > 0.2 || joyDrive.getRawAxis(2)>0.2 || joyDrive.getRawAxis(3) > 0.2) return true;
    if (m_targetPose == null) { return true; }
    if (m_targetPose.getX() == 0 && m_targetPose.getY() == 0) { return true; }

    System.out.println("still waiting");
    Translation2d finalPose = m_drive.getPose().getTranslation().minus(m_targetPose.getTranslation());
    System.out.println("Difference: " + finalPose.getX() + " / " + finalPose.getY() + " / " + finalPose.getAngle().getRadians());

    return m_driveController.atGoal() && m_headingController.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.setRobotSpeeds(new ChassisSpeeds());
    ourTimer.stop();
  }
}