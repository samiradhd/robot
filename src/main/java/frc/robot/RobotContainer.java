// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.DriveManual;
import frc.robot.Commands.FollowPath;
import frc.robot.Constants.Coral;
import frc.robot.Subsystems.Algae.Algae;
import frc.robot.Subsystems.Algae.AlgaeDescore;
import frc.robot.Subsystems.Cage.Cage;
import frc.robot.Subsystems.Coral.CoralElevator;
import frc.robot.Subsystems.Coral.CoralEndEffector;
import frc.robot.Subsystems.Drivetrain.Drive;
import frc.robot.Utilities.PoseBasedLocator;
import frc.robot.Commands.AlgaeIntake;
import frc.robot.Commands.AlgaeScore;
import frc.robot.Commands.AlignCommands;
import frc.robot.Commands.CoralIntake;
import frc.robot.Commands.CoralIntakeOutake;
import frc.robot.Commands.CoralScore;
import frc.robot.Commands.DescoreAlgae;
import frc.robot.Commands.CageManual;
import frc.robot.Commands.ElevatorPositon;



public class RobotContainer {
  private Joystick joyDrive = new Joystick(0);
  private Joystick joyOpperator = new Joystick(1);
  
  private Drive drive = new Drive();
  private static CoralElevator elevator = new CoralElevator(Constants.Coral.elevatorCANID );
  private static CoralEndEffector effector = new CoralEndEffector(Constants.Coral.endEffectorCANID);
  private static Algae algae = new Algae(Constants.Algae.anglerCANID, Constants.Algae.intakeCANID);
  private static AlgaeDescore descore = new AlgaeDescore(Constants.Algae.descoreCANID);
  private static Cage cage = new Cage(Constants.Cage.cageCANID);

  private PoseBasedLocator poseLocator = new PoseBasedLocator(drive);
  private DriveManual teleopDriveCmd = new DriveManual(drive, joyDrive);
  private CageManual cageman = new CageManual(cage, joyOpperator);
  private DescoreAlgae descorman = new DescoreAlgae(descore, joyOpperator);


  @SuppressWarnings("unused")
  private UsbCamera camera = CameraServer.startAutomaticCapture();
  

  public RobotContainer() {
    configureBindings();
    // Log telemetry to SmartDashboard and Shuffleboard
    drive.log();

   drive.manualDrive = teleopDriveCmd;
   drive.poseBasedLocator = poseLocator;

   drive.setDefaultCommand(teleopDriveCmd);
   cage.setDefaultCommand(cageman); // pov up to pull up pov down to pull down
   descore.setDefaultCommand(descorman);
    
    configureNamedCommands();

    // updateAutoSelector must be the last thing called !!!
    FollowPath.updateAutoSelector(drive);
  }

  private void configureNamedCommands(){

    // auto

    for (int x = 1; x < 23; x++) {
      NamedCommands.registerCommand("LeftCoral"+(x), AlignCommands.goToTag(drive, x, Constants.Location.FRONT_REEF_LEFT_OFFSET));
      NamedCommands.registerCommand("RightCoral"+(x), AlignCommands.goToTag(drive, x, Constants.Location.FRONT_REEF_RIGHT_OFFSET));
    }

    // NamedCommands.registerCommand("LeftCoralToClosestTag", AlignCommands.frontAlignToReefLeft(drive));
    // NamedCommands.registerCommand("RightCoralToClosestTag", AlignCommands.frontAlignToReefRight(drive));

    NamedCommands.registerCommand("IntakePosition", new ElevatorPositon(elevator, Constants.Coral.intakePos));

    NamedCommands.registerCommand("CoralScoreL1", new CoralScore(elevator, effector, Constants.Coral.l1Pos));
    NamedCommands.registerCommand("CoralScoreL2", new CoralScore(elevator, effector, Constants.Coral.l2Pos));
    NamedCommands.registerCommand("CoralScoreL3", new CoralScore(elevator, effector, Constants.Coral.l3Pos));

    NamedCommands.registerCommand("CoralL1", new ElevatorPositon(elevator, Constants.Coral.l1Pos));
    NamedCommands.registerCommand("CoralL2", new ElevatorPositon(elevator, Constants.Coral.l2Pos));
    NamedCommands.registerCommand("CoralL3", new ElevatorPositon(elevator, Constants.Coral.l3Pos));

    NamedCommands.registerCommand("ScoreCoral", new CoralIntakeOutake(effector));

    NamedCommands.registerCommand("YieldIntake", new CoralIntake(elevator, effector));

    
  }

  public Command getAutonomousCommand() {
    return FollowPath.autoChooser.getSelected();
  }

  private void configureBindings() {
    // left and right to reef
    new Trigger(() -> joyDrive.getRawButton(1)).debounce(.1).onTrue(AlignCommands.frontAlignToReefLeft(drive));
    new Trigger(() -> joyDrive.getRawButton(3)).debounce(.1).onTrue(AlignCommands.frontAlignToReefRight(drive));

    // // up to algae processor
    new Trigger(() -> joyDrive.getRawButton(4)).debounce(.1).onTrue(AlignCommands.alignToProcessor(drive));

    // // center coral station
    new Trigger(() -> joyDrive.getRawButton(2)).debounce(.1).onTrue(AlignCommands.backAlignToCoralStationCenter(drive));
    new Trigger(() -> joyDrive.getPOV() == 270).debounce(.1).onTrue(AlignCommands.backAlignToCoralStationLeft(drive));
    new Trigger(() -> joyDrive.getPOV() == 90).debounce(.1).onTrue(AlignCommands.backAlignToCoralStationRight(drive));

    // Left and right L1 coral alignments
    // new Trigger(() -> joyDrive.getPOV() == 0).debounce(.1).onTrue(AlignCommands.backAlignToCoralStationRight(drive));
    // new Trigger(() -> joyDrive.getPOV() == 180).debounce(.1).onTrue(AlignCommands.backAlignToCoralStationRight(drive));


    new Trigger(() -> joyOpperator.getRawButton(5)).debounce(.1).whileTrue(new AlgaeIntake(algae));
    new Trigger(() -> joyOpperator.getRawButton(6)).debounce(.1).whileTrue(new AlgaeScore(algae));

    new Trigger(() -> joyOpperator.getRawButton(1)).debounce(.1).onTrue(new CoralIntakeOutake(effector));
    new Trigger(() -> joyOpperator.getRawButton(2)).debounce(.1).onTrue(new ElevatorPositon(elevator, Constants.Coral.l1Pos));
    new Trigger(() -> joyOpperator.getRawButton(3)).debounce(.1).onTrue(new ElevatorPositon(elevator, Constants.Coral.l2Pos));
    new Trigger(() -> joyOpperator.getRawButton(4)).debounce(.1).onTrue(new ElevatorPositon(elevator, Constants.Coral.l3Pos));

  }
  

}
