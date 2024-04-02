// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoNoteBlocks;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.photonVisionConstants;
import frc.robot.commands.AutoCommandBlocks.autoEatNoteCommand;
import frc.robot.commands.AutoCommandBlocks.autoHuntNoteCommand;
import frc.robot.commands.AutoCommandBlocks.autoHuntSpecificTag;
import frc.robot.commands.AutoCommandBlocks.autoRevUpCommand;
import frc.robot.commands.AutoCommandBlocks.autoShootNoteCommand;
import frc.robot.commands.AutoCommandBlocks.autoSimpleDigestCommand;
import frc.robot.commands.DriveCommands.DriveForwardSetDistance;
import frc.robot.commands.DriveCommands.DriveTurnGyroCommand;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class note3Block extends SequentialCommandGroup {
  /** Creates a new note3Block. */
  public note3Block(DriveSubsystem drive, ShooterSubsystem shooter, ConveyerSubsystem conveyer, LightSubsystem led, PhotonSubsystem photon, VisionSubsystem vision, IntakeSubsystem intake, int targetId) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      
      new DriveForwardSetDistance(drive, 10, DriveConstants.autoSpeed), 
      new DriveTurnGyroCommand(drive, 50, false),
      new DriveForwardSetDistance(drive, 15, DriveConstants.autoSpeed), 
      new autoHuntNoteCommand(drive, vision), 
      new autoEatNoteCommand(drive, conveyer, intake), 
      
      new ParallelCommandGroup(
        new DriveForwardSetDistance(drive, -45, DriveConstants.autoSpeed), 
        new autoSimpleDigestCommand(conveyer, intake, shooter)
      ), 
 
      new DriveTurnGyroCommand(drive, -40, false), 

      new ParallelDeadlineGroup(
        new autoHuntSpecificTag(drive, photon, targetId, 0, photonVisionConstants.speakerMiddleApproachPitch),
        new autoRevUpCommand(shooter, ShooterConstants.speakerTopMotorSpeed, ShooterConstants.speakerBottomMotorSpeed)
      ), 

      // new autoRevUpCommand(shooter, ShooterConstants.speakerTopMotorSpeed, ShooterConstants.speakerBottomMotorSpeed), 
      new DriveForwardSetDistance(drive, -7, DriveConstants.autoSpeed), 
      new autoShootNoteCommand(drive, photon, conveyer, targetId)
    );
  }
}
