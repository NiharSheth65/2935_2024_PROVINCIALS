// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommandBlocks;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.photonVisionConstants;
import frc.robot.commands.PhotonCommands.photonAlignToTrap;
import frc.robot.commands.PhotonCommands.photonDriveToTrap;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoScoreTrap extends SequentialCommandGroup {
  /** Creates a new autoScoreTrap. */
  public autoScoreTrap(DriveSubsystem drive, PhotonSubsystem photon, ShooterSubsystem shooter,  ConveyerSubsystem conveyer){
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      
      new ParallelDeadlineGroup(

        new SequentialCommandGroup(
          new photonAlignToTrap(photon, drive, photonVisionConstants.photonTrapTargetYaw, false), 
          new photonDriveToTrap(photon, drive, photonVisionConstants.photonTrapTargetPitch, false), 
          new photonAlignToTrap(photon, drive, photonVisionConstants.photonTrapTargetYaw, false)
        ), 

        new autoRevUpCommand(shooter, ShooterConstants.speakerTopMotorSpeed, ShooterConstants.speakerBottomMotorSpeed)
      ), 

      new autoShootNoteCommand(drive, photon, conveyer, 0)


    );
  }
}
