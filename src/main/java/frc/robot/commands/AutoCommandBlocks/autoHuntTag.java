// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommandBlocks;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.photonVisionConstants;
import frc.robot.commands.PhotonCommands.photonAlignToAnyTag;
import frc.robot.commands.PhotonCommands.photonDriveToAnyTag;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoHuntTag extends SequentialCommandGroup {
  /** Creates a new autoHuntTag. */
  public autoHuntTag(DriveSubsystem drive, PhotonSubsystem photon) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new photonAlignToAnyTag(photon, drive, 0, false), 
      new photonDriveToAnyTag(photon, drive, photonVisionConstants.speakerMiddleApproachPitch, false), 
      new photonAlignToAnyTag(photon, drive, 0, false)
    );
  }
}
