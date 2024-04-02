// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PhotonCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.photonVisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonSubsystem;

public class photonDriveToAnyTag extends Command {

  private DriveSubsystem DRIVE_SUBSYSTEM; 
  private PhotonSubsystem PHOTON_SUBSYSTEM; 

  private double driveSetpoint; 

  private double driveSpeed;

  private double bestTargetPitch; 
  private boolean endCommand;

  private PIDController driveController; 

  private double totalTimeSinceLastSeen; 
  private double totalRunTime; 

  private SlewRateLimiter driveLimiter = new SlewRateLimiter(DriveConstants.driveSlew); 

  /** Creates a new photonDriveToAnyTag. */
  public photonDriveToAnyTag(PhotonSubsystem photon, DriveSubsystem drive, double targetPitch, boolean endCommand) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSetpoint = targetPitch; 
    this.DRIVE_SUBSYSTEM = drive; 
    this.PHOTON_SUBSYSTEM = photon;
    this.endCommand = endCommand; 

    this.driveController = new PIDController(photonVisionConstants.driveKp, photonVisionConstants.driveKi, photonVisionConstants.driveKd); 

    addRequirements(DRIVE_SUBSYSTEM);
    addRequirements(PHOTON_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveController.reset();
    driveSpeed = 0; 


    totalRunTime = System.currentTimeMillis(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(PHOTON_SUBSYSTEM.photonHasTargets()){
      bestTargetPitch = PHOTON_SUBSYSTEM.getPitch(); 
    }else{
      bestTargetPitch = photonVisionConstants.speakerMiddleApproachPitch; 
    }

    driveSpeed = -driveController.calculate(bestTargetPitch, driveSetpoint);

    if(driveSpeed > photonVisionConstants.photonMaxDriveSpeed){
      driveSpeed = photonVisionConstants.photonMaxDriveSpeed; 
    }

    else if(driveSpeed < -photonVisionConstants.photonMaxDriveSpeed){
      driveSpeed = -photonVisionConstants.photonMaxDriveSpeed; 
    }


    DRIVE_SUBSYSTEM.setTank(driveLimiter.calculate(driveSpeed), driveLimiter.calculate(driveSpeed));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DRIVE_SUBSYSTEM.stop(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(endCommand == true){
      return true; 
    }

    else if(PHOTON_SUBSYSTEM.getPitch() > driveSetpoint){
      return true; 
    }
    
    else{
      return false;
    }
  }
}
