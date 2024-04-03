// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PhotonCommands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.autoConstants;
import frc.robot.Constants.photonVisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonSubsystem;

public class photonDriveToTrap extends Command {

  private DriveSubsystem DRIVE_SUBSYSTEM; 
  private PhotonSubsystem PHOTON_SUBSYSTEM; 

  private double driveSetpoint; 

  private double driveSpeed;

  private double bestTargetPitch; 
  private boolean endCommand;

  private PIDController driveController; 

  private double totalTimeSinceLastSeen; 
  private double totalRunTime; 
  
  private double inPosition = 0; 

  private SlewRateLimiter driveLimiter = new SlewRateLimiter(DriveConstants.driveSlew); 

  /** Creates a new photonDriveToTrap. */
  public photonDriveToTrap(PhotonSubsystem photon, DriveSubsystem drive, double targetPitch, boolean endCommand) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSetpoint = targetPitch; 
    this.DRIVE_SUBSYSTEM = drive; 
    this.PHOTON_SUBSYSTEM = photon;
    this.endCommand = endCommand; 

    this.driveController = new PIDController(0.175, photonVisionConstants.driveKi, photonVisionConstants.driveKd); 

    addRequirements(DRIVE_SUBSYSTEM);
    addRequirements(PHOTON_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveController.reset();
    driveSpeed = 0; 
    inPosition = 0; 
    totalRunTime = System.currentTimeMillis(); 
    DRIVE_SUBSYSTEM.resetEncoders(); 
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

    if(driveSpeed > photonVisionConstants.photonTrapDriveSpeed){
      driveSpeed = photonVisionConstants.photonTrapDriveSpeed; 
    }

    else if(driveSpeed < -photonVisionConstants.photonTrapDriveSpeed){
      driveSpeed = -photonVisionConstants.photonTrapDriveSpeed; 
    }


    if(Math.abs(PHOTON_SUBSYSTEM.getPitch() - driveSetpoint) < 0.5){
      inPosition++; 
    }

    DRIVE_SUBSYSTEM.setTank(driveLimiter.calculate(driveSpeed), driveLimiter.calculate(driveSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    autoConstants.distanceDrivenDurringPhoton = DRIVE_SUBSYSTEM.getAverageEncoderDistanceInInches(); 
    DRIVE_SUBSYSTEM.stop(); 
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(endCommand == true){
      return true; 
    }

    else if(Math.abs(PHOTON_SUBSYSTEM.getPitch() - driveSetpoint) < 1 && inPosition > 30){
      return true; 
    }


    // else if(Math.abs(System.currentTimeMillis() - totalTimeSinceLastSeen) > photonVisionConstants.photonTargetAcquiredTimeOut){
    //   return true; 
    // }

    // else if(Math.abs(System.currentTimeMillis() - totalRunTime) > photonVisionConstants.photonTurnTargetingTimeOut){
    //   return true; 
    // }

    else{
      return false;
    }
  }
}
