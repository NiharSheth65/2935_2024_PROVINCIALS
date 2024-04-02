// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PhotonCommands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.autoConstants;
import frc.robot.Constants.photonVisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonSubsystem;

public class photonDriveToTagWithID extends Command {

  private DriveSubsystem DRIVE_SUBSYSTEM; 
  private PhotonSubsystem PHOTON_SUBSYSTEM; 

  private double driveSetpoint; 

  private double driveSpeed;

  private double bestTargetPitch; 
  private boolean endCommand;

  private PIDController driveController; 

  private double totalTimeSinceLastSeen; 
  private double totalRunTime; 
  
  private int targetId; 

  private boolean photonDriveEnded; 

  private int positionReached; 

  private SlewRateLimiter driveLimiter = new SlewRateLimiter(DriveConstants.driveSlew); 
  /** Creates a new photonDriveToTagWithID. */
  public photonDriveToTagWithID(PhotonSubsystem photon, DriveSubsystem drive, double targetPitch, int targetId, boolean endCommand) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSetpoint = targetPitch; 
    this.DRIVE_SUBSYSTEM = drive; 
    this.PHOTON_SUBSYSTEM = photon;
    this.targetId = targetId; 
    this.endCommand = endCommand; 

    this.driveController = new PIDController(photonVisionConstants.driveKp, photonVisionConstants.driveKi, photonVisionConstants.driveKd); 

    addRequirements(DRIVE_SUBSYSTEM);
    addRequirements(PHOTON_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("THE PHOTON DRIVE COMMAND HAS STARTED");
    driveController.reset();
    driveSpeed = 0; 
    totalRunTime = System.currentTimeMillis(); 
    DRIVE_SUBSYSTEM.resetEncoders(); 
    photonDriveEnded = false; 
    positionReached = 0; 
    bestTargetPitch = 0;

  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    PhotonTrackedTarget bestTarget = null; 

    if(PHOTON_SUBSYSTEM.photonHasTargets()){
      if(PHOTON_SUBSYSTEM.getBestTarget(targetId) != null){
        bestTarget = PHOTON_SUBSYSTEM.getBestTarget(targetId); 
        bestTargetPitch = PHOTON_SUBSYSTEM.getBestTargetPitch(); 
        totalTimeSinceLastSeen = System.currentTimeMillis(); 
      }//else{}
    }
    // else{
    //   bestTargetPitch = -2; 
    // }

    driveSpeed = -driveController.calculate(bestTargetPitch, driveSetpoint);

    if(driveSpeed > photonVisionConstants.photonMaxDriveSpeed){
      driveSpeed = photonVisionConstants.photonMaxDriveSpeed; 
    }

    else if(driveSpeed < -photonVisionConstants.photonMaxDriveSpeed){
      driveSpeed = -photonVisionConstants.photonMaxDriveSpeed; 
    }

    if(bestTargetPitch > driveSetpoint){
      photonDriveEnded = true; 
    }


    DRIVE_SUBSYSTEM.setTank(driveLimiter.calculate(driveSpeed), driveLimiter.calculate(driveSpeed));


    System.out.println("CURRENT PITCH: " +  bestTargetPitch); 

    if(bestTargetPitch > driveSetpoint){
      positionReached++; 
      System.out.println("POSITOIN REACHED: "  + positionReached); 
    }

    // else{
    //   positionReached = 0; 
    // }


    SmartDashboard.putBoolean("photon drive ended", photonDriveEnded); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("THE PHOTON DRIVE COMMAND HAS ENDED");
    autoConstants.distanceDrivenDurringPhoton = DRIVE_SUBSYSTEM.getAverageEncoderDistanceInInches(); 
    DRIVE_SUBSYSTEM.stop(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(endCommand == true){
      return true; 
    }

    if(bestTargetPitch > driveSetpoint && positionReached >= 20){
      System.out.println("PHOTON PITCH: " + bestTargetPitch); 
      System.out.println("POSITION REACHED" + positionReached); 
      return true; 
    }

    // else if(Math.abs(System.currentTimeMillis() - totalTimeSinceLastSeen) > photonVisionConstants.photonTargetAcquiredTimeOut){
    //   return true; 
    // }

    else if(Math.abs(System.currentTimeMillis() - totalRunTime) > photonVisionConstants.photonTurnTargetingTimeOut){
      return true; 
    }

    else{
      return false;
    }
    
  }
}
