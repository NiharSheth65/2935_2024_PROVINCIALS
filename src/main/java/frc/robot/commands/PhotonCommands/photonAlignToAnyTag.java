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

public class photonAlignToAnyTag extends Command {

  private DriveSubsystem DRIVE_SUBSYSTEM; 
  private PhotonSubsystem PHOTON_SUBSYSTEM; 

  private double alignmentSetpoint; 

  private double rotationSpeed;

  private double bestTargetYaw; 
  private boolean endCommand;

  private PIDController turnController; 

  private double totalTimeSinceLastSeen; 
  private double totalRunTime; 

  private SlewRateLimiter turnLimiter = new SlewRateLimiter(DriveConstants.turnSlew); 
  /** Creates a new photonAlignToAnyTag. */
  public photonAlignToAnyTag(PhotonSubsystem photon, DriveSubsystem drive, double targetAngle, boolean endCommand) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.alignmentSetpoint = targetAngle; 
    this.DRIVE_SUBSYSTEM = drive; 
    this.PHOTON_SUBSYSTEM = photon;
    this.endCommand = endCommand; 

    this.turnController = new PIDController(photonVisionConstants.turnKp, photonVisionConstants.turnKi, photonVisionConstants.turnKd); 

    addRequirements(DRIVE_SUBSYSTEM);
    addRequirements(PHOTON_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController.reset();
    rotationSpeed = 0; 


    totalRunTime = System.currentTimeMillis(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(PHOTON_SUBSYSTEM.photonHasTargets()){
      bestTargetYaw = PHOTON_SUBSYSTEM.getYaw(); 
    }else{
      bestTargetYaw = 0; 
    }

    rotationSpeed = turnController.calculate(bestTargetYaw, alignmentSetpoint);

    if(rotationSpeed > photonVisionConstants.photonMaxTurnSpeed){
      rotationSpeed = photonVisionConstants.photonMaxTurnSpeed; 
    }

    else if(rotationSpeed < -photonVisionConstants.photonMaxTurnSpeed){
      rotationSpeed = -photonVisionConstants.photonMaxTurnSpeed; 
    }


    DRIVE_SUBSYSTEM.setTank(turnLimiter.calculate(rotationSpeed), -turnLimiter.calculate(rotationSpeed));

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

    else if(Math.abs(PHOTON_SUBSYSTEM.getYaw() - alignmentSetpoint) < photonVisionConstants.photonTightTolerance){
      return true; 
    }

    else{
      return false;
    }
  }
}
