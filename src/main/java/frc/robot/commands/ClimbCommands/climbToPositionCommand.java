// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.ClimbCommands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.ClimbConstants;
// import frc.robot.subsystems.ClimbSubsystem;

// public class climbToPositionCommand extends Command {
  
//   private ClimbSubsystem CLIMB_SUBSYSTEM; 
//   private PIDController pidController; 
 
//   private double climbSetPoint; 
//   private double measurement; 
  
//   private boolean endCommand; 

//   /** Creates a new climbToPositionCommand. */
//   public climbToPositionCommand(ClimbSubsystem climb, double setpoint, boolean end) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.CLIMB_SUBSYSTEM = climb; 
//     this.pidController = new PIDController(ClimbConstants.climbKp, ClimbConstants.climbKi, ClimbConstants.climbKd); 
//     this.climbSetPoint = setpoint; 
//     this.endCommand = end;

//     addRequirements(CLIMB_SUBSYSTEM);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     pidController.reset();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     double climbEncoderPosition = CLIMB_SUBSYSTEM.getClimbEncoderPosition(); 
//     double climbSpeed = pidController.calculate(climbEncoderPosition, climbSetPoint); 

//     if(climbSpeed > ClimbConstants.climbMaxSpeed){
//       climbSpeed = ClimbConstants.climbMaxSpeed; 
//     }

//     else if(climbSpeed < -ClimbConstants.climbMaxSpeed){
//       climbSpeed = -ClimbConstants.climbMaxSpeed;
//     }

//     CLIMB_SUBSYSTEM.setClimb(climbSpeed);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     CLIMB_SUBSYSTEM.stop();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
    
//     if(Math.abs(CLIMB_SUBSYSTEM.getClimbEncoderPosition() - climbSetPoint) < 2){
//       return true;
//     }

//     else if(endCommand){
//       return true; 
//     }
    
//     else{
//       return false; 
//     }
    
//   }
// }
