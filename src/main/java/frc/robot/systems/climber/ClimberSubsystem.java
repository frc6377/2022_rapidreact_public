package frc.robot.systems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
  private TalonFX climberLead = null;
  private ClimberHardware CH = null;
  private ClimberConfig cfg;
  private double manualSpeed;
  private int step = 0;
  private int climbStart = 0;
  public Command[] climbSeq;
  public boolean underAutoControl = false;

  public ClimberSubsystem(ClimberHardware CH, ClimberConfig cfg) {
    climberLead = CH.climberLead;
    manualSpeed = cfg.manualVelocity;
    this.cfg = cfg;
    this.CH = CH;

    addChild("Lead", this.CH.climberLead);
    addChild("Follower", this.CH.climberFollow);

  }

  // --------- Auto-move ---------

  public Command getNextClimbStep(){
    if(step < 0 || step > climbSeq.length) return null;
    return climbSeq[step];
  }

  public void setClimbState(int state){
    if(state < climbSeq.length || state < 0) return;
    step = state;
  }

  public void setClimbStart(int climbStart){
    this.climbStart = climbStart;
  }

  public void resetClimb(){
    step = climbStart;
  }

  public void incrementClimb(){
    if(step + 1 >= climbSeq.length) return;
    step += 1;
  }

  public void decrementClimb(){
    if(step - 1 < 0) return;
    step -= 1;
  }

  // --------- "Manual" Controls ---------

  public void changeManualSpeed(double delta){
    if(manualSpeed + delta > cfg.maxManualSpeed || manualSpeed + delta < cfg.minimumManualSpeed){
      return;
    }

    manualSpeed += delta;
  }

  public void setPos(double position){
    climberLead.set(ControlMode.MotionMagic, position);    
  }

  public void setVelocity(double vel){
    climberLead.set(ControlMode.Velocity, vel);
  }

  /**
   * Affects motion magic speed.
   * @param sped - If too go fast or slow
   */
  public void setSpeed(ClimberSpeed sped){
    CH.setSpeed(sped);
  }

  /**
   * Set the velocity of the climber
   * Manual moves the climber up or down at the given velocity.
   */
  public void manualControl(ClimberDirection dir){
    switch(dir){
      case DOWN:
       climberLead.set(ControlMode.Velocity, -manualSpeed);  
      break;
      case STAY:
        climberLead.set(ControlMode.Position, getMotorPos());
      break;
      case UP:
        climberLead.set(ControlMode.Velocity, manualSpeed);
        break;
      default:
        break;
    }
  }

  //--------- Acccessors ---------
  
  public void setClimbSeq(Command[] climbSeq){
    this.climbSeq = climbSeq;
  }

  public void setPosition(double pos){
    climberLead.set(ControlMode.MotionMagic, pos);
  }

  public double getMotorPos(){
    return climberLead.getSelectedSensorPosition();
  }

  public boolean isLimitSwitchHit(){
    return climberLead.isRevLimitSwitchClosed() == 1;
  }

  public boolean isExtended() {
    return Math.abs(climberLead.getSelectedSensorPosition() - cfg.extendedMotorPos) < cfg.allowedError;
  }

  public boolean atPoint(double point){
    return Math.abs(climberLead.getSelectedSensorPosition() - point) < cfg.allowedError;
  }

  public boolean atSetPoint() {
    return Math.abs(climberLead.getSelectedSensorPosition()) < cfg.allowedError;
  }

  public double getCurrent() {
    return climberLead.getStatorCurrent();
  }

  public boolean currentAbove(double trip){
    return climberLead.getStatorCurrent() > trip;
  }

  public void setZero(){
    climberLead.setSelectedSensorPosition(0);
  }

  //--------- Enums ---------

  public enum ClimberSpeed{
    SLOW,
    FAST;
  }

  public enum ClimberDirection{
    UP,
    DOWN,
    STAY
  }

  public enum ClimberPosition{
    LOWER,
    UPPER,
    TRAVERSAL_PAUSE,
    TRAVERSAL_CLIMB,
    SWINGING,
    EXTEND,
    RETRACT;
  }

}
