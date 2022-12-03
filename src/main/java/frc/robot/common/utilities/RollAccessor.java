package frc.robot.common.utilities;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;


public class RollAccessor {
  private Type type = null;
  private WPI_PigeonIMU pigeon;
  private WPI_Pigeon2 pigeon2;

  /**
   * Make a accessor for a pigeon or a pigeon 2.
   * @param pigeon the pigeon to encapsulate
   */
  public RollAccessor(WPI_PigeonIMU pigeon){
    this.pigeon = pigeon;
    type = Type.PIGEON;
  }

  /**
   * Make a accessor for a pigeon or a pigeon 2.
   * @param pigeon the pigeon to encapsulate
   */
  public RollAccessor(WPI_Pigeon2 pigeon){
    this.pigeon2 = pigeon;
    type = Type.PIGEON2;
  }

  /**
   * Get the pitch from the pigeon.
   * @return the pitch from the pigeon
   */
  public double getRoll(){
    switch(type){
      case PIGEON:
        return pigeon.getPitch(); // CTRE WPI disconect in axises
      case PIGEON2:
        return pigeon2.getPitch();
      default:
        return 0;
    }    
  }

  enum Type{
    PIGEON,
    PIGEON2
  }
}