package frc.robot.common.utilities;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.common.config.ColorSensorConfig;

public class ColorSensor {
  private final ColorSensorV3 colorSensor;
  private final ColorMatch colorMatcher = new ColorMatch();
  private static final Color kRedTarget = new Color(.4, .3, .1);
  private static final Color kBlueTarget = new Color(.1, .5, .4);
  private static final Color kBlack = new Color(.1, .1, .1);
  private static final Color kWhite = new Color(.7, .7, .7);
  private static final Color kGreen = new Color(.3, .7, .4);
  private static final Color kYellow = new Color(.5, .6, .2);
  private static final Color kGrey = new Color(.3, .3, .3);


  private static final double blinkinRed = 0.61;
  private static final double blinkinBlue = 0.87;
  private static final double blinkinBlack = 0.99;

  public ColorSensor(ColorSensorConfig cfg) {
    colorSensor = new ColorSensorV3(cfg.port);
    
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kBlack);
    colorMatcher.addColorMatch(kWhite);
    colorMatcher.addColorMatch(kGreen);
    colorMatcher.addColorMatch(kYellow);
    colorMatcher.addColorMatch(kGrey);
  }

  public MatchColor ReadSensor() {
    var detectedColor = colorSensor.getColor();
    var match = colorMatcher.matchClosestColor(detectedColor);

    var resultColor = MatchColor.None;
    if (match.color == kRedTarget) {
      resultColor = MatchColor.Red;
    } else if (match.color == kBlueTarget) {
      resultColor = MatchColor.Blue;
    }

    return resultColor;
  }

  public enum MatchColor {
    None,
    Red,
    Blue
  }

  public static double ToBlinkin(MatchColor color) {
    var blinkIncolor = blinkinBlack;
    if (color == MatchColor.Blue) {
      blinkIncolor = blinkinBlue;
    }
    if (color == MatchColor.Red) {
      blinkIncolor = blinkinRed;
    }

    return blinkIncolor;
  }
}
