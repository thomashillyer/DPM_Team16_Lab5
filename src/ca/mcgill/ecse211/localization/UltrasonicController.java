package ca.mcgill.ecse211.localization;

public interface UltrasonicController {

  public void processUSData(int distance);

  public int readUSDistance();
}
