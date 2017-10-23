package ca.mcgill.ecse211.zipline;

public interface UltrasonicController {

	public void processUSData(int distance);

	public int readUSDistance();
}
