package frc.robot.util.filters;

public interface ValueFilter {
    public void reset();
    public double filter(double value);
    public double get();
}