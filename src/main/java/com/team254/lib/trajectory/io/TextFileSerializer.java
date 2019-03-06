package com.team254.lib.trajectory.io;

import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.Trajectory.Segment;
import com.team254.lib.trajectory.Path;

/**
 * Serializes a Path to a simple space and CR separated text file.
 * 
 * @author Jared341
 */
public class TextFileSerializer {

  /**
   * Format:
   *   PathName
   *   NumSegments
   *   LeftSegment1
   *   ...
   *   LeftSegmentN
   *   RightSegment1
   *   ...
   *   RightSegmentN
   * 
   * Each segment is in the format:
   *   pos vel acc jerk heading dt x y
   * 
   * @param path The path to serialize.
   * @return A string representation.
   */
  public String[] serialize(Path path) {
    String[] serialized = new String[3];

    serialized[0] = serializeTrajectory(path.getLeftTrajectory());
    serialized[1] = serializeTrajectory(path.getTrajectory());
    serialized[2] = serializeTrajectory(path.getRightTrajectory());

    return serialized;
  }
  
  private String serializeTrajectory(Trajectory trajectory) {
    String content = "position,velocity,acceleration,jerk,heading,dt,x,y\n";
    for (int i = 0; i < trajectory.getNumSegments(); ++i) {
      Segment segment = trajectory.getSegment(i);
      content += String.format(
              "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", 
              segment.pos, segment.vel, segment.acc, segment.jerk,
              segment.heading, segment.dt, segment.x, segment.y);
    }
    return content;
  }
  
}
