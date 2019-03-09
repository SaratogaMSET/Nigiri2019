package com.team254.lib.trajectory.io;

import com.team254.lib.trajectory.Path;
import java.util.StringTokenizer;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.Trajectory.Segment;

/**
 *
 * @author Jared341
 */
public class TextFileDeserializer {

  public Trajectory deserialize(String serialized) {
    StringTokenizer tokenizer = new StringTokenizer(serialized, "\n");
    System.out.println("Parsing path string...");
    System.out.println("String has " + serialized.length() + " chars");
    System.out.println("Found " + tokenizer.countTokens() + " tokens");
    tokenizer.nextToken();
    int len =  tokenizer.countTokens()-1;
    Trajectory t = new Trajectory(len);
    for (int i = 0; i < len; ++i) {
      Trajectory.Segment segment = new Trajectory.Segment();
      StringTokenizer line_tokenizer = new StringTokenizer(
              tokenizer.nextToken(), ", ");
      
      segment.pos = Double.parseDouble(line_tokenizer.nextToken());
      segment.vel = Double.parseDouble(line_tokenizer.nextToken());
      segment.acc = Double.parseDouble(line_tokenizer.nextToken());
      segment.jerk = Double.parseDouble(line_tokenizer.nextToken());
      segment.heading = Double.parseDouble(line_tokenizer.nextToken());
      segment.dt = Double.parseDouble(line_tokenizer.nextToken());
      segment.x = Double.parseDouble(line_tokenizer.nextToken());
      segment.y = Double.parseDouble(line_tokenizer.nextToken());
      System.out.println(segment.vel);
      t.setSegment(i, segment);
    }
    
    
    System.out.println("...finished parsing path from string.");
    return t;
  }
  
}
