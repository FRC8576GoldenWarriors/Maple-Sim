package frc.robot;

import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static class SwerveConstants{
        public static double xCoeffecient = 2.25*1.2;
        public static double yCoefficient = 2.25*1.2;
        public static double rotCoefficinet = 1.75*2;
    }

    public static class PathPlannerConstants {
        public static PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));
    }

    public static class GameObjectIDConstants {
        public static final List<Integer> REEF_TAG_IDS =
            Arrays.asList(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
        public static final List<Integer> BARGE_TAG_IDS = Arrays.asList(4, 5, 14, 15);
        public static final List<Integer> PROCESSOR_TAG_IDS = Arrays.asList(3, 16);
        public static final List<Integer> CORAL_STATION_TAG_IDS = Arrays.asList(1, 2, 12, 13);
    }
}
