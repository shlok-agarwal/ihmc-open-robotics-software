package us.ihmc.simulationConstructionSetTools.robotController;

import us.ihmc.robotics.robotController.RobotController;

public abstract class SliderBoardRobotController implements RobotController
{
   public abstract void initializeSliders();

   public abstract void register();
}
