package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.primitives.HandTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;

public class HandTrajectoryTask extends BehaviorAction
{
   private final HandTrajectoryMessage handTrajectoryMessage;
   private final HandTrajectoryBehavior handTrajectoryBehavior;

   public HandTrajectoryTask(HandTrajectoryMessage handTrajectoryMessage, HandTrajectoryBehavior handTrajectoryBehavior)
   {
      super(handTrajectoryBehavior);
      this.handTrajectoryBehavior = handTrajectoryBehavior;
      this.handTrajectoryMessage = handTrajectoryMessage;
   }

   @Override
   protected void setBehaviorInput()
   {
      handTrajectoryBehavior.setInput(handTrajectoryMessage);
   }
}
