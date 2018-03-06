package us.ihmc.robotics.stateMachine.extra;

import us.ihmc.robotics.stateMachine.core.State;

public interface FinishableState extends State
{
   boolean isDone(double timeInState);
}
