package us.ihmc.robotics.stateMachine.extra;

public interface StateTransitionCondition
{
   boolean testCondition(double timeInCurrentState);
}
