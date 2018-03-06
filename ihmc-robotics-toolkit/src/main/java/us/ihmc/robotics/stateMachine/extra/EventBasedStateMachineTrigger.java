package us.ihmc.robotics.stateMachine.extra;

public interface EventBasedStateMachineTrigger
{
   <E extends Enum<E>> void trigger(E event);
}
