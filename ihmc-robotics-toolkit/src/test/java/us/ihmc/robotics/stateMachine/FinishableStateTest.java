package us.ihmc.robotics.stateMachine;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.extra.FinishableState;
import us.ihmc.robotics.stateMachine.factories.FinishableStateMachineFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FinishableStateTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testExampleStateMachineWithFinishableStates()
   {
      FinishableStateMachineFactory<StateEnum, ExampleFinishableState> factory = new  FinishableStateMachineFactory<>(StateEnum.class);

      ExampleFinishableState stateOne = new ExampleFinishableState(StateEnum.ONE);
      ExampleFinishableState stateTwo = new ExampleFinishableState(StateEnum.TWO);
      ExampleFinishableState stateThree = new ExampleFinishableState(StateEnum.THREE);

      factory.addFinishableState(StateEnum.ONE, stateOne, StateEnum.TWO);
      factory.addFinishableState(StateEnum.TWO, stateTwo, StateEnum.THREE);
      factory.addFinishableState(StateEnum.THREE, stateThree, StateEnum.ONE);
      factory.setNamePrefix("example");
      factory.setRegistry(new YoVariableRegistry("dummy"));
      StateMachine<StateEnum, ExampleFinishableState> stateMachine = factory.build(StateEnum.ONE);

      stateMachine.doControlAndTransition();

      // Run through some tests:
      assertTrue(stateOne.inState);
      assertTrue(stateOne.didAction);
      assertFalse(stateOne.didTransitionOutOfAction);

      stateMachine.doControlAndTransition();

      assertEquals(stateMachine.getCurrentState(), stateOne);
      assertTrue(stateOne.didAction);
      assertFalse(stateOne.didTransitionOutOfAction);

      stateOne.setIsDone(true);
      stateMachine.doControlAndTransition();

      assertTrue(stateOne.didTransitionOutOfAction);
      assertTrue(stateTwo.inState);
      assertFalse(stateTwo.didAction);
      assertFalse(stateTwo.didTransitionOutOfAction);
      assertEquals(stateMachine.getCurrentState(), stateTwo);

      stateTwo.setIsDone(true);
      stateMachine.doControlAndTransition();

      assertTrue(stateTwo.didTransitionOutOfAction);
      assertTrue(stateThree.inState);
      assertFalse(stateThree.didAction);
      assertFalse(stateThree.didTransitionOutOfAction);
      assertEquals(stateMachine.getCurrentState(), stateThree);

      stateMachine.doControlAndTransition();
      assertTrue(stateThree.inState);
      assertTrue(stateThree.didAction);
      assertFalse(stateThree.didTransitionOutOfAction);
      assertEquals(stateMachine.getCurrentState(), stateThree);

      stateThree.setIsDone(true);
      stateMachine.doControlAndTransition();
      assertTrue(stateThree.didTransitionOutOfAction);
      assertEquals(stateMachine.getCurrentState(), stateOne);
   }

   private class ExampleFinishableState implements FinishableState
   {
      private boolean isDone = false;
      public boolean inState = false;
      public boolean didAction = false;
      public boolean didTransitionOutOfAction = false;

      public ExampleFinishableState(StateEnum stateEnum)
      {
      }

      public void setIsDone(boolean isDone)
      {
         this.isDone = isDone;
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return isDone;
      }

      @Override
      public void doAction(double timeInState)
      {
         didAction = true;
      }

      @Override
      public void onEntry()
      {
         inState = true;
      }

      @Override
      public void onExit()
      {
         didTransitionOutOfAction = true;
         inState = false;
      }

   }

   private enum StateEnum
   {
      ONE, TWO, THREE, FOUR;
   }
}
