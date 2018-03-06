package us.ihmc.robotics.stateMachine.factories;

import us.ihmc.robotics.stateMachine.core.StateChangedListener;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.robotics.stateMachine.extra.FinishableState;
import us.ihmc.robotics.stateMachine.extra.StateTransitionCondition;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

public class FinishableStateMachineFactory<K extends Enum<K>, S extends FinishableState> extends StateMachineFactory<K, S>
{
   public FinishableStateMachineFactory(Class<K> keyType)
   {
      super(keyType);
   }

   @Override
   public FinishableStateMachineFactory<K, S> setNamePrefix(String namePrefix)
   {
      super.setNamePrefix(namePrefix);
      return this;
   }

   @Override
   public FinishableStateMachineFactory<K, S> setRegistry(YoVariableRegistry registry)
   {
      super.setRegistry(registry);
      return this;
   }

   @Override
   public FinishableStateMachineFactory<K, S> buildClock(DoubleProvider timeProvider)
   {
      super.buildClock(timeProvider);
      return this;
   }

   @Override
   public FinishableStateMachineFactory<K, S> buildYoClock(DoubleProvider timeProvider)
   {
      super.buildYoClock(timeProvider);
      return this;
   }

   @Override
   public FinishableStateMachineFactory<K, S> addState(K key, S state)
   {
      super.addState(key, state);
      return this;
   }

   public FinishableStateMachineFactory<K, S> addFinishableState(K key, S state, K nextStateKey)
   {
      addState(key, state);
      addTransition(key, nextStateKey, state::isDone);
      return this;
   }

   @Override
   public FinishableStateMachineFactory<K, S> addRequestedTransition(K from, YoEnum<K> requestedState)
   {
      super.addRequestedTransition(from, requestedState);
      return this;
   }

   @Override
   public FinishableStateMachineFactory<K, S> addRequestedTransition(K from, K to, YoEnum<K> requestedState)
   {
      super.addRequestedTransition(from, to, requestedState);
      return this;
   }

   public FinishableStateMachineFactory<K, S> addRequestedTransition(K from, YoEnum<K> requestedState, boolean waitUntilDone)
   {
      if (!waitUntilDone)
      {
         return addRequestedTransition(from, requestedState);
      }

      return addTransition(from, new StateTransition<K>()
      {

         @Override
         public K isTransitionRequested(double timeInCurrentState)
         {
            if (!states.get(from).isDone(timeInCurrentState))
               return null;

            K nextStateKey = requestedState.getEnumValue();
            if (nextStateKey != null)
               requestedState.set(null);
            return nextStateKey;
         }
      });
   }

   public FinishableStateMachineFactory<K, S> addRequestedTransition(K from, K to, YoEnum<K> requestedState, boolean waitUntilDone)
   {
      if (!waitUntilDone)
      {
         return addRequestedTransition(from, to, requestedState);
      }

      return addTransition(from, to, new StateTransitionCondition()
      {
         @Override
         public boolean testCondition(double timeInCurrentState)
         {
            if (to == requestedState.getEnumValue() && states.get(from).isDone(timeInCurrentState))
            {
               requestedState.set(null);
               return true;
            }
            return false;
         }
      });
   }

   @Override
   public FinishableStateMachineFactory<K, S> addTimeBasedTransition(K from, K to, double durationBeforeTransition)
   {
      super.addTimeBasedTransition(from, to, durationBeforeTransition);
      return this;
   }

   @Override
   public FinishableStateMachineFactory<K, S> addTimeBasedTransition(K from, K to, DoubleProvider durationBeforeTransition)
   {
      super.addTimeBasedTransition(from, to, durationBeforeTransition);
      return this;
   }

   @Override
   public StateMachineFactory<K, S> addTransition(Iterable<? extends K> froms, K to, StateTransitionCondition condition)
   {
      super.addTransition(froms, to, condition);
      return this;
   }

   @Override
   public FinishableStateMachineFactory<K, S> addTransition(K from, K to, StateTransitionCondition condition)
   {
      super.addTransition(from, to, condition);
      return this;
   }

   @Override
   public FinishableStateMachineFactory<K, S> addTransition(K from, StateTransition<K> stateTransition)
   {
      super.addTransition(from, stateTransition);
      return this;
   }

   @Override
   public FinishableStateMachineFactory<K, S> addStateChangedListener(StateChangedListener<K> stateChangeListener)
   {
      super.addStateChangedListener(stateChangeListener);
      return this;
   }
}
