package us.ihmc.robotics.stateMachine.factories;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateChangedListener;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateMachineClock;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.robotics.stateMachine.extra.EventBasedStateMachineTrigger;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class EventBasedStateMachineFactory<K extends Enum<K>, S extends State>
{
   private String namePrefix;
   private YoVariableRegistry registry;

   private final Map<K, S> states;
   private final Map<K, EventBasedTransition<K>> eventBasedTransitionsMap;
   private final List<StateChangedListener<K>> stateChangedListeners = new ArrayList<>();
   private final AtomicReference<Object> eventFired = new AtomicReference<>(null);
   private StateMachine<K, S> stateMachine;
   private StateMachineClock clock = StateMachineClock.createDummyClock();

   public EventBasedStateMachineFactory(Class<K> keyType)
   {
      states = new EnumMap<>(keyType);
      eventBasedTransitionsMap = new EnumMap<>(keyType);
   }

   public EventBasedStateMachineFactory<K, S> setNamePrefix(String namePrefix)
   {
      this.namePrefix = namePrefix;
      return this;
   }

   public EventBasedStateMachineFactory<K, S> setRegistry(YoVariableRegistry registry)
   {
      this.registry = registry;
      return this;
   }

   public EventBasedStateMachineFactory<K, S> buildClock(DoubleProvider timeProvider)
   {
      clock = StateMachineClock.createClock(timeProvider);
      return this;
   }

   public EventBasedStateMachineFactory<K, S> buildYoClock(DoubleProvider timeProvider)
   {
      if (namePrefix == null || registry == null)
         throw new RuntimeException("The namePrefix and registry fields have to be set in order to create yo-variables.");
      clock = StateMachineClock.createYoClock(timeProvider, namePrefix, registry);
      return this;
   }

   public EventBasedStateMachineFactory<K, S> addState(K key, S state)
   {
      S oldState = states.put(key, state);
      if (oldState != null)
         PrintTools.warn("The state " + oldState.getClass().getSimpleName() + " at the key " + key + " has been replaced with "
               + state.getClass().getSimpleName());
      return this;
   }

   public <E extends Enum<E>> EventBasedStateMachineFactory<K, S> addTransition(E event, K from, K to)
   {
      getStateEventBasedTransitions(from).registerEvent(event, to);
      return this;
   }

   public <E extends Enum<E>> EventBasedStateMachineFactory<K, S> addCallback(E event, K stateKey, Runnable callback)
   {
      getStateEventBasedTransitions(stateKey).registerCallback(event, callback);
      return this;
   }

   private EventBasedTransition<K> getStateEventBasedTransitions(K state)
   {
      EventBasedTransition<K> stateEventBasedTransitions = eventBasedTransitionsMap.get(state);
      if (stateEventBasedTransitions == null)
      {
         stateEventBasedTransitions = new EventBasedTransition<>(eventFired);
         eventBasedTransitionsMap.put(state, stateEventBasedTransitions);
      }
      return stateEventBasedTransitions;
   }

   public EventBasedStateMachineFactory<K, S> addStateChangedListener(StateChangedListener<K> stateChangeListener)
   {
      stateChangedListeners.add(stateChangeListener);
      return this;
   }

   public StateMachine<K, S> buildStateMachine(K initialStateKey)
   {
      stateMachine = new StateMachine<K, S>(initialStateKey, states, eventBasedTransitionsMap, stateChangedListeners, clock, namePrefix, registry);
      return stateMachine;
   }

   public EventBasedStateMachineTrigger buildEventTrigger()
   {
      return new EventBasedStateMachineTrigger()
      {
         @Override
         public <E extends Enum<E>> void trigger(E event)
         {
            eventFired.set(event);
            stateMachine.doTransitions();
         }
      };
   }

   private static class EventBasedTransition<K extends Enum<K>> implements StateTransition<K>
   {
      private final Map<Object, K> eventToNextStateKeyMap = new HashMap<>();
      private final Map<Object, Runnable> eventToCallbacksMap = new HashMap<>();
      private final AtomicReference<Object> eventHolder;

      public EventBasedTransition(AtomicReference<Object> eventHolder)
      {
         this.eventHolder = eventHolder;
      }

      public void registerEvent(Object event, K nextStateKey)
      {
         eventToNextStateKeyMap.put(event, nextStateKey);
      }

      public void registerCallback(Object event, Runnable callback)
      {
         eventToCallbacksMap.put(event, callback);
      }

      @Override
      public K isTransitionRequested(double timeInCurrentState)
      {
         Object eventFired = eventHolder.getAndSet(null);
         if (eventFired == null)
            return null;

         Runnable callback = eventToCallbacksMap.get(eventFired);
         if (callback != null)
            callback.run();

         return eventToNextStateKeyMap.get(eventFired);
      }
   }
}
