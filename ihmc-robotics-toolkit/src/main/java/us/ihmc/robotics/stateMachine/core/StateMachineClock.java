package us.ihmc.robotics.stateMachine.core;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public interface StateMachineClock
{
   void notifyStateChanged();

   double getTime();

   double getTimeInCurrentState();

   double getTimeOfLastStateChange();

   public static StateMachineClock createDummyClock()
   {
      return new StateMachineClock()
      {
         @Override
         public void notifyStateChanged()
         {
         }

         @Override
         public double getTime()
         {
            return Double.NaN;
         }

         @Override
         public double getTimeInCurrentState()
         {
            return Double.NaN;
         }

         @Override
         public double getTimeOfLastStateChange()
         {
            return Double.NaN;
         }
      };
   }

   public static StateMachineClock createClock(DoubleProvider timeProvider)
   {
      return new StateMachineClock()
      {
         double timeOfLastStateChange = Double.NaN;

         @Override
         public void notifyStateChanged()
         {
            timeOfLastStateChange = timeProvider.getValue();
         }

         @Override
         public double getTime()
         {
            return timeProvider.getValue();
         }

         @Override
         public double getTimeInCurrentState()
         {
            return timeProvider.getValue() - timeOfLastStateChange;
         }

         @Override
         public double getTimeOfLastStateChange()
         {
            return timeOfLastStateChange;
         }
      };
   }

   public static StateMachineClock createYoClock(DoubleProvider timeProvider, String namePrefix, YoVariableRegistry registry)
   {
      YoDouble timeOfLastStateChange = new YoDouble(namePrefix + "SwitchTime", "Time at which the last state change occured.", registry);
      YoDouble timeInCurrentState = new YoDouble(namePrefix + "StateTime", "Time relative to the start of the current state.", registry);
      timeOfLastStateChange.setToNaN();
      timeInCurrentState.setToNaN();

      return new StateMachineClock()
      {
         @Override
         public void notifyStateChanged()
         {
            timeOfLastStateChange.set(timeProvider.getValue());
         }

         @Override
         public double getTime()
         {
            return timeProvider.getValue();
         }

         @Override
         public double getTimeInCurrentState()
         {
            double t = timeProvider.getValue() - timeOfLastStateChange.getDoubleValue();
            timeInCurrentState.set(t);
            return t;
         }

         @Override
         public double getTimeOfLastStateChange()
         {
            return timeOfLastStateChange.getDoubleValue();
         }
      };
   }
}
