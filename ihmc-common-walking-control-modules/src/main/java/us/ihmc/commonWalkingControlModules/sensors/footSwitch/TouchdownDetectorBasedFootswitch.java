package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.touchdownDetector.TouchdownDetector;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.robotics.sensors.FootSwitchInterface;

public abstract class TouchdownDetectorBasedFootswitch implements FootSwitchInterface
{
   protected final YoVariableRegistry registry;
   protected final ArrayList<TouchdownDetector> touchdownDetectors = new ArrayList<>();

   protected final YoBoolean controllerThinksHasTouchedDown;

   public TouchdownDetectorBasedFootswitch(String name, YoVariableRegistry parentRegistry)
   {
      this.registry = parentRegistry;
      controllerThinksHasTouchedDown = new YoBoolean(name + "_controllerThinksHasTouchedDown", parentRegistry);
   }

   @Override
   public void reset()
   {
      controllerThinksHasTouchedDown.set(false);
   }

   @Override
   public boolean getForceMagnitudePastThreshhold()
   {
      return false;
   }

   @Override
   public void setFootContactState(boolean hasFootHitGround)
   {
      controllerThinksHasTouchedDown.set(hasFootHitGround);
   }
}
