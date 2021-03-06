package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPostureInputProviderInterface;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedControlManagerFactory
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrupedForceControllerToolbox toolbox;
   private final QuadrupedPostureInputProviderInterface postureProvider;

   private QuadrupedFeetManager feetManager;
   private QuadrupedBodyOrientationManager bodyOrientationManager;
   private QuadrupedBalanceManager balanceManager;

   public QuadrupedControlManagerFactory(QuadrupedForceControllerToolbox toolbox, QuadrupedPostureInputProviderInterface postureProvider, YoVariableRegistry parentRegistry)
   {
      this.toolbox = toolbox;
      this.postureProvider = postureProvider;

      parentRegistry.addChild(registry);
   }

   public QuadrupedFeetManager getOrCreateFeetManager()
   {
      if (feetManager != null)
         return feetManager;

      feetManager = new QuadrupedFeetManager(toolbox, registry);
      return feetManager;
   }

   public QuadrupedBodyOrientationManager getOrCreateBodyOrientationManager()
   {
      if (bodyOrientationManager != null)
         return bodyOrientationManager;

      bodyOrientationManager = new QuadrupedBodyOrientationManager(toolbox, postureProvider, registry);
      return bodyOrientationManager;
   }

   public QuadrupedBalanceManager getOrCreateBalanceManager()
   {
      if (balanceManager != null)
         return balanceManager;

      balanceManager = new QuadrupedBalanceManager(toolbox, postureProvider, registry, toolbox.getRuntimeEnvironment().getGraphicsListRegistry());
      return balanceManager;
   }

   public QuadrupedSolePositionController getOrCreateSolePositionController(RobotQuadrant robotQuadrant)
   {
      return getOrCreateFeetManager().getSolePositionController(robotQuadrant);
   }
}
