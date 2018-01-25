package us.ihmc.manipulation.collision;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionShapeFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class CollisionShapeSphere extends AbstractCollisionShape
{
   private double radius;

   public CollisionShapeSphere(String name, YoVariableRegistry parentRegistry, SimpleCollisionShapeFactory shapeFactory, ReferenceFrame referenceFrame,
                               RigidBodyTransform transformToReferenceFrame, double radius)
   {
      super(name, parentRegistry, shapeFactory, referenceFrame, transformToReferenceFrame);
      this.radius = radius;
   }

   @Override
   public void createCollisionShape()
   {
      CollisionShapeDescription<?> collisionShapeDescription = shapeFactory.createSphere(radius);
      collisionShape = shapeFactory.addShape(collisionShapeDescription);
   }

   @Override
   public void createYoGraphic()
   {
      FramePose3D framePose = new FramePose3D(referenceFrame);

      framePose.setToZero();
      framePose.changeFrame(ReferenceFrame.getWorldFrame());

      yoFramePose.set(framePose);

      yoGraphic = new YoGraphicPosition(name + "yographics", yoFramePose.getPosition(), radius, YoAppearance.Yellow());
      yoGraphic.setVisible(true);
   }
}