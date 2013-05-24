package us.ihmc.commonWalkingControlModules.momentumBasedController;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;

public class TaskspaceConstraintData
{
   private final SpatialAccelerationVector spatialAcceleration = new SpatialAccelerationVector();
   private final DenseMatrix64F nullspaceMultipliers = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);
   private final DenseMatrix64F selectionMatrix = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);

   public void set(SpatialAccelerationVector spatialAcceleration)
   {
      this.spatialAcceleration.set(spatialAcceleration);
      this.nullspaceMultipliers.reshape(0, 1);
      this.selectionMatrix.reshape(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);
   }

   public void set(SpatialAccelerationVector spatialAcceleration, DenseMatrix64F nullspaceMultipliers, DenseMatrix64F selectionMatrix)
   {
      this.spatialAcceleration.set(spatialAcceleration);
      this.nullspaceMultipliers.setReshape(nullspaceMultipliers);
      this.selectionMatrix.setReshape(selectionMatrix);
   }

   public void set(SpatialAccelerationVector spatialAcceleration, DenseMatrix64F nullspaceMultipliers)
   {
      this.spatialAcceleration.set(spatialAcceleration);
      this.nullspaceMultipliers.setReshape(nullspaceMultipliers);
      this.selectionMatrix.reshape(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);
   }

   public void setAngularAcceleration(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, FrameVector desiredAngularAcceleration,
         DenseMatrix64F nullspaceMultipliers)
   {
      this.spatialAcceleration.setToZero(bodyFrame, baseFrame, desiredAngularAcceleration.getReferenceFrame());
      this.spatialAcceleration.setAngularPart(desiredAngularAcceleration.getVector());

      this.nullspaceMultipliers.setReshape(nullspaceMultipliers);
      
      this.selectionMatrix.reshape(3, SpatialMotionVector.SIZE);
      this.selectionMatrix.set(0, 0, 1.0);
      this.selectionMatrix.set(1, 1, 1.0);
      this.selectionMatrix.set(2, 2, 1.0);
   }

   public void setLinearAcceleration(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, FrameVector desiredLinearAcceleration)
   {
      this.spatialAcceleration.setToZero(bodyFrame, baseFrame, desiredLinearAcceleration.getReferenceFrame());
      this.spatialAcceleration.setLinearPart(desiredLinearAcceleration.getVector());
      spatialAcceleration.changeFrameNoRelativeMotion(bodyFrame);

      this.nullspaceMultipliers.reshape(0, 1);

      this.selectionMatrix.reshape(3, SpatialMotionVector.SIZE);
      this.selectionMatrix.set(0, 3, 1.0);
      this.selectionMatrix.set(1, 4, 1.0);
      this.selectionMatrix.set(2, 5, 1.0);
   }

   public SpatialAccelerationVector getSpatialAcceleration()
   {
      return spatialAcceleration;
   }

   public DenseMatrix64F getNullspaceMultipliers()
   {
      return nullspaceMultipliers;
   }

   public DenseMatrix64F getSelectionMatrix()
   {
      return selectionMatrix;
   }
}
