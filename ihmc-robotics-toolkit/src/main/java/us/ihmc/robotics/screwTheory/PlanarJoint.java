package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class PlanarJoint extends AbstractInverseDynamicsJoint implements FloatingInverseDynamicsJoint
{
   private final Quaternion jointRotation = new Quaternion();
   private final Vector3D jointTranslation = new Vector3D();
   private final Twist jointTwist;
   private final SpatialAccelerationVector jointAcceleration;
   private final SpatialAccelerationVector jointAccelerationDesired;

   private List<Twist> unitTwists;

   private Wrench successorWrench;

   public PlanarJoint(String name, RigidBody predecessor)
   {
      this(name, predecessor, null);
   }

   public PlanarJoint(String name, RigidBody predecessor, RigidBodyTransform transformToParent)
   {
      super(name, predecessor, transformToParent);

      jointTwist = new Twist(afterJointFrame, beforeJointFrame, afterJointFrame);
      jointAcceleration = new SpatialAccelerationVector(afterJointFrame, beforeJointFrame, afterJointFrame);
      jointAccelerationDesired = new SpatialAccelerationVector(afterJointFrame, beforeJointFrame, afterJointFrame);
   }

   @Override
   protected void updateJointTransform(RigidBodyTransform jointTransform)
   {
      jointTransform.set(jointRotation, jointTranslation);
   }

   @Override
   public void updateMotionSubspace()
   {
      // empty
   }

   @Override
   public void setDesiredAccelerationToZero()
   {
      jointAccelerationDesired.setToZero();
   }

   @Override
   public void setJointPositionVelocityAndAcceleration(InverseDynamicsJoint originalJoint)
   {
      PlanarJoint floatingJoint = checkAndGetAsPlanarJoint(originalJoint);

      setPosition(floatingJoint.jointTranslation);
      setRotation(floatingJoint.jointRotation);

      setJointTwist(floatingJoint.jointTwist);
      setAcceleration(floatingJoint.jointAcceleration);
   }

   @Override
   public void setQddDesired(InverseDynamicsJoint originalJoint)
   {
      PlanarJoint floatingJoint = checkAndGetAsPlanarJoint(originalJoint);

      jointAccelerationDesired.setAngularPart(floatingJoint.jointAccelerationDesired.getAngularPart());
      jointAccelerationDesired.setLinearPart(floatingJoint.jointAccelerationDesired.getLinearPart());
   }

   @Override
   public void setSuccessor(RigidBody successor)
   {
      this.successor = successor;
      setMotionSubspace();

      ReferenceFrame successorFrame = successor.getBodyFixedFrame();

      successorWrench = new Wrench(successorFrame, successorFrame);
   }

   @Override
   public void getJointTwist(Twist twistToPack)
   {
      twistToPack.setToZero(jointTwist.getBodyFrame(), jointTwist.getBaseFrame(), jointTwist.getExpressedInFrame());

      twistToPack.setAngularPartY(jointTwist.getAngularPartY());
      twistToPack.setLinearPartX(jointTwist.getLinearPartX());
      twistToPack.setLinearPartZ(jointTwist.getLinearPartZ());
   }

   @Override
   public void getJointAcceleration(SpatialAccelerationVector accelerationToPack)
   {
      accelerationToPack.setToZero(jointAcceleration.getBodyFrame(), jointAcceleration.getBaseFrame(), jointAcceleration.getExpressedInFrame());

      accelerationToPack.setAngularPartY(jointAcceleration.getAngularPartY());
      accelerationToPack.setLinearPartX(jointAcceleration.getLinearPartX());
      accelerationToPack.setLinearPartZ(jointAcceleration.getLinearPartZ());
   }

   @Override
   public void getDesiredJointAcceleration(SpatialAccelerationVector accelerationToPack)
   {
      accelerationToPack.setToZero(jointAccelerationDesired.getBodyFrame(), jointAccelerationDesired.getBaseFrame(),
                                   jointAccelerationDesired.getExpressedInFrame());

      accelerationToPack.setAngularPartY(jointAccelerationDesired.getAngularPartY());
      accelerationToPack.setLinearPartX(jointAccelerationDesired.getLinearPartX());
      accelerationToPack.setLinearPartZ(jointAccelerationDesired.getLinearPartZ());
   }

   @Override
   public void getTauMatrix(DenseMatrix64F matrix)
   {
      matrix.set(0, 0, successorWrench.getAngularPartY());
      matrix.set(1, 0, successorWrench.getLinearPartX());
      matrix.set(2, 0, successorWrench.getLinearPartZ());
   }

   @Override
   public void getVelocityMatrix(DenseMatrix64F matrix, int rowStart)
   {
      matrix.set(rowStart + 0, 0, jointTwist.getAngularPartY());
      matrix.set(rowStart + 1, 0, jointTwist.getLinearPartX());
      matrix.set(rowStart + 2, 0, jointTwist.getLinearPartZ());
   }

   @Override
   public void getDesiredAccelerationMatrix(DenseMatrix64F matrix, int rowStart)
   {
      matrix.set(rowStart + 0, 0, jointAccelerationDesired.getAngularPartY());
      matrix.set(rowStart + 1, 0, jointAccelerationDesired.getLinearPartX());
      matrix.set(rowStart + 2, 0, jointAccelerationDesired.getLinearPartZ());
   }

   @Override
   public void setTorqueFromWrench(Wrench jointWrench)
   {
      jointWrench.getBodyFrame().checkReferenceFrameMatch(successor.getBodyFixedFrame());
      jointWrench.setToZero(successor.getBodyFixedFrame(), successorWrench.getExpressedInFrame());
      jointWrench.setAngularPartY(successorWrench.getAngularPartY());
      jointWrench.setLinearPartX(successorWrench.getLinearPartX());
      jointWrench.setLinearPartZ(successorWrench.getLinearPartZ());
      jointWrench.changeFrame(successor.getBodyFixedFrame());
   }

   @Override
   public int getDegreesOfFreedom()
   {
      return 3;
   }

   @Override
   public void setDesiredAcceleration(DenseMatrix64F matrix, int rowStart)
   {
      jointAccelerationDesired.setToZero();
      jointAccelerationDesired.setAngularPartY(matrix.get(rowStart + 0, 0));
      jointAccelerationDesired.setLinearPartX(matrix.get(rowStart + 1, 0));
      jointAccelerationDesired.setLinearPartZ(matrix.get(rowStart + 2, 0));
   }

   @Override
   public void setJointTorque(DenseMatrix64F matrix, int rowStart)
   {
      successorWrench.setAngularPartY(matrix.get(rowStart + 0));
      successorWrench.setLinearPartX(matrix.get(rowStart + 1));
      successorWrench.setLinearPartZ(matrix.get(rowStart + 2));
   }

   public void setRotation(double yaw, double pitch, double roll)
   {
      jointRotation.setToPitchQuaternion(pitch);
   }

   @Override
   public void setRotation(QuaternionReadOnly jointRotation)
   {
      this.jointRotation.setToPitchQuaternion(jointRotation.getPitch());
   }

   @Override
   public void setRotation(double x, double y, double z, double s)
   {
      jointRotation.set(x, y, z, s);
      setRotation(jointRotation);
   }

   @Override
   public void setRotation(RotationMatrixReadOnly jointRotation)
   {
      this.jointRotation.setToPitchQuaternion(jointRotation.getPitch());
   }

   @Override
   public void setPosition(Tuple3DReadOnly jointTranslation)
   {
      this.jointTranslation.set(jointTranslation.getX(), 0.0, jointTranslation.getZ());
   }

   @Override
   public void setPosition(double x, double y, double z)
   {
      jointTranslation.set(x, 0.0, z);
   }

   @Override
   public void setJointTwist(Twist jointTwist)
   {
      this.jointTwist.checkReferenceFramesMatch(jointTwist.getBodyFrame(), jointTwist.getBaseFrame(), jointTwist.getExpressedInFrame());
      this.jointTwist.setAngularPartY(jointTwist.getAngularPartY());
      this.jointTwist.setLinearPartX(jointTwist.getLinearPartX());
      this.jointTwist.setLinearPartZ(jointTwist.getLinearPartZ());
   }

   @Override
   public void setAcceleration(SpatialAccelerationVector jointAcceleration)
   {
      this.jointAcceleration.checkReferenceFramesMatch(jointAcceleration);
      this.jointAcceleration.setAngularPartY(jointAcceleration.getAngularPartY());
      this.jointAcceleration.setLinearPartX(jointAcceleration.getLinearPartX());
      this.jointAcceleration.setLinearPartZ(jointAcceleration.getLinearPartZ());
   }

   @Override
   public void setDesiredAcceleration(SpatialAccelerationVector jointAcceleration)
   {
      jointAccelerationDesired.checkReferenceFramesMatch(jointAcceleration);
      jointAccelerationDesired.setAngularPartY(jointAcceleration.getAngularPartY());
      jointAccelerationDesired.setLinearPartX(jointAcceleration.getLinearPartX());
      jointAccelerationDesired.setLinearPartZ(jointAcceleration.getLinearPartZ());
   }

   @Override
   public void setWrench(Wrench jointWrench)
   {
      successorWrench.getBodyFrame().checkReferenceFrameMatch(jointWrench.getBodyFrame());
      successorWrench.getExpressedInFrame().checkReferenceFrameMatch(jointWrench.getExpressedInFrame());
      successorWrench.setAngularPartY(jointWrench.getAngularPartY());
      successorWrench.setLinearPartX(jointWrench.getLinearPartX());
      successorWrench.setLinearPartZ(jointWrench.getLinearPartZ());
   }

   @Override
   public void getRotation(QuaternionBasics rotationToPack)
   {
      rotationToPack.setToPitchQuaternion(jointRotation.getPitch());
   }

   @Override
   public void getRotation(RotationMatrix rotationToPack)
   {
      rotationToPack.setToPitchMatrix(jointRotation.getPitch());
   }

   @Override
   public void getRotation(double[] yawPitchRoll)
   {
      yawPitchRoll[0] = 0.0;
      yawPitchRoll[1] = jointRotation.getPitch();
      yawPitchRoll[2] = 0.0;
   }

   @Override
   public void getTranslation(Tuple3DBasics translationToPack)
   {
      translationToPack.setX(jointTranslation.getX());
      translationToPack.setZ(jointTranslation.getZ());
   }

   @Override
   public void setPositionAndRotation(RigidBodyTransform transform)
   {
      jointRotation.setToPitchQuaternion(transform.getRotationMatrix().getPitch());
      jointRotation.checkIfUnitary();

      transform.getTranslation(jointTranslation);
      jointTranslation.setY(0.0);
   }

   @Override
   public Tuple3DReadOnly getTranslationForReading()
   {
      return jointTranslation;
   }

   @Override
   public QuaternionReadOnly getRotationForReading()
   {
      return jointRotation;
   }

   @Override
   public Vector3DReadOnly getLinearVelocityForReading()
   {
      jointTwist.setLinearPartY(0.0);
      return jointTwist.getLinearPart();
   }

   @Override
   public Vector3DReadOnly getAngularVelocityForReading()
   {
      jointTwist.setAngularPartX(0.0);
      jointTwist.setAngularPartZ(0.0);

      return jointTwist.getAngularPart();
   }

   public void getAngularVelocity(Vector3DBasics angularVelocityToPack)
   {
      angularVelocityToPack.set(0.0, jointTwist.getAngularPartY(), 0.0);
   }

   public void getLinearVelocity(Vector3DBasics linearVelocityToPack)
   {
      linearVelocityToPack.set(jointTwist.getLinearPartX(), 0.0, jointTwist.getLinearPartZ());
   }

   @Override
   public void getWrench(Wrench wrenchToPack)
   {
      wrenchToPack.setToZero(successorWrench.getBodyFrame(), successorWrench.getExpressedInFrame());

      wrenchToPack.setAngularPartY(successorWrench.getAngularPartY());
      wrenchToPack.setLinearPartX(successorWrench.getLinearPartX());
      wrenchToPack.setLinearPartZ(successorWrench.getLinearPartZ());
   }

   public void getLinearAcceleration(Vector3DBasics linearAccelerationToPack)
   {
      linearAccelerationToPack.setX(jointAcceleration.getLinearPartX());
      linearAccelerationToPack.setY(0.0);
      linearAccelerationToPack.setZ(jointAcceleration.getLinearPartZ());
   }

   @Override
   public void getConfigurationMatrix(DenseMatrix64F matrix, int rowStart)
   {
      int index = rowStart;
      matrix.set(index++, 0, jointRotation.getPitch());
      matrix.set(index++, 0, jointTranslation.getX());
      matrix.set(index++, 0, jointTranslation.getZ());
   }

   @Override
   public void setConfiguration(DenseMatrix64F matrix, int rowStart)
   {
      int index = rowStart;
      double qRot = matrix.get(index++, 0);
      double x = matrix.get(index++, 0);
      double z = matrix.get(index++, 0);
      jointRotation.setToPitchQuaternion(qRot);
      jointTranslation.set(x, 0.0, z);
   }

   @Override
   public void setVelocity(DenseMatrix64F matrix, int rowStart)
   {
      int index = rowStart;
      double qdRot = matrix.get(index++, 0);
      double xd = matrix.get(index++, 0);
      double zd = matrix.get(index++, 0);
      jointTwist.setToZero();
      jointTwist.setAngularPartY(qdRot);
      jointTwist.setLinearPart(xd, 0.0, zd);
   }

   @Override
   public void calculateJointStateChecksum(GenericCRC32 checksum)
   {
      checksum.update(jointRotation.getPitch());
      checksum.update(jointTwist.getAngularPartY());
      checksum.update(jointAcceleration.getAngularPartY());

      checksum.update(jointTranslation.getX());
      checksum.update(jointTranslation.getY());
      checksum.update(jointTwist.getLinearPartX());
      checksum.update(jointTwist.getLinearPartZ());
      checksum.update(jointAcceleration.getLinearPartX());
      checksum.update(jointAcceleration.getLinearPartZ());
   }


   @Override
   public int getConfigurationMatrixSize()
   {
      return getDegreesOfFreedom();
   }

   private void setMotionSubspace()
   {
      int nDegreesOfFreedom = getDegreesOfFreedom();

      ArrayList<Twist> unitTwistsInBodyFrame = new ArrayList<>();
      RigidBodyTransform identity = new RigidBodyTransform();

      ReferenceFrame[] intermediateFrames = new ReferenceFrame[nDegreesOfFreedom - 1];
      ReferenceFrame previousFrame = afterJointFrame;
      for (int i = 0; i < nDegreesOfFreedom - 1; i++) // from afterJointFrame to beforeJointFrame
      {
         int index = intermediateFrames.length - i - 1;
         ReferenceFrame frame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("intermediateFrame" + index, previousFrame, identity);
         intermediateFrames[index] = frame;
         previousFrame = frame;
      }

      previousFrame = beforeJointFrame;

      for (int i = 0; i < nDegreesOfFreedom; i++) // from beforeJointFrame to afterJointFrame
      {
         ReferenceFrame frame;
         if (i < nDegreesOfFreedom - 1)
         {
            frame = intermediateFrames[i];
         }
         else
         {
            frame = afterJointFrame;
         }

         DenseMatrix64F twistMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);
         int startIndex = 0;
         twistMatrix.set(startIndex + i, 0, 1.0);
         Twist twist = new Twist(frame, previousFrame, frame, twistMatrix);
         unitTwistsInBodyFrame.add(twist);

         previousFrame = frame;
      }

      unitTwists = Collections.unmodifiableList(unitTwistsInBodyFrame);

      motionSubspace = new GeometricJacobian(this, afterJointFrame);
      motionSubspace.compute();
   }

   @Override
   public void getUnitTwist(int dofIndex, Twist unitTwistToPack)
   {
      if (dofIndex < 0 || dofIndex >= getDegreesOfFreedom())
         throw new ArrayIndexOutOfBoundsException("Illegal index: " + dofIndex + ", was expecting dofIndex in [0, " + getDegreesOfFreedom() + "[.");
      unitTwistToPack.set(unitTwists.get(dofIndex));
   }

   private PlanarJoint checkAndGetAsPlanarJoint(InverseDynamicsJoint originalJoint)
   {
      if (originalJoint instanceof PlanarJoint)
      {
         return (PlanarJoint) originalJoint;
      }
      else
      {
         throw new RuntimeException("Cannot set " + getClass().getSimpleName() + " to " + originalJoint.getClass().getSimpleName());
      }
   }
}
