
import com.neuronrobotics.bowlerstudio.physics.TransformFactory
import com.neuronrobotics.sdk.addons.kinematics.AbstractKinematicsNR
import com.neuronrobotics.sdk.addons.kinematics.DHChain;
import com.neuronrobotics.sdk.addons.kinematics.DhInverseSolver;
import com.neuronrobotics.sdk.addons.kinematics.WristNormalizer
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR;
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR;

import java.lang.reflect.Method
import java.text.DecimalFormat
import java.util.ArrayList;

import com.neuronrobotics.sdk.addons.kinematics.DHChain;
import com.neuronrobotics.sdk.addons.kinematics.DHLink;
import com.neuronrobotics.sdk.addons.kinematics.DhInverseSolver;
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR;
import com.neuronrobotics.sdk.common.Log;
import Jama.Matrix;
import eu.mihosoft.vrl.v3d.CSG
import eu.mihosoft.vrl.v3d.Cube
import eu.mihosoft.vrl.v3d.Cylinder
import eu.mihosoft.vrl.v3d.Transform;
import javafx.application.Platform
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class scriptJavaIKModel implements DhInverseSolver {
	private def l0Length
	private def l1Length
	boolean debug = false;
	CSG blue =null;
	CSG green =null;
	CSG red =null;
	CSG white =null;
	int limbIndex =0;
	public scriptJavaIKModel(int index){
		limbIndex=index;
	}
	@Override
	public double[] inverseKinematics(TransformNR target, double[] jointSpaceVector, DHChain chain) {
		ArrayList<DHLink> links = chain.getLinks();
		if(links.size()==3 || (links.size()==4 && (Math.abs(links.get(2).alpha)<0.001)))
			return inverseKinematics34dof(target,jointSpaceVector,chain);
		return inverseKinematics6dof(target,jointSpaceVector,chain);
	}
	TransformNR linkOffset(DHLink link) {
		return new TransformNR(link.DhStep(0))
	}
	double length(TransformNR tr) {
		return Math.sqrt(
				Math.pow(tr.getX(), 2)+
				Math.pow(tr.getY(), 2)+
				Math.pow(tr.getZ(), 2)
				);
	}
	public double[] inverseKinematics6dof(TransformNR target, double[] jointSpaceVector, DHChain chain) {
		if(debug)println "Target "+target
		double[] current = new double[jointSpaceVector.length]
		for(int i=0;i<current.length;i++) {
			current[i]=jointSpaceVector[i];
		}
		ArrayList<DHLink> links = chain.getLinks();
		int linkNum = jointSpaceVector.length;
		TransformNR l0Offset = linkOffset(links.get(0))
		TransformNR l1Offset = linkOffset(links.get(1))
		TransformNR l2Offset = linkOffset(links.get(2))
		TransformNR l3Offset = linkOffset(links.get(3))
		// Vector decompose the tip target
		double z = target.getZ();
		double y = target.getY();
		double x = target.getX();
		def targetNoRot =new TransformNR(x,y,z,new RotationNR())

		RotationNR q = target.getRotation();
		def newCenter =target.copy()
		// Start by finding the IK to the wrist center
		if(linkNum>=5) {
			//offset for tool
			if(debug)println "Offestting for tool"
			def tool = new TransformNR()
			if(linkNum==6)
				tool=linkOffset(links.get(5))
			// compute the transform from tip to wrist center
			def wristCenterOffsetTransform = linkOffset(links.get(4)).times(tool)
			//println wristCenterOffsetTransform
			// take off the tool from the target to get the center of the wrist
			newCenter = target.times(wristCenterOffsetTransform.inverse())
			//if(debug)Platform.runLater({TransformFactory.nrToAffine(newCenter,tipPointer2.getManipulator())})
		}
		// recompute the X,y,z with the new center
		z = newCenter.getZ();
		y = newCenter.getY();
		x = newCenter.getX();
		if(debug)println "newCenter "+newCenter
		if(newCenter.getZ()<chain.getlowerLimits()[2]) {
			throw new RuntimeException( "Dug too greedily and too deep!")
		}
		if(newCenter.getZ()>chain.getUpperLimits()[2]) {
			throw new RuntimeException( "Alas, Icrus flew too high!")
		}
		//xyz now are at the wrist center
		// Compute the xy plane projection of the tip
		// this is the angle of the tipto the base link
		if(x==0&&y==0) {
			Thread.sleep(10)
			throw new RuntimeException( "Singularity! try something else")
		}
		double baseVectorAngle = Math.atan2(y , x);
		double a1d = Math.toDegrees(baseVectorAngle);
		// this projection number becomes the base link angle directly
		if(debug)println "New base "+a1d
		//jointSpaceVector[0]=0;// TESTING

		if(debug)println "Base link to tip angle elevation "+a1d
		def transformAngleOfTipToTriangle = new TransformNR(0,0,0,new RotationNR(0,-a1d,0))
		def xyTip = new TransformNR(x,y,0,new RotationNR())
		//Transform the tip into the x Vector
		def tipXPlane =transformAngleOfTipToTriangle
				.times(xyTip)
		//println tipXYPlane
		double wristVect = tipXPlane.getX();
		if(debug)println "Hypotinuse of elbow "+wristVect
		// Use the law of cosines to calculate the elbow and the shoulder tilt
		l0Length = length(l0Offset)
		l1Length = length(l1Offset)
		if(wristVect>(l1Length+l0Length)) {
			Thread.sleep(10)
			throw new RuntimeException( "Reach too far! "+l1Length+" + "+l0Length+" is > "+wristVect)
		}
		

		double shoulderTiltAngle =solveForAngleLawOfCosine(l0Length,wristVect,l1Length)

		double elbowTiltAngle =solveForAngleLawOfCosine(l0Length,l1Length,wristVect)
		
		jointSpaceVector[0]=normalizeBase(-(shoulderTiltAngle-a1d),chain.getUpperLimits()[0], chain.getlowerLimits()[0], 0)
		jointSpaceVector[1]=-(elbowTiltAngle-Math.toDegrees(links.get(1).getTheta()))
		jointSpaceVector[2]=newCenter.getZ()
		jointSpaceVector[3]=-Math.toDegrees(target.getRotation().getRotationAzimuth())+jointSpaceVector[0]+jointSpaceVector[1]+Math.toDegrees(links.get(1).getTheta())
		jointSpaceVector[4]=0
		return jointSpaceVector;
	}
	
	double normalizeBase(double calc,double upper, double lower, int depth) {
		if(calc<upper && calc>lower)
			return calc;
		if(depth>2)
			return calc;
		def local = (depth==0?360:-360)
		return normalizeBase(calc+local,upper,lower,depth+1)
	}
	
	double solveForAngleLawOfCosine(double sideCCWtoAngle, double sideCWtoAngle, double sideOppisiteAngle) {
		double a=sideCCWtoAngle
		double b= sideCWtoAngle;
		double c= sideOppisiteAngle;
		return Math.toDegrees(
			Math.acos(
			((a*a)+(b*b)-(c*c))/
			(2*a*b)
			)
			)
	}
}



if(args==null)
	args=[0]
return new scriptJavaIKModel (args[0])
