import com.neuronrobotics.sdk.common.BowlerAbstractDevice

import com.neuronrobotics.bowlerstudio.creature.ICadGenerator;
import com.neuronrobotics.bowlerstudio.physics.TransformFactory
import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine
import org.apache.commons.io.IOUtils;
import org.eclipse.jetty.server.handler.MovedContextHandler

import com.neuronrobotics.bowlerstudio.vitamins.*;
import com.neuronrobotics.sdk.addons.kinematics.AbstractLink
import com.neuronrobotics.sdk.addons.kinematics.DHLink
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.LinkConfiguration
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR
import com.neuronrobotics.sdk.common.IDeviceAddedListener
import com.neuronrobotics.sdk.common.IDeviceConnectionEventListener

import java.nio.file.Paths;

import eu.mihosoft.vrl.v3d.CSG
import eu.mihosoft.vrl.v3d.Cube
import eu.mihosoft.vrl.v3d.Cylinder
import eu.mihosoft.vrl.v3d.FileUtil
import eu.mihosoft.vrl.v3d.Parabola
import eu.mihosoft.vrl.v3d.RoundedCube
import eu.mihosoft.vrl.v3d.RoundedCylinder
import eu.mihosoft.vrl.v3d.Sphere
import eu.mihosoft.vrl.v3d.Transform
import javafx.scene.paint.Color
import javafx.scene.transform.Affine;
import  eu.mihosoft.vrl.v3d.ext.quickhull3d.*
import eu.mihosoft.vrl.v3d.parametrics.LengthParameter
import eu.mihosoft.vrl.v3d.Vector3d

CSG reverseDHValues(CSG incoming,DHLink dh ){
	//println "Reversing "+dh
	TransformNR step = new TransformNR(dh.DhStep(0))
	Transform move = com.neuronrobotics.bowlerstudio.physics.TransformFactory.nrToCSG(step)
	return incoming.transformed(move)
}

CSG moveDHValues(CSG incoming,DHLink dh ){
	TransformNR step = new TransformNR(dh.DhStep(0)).inverse()
	Transform move = com.neuronrobotics.bowlerstudio.physics.TransformFactory.nrToCSG(step)
	return incoming.transformed(move)
}
return new ICadGenerator(){
			private ArrayList<CSG> getHandParts(MobileBase handMB){
				return ScriptingEngine.gitScriptRun(
						"https://github.com/madhephaestus/Fanuc_LR_Mate_200id_7L.git",
						"makeModularHand.groovy",[handMB])
			}
			@Override
			public ArrayList<CSG> generateCad(DHParameterKinematics arg0, int arg1) {
				ArrayList<CSG> parts =  new ArrayList<>();
				
				DHLink dh = arg0.getDhLink(arg1)
				Affine manipulator = arg0.getLinkObjectManipulator(arg1)
				if(arg1==0) {
					if(!arg0.getScriptingName().endsWith("a")) {
						def name = "stl/link_"+(arg1+1)+".STL"
						println "Loading "+name
						def linkcenter=95
						def ballcenter = 10
						CSG link  = Vitamins.get(ScriptingEngine.fileFromGit(
								"https://github.com/madhephaestus/Fanuc_Delta_DR-3iB.git",
								name)).roty(194).toXMin().toYMin().toZMax()
								.movez(linkcenter)
								.movex(-linkcenter+2)
								.roty(90)
								.rotz(180)
								.toYMin()
								.movey(-ballcenter)
						link.setManipulator(manipulator)
						link.setColor(Color.SILVER)
						parts.add(link)
					}
				}else if(arg1==2) {
					CSG rotZPlate =  Vitamins.get(ScriptingEngine.fileFromGit(
						"https://github.com/madhephaestus/Fanuc_Delta_DR-3iB.git",
						"stl/passive-a.STL"))
						.roty(-28.5)
						.rotx(-1)
					rotZPlate=rotZPlate
								.movex(-rotZPlate.getCenterX())
								.movey(-rotZPlate.getCenterY())
								.toZMin()
								//.movex(-20)
								.movez(-20)
					if(!arg0.getScriptingName().endsWith("a")) {
						rotZPlate=rotZPlate.rotz(180)
					}
					rotZPlate.setManipulator(manipulator)
					rotZPlate.setColor(Color.BLACK)
					parts.add(rotZPlate)
					
				}else if(arg1==6) {
					CSG eoatPlate  = Vitamins.get(ScriptingEngine.fileFromGit(
						"https://github.com/madhephaestus/Fanuc_Delta_DR-3iB.git",
						"stl/endPlate.STL"))
					eoatPlate=eoatPlate
						.movex(-eoatPlate.getCenterX())
						.movey(-eoatPlate.getCenterY())
						.toZMax()
						.rotx(180)
						.movex(10)
						.movez(-20)
					eoatPlate.setManipulator(manipulator)
					eoatPlate.setColor(Color.SILVER)
					parts.add(eoatPlate)
					
				}else if(arg1==7) {
					MobileBase handMB = arg0.getSlaveMobileBase(arg1)
					parts.addAll(getHandParts(handMB))
					
					CSG rotZPlate =  Vitamins.get(ScriptingEngine.fileFromGit(
						"https://github.com/madhephaestus/Fanuc_Delta_DR-3iB.git",
						"stl/rotzLink.STL"))
					rotZPlate=rotZPlate
								.movex(-rotZPlate.getCenterX())
								.movey(-rotZPlate.getCenterY())
								.toZMin()
								.movez(160)
					
					rotZPlate.setManipulator(manipulator)
					rotZPlate.setColor(Color.SILVER)
					parts.add(rotZPlate)
					
				}
				for(int i=0;i<parts.size();i++) {
					parts.get(i).setName("Fanuc link "+arg1+" part "+i)
				}
				if(parts.size()==0)parts.add(new Cube(0.001).toCSG())
				return parts;
			}

			@Override
			public ArrayList<CSG> generateBody(MobileBase arg0) {
				ArrayList<CSG> parts =  new ArrayList<>();
				CSG base  = Vitamins.get(ScriptingEngine.fileFromGit(
						"https://github.com/madhephaestus/Fanuc_Delta_DR-3iB.git",
						"stl/BASE.STL"))
						.roty(180)
						.toZMax()
						.movez(100)
				base=base.movey(-base.getCenterY())
				base=base.movex(-base.getCenterX())
						.movex(-85)
						.movey(20)
				Affine manipulator = arg0.getRootListener()
				base.setManipulator(manipulator)

				parts.add(base)
				for(CSG part :parts) {
					part.setColor(Color.web("#f3da0b"))
				}
				for(int i=0;i<parts.size();i++) {
					parts.get(i).setName("Fanuc link base part "+i)
				}
				return parts;
			}
		};
