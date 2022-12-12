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
				String type = "sr-12ia"
				if(arg0.getScriptingName().toLowerCase().contains("staubli")) {
					type = "ts2-100-vb"
				}
				if(arg1==0) {
					if(!arg0.getScriptingName().endsWith("a")) {
						def name = "stl/"+type+"/"+"l1.STL"
						CSG link  = Vitamins.get(ScriptingEngine.fileFromGit(
								"https://github.com/madhephaestus/Fanuc_SR_12ia-Scara.git",
								name))
						link.setColor(Color.web("#f3da0b"))
						parts.add(link)
					}
				}else if(arg1==1 ) {
					CSG rotZPlate =  Vitamins.get(ScriptingEngine.fileFromGit(
						"https://github.com/madhephaestus/Fanuc_SR_12ia-Scara.git",
						"stl/"+type+"/"+"l2.STL"))

					
					rotZPlate=rotZPlate.rotz(-90)
					
					rotZPlate.setColor(Color.web("#f3da0b"))
					parts.add(rotZPlate)
					
				}
				else if(arg1==2 ) {
					CSG rotZPlate =  Vitamins.get(ScriptingEngine.fileFromGit(
						"https://github.com/madhephaestus/Fanuc_SR_12ia-Scara.git",
						"stl/"+type+"/"+"l3.STL"))
					rotZPlate=rotZPlate

					rotZPlate.setColor(Color.SILVER)
					parts.add(rotZPlate)
				}
				for(int i=0;i<parts.size();i++) {
					def p = parts.get(i)
					p=moveDHValues(p,dh)
					p.setManipulator(manipulator)
					parts.set(i,p )
				}
				
				MobileBase handMB = arg0.getSlaveMobileBase(arg1)
				if(handMB!=null)
					parts.addAll(getHandParts(handMB))
				for(int i=0;i<parts.size();i++) {
					parts.get(i).setName(type+" link "+arg1+" part "+i)
				}
				if(parts.size()==0)parts.add(new Cube(0.001).toCSG())
				return parts;
			}

			@Override
			public ArrayList<CSG> generateBody(MobileBase arg0) {
				ArrayList<CSG> parts =  new ArrayList<>();
				String type = "sr-12ia"
				if(arg0.getScriptingName().toLowerCase().contains("staubli")) {
					type = "ts2-100-vb"
				}
				CSG base  = Vitamins.get(ScriptingEngine.fileFromGit(
						"https://github.com/madhephaestus/Fanuc_SR_12ia-Scara.git",
						"stl/"+type+"/"+"base.STL"))

				Affine manipulator = arg0.getRootListener()
				base.setManipulator(manipulator)

				parts.add(base)
				for(CSG part :parts) {
					part.setColor(Color.DARKGREY)
				}
				for(int i=0;i<parts.size();i++) {
					parts.get(i).setName("Fanuc link base part "+i)
				}
				return parts;
			}
		};
