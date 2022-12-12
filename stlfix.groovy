import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine
import com.neuronrobotics.bowlerstudio.vitamins.Vitamins

import eu.mihosoft.vrl.v3d.CSG
String type = "sr-12ia"
type = "ts2-100-vb"
def name = "stl/"+type+"/"+"l1.STL"
File servoFile = ScriptingEngine.fileFromGit(
	"https://github.com/madhephaestus/Fanuc_SR_12ia-Scara.git",
	name);
// Load the .CSG from the disk and cache it in memory
CSG servo  = Vitamins.get(servoFile)
			.toXMin()
			.toYMin()
			.toZMin()
servo=servo
			.movey(-servo.getTotalY()/2)
			.movex(-servo.getTotalY()/2)
String filename =servoFile.getAbsolutePath()
println filename
FileUtil.write(Paths.get(filename),
		servo.toStlString());
println "STL EXPORT to "+filename
return servo;