//Your code here

MobileBase base=DeviceManager.getSpecificDevice( "Fanuc_Delta_DR",{ScriptingEngine.gitScriptRun(	"https://github.com/madhephaestus/Fanuc_Delta_DR-3iB.git", "Fanuc_Delta_DR-3iB.xml", null )})



println "Now we will move just one leg"
DHParameterKinematics mainLimb = base.getAllDHChains().get(0)

return null