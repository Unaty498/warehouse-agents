package simulator;

import fr.emse.fayol.maqit.simulator.configuration.SimProperties;
import fr.emse.fayol.maqit.simulator.components.SituatedComponent;
import fr.emse.fayol.maqit.simulator.environment.ColorCell;
import fr.emse.fayol.maqit.simulator.environment.GridEnvironment;
import fr.emse.fayol.maqit.simulator.display.GraphicalWindow;

/**
 * A class to define the core of the simulator
 * test
 */
public abstract class SimFactory {

    protected SimProperties sp; //!< properties of the simulation
    protected GridEnvironment environment; //!< the discrete environment of the simulation 
    protected static int idComponent = 1; //!< index of component id (from 1 to +inf) 
    protected GraphicalWindow gwindow;

    /**
     * initialize SimProperties object and GridManagement object with constructor parameters
     * and initialize the obsctacle and robot list
     * @param sp properties of the simulation
     */
    public SimFactory(SimProperties sp){
        this.sp = sp;
        gwindow = null;
	}

    public void initializeGW() {
        gwindow = new GraphicalWindow((ColorCell[][])(environment.getGrid()),sp.display_x,sp.display_y,sp.display_width,sp.display_height,sp.display_title);
        gwindow.init();
    }

    public void refreshGW() {
        gwindow.refresh();
    }

    /**
     * Function to create the simulation environment
     */
    public abstract void createEnvironment();

    /**
     * Function to generate the obsctacles in the environment
     */
    public abstract void createObstacle();
    
    /**
     * Function to generate the robots in the environment
     */
    public abstract void createRobot();

    /**
     * Function to generate the goals in the environment
     */
    public abstract void createGoal();

    /**
     * Add a new situated component in the environment
     * @param sc situated component 
     */
    public void addNewComponent(SituatedComponent sc){
        int[] pos = sc.getLocation();
    	environment.setCell(pos[0],pos[1],sc);
    }

    /**
     * Move a component from one cell to another cell
     * @param from int array of the origin cell ([x,y])
     * @param to int array of the destination cell ([x,y])
     */
    public void updateEnvironment(int[] from, int[] to){
	   environment.moveComponent(from,to);
    }

    /**
     * Scheduler of the simulation     
     */
    public abstract void schedule();


}
