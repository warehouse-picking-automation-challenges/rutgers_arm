#Let's figure out some of the global params
NUM_SYSTEMS = 1
OBJECT_K = 1
GRASP_SAMPLES = 2
SAMPLE_ATTEMPTS = 2
APPROACH_ATTEMPTS = 3
PRETTY = false
NUM_GRASPS = 3

#A Mode for what benchmark we are running
MODE = 0 # Mode 0 is for Kiva Shelves, Mode 1 is for tables

#Whether or not to create a planning node
CREATE_PLANNING = true

#Whether we are $preprocessing or running online
$preprocessing = true

$cupheight = 0.3
$graspdist = 0.05

#Whether we are doing discrete search or forward tree search
DISCRETE_SEARCH = false
DELAY_PLANNING = false

#Random handling
SPECIFY_RANDOM = true #Whether or not to have a random seed specified in the generated file
RANDOM_RANDOM = true #When we are specifying a seed, this says whether or not we use the value below
RANDOM_SEED = 651425

#Number of collection bins.  This should be between 1 and 4, depending on the problem mode
# This also represents the number of different object destinations
NUM_BINS = 1

#The number of objects we want to generate per shelf/table
NUM_OBJECTS = 1

#What type of motion planner should be instantiated for each arm
PLANNER_TYPE = "prm_star"
ITERATIONS = 120

#Additional parameters for other uses
TDK_GENERATE_POSES = false
TDK_NUM_POSES = 100
TDK_POSE_BUFFER = 0.05


# = = Some Additional Information here == #

# KIVA SHELF INFORMATION (For input model at Oct 31, 2014)
# This provides the relative offset of the center of each shelf surface (approximately)
# Bin names are tentative

#= - - - - - = - - - - - = - - - - - = - - - - - =
#=  -0.306   =  -0.102   =   0.102   =   0.306   =
#= - - - - - = - - - - - = - - - - - = - - - - - = - - - - - =
#=     A     =     B     =     C     =     D     =   2.215   =
#= - - - - - = - - - - - = - - - - - = - - - - - = - - - - - =
#=     E     =     F     =     G     =     H     =   1.985   =
#= - - - - - = - - - - - = - - - - - = - - - - - = - - - - - =
#=     I     =     J     =     K     =     L     =   1.720   =
#= - - - - - = - - - - - = - - - - - = - - - - - = - - - - - =
#=     M     =     N     =     O     =     P     =   1.490   =
#= - - - - - = - - - - - = - - - - - = - - - - - = - - - - - =
#=     Q     =     R     =     S     =     T     =   1.225   =
#= - - - - - = - - - - - = - - - - - = - - - - - = - - - - - =
#=     U     =     V     =     W     =     X     =   0.880   =
#= - - - - - = - - - - - = - - - - - = - - - - - = - - - - - =
#=     Y     =     Z     =    AA     =    AB     =   0.485   =
#= - - - - - = - - - - - = - - - - - = - - - - - = - - - - - =

#= Note: All shelf centers have a y-offset of: -0.320

#Constants for indexing into the center points of the shelves
COL_1 = -0.306
COL_2 = -0.102
COL_3 = 0.102
COL_4 = 0.306

ROW_1 = 2.215
ROW_2 = 1.985
ROW_3 = 1.720
ROW_4 = 1.490
ROW_5 = 1.225
ROW_6 = 0.880
ROW_7 = 0.485

Y_OFF = -0.320

# = = Methods = = #

# = Generate a bin at the target location = #
def generate_bin( f, x, y, theta, i )
    quat = "[0,0," << (Math::sin(theta/2.0)).to_s << "," << (Math::cos(theta/2.0)).to_s << "]"

    #print "Generating Bin[" << i.to_s << "]: " << x.to_s << " , " << y.to_s << "\n"

    #Put one in the simulator
    f << "  <rosparam ns=\"simulation/simulator/obstacles\">\n"
    f << "    bin" << i.to_s << ":\n"
    f << "      type: obstacle\n"
    f << "      geometries:\n"
    f << "        -\n"
    f << "          name: base\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.6,1.0,0.6]\n"
    f << "            material: yellow\n"
    f << "          config:\n"
    f << "            position: [" << x.to_s << "," << y.to_s << ",0.3]\n"
    f << "            orientation: " << quat << "\n"
    f << "  </rosparam>\n\n"

    #And one in the planning
    f << "  <rosparam ns=\"planning/world_model/simulator/obstacles\">\n"
    f << "    bin" << i.to_s << ":\n"
    f << "      type: obstacle\n"
    f << "      geometries:\n"
    f << "        -\n"
    f << "          name: base\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.6,1.0,0.6]\n"
    f << "            material: yellow\n"
    f << "          config:\n"
    f << "            position: [" << x.to_s << "," << y.to_s << ",0.3]\n"
    f << "            orientation: " << quat << "\n"
    f << "  </rosparam>\n\n"

end

# = Generate a baxter at the target location = #
def generate_baxter( f, x, y, theta, i )
    f << "  <!-- Baxter " << i.to_s << "-->\n"

    #SIMULATION
    #Load this dude's consumer controller
    #f << "  <rosparam command=\"load\" ns=\"simulation/simulator/subsystems/consumer" << i.to_s << "_left\">\n"
    #f << "    template: \"controller\"\n"
    #f << "  </rosparam>\n\n"

    #f << "  <rosparam command=\"load\" ns=\"simulation/simulator/subsystems/consumer" << i.to_s << "_right\">\n"
    #f << "    template: \"controller\"\n"
    #f << "  </rosparam>\n\n"

    #Set the plant parameters
    f << "  <rosparam command=\"load\" ns=\"simulation/simulator/subsystems/consumer/subsystems/router/subsystems/baxter_" << i.to_s << "_left\">\n"
    f << "    template: \"plant\"\n"
    f << "    initial_state: [0.0,-1.34394,-0.42624,2.61800,-2.84100,2.09400,-0.06762,0]\n"
    f << "    hand: left\n"
    a = Math::cos(-theta)
    if a.abs < 0.000001
        a = 0
    end
    b = Math::sin(-theta)
    if b.abs < 0.000001
        b = 0
    end
    c = -1.0*Math::sin(-theta)
    if c.abs < 0.000001
        c = 0
    end
    d = Math::cos(-theta)
    if d.abs < 0.000001
        d = 0
    end
    f << "    forward_transform_rotation: [" << a.to_s << "," << b.to_s << ",0," << c.to_s << "," << d.to_s << ",0,0,0,1]\n"
    f << "    forward_transform_translation: [" << x.to_s << "," << y.to_s << ",0.785]\n"
    #f << "    inverse_transform_rotation: [" << (Math::cos(theta)).to_s << "," << (Math::sin(theta)).to_s << ",0," << (-1.0*Math::sin(theta)).to_s << "," << (Math::cos(theta)).to_s << ",0,0,0,1]\n"
    #f << "    inverse_transform_translation: [" << (-1.0*x).to_s << "," << (-1.0*y).to_s << ",-0.785]\n"
    f << "  </rosparam>\n\n"

    f << "  <rosparam command=\"load\" ns=\"simulation/simulator/subsystems/consumer/subsystems/router/subsystems/baxter_" << i.to_s << "_right\">\n"
    f << "    template: \"plant\"\n"
    f << "    initial_state: [0.0,-1.34394,0.42624,2.61800,2.84100,2.09400,-0.06762,0]\n"
    f << "    hand: right\n"
    f << "    forward_transform_rotation: [" << a.to_s << "," << b.to_s << ",0," << c.to_s << "," << d.to_s << ",0,0,0,1]\n"
    f << "    forward_transform_translation: [" << x.to_s << "," << y.to_s << ",0.785]\n"
    #f << "    inverse_transform_rotation: [" << (Math::cos(theta)).to_s << "," << (Math::sin(theta)).to_s << ",0," << (-1.0*Math::sin(theta)).to_s << "," << (Math::cos(theta)).to_s << ",0,0,0,1]\n"
    #f << "    inverse_transform_translation: [" << (-1.0*x).to_s << "," << (-1.0*y).to_s << ",-0.785]\n"
    f << "  </rosparam>\n\n"

    #Then, need to place a torso O_o
    f << "  <rosparam ns=\"simulation/simulator/obstacles\">\n"
    generate_torso( f, x, y, theta, i )
    f << "  </rosparam>\n\n"

    #PLANNING
    #Set the plant parameters
    f << "  <rosparam command=\"load\" ns=\"planning/world_model/simulator/subsystems/baxter_" << i.to_s << "_left\">\n"
    f << "    template: \"plant\"\n"
    f << "    initial_state: [0.0,-1.34394,-0.42624,2.61800,-2.84100,2.09400,-0.06762,0]\n"
    f << "    hand: left\n"
    f << "    forward_transform_rotation: [" << a.to_s << "," << b.to_s << ",0," << c.to_s << "," << d.to_s << ",0,0,0,1]\n"
    f << "    forward_transform_translation: [" << x.to_s << "," << y.to_s << ",0.785]\n"
    #f << "    inverse_transform_rotation: [" << (Math::cos(theta)).to_s << "," << (Math::sin(theta)).to_s << ",0," << (-1.0*Math::sin(theta)).to_s << "," << (Math::cos(theta)).to_s << ",0,0,0,1]\n"
    #f << "    inverse_transform_translation: [" << (-1.0*x).to_s << "," << (-1.0*y).to_s << ",-0.785]\n"
    #f << "    planning_contexts:\n"
    #f << "      space" << i.to_s << ":\n"
    #f << "        type: full_mapping\n"
    f << "  </rosparam>\n\n"

    f << "  <rosparam command=\"load\" ns=\"planning/world_model/simulator/subsystems/baxter_" << i.to_s << "_right\">\n"
    f << "    template: \"plant\"\n"
    f << "    initial_state: [0.0,-1.34394,0.42624,2.61800,2.84100,2.09400,-0.06762,0]\n"
    f << "    hand: right\n"
    f << "    forward_transform_rotation: [" << a.to_s << "," << b.to_s << ",0," << c.to_s << "," << d.to_s << ",0,0,0,1]\n"
    f << "    forward_transform_translation: [" << x.to_s << "," << y.to_s << ",0.785]\n"
    #f << "    inverse_transform_rotation: [" << (Math::cos(theta)).to_s << "," << (Math::sin(theta)).to_s << ",0," << (-1.0*Math::sin(theta)).to_s << "," << (Math::cos(theta)).to_s << ",0,0,0,1]\n"
    #f << "    inverse_transform_translation: [" << (-1.0*x).to_s << "," << (-1.0*y).to_s << ",-0.785]\n"
    #f << "    planning_contexts:\n"
    #f << "      space" << i.to_s << ":\n"
    #f << "        type: full_mapping\n"
    f << "  </rosparam>\n\n"

    #Create the motion planners responsible for this arm
    #Transit
    f << "  <rosparam ns=\"planning/task_planner/planners/\" >\n"
    f << "    planner_" << (i).to_s << "_left_move:\n"
    f << "      type: " << PLANNER_TYPE << "\n"
    f << "      heuristic_search:\n"
    f << "        type: prm_astar\n"
    f << "      space_name: \"full_space\"\n"
    f << "      visualization_body: \"simulator/baxter_left/end_effector\"\n"
    f << "      visualization_bodies: [\"simulator/baxter_left/end_effector\"]\n"
    f << "      visualize_graph: false\n"
    f << "      visualize_solutions: false\n"
    f << "      graph_color: \"green\"\n"
    serfile = "graph_" << i.to_s << "_left_move.txt"
    if $preprocessing
        f << "      serialize_file: " << serfile << "\n"
    else
        f << "      deserialize_file: " << serfile << "\n"
    end
    f << "  </rosparam>\n\n"

    #Transfer
    f << "  <rosparam ns=\"planning/task_planner/planners/\" >\n"
    for g in 0 .. NUM_GRASPS-1
        f << "    planner_" << (i).to_s << "_left_transfer_" + g.to_s + ":\n"
        f << "      type: " << PLANNER_TYPE << "\n"
        f << "      heuristic_search:\n"
        f << "        type: prm_astar\n"
        f << "      space_name: \"full_space\"\n"
        f << "      visualization_body: \"simulator/baxter_left/end_effector\"\n"
        f << "      visualization_bodies: [\"simulator/baxter_left/end_effector\"]\n"
        f << "      visualize_graph: false\n"
        f << "      visualize_solutions: false\n"
        f << "      graph_color: \"green\"\n"
        serfile = "graph_" << i.to_s << "_left_transfer_" + g.to_s + ".txt"
        if $preprocessing
            f << "      serialize_file: " << serfile << "\n"
        else
            f << "      deserialize_file: " << serfile << "\n"
        end
    end
    f << "  </rosparam>\n\n"


    #Transit
    f << "  <rosparam ns=\"planning/task_planner/planners/\" >\n"
    f << "    planner_" << (i).to_s << "_right_move:\n"
    f << "      type: " << PLANNER_TYPE << "\n"
    f << "      heuristic_search:\n"
    f << "        type: prm_astar\n"
    f << "      space_name: \"full_space\"\n"
    f << "      visualization_body: \"simulator/baxter_right/end_effector\"\n"
    f << "      visualization_bodies: [\"simulator/baxter_right/end_effector\"]\n"
    f << "      visualize_graph: false\n"
    f << "      visualize_solutions: false\n"
    f << "      graph_color: \"green\"\n"
    serfile = "graph_" << i.to_s << "_right_move.txt"
    if $preprocessing
        f << "      serialize_file: " << serfile << "\n"
    else
        f << "      deserialize_file: " << serfile << "\n"
    end
    f << "  </rosparam>\n\n"

    #Transfer
    f << "  <rosparam ns=\"planning/task_planner/planners/\" >\n"
    for g in 0 .. NUM_GRASPS-1
        f << "    planner_" << (i).to_s << "_right_transfer_" + g.to_s + ":\n"
        f << "      type: " << PLANNER_TYPE << "\n"
        f << "      heuristic_search:\n"
        f << "        type: prm_astar\n"
        f << "      space_name: \"full_space\"\n"
        f << "      visualization_body: \"simulator/baxter_right/end_effector\"\n"
        f << "      visualization_bodies: [\"simulator/baxter_right/end_effector\"]\n"
        f << "      visualize_graph: false\n"
        f << "      visualize_solutions: false\n"
        f << "      graph_color: \"green\"\n"
        serfile = "graph_" << i.to_s << "_right_transfer_" + g.to_s + ".txt"
        if $preprocessing
            f << "      serialize_file: " << serfile << "\n"
        else
            f << "      deserialize_file: " << serfile << "\n"
        end
    end
    f << "  </rosparam>\n\n"


    #Then, need to place a torso
    f << "  <rosparam ns=\"planning/world_model/simulator/obstacles\">\n"
    generate_torso( f, x, y, theta, i )
    f << "  </rosparam>\n\n"

end

# = Generate a Kiva shelf at the target location = #
def generate_shelf( f, x, y, i )
    #Create a local offset y
    ly = y + 0.12

    #SIMULATION
    f << "  <rosparam ns=\"simulation/simulator/obstacles\">\n"
    f << "    kiva_shelf" << i.to_s << ":\n"
    f << "      type: obstacle\n"
    f << "      geometries:\n"
    f << "        -\n"
    f << "          name: base\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [1.0,1.0,1.2]\n"
    f << "            material: light_grey\n"
    f << "          config:\n"
    f << "            position: [" << x.to_s << "," << ly.to_s << ",0.6]\n"
    f << "            orientation: [0, 0, 0, 1]\n"
    f << "        -\n"
    f << "          name: center\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.05,1.0,0.3]\n"
    f << "            material: light_grey\n"
    f << "          config:\n"
    f << "            position: [" << x.to_s << "," << ly.to_s << ",1.35]\n"
    f << "            orientation: [0, 0, 0, 1]\n"
    f << "        -\n"
    f << "          name: left\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.05,1.0,0.3]\n"
    f << "            material: light_grey\n"
    f << "          config:\n"
    f << "            position: [" << (x - 0.475).to_s << "," << ly.to_s << ",1.35]\n"
    f << "            orientation: [0, 0, 0, 1]\n"
    f << "        -\n"
    f << "          name: right\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.05,1.0,0.3]\n"
    f << "            material: light_grey\n"
    f << "          config:\n"
    f << "            position: [" << (x + 0.475).to_s << "," << ly.to_s << ",1.35]\n"
    f << "            orientation: [0, 0, 0, 1]\n"
    f << "        -\n"
    f << "          name: top\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [1.0,1.0,0.2]\n"
    f << "            material: light_grey\n"
    f << "          config:\n"
    f << "            position: [" << x.to_s << "," << ly.to_s << ",1.6]\n"
    f << "            orientation: [0, 0, 0, 1]\n"
    f << "  </rosparam>\n\n"

    #PLANNING
    f << "  <rosparam ns=\"planning/world_model/simulator/obstacles\">\n"
    f << "    kiva_shelf" << i.to_s << ":\n"
    f << "      type: obstacle\n"
    f << "      geometries:\n"
    f << "        -\n"
    f << "          name: base\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [1.0,1.0,1.2]\n"
    f << "            material: light_grey\n"
    f << "          config:\n"
    f << "            position: [" << x.to_s << "," << ly.to_s << ",0.6]\n"
    f << "            orientation: [0, 0, 0, 1]\n"
    f << "        -\n"
    f << "          name: center\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.05,1.0,0.3]\n"
    f << "            material: light_grey\n"
    f << "          config:\n"
    f << "            position: [" << x.to_s << "," << ly.to_s << ",1.35]\n"
    f << "            orientation: [0, 0, 0, 1]\n"
    f << "        -\n"
    f << "          name: left\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.05,1.0,0.3]\n"
    f << "            material: light_grey\n"
    f << "          config:\n"
    f << "            position: [" << (x - 0.475).to_s << "," << ly.to_s << ",1.35]\n"
    f << "            orientation: [0, 0, 0, 1]\n"
    f << "        -\n"
    f << "          name: right\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.05,1.0,0.3]\n"
    f << "            material: light_grey\n"
    f << "          config:\n"
    f << "            position: [" << (x + 0.475).to_s << "," << ly.to_s << ",1.35]\n"
    f << "            orientation: [0, 0, 0, 1]\n"
    f << "        -\n"
    f << "          name: top\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [1.0,1.0,0.2]\n"
    f << "            material: light_grey\n"
    f << "          config:\n"
    f << "            position: [" << x.to_s << "," << ly.to_s << ",1.6]\n"
    f << "            orientation: [0, 0, 0, 1]\n"
    f << "  </rosparam>\n\n"

end

# = Generate a torso at the target location = #
def generate_torso( f, x, y, theta, i )
    #Simple Torso generation - Begin by figuring out where these thigns are
    quat = "[0,0," << (Math::sin(theta/2.0)).to_s << "," << (Math::cos(theta/2.0)).to_s << "]"
    ped_x = -0.04 * Math::cos(theta)
    ped_y = -0.04 * Math::sin(theta)

    tor_x = -0.065 * Math::cos(theta)
    tor_y = -0.065 * Math::sin(theta)

    hed_x = 0.04 * Math::cos(theta)
    hed_y = 0.04 * Math::sin(theta)

    #Now actually output the things
    f << "    baxter" << i.to_s << "_torso:\n"
    f << "      type: obstacle\n"
    f << "      geometries:\n"
    f << "        -\n"
    f << "          name: pedestal" << i.to_s << "\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.90, 0.80, 0.05]\n"
    f << "            material: baxter_grey\n"
    f << "          config:\n"
    f << "            position: [" << (ped_x + x).to_s << ", " << (ped_y + y).to_s << ", 0.31]\n"
    f << "            orientation: " << quat << "\n"
    if PRETTY
        f << "          visualization_geometry:\n"
        f << "            type: mesh\n"
        f << "            filename: meshes/base/pedestal.osg\n"
        f << "            material: baxter_grey\n"
    end
    f << "        -\n"
    f << "          name: torso" << i.to_s << "\n"
    f << "          collision_geometry:\n"
    f << "            type: cylinder\n"
    f << "            radius: 0.15\n"
    f << "            height: 1.6\n"
    f << "            material: baxter_red\n"
    f << "          config:\n"
    f << "            position: [" << (tor_x + x).to_s << ", " << (tor_y + y).to_s << ", 1.085]\n"
    f << "            orientation: " << quat << "\n"
    if PRETTY
        f << "          visualization_geometry:\n"
        f << "            type: mesh\n"
        f << "            filename: meshes/torso/base.osg\n"
        f << "            material: baxter_red\n"
    end
    f << "        -\n"
    f << "          name: head" << i.to_s << "\n"
    f << "          collision_geometry:\n"
    f << "            type: cylinder\n"
    f << "            radius: 0.1\n"
    f << "            height: 0.25\n"
    f << "            material: baxter_grey\n"
    f << "          config:\n"
    f << "            position: [" << (hed_x + x).to_s << ", " << (hed_y + y).to_s << ", 1.585]\n"
    f << "            orientation: " << quat << "\n"
    if PRETTY
        f << "          visualization_geometry:\n"
        f << "            type: box\n"
        f << "            dims: [0.01, 0.01, 0.01]\n"
        f << "            material: baxter_grey\n"
    end
end

# = Generate a Table at the target location = #
def generate_table( f, x, y, i )
    f << "  <!-- Table " << i.to_s << "-->\n"
    #Simulation's copy
    f << "  <rosparam ns=\"simulation/simulator/obstacles\">\n"
    f << "    table" << i.to_s << ":\n"
    f << "      type: obstacle \n"
    f << "      geometries:\n"
    f << "        -\n"
    f << "          name: top" << i.to_s << "\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [1.0,1.0,0.06]\n"
    f << "            material: sparky_orange\n"
    f << "          config:\n"
    f << "            position: [" << x.to_s << ", " << y.to_s << ", 0.77]\n"
    f << "            orientation: [0,0,0,1]\n"
    f << "        -\n"
    f << "          name: leg_1_" << i.to_s << "\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.06,0.06,0.78]\n"
    f << "            material: gold\n"
    f << "          config:\n"
    f << "            position: [" << (x+0.465).to_s << ", " << (y+0.465).to_s << ", 0.39]\n"
    f << "            orientation: [0,0,0,1]\n"
    f << "        -\n"
    f << "          name: leg_2_" << i.to_s << "\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.06,0.06,0.78]\n"
    f << "            material: gold\n"
    f << "          config:\n"
    f << "            position: [" << (x-0.465).to_s << ", " << (y+0.465).to_s << ", 0.39]\n"
    f << "            orientation: [0,0,0,1]\n"
    f << "        -\n"
    f << "          name: leg_3_" << i.to_s << "\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.06,0.06,0.78]\n"
    f << "            material: gold\n"
    f << "          config:\n"
    f << "            position: [" << (x-0.465).to_s << ", " << (y-0.465).to_s << ", 0.39]\n"
    f << "            orientation: [0,0,0,1]\n"
    f << "        -\n"
    f << "          name: leg_4_" << i.to_s << "\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.06,0.06,0.78]\n"
    f << "            material: gold\n"
    f << "          config:\n"
    f << "            position: [" << (x+0.465).to_s << ", " << (y-0.465).to_s << ", 0.39]\n"
    f << "            orientation: [0,0,0,1]\n"
    f << "  </rosparam>\n\n"

    #Planning's Copy
    f << "  <rosparam ns=\"planning/world_model/simulator/obstacles\">\n"
    f << "    table" << i.to_s << ":\n"
    f << "      type: obstacle \n"
    f << "      geometries:\n"
    f << "        -\n"
    f << "          name: top" << i.to_s << "\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [1.0,1.0,0.06]\n"
    f << "            material: sparky_orange\n"
    f << "          config:\n"
    f << "            position: [" << x.to_s << ", " << y.to_s << ", 0.77]\n"
    f << "            orientation: [0,0,0,1]\n"
    f << "        -\n"
    f << "          name: leg_1_" << i.to_s << "\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.06,0.06,0.78]\n"
    f << "            material: gold\n"
    f << "          config:\n"
    f << "            position: [" << (x+0.465).to_s << ", " << (y+0.465).to_s << ", 0.39]\n"
    f << "            orientation: [0,0,0,1]\n"
    f << "        -\n"
    f << "          name: leg_2_" << i.to_s << "\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.06,0.06,0.78]\n"
    f << "            material: gold\n"
    f << "          config:\n"
    f << "            position: [" << (x-0.465).to_s << ", " << (y+0.465).to_s << ", 0.39]\n"
    f << "            orientation: [0,0,0,1]\n"
    f << "        -\n"
    f << "          name: leg_3_" << i.to_s << "\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.06,0.06,0.78]\n"
    f << "            material: gold\n"
    f << "          config:\n"
    f << "            position: [" << (x-0.465).to_s << ", " << (y-0.465).to_s << ", 0.39]\n"
    f << "            orientation: [0,0,0,1]\n"
    f << "        -\n"
    f << "          name: leg_4_" << i.to_s << "\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.06,0.06,0.78]\n"
    f << "            material: gold\n"
    f << "          config:\n"
    f << "            position: [" << (x+0.465).to_s << ", " << (y-0.465).to_s << ", 0.39]\n"
    f << "            orientation: [0,0,0,1]\n"
    f << "  </rosparam>\n\n"
end

# ====================================================================
# ====================================================================
# ====================================================================
# ====================================================================
# ====================================================================
# ====================================================================
# ====================================================================
# ====================================================================
# ====================================================================
# ====================================================================
# ====================================================================
# ====================================================================
# ====================================================================
# ====================================================================
# ====================================================================
# ====================================================================
# ====================================================================
# ====================================================================

# = = = = = = #
# = Driver  = #
def main()
    expr = ""
    if MODE == 0 #KIVA shelves
        expr = "shelves"
        $cupheight = "0.16"
    else
        expr = "table"
        $cupheight = "0.3"
    end

    numstring = NUM_SYSTEMS.to_s

    launchfile = ""
    if $preprocessing
        launchfile = expr + "_" + numstring + "_preprocess.launch"
    else
        launchfile = expr + "_" + numstring + "_search.launch"
    end

    f = File.open(launchfile, "w")

    #Output basic info and tag
    f << "<!-- Auto generated launch file for Baxter: " << NUM_SYSTEMS.to_s << " " << NUM_BINS.to_s << " " << MODE.to_s << " -->\n"
    f << "<launch>\n"

    #Also output the space names xml include
    f << "  <rosparam command=\"load\" file=\"$(find prx_input)/templates/spaces/space_types.yaml\"/>\n\n"

    # = = = = = = = = = = = = =
    #Parameters: Simulation
    # = = = = = = = = = = = = =
    f << "  <!-- = = = = = = = -->\n"
    f << "  <!--   Simulation  -->\n"
    f << "  <!-- = = = = = = = -->\n"

    f << "  <!-- Load independent files -->\n"
    f << "  <rosparam command=\"load\" ns=\"simulation\" file=\"$(find prx_input)/templates/applications/empty_application.yaml\"/>\n"
    f << "  <rosparam command=\"load\" ns=\"simulation\" file=\"$(find manipulation)/input/simulation/manipulation_simulator.yaml\"/>\n"
    f << "  <rosparam command=\"load\" ns=\"simulation/simulator/subsystems/consumer/\" file=\"$(find prx_input)/templates/controllers/consumer.yaml\"/>\n"
    f << "  <rosparam command=\"load\" ns=\"simulation/simulator/subsystems/consumer/subsystems/router/\" file=\"$(find prx_input)/templates/controllers/router.yaml\"/>\n\n"

    f << "  <!-- Load template files -->\n"
    if PRETTY
        f << "  <rosparam command=\"load\" ns=\"simulation/plant\" file=\"$(find baxter)input/urdf/baxter.yaml\"/>\n\n"
    else
        f << "  <rosparam command=\"load\" ns=\"simulation/plant\" file=\"$(find baxter)input/urdf/baxter_nomesh.yaml\"/>\n\n"
    end

    #Now, since we do know how many things we are creating, we can set up the manipulation simulator lists
    f << "  <rosparam command=\"load\" ns=\"simulation/simulator\">\n"
    f << "    manipulator_names:\n"
    for i in 1 .. NUM_SYSTEMS
        f << "      -\n"
        f << "        simulator/consumer/router/baxter_" << (i).to_s << "_left\n"
        f << "      -\n"
        f << "        simulator/consumer/router/baxter_" << (i).to_s << "_right\n"
    end
    f << "    sensing_info:\n"
    f << "      type: grasp_sensing_info\n"
    f << "      sources: [\"grasp_sensor1\"]\n"
    f << "      update_delay: 0.02\n"
    f << "    grasp_sensor_source: \"grasp_sensor1\"\n"
    f << "  </rosparam>\n"

    f << "  <rosparam ns=\"simulation/simulator/sensing\">\n"
    f << "    type: sensing_model\n"
    f << "    sensors:\n"
    f << "      grasp_sensor1:\n"
    f << "        type: simulated_grasp_sensor\n"
    f << "        source: \"grasp_sensor1\"\n"
    f << "        sensor_delay: 0.02\n"
    f << "  </rosparam>\n"


    # = = = = = = = = = = = = =
    #Parameters: Planning
    # = = = = = = = = = = = = =
    f << "  <!-- = = = = = = = -->\n"
    f << "  <!--    Planning   -->\n"
    f << "  <!-- = = = = = = = -->\n\n"

    #Now, create the consumer mapping as well
    f << "  <!-- System Mappings -->\n"
    f << "  <rosparam ns=\"planning\">\n"
    f << "    system_mapping:\n"
    for i in 1 .. NUM_SYSTEMS
        f << "      -\n"
        f << "        pair: [simulator/consumer/router/baxter_" << i.to_s << "_left, world_model/simulator/baxter_" << i.to_s << "_left]\n"
        f << "      -\n"
        f << "        pair: [simulator/consumer/router/baxter_" << i.to_s << "_right, world_model/simulator/baxter_" << i.to_s << "_right]\n"
    end
    f << "    consumer: simulator/consumer\n"
    f << "  </rosparam>\n\n"

    f << "  <rosparam command=\"load\" ns=\"planning\" file=\"$(find prx_input)/templates/spaces/space_types.yaml\"/>\n"
    f << "  <rosparam command=\"load\" ns=\"planning/world_model\" file=\"$(find manipulation)/input/simulation/manipulation_simulator.yaml\"/>\n"
    f << "  <rosparam command=\"load\" ns=\"planning\" file=\"$(find prx_input)/templates/planning_applications/single_query_application.yaml\"/>\n"
    f << "  <rosparam command=\"load\" ns=\"planning/task_planner/\" file=\"$(find many_arm)/input/templates/mm_forward_tp.yaml\"/>\n\n"

    f << "  <!-- Load template files -->\n"
    f << "  <rosparam command=\"load\" ns=\"planning/plant\" file=\"$(find baxter)input/urdf/baxter.yaml\"/>\n\n"

    #Output planning seed stuff
    f << "  <rosparam ns=\"planning\">\n"
    if SPECIFY_RANDOM
        f << "    random_seed: "
        if(RANDOM_RANDOM)
            f << rand(65535).to_s
        else
            f << RANDOM_SEED.to_s
        end
    end
    f << "\n"
    f << "    consumer_mapping:\n"
    f << "      -\n"
    f << "        pair: [simulator/consumer, full_space]\n"
    f << "  </rosparam>\n\n"

    #Alright, we need the task planner to be set up in $preprocessing mode or not
    f << "  <rosparam ns=\"planning/task_planner\">\n"
    f << "    preprocess: " << $preprocessing.to_s << "\n"
    f << "    automaton_file: automaton.txt\n"
    f << "    stable_sig_filename: stable_sig_" << expr << "_" << NUM_SYSTEMS << "_" << OBJECT_K << ".txt\n"
    f << "    handoff_sig_filename: handoff_sig_" << expr << "_" << NUM_SYSTEMS << "_" << OBJECT_K << ".txt\n"
    f << "    object_k: " << OBJECT_K << "\n"
    f << "    pose_samples: " << SAMPLE_ATTEMPTS << "\n"
    f << "    grasp_samples: " << GRASP_SAMPLES << "\n"
    f << "    discrete_search: " << DISCRETE_SEARCH << "\n"
    f << "    delay_planning: " << DELAY_PLANNING << "\n"
    f << "    approach_attempts: " << APPROACH_ATTEMPTS << "\n"
    f << "    manipulator_mapping:\n"
    for i in 1 .. NUM_SYSTEMS
        f << "      -\n"
        f << "        system: simulator/baxter_" << i.to_s << "_left\n"
        f << "        index: " << (2*(i-1)).to_s << "\n"
        f << "      -\n"
        f << "        system: simulator/baxter_" << i.to_s << "_right\n"
        f << "        index: " << (2*i-1).to_s << "\n"
    end
    f << "  </rosparam>\n\n"

    f << "  <rosparam ns=\"planning/problems\">\n"
    f << "    benchmark_problem:\n"
    f << "      specification:\n"
    f << "        type: \"manipulation_specification\"\n"
    f << "        sampler:\n"
    f << "          type: manip_sampler\n"
    f << "          min_theta: 0.0\n"
    f << "          max_theta: 6.284\n"
    f << "          max_tries: 15\n"
    f << "        validity_checker:\n"
    f << "          type: world_model_validity_checker\n"
    f << "        distance_metric:\n"
    f << "          type: linear_distance_metric\n"
    f << "        local_planner:\n"
    f << "          type: bvp_local_planner\n"
    f << "        stopping_criteria:\n"
    f << "          elements:\n"
    f << "            criterion1:\n"
    f << "              type: iteration_criterion\n"
    f << "              condition: " << ITERATIONS << "\n"
    f << "        manipulation_sampler:\n"
    f << "          type: manip_sampler\n"
    f << "          min_theta: 0.0\n"
    f << "          max_theta: 6.284\n"
    f << "          max_tries: 15\n"
    f << "      query:\n"
    f << "        type: \"motion_planning_query\"\n"
    f << "        goal:\n"
    f << "          type: \"goal_state\"\n"
    f << "          distance_metric:\n"
    f << "            type: linear_distance_metric\n"
    f << "          goal_state: ["
    for i in 1 .. NUM_SYSTEMS
        f << "0.00000,-1.34394,-0.42624,2.61800,-2.84100,2.09400,-0.06762,0.00000,0.00000,-1.34394,0.42624,2.61800,2.84100,2.09400,-0.06762,0.00000,"
    end
    #Mode table
    if MODE == 1
        f << "-0.700,1.000,0.800,0.000,0.000,0.000,1.000"
    else #Mode shelves
        f << "0.700,0.000,0.700,0.0,0.0,0.0,1.0"
    end
    f << "]\n"
    f << "  </rosparam>\n\n"

    # All of the manipulation simulator information also needs to be in the planning side as well
    f << "  <rosparam command=\"load\" ns=\"planning/world_model/simulator\">\n"
    f << "    manipulator_names:\n"
    for i in 1 .. NUM_SYSTEMS
        f << "      -\n"
        f << "        simulator/baxter_" << (i).to_s << "_left\n"
        f << "      -\n"
        f << "        simulator/baxter_" << (i).to_s << "_right\n"
    end
    f << "    sensing_info:\n"
    f << "      type: grasp_sensing_info\n"
    f << "      sources: [\"grasp_sensor2\"]\n"
    f << "      update_delay: 0.02\n"
    f << "    grasp_sensor_source: \"grasp_sensor2\"\n"
    f << "  </rosparam>\n"

    f << "  <rosparam ns=\"planning/world_model/simulator/sensing\">\n"
    f << "    type: sensing_model\n"
    f << "    sensors:\n"
    f << "      grasp_sensor2:\n"
    f << "        type: simulated_grasp_sensor\n"
    f << "        source: \"grasp_sensor2\"\n"
    f << "        sensor_delay: 0.02\n"
    f << "  </rosparam>\n\n"

    # = = = = = = = = = = = = =
    #Parameters: Environment
    # = = = = = = = = = = = = =
    f << "  <!-- = = = = = = = -->\n"
    f << "  <!--  Environment  -->\n"
    f << "  <!-- = = = = = = = -->\n\n"

    #Before we get into the nitty-gritty, there are objects to be manipulated!
    f << "  <rosparam command=\"load\" ns=\"simulation/cup\" file=\"$(find manipulation)/input/simulation/plants/cups.yaml\"/>\n"
    f << "  <rosparam ns=\"simulation/cup\">\n"
    f << "    state_space:\n"
    f << "        min: [-2, -3, -100, -1, -1, -1, -1]\n"
    f << "        max: [22, 3, 100, 1, 1, 1, 1]\n"
    f << "        scale: [0,0,0,0,0,0,0]\n"
    f << "    geometries:\n"
    f << "      -\n"
    f << "        name: body\n"
    f << "        collision_geometry:\n"
    f << "          type: cylinder\n"
    f << "          radius: 0.02\n"
    f << "          height: " + $cupheight + "\n"
    f << "          material: orange\n"
    f << "  </rosparam>\n\n"

    f << "  <rosparam command=\"load\" ns=\"planning/cup\" file=\"$(find manipulation)/input/simulation/plants/cups.yaml\"/>\n"
    f << "  <rosparam ns=\"planning/cup\">\n"
    f << "    state_space:\n"
    f << "        min: [-2, -3, -100, -1, -1, -1, -1]\n"
    f << "        max: [22, 3, 100, 1, 1, 1, 1]\n"
    f << "        scale: [0,0,0,0,0,0,0]\n"
    f << "    geometries:\n"
    f << "      -\n"
    f << "        name: body\n"
    f << "        collision_geometry:\n"
    f << "          type: cylinder\n"
    f << "          radius: 0.02\n"
    f << "          height: " + $cupheight + "\n"
    f << "          material: orange\n"
    f << "  </rosparam>\n\n"

    #First of all, we need to figure out which benchmark we are running
    # = = = = = = =
    # = Shelves
    # = = = = = = =
    if MODE == 0 #Kiva Shelves
        #First, let's place some Kiva Shelves and the Baxters
        for i in 1 .. NUM_SYSTEMS
            #Generate the shelf and Baxter in front of it
            base_x = 1.7*i
            base_y = 1.23
            generate_shelf( f, base_x, base_y, i )
            generate_baxter( f, base_x, 0, Math::PI/2.0, i )
            # = Then, let's generate some "Objects" in the shelf =
            f << "  <!-- Objects -->\n"
            bins = Array.new(12,0)
            #for j in 1 .. NUM_OBJECTS
            for j in [1]
                done = false
                #Select a bin
                while !done
                    b = rand(12)
                    if( bins[b] == 0 )
                        bins[b] = 1
                        done = true
                    end
                end
            end
            for j in 0 .. 11
                #If this bin was selected to have something
                if bins[j] == 1
                    #Compute the x_off depending on the bin
                    x_off = 0
                    xmod = j%4
                    if xmod == 0
                        x_off = COL_1
                    elsif xmod == 1
                        x_off = COL_2
                    elsif xmod == 2
                        x_off = COL_3
                    elsif xmod == 3
                        x_off = COL_4
                    end
                    #Then Compute the z_off based on the bin
                    z_off = 0
                    zmod = j/4
                    if zmod == 0
                        z_off = ROW_5
                    elsif zmod == 1
                        z_off = ROW_5
                    elsif zmod == 2
                        z_off = ROW_5
                    end
                    #Then output the stuff
                    f << "  <rosparam command=\"load\" ns=\"simulation/simulator/subsystems/cup" << i.to_s << "_" << j.to_s << "\">\n"
                    f << "    template: \"cup\"\n"
                    f << "    initial_state: [" << (x_off + base_x).to_s << ", " << (Y_OFF + base_y).to_s << ", " << (z_off + 0.11).to_s << ", 0,0,0,1]\n"
                    f << "  </rosparam>\n\n"

                    f << "  <rosparam command=\"load\" ns=\"planning/world_model/simulator/subsystems/cup" << i.to_s << "_" << j.to_s << "\">\n"
                    f << "    template: \"cup\"\n"
                    f << "    initial_state: [" << (x_off + base_x).to_s << ", " << (Y_OFF + base_y).to_s << ", " << (z_off + 0.11).to_s << ", 0,0,0,1]\n"
                    f << "  </rosparam>\n\n"

                    #print "Putting the object in bin: " << j.to_s << ": " << x_off.to_s << " , " << z_off.to_s << "\n"
                end
            end
            # = TDK: Generate some random poses
            if TDK_GENERATE_POSES
                tdk = File.open("kiva_poses_" << i.to_s << ".txt", "w")
                tdkout = File.open("kiva_front_poses_" << i.to_s << ".txt", "w")
                for j in 1 .. TDK_NUM_POSES
                    #Select a random bin to find its offsets
                    r = rand(12)
                    #Compute the x_off depending on the bin
                    x_off = 0
                    xmod = r%4
                    if xmod == 0
                        x_off = COL_1
                    elsif xmod == 1
                        x_off = COL_2
                    elsif xmod == 2
                        x_off = COL_3
                    elsif xmod == 3
                        x_off = COL_4
                    end
                    #Then Compute the z_off based on the bin
                    z_off = 0
                    zmod = r/4
                    if zmod == 0
                        z_off = ROW_4
                    elsif zmod == 1
                        z_off = ROW_5
                    elsif zmod == 2
                        z_off = ROW_6
                    end
                    #Compute a random offset from the center
                    # TODO: Should probably parameterize the shelf sizes
                    x_rand = (rand() * (0.195 - 2.0*(TDK_POSE_BUFFER))) - (0.0975 - TDK_POSE_BUFFER)
                    y_rand = (rand() * (0.190 - 2.0*(TDK_POSE_BUFFER))) - (0.1050 - TDK_POSE_BUFFER)
                    tdk << "-\n"
                    tdk << "  pose: [" << (x_off + x_rand).to_s << "," << (Y_OFF + base_y + y_rand + 0.05).to_s << "," << (z_off + 0.07 + 2.0).to_s << ",0,0,0,1]\n"
                    tdkout << "-\n"
                    tdkout << "  pose: [" << (x_off + x_rand).to_s << "," << (Y_OFF + base_y - 0.055).to_s << "," << (z_off + 2.102).to_s << ",0.00,0.00,0.7071067811800000324495841,0.7071067811800000324495841]\n"
                end
                tdk.close()
                tdkout.close()
            end

        end
        f << "  <!-- Bins -->\n"
        #Now, we need to place bins where appropriate
        if NUM_BINS >= 1
            generate_bin( f, 0.7, 0, 0, 1 )
        end
        if NUM_BINS >= 2
            generate_bin( f, 1.0 + 1.7*NUM_SYSTEMS, 0, Math::PI, 2 )
        end
        if NUM_BINS > 2
            print "Requesting more bins than this setup knows how to place!\n"
        end
    # = = = = = = =
    # = Long Table
    # = = = = = = =
    else #Zhe long table
        #First, let's make some tables
        for i in 1 .. (NUM_SYSTEMS+1)
            generate_table( f, i-1, 0, i )
        end
        #Then, place some Baxters around that loooong table
        for i in 1 .. NUM_SYSTEMS
            theta = (Math::PI/2.0) * ((-2*(i%2))+1)
            generate_baxter( f, i-0.5, (2*(i%2))-1, theta, i )
        end
        #Now, let's put some items on that long table
        nr_bins = (NUM_SYSTEMS+1) * 25
        bins = Array.new(nr_bins, 0)
        #for j in 1 .. (NUM_OBJECTS * (NUM_SYSTEMS+1))
        for j in [1]
            done = false
            #Select bins
            while !done
                b = rand(nr_bins)
                if( bins[b] == 0 )
                    bins[b] = 1
                    done = true
                end
            end
        end
        #Now, for every one of those that was selected, go ahead and put the object
        for j in 0 .. (nr_bins-1)
            if bins[j] == 1
                #Place it heah!!
                x_pos = 0.2 * (j % (5*(NUM_SYSTEMS+1))) - 0.4
                y_pos = 0.2 * (j / (5*(NUM_SYSTEMS+1))) - 0.4
                #Then output the stuff
                f << "  <rosparam command=\"load\" ns=\"simulation/simulator/subsystems/cup" << j.to_s << "\">\n"
                f << "    template: \"cup\"\n"
                f << "    initial_state: [" << x_pos.to_s << ", " << y_pos.to_s << ", 0.98, 0,0,0,1]\n"
                f << "  </rosparam>\n\n"

                f << "  <rosparam command=\"load\" ns=\"planning/world_model/simulator/subsystems/cup" << j.to_s << "\">\n"
                f << "    template: \"cup\"\n"
                f << "    initial_state: [" << x_pos.to_s << ", " << y_pos.to_s << ", 0.98, 0,0,0,1]\n"
                f << "  </rosparam>\n\n"

                #print "Putting the object in bin: " << j.to_s << ": " << x_pos.to_s << " , " << y_pos.to_s << "\n"
            end
        end
        #Finally, let's place some order bins
        if NUM_BINS > 0
            #The first bin goes on the top side next to the first Baxter
            generate_bin( f, -0.5, 1, 0, 1 )
        end
        if NUM_BINS > 1
            #The second bin goes on the top side next to the (second-to-)last Baxter
            generate_bin( f, 2*(((NUM_SYSTEMS-1)/2.0).floor) + 1.5, 1, Math::PI, 2 )
        end
        if NUM_BINS > 2
            #The third bin goes on the bottom side next to the first Baxter
            generate_bin( f, 0.5, -1, 0, 3 )
        end
        if NUM_BINS > 3
            #The fourth bin goes on the bottom side next to the first Baxter
            generate_bin( f, 2*(( (NUM_SYSTEMS) /2.0).floor) + 0.5, -1, 0, 4 )
        end
        if NUM_BINS > 4
            #TOO MANY BINS
            print "Requesting more bins than this setup knows how to place!\n"
        end
    end

    # = = = = = = = = = = = = =
    #Parameters: Visualization
    # = = = = = = = = = = = = =
    f << "  <!-- = = = = = = = -->\n"
    f << "  <!-- Visualization -->\n"
    f << "  <!-- = = = = = = = -->\n"
    f << "  <include file=\"$(find many_arm)input/visualization.launch\"/>\n\n"

    #Define the Nodes to launch
    if CREATE_PLANNING
        f << "  <!-- Define the planning node -->\n"
        f << "  <node name=\"planning\" pkg=\"prx_planning\" type=\"prx_planning\" required=\"false\" launch-prefix=\"\" output=\"screen\" args=\"planning\" />\n"
    end

    f << "  <!-- Define the simulation node -->\n"
    f << "  <node name=\"simulation\" pkg=\"prx_simulation\" type=\"prx_simulation\" required=\"true\" launch-prefix=\"\" output=\"screen\" args=\"simulation\" />\n"

    #Close out the launch tag: we're finally done!
    f << "\n</launch>\n\n"
end

$preprocessing = true
main()
$preprocessing = false
main()
