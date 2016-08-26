
number = 400

fout = File.open( number.to_s + "_agents.launch",'w')

fout << "<launch>\n\n"

for i in 1 .. number

  id = i.to_s

  fout << "  <rosparam command=\"load\" ns=\"simulation/simulator/subsystems/behavior" + id + "\">\n"
  fout << "    template: \"behavior_controller\"\n"
  fout << "    goal: [501, " + id + ", 1.02]\n"
  fout << "  </rosparam>\n\n"

  fout << "  <rosparam command=\"load\" ns=\"simulation/simulator/subsystems/behavior" + id + "/subsystems/follower" + id + "\">\n"
  fout << "    template: \"path_follower\"\n"
  fout << "  </rosparam>\n\n"

  fout << "  <rosparam command=\"load\" ns=\"simulation/simulator/subsystems/behavior" + id + "/subsystems/follower" + id + "/subsystems/vo" + id + "\">\n"
  fout << "    template: \"vo_controller\"\n"
  fout << "    sensing_info:\n"
  fout << "        system_path: \"simulator/behavior" + id + "/follower" + id + "/vo" + id + "/agent" + id + "/body\"\n"
  fout << "  </rosparam>\n\n"

  fout << "  <rosparam command=\"load\" ns=\"simulation/simulator/subsystems/behavior" + id + "/subsystems/follower" + id + "/subsystems/vo" + id + "/subsystems/agent" + id + "\">\n"
  fout << "    template: \"agent\"\n"
  fout << "    initial_state: [500, " + id + ", 1.02]\n"
  fout << "    input_control_space:\n"
  fout << "      min: [0, -3.1416]\n"
  fout << "      max: [2.7, 3.1416]\n"
  fout << "  </rosparam>\n\n"

end

fout << "<\/launch>\n"

fout.close()

