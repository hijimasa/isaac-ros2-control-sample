(sleep 1; echo -e 'import spawn\nspawn.main("URDF_PATH", ROBOT_X, ROBOT_Y, ROBOT_Z, ROBOT_ROLL, ROBOT_PITCH, ROBOT_YAW, ROBOT_FIXED)\n'; sleep 5) | telnet localhost 8223

