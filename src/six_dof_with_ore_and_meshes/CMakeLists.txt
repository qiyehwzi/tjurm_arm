cmake_minimum_required(VERSION 2.8.3)

project(six_dof_with_ore_and_meshes)



find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


