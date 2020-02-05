cmake_minimum_required(VERSION 2.8.3)
project(controller_mpc)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED 
	COMPONENTS
	message_generation
	roscpp
	tf
	std_msgs
	testbed_msgs
)

find_package (Eigen3 REQUIRED NO_MODULE)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS 
	roscpp
	message_runtime
	tf
	std_msgs
	testbed_msgs
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_library(mpc_solver
  externals/qpoases/SRC/Bounds.cpp
  externals/qpoases/SRC/Constraints.cpp
  externals/qpoases/SRC/CyclingManager.cpp
  externals/qpoases/SRC/Indexlist.cpp
  externals/qpoases/SRC/MessageHandling.cpp
  externals/qpoases/SRC/QProblem.cpp
  externals/qpoases/SRC/QProblemB.cpp
  externals/qpoases/SRC/SubjectTo.cpp
  externals/qpoases/SRC/Utils.cpp
  externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  model/quadrotor_mpc_codegen/acado_qpoases_interface.cpp
  model/quadrotor_mpc_codegen/acado_integrator.c
  model/quadrotor_mpc_codegen/acado_solver.c
  model/quadrotor_mpc_codegen/acado_auxiliary_functions.c)

target_link_libraries(mpc_solver
	${catkin_LIBRARIES})

target_include_directories(mpc_solver PUBLIC
  model/quadrotor_mpc_codegen/
  externals/qpoases
  externals/qpoases/INCLUDE
  externals/qpoases/SRC)

add_executable(mpc_controller_node 
	exec/mpc_controller_node.cpp 
	src/mpc_controller.cpp
)

add_dependencies(mpc_controller_node
	${catkin_EXPORTED_TARGETS}
	)

target_link_libraries(
	mpc_controller_node
	mpc_solver
	Eigen3::Eigen 
	${catkin_LIBRARIES}
)
