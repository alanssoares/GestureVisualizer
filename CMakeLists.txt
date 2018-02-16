# Define a versão minima do cmake para poder executar esse script
cmake_minimum_required(VERSION 2.8)

# Adiciona o flag ao compilador para usar c++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated -g")

# Define o nome do projeto
project(gesture_tracking_project)

#Acha diretorios do opengl, glut, pcl e vtk
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common io visualization console)

#Acha e linka diretorios
FIND_LIBRARY(NAMES HINTS "/usr/lib" "/opt/local/lib")
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})

# Inclui diretorios necessarios já encontrados
include_directories(${PCL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})

# Inclui definições da lib PCL
add_definitions (${PCL_DEFINITIONS})

# Inclui diretorios do projeto
include_directories ("${PROJECT_SOURCE_DIR}")
include_directories ("${PROJECT_SOURCE_DIR}/util")

# Inclui todas libraries nas pastas do projeto
file(GLOB SOURCES
	"util/*.cpp"
	"*.cpp")

# Cria executavel do programa
add_executable(start ${SOURCES})

# Linka as aplicações com as libs
target_link_libraries(start ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${PCL_LIBRARIES} util)
