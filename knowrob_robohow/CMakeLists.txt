##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(knowrob_robohow)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED knowrob_mongo)

catkin_package(
  DEPENDS knowrob_mongo
  DEPENDS knowrob_meshes
)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
