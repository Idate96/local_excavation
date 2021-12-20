//
// Created by lorenzo on 28.10.21.
//
#include "local_excavation/LocalPlanner.hpp"
#include "excavation_mapping/ExcavationMapping.h"


// create gtest main function
int main(int argc, char **argv) {
  excavation_mapping::ExcavationMapping mapping = excavation_mapping::ExcavationMapping();
  local_excavation::LocalPlanner planning = local_excavation::LocalPlanner(mapping);
}