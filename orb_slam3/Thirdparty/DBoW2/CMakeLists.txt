cmake_minimum_required(VERSION 2.8)
project(DBoW2)

include_directories(${OpenCV_INCLUDE_DIRS})
add_library(DBoW2 SHARED

  # DBoW2 source files.
  "DBoW2/BowVector.cpp"
  "DBoW2/FORB.cpp"
  "DBoW2/FeatureVector.cpp"
  "DBoW2/ScoringObject.cpp"

  # DUtils source files.
  "DUtils/Random.cpp"
  "DUtils/Timestamp.cpp"
)
target_link_libraries(DBoW2 ${OpenCV_LIBS})