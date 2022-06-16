FILE(GLOB headers "*.hh" "*.h" "*.hpp")
FILE(GLOB_RECURSE sources *.cc)

add_library (goldfarb SHARED ${headers} ${sources})
