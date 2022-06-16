find_path(include_hpipm-cpp
         NAMES hpipm-cpp.hpp
         PATHS "/usr/local/include/hpipm-cpp"
         )

find_path(include_blasfeo
         NAMES blasfeo.h
         PATHS "/usr/local/include/hpipm-cpp/blasfeo"
         )

find_path(include_hpipm
         NAMES hpipm_tree.h
         PATHS "/usr/local/include/hpipm-cpp/hpipm"
         )


find_library(lib_blasfeo
             NAMES blasfeo
             PATHS "/usr/local/lib/hpipm-cpp"
             )

find_library(lib_hpipm
             NAMES hpipm
             PATHS "/usr/local/lib/hpipm-cpp"
             )

find_library(lib_hpipm-cpp
             NAMES hpipm-cpp
             PATHS "/usr/local/lib"
             )

set(HPIPM_LIBRARIES "${lib_blasfeo};${lib_hpipm};${lib_hpipm-cpp}")
set(HPIPM_INCLUDE_DIRECTORIES "${include_hpipm};${include_hpipm-cpp};${include_blasfeo}")

message(${HPIPM_LIBRARIES})
message(${HPIPM_INCLUDE_DIRECTORIES})
